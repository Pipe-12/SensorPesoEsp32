#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLEUtils.h"
#include "BLE2902.h"
#include <HX711.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>
#include "esp_bt.h"
#include "esp_wifi.h"
#include "esp_task_wdt.h"

//BLE server name
#define bleServerName "CamperGas_Sensor"

// Configuración de ahorro de energía
#define LIGHT_SLEEP_TIME_CONNECTED 5 // 5 segundos cuando conectado
#define CPU_FREQ_LOW 80 // MHz para bajo consumo
#define CPU_FREQ_NORMAL 240 // MHz para operación normal

// Pin de datos y de reloj para HX711
byte pinData = 4;
byte pinClk = 2;

HX711 bascula;

// Crear el objeto del sensor ADXL345
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

float factor_calibracion = 25000.0;  //Factor de calibracion
float peso;
float pitch, roll; // Variables para inclinación

// Variables de estado de energía
bool sensorsInitialized = false;
bool tareCompleted = false;

// Timer variables para offline
unsigned long lastOfflineTime = 0;
unsigned long offlineTimerDelay = 900000; // 15 minutos (15 * 60 * 1000 ms)
unsigned long initialOfflineDelay = 900000; // 15 minutos inicial
unsigned long maxOfflineDelay = 86400000; // 24 horas (24 * 60 * 60 * 1000 ms)
int offlineIndex = 0; // Índice circular para reemplazar medidas antiguas

// Estructura para almacenar datos offline
struct MeasurementData {
  float weight;
  unsigned long timestamp; // Timestamp en milisegundos desde boot (millis())
};

// Array para almacenar medidas offline (máximo 100 medidas)
#define MAX_OFFLINE_MEASUREMENTS 100
MeasurementData offlineMeasurements[MAX_OFFLINE_MEASUREMENTS];
int offlineMeasurementCount = 0;

bool deviceConnected = false;

// Declaraciones de funciones
void sendOfflineData();
void resetOfflineSystem();
void powerDownSensors();
void powerUpSensors();
void initSensors();
void initHX711();
void initHX711WithTare();
void initADXL345();
void readInclination();
void storeOfflineMeasurement(float weight, unsigned long timestamp);
void enterLightSleep();

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

// Un único servicio para el sensor de peso con inclinación
#define SENSOR_SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42f59"

// Tres características dentro del mismo servicio
#define WEIGHT_CHARACTERISTIC_UUID "cba1d466-344c-4be3-ab3f-189f80dd7518"
#define OFFLINE_CHARACTERISTIC_UUID "87654321-4321-4321-4321-cba987654321"
#define INCLINATION_CHARACTERISTIC_UUID "fedcba09-8765-4321-fedc-ba0987654321"

// Weight Characteristic (READ-only, sin descriptor)
BLECharacteristic weightCharacteristics("cba1d466-344c-4be3-ab3f-189f80dd7518", BLECharacteristic::PROPERTY_READ);

// Offline Data Characteristic (READ-only, sin descriptor)
BLECharacteristic offlineDataCharacteristics("87654321-4321-4321-4321-cba987654321", BLECharacteristic::PROPERTY_READ);

// Inclination Characteristic (READ-only, sin descriptor)
BLECharacteristic inclinationCharacteristics("fedcba09-8765-4321-fedc-ba0987654321", BLECharacteristic::PROPERTY_READ);

// Callback para lecturas de características
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic* pCharacteristic) {
        String uuid = pCharacteristic->getUUID().toString();
        
        if (uuid == WEIGHT_CHARACTERISTIC_UUID) {
            Serial.println("📖 Dispositivo móvil solicita PESO - tomando medida...");
            
            // Asegurar que los sensores están listos
            if (!sensorsInitialized) {
                powerUpSensors();
            }
            
            // Verificar que HX711 está listo
            if (bascula.is_ready()) {
                float peso = -1 * bascula.get_units(3);
                
                if (!isnan(peso)) {
                    static char weightData[20];
                    sprintf(weightData, "{\"w\":%.1f}", peso);
                    pCharacteristic->setValue(weightData);
                    
                    Serial.print("✅ Peso enviado: ");
                    Serial.print(peso);
                    Serial.println(" kg");
                } else {
                    Serial.println("❌ ERROR: Lectura NaN de HX711");
                    pCharacteristic->setValue("{\"w\":0.0}");
                }
            } else {
                Serial.println("❌ ERROR: HX711 no está listo");
                pCharacteristic->setValue("{\"w\":0.0}");
            }
        }
        else if (uuid == INCLINATION_CHARACTERISTIC_UUID) {
            Serial.println("📖 Dispositivo móvil solicita INCLINACIÓN - tomando medida...");
            
            // Asegurar que los sensores están listos
            if (!sensorsInitialized) {
                powerUpSensors();
            }
            
            // Leer inclinación
            readInclination();
            
            static char inclinationData[30];
            sprintf(inclinationData, "{\"p\":%.1f,\"r\":%.1f}", pitch, roll);
            pCharacteristic->setValue(inclinationData);
            
            Serial.print("✅ Inclinación enviada - Pitch: ");
            Serial.print(pitch);
            Serial.print("° | Roll: ");
            Serial.print(roll);
            Serial.println("°");
        }
        else if (uuid == OFFLINE_CHARACTERISTIC_UUID) {
            Serial.println("📖 Dispositivo móvil solicita DATOS OFFLINE");
            // La característica offline ya maneja su propio contenido
            // No necesita callback especial porque ya está preparada
        }
    }
};

//Setup callbacks onConnect and onDisconnect
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    setCpuFrequencyMhz(CPU_FREQ_NORMAL); // Aumentar frecuencia cuando conectado
    Serial.println("🔗 Cliente conectado - modo activo");
    
    Serial.println("🎉 Proceso de conexión completado");
    Serial.println("📋 MODO PULL ACTIVADO:");
    Serial.println("📋 - El dispositivo móvil debe LEER las características para obtener datos");
    Serial.println("📋 - PESO: cba1d466-344c-4be3-ab3f-189f80dd7518 (toma medida al leer)");
    Serial.println("  - INCLINACIÓN: fedcba09-8765-4321-fedc-ba0987654321 (toma medida al leer)");
    Serial.println("📋 - DATOS OFFLINE: 87654321-4321-4321-4321-cba987654321 (datos históricos)");
    
    // Preparar datos offline si hay datos almacenados
    if (offlineMeasurementCount > 0) {
      Serial.println("📋 Preparando datos offline...");
      sendOfflineData();
      resetOfflineSystem();
    }
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    setCpuFrequencyMhz(CPU_FREQ_LOW); // Reducir frecuencia cuando desconectado
    Serial.println("Cliente desconectado - modo ahorro energía");
    powerDownSensors(); // Apagar sensores no críticos
    
    // Reiniciar advertising después de la desconexión
    delay(500);
    pServer->getAdvertising()->start();
    Serial.println("Advertising reiniciado");
  }
};

// Función para almacenar medida offline con sistema circular
void storeOfflineMeasurement(float weight, unsigned long timestamp) {
  // Mostrar datos antes de almacenar
  Serial.print("📊 Preparando almacenar - Peso: ");
  Serial.print(weight);
  Serial.print(" kg | Timestamp: ");
  Serial.print(timestamp);
  Serial.println(" ms");
  
  if (offlineMeasurementCount < MAX_OFFLINE_MEASUREMENTS) {
    // Memoria no llena, añadir normalmente
    offlineMeasurements[offlineMeasurementCount].weight = weight;
    offlineMeasurements[offlineMeasurementCount].timestamp = timestamp;
    offlineMeasurementCount++;
    Serial.print("✅ Medida offline almacenada: ");
    Serial.print(weight);
    Serial.print(" kg, timestamp: ");
    Serial.print(timestamp);
    Serial.print(" ms. Total almacenadas: ");
    Serial.println(offlineMeasurementCount);
  } else {
    // Memoria llena, reemplazar la más antigua usando índice circular
    Serial.print("⚠️ Memoria llena - Reemplazando en índice ");
    Serial.print(offlineIndex);
    Serial.print(" - Peso: ");
    Serial.print(weight);
    Serial.print(" kg | Timestamp: ");
    Serial.print(timestamp);
    Serial.println(" ms");
    
    offlineMeasurements[offlineIndex].weight = weight;
    offlineMeasurements[offlineIndex].timestamp = timestamp;
    offlineIndex = (offlineIndex + 1) % MAX_OFFLINE_MEASUREMENTS;
    
    Serial.print("✅ Medida antigua reemplazada: ");
    Serial.print(weight);
    Serial.print(" kg, timestamp: ");
    Serial.print(timestamp);
    Serial.print(" ms. Nuevo índice: ");
    Serial.println(offlineIndex);
    
    // Duplicar el tiempo entre medidas (máximo 24 horas)
    if (offlineTimerDelay < maxOfflineDelay) {
      offlineTimerDelay *= 2;
      if (offlineTimerDelay > maxOfflineDelay) {
        offlineTimerDelay = maxOfflineDelay;
      }
      Serial.print("Intervalo aumentado a: ");
      Serial.print(offlineTimerDelay / 1000);
      Serial.print(" segundos (");
      Serial.print(offlineTimerDelay / 60000);
      Serial.println(" minutos)");
    }
  }
}

// Función para reiniciar el sistema offline
void resetOfflineSystem() {
  offlineTimerDelay = initialOfflineDelay;
  offlineIndex = 0;
  Serial.println("Sistema offline reiniciado");
}

// Funciones de gestión de energía simplificadas
void powerDownSensors() {
  Serial.println("Sensores en modo ahorro (ADXL345 permanece activo)");
}

void powerUpSensors() {
  if (!sensorsInitialized) {
    initSensors();
  }
  Serial.println("Sensores activados");
}

void initSensors() {
  Serial.println("=== INICIALIZANDO SENSORES ===");
  
  initHX711WithTare();
  initADXL345();
  
  sensorsInitialized = true;
  Serial.println("=== SENSORES INICIALIZADOS ===");
}

void enterLightSleep() {
  // Evitar light sleep que causa watchdog timer resets - usar delay normal
  if (deviceConnected) {
    delay(100); // Delay corto cuando conectado para responder rápido
  } else {
    // Usar delay normal en lugar de light sleep para evitar watchdog timer reset
    delay(5000); // 5 segundos de delay normal - evita conflictos con BLE
  }
}

// Función para enviar datos offline cuando se conecta BLE
void sendOfflineData() {
  if (offlineMeasurementCount > 0) {
    Serial.print("=== PREPARANDO ");
    Serial.print(offlineMeasurementCount);
    Serial.println(" MEDIDAS OFFLINE VIA CARACTERÍSTICA READ-ONLY (BATCH MODE) ===");
    
    // Verificar que el dispositivo sigue conectado
    if (!deviceConnected) {
      Serial.println("ERROR: Dispositivo desconectado durante preparación offline");
      return;
    }
    
    // Información para el usuario sobre cómo leer los datos
    Serial.println("💡 DATOS OFFLINE LISTOS PARA LECTURA");
    Serial.println("💡 El cliente debe LEER la característica: 87654321-4321-4321-4321-cba987654321");
    Serial.println("💡 Los datos NO se envían automáticamente (característica READ-only)");
    
    // Preparar datos en lotes para lectura posterior
    static char offlineDataString[500];  // Buffer grande para múltiples medidas
    int batchCount = 0;
    int totalBatches = 0; // Contador de lotes preparados
    
    // Construir JSON array con múltiples medidas
    strcpy(offlineDataString, "[");  // Iniciar array JSON
    
    // Timestamp actual para calcular tiempo transcurrido
    unsigned long currentReadTime = millis();
    Serial.print("⏰ Tiempo actual de lectura: ");
    Serial.print(currentReadTime);
    Serial.println(" ms");
    
    for (int i = 0; i < offlineMeasurementCount; i++) {
      // Calcular tiempo transcurrido desde la medida hasta ahora
      unsigned long elapsedTime = currentReadTime - offlineMeasurements[i].timestamp;
      
      Serial.print("📊 Medida ");
      Serial.print(i);
      Serial.print(": Peso=");
      Serial.print(offlineMeasurements[i].weight);
      Serial.print("kg, Almacenada en=");
      Serial.print(offlineMeasurements[i].timestamp);
      Serial.print("ms, Transcurrido=");
      Serial.print(elapsedTime);
      Serial.print("ms (");
      Serial.print(elapsedTime / 1000);
      Serial.println(" segundos)");
      
      char singleMeasurement[50];  // Aumentado a 50 para timestamp completo
      sprintf(singleMeasurement, "{\"w\":%.1f,\"t\":%lu}", 
              offlineMeasurements[i].weight, 
              elapsedTime); // Usar tiempo transcurrido en lugar de timestamp absoluto
      
      // Verificar si cabe en el buffer actual
      if (strlen(offlineDataString) + strlen(singleMeasurement) + 10 < 500) {
        // Añadir coma si no es el primer elemento
        if (i > 0) strcat(offlineDataString, ",");
        strcat(offlineDataString, singleMeasurement);
        batchCount++;
      } else {
        // Buffer lleno, preparar lote actual
        strcat(offlineDataString, "]");  // Cerrar array JSON
        
        totalBatches++;
        Serial.print("  Preparando LOTE ");
        Serial.print(totalBatches);
        Serial.print(" [");
        Serial.print(batchCount);
        Serial.print(" medidas]: ");
        Serial.println(offlineDataString);
        Serial.print("   🎯 UUID OFFLINE: 87654321-4321-4321-4321-cba987654321, Tamaño: ");
        Serial.print(strlen(offlineDataString));
        Serial.println(" bytes");
        
        // Verificar conexión
        if (!deviceConnected) {
          Serial.println("❌ Conexión perdida durante preparación");
          break;
        }
        
        // Solo establecer valor para lectura posterior (NO notify)
        offlineDataCharacteristics.setValue(offlineDataString);
        Serial.println("   ✅ Lote preparado para lectura (característica actualizada)");
        
        // Delay entre preparaciones para estabilidad
        delay(500);
        
        // Reiniciar buffer para siguiente lote
        strcpy(offlineDataString, "[");
        strcat(offlineDataString, singleMeasurement);
        batchCount = 1;
      }
    }
    
    // Enviar último lote si queda algo
    if (batchCount > 0) {
      strcat(offlineDataString, "]");  // Cerrar array JSON
      
      totalBatches++;
      Serial.print("  Preparando lote FINAL ");
      Serial.print(totalBatches);
      Serial.print(" [");
      Serial.print(batchCount);
      Serial.print(" medidas]: ");
      Serial.println(offlineDataString);
      Serial.print("   🎯 UUID OFFLINE: 87654321-4321-4321-4321-cba987654321, Tamaño: ");
      Serial.print(strlen(offlineDataString));
      Serial.println(" bytes");
      
      if (deviceConnected) {
        offlineDataCharacteristics.setValue(offlineDataString);
        Serial.println("   ✅ Lote final preparado para lectura (característica actualizada)");
      }
    }
    
    // Vaciar memoria después de preparar datos
    offlineMeasurementCount = 0;
    Serial.println("=== DATOS OFFLINE PREPARADOS Y MEMORIA VACIADA ===");
    Serial.print("🎯 RESUMEN FINAL: ");
    Serial.print(totalBatches);
    Serial.println(" lotes preparados total");
    Serial.println("💡 El cliente debe LEER la característica para obtener los datos");
    Serial.println("💡 FORMATO JSON: {\"w\":peso_kg,\"t\":milisegundos_transcurridos_desde_medida}");
  } else {
    Serial.println("⚠️ No hay datos offline para preparar");
  }
}

void initHX711(){
  Serial.println("Iniciando la Bascula...");
  
  // Siempre inicializar la báscula
  bascula.begin(pinData, pinClk);
  
  // Esperar a que el HX711 esté listo
  Serial.print("Esperando HX711");
  int attempts = 0;
  while (!bascula.is_ready() && attempts < 50) {
    delay(100);
    Serial.print(".");
    attempts++;
  }
  Serial.println("");
  
  if (!bascula.is_ready()) {
    Serial.println("ERROR: HX711 no responde después de 5 segundos");
    return;
  }
  
  Serial.println("HX711 listo, configurando...");
  
  // Configurar la escala
  bascula.set_scale(factor_calibracion);
  
  Serial.println("HX711 inicializado correctamente");
}

// Función específica para inicialización completa con tara (solo arranque inicial)
void initHX711WithTare(){
  Serial.println("Iniciando la Bascula con tara...");
  
  initHX711();
  
  if (!bascula.is_ready()) {
    Serial.println("ERROR: No se puede hacer tara, HX711 no está listo");
    return;
  }
  
  if (!tareCompleted) {
    Serial.println("Haciendo tara inicial...");
    bascula.tare(10);
    tareCompleted = true;
    
    long zero_factor = bascula.read_average(5);
    Serial.print("Zero factor: ");
    Serial.println(zero_factor);
    
    float test_reading = bascula.get_units(3);
    Serial.print("Lectura de prueba: ");
    Serial.print(test_reading);
    Serial.println(" kg");
    
    Serial.println("HX711 inicializado completamente con tara inicial");
  } else {
    Serial.println("Tara ya completada anteriormente - saltando tara");
    Serial.println("HX711 inicializado sin tara");
  }
}

void initADXL345(){
  Serial.println("Iniciando el ADXL345...");

  if (!accel.begin()) {
    Serial.println("No se pudo encontrar el ADXL345");
    return;
  }

  Serial.println("ADXL345 conectado correctamente");
}

void readInclination() {
  sensors_event_t event;
  accel.getEvent(&event);

  // Obtener los valores de aceleración en los tres ejes
  float x = event.acceleration.x;
  float y = event.acceleration.y;
  float z = event.acceleration.z;

  // Calcular el pitch y roll
  pitch = atan2(y, sqrt(x * x + z * z)) * 180.0 / PI;
  roll = atan2(-x, sqrt(y * y + z * z)) * 180.0 / PI;

  // Debug: Mostrar valores
  Serial.print("Valores crudos ADXL345 - X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.println(z);

  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print("° | Roll: ");
  Serial.print(roll);
  Serial.println("°");
}

void setup() {
  // Deshabilitar watchdog timer para evitar resets
  esp_task_wdt_delete(NULL);
  
  // Configurar CPU a baja frecuencia inicialmente
  setCpuFrequencyMhz(CPU_FREQ_LOW);
  
  // Start serial communication 
  Serial.begin(115200);
  delay(500); // Reducir delay inicial
  
  // Inicialización completa
  Serial.println("Inicialización completa...");
  
  // Deshabilitar WiFi para ahorrar energía
  esp_wifi_stop();
  
  // Init Sensors
  initSensors();

  // Create the BLE Device
  BLEDevice::init(bleServerName);
  
  // Configurar MTU máximo para transmisiones más grandes
  BLEDevice::setMTU(512); // MTU más grande para mayor capacidad de datos

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Services (un solo servicio con 3 características)
  BLEService *sensorService = pServer->createService(SENSOR_SERVICE_UUID);

  // Create BLE Characteristics (todas READ-only)
  // Weight (READ-only, sin descriptor)
  sensorService->addCharacteristic(&weightCharacteristics);
  weightCharacteristics.setCallbacks(new MyCharacteristicCallbacks());
  
  // Offline Data (READ-only, sin descriptor)
  sensorService->addCharacteristic(&offlineDataCharacteristics);
  // No agregar descriptor ni callback para característica offline
  
  // Inclination Data (READ-only, sin descriptor)
  sensorService->addCharacteristic(&inclinationCharacteristics);
  inclinationCharacteristics.setCallbacks(new MyCharacteristicCallbacks());
  
  // Start the service
  sensorService->start();
  
  Serial.println("Servicio BLE iniciado correctamente");

  // Start advertising con configuración optimizada para conexión estable
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SENSOR_SERVICE_UUID);
  
  // Configurar advertising para conexión más estable
  pAdvertising->setMinInterval(320);  // 200ms - más rápido para descubrimiento
  pAdvertising->setMaxInterval(640);  // 400ms - más rápido para descubrimiento
  
  // Habilitar scan response para mejor compatibilidad
  pAdvertising->setScanResponse(true);
  
  pServer->getAdvertising()->start();
  Serial.println("Sistema BLE activo - esperando conexión...");
  Serial.println("📋 INFORMACIÓN IMPORTANTE:");
  Serial.println("📋 - MODO PULL ACTIVADO: El dispositivo móvil debe LEER las características");
  Serial.println("📋 - PESO: cba1d466-344c-4be3-ab3f-189f80dd7518 (toma medida al leer)");
  Serial.println("📋 - INCLINACIÓN: fedcba09-8765-4321-fedc-ba0987654321 (toma medida al leer)");
  Serial.println("📋 - DATOS OFFLINE: 87654321-4321-4321-4321-cba987654321 (datos históricos)");
  
  // Apagar sensores no críticos inicialmente
  powerDownSensors();
}

void loop() {
  unsigned long currentTime = millis();
  
  if (!deviceConnected) {
    // Modo offline - tomar medidas periódicamente para almacenar datos históricos
    if ((currentTime - lastOfflineTime) > offlineTimerDelay) {
      Serial.println("Tomando medida offline (sin conexión BLE)...");
      
      // Asegurar que los sensores están listos
      if (!sensorsInitialized) {
        powerUpSensors();
      }
      
      // Verificar que HX711 está listo
      if (bascula.is_ready()) {
        float offlineWeight = -1 * bascula.get_units(3);
        unsigned long timestamp = currentTime; // Usar millis() directamente
        
        Serial.print("🔍 Medida offline obtenida - Peso: ");
        Serial.print(offlineWeight);
        Serial.print(" kg | Timestamp generado: ");
        Serial.print(timestamp);
        Serial.print(" ms (");
        Serial.print(currentTime);
        Serial.println(" ms)");
        
        if (!isnan(offlineWeight)) {
          storeOfflineMeasurement(offlineWeight, timestamp);
          Serial.print("📦 Proceso completado - Peso almacenado: ");
          Serial.print(offlineWeight);
          Serial.println(" kg");
        } else {
          Serial.println("❌ ERROR: Lectura NaN en modo offline - no se almacena");
        }
      } else {
        Serial.println("WARNING: HX711 no está listo en modo offline");
      }
      
      lastOfflineTime = currentTime;
    }
    
    // Usar enterLightSleep para ahorrar energía manteniendo BLE disponible
    enterLightSleep();
    return;
  }
  
  // Dispositivo conectado - modo PULL activo
  if (!sensorsInitialized) {
    powerUpSensors();
  }
  
  // En modo PULL, no hay medidas automáticas
  // Las medidas se toman solo cuando el dispositivo móvil lee las características
  // Esto se maneja en los callbacks de MyCharacteristicCallbacks
  
  // Usar enterLightSleep entre verificaciones para ahorrar energía
  enterLightSleep();
}
