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
bool hx711Initialized = false;
bool bleActive = false;

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 5000; // 5 segundos para datos en tiempo real
unsigned long lastOfflineTime = 0;
unsigned long offlineTimerDelay = 900000; // 15 minutos (15 * 60 * 1000 ms)
unsigned long initialOfflineDelay = 900000; // 15 minutos inicial
unsigned long maxOfflineDelay = 86400000; // 24 horas (24 * 60 * 60 * 1000 ms)
int offlineIndex = 0; // Índice circular para reemplazar medidas antiguas

// Estructura para almacenar datos offline
struct MeasurementData {
  float weight;
  unsigned long timestamp; // Timestamp en segundos (no milisegundos)
};

// Array para almacenar medidas offline (máximo 100 medidas)
#define MAX_OFFLINE_MEASUREMENTS 100
MeasurementData offlineMeasurements[MAX_OFFLINE_MEASUREMENTS];
int offlineMeasurementCount = 0;

bool deviceConnected = false;
bool offlineDataSent = false; // Controlar si ya se enviaron los datos offline

// Declaraciones de funciones
void enableAllNotifications();
void sendOfflineData();
void resetOfflineSystem();
void powerDownSensors();
void powerUpSensors();
void initSensors();
void initHX711();
void initHX711WithTare(); // Nueva función para arranque inicial
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

// Weight Characteristic and Descriptor (includes timestamp)
BLECharacteristic weightCharacteristics("cba1d466-344c-4be3-ab3f-189f80dd7518", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor weightDescriptor(BLEUUID((uint16_t)0x2902)); // Client Characteristic Configuration

// Offline Data Characteristic (READ-only, sin descriptor)
BLECharacteristic offlineDataCharacteristics("87654321-4321-4321-4321-cba987654321", BLECharacteristic::PROPERTY_READ);

// Inclination Characteristic and Descriptor
BLECharacteristic inclinationCharacteristics("fedcba09-8765-4321-fedc-ba0987654321", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor inclinationDescriptor(BLEUUID((uint16_t)0x2902)); // Client Characteristic Configuration

//Setup callbacks onConnect and onDisconnect
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    bleActive = true;
    setCpuFrequencyMhz(CPU_FREQ_NORMAL); // Aumentar frecuencia cuando conectado
    Serial.println("🔗 Cliente conectado - modo activo");
    
    // Delay LARGO para que nRF Connect habilite completamente las notificaciones
    Serial.println("⏳ Esperando que nRF Connect termine de conectarse...");
    delay(3000); // 3 segundos para estabilizar
    
    // Habilitar automáticamente las notificaciones para todos los servicios
    enableAllNotifications();
    
    Serial.println("🎉 Proceso de conexión completado");
    Serial.println("🔄 Los datos en tiempo real comenzarán en 5 segundos...");
    Serial.println("📋 Los datos offline se prepararán después de la primera medida en tiempo real (si hay datos almacenados)");
    Serial.println("📋 Para leer datos offline: LEER característica 87654321-4321-4321-4321-cba987654321");
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    offlineDataSent = false; // Reset para próxima conexión
    bleActive = false;
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
  if (offlineMeasurementCount < MAX_OFFLINE_MEASUREMENTS) {
    // Memoria no llena, añadir normalmente
    offlineMeasurements[offlineMeasurementCount].weight = weight;
    offlineMeasurements[offlineMeasurementCount].timestamp = timestamp;
    offlineMeasurementCount++;
    Serial.print("Medida offline almacenada: ");
    Serial.print(weight);
    Serial.print(" kg, timestamp: ");
    Serial.print(timestamp);
    Serial.print(" seg. Total almacenadas: ");
    Serial.println(offlineMeasurementCount);
  } else {
    // Memoria llena, reemplazar la más antigua usando índice circular
    offlineMeasurements[offlineIndex].weight = weight;
    offlineMeasurements[offlineIndex].timestamp = timestamp;
    offlineIndex = (offlineIndex + 1) % MAX_OFFLINE_MEASUREMENTS;
    
    Serial.print("Memoria llena - Reemplazando medida antigua: ");
    Serial.print(weight);
    Serial.print(" kg, timestamp: ");
    Serial.print(timestamp);
    Serial.print(" seg. Índice: ");
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
  Serial.print("Sistema offline reiniciado - Intervalo: ");
  Serial.print(offlineTimerDelay / 60000);
  Serial.println(" minutos");
}

// Función para habilitar automáticamente las notificaciones (excluye offline)
void enableAllNotifications() {
  // Habilitar notificaciones automáticamente
  uint8_t notificationOn[] = {0x01, 0x00};
  
  Serial.println("🔧 Habilitando notificaciones automáticamente...");
  
  // OFFLINE - Ahora es READ-only, no necesita notificaciones
  Serial.println("✓ Característica OFFLINE configurada como READ-only (sin notificaciones)");
  
  // PESO - para datos en tiempo real
  weightCharacteristics.getDescriptorByUUID(BLEUUID((uint16_t)0x2902))->setValue(notificationOn, 2);
  Serial.println("✓ Notificaciones PESO habilitadas (datos en tiempo real)");
  
  // Inclinación - para datos en tiempo real
  inclinationCharacteristics.getDescriptorByUUID(BLEUUID((uint16_t)0x2902))->setValue(notificationOn, 2);
  Serial.println("✓ Notificaciones INCLINACIÓN habilitadas");
  
  Serial.println("=== CONFIGURACIÓN AUTOMÁTICA COMPLETADA ===");
}

// Funciones de gestión de energía - SIMPLIFICADAS
void powerDownSensors() {
  // NO apagar ADXL345 - mantenerlo siempre activo como en el código que funciona
  Serial.println("Sensores en modo ahorro (ADXL345 permanece activo)");
}

void powerUpSensors() {
  if (!sensorsInitialized) {
    initSensors();
  }
  // NO reinicializar ADXL345 - mantenerlo estable
  Serial.println("Sensores activados");
}

void initSensors() {
  Serial.println("=== INICIALIZANDO SENSORES ===");
  
  initHX711WithTare(); // Usar versión con tara para arranque inicial
  hx711Initialized = true;
  
  initADXL345();
  
  sensorsInitialized = true;
  Serial.println("=== SENSORES INICIALIZADOS ===");
}



void enterLightSleep() {
  // Solo hacer light sleep muy corto cuando está conectado para no interferir con BLE
  if (deviceConnected) {
    delay(100); // Solo delay corto cuando conectado
  } else {
    esp_sleep_enable_timer_wakeup(LIGHT_SLEEP_TIME_CONNECTED * 1000000ULL);
    esp_light_sleep_start();
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
    
    for (int i = 0; i < offlineMeasurementCount; i++) {
      char singleMeasurement[50];  // Aumentado a 50 para timestamp completo
      sprintf(singleMeasurement, "{\"w\":%.1f,\"t\":%lu}", 
              offlineMeasurements[i].weight, 
              offlineMeasurements[i].timestamp);
      
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
        Serial.print("� Preparando LOTE ");
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
      Serial.print("� Preparando lote FINAL ");
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
  
  // Inicializar básicamente
  initHX711();
  
  if (!bascula.is_ready()) {
    Serial.println("ERROR: No se puede hacer tara, HX711 no está listo");
    return;
  }
  
  // Solo hacer tara en arranque inicial
  Serial.println("Haciendo tara...");
  bascula.tare(10); // Promedio de 10 lecturas
  
  // Verificar funcionamiento
  long zero_factor = bascula.read_average(5);
  Serial.print("Zero factor: ");
  Serial.println(zero_factor);
  
  // Lectura de prueba
  float test_reading = bascula.get_units(3);
  Serial.print("Lectura de prueba: ");
  Serial.print(test_reading);
  Serial.println(" kg");
  
  Serial.println("HX711 inicializado completamente con tara");
}

void initADXL345(){
  Serial.println("Iniciando el ADXL345...");

  // Inicializar el ADXL345 - IGUAL que el código que funciona
  if (!accel.begin()) {
    Serial.println("No se pudo encontrar el ADXL345");
    return; // No bloquear con while(1) para no parar BLE
  }

  Serial.println("ADXL345 conectado correctamente");
}

// Función para leer inclinación del ADXL345 - EXACTAMENTE IGUAL que el código que funciona
void readInclination() {
  sensors_event_t event;
  accel.getEvent(&event);

  // Obtener los valores de aceleración en los tres ejes - IGUAL que el código que funciona
  float x = event.acceleration.x;
  float y = event.acceleration.y;
  float z = event.acceleration.z;

  // Calcular el pitch y roll - EXACTAMENTE IGUAL que el código que funciona
  pitch = atan2(y, sqrt(x * x + z * z)) * 180.0 / PI;
  roll = atan2(-x, sqrt(y * y + z * z)) * 180.0 / PI;

  // Debug: Mostrar valores como en el código que funciona
  Serial.print("Valores crudos ADXL345 - X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.println(z);

  Serial.print("Pitch calculado: ");
  Serial.print(pitch);
  Serial.print("° | Roll calculado: ");
  Serial.print(roll);
  Serial.println("°");
}

void setup() {
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

  // Create BLE Characteristics and Create a BLE Descriptor
  // Weight (includes timestamp)
  sensorService->addCharacteristic(&weightCharacteristics);
  weightDescriptor.setValue("Weight");
  weightCharacteristics.addDescriptor(&weightDescriptor);
  
  // Offline Data (READ-only, sin descriptor)
  sensorService->addCharacteristic(&offlineDataCharacteristics);
  // No agregar descriptor para característica read-only
  
  // Inclination Data
  sensorService->addCharacteristic(&inclinationCharacteristics);
  inclinationDescriptor.setValue("Inclination");
  inclinationCharacteristics.addDescriptor(&inclinationDescriptor);
  
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
  Serial.println("📋 - Datos en tiempo real: PESO y INCLINACIÓN se envían por notificación");
  Serial.println("📋 - Datos offline/históricos: El cliente debe LEER la característica 87654321-4321-4321-4321-cba987654321");
  Serial.println("📋 - Los datos offline NO se envían automáticamente");
  
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
        unsigned long timestamp = currentTime / 1000; // Convertir a segundos
        
        if (!isnan(offlineWeight)) {
          storeOfflineMeasurement(offlineWeight, timestamp);
          Serial.print("Medida offline almacenada: ");
          Serial.print(offlineWeight);
          Serial.println(" kg");
        } else {
          Serial.println("ERROR: Lectura NaN en modo offline");
        }
      } else {
        Serial.println("WARNING: HX711 no está listo en modo offline");
      }
      
      lastOfflineTime = currentTime;
    }
    
    // Light sleep para ahorrar energía manteniendo BLE disponible
    delay(5000); // Esperar 5 segundos antes de verificar conexión nuevamente
    return;
  }
  
  // Dispositivo conectado - modo activo
  if (!sensorsInitialized) {
    powerUpSensors();
  }
  
  // Medidas en tiempo real cada 5 segundos (peso + inclinación simultáneamente)
  if ((currentTime - lastTime) > timerDelay) {
    // **ENVIAR DATOS OFFLINE DESPUÉS DE LA PRIMERA MEDIDA EN TIEMPO REAL**
    if (!offlineDataSent && offlineMeasurementCount > 0) {
      Serial.println("📋 ¡ENVIANDO DATOS OFFLINE DESPUÉS DE ACTIVAR TIEMPO REAL!");
      sendOfflineData();
      resetOfflineSystem();
      offlineDataSent = true;
      Serial.println("✅ Datos offline enviados, continuando con tiempo real...");
    }
    
    // Verificar que HX711 está listo antes de leer
    if (!bascula.is_ready()) {
      Serial.println("WARNING: HX711 no está listo, reinicializando...");
      initHX711();
      return; // Saltar esta iteración
    }
    
    // Read weight from HX711 sensor
    peso = -1 * bascula.get_units(3); // Promedio de 3 lecturas
    
    // Verificar lectura válida
    if (isnan(peso)) {
      Serial.println("ERROR: Lectura NaN de HX711");
      return;
    }
    
    Serial.print("DEBUG - Peso leído: ");
    Serial.println(peso);
    
    // Read inclination from ADXL345 sensor (en tiempo real junto con peso)
    readInclination();

    // Notify weight reading - Sin timestamp para máxima compactación
    static char weightData[20];
    sprintf(weightData, "{\"w\":%.1f}", peso);
    weightCharacteristics.setValue(weightData);
    weightCharacteristics.notify();
    
    Serial.print("Enviando peso JSON: ");
    Serial.println(weightData);
    
    // Notify inclination reading - Sin timestamp para máxima compactación
    static char inclinationData[30];
    sprintf(inclinationData, "{\"p\":%.1f,\"r\":%.1f}", 
            pitch, roll);
    inclinationCharacteristics.setValue(inclinationData);
    inclinationCharacteristics.notify();
    
    Serial.print("Enviando inclinación JSON: ");
    Serial.println(inclinationData);
    
    Serial.print("Datos en tiempo real - Peso: ");
    Serial.print(peso, 1);
    Serial.print(" kg | Pitch: ");
    Serial.print(pitch, 2);
    Serial.print("° | Roll: ");
    Serial.print(roll, 2);
    Serial.print("° | Timestamp: ");
    Serial.print(currentTime);
    Serial.print(" ms | CPU: ");
    Serial.print(getCpuFrequencyMhz());
    Serial.println(" MHz");
    
    lastTime = currentTime;
  }
  
  // Light sleep entre lecturas para ahorrar energía
  enterLightSleep();
}
