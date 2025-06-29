#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLEUtils.h"
#include "BLE2902.h"
#include <HX711.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "esp_sleep.h"
#include "esp_bt.h"
#include "esp_wifi.h"

//BLE server name
#define bleServerName "CamperGas_Sensor_TEST"

// Configuración de ahorro de energía
#define DEEP_SLEEP_TIME_OFFLINE 900 // 15 minutos en segundos
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
void initADXL345();
void readInclination();
void storeOfflineMeasurement(float weight, unsigned long timestamp);
void enterDeepSleep();
void enterLightSleep();
void generateTestOfflineData(); // Nueva función para test

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

// Offline Data Characteristic and Descriptor
BLECharacteristic offlineDataCharacteristics("87654321-4321-4321-4321-cba987654321", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor offlineDataDescriptor(BLEUUID((uint16_t)0x2902)); // Client Characteristic Configuration

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
    Serial.println("⏳ DURANTE ESTE TIEMPO: Habilita notificaciones manualmente si es necesario");
    delay(6000); // 6 segundos - tiempo para que el usuario actúe
    
    // Habilitar automáticamente las notificaciones (respaldo)
    enableAllNotifications();
    
    // **CRÍTICO**: Activar las características enviando datos de prueba ANTES del envío masivo
    Serial.println("🔧 ACTIVANDO características con datos de prueba...");
    
    // Activar característica de PESO
    weightCharacteristics.setValue("{\"w\":0.0}");  // Dato de prueba
    weightCharacteristics.notify();
    Serial.println("✅ Característica PESO activada con dato de prueba");
    delay(500);
    
    // Activar característica de INCLINACIÓN  
    inclinationCharacteristics.setValue("{\"p\":0.0,\"r\":0.0}");  // Dato de prueba
    inclinationCharacteristics.notify();
    Serial.println("✅ Característica INCLINACIÓN activada con dato de prueba");
    delay(500);
    
    // **MUY IMPORTANTE**: Activar característica OFFLINE con dato de prueba
    offlineDataCharacteristics.setValue("[{\"test\":\"inicio\"}]");  // Dato de activación
    offlineDataCharacteristics.notify();
    Serial.println("✅ Característica OFFLINE activada con dato de prueba");
    Serial.println("📱 ¡Ahora deberías ver notificaciones en nRF Connect!");
    delay(2000);
    
    // **TEST**: Generar datos offline de prueba PERO NO ENVIARLOS AÚN
    generateTestOfflineData();
    
    Serial.println("🎉 Proceso de conexión completado");
    Serial.println("🔄 Los datos en tiempo real comenzarán en 5 segundos...");
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

// **NUEVA FUNCIÓN TEST**: Generar datos offline de prueba
void generateTestOfflineData() {
  Serial.println("=== GENERANDO DATOS OFFLINE DE PRUEBA ===");
  
  // Fecha base: 29 de junio de 2025, 12:00:00 (en timestamp Unix)
  // 29 jun 2025 = 1751011200 segundos desde epoch (aproximadamente)
  unsigned long baseTime = 1751011200; // 29 de junio de 2025, 12:00:00 UTC
  
  Serial.print("Timestamp base (29 jun 2025, 12:00): ");
  Serial.println(baseTime);
  
  // Generar 50 medidas offline históricas (cada 30 minutos hacia atrás)
  for (int i = 0; i < 50; i++) {
    // Timestamp hacia atrás: cada 30 minutos (1800 segundos)
    unsigned long testTimestamp = baseTime - (i * 1800); // 30 minutos hacia atrás
    
    // Generar peso variado pero realista (entre 2.0 y 4.0 kg)
    float testWeight = 2.0 + (i % 20) * 0.1 + (sin(i * 0.3) * 0.5); // Variación realista
    
    storeOfflineMeasurement(testWeight, testTimestamp);
  }
  
  Serial.print("Generadas ");
  Serial.print(offlineMeasurementCount);
  Serial.println(" medidas offline de prueba (50 elementos con fecha 29/jun/2025)");
}

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

// Función para habilitar automáticamente todas las notificaciones
void enableAllNotifications() {
  // Habilitar notificaciones automáticamente
  uint8_t notificationOn[] = {0x01, 0x00};
  
  Serial.println("🔧 Habilitando notificaciones automáticamente...");
  
  // OFFLINE - La más importante para los datos históricos
  offlineDataCharacteristics.getDescriptorByUUID(BLEUUID((uint16_t)0x2902))->setValue(notificationOn, 2);
  Serial.println("✅ Notificaciones OFFLINE habilitadas (CRÍTICO para datos históricos)");
  
  // PESO - para datos en tiempo real
  weightCharacteristics.getDescriptorByUUID(BLEUUID((uint16_t)0x2902))->setValue(notificationOn, 2);
  Serial.println("✓ Notificaciones PESO habilitadas (datos en tiempo real)");
  
  // Inclinación - para datos en tiempo real
  inclinationCharacteristics.getDescriptorByUUID(BLEUUID((uint16_t)0x2902))->setValue(notificationOn, 2);
  Serial.println("✓ Notificaciones INCLINACIÓN habilitadas");
  
  Serial.println("=== CONFIGURACIÓN AUTOMÁTICA COMPLETADA ===");
  Serial.println("⚠️ IMPORTANTE: Si no recibes datos offline, habilita manualmente las notificaciones");
  Serial.println("⚠️ en la característica: 87654321-4321-4321-4321-cba987654321");
}

// Funciones de gestión de energía
void powerDownSensors() {
  // Apagar ADXL345 en modo sleep
  accel.writeRegister(ADXL345_REG_POWER_CTL, 0x00); // Standby mode
  Serial.println("ADXL345 en modo sleep");
}

void powerUpSensors() {
  if (!sensorsInitialized) {
    initSensors();
  }
  // Despertar ADXL345
  accel.writeRegister(ADXL345_REG_POWER_CTL, 0x08); // Measurement mode
  Serial.println("Sensores activados");
}

void initSensors() {
  Serial.println("=== INICIALIZANDO SENSORES ===");
  
  initHX711();
  hx711Initialized = true;
  
  initADXL345();
  
  sensorsInitialized = true;
  Serial.println("=== SENSORES INICIALIZADOS ===");
}

void enterDeepSleep() {
  Serial.println("Entrando en deep sleep...");
  Serial.flush();
  
  // Desactivar WiFi y BT para ahorrar energía
  esp_wifi_stop();
  esp_bt_controller_disable();
  
  // Configurar timer para despertar
  esp_sleep_enable_timer_wakeup(DEEP_SLEEP_TIME_OFFLINE * 1000000ULL); // microsegundos
  
  // Entrar en deep sleep
  esp_deep_sleep_start();
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
    Serial.print("=== ENVIANDO ");
    Serial.print(offlineMeasurementCount);
    Serial.println(" MEDIDAS OFFLINE VIA CARACTERÍSTICA OFFLINE (BATCH MODE) ===");
    
    // Verificar que el dispositivo sigue conectado
    if (!deviceConnected) {
      Serial.println("ERROR: Dispositivo desconectado durante envío offline");
      return;
    }
    
    // Verificar estado de notificaciones de la característica OFFLINE
    BLEDescriptor* offlineDescriptor = offlineDataCharacteristics.getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
    if (offlineDescriptor) {
      Serial.println("🔍 Verificando estado de notificaciones OFFLINE...");
      uint8_t* value = offlineDescriptor->getValue();
      if (value && (value[0] & 0x01)) {
        Serial.println("✅ Notificaciones OFFLINE CONFIRMADAS como habilitadas");
      } else {
        Serial.println("⚠️ Notificaciones OFFLINE NO detectadas - PROBLEMA CRÍTICO");
        Serial.println("⚠️ El usuario debe habilitar notificaciones en 87654321-4321-4321-4321-cba987654321");
      }
    }
    
    // Enviar datos en lotes para aprovechar el MTU grande
    static char offlineDataString[500];  // Buffer grande para múltiples medidas
    int batchCount = 0;
    int totalBatchesSent = 0; // Contador de lotes enviados
    
    // Construir JSON array con múltiples medidas
    strcpy(offlineDataString, "[");  // Iniciar array JSON
    
    for (int i = 0; i < offlineMeasurementCount; i++) {
      char singleMeasurement[30];
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
        // Buffer lleno, enviar lote actual
        strcat(offlineDataString, "]");  // Cerrar array JSON
        
        totalBatchesSent++;
        Serial.print("📤 Enviando LOTE ");
        Serial.print(totalBatchesSent);
        Serial.print(" [");
        Serial.print(batchCount);
        Serial.print(" medidas]: ");
        Serial.println(offlineDataString);
        Serial.print("   🎯 UUID OFFLINE: 87654321-4321-4321-4321-cba987654321, Tamaño: ");
        Serial.print(strlen(offlineDataString));
        Serial.println(" bytes");
        
        // Verificar conexión antes de envío
        if (!deviceConnected) {
          Serial.println("❌ Conexión perdida durante envío");
          break;
        }
        
        // Enviar lote por característica OFFLINE
        offlineDataCharacteristics.setValue(offlineDataString);
        offlineDataCharacteristics.notify();
        Serial.println("   ✅ Lote enviado por característica OFFLINE");
        
        // DELAY LARGO entre lotes para poder ver cada uno por separado en nRF Connect
        Serial.println("   ⏳ Esperando 10 segundos antes del siguiente lote...");
        delay(10000); // 3 segundos entre lotes para poder observar cada uno
        
        // Reiniciar buffer para siguiente lote
        strcpy(offlineDataString, "[");
        strcat(offlineDataString, singleMeasurement);
        batchCount = 1;
      }
    }
    
    // Enviar último lote si queda algo
    if (batchCount > 0) {
      strcat(offlineDataString, "]");  // Cerrar array JSON
      
      totalBatchesSent++;
      Serial.print("📤 Enviando lote FINAL ");
      Serial.print(totalBatchesSent);
      Serial.print(" [");
      Serial.print(batchCount);
      Serial.print(" medidas]: ");
      Serial.println(offlineDataString);
      Serial.print("   🎯 UUID OFFLINE: 87654321-4321-4321-4321-cba987654321, Tamaño: ");
      Serial.print(strlen(offlineDataString));
      Serial.println(" bytes");
      
      if (deviceConnected) {
        offlineDataCharacteristics.setValue(offlineDataString);
        offlineDataCharacteristics.notify();
        Serial.println("   ✅ Lote final enviado por característica OFFLINE");
      }
    }
    
    // Vaciar memoria después de enviar
    offlineMeasurementCount = 0;
    Serial.println("=== DATOS OFFLINE ENVIADOS EN LOTES Y MEMORIA VACIADA ===");
    Serial.print("🎯 RESUMEN FINAL: ");
    Serial.print(totalBatchesSent);
    Serial.println(" lotes enviados total");
    Serial.println("📱 En nRF Connect deberías ver cada lote por separado con 10 segundos de diferencia");
  } else {
    Serial.println("⚠️ No hay datos offline para enviar");
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
  
  // Hacer tara
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
  
  Serial.println("HX711 inicializado correctamente");
}

void initADXL345(){
  // Inicializar I2C con frecuencia reducida para ahorrar energía
  Wire.begin(21, 22);
  Wire.setClock(100000); // 100kHz en lugar de 400kHz por defecto

  Serial.println("Iniciando el ADXL345...");

  if (!accel.begin()) {
    Serial.println("No se pudo encontrar el ADXL345");
    while (1);
  }

  // Configurar ADXL345 para bajo consumo
  accel.setRange(ADXL345_RANGE_2_G); // Rango mínimo para menor consumo
  accel.setDataRate(ADXL345_DATARATE_12_5_HZ); // Frecuencia baja
  
  Serial.println("ADXL345 conectado en modo bajo consumo");
}

// Función para leer inclinación del ADXL345
void readInclination() {
  sensors_event_t event;
  accel.getEvent(&event);

  // Obtener valores
  float x = event.acceleration.x;
  float y = event.acceleration.y;
  float z = event.acceleration.z;

  // Calcular pitch y roll
  pitch = atan2(y, sqrt(x * x + z * z)) * 180.0 / PI;
  roll = atan2(-x, sqrt(y * y + z * z)) * 180.0 / PI;
}

void setup() {
  // Configurar CPU a baja frecuencia inicialmente
  setCpuFrequencyMhz(CPU_FREQ_LOW);
  
  // Start serial communication 
  Serial.begin(115200);
  delay(500); // Reducir delay inicial
  
  Serial.println("=== MODO TEST OFFLINE ===");

  // **SIMPLIFICADO PARA TEST**: Saltar deep sleep y ir directo a inicialización
  
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
  // Weight (real-time data)
  sensorService->addCharacteristic(&weightCharacteristics);
  weightDescriptor.setValue("Weight");
  weightCharacteristics.addDescriptor(&weightDescriptor);
  Serial.println("✓ Característica PESO creada: cba1d466-344c-4be3-ab3f-189f80dd7518");
  
  // Offline Data (historical data)
  sensorService->addCharacteristic(&offlineDataCharacteristics);
  offlineDataDescriptor.setValue("Offline");
  offlineDataCharacteristics.addDescriptor(&offlineDataDescriptor);
  Serial.println("✓ Característica OFFLINE creada: 87654321-4321-4321-4321-cba987654321");
  
  // Inclination Data (real-time data)
  sensorService->addCharacteristic(&inclinationCharacteristics);
  inclinationDescriptor.setValue("Inclination");
  inclinationCharacteristics.addDescriptor(&inclinationDescriptor);
  Serial.println("✓ Característica INCLINACIÓN creada: fedcba09-8765-4321-fedc-ba0987654321");
  
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
  Serial.println("📡 Sistema BLE TEST activo - esperando conexión...");
  Serial.println("📱 INSTRUCCIONES CRÍTICAS para nRF Connect:");
  Serial.println("   1. Conectar a 'CamperGas_Sensor_TEST'");
  Serial.println("   2. Ir al servicio: 91bad492-b950-4226-aa2b-4ede9fa42f59");
  Serial.println("   3. ⚠️ IMPORTANTE: Habilitar notificaciones en:");
  Serial.println("      87654321-4321-4321-4321-cba987654321 (OFFLINE DATA)");
  Serial.println("   4. Los datos offline se enviarán en LOTES por su propia característica");
  Serial.println("   5. Formato: [{\"w\":peso,\"t\":timestamp_seg},{\"w\":peso2,\"t\":timestamp_seg2}]");
  Serial.println("   6. MTU: 512 bytes para máxima eficiencia");
  Serial.println("   7. Timestamp en SEGUNDOS (no milisegundos)");
  Serial.println("═══════════════════════════════════════════════════════════");
  
  // Apagar sensores no críticos inicialmente
  powerDownSensors();
}

void loop() {
  unsigned long currentTime = millis();
  
  if (!deviceConnected) {
    // **SIMPLIFICADO PARA TEST**: No usar deep sleep, solo esperar conexión
    delay(1000);
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
      Serial.println("📱 Observa tu app nRF Connect ahora...");
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
    
    // Almacenar medida offline si es necesario (usar timestamp en segundos)
    unsigned long currentTimeSeconds = currentTime / 1000;
    // Nota: En el código de producción, aquí se almacenaría la medida offline cuando no hay conexión
    // storeOfflineMeasurement(peso, currentTimeSeconds);
    
    lastTime = currentTime;
  }
  
  // Light sleep entre lecturas para ahorrar energía
  enterLightSleep();
}
