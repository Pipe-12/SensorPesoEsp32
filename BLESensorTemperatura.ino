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
#define bleServerName "CamperGas_Sensor"

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
unsigned long timerDelay = 60000; // 60 segundos - aumentado para dar más tiempo a la conexión
unsigned long lastOfflineTime = 0;
unsigned long offlineTimerDelay = 900000; // 15 minutos (15 * 60 * 1000 ms)
unsigned long initialOfflineDelay = 900000; // 15 minutos inicial
unsigned long maxOfflineDelay = 86400000; // 24 horas (24 * 60 * 60 * 1000 ms)
int offlineIndex = 0; // Índice circular para reemplazar medidas antiguas

// Estructura para almacenar datos offline
struct MeasurementData {
  float weight;
  unsigned long timestamp;
};

// Array para almacenar medidas offline (máximo 100 medidas)
#define MAX_OFFLINE_MEASUREMENTS 100
MeasurementData offlineMeasurements[MAX_OFFLINE_MEASUREMENTS];
int offlineMeasurementCount = 0;

bool deviceConnected = false;

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
    Serial.println("Cliente conectado - modo activo");
    
    // Delay para estabilizar la conexión antes de habilitar notificaciones
    delay(1000);
    
    // Habilitar automáticamente las notificaciones para todos los servicios
    enableAllNotifications();
    
    sendOfflineData();
    resetOfflineSystem();
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
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
    Serial.print(" ms. Total almacenadas: ");
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
    Serial.print(" ms. Índice: ");
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
  // Habilitar notificaciones para el servicio de peso
  uint8_t notificationOn[] = {0x01, 0x00};
  weightCharacteristics.getDescriptorByUUID(BLEUUID((uint16_t)0x2902))->setValue(notificationOn, 2);
  
  // Habilitar notificaciones para el servicio offline
  offlineDataCharacteristics.getDescriptorByUUID(BLEUUID((uint16_t)0x2902))->setValue(notificationOn, 2);
  
  // Habilitar notificaciones para el servicio de inclinación
  inclinationCharacteristics.getDescriptorByUUID(BLEUUID((uint16_t)0x2902))->setValue(notificationOn, 2);
  
  Serial.println("Notificaciones habilitadas automáticamente para todos los servicios");
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
    Serial.print("Enviando ");
    Serial.print(offlineMeasurementCount);
    Serial.println(" medidas offline...");
    
    for (int i = 0; i < offlineMeasurementCount; i++) {
      static char offlineDataString[60];
      // Formato JSON simplificado - solo datos esenciales
      sprintf(offlineDataString, "{\"weight\":%.2f,\"timestamp\":%lu}", 
              offlineMeasurements[i].weight, 
              offlineMeasurements[i].timestamp);
      
      offlineDataCharacteristics.setValue(offlineDataString);
      offlineDataCharacteristics.notify();
      delay(100); // Pequeño delay entre envíos para evitar saturar BLE
    }
    
    // Vaciar memoria después de enviar
    offlineMeasurementCount = 0;
    Serial.println("Datos offline enviados y memoria vaciada.");
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
  
  // Verificar causa del reset/despertar
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  
  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    Serial.println("Despertar desde deep sleep - tomando medida offline");
    
    // Solo inicializar HX711 para medida rápida
    initHX711();
    
    // Verificar que la báscula funciona antes de leer
    if (bascula.is_ready()) {
      float offlineWeight = -1 * bascula.get_units(3);
      unsigned long currentTime = millis();
      storeOfflineMeasurement(offlineWeight, currentTime);
      Serial.print("Medida offline tomada: ");
      Serial.print(offlineWeight);
      Serial.println(" kg");
    } else {
      Serial.println("ERROR: HX711 no está listo para medida offline");
    }
    
    // Volver a deep sleep inmediatamente
    enterDeepSleep();
  }

  // Inicialización completa solo en arranque normal
  Serial.println("Inicialización completa...");
  
  // Deshabilitar WiFi para ahorrar energía
  esp_wifi_stop();
  
  // Init Sensors
  initSensors();

  // Create the BLE Device
  BLEDevice::init(bleServerName);
  
  // Configurar parámetros de conexión para mayor estabilidad
  BLEDevice::setMTU(185); // Reducir MTU para mayor compatibilidad

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
  
  // Offline Data
  sensorService->addCharacteristic(&offlineDataCharacteristics);
  offlineDataDescriptor.setValue("Offline");
  offlineDataCharacteristics.addDescriptor(&offlineDataDescriptor);
  
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
  
  // Apagar sensores no críticos inicialmente
  powerDownSensors();
}

void loop() {
  unsigned long currentTime = millis();
  
  if (!deviceConnected) {
    // Modo offline - usar deep sleep para máximo ahorro
    static unsigned long lastDeepSleep = 0;
    
    if ((currentTime - lastDeepSleep) > 30000) { // Esperar 30s antes de deep sleep
      Serial.println("No hay conexión - entrando en deep sleep");
      enterDeepSleep(); // Nunca retorna de aquí
    }
    
    // Light sleep corto mientras espera conexión
    delay(1000);
    return;
  }
  
  // Dispositivo conectado - modo activo
  if (!sensorsInitialized) {
    powerUpSensors();
  }
  
  // Medidas en tiempo real cada 30 segundos (peso + inclinación simultáneamente)
  if ((currentTime - lastTime) > timerDelay) {
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

    // Notify weight reading with timestamp - Formato JSON simplificado
    static char weightData[60];
    sprintf(weightData, "{\"weight\":%.2f,\"timestamp\":%lu}", peso, currentTime);
    weightCharacteristics.setValue(weightData);
    weightCharacteristics.notify();
    
    // Notify inclination reading - Formato JSON simplificado
    static char inclinationData[80];
    sprintf(inclinationData, "{\"pitch\":%.2f,\"roll\":%.2f,\"timestamp\":%lu}", 
            pitch, roll, currentTime);
    inclinationCharacteristics.setValue(inclinationData);
    inclinationCharacteristics.notify();
    
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