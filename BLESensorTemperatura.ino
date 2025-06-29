#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
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
bool bleActive = false;

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 30000; // 30 segundos
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
  initHX711();
  initADXL345();
  sensorsInitialized = true;
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
  // Light sleep por corto tiempo cuando está conectado
  esp_sleep_enable_timer_wakeup(LIGHT_SLEEP_TIME_CONNECTED * 1000000ULL);
  esp_light_sleep_start();
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
  // Iniciar sensor solo si no está inicializado
  if (!bascula.is_ready()) {
    bascula.begin(pinData, pinClk);
    bascula.set_scale(factor_calibracion);
    bascula.tare();
    
    long zero_factor = bascula.read_average();
    Serial.print("Zero factor: ");
    Serial.println(zero_factor);
  }
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
    
    // Tomar medida offline rápida
    float offlineWeight = -1 * bascula.get_units();
    unsigned long currentTime = millis();
    storeOfflineMeasurement(offlineWeight, currentTime);
    
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

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Services (un solo servicio con 3 características)
  BLEService *sensorService = pServer->createService(SENSOR_SERVICE_UUID);

  // Create BLE Characteristics and Create a BLE Descriptor
  // Weight (includes timestamp)
  sensorService->addCharacteristic(&weightCharacteristics);
  weightDescriptor.setValue("HX711 weight reading with timestamp");
  weightCharacteristics.addDescriptor(&weightDescriptor);
  
  // Offline Data
  sensorService->addCharacteristic(&offlineDataCharacteristics);
  offlineDataDescriptor.setValue("HX711 offline stored data");
  offlineDataCharacteristics.addDescriptor(&offlineDataDescriptor);
  
  // Inclination Data
  sensorService->addCharacteristic(&inclinationCharacteristics);
  inclinationDescriptor.setValue("ADXL345 inclination pitch and roll");
  inclinationCharacteristics.addDescriptor(&inclinationDescriptor);
  
  // Start the service
  sensorService->start();

  // Start advertising con configuración de bajo consumo
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SENSOR_SERVICE_UUID);
  
  // Configurar advertising para ahorro de energía
  pAdvertising->setMinInterval(1600); // 1000ms
  pAdvertising->setMaxInterval(3200); // 2000ms
  
  pServer->getAdvertising()->start();
  Serial.println("Sistema en modo bajo consumo - esperando conexión...");
  
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
    // Read weight from HX711 sensor
    peso = -1 * bascula.get_units();
    
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