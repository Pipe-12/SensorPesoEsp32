#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <HX711.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

//BLE server name
#define bleServerName "HX711_ESP32"

// Pin de datos y de reloj para HX711
byte pinData = 4;
byte pinClk = 2;

HX711 bascula;

// Crear el objeto del sensor ADXL345
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

float factor_calibracion = 25000.0;  //Factor de calibracion
float peso;
float pitch, roll; // Variables para inclinación

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 30000; // 30 segundos
unsigned long lastInclinationTime = 0;
unsigned long inclinationTimerDelay = 10000; // 10 segundos para inclinación
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
#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42f59"
#define OFFLINE_SERVICE_UUID "12345678-1234-1234-1234-123456789abc"
#define INCLINATION_SERVICE_UUID "abcdef12-3456-7890-abcd-ef1234567890"

// Weight Characteristic and Descriptor (includes timestamp)
BLECharacteristic weightCharacteristics("cba1d466-344c-4be3-ab3f-189f80dd7518", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor weightDescriptor(BLEUUID((uint16_t)0x2902));

// Offline Data Characteristic and Descriptor
BLECharacteristic offlineDataCharacteristics("87654321-4321-4321-4321-cba987654321", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor offlineDataDescriptor(BLEUUID((uint16_t)0x2904));

// Inclination Characteristic and Descriptor
BLECharacteristic inclinationCharacteristics("fedcba09-8765-4321-fedc-ba0987654321", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor inclinationDescriptor(BLEUUID((uint16_t)0x2905));

//Setup callbacks onConnect and onDisconnect
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Cliente conectado - enviando datos offline...");
    sendOfflineData();
    // Reiniciar sistema offline al conectarse
    resetOfflineSystem();
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Cliente desconectado - modo offline activado");
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

// Función para enviar datos offline cuando se conecta BLE
void sendOfflineData() {
  if (offlineMeasurementCount > 0) {
    Serial.print("Enviando ");
    Serial.print(offlineMeasurementCount);
    Serial.println(" medidas offline...");
    
    for (int i = 0; i < offlineMeasurementCount; i++) {
      static char offlineDataString[50];
      sprintf(offlineDataString, "OFFLINE: %.2f kg | %lu ms", 
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
  // Iniciar sensor
  bascula.begin(pinData, pinClk);

  // Aplicar la calibracion
  bascula.set_scale(factor_calibracion);

  bascula.tare();

  // Obtener una lectura de referencia
  long zero_factor = bascula.read_average();
  // Mostrar la primera desviación
  Serial.print("Zero factor: ");
  Serial.println(zero_factor);
}

void initADXL345(){
  // Inicializar I2C explícitamente con los pines correctos
  Wire.begin(21, 22);  // SDA a GPIO21, SCL a GPIO22 en ESP32

  Serial.println("Iniciando el ADXL345...");

  // Inicializar el ADXL345
  if (!accel.begin()) {
    Serial.println("No se pudo encontrar el ADXL345");
    while (1);  // Detener ejecución
  }

  Serial.println("ADXL345 conectado correctamente");
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
  // Start serial communication 
  Serial.begin(115200);
  delay(1000);

  // Init HX711 Sensor
  initHX711();

  // Init ADXL345 Sensor
  initADXL345();

  // Create the BLE Device
  BLEDevice::init(bleServerName);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Services
  BLEService *weightService = pServer->createService(SERVICE_UUID);
  BLEService *offlineService = pServer->createService(OFFLINE_SERVICE_UUID);
  BLEService *inclinationService = pServer->createService(INCLINATION_SERVICE_UUID);

  // Create BLE Characteristics and Create a BLE Descriptor
  // Weight (includes timestamp)
  weightService->addCharacteristic(&weightCharacteristics);
  weightDescriptor.setValue("HX711 weight reading with timestamp");
  weightCharacteristics.addDescriptor(&weightDescriptor);
  
  // Offline Data
  offlineService->addCharacteristic(&offlineDataCharacteristics);
  offlineDataDescriptor.setValue("HX711 offline stored data");
  offlineDataCharacteristics.addDescriptor(&offlineDataDescriptor);
  
  // Inclination Data
  inclinationService->addCharacteristic(&inclinationCharacteristics);
  inclinationDescriptor.setValue("ADXL345 inclination pitch and roll");
  inclinationCharacteristics.addDescriptor(&inclinationDescriptor);
  
  // Start the services
  weightService->start();
  offlineService->start();
  inclinationService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->addServiceUUID(OFFLINE_SERVICE_UUID);
  pAdvertising->addServiceUUID(INCLINATION_SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Medidas offline cada 15 minutos (siempre activo)
  if ((currentTime - lastOfflineTime) > offlineTimerDelay) {
    // Read weight from HX711 sensor
    float offlineWeight = -1 * bascula.get_units();
    
    if (!deviceConnected) {
      // Si no está conectado, almacenar en memoria
      storeOfflineMeasurement(offlineWeight, currentTime);
    }
    
    lastOfflineTime = currentTime;
  }
  
  // Medidas en tiempo real cada 30 segundos (solo cuando está conectado)
  if (deviceConnected) {
    if ((currentTime - lastTime) > timerDelay) {
      // Read weight from HX711 sensor
      peso = -1 * bascula.get_units();
  
      //Notify weight reading with timestamp from HX711 sensor
      static char weightWithTimestamp[30];
      sprintf(weightWithTimestamp, "LIVE: %.2f kg | %lu ms", peso, currentTime);
      //Set weight+timestamp Characteristic value and notify connected client
      weightCharacteristics.setValue(weightWithTimestamp);
      weightCharacteristics.notify();
      
      Serial.print("Peso en vivo: ");
      Serial.print(peso, 1);
      Serial.print(" kgs");
      Serial.print(" | Timestamp: ");
      Serial.print(currentTime);
      Serial.print(" ms");
      Serial.print(" | factor_calibracion: ");
      Serial.print(factor_calibracion);
      Serial.println();
      
      lastTime = currentTime;
    }
    
    // Medidas de inclinación cada 10 segundos (solo cuando está conectado)
    if ((currentTime - lastInclinationTime) > inclinationTimerDelay) {
      // Read inclination from ADXL345 sensor
      readInclination();
      
      //Notify inclination reading from ADXL345 sensor
      static char inclinationData[40];
      sprintf(inclinationData, "Pitch: %.2f° | Roll: %.2f° | %lu ms", pitch, roll, currentTime);
      //Set inclination Characteristic value and notify connected client
      inclinationCharacteristics.setValue(inclinationData);
      inclinationCharacteristics.notify();
      
      Serial.print("Inclinación - Pitch: ");
      Serial.print(pitch, 2);
      Serial.print("°, Roll: ");
      Serial.print(roll, 2);
      Serial.print("° | Timestamp: ");
      Serial.print(currentTime);
      Serial.println(" ms");
      
      lastInclinationTime = currentTime;
    }
  }
}