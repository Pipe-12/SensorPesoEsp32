# Changelog - Optimización del Sensor BLE

## Cambios realizados en BLESensorPesoInclinacion.ino

### 🔄 Renombrado del archivo
- **Antes**: `BLESensorTemperatura.ino` (nombre incorrecto)
- **Después**: `BLESensorPesoInclinacion.ino` (nombre que refleja la funcionalidad real)
- **Razón**: El sensor mide peso e inclinación, no temperatura

### ⚡ Optimizaciones de energía
1. **Gestión mejorada del ADXL345**:
   - Añadidos registros de control de energía del ADXL345
   - `powerDownSensors()` ahora pone realmente el ADXL345 en modo standby
   - `powerUpSensors()` despierta el ADXL345 al modo de medición

2. **Implementación real de light sleep**:
   - `enterLightSleep()` ahora usa `esp_light_sleep_start()` cuando no está conectado
   - Mejor ahorro de energía en modo offline

3. **Eliminación del watchdog timer disable**:
   - Removido `esp_task_wdt_delete(NULL)` que podía causar problemas

### 🔧 Optimizaciones de código
1. **Constantes nombradas**:
   - Añadidas constantes para registros del ADXL345
   - Constantes para timeouts y configuraciones del HX711
   - Constantes para número de lecturas promedio

2. **Variables const apropiadas**:
   - `factor_calibracion` marcada como const
   - `pinData` y `pinClk` marcadas como const
   - Delays configurables como constantes

3. **Mejoras en manejo de errores**:
   - Mejor reporte de timeouts en HX711
   - Uso de constantes para número de intentos y lecturas

### 🛠️ Correcciones de bugs
1. **Gestión de energía del ADXL345**:
   - Corregido: `powerDownSensors()` ahora realmente apaga el sensor
   - Añadida configuración inicial del ADXL345 en modo medición

2. **Uso consistente de constantes**:
   - Reemplazados números mágicos con constantes nombradas
   - Mejor mantenibilidad del código

### 📊 Funcionalidad preservada
- ✅ Todas las funciones BLE mantienen su comportamiento
- ✅ Sistema de medidas offline sin cambios
- ✅ Callbacks y características BLE intactas
- ✅ Gestión de memoria circular preservada
- ✅ Formato JSON de datos sin cambios

### 🔍 Beneficios obtenidos
1. **Mejor consumo de energía**: ADXL345 en standby cuando no se usa
2. **Código más mantenible**: Constantes nombradas en lugar de números mágicos  
3. **Nombre de archivo correcto**: Refleja la funcionalidad real del sensor
4. **Light sleep real**: Mejor ahorro de energía cuando no está conectado
5. **Mejor legibilidad**: Código más organizado y documentado