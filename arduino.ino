// ==================== INCLUDES ====================
#include <ArduinoJson.h>
#include <DHT.h>

// ==================== IDENTIFICACIÓN DEL DISPOSITIVO ====================
const char* HARDWARE_ID = "245_HWID_1751010607747";

// ==================== CONFIGURACIÓN INICIAL ====================
unsigned long measurementIntervalMs = 60000;       // 1 minuto
unsigned long photoCaptureIntervalMs = 6 * 3600000UL; // 6 horas
unsigned long lastSensorReadTime = 0;
unsigned long lastPhotoTime = 0;
unsigned long lastDebugPrintTime = 0;
const unsigned long debugPrintIntervalMs = 5000;
bool modoManualLuz = false; 
bool modoManualVentilador = false;
bool modoManualValvula = false;

String temperatureUnit = "CELSIUS";
bool autoIrrigationEnabled = true;
int irrigationThresholdPercent = 30;
bool autoVentilationEnabled = true;
float temperatureOnThreshold = 30.0;
float temperatureOffThreshold = 28.0;
bool modoPruebaActivo = false;
bool hardwareIdEnviado = false;

// ==================== CONFIGURACIÓN DE PINES ====================
#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

const int ldrPin = A1;
const int VALVE_PIN = A2;
const int FAN_PIN = A3;
const int echoPin = A4;
const int trigPin = A5;
const int ledPin = 3;
const int waterLevelPin = 12;
const int ledInterno = 13;
const int umbralLuz = 200;
int lightLevel = 0;

// ==================== SETUP ====================
void setup() {
  Serial.begin(9600);
  while (!Serial);

  while(Serial.available() > 0){
    Serial.read();
  }

  delay(2000);

  for (int i = 0; i < 3; i++){
    StaticJsonDocument<200> helloDoc;
    helloDoc["type"] = "hello_arduino";
    helloDoc["hardwareId"] = HARDWARE_ID;
    serializeJson(helloDoc, Serial);
    Serial.println();
    delay(500);
  }

  hardwareIdEnviado = true;

  pinMode(ledPin, OUTPUT);
  pinMode(waterLevelPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(VALVE_PIN, OUTPUT);
  pinMode(ledInterno, OUTPUT);

  digitalWrite(ledPin, LOW);
  digitalWrite(FAN_PIN, HIGH);
  digitalWrite(VALVE_PIN, HIGH);
  digitalWrite(ledInterno, LOW);

  dht.begin();
}

// ==================== LOOP ====================
void loop() {
  unsigned long currentMillis = millis();

  if (!modoPruebaActivo) {
    processSerialCommands();

    if (currentMillis - lastSensorReadTime >= measurementIntervalMs) {
      sendSensorData();
      lastSensorReadTime = currentMillis;
    }

    if (currentMillis - lastPhotoTime >= photoCaptureIntervalMs) {
      lastPhotoTime = currentMillis;
      // Lógica de captura de foto iría aquí
    }

    if (currentMillis - lastDebugPrintTime >= debugPrintIntervalMs) {
      //printSensorDataDebugPeriodically();
      lastDebugPrintTime = currentMillis;
    }

    actualizarLightLevel();
    controlarLuz();
    checkAutoVentilation();
    checkAutoIrrigation();
  } else {
    modoDePruebaInteractivo();
    modoPruebaActivo = false;
  }
}

template<typename T>
void sendAck(const char* ackType, const char* valueKey, T value, const char* hwId) 
{
  StaticJsonDocument<300> ackDoc;
  ackDoc["type"] = ackType;
  ackDoc[valueKey] = value;
  ackDoc["hardwareId"] = hwId;
  serializeJson(ackDoc, Serial);
  Serial.println();
}

void processSerialCommands() {
  if (!hardwareIdEnviado) return;

  while (Serial.available() > 0) {
    String rawInput = Serial.readStringUntil('\n');
    rawInput.trim();

    if (rawInput.length() == 0) continue;

    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, rawInput);

    if (error) {
      if (rawInput == "test") {
        modoPruebaActivo = true;
      } else {
        StaticJsonDocument<200> errorDoc;
        errorDoc["type"] = "parse_error";
        errorDoc["hardwareId"] = HARDWARE_ID;
        errorDoc["error"] = error.c_str();
        errorDoc["raw_input"] = rawInput;
        serializeJson(errorDoc, Serial);
        Serial.println();
      }
      return;
    }

    const char* command = doc["command"];
    if (command) {
      // ------------------ LED ------------------
      if (strcmp(command, "set_led") == 0) {
        if (doc.containsKey("state") && doc["state"].is<int>()) {
          int state = doc["state"].as<int>();
          if (state == 0 || state == 1) {
            modoManualLuz = true;  // SIEMPRE activar modo manual
            digitalWrite(ledPin, state == 1 ? HIGH : LOW);
            sendAck("ack_led_set", "state", state, HARDWARE_ID);
          }
        }
      }

      // ------------------ FAN ------------------
      else if (strcmp(command, "set_fan") == 0) {
        if (doc.containsKey("state") && doc["state"].is<int>()) {
          int state = doc["state"].as<int>();
          if (state == 0 || state == 1) {
            modoManualVentilador = true;  // SIEMPRE activar modo manual
            digitalWrite(FAN_PIN, state == 1 ? LOW : HIGH);
            sendAck("ack_fan_set", "state", state, HARDWARE_ID);
          }
        }
      }

      // ------------------ VALVE ------------------
      else if (strcmp(command, "set_valve") == 0) {
        if (doc.containsKey("state") && doc["state"].is<int>()) {
          int state = doc["state"].as<int>();
          if (state == 0 || state == 1) {
            modoManualValvula = true;  // SIEMPRE activar modo manual
            digitalWrite(VALVE_PIN, state == 1 ? LOW : HIGH);
            sendAck("ack_valve_set", "state", state, HARDWARE_ID);
          }
        }
      }

      // ------------------ INTERVALO ------------------
      else if (strcmp(command, "set_interval") == 0) {
        if (doc.containsKey("value_ms")) {
          measurementIntervalMs = doc["value_ms"].as<unsigned long>();
          sendAck("ack_interval_set", "new_interval_ms", measurementIntervalMs, HARDWARE_ID);
        }
      }

      // ------------------ INTERVALO DE FOTOS ------------------
      else if (strcmp(command, "set_photo_interval") == 0) {
        if (doc.containsKey("value_hours")) {
          int hours = doc["value_hours"].as<int>();
          photoCaptureIntervalMs = (unsigned long)hours * 3600000UL;
          sendAck("ack_photo_interval_set", "new_interval_hours", hours, HARDWARE_ID);
        }
      }

      // ------------------ UNIDAD DE TEMPERATURA ------------------
      else if (strcmp(command, "set_temp_unit") == 0) {
        if (doc.containsKey("unit")) {
          temperatureUnit = doc["unit"].as<String>();
          sendAck("ack_temp_unit_set", "new_unit", temperatureUnit.c_str(), HARDWARE_ID);
        }
      }

      // ------------------ RIEGO AUTOMÁTICO ------------------
      else if (strcmp(command, "set_auto_irrigation") == 0) {
        if (doc.containsKey("enabled") && doc.containsKey("threshold")) {
          autoIrrigationEnabled = doc["enabled"].as<bool>();
          irrigationThresholdPercent = doc["threshold"].as<int>();

          StaticJsonDocument<256> ackDoc;
          ackDoc["type"] = "ack_auto_irrigation_set";
          ackDoc["enabled"] = autoIrrigationEnabled;
          ackDoc["threshold"] = irrigationThresholdPercent;
          ackDoc["hardwareId"] = HARDWARE_ID;
          serializeJson(ackDoc, Serial);
          Serial.println();
        }
      }

      // ------------------ VENTILACIÓN AUTOMÁTICA ------------------
      else if (strcmp(command, "set_auto_ventilation") == 0) {
        if (doc.containsKey("enabled") && doc.containsKey("temp_on") && doc.containsKey("temp_off")) {
          autoVentilationEnabled = doc["enabled"].as<bool>();
          temperatureOnThreshold = doc["temp_on"].as<float>();
          temperatureOffThreshold = doc["temp_off"].as<float>();

          StaticJsonDocument<300> ackDoc;
          ackDoc["type"] = "ack_auto_ventilation_set";
          ackDoc["enabled"] = autoVentilationEnabled;
          ackDoc["temp_on"] = temperatureOnThreshold;
          ackDoc["temp_off"] = temperatureOffThreshold;
          ackDoc["hardwareId"] = HARDWARE_ID;
          serializeJson(ackDoc, Serial);
          Serial.println();
        }
      }

      // ------------------ MODO PRUEBA ------------------
      else if (strcmp(command, "test") == 0) {
        modoPruebaActivo = true;
      }
    }
  }
}

void sendSensorData() {
  // --- LECTURA DE SENSORES ---
  float temperature = dht.readTemperature();      
  float airHumidity = dht.readHumidity();         
  int soilHumidity = map(analogRead(A0), 0, 1023, 100, 0); 
  int rawWaterLevel = digitalRead(waterLevelPin);      
  int waterLevel = !rawWaterLevel;               

  actualizarLightLevel();

  long drainageDistance = leerDistanciaUltrasonico();
  bool isDraining = (drainageDistance <= 20);         

  // --- CONSTRUCCIÓN DEL JSON DE DATOS A ENVIAR ---
  StaticJsonDocument<384> dataDoc;
  dataDoc["hardwareId"] = HARDWARE_ID;
  dataDoc["temperature"] = temperature;
  dataDoc["airHumidity"] = airHumidity;
  dataDoc["soilHumidity"] = soilHumidity;
  dataDoc["lightLevel"] = lightLevel;
  dataDoc["waterLevel"] = waterLevel;              
  dataDoc["drainageDistance"] = drainageDistance;
  dataDoc["draining"] = isDraining;

  // --- ENVÍO DEL JSON POR SERIAL ---
  serializeJson(dataDoc, Serial);
  Serial.println(); 
}

void printSensorDataDebugPeriodically() {
  if (millis() - lastDebugPrintTime >= debugPrintIntervalMs) {
    float temperature = dht.readTemperature();     
    float airHumidity = dht.readHumidity();       

    // Validación para evitar imprimir NaN si hay error
    if (isnan(temperature) || isnan(airHumidity)) {
      Serial.println("Failed to read from DHT sensor (DEBUG)!");
      return;
    }

    int soilHumidity = map(analogRead(A0), 0, 1023, 100, 0);
    int lightLevel = analogRead(A1);
    int waterLevel = digitalRead(waterLevelPin);

    printSensorDataDebug(temperature, airHumidity, soilHumidity, lightLevel, waterLevel);

    lastDebugPrintTime = millis();
  }
}


void actualizarLightLevel() {
  lightLevel = analogRead(A1);
}

void controlarLuz() {
  if (modoManualLuz) return;

  int valorLuz = analogRead(ldrPin);
  if (valorLuz < umbralLuz) {
    digitalWrite(ledPin, HIGH);  
  } else {
    digitalWrite(ledPin, LOW);  
  }
}


void printSensorDataDebug(float temperature, float airHumidity, int soilHumidity, int lightLevel, int waterLevel) {
  //Serial.println("----- Sensor Readings (DEBUG) -----");
  //Serial.print("Temperature: ");
  //Serial.print(temperature);
  //Serial.println(" °C");

  //Serial.print("Air Humidity: ");
  //Serial.print(airHumidity);
  //Serial.println(" %");

  //Serial.print("Soil Humidity: ");
  //Serial.print(soilHumidity);
  //Serial.println(" %");

  //Serial.print("Light Level: ");
  //Serial.println(lightLevel);

  //Serial.print("Water Level (digital): ");
  //Serial.println(waterLevel == HIGH ? "LOW" : "HIGH");

  long drainageDistance = leerDistanciaUltrasonico();
  bool isDraining = (drainageDistance <= 20);

  //Serial.print("Drainage Distance: ");
  //Serial.print(drainageDistance);
  //Serial.println(" cm");

  //Serial.print("Draining: ");
  //Serial.println(isDraining ? "YES" : "NO");

  //Serial.print("Fan Relay: ");
  //Serial.println(digitalRead(FAN_PIN) == LOW ? "ON" : "OFF");

  //Serial.print("Valve Relay: ");
  //Serial.println(digitalRead(VALVE_PIN) == LOW ? "ON" : "OFF");
}

long leerDistanciaUltrasonico() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duracion = pulseIn(echoPin, HIGH, 30000); 
  long distanciaCm = duracion * 0.034 / 2; 

  return distanciaCm; 
}

void checkAutoIrrigation() {
  if (modoManualValvula) return;

  if (autoIrrigationEnabled) {
    int currentSoilHumidity = map(analogRead(A0), 0, 1023, 100, 0);
    if (currentSoilHumidity < irrigationThresholdPercent) {
      digitalWrite(VALVE_PIN, LOW); 
    } else {
      digitalWrite(VALVE_PIN, HIGH); 
    }
  } else {
    digitalWrite(VALVE_PIN, HIGH);
  }
}


void checkAutoVentilation() {
  if (modoManualVentilador) return; 

  if (autoVentilationEnabled) {
    float currentTemperature = dht.readTemperature();

    if (isnan(currentTemperature)) {
      Serial.println("Failed to read temperature for ventilation control.");
      return;
    }

    if (currentTemperature > temperatureOnThreshold) {
      digitalWrite(FAN_PIN, LOW);  // Encender (relé activo en LOW)
    } else if (currentTemperature < temperatureOffThreshold) {
      digitalWrite(FAN_PIN, HIGH); // Apagar
    }
  } else {
    digitalWrite(FAN_PIN, LOW); 
  }
}


void modoDePruebaInteractivo() {
  Serial.println("=== MODO DE PRUEBA ACTIVADO ===");
  Serial.println("Escribe: led 1, led 0, fan 1, fan 0, valve 1, valve 0 para controlar.");
  Serial.println("Escribe: exit para salir del modo de prueba.");

  bool prevAutoVent = autoVentilationEnabled;
  bool prevAutoIrrig = autoIrrigationEnabled;

  autoVentilationEnabled = false;
  autoIrrigationEnabled = false;

  String input = "";

  while (true) {
    if (Serial.available() > 0) {
      char ch = Serial.read();
      if (ch == '\n') {
        input.trim();

        if (input == "exit") {
          Serial.println("Saliendo del modo de prueba...");
          break;
        } else if (input.startsWith("led ")) {
          int val = input.substring(4).toInt();
          digitalWrite(ledPin, val ? HIGH : LOW);
          Serial.print("LED: ");
          Serial.println(val ? "ON" : "OFF");
        } else if (input.startsWith("fan ")) {
          int val = input.substring(4).toInt();
          digitalWrite(FAN_PIN, val ? LOW : HIGH);
          Serial.print("FAN: ");
          Serial.println(val ? "ON" : "OFF");
        } else if (input.startsWith("valve ")) {
          int val = input.substring(6).toInt();
          digitalWrite(VALVE_PIN, val ? LOW : HIGH);
          Serial.print("VALVE: ");
          Serial.println(val ? "ON" : "OFF");
        } else {
          Serial.println("Comando inválido. Usa: led 1, fan 0, valve 1, exit");
        }

        input = "";
      } else {
        input += ch;
      }
    }
  }

  autoVentilationEnabled = prevAutoVent;
  autoIrrigationEnabled = prevAutoIrrig;

  Serial.println("=== MODO DE PRUEBA DESACTIVADO ===");
}