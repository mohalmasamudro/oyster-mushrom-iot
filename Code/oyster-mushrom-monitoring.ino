#include <Wire.h>
#include <BH1750.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <MQUnifiedsensor.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#define SEALEVELPRESSURE_HPA (1013.25)

#define placa "ESP-32"
#define Voltage_Resolution 3.3
#define pin 34 
#define type "MQ-135" 
#define ADC_Bit_Resolution 12 
#define RatioMQ135CleanAir 3.6

MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);

//=== Sensor Lingkungan ====
BH1750 luxSensor;
Adafruit_BME280 bme;

//=== Sensor Baglog ===
const int phpin = 35;
const int humpin = 32;

// Variable Sensor Lingkungan
float lux; // Cahaya
float temperature, humidity, pressure; // Sensor BME

// Variable Sensor Baglog
float adcph; // pH
float voltage;
float pH;
float adchum; // Kelembaban
float hum;

// variable sensor MQ135
float CO2 = 0.0;
float NH4 = 0.0;
float CO = 0.0;
float Alcohol = 0.0;
float Aceton = 0.0;
float Toluen  = 0.0;

// paket data
char data_gas[100];
char data_allsensor[300];
/*
const char* ssid = "BRIN_Net"; //BRINnet
const char* password = "";
const char* mqtt_server = "192.168.101.5";
const char* topic = "smartfarm/70/sensors";
*/

const char* ssid = "tanijamur";
const char* password = "jamurcikuda2024";
const char* mqtt_server = "103.144.45.116";
const char* topic = "smartfarm/70/sensors";

WiFiClient espClient;
PubSubClient client(espClient);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 25200, 60000); // Adjust the offset according to your timezone

unsigned long lastReadTime = 0;
unsigned long lastPublishTime = 0;
const long readInterval = 1000; // Interval baca sensor 1 detik
const long publishInterval = 5000; // Interval publish 1 menit (60000 ms)

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      client.subscribe(topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void statusluxsensor() {
  Wire.begin();
  if (luxSensor.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("BH1750 sensor started successfully");
  } else {
    Serial.println("Error initializing BH1750 sensor");
  }
}

void statusbmesensor() {
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  } else {
    Serial.println("BME280 sensor initialized successfully");
  }
}

void Read_Lux() 
{
  lux = luxSensor.readLightLevel();
}
void Read_Bme() { 
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
}

void Read_PH() {
  adcph = analogRead(phpin);
  voltage = adcph * (3.3 / 4095.0); 
  pH = -0.0152*adcph + 17.876;
  if (pH < 0) pH = 0;
  if (pH > 14) pH = 14;
}

void Read_HUM() {
  adchum = analogRead(humpin);
  hum=-0.04* adchum + 120.78;
}

void setup_MQ135() {
  MQ135.setRegressionMethod(1); // _PPM =  a*ratio^b
  MQ135.init(); 
  
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i <= 10; i++) {
    MQ135.update();
    float val = MQ135.calibrate(RatioMQ135CleanAir);
    calcR0 += val;
    Serial.print(".");
    delay(500);
  }
  calcR0 = calcR0 / 10;
  MQ135.setR0(calcR0);
  Serial.println(" done!");

  Serial.print("R0: ");
  Serial.println(calcR0);

  if (isinf(calcR0)) {
    Serial.println("Warning: Connection issue, R0 is infinite (Open circuit detected). Please check your wiring and supply.");
    while(1);
  }
  if (calcR0 == 0) {
    Serial.println("Warning: Connection issue found, R0 is zero (Analog pin shorts to ground). Please check your wiring and supply.");
    while(1);
  }
  Serial.println("MQ135 Calibration Complete!");
}

void Read_Gas()
{
  MQ135.update();
  MQ135.setA(605.18); MQ135.setB(-3.937); 
   CO = MQ135.readSensor(); 
  MQ135.setA(77.255); MQ135.setB(-3.18); 
   Alcohol = MQ135.readSensor(); 
  MQ135.setA(110.47); MQ135.setB(-2.862); 
   CO2 = MQ135.readSensor(); 
  MQ135.setA(44.947); MQ135.setB(-3.445); 
  Toluen = MQ135.readSensor(); 
  MQ135.setA(102.2); MQ135.setB(-2.473); 
   NH4 = MQ135.readSensor(); 
  MQ135.setA(34.668); MQ135.setB(-3.369); 
   Aceton = MQ135.readSensor(); 

  sprintf(data_gas, "{\"CO\": %.d, \"Alcohol\": %.d, \"CO2\": %.d, \"Toluen\": %.d, \"NH4\": %.d, \"Aceton\": %.d}",
          CO, Alcohol, CO2 + 400, Toluen, NH4, Aceton);
  delay(500); 
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Message arrived: ");
  Serial.println(message);

  if (message == "1") {
    Serial.println("Restart di mulai");
    delay(1000);
    esp_restart();
    
  } 
}

String getFormattedDate() {
  time_t rawtime = timeClient.getEpochTime();
  struct tm * timeinfo;
  char buffer[80];

  timeinfo = localtime(&rawtime);
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);

  return String(buffer);
}

//=====================================setup===========================================
void setup() {
  Serial.begin(9600);
  while (!Serial);
  setup_MQ135();
  statusbmesensor();
  statusluxsensor();
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  timeClient.begin();
}

//================================= loop==================================================
void loop() {
  if (!client.connected()) { 
    reconnect();
  }
  client.loop();
  timeClient.update();

  unsigned long now = millis();
  if (now - lastReadTime > readInterval) {
    lastReadTime = now;
    Read_Lux();
    Read_Gas();
    Read_Bme();
    Read_PH();
    Read_HUM();

    StaticJsonDocument<300> doc;
    doc["TIPEDATA"] = "A";
    doc["FARMID"] = "70";
    doc["timeNow"] = getFormattedDate();
    doc["LUX01"] = lux;
    doc["COO01"] = CO;
    doc["CO002"] = CO2 + 400;
    doc["NHO01"] = NH4;
    doc["TEM01"] = temperature;
    doc["HUM01"] = humidity;
    doc["HUM02"] = hum;
    doc["PRE01"] = pressure;
    doc["PH001"] = pH;
    serializeJson(doc, data_allsensor);
  }
  if (now - lastPublishTime > publishInterval) {
    lastPublishTime = now;
    
    StaticJsonDocument<300> doc;
    doc["TIPEDATA"] = "A";
    doc["FARMID"] = "70";
    doc["timeNow"] = getFormattedDate();
    doc["LUX01"] = lux;
    doc["COO01"] = CO;
    doc["CO002"] = CO2 + 400;
    doc["NHO01"] = NH4;
    doc["TEM01"] = temperature;
    doc["HUM01"] = humidity;
    doc["HUM02"] = hum;
    doc["PRE01"] = pressure;
    doc["PH001"] = pH;
    serializeJson(doc, data_allsensor);
    Serial.println(data_allsensor);
    client.publish(topic, data_allsensor);
  }
}
