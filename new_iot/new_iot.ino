#include <Adafruit_ADS1X15.h>
#include <MQUnifiedsensor.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
#include <Arduino_JSON.h>

#define PM1PIN 12
#define PM25PIN 14
byte buff[2];
unsigned long durationPM1;
unsigned long durationPM25;
unsigned long starttime;
unsigned long endtime;
unsigned long sampletime_ms = 30000;
unsigned long lowpulseoccupancyPM1 = 0;
unsigned long lowpulseoccupancyPM25 = 0;
float conPM1 = 0;
float conPM25 = 0;
float PM10;
float PM25;

unsigned long previousMillis = 0;
const long interval = 2000;

// Definitions
#define placa "ESP8266"
#define mq2 "MQ-2"
#define mq135 "MQ-135"
#define mq131 "MQ-131"
#define Voltage_Resolution 5
#define pin A0
#define ADC_Bit_Resolution 10
#define RatioMQ2CleanAir 9.83
#define RatioMQ135CleanAir 3.6
#define RatioMQ131CleanAir 15  // Adjust this value based on the sensor's datasheet

float factorEscala = 0.16875F;

MQUnifiedsensor MQ2(placa, mq2);
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, mq135);
MQUnifiedsensor MQ131(placa,  mq131);  // Initialize MQ131 sensor

Adafruit_ADS1115 ads;

const char* ssid = "KEDAI FILOSOFI";
const char* password = "filosofi00";
const String serverPath = "http://192.168.2.12:5000/api/esp/data"; // Use HTTP for simplicity
String espId = "ESP8266_1";

void setup(void)
{
  Serial.begin(9600);
  delay(200);

  pinMode(LED_BUILTIN, OUTPUT);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
  }
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(PM1PIN, INPUT);
  pinMode(PM25PIN, INPUT);
  starttime = millis();

  MQ2.setRegressionMethod(1);
  MQ135.setRegressionMethod(1);
  MQ131.setRegressionMethod(1);

  MQ2.init();
  MQ135.init();
  MQ131.init();

  if (!ads.begin())
  {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }

  Serial.print("Calibrating MQ-135 please wait.");
  float calcR0MQ135 = 0;
  for (int i = 1; i <= 10; i++)
  {
    MQ135.update();
    calcR0MQ135 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0MQ135 / 10);
  Serial.println("  done for MQ-135.");

  Serial.print("Calibrating MQ-2 please wait.");
  float calcR0MQ2 = 0;
  for (int i = 1; i <= 10; i++)
  {
    short adc0 = ads.readADC_SingleEnded(0);
    float voltiosMQ2 = (adc0 * factorEscala) / 1000.0;
    MQ2.externalADCUpdate(voltiosMQ2);
    calcR0MQ2 += MQ2.calibrate(RatioMQ2CleanAir);
    Serial.print(".");
  }
  MQ2.setR0(calcR0MQ2 / 10);
  Serial.println("  done for MQ-2.");

  Serial.print("Calibrating MQ-131 please wait.");
  float calcR0MQ131 = 0;
  for (int i = 1; i <= 10; i++)
  {
    short adc2 = ads.readADC_SingleEnded(1);
    float voltiosMQ131 = (adc2 * factorEscala) / 1000.0;
    MQ131.externalADCUpdate(voltiosMQ131);
    calcR0MQ131 += MQ131.calibrate(RatioMQ131CleanAir);
    Serial.print(".");
  }
  MQ131.setR0(calcR0MQ131 / 10);
  Serial.println("  done for MQ-131.");
}

void reconnectWiFi()
{
  Serial.println("Reconnecting to Wi-Fi...");
  WiFi.reconnect();
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30)
  {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
  else
  {
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    reconnectWiFi();
  }

  MQ135.update();
  MQ2.update();
  MQ131.update();

  MQ135.setA(605.18);
  MQ135.setB(-3.937);
  float MQ135_CO = MQ135.readSensor();

  MQ2.setA(36974);
  MQ2.setB(-3.109);
  float MQ2_HC = MQ2.readSensor();

  MQ131.setA(23.943); // Example values, please adjust based on the sensor's datasheet
  MQ131.setB(-1.11);
  float MQ131_O3 = MQ131.readSensor();

  durationPM1 = pulseIn(PM1PIN, LOW);
  durationPM25 = pulseIn(PM25PIN, LOW);

  lowpulseoccupancyPM1 += durationPM1;
  lowpulseoccupancyPM25 += durationPM25;

  endtime = millis();
  if ((endtime - starttime) > sampletime_ms)
  {
    conPM1 = calculateConcentration10(lowpulseoccupancyPM1, 30);
    conPM25 = calculateConcentration25(lowpulseoccupancyPM25, 30);
 if (conPM1 < 0)
    {
      conPM1 = 0;
    }
    if (conPM25 < 0)
    {
      conPM25 = 0;
    }
    PM10 = conPM1*1000;
    PM25 = conPM25*1000;
    Serial.print("PM1 ");
    Serial.print(conPM1);
    Serial.print("  PM25 ");
    Serial.println(conPM25);
    lowpulseoccupancyPM1 = 0;
    lowpulseoccupancyPM25 = 0;
    starttime = millis();

  }

  float ISPU_PM10 = calculateISPU_PM10(PM10);
  float ISPU_PM25 = calculateISPU_PM25(PM25);
  float ISPU_CO = calculateISPU_CO(MQ135_CO);
  float ISPU_HC = calculateISPU_HC(MQ2_HC);
  float ISPU_O3 = calculateISPU_O3(MQ131_O3); // Add this line to calculate ISPU for O3

  Serial.print("Sensor\t\t\t");
  Serial.print("Value (ppm)");
  Serial.println();

  Serial.print("MQ-135 CO:\t\t");
  Serial.print(MQ135_CO, 2);
  Serial.println(" ppm");

  Serial.print("MQ-2 HC:\t\t");
  Serial.print(MQ2_HC, 2);
  Serial.println(" ppm");

  Serial.print("MQ-131 O3:\t\t");
  Serial.print(MQ131_O3, 2);
  Serial.println(" ppm");

  Serial.print("PM 2.5:\t");
  Serial.print(PM25, 2);
  Serial.println(" ");
  Serial.print("PM 10:\t");
  Serial.print(PM10, 2);
  Serial.println(" ");

  Serial.print("ISPU PM10:\t");
  Serial.print(ISPU_PM10, 2);
  Serial.println(" ");
  Serial.print("ISPU PM25:\t");
  Serial.print(ISPU_PM25, 2);
  Serial.println(" ");
  Serial.print("ISPU CO:\t");
  Serial.print(ISPU_CO, 2);
  Serial.println(" ");
  Serial.print("ISPU HC:\t");
  Serial.print(ISPU_HC, 2);
  Serial.println(" ");
  Serial.print("ISPU O3:\t");
  Serial.print(ISPU_O3, 2);
  Serial.println(" ");

    unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    Serial.print("Sending data to server: ");
    if (WiFi.status() == WL_CONNECTED)
    {
      WiFiClient client; // Use WiFiClient for HTTP
      HTTPClient http;

      // Create JSON object
      JSONVar jsonData;
      jsonData["ESP-ID"] = espId;
      jsonData["PM10"] = PM10;
      jsonData["PM25"] = PM25;
      jsonData["CO"] = MQ135_CO;
      jsonData["HC"] = MQ2_HC;
      jsonData["O3"] = MQ131_O3;
      jsonData["ISPU_PM10"] = ISPU_PM10;
      jsonData["ISPU_PM25"] = ISPU_PM25;
      jsonData["ISPU_CO"] = ISPU_CO;
      jsonData["ISPU_HC"] = ISPU_HC;
      jsonData["ISPU_O3"] = ISPU_O3;

      String jsonString = JSON.stringify(jsonData);
      
      Serial.println("Sending JSON:");
      Serial.println(jsonString);

      // Set up the HTTP POST request
      http.begin(client, serverPath);
      http.addHeader("Content-Type", "application/json");
      int httpResponseCode = http.POST(jsonString);

      if (httpResponseCode == HTTP_CODE_TEMPORARY_REDIRECT || httpResponseCode == HTTP_CODE_PERMANENT_REDIRECT)
      {
        String newLocation = http.getLocation();
        Serial.print("Redirecting to: ");
        Serial.println(newLocation);
        http.end(); // End the previous connection
        http.begin(client, newLocation); // Begin a new connection with the new URL
        httpResponseCode = http.POST(jsonString); // Retry the request
      }

      if (httpResponseCode > 0)
      {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
      }
      else
      {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      http.end();
    }
    else
    {
      Serial.println("WiFi not connected");
    }
  }
}

float calculateConcentrationpm10(long lowpulseInMicroSeconds, long durationinSeconds)
{

  // float ratio = (lowpulseInMicroSeconds / 1000000.0) / 30.0 * 100.0; // Calculate the ratio
  // float concentration = 0.001915 * pow(ratio,2) + 0.09522 * ratio - 0.04884;//Calculate the mg/m3
    float ratio = lowpulseInMicroSeconds/(sampletime_ms*10.0);  // Integer percentage 0=>100
    float concentration = 0.518*pow(ratio,3)-4.25*pow(ratio,2)+570.7*ratio+0.78;
  // Serial.print("lowpulseoccupancy:");
  // Serial.print(lowpulseInMicroSeconds);
  // Serial.print("    ratio:");
  // Serial.print(ratio);
  // Serial.print("    Concentration:");
  // Serial.println(concentration);
  return concentration;
}

float calculateConcentration25(long lowpulseInMicroSeconds, long durationinSeconds)
{

  float ratio = (lowpulseInMicroSeconds / 1000000.0) / 30.0 * 100.0; // Calculate the ratio
  // float concentration = 0.001915 * pow(ratio,2) + 0.09522 * ratio - 0.04884;//Calculate the mg/m3
  float concentration = 0.001915 * pow(ratio,2) + 0.09522 * ratio;//Calculate the mg/m3
  // Serial.print("lowpulseoccupancy:");
  // Serial.print(lowpulseInMicroSeconds);
  // Serial.print("    ratio:");
  // Serial.print(ratio);
  // Serial.print("    Concentration:");
  // Serial.println(concentration);
  return concentration;
}

float calculateConcentration10(long lowpulseInMicroSeconds, long durationinSeconds)
{

  float ratio = (lowpulseInMicroSeconds / 1000000.0) / 30.0 * 100.0; // Calculate the ratio
  float concentration = 0.001915 * pow(ratio,2) + 0.09522 * ratio - 0.04884;//Calculate the mg/m3
  // float concentration = 0.001915 * pow(ratio,2) + 0.09522 * ratio;//Calculate the mg/m3
  // Serial.print("lowpulseoccupancy:");
  // Serial.print(lowpulseInMicroSeconds);
  // Serial.print("    ratio:");
  // Serial.print(ratio);
  // Serial.print("    Concentration:");
  // Serial.println(concentration);
  return concentration;
}

float calculateISPU_PM10(float concentration)
{
  if (concentration <= 50) return concentration * 50 / 50;
  else if (concentration <= 150) return (concentration - 50) * (100 - 50) / (150 - 50) + 50;
  else if (concentration <= 350) return (concentration - 150) * (200 - 100) / (350 - 150) + 100;
  else if (concentration <= 420) return (concentration - 350) * (300 - 200) / (420 - 350) + 200;
  else if (concentration <= 500) return (concentration - 420) * (400 - 300) / (500 - 420) + 300;
  else return 500; // Above 500 is considered as 500
}

float calculateISPU_PM25(float concentration)
{
  if (concentration <= 12) return concentration * 50 / 12;
  else if (concentration <= 35.4) return (concentration - 12) * (100 - 50) / (35.4 - 12) + 50;
  else if (concentration <= 55.4) return (concentration - 35.4) * (150 - 100) / (55.4 - 35.4) + 100;
  else if (concentration <= 150.4) return (concentration - 55.4) * (200 - 150) / (150.4 - 55.4) + 150;
  else if (concentration <= 250.4) return (concentration - 150.4) * (300 - 200) / (250.4 - 150.4) + 200;
  else return 500; // Above 250.4 is considered as 500
}

float calculateISPU_CO(float concentration)
{
  if (concentration <= 50) return concentration * 50 / 50;
  else if (concentration <= 100) return (concentration - 50) * (100 - 50) / (100 - 50) + 50;
  else if (concentration <= 200) return (concentration - 100) * (150 - 100) / (200 - 100) + 100;
  else if (concentration <= 400) return (concentration - 200) * (200 - 150) / (400 - 200) + 150;
  else if (concentration <= 1000) return (concentration - 400) * (300 - 200) / (1000 - 400) + 200;
  else return 500; // Above 1000 is considered as 500
}

float calculateISPU_HC(float concentration)
{
  if (concentration <= 120) return concentration * 50 / 120;
  else if (concentration <= 180) return (concentration - 120) * (100 - 50) / (180 - 120) + 50;
  else if (concentration <= 240) return (concentration - 180) * (150 - 100) / (240 - 180) + 100;
  else if (concentration <= 300) return (concentration - 240) * (200 - 150) / (300 - 240) + 150;
  else if (concentration <= 360) return (concentration - 300) * (300 - 200) / (360 - 300) + 200;
  else return 500; // Above 360 is considered as 500
}
float calculateISPU_O3(float concentration)
{
  if (concentration <= 0.05) return concentration * 50 / 0.05;
  else if (concentration <= 0.1) return (concentration - 0.05) * (100 - 50) / (0.1 - 0.05) + 50;
  else if (concentration <= 0.2) return (concentration - 0.1) * (150 - 100) / (0.2 - 0.1) + 100;
  else if (concentration <= 0.3) return (concentration - 0.2) * (200 - 150) / (0.3 - 0.2) + 150;
  else if (concentration <= 0.4) return (concentration - 0.3) * (300 - 200) / (0.4 - 0.3) + 200;
  else return 500; // Above 0.4 is considered as 500
}
