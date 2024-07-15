#include <Adafruit_ADS1X15.h>
#include <MQUnifiedsensor.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
#include <Arduino_JSON.h>

#define PM1PIN 12
#define PM25PIN 14
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
#define Voltage_Resolution 5
#define pin A0
#define ADC_Bit_Resolution 10
#define RatioMQ2CleanAir 9.83
#define RatioMQ135CleanAir 3.6
float factorEscala = 0.16875F;

MQUnifiedsensor MQ2(placa, mq2);
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, mq135);

Adafruit_ADS1115 ads;

const char* ssid = "iPhone Maulana";
const char* password = "hanyatuhanyangtau";
const String serverPath = "https://428b-182-1-67-121.ngrok-free.app/api/esp/data";
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

  MQ2.init();
  MQ135.init();

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

  MQ135.setA(605.18);
  MQ135.setB(-3.937);
  float MQ135_CO = MQ135.readSensor();

  // Assuming MQ2 can be used for HC
  MQ2.setA(36974);
  MQ2.setB(-3.109);
  float MQ2_HC = MQ2.readSensor();

  durationPM1 = pulseIn(PM1PIN, LOW);
  durationPM25 = pulseIn(PM25PIN, LOW);

  lowpulseoccupancyPM1 += durationPM1;
  lowpulseoccupancyPM25 += durationPM25;

  endtime = millis();
  if ((endtime - starttime) > sampletime_ms)
  {
    conPM1 = calculateConcentration10(lowpulseoccupancyPM1, 30);
    conPM25 = calculateConcentration25(lowpulseoccupancyPM25, 30);
    if (conPM1 < 0) conPM1 = 0;
    if (conPM25 < 0) conPM25 = 0;
    PM10 = conPM1 * 1000;
    PM25 = conPM25 * 1000;
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

  Serial.print("Sensor\t\t\t");
  Serial.print("Value (ppm)");
  Serial.println();

  Serial.print("MQ-135 CO:\t\t");
  Serial.print(MQ135_CO, 2);
  Serial.println(" ppm");

  Serial.print("MQ-2 HC:\t\t");
  Serial.print(MQ2_HC, 2);
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

  unsigned long currentMillis = millis();

     if (currentMillis - previousMillis >= interval)
    {
        previousMillis = currentMillis;
        Serial.print("Sending data to server: ");
        if (WiFi.status() == WL_CONNECTED)
        {
            WiFiClientSecure client;
            client.setInsecure();
            HTTPClient http;

            String url = serverPath + "?ESP-ID=" + espId + "&PM10=" + PM10 + "&PM25=" + PM25 + "&CO=" + MQ135_CO + "&HC=" + MQ2_HC +
                         "&ISPU_PM10=" + ISPU_PM10 + "&ISPU_PM25=" + ISPU_PM25 + "&ISPU_CO=" + ISPU_CO + "&ISPU_HC=" + ISPU_HC;
            http.begin(client, url);
            int httpResponseCode = http.GET();

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

float calculateConcentration10(unsigned long duration, int time)
{
  float ratio = duration / (sampletime_ms * 10.0);
  return 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62;
}

float calculateConcentration25(unsigned long duration, int time)
{
  float ratio = duration / (sampletime_ms * 10.0);
  return 0.52 * pow(ratio, 3) - 0.85 * pow(ratio, 2) + 850 * ratio + 0.01;
}

float calculateISPU_PM10(float concentration)
{
  if (concentration <= 50) return concentration * 50 / 50;
  else if (concentration <= 150) return (concentration - 50) * (100 - 50) / (150 - 50) + 50;
  else if (concentration <= 350) return (concentration - 150) * (200 - 100) / (350 - 150) + 100;
  else if (concentration <= 420) return (concentration - 350) * (300 - 200) / (420 - 350) + 200;
  else if (concentration <= 500) return (concentration - 420) * (400 - 300) / (500 - 420) + 300;
  else return (concentration - 500) * (500 - 400) / (600 - 500) + 400;
}

float calculateISPU_PM25(float concentration)
{
  if (concentration <= 15.5) return concentration * 50 / 15.5;
  else if (concentration <= 55.5) return (concentration - 15.5) * (100 - 50) / (55.5 - 15.5) + 50;
  else if (concentration <= 150.5) return (concentration - 55.5) * (200 - 100) / (150.5 - 55.5) + 100;
  else if (concentration <= 250.5) return (concentration - 150.5) * (300 - 200) / (250.5 - 150.5) + 200;
  else if (concentration <= 350.5) return (concentration - 250.5) * (400 - 300) / (350.5 - 250.5) + 300;
  else return (concentration - 350.5) * (500 - 400) / (500.5 - 350.5) + 400;
}

float calculateISPU_CO(float concentration)
{
  if (concentration <= 5) return concentration * 50 / 5;
  else if (concentration <= 10) return (concentration - 5) * (100 - 50) / (10 - 5) + 50;
  else if (concentration <= 17) return (concentration - 10) * (200 - 100) / (17 - 10) + 100;
  else if (concentration <= 34) return (concentration - 17) * (300 - 200) / (34 - 17) + 200;
  else if (concentration <= 46) return (concentration - 34) * (400 - 300) / (46 - 34) + 300;
  else return (concentration - 46) * (500 - 400) / (57.5 - 46) + 400;
}

float calculateISPU_HC(float concentration)
{
  // Placeholder values; replace with correct calculation if needed
  return calculateISPU_CO(concentration);
}
