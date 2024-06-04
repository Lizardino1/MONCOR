#include<ESP8266HTTPClient.h>
#include<ESP8266WiFi.h>
#include<WiFiClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
// pino trigger 5 pino echo 6 ////////////////////////////////////////////////////////////////////////

const char* ssid = "MONCOR";
const char* password = "corregos";
const char* serverUrl = "http://lavoisier.eletrica.ufpr.br:8080/input/post?node=Sensores&json={Distancia_C:";
const char* apiKey = "}&apikey=86e5ab0a004ced32b9e108eba832f7d0";
const int trigPin = 14; //D5
const int echoPin = 12; //D6
const int wifiledPin = LED_BUILTIN;
long duration;
float distanceCm;
double temp, velossom;
int i;

WiFiClient client;
HTTPClient http;
Adafruit_BMP280 bmp;

void setup() {
  Serial.begin(115200);
  
  delay(10);
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(wifiledPin, OUTPUT);
    
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  Serial.println(F("BME280 test"));
  bool status;
  status = bmp.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando ao WiFi...");
     for(i=3;i>=0;i--){
      digitalWrite(wifiledPin,HIGH);
      delay(500);
      digitalWrite(wifiledPin,LOW);
    }
    }
  

  Serial.println("Conectado ao WiFi!");
  for(i=3;i>=0;i--){
      digitalWrite(wifiledPin,HIGH);
      delay(100);
      digitalWrite(wifiledPin,LOW);  
     }
     Serial.println(bmp.readTemperature());
}
  
void loop() {

  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  temp = (bmp.readTemperature());
  Serial.println("temp");
  Serial.println(bmp.readTemperature());
  
  velossom = 330.40000+(0.59000*temp);
  velossom = velossom / 10000;
  Serial.println("velossom ");
  Serial.print(velossom);
  distanceCm = duration * (velossom/2);
  
  
  String url = serverUrl;
  url += String(distanceCm);
  url += ",Temperatura_C:";
  url += String(bmp.readTemperature());
  url += ",Pressão_C:";
  url += String(bmp.readPressure());
  url += apiKey;
  Serial.println(url);

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(client,url);
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      Serial.println(payload);
    }
  }
    http.end();

  delay(3000);

}
