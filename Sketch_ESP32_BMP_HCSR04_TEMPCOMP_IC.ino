#include <WiFi.h>
#include<Wire.h>
#include <HTTPClient.h>
#include <Adafruit_BMP085.h>
// pino trigger 5 pino echo 15 ////////////////////////////////////////////////////////////////////////

const char* ssid = "MONCOR";
const char* password = "corregos";
const char* serverUrl = "http://lavoisier.eletrica.ufpr.br:8080/input/post?node=Sensores&json={Distancia_A:";
const char* apiKey = "}&apikey=86e5ab0a004ced32b9e108eba832f7d0";
const int trigPin = 5; //5 para esp32 sem pinout visível (A) 26 para a outra
const int echoPin = 18; //18 para esp32 sem pinout visível (A) 22 para a outra
const int wifiledPin = 2;
long duration;
float distanceCm;
double velossom, temp;
int i;



HTTPClient http;
Adafruit_BMP085 bmp;



void setup() {
  Serial.begin(115200);
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(wifiledPin, OUTPUT); 
  
  while (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
    }
     
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando ao WiFi...");
    for(int i=3;i>=0;i--){
      digitalWrite(wifiledPin,HIGH);
      delay(500);
      digitalWrite(wifiledPin,LOW);
    }
  }
  Serial.println("Conectado ao WiFi!");
  for(int i=3;i>=0;i--){
      digitalWrite(wifiledPin,HIGH);
      delay(100);
      digitalWrite(wifiledPin,LOW);  
     }
     
      digitalWrite(wifiledPin,HIGH);
}
  
void loop() {

  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  temp = (bmp.readTemperature());
  
  velossom = 330.40000+(0.59000*temp);
  velossom = velossom / 10000;
  Serial.println("velossom ");
  Serial.print(velossom);
  distanceCm = duration * (velossom/2);
  
  
  String url = serverUrl;
  url += String(distanceCm);
  url += ",Temperatura_A:";
  url += String(bmp.readTemperature());
  url += ",Pressão_A:";
  url += String(bmp.readPressure());
  url += apiKey;
  Serial.println(url);
  
  if (WiFi.status() == WL_CONNECTED) {
    http.begin(url);
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      Serial.println(payload);
    }
  }  
  else if(WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Queda de conexão!!!");
    for(int i=3;i>=0;i--){
      digitalWrite(wifiledPin,HIGH);
      delay(500);
      digitalWrite(wifiledPin,LOW);
    }
}
  http.end();
  delay(3000);
}
