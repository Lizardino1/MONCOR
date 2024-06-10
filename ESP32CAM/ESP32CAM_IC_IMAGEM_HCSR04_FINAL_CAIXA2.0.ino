#include <WiFi.h>
#include <Wire.h>
#include <HTTPClient.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <Adafruit_BMP085.h>
#include <Adafruit_BMP280.h>
#define SOUND_SPEED 0.034
#define SDA 14
#define SCL 15
#define TOLERANCIA 0.15

const char* ssid = "MONCOR";
const char* password = "corregos";
const char* serverUrl = "http://lavoisier.eletrica.ufpr.br/emoncms/input/post?node=Sensores&json={Distancia_B:";
const char* apiKey = "}&apikey=8d3cdd9e751fb13cc20d970b26fa96f7";
const int trigPin = 12;
const int echoPin = 13;
long duration;
float distanceCm, distanceCm1, distanceCm2;
double velossom, temp;
int i, b, n;
float tempdist[3], tempdistMax[3],tempdistMin[3];
float PrevDist[2], MedDist;





String serverName = "lavoisier.eletrica.ufpr.br";   // REPLACE WITH YOUR Raspberry Pi IP ADDRESS
//String serverName = "example.com";   // OR REPLACE WITH YOUR DOMAIN NAME

String serverPath = "/taruma3/imgup.php";     // The default serverPath should be upload.php

const int serverPort = 80;



WiFiClient client;
HTTPClient http;
bool status;
Adafruit_BMP085 bmp;
Adafruit_BMP280 bmp1;



// CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

const int timerInterval = 10000;    // time (ms) between each HTTP POST image
unsigned long previousMillis = 0; // last time image was sent
unsigned long previousMillis2 = 0;





/////////////////////////////////////////////////////////////////////////////

                        /*SETUP*/


/////////////////////////////////////////////////////////////////////////////

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  Serial.begin(115200);

  

  pinMode(trigPin,OUTPUT);
  pinMode(echoPin, INPUT);

  
  
  Wire.begin(SDA, SCL);

  while(!bmp.begin(0x77)) {
  Serial.println("Não encontrou o sensor BMP180");                       // Para sensor de temperatura
  delay(1000);
  }
  
  while(!bmp1.begin(0x76)) {
  Serial.println("Não encontrou o sensor BMP280");                        //Para sensor de temperatura
  delay(1000);
  }

  

  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }
  
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }

  sendPhoto(); 
  b = 0;
}
  


float distancia(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(20);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  
  temp = bmp.readTemperature();
  

  Serial.print("duração de pulso: ");
  Serial.println(duration);
  
  velossom = 330.40 + (0.59*temp);
  velossom = velossom / 10000;
  Serial.print("velocidade do som: ");
  Serial.println(velossom);
  distanceCm = duration * (velossom/2);
  return distanceCm;
}


float distanciaFinal(){
  float tol = TOLERANCIA;

  for(i=0;i<3;i++){
    tempdist[i] = distancia();
    tempdistMax[i] = tempdist[i] + tol * tempdist[i];
    tempdistMin[i] = tempdist[i] - tol * tempdist[i];
    delay(100);
  }

  if(tempdist[2] > tempdistMax[0] || tempdist[2] > tempdistMax[1] || tempdist[2] < tempdistMin[0] || tempdist[2] < tempdistMin[1]){
    distanciaFinal();
    delay(100);
  }
    else
    return tempdist[2];

  }

  
float distanciaFinal1(float PrevDist[], int n){
  
    float dist = distancia();
    MedDist = 0;
    
    for(i=0;i<3;i++){
      if(PrevDist[i] > 0){
        MedDist = MedDist + PrevDist[i];
      }
      else{
        return dist;
      }
    }
    
    MedDist = MedDist/3;

    if(dist <= ((MedDist * TOLERANCIA) + MedDist) && dist >= ((MedDist * TOLERANCIA) - MedDist)){
      n = 0;
      return MedDist;
      }
    else if(n<3){
      n++;
      distanciaFinal1;
    }
    else{
      n = 0;
      return dist;
    }
     
    
  
}


/////////////////////////////////////////////////////////////////////////////

                                  /*LOOP*/


/////////////////////////////////////////////////////////////////////////////


void loop() {

  unsigned long currentMillis = millis();
  
  //n = 0;
  
  distanceCm = distancia();
  distanceCm1 = distanciaFinal();
  distanceCm2 = distanciaFinal1(PrevDist, n);
  PrevDist[b] = distanceCm2;


  b++;
  if(b == 3){
  b = 0;
  }
 
  
  String url = serverUrl;
  url += String(distanceCm);
  url += ",Distancia_B1:";
  url += String(distanceCm1);  
  url += ",Distancia_B2:";
  url += String(distanceCm2);
  url += ",Temperatura_BMP180:";
  url += String(bmp.readTemperature());
  url += ",Pressao_BMP180:";
  url += String(bmp.readPressure());
  url += ",Temperatura_BMP280:";
  url += String((bmp1.readTemperature() - 2));
  url += ",Pressao_BMP280:";
  url += String(bmp1.readPressure());
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
  sendPhoto();
  previousMillis = currentMillis;
 
 
 http.end();
 delay(30000);
}

String sendPhoto() {
  String getAll;
  String getBody;

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if(!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
  }
  
  Serial.println("Connecting to server: " + serverName);

  if (client.connect(serverName.c_str(), serverPort)) {
    Serial.println("Connection successful!");    
    String head = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--RandomNerdTutorials--\r\n";

    uint32_t imageLen = fb->len;
    uint32_t extraLen = head.length() + tail.length();
    uint32_t totalLen = imageLen + extraLen;
  
    client.println("POST " + serverPath + " HTTP/1.1");
    client.println("Host: " + serverName);
    client.println("Content-Length: " + String(totalLen));
    client.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
    client.println();
    client.print(head);
  
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n=0; n<fbLen; n=n+1024) {
      if (n+1024 < fbLen) {
        client.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen%1024>0) {
        size_t remainder = fbLen%1024;
        client.write(fbBuf, remainder);
      }
    }   
    client.print(tail);
    
    esp_camera_fb_return(fb);
    
    int timoutTimer = 10000;
    long startTimer = millis();
    boolean state = false;
    
    while ((startTimer + timoutTimer) > millis()) {
      Serial.print(".");
      delay(100);      
      while (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (getAll.length()==0) { state=true; }
          getAll = "";
        }
        else if (c != '\r') { getAll += String(c); }
        if (state==true) { getBody += String(c); }
        startTimer = millis();
      }
      if (getBody.length()>0) { break; }
    }
    Serial.println();
    client.stop();
    Serial.println(getBody);
  }
  else {
    getBody = "Connection to " + serverName +  " failed.";
    Serial.println(getBody);
  }
  return getBody;
}
