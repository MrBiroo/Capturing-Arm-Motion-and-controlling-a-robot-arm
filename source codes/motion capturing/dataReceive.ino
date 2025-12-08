#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Wire.h>

#define bluepin 17
#define MG99pin 16
#define wrist1pin 4
#define wrist2pin 0
#define NANO_ADD 0x69

String data;
Servo blue;  //for the 45kg servo
Servo MG99; //for mg99 servo
Servo wrist1;
Servo wrist2;

void setup() {

  Serial.begin(115200);
  Wire.begin();

  blue.attach(bluepin);
  MG99.attach(MG99pin);
  wrist1.attach(wrist1pin);
  wrist2.attach(wrist2pin);

  ESPnow();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void OnDataRecv(const esp_now_recv_info* recvInfo, const uint8_t *incomingData, int len){
  memcpy(&data, incomingData, sizeof(data));
 // Serial.print("Data received: ");
 // Serial.println(len);
 Serial.println(data);
 Serial.println();
 handle(data);
}

void handle(String data)
{
  if (data.startsWith("#A"))
  {
    String numberPart = data.substring(2); // Skip '#' and 'A'
    int value = numberPart.toInt()*1.6;
    blue.write(value);
    Serial.println("Massge A reseved");
    
  }else if ((data.startsWith("#B")))
  {
    String numberPart = data.substring(2);
    int value = numberPart.toInt();
    MG99.write(value);
  }else if ((data.startsWith("#W"))) //not going to be used
  {
    Wire.beginTransmission(NANO_ADD);
    Wire.print(data);
    uint8_t error = Wire.endTransmission(true);
  } 
  else if (data.startsWith("#a"))
  {
    int x,y;
    String firstStage = data.substring(2); // Skip '#' and 'A'
    if (firstStage.startsWith("x"))
    {
     String secoundStage = data.substring(1);
     x = secoundStage.toInt(); 
    } else if (firstStage.startsWith("y"))
    {
     String secoundStage = data.substring(1);
     y = secoundStage.toInt();      
    }
    wrist(x,y);  
  }

}

void wrist(int x,int y)

{
int qq =y+x;
int qqq=y-x;
wrist1.write(qq+90);
wrist2.write(90-qqq);
}

void ESPnow()
{
    WiFi.mode(WIFI_STA);

  if(esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}
