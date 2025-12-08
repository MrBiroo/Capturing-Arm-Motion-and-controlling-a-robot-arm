#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <esp_now.h>
#include <WiFi.h>

#define CHANNEL 1 //the same channel should be on both esps

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno; // Declare an array of Adafruit_BNO055 objects

// MAC Address of responder - edit as required
uint8_t broadcastAddress[] = {0x3C, 0x61, 0x05, 0x15, 0x78, 0x6C}; //For the Dev kit 0x10, 0x06, 0x1C, 0xF6, 0x83, 0x78
String data;
esp_now_peer_info_t peerInfo;// Peer info


void setup() {
  Serial.begin(115200);
  Serial.println("Setup started");
  int_BNO();
  ESPnow();

}

esp_err_t result;


void loop() {
   /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);

    float yaw =  (float)event.orientation.x;
    if (yaw < 0) {
       yaw = -yaw;
    } else if (yaw > 180) {
      yaw -= 360;
      }

    float pitch = (float)event.orientation.y;
    

    float roll = (float)event.orientation.z;

   Serial.println(Packet("#",roll,"A"));
   result = esp_now_send(broadcastAddress , (uint8_t*) &data, sizeof(data));
    delay(5);
   Serial.println(Packet("#",pitch,"B"));
   result = esp_now_send(broadcastAddress , (uint8_t*) &data, sizeof(data));
    delay(5);   
   Serial.println(Packet("#",yaw,"C"));
   result = esp_now_send(broadcastAddress , (uint8_t*) &data, sizeof(data));


   delay(BNO055_SAMPLERATE_DELAY_MS);
}

void int_BNO()
{
    while (!Serial) delay(10);  // wait for serial port to open!

   bno = Adafruit_BNO055(55, 0x29, &Wire);
    Serial.print("Initializing BNO055 ");
     if (!bno.begin())/* Initialise the sensor */
    {
     /* There was a problem detecting the BNO055 ... check your connections */
     Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
     while (1);
     }
     bno.setExtCrystalUse(true);
}
String Packet(String startBy, float VAL, String dest) {
  String packet = "";
  packet += String(startBy);
  packet += String(dest);
  packet += String(VAL, 2);
  data=packet;
  return data;
}
void ESPnow()
{
     WiFi.mode(WIFI_STA);
  // Initialize ESP-NOW
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESP-NOW initialized successfully");
  } else {
    Serial.println("Error initializing ESP-NOW");
  }
  // Register the send callback
  esp_now_register_send_cb(OnDataSent);
  //Register peer 
  memcpy(peerInfo.peer_addr , broadcastAddress,6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  //Add peer 
  if(esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Fail to add peer!");
  } else 
  {
    Serial.println("peer added successfully") ;
  }
} 
void OnDataSent(const uint8_t *mac_addr , esp_now_send_status_t status){
  Serial.print("\r\nLast Packet Send Status: \t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");

}
