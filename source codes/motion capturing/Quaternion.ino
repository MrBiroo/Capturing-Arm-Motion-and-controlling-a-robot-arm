#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

#define CHANNEL 1 //the same channel should be on both esps

String startByte = "#";
String destROLL[3] = {"A","D", "G"}; // Destination bytes for roll
String destPITCH[3] = {"B","E","H"}; // Destination bytes for pitch
String destYAW[3] = {"C","F","I"}; // Destination bytes for yaw

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno; // Declare an array of Adafruit_BNO055 objects

// MAC Address of responder - edit as required
uint8_t broadcastAddress[] = {0x9C, 0x9C, 0x1F, 0xC7, 0x69, 0xEC}; //For the Dev kit 0x10, 0x06, 0x1C, 0xF6, 0x83, 0x78
String data;
esp_now_peer_info_t peerInfo;// Peer info



void setup() {
  Serial.begin(115200); // Set serial baud rate  
  Serial.println("Setup started");

  ESPnow();

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

esp_err_t result;
float R[3][3];
void loop() {
  imu::Quaternion quat = bno.getQuat(); // Get quaternion

  float w = quat.w();
  float x = quat.x();
  float y = quat.y();
  float z = quat.z();



  quat_to_matrix(w,x,y,z,R);
  float t2=atan(-R[0][1]/R[1][2]) * 180.0/PI;
  float t4=atan(-R[1][0]/R[1][2]) * 180.0/PI;
  float t3=atan(-R[0][1]/R[1][1]*sin(t2)) * 180.0/PI;
  Serial.println(Packet("#",t2,"A"));
  Serial.println(Packet("#",t3,"B"));
  Serial.println(Packet("#",t4,"C"));
  /*Serial.print(t2);
  Serial.print("   ");
  Serial.print(t3);
  Serial.print("   ");
  Serial.println(t4);*/
 

  delay(100);
}

String Packet(String startBy, float VAL, String dest) {
  String packet = "";
  packet += String(startBy);
  packet += String(dest);
  packet += String(VAL, 2);
  data=packet;
  return data;
}

void OnDataSent(const uint8_t *mac_addr , esp_now_send_status_t status){
  Serial.print("\r\nLast Packet Send Status: \t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
void quat_to_matrix (float w, float x, float y, float z, float R[3][3])
{
  // Normalize the quaternion to ensure valid rotation matrix
  float norm = sqrt(w * w + x * x + y * y + z * z);
  w /= norm;
  x /= norm;
  y /= norm;
  z /= norm;

  R[0][0] = 1 - 2 * y * y - 2 * z * z;
  R[0][1] = 2 * x * y - 2 * z * w;
  R[0][2] = 2 * x * z + 2 * y * w;

  R[1][0] = 2 * x * y + 2 * z * w;
  R[1][1] = 1 - 2 * x * x - 2 * z * z;
  R[1][2] = 2 * y * z - 2 * x * w;

  R[2][0] = 2 * x * z - 2 * y * w;
  R[2][1] = 2 * y * z + 2 * x * w;
  R[2][2] = 1 - 2 * x * x - 2 * y * y;
}
/*void print_Matrix(float R)
{
  Serial.print(R[0][0]);  Serial.print("    "); Serial.print(R[0][1]);  Serial.print("    ");  Serial.println(R[0][2]);
  Serial.print(R[1][0]);  Serial.print("    "); Serial.print(R[1][1]);  Serial.print("    ");  Serial.println(R[1][2]);
  Serial.print(R[0][0]);  Serial.print("    "); Serial.print(R[0][1]);  Serial.print("    ");  Serial.println(R[0][2]);  
}*/
void print_Quaternion(float w, float x, float y, float z)
{
  // Print quaternion
  Serial.print("Quaternion -> ");
  Serial.print("w: "); Serial.print(w, 4);
  Serial.print(" x: "); Serial.print(x, 4);
  Serial.print(" y: "); Serial.print(y, 4);
  Serial.print(" z: "); Serial.println(z, 4);  
}
