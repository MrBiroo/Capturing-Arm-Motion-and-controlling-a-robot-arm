#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TCA9548.h>

#define NUM_IMUS 3 // Number of BNO055 modules
#define MUX_ADDRESS 0x70 // Address of the PCA9548A multiplexer
#define CHANNEL 1 // The same channel should be on both ESPs
#define BNO055_ADDRESS 0x29 // I2C address of the BNO055 module
#define BNO055_ID 0x55 // ID of the BNO055 module

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

TCA9548 mux = TCA9548(MUX_ADDRESS); // PCA9548A multiplexer object

Adafruit_BNO055 bno0 = Adafruit_BNO055(55, 0x29, &Wire);
Adafruit_BNO055 bno1 = Adafruit_BNO055(55, 0x29, &Wire);
Adafruit_BNO055 bno2 = Adafruit_BNO055(55, 0x29, &Wire);
Adafruit_BNO055* bno[NUM_IMUS] = { &bno0, &bno1, &bno2 };

String startByte = "#";
String destROLL[3] = {"A", "D", "G"}; // Destination bytes for roll
String destPITCH[3] = {"B", "E", "H"}; // Destination bytes for pitch
String destYAW[3] = {"C", "F", "I"}; // Destination bytes for yaw

void setup() {
  Serial.begin(115200);
  Serial.println("Setup started");
  while (!Serial) delay(10); // Wait for serial port to open

  mux.begin(); // Initialize the PCA9548A multiplexer

  for (int i = 0; i < NUM_IMUS; i++) {
    mux.selectChannel(i); // Select the I2C channel for the current BNO055 module
    Serial.print("Initializing BNO055 ");
    Serial.println(i);

    if (!bno[i]->begin()) { // Use the '->' operator for pointer access
      Serial.print("Ooops, no BNO055 ");
      Serial.print(i);
      Serial.print(" detected ... Check your wiring or I2C ADDR!");

      // Add a delay and retry mechanism to recover from faulty modules
      delay(5000);
      i--; // Retry the current module
    } else {
      /* Use external crystal for better accuracy */
      bno[i]->setExtCrystalUse(true); // Use '->' to call setExtCrystalUse
    }
  }
  delay(1000);
}

void loop() {
  for (int i = 0; i < NUM_IMUS; i++) {
    mux.selectChannel(i); // Select the I2C channel for the current BNO055 module
    /* Get a new sensor event */
    sensors_event_t event;
    if (bno[i]->getEvent(&event)) { // Use '->' to call getEvent
      float yaw = 360 - (float)event.orientation.x;
      Serial.println(Packet(startByte, yaw, destYAW[i]));

      float pitch = (float)event.orientation.y;
      Serial.println(Packet(startByte, pitch, destPITCH[i]));

      float roll = (float)event.orientation.z;
      Serial.println(Packet(startByte, roll, destROLL[i]));
    } else {
      Serial.print("Error getting sensor event from BNO055 ");
      Serial.println(i);
    }
  }
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
