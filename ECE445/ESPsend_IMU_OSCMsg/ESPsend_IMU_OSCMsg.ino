#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include "OSCMessage.h"
#include "MPU9250.h"
extern "C" {
  #include "MadgwickAHRS.h"
};

// WiFi
char ssid[] = "OSC_2";                    // WiFi network SSID
char pass[] = "SuperCollider";            // WiFi network password
WiFiUDP Udp;                              // UDP instance for sending / receiving packets
const IPAddress outIp(192,168,1,123);     // Remote IP of your computer
const unsigned int outPort = 57120;       // Remote port of your computer where OSC packets are sent
const unsigned int localPort = 8888;      // Local port of ESP32 where OSC packets are received

// OSC
char device[] = "/testAddr";              // Device name sent through OSC
OSCMessage OSCMsg(device);                // OSC message instance for sending / receiving packets

// IMU
MPU9250 IMU(Wire, 0x68);                  // MPU9250 sensor on I2C bus 0 with address 0x68
int status;                               // Status signal
float yaw, pitch, roll;                   // Orientaion values

// RGB LED
const int resolution = 8;                 // PWM resolution
const int freq = 5000;                    // PWM frequency
const int ledPinR = 16;                   // Red LED connects to GPIO 16
const int ledPinG = 17;                   // Green LED connects to GPIO 17
const int ledPinB = 18;                   // Blue LED connects to GPIO 18
const int ledChannelR = 2;                // Red LED uses channel 2
const int ledChannelG = 0;                // Green LED uses channel 0
const int ledChannelB = 1;                // Blue LED uses channel 1

// Miscellaneous
int counter = 0;                          // Counter to limit prints to the serial monitor


// Setup function (run once)
void setup() {
  // Set serial monitor baud rate
  Serial.begin(115200);

  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Subnet mask: ");
  Serial.println(WiFi.subnetMask());
  
  // Initialize UDP protocol
  Serial.println("Starting UDP");
  Udp.begin(localPort);
  Serial.print("Local port: ");
  #ifdef ESP32
    Serial.println(localPort);
  #else
    Serial.println(Udp.localPort());
  #endif

  // Initialize IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }

  // RBG LED PWM Setup
  ledcSetup(ledChannelR, freq, resolution);
  ledcSetup(ledChannelG, freq, resolution);
  ledcSetup(ledChannelB, freq, resolution);
  ledcAttachPin(ledPinR, ledChannelR);
  ledcAttachPin(ledPinG, ledChannelG);
  ledcAttachPin(ledPinB, ledChannelB);
}

// Loop function (run continuously)
void loop() {
  // Increment counter
  counter++;

  // Read IMU sensor data
  IMU.readSensor();
  
  // Use Madgwick sensor fusion algorithm to get orientation quaternion (saved to q0,q1,q2,q3)
  MadgwickAHRSupdate(IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads(), IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss(), IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT());
  
  // Calculate yaw, pitch, and roll from quaternion
  yaw = abs(atan2(2*q1*q2-2*q0*q3,2*q0*q0+2*q1*q1-1)*180/PI);
  pitch = abs(-1*asin(2*q1*q3+2*q0*q2)*180/PI);
  roll = abs(atan2(2*q2*q3-2*q0*q1,2*q0*q0+2*q3*q3-1)*180/PI);

  // Every 10 cycles
  if (counter%10 == 0){
    // Send OSC message
    OSCMsg.add(roll).add(pitch).add(yaw);
    Udp.beginPacket(outIp, outPort);
    OSCMsg.send(Udp);
    Udp.endPacket();
    OSCMsg.empty();

    // Update LED colors
    ledcWrite(ledChannelR, (int)((yaw/180.0)*255));
    ledcWrite(ledChannelG, (int)((pitch/90.0)*255));
    ledcWrite(ledChannelB, (int)((roll/180.0)*255));
  }

  // Every 100 cycles
  if(counter%100 == 0){
    // Display orientation to serial monitor
    Serial.println(yaw);
    Serial.println(pitch);
    Serial.println(roll);
    Serial.println("_____________");
  }
}
