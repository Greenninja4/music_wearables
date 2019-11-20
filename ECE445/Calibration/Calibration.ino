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

float q00 = 1;                            // Calibration quaternions
float q10 = 0;
float q20 = 0;
float q30 = 0;
float q0c, q1c, q2c, q3c;

// Mode switching
int mode = 0;
int numModes = 4;

// Push-buttons
const int button0 = 12;
int buttonPrev0 = 0;
int buttonCurr0 = 0;
const int button1 = 13;
int buttonPrev1 = 0;
int buttonCurr1 = 0;
const int button2 = 14;
int buttonPrev2 = 0;
int buttonCurr2 = 0;
const int button3 = 15;
int buttonPrev3 = 0;
int buttonCurr3 = 0;

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

  // Push-button setup
  pinMode(button0, INPUT);
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(button3, INPUT);

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
  // Read IMU sensor data
  IMU.readSensor();
  
  // Use Madgwick sensor fusion algorithm to get orientation quaternion (saved to q0,q1,q2,q3)
  MadgwickAHRSupdate( IMU.getGyroX_rads(), 
                      IMU.getGyroY_rads(), 
                      IMU.getGyroZ_rads(), 
                      IMU.getAccelX_mss(), 
                      IMU.getAccelY_mss(), 
                      IMU.getAccelZ_mss(), 
                      IMU.getMagX_uT(), 
                      IMU.getMagY_uT(), 
                      IMU.getMagZ_uT());

  // // Calculate yaw, pitch, and roll from quaternion
  yaw = abs(atan2(2*q1*q2-2*q0*q3,2*q0*q0+2*q1*q1-1)*180/PI);
  pitch = abs(-1*asin(2*q1*q3+2*q0*q2)*180/PI);
  roll = abs(atan2(2*q2*q3-2*q0*q1,2*q0*q0+2*q3*q3-1)*180/PI);

  // Button ... Calibration
  buttonCurr0 = digitalRead(button0);
  if (buttonCurr0 != buttonPrev0){        // Change in button press
    if (buttonCurr0 == 1){                // Pressed
      buttonPrev0 = 1;
    } else {                              // Released
      buttonPrev0 = 0;
    }
  }

  // Button ... Mode switching
  buttonCurr1 = digitalRead(button1);
  if (buttonCurr1 != buttonPrev1){        // Change in button press
    if (buttonCurr1 == 1){                // Pressed
      mode = (mode+1) % numModes;
      buttonPrev1 = 1;
    } else {                              // Released
      buttonPrev1 = 0;
    }
  }

  // Button ... TBD
  buttonCurr2 = digitalRead(button2);
  if (buttonCurr2 != buttonPrev2){        // Change in button press
    if (buttonCurr2 == 1){                // Pressed
      buttonPrev2 = 1;
    } else {                              // Released
      buttonPrev2 = 0;
    }
  }

  // Button ... TBD
  buttonCurr3 = digitalRead(button3);
  if (buttonCurr3 != buttonPrev3){        // Change in button press
    if (buttonCurr3 == 1){                // Pressed
      buttonPrev3 = 1;
    } else {                              // Released
      buttonPrev3 = 0;
    }
  }

  // Increment counter
  counter++;

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
//    // Display orientation to serial monitor
//    Serial.println("_____________");
//    Serial.println(yaw);
//    Serial.println(pitch);
//    Serial.println(roll);
//    Serial.print("Mode: ");
//    Serial.println(mode);
    
    //Plot the orientation
    Serial.println("_____________");
    Serial.print(yaw);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.print(roll);
    Serial.println("");
  }
}
