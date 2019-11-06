#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include "OSCMessage.h"
#include "MPU9250.h"


extern "C" {

#include "MadgwickAHRS.h"
};

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;
int counter = 0;
char ssid[] = "OSC_2";          // your network SSID (name)
char pass[] = "SuperCollider";                    // your network password

WiFiUDP Udp;                                // A UDP instance to let us send and receive packets over UDP
//const IPAddress outIp(172,20,10,3);     // remote IP of your computer
const IPAddress outIp(192,168,1,123);     // remote IP of your computer
//const IPAddress outIp(10,192,202,233);     // remote IP of your computer
const unsigned int outPort = 57120;          // remote port to receive OSC
//const unsigned int outPort = 5005;          // remote port to receive OSC
const unsigned int localPort = 8888;        // local port to listen for OSC packets (actually not used for sending)

float q0Prev = 1.0f, q1Prev = 0.0f, q2Prev = 0.0f, q3Prev = 0.0f;
OSCMessage msg("/testAddr");

// the number of the LED pin
const int ledPinR = 16;  // 16 corresponds to GPIO16
const int ledPinG = 17;  // 16 corresponds to GPIO16
const int ledPinB = 18;  // 16 corresponds to GPIO16

// setting PWM properties
const int freq = 5000;
const int ledChannelR = 2;
const int ledChannelG = 0;
const int ledChannelB = 1;
const int resolution = 8;

void setup() {
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

  
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Subnet mask: ");
  Serial.println(WiFi.subnetMask());
  

  Serial.println("Starting UDP");
  Udp.begin(localPort);
  Serial.print("Local port: ");
#ifdef ESP32
  Serial.println(localPort);
#else
  Serial.println(Udp.localPort());
#endif

  // start communication with IMU
  status = IMU.begin();
  //  IMU.setSampleRate()
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }

  // configure LED PWM functionalitites
  ledcSetup(ledChannelR, freq, resolution);
  ledcSetup(ledChannelG, freq, resolution);
  ledcSetup(ledChannelB, freq, resolution);
//  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPinR, ledChannelR);
  ledcAttachPin(ledPinG, ledChannelG);
  ledcAttachPin(ledPinB, ledChannelB);

}

void loop() {
  // read the sensor
  IMU.readSensor();

//  MahonyAHRSupdate(IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads(), IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss(), IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT());
  MadgwickAHRSupdate(IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads(), IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss(), IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT());

  float yaw;
  float pitch;
  float roll;
  
  yaw   = abs(atan2(2*q1*q2-2*q0*q3,2*q0*q0+2*q1*q1-1)*180/PI);
  pitch = abs(-1*asin(2*q1*q3+2*q0*q2)*180/PI);
  roll  = abs(atan2(2*q2*q3-2*q0*q1,2*q0*q0+2*q3*q3-1)*180/PI);
  if(counter%100 < 1){
    q0Prev = q0;
    q1Prev = q1;
    q2Prev = q2;
    q3Prev = q3;
    
    Serial.println(yaw);
    Serial.println(pitch);
    Serial.println(roll);
    Serial.println("_____________");
    
}
counter++;

  if (counter%10 < 1){
  msg.add(roll).add(pitch).add(yaw);
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
  }

  ledcWrite(ledChannelR, (int)((yaw/180.0)*255));
  ledcWrite(ledChannelG, (int)((pitch/90.0)*255));
  ledcWrite(ledChannelB, (int)((roll/180.0)*255));
  
}
