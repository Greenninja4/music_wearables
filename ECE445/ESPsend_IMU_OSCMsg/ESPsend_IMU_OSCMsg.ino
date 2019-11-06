/*---------------------------------------------------------------------------------------------
  Open Sound Control (OSC) library for the ESP8266/ESP32
  Example for sending messages from the ESP8266/ESP32 to a remote computer
  The example is sending "hello, osc." to the address "/test".
  This example code is in the public domain.
  --------------------------------------------------------------------------------------------- */
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif
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
const IPAddress outIp(192, 168, 1, 126);     // remote IP of your computer
const unsigned int outPort = 57120;          // remote port to receive OSC
const unsigned int localPort = 8888;        // local port to listen for OSC packets (actually not used for sending)

// the number of the LED pin
const int ledPin = 16;  // 16 corresponds to GPIO16

// setting PWM properties
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

void setup() {
  Serial.begin(115200);

  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Starting UDP");
  Udp.begin(localPort);
  Serial.print("Local port: ");
  
#ifdef ESP32
  Serial.println(localPort);
#else
  Serial.println(Udp.localPort());
#endif

  // serial to display data
  Serial.begin(115200);
  while (!Serial) {}

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
  ledcSetup(ledChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin, ledChannel);

}

void loop() {
  // read the sensor
  IMU.readSensor();
  
  //   display the data
  //  Serial.print(IMU.getAccelX_mss(), 6);
  //  Serial.print("\t");
  //  Serial.print(IMU.getAccelY_mss(), 6);
  //  Serial.print("\t");
  //  Serial.print(IMU.getAccelZ_mss(), 6);
  //  Serial.print("\t");
  //  Serial.print(IMU.getGyroX_rads(), 6);
  //  Serial.print("\t");
  //  Serial.print(IMU.getGyroY_rads(), 6);
  //  Serial.print("\t");
  //  Serial.print(IMU.getGyroZ_rads(), 6);
  //  Serial.print("\t");
  //  Serial.print(IMU.getMagX_uT(), 6);
  //  Serial.print("\t");
  //  Serial.print(IMU.getMagY_uT(), 6);
  //  Serial.print("\t");
  //  Serial.print(IMU.getMagZ_uT(), 6);
  //  Serial.print("\t");
  //  Serial.println(IMU.getTemperature_C(), 6);
  
  MadgwickAHRSupdate(IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads(), IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss(), IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT());
  
  //  Serial.println(q0, 6);
  //  Serial.print("\t");
  //  Serial.println(q1, 6);
  //  Serial.print("\t");
  //  Serial.println(q2, 6);
  //  Serial.print("\t");
  //  Serial.println(q3, 6);
  //  delay(100);

  float yaw;
  float pitch;
  float roll;

  yaw   = atan2(2 * q1 * q2 - 2 * q0 * q3, 2 * q0 * q0 + 2 * q1 * q1 - 1) * 180 / PI;
  pitch = -1 * asin(2 * q1 * q3 + 2 * q0 * q2) * 180 / PI;
  roll  = atan2(2 * q2 * q3 - 2 * q0 * q1, 2 * q0 * q0 + 2 * q3 * q3 - 1) * 180 / PI;
  
  if (counter % 100 < 1) {
    Serial.println(yaw, 6);
    Serial.println(pitch, 6);
    Serial.println(roll, 6);
    Serial.println("_______");
  }
  
  counter++;
  
  OSCMessage msg("/testAddr");
  msg.add(roll).add(pitch).add(yaw);
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  //check the status of packet send
  if(Udp.endPacket() != 1){
    Serial.print("packet lost!");
    }
  
  msg.empty();

  // increase the LED brightness
  for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
    // changing the LED brightness with PWM
    ledcWrite(ledChannel, dutyCycle);
    delay(15);
  }

  // decrease the LED brightness
  for (int dutyCycle = 255; dutyCycle >= 0; dutyCycle--) {
    // changing the LED brightness with PWM
    ledcWrite(ledChannel, dutyCycle);
    delay(15);
  }

  //periodically check the wifi connection
  if (counter % 100 < 1) {
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
  }
  
  // adjust the delay for udp sending rate
  delay(1);
}
