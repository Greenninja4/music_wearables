#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include "OSCMessage.h"
#include <SparkFunMPU9250-DMP.h>
//#include "MPU9250.h"
extern "C" {
#include "MadgwickAHRS.h"
};

// WiFi
char ssid[] = "OSC_2";                    // WiFi network SSID
char pass[] = "SuperCollider";            // WiFi network password
WiFiUDP Udp;                              // UDP instance for sending / receiving packets
const IPAddress outIp(192, 168, 1, 123);  // Remote IP of your computer
const unsigned int outPort = 57120;       // Remote port of your computer where OSC packets are sent
const unsigned int localPort = 8888;      // Local port of ESP32 where OSC packets are received

// OSC
char device[] = "/testAddr";              // Device name sent through OSC
OSCMessage OSCMsg(device);                // OSC message instance for sending / receiving packets

// IMU
MPU9250_DMP imu; 
//MPU9250 IMU(Wire, 0x68);                  // MPU9250 sensor on I2C bus 0 with address 0x68
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
//  WiFi.begin(ssid, pass);
//  while (WiFi.status() != WL_CONNECTED) {
//    delay(500);
//    Serial.print(".");
//  }
  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Subnet mask: ");
  Serial.println(WiFi.subnetMask());

  // Initialize UDP protocol
  Serial.println("Starting UDP");
//  Udp.begin(localPort);
  Serial.print("Local port: ");
#ifdef ESP32
  Serial.println(localPort);
#else
  Serial.println(Udp.localPort());
#endif

  // Initialize IMU
//  status = IMU.begin();
//  if (status < 0) {
//    Serial.println("IMU initialization unsuccessful");
//    Serial.println("Check IMU wiring or try cycling power");
//    Serial.print("Status: ");
//    Serial.println(status);
//    while (1) {}
//  }

if (imu.begin() != INV_SUCCESS)
{
    while (1)
    {
        // Failed to initialize MPU-9250, loop forever
    }
}
    // Initialize the digital motion processor
    imu.dmpBegin(DMP_FEATURE_SEND_RAW_ACCEL | // Send accelerometer data
                 DMP_FEATURE_GYRO_CAL       | // Calibrate the gyro data
                 DMP_FEATURE_SEND_CAL_GYRO  | // Send calibrated gyro data
                 DMP_FEATURE_6X_LP_QUAT     , // Calculate quat's with accel/gyro
                 10);                         // Set update rate to 10Hz.
imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

imu.setGyroFSR(2000); // Set gyro to 2000 dps
// Accel options are +/- 2, 4, 8, or 16 g
imu.setAccelFSR(2); // Set accel to +/-2g
imu.setLPF(5);
imu.setSampleRate(10);
imu.setCompassSampleRate(10);
  // RBG LED PWM Setup
//  ledcSetup(ledChannelR, freq, resolution);
//  ledcSetup(ledChannelG, freq, resolution);
//  ledcSetup(ledChannelB, freq, resolution);
//  ledcAttachPin(ledPinR, ledChannelR);
//  ledcAttachPin(ledPinG, ledChannelG);
//  ledcAttachPin(ledPinB, ledChannelB);
}

// Loop function (run continuously)
void loop() {
  // Increment counter
  counter++;

  // Read IMU sensor data
//  IMU.readSensor();
imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
float accelX = imu.calcAccel(imu.ax); // accelX is x-axis acceleration in g's
float accelY = imu.calcAccel(imu.ay); // accelY is y-axis acceleration in g's
float accelZ = imu.calcAccel(imu.az); // accelZ is z-axis acceleration in g's

float gyroX = imu.calcGyro(imu.gx); // gyroX is x-axis rotation in dps
float gyroY = imu.calcGyro(imu.gy); // gyroY is y-axis rotation in dps
float gyroZ = imu.calcGyro(imu.gz); // gyroZ is z-axis rotation in dps

float magX = imu.calcMag(imu.mx); // magX is x-axis magnetic field in uT
float magY = imu.calcMag(imu.my); // magY is y-axis magnetic field in uT
float magZ = imu.calcMag(imu.mz); // magZ is z-axis magnetic field in uT

if ( imu.fifoAvailable() > 0 ) // Check for new data in the FIFO
{
    // Use dmpUpdateFifo to update the ax, gx, qx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS )
    {
        // The following variables will have data from the top of the FIFO:
        // imu.ax, imu.ay, imu.az, -- Accelerometer
        // imu.gx, imu.gy, imu.gz -- calibrated gyroscope
        // and imu.qw, imu.qx, imu.qy, and imu.qz -- quaternions
    }
}


  // Use Madgwick sensor fusion algorithm to get orientation quaternion (saved to q0,q1,q2,q3)
//  MadgwickAHRSupdate(IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads(), IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss(), IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT());

  // Calculate yaw, pitch, and roll from quaternion
//  yaw = (atan2(2 * imu.qx * imu.qy + 2 * imu.qw * imu.qz, imu.qw*imu.qw - imu.qz*imu.qz - imu.qy * imu.qy + imu.qx*imu.qx ) * 180 / PI);
//  pitch = (asin(2 * imu.qy * imu.qw - 2 * imu.qx * imu.qz) * 180 / PI);
//  roll = (atan2(2 * imu.qy * imu.qz + 2 * imu.qx * imu.qw, imu.qw*imu.qw + imu.qz*imu.qz - imu.qy * imu.qy - imu.qx*imu.qx) * 180 / PI);

          float q0 = imu.calcQuat(imu.qw);
          float q1 = imu.calcQuat(imu.qx);
          float q2 = imu.calcQuat(imu.qy);
          float q3 = imu.calcQuat(imu.qz);

//          float roll = imu.roll;
//          float pitch = imu.pitch;
//          float yaw = imu.yaw;
  // Every 10 cycles
//  if (counter % 10 == 0) {
//    // Send OSC message
//    OSCMsg.add(roll).add(pitch).add(yaw);
//    Udp.beginPacket(outIp, outPort);
//    OSCMsg.send(Udp);
//    Udp.endPacket();
//    OSCMsg.empty();
//
//    // Update LED colors
//    ledcWrite(ledChannelR, (int)((yaw / 180.0) * 255));
//    ledcWrite(ledChannelG, (int)((pitch / 90.0) * 255));
//    ledcWrite(ledChannelB, (int)((roll / 180.0) * 255));
//  }


    float sinr_cosp = 2 * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
    roll = atan2(sinr_cosp, cosr_cosp)* 180 / PI;

    // pitch (y-axis rotation)
    float sinp = 2 * (q0 * q2 - q3 * q1);
    if (sinp >= 1)
        pitch = 90; // use 90 degrees if out of range
    else if (sinp <= -1)
        pitch = -90;
    else
        pitch = asin(sinp)* 180 / PI;

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
    yaw = atan2(siny_cosp, cosy_cosp)* 180 / PI;


  // Every 100 cycles
  if (counter % 1 == 0) {
    // Display orientation to serial monitor or serial plotter
    Serial.print(yaw);
    Serial.print(", ");
    Serial.print(pitch);
    Serial.print(", ");
    Serial.print(roll);
    Serial.println();
  }
}
