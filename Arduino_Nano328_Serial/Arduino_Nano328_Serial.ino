/****************************************************************************************	
**  This is example LINX firmware for use with the Arduino Uno with the serial 
**  interface enabled.
**
**  For more information see:           www.labviewmakerhub.com/linx
**  For support visit the forums at:    www.labviewmakerhub.com/forums/linx
**  
**  Written By Sam Kristoff
**
**  BSD2 License.
****************************************************************************************/

//Include All Peripheral Libraries Used By LINX
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Servo.h>

//Include Device Specific Header From Sketch>>Import Library (In This Case LinxChipkitMax32.h)
//Also Include Desired LINX Listener From Sketch>>Import Library (In This Case LinxSerialListener.h)
#include <LinxArduinoNano328.h>
#include <LinxSerialListener.h>
 
//Create A Pointer To The LINX Device Object We Instantiate In Setup()
LinxArduinoNano328* LinxDevice;

int myCustomCommand();


#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define EARTH_GRAVITY_MS2 9.80665  // m/s2
#define DEG_TO_RAD        0.017453292519943295769236907684886
#define RAD_TO_DEG        57.295779513082320876798154814105

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gg;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorInt16 ggWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

unsigned char yawB;
unsigned char yawC;
unsigned char rollB;
unsigned char rollC;
unsigned char pitchB;
unsigned char pitchC;



//Initialize LINX Device And Listener
void setup()
{
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    mpu.initialize();
//    delay(100);
    devStatus = mpu.dmpInitialize();

//    // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-4);
  mpu.setYGyroOffset(24);
  mpu.setZGyroOffset(-38);
  mpu.setXAccelOffset(203.5);
  mpu.setYAccelOffset(-2112);
  mpu.setZAccelOffset(4996); // 1688 factory default for my test chip
//
//  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {

      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
//      mpu.PrintActiveOffsets();
      mpu.setDMPEnabled(true);

      mpuIntStatus = mpu.getIntStatus();

      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
  }
  //Instantiate The LINX Device
  LinxDevice = new LinxArduinoNano328();
  
  //The LINXT Listener Is Pre Instantiated, Call Start And Pass A Pointer To The LINX Device And The UART Channel To Listen On
  LinxSerialConnection.Start(LinxDevice, 0);  
  LinxSerialConnection.AttachCustomCommand(0, myCustomCommand);

}

void loop()
{
  //Listen For New Packets From LabVIEW
  LinxSerialConnection.CheckForCommands();
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

      // display quaternion values in easy matrix form: w x y z
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      
      float yawA = ypr[0] * RAD_TO_DEG;
      yawA += 180;
      yawA *= 256.0/360.0;
      yawB = yawA;
      yawC = (int)(yawA * (1 << 8));

      float pitchA = ypr[1] * RAD_TO_DEG;
      pitchA += 180;
      pitchA *= 256.0/360.0;
      pitchB = pitchA;
      pitchC = (int)(pitchA * (1 << 8));

      float rollA = ypr[2] * RAD_TO_DEG;
      rollA += 180;
      rollA *= 256.0/360.0;
      rollB = rollA;
      rollC = (int)(rollA * (1 << 8));
  }
  
  else {
//    devStatus = mpu.dmpInitialize();
    mpu.setDMPEnabled(true);
    }
}

int myCustomCommand(unsigned char numInputBytes, unsigned char* input, unsigned char* numResponseBytes, unsigned char* response)
{  
  *numResponseBytes = 7;

  

      response[0] = 0;
      response[1] = yawB;
      response[2] =  yawC;
      response[3] = pitchB;
      response[4] = pitchC;
      response[5] = rollB;
      response[6] = rollC;
//      response[0] = 0;
//      response[1] = 2;
//      response[2] =  3;
//      response[3] = 4;
//      response[4] = 5;
//      response[5] = 99;
//      response[6] = 100;
 

  return 0;

}
