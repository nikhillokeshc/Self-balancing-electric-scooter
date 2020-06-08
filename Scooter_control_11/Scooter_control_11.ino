//Bluetooth control start stop
//Papadopoulos model (Stable)
//Serial communication with second ESP32

#include<stdint.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define In3 32//41
#define In4 33//40
#define enf 25//6
#define encoderPin1 34//2
#define encoderPin2 35//3
#define channel0 0
#define channel1 1
#define pwmFrequency 5000
#define resolution 12           //12 bits pwm resolution
#define stp 19
//#define pi 3.1415926

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


//Mpu control and status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz, ypr_ref = 0;

//Encoder Variables
volatile char control;
const char b = '2';
volatile int lastEncoded = 0;
volatile long encoderValue = 0;
long lastencoderValue = 0;
int lastMSB = 0;
int lastLSB = 0;

//State space variables
float p = 0;
float nz = 0;
//static float z = 0;
unsigned long now, lastTime;
int Time, dt = 0.01;

/*
// v=3
const float 
A[4][4] = {{0, 1.0, 0, 0}, {18.1422, -0.0725, -20.6512, -2.9117}, {0, 0, 0, 1.0}, {76.6397, 2.5627, -83.5784, -25.4702}},
B[4] = {0, -0.0641, 0, 2.2661},
K[4][3] = {{2.9828, 1.2534, 2.4221},{12.534, 12.276, -183.98},{0.19023, -1.4450, 44.756},{5.4193, -49.413, 1134.7}}, //V=1e3;W=diag(0.01,0.1,pi/4000)
//G[4] = {-58.0205, -16.5046, 29.9594, 2.6784}; //Q=diag(100,0,10,0) R=1e-1
G[4] = {-140.52, -29.676, 43.022, 4.1264}; //Q=diag(100,0,10,0) R=1e-2


// v=2
const float 
A[4][4] = {{0, 1.0, 0, 0}, {18.1422, -0.0483, -6.7716, -1.94}, {0, 0, 0, 1.0}, {76.6397, 1.7085, -26.0262, -17.0198}},
B[4] = {0, -0.0641, 0, 2.2661},
K[4][3] = {{2.97, 1.2828, 2.1982},{12.828, 9.8846, -151.76},{0.1727, -1.1919, 53.097},{6.3089, -49.535, 1500.3}}, //V=1e3;W=diag(0.01,0.1,pi/4000)
G[4] = {-180.7071, -40.6922, 19.1597, 4.1666}; //Q=diag(100,0,10,0) R=1e-2
*/

// v=1.6
const float 
A[4][4] = {{0, 1.0, 0, 0}, {18.1422, -0.0387, -2.7743, -1.5513}, {0, 0, 0, 1.0}, {76.6397, 1.3668, -9.4511, -13.6397}},
B[4] = {0, -0.0641, 0, 2.2661},
K[4][3] = {{2.9681, 1.2913, 2.0853},{12.913, 9.0778, -135.56},{0.16378, -1.0647, 56.596},{6.6535, -48.421, 1673.9}}, //V=1e3;W=diag(0.01,0.1,pi/4000)
G[4] = {-237.7014, -54.9079, 1.4912, 4.1391}; //Q=diag(100,0,10,0) R=1e-2


float X[4] = {0, 0, 0, 0},
      nX[4] = {0, 0, 0, 0},
           err[3] = {0, 0, 0};
float temp = 0;
static float u=0;

volatile float steerAngle, tiltAngle, tiltAngleRate;
volatile float steerRef, tiltRef;
const float pi = 3.1457;

//Bluetooth Variables and callbacks
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;
std::string rxt;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      rxt = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        //Serial.println("*********");
        //Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);
        //Serial.println();
        //Serial.println("*********");
      }
    }
};

void setup() {
  
  //BLE Setup
  // Create the BLE Device
  BLEDevice::init("Electric Scooter");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );

  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE
                                          );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  //Serial.println("Waiting a client connection to notify...");

  //dmp setup
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  Serial2.begin(115200);
  
  //Serial.println("Initializing I2C device");
  mpu.initialize();

  //Serial.println("Testing device connections...");
  //Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection Failed");

  //Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(133);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(-15);
  mpu.setZAccelOffset(1688);

  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    //Serial.println(F(")"));
  }

  //pins setup
  pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  pinMode(stp, OUTPUT);
  ledcSetup(channel1, pwmFrequency, resolution);  //(enf, OUTPUT);
  ledcAttachPin(enf, channel1);

  digitalWrite(encoderPin1, HIGH);
  digitalWrite(encoderPin2, HIGH);
  digitalWrite(stp, LOW);

  //interrupts setup
  attachInterrupt(digitalPinToInterrupt(encoderPin1), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2), updateEncoder, CHANGE);
}

void loop() {

    if (deviceConnected) {
      
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
    }
    char a = rxt[2];
    while (a == b) {
      //Serial.println("Process halted by user!");
      a = rxt[2];
      digitalWrite(stp, HIGH);
      digitalWrite(In3, LOW);
      digitalWrite(In4, LOW);
      if (a != b)
      {
        //Reset
        for(int i=0;i<4;i++) {X[i] = 0;}
        u=0;
        encoderValue = 0;
        break;
      }
      else
        delay(1);
        //Serial.println("process halted by user!");
    }

    //Restart.
    digitalWrite(stp, LOW);
    
    // Wait till 10ms has elapsed. This is for synchronization.
    while (10 - Time > 0) {
      now = millis();
      Time = now - lastTime;
      //Serial.println("5");
    }
    lastTime = now;
    Time = 0;

    // from MPU6050
    getYawPitchRoll();
    mpu.getRotation(&gx, &gy, &gz);

    //Store the measurements in a variable.
    steerAngle = (float)encoderValue * pi / 4000.0;
    tiltAngle = 1*pi/180-(ypr[1]); // * 180 / M_PI;
    tiltAngleRate = gy / 16.4 * pi / 180;

  // Full state feedback with Full order Kalman observer continuous time implementation.
  double err[] = {tiltAngle - X[0], tiltAngleRate-X[1], steerAngle - X[2]};       // E = Y-CX
  for (int i = 0; i < 4; i++) {                               // tempXcap = AX + BU
    for (int j = 0; j < 4; j++) {
      nX[i] += A[i][j] * X[j];
    }
    nX[i] += B[i] * u;
  }
  for (int i = 0; i < 4; i++) {                              // tempXcap = tempXcap + KE
    for (int j = 0; j < 3; j++) {
      nX[i] += K[i][j] * err[j];
    }
  }
  for (int i = 0; i < 4; i++) {                              // Xcap = Xcap + tempXcap*dt
    X[i] += nX[i] * 0.01;
    nX[i] = 0;                                               // tempXcap = 0
  }

  if(abs(X[0]>30*pi/180)) X[0] = sgn(X[0])*30*pi/180;
  if(abs(X[2]>45*pi/180)) X[2] = sgn(X[2])*45*pi/180;
      if (rxt[2] == '5'){}
      else if (rxt[2] == '7'){X[0] -= 0.0043;} //0.125*pi/180;}
      else if (rxt[2] == '8'){X[0] += 0.0043;} //0.125*pi/180;}

    u = 0;
    for (int i = 0; i < 4; i++) {                              // u = -GX
      u -= G[i] * X[i];
      //u = 0.5*c + 0.5*u;
    }

    // Control to actuator (motor driver in this case.)
    if (abs(u) > 12) u = 12 * sgn(u);
      if (tiltAngle < pi / 6 && tiltAngle > -pi / 6) {
      if (u >= 0 && u <= 12) {
        digitalWrite(In3, HIGH);
        digitalWrite(In4, LOW);
        double p = mapf(u, 0, 12.0, 0, 2048);
        ledcWrite(channel1, p);                    //analogWrite(enf, p);
      }
      else if (u < 0 && u >= -12) {
        digitalWrite(In3, LOW);
        digitalWrite(In4, HIGH);
        double p = mapf(u, -12.0, 0, 2048, 0);
        ledcWrite(channel1, p);                    //analogWrite(enf, p);
        //  Serial.println("4");
      }
    }
    else {
      digitalWrite(In3, LOW);
      digitalWrite(In4, LOW);
      //   Serial.println("5");
    }
    //Control(u);
    
    // Monitor state variables and status variables.
    Serial.print(X[0] * 180 / pi);
    Serial.print(" ");
    Serial.print(X[1] * 180 / pi);
    Serial.print(" ");
    Serial.print(X[2] * 180 / pi);
    Serial.print(" ");
    Serial.print(X[3] * 180 / pi);
    Serial.print(" ");
    Serial.println(u);
    
    float dat[5] = {X[0] * (float)180/pi, X[1] * (float)180/pi, X[2] * (float)180/pi, X[3] * (float)180/pi, u};
    byte* b = (byte* ) dat;
    Serial2.write(b, 20);
}

void Control(float vol){
  }

// encoderPins interrupt Sub-Routine.
void updateEncoder() {
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

  lastEncoded = encoded; //store this value for next time
}

// map function to map between double precision limits
static inline float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Signum function
static inline int8_t sgn(float val) {
  if (val < 0) return -1;
  if (val == 0) return 0;
  return 1;
}

//I2C communication with MPU-6050 to acquire YawPitchRoll into ypr array.
void getYawPitchRoll() {
  if (!dmpReady) return;

  if (fifoCount < packetSize) {
    fifoCount = mpu.getFIFOCount();
  }
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
}
