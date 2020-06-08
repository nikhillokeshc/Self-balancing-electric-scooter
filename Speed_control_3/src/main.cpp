//Esp32 speed measurement, control, serial data recieve from main esp32 and transmit to puTTy client.
//Data transmit over wifi to puTTy terminal

#include <Arduino.h>
#include <WiFi.h>

#define en 21
#define In1 22
#define In2 18
#define velocityPin 5
#define stp 23
#define channel0 0
#define pwmFrequency 5000
#define resolution 12 //12 bits pwm resolution

double mapf(double x, double in_min, double in_max, double out_min, double out_max);
// Signum function
static inline int8_t sgn(double val);
void updateSpeed();

double speedRef = 3;
double currTime, prevTime, elapsTime, error, cumErr, rateErr, lastErr;
        double Kp = 100,Ki = 40,Kd = 20;

volatile double Velocity = 0;
volatile int speedPulse = 0; 
volatile double u = 0;
volatile int last;

const char *ssid = "NJIoT";
const char *password = "mythings";

WiFiServer wifiserver(80);

void setup()
{

  Serial.begin(115200);
  Serial2.begin(115200);
  delay(500);

  //WiFi server setup
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());
  wifiserver.begin();
  //wifiServer.begin();

  //pinMode(en, OUTPUT);
  ledcSetup(channel0, pwmFrequency, resolution);
  ledcAttachPin(en, channel0);

  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);

  pinMode(velocityPin, INPUT_PULLUP);
  digitalWrite(velocityPin, HIGH);

  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);

  attachInterrupt(digitalPinToInterrupt(velocityPin), updateSpeed, RISING);
  last = millis();
}

void loop()
{
  WiFiClient client = wifiserver.available();

  if (client)
  {
    while (client.connected())
    {

      while (millis() - last > 50)
      {
        last = millis();

        //Compute velocity
        Velocity = (double)speedPulse * 20 * 0.0122; //0.14*pi/(9*4)/0.05
        speedPulse = 0;
        
        currTime = millis();
        elapsTime = currTime - prevTime;
        error = speedRef - Velocity;
        cumErr += error * elapsTime;
        rateErr = (error - lastErr)/elapsTime;
        //u = Kp * error + Ki * cumErr + Kd * rateErr;
        u = 21;
        lastErr = error;
        prevTime = currTime;

        if(abs(u) > 24){u = 24 * sgn(u);}
        if (u >= 0 && u <= 24) {
        digitalWrite(In1, LOW);
        digitalWrite(In2, HIGH);
        float p = mapf(u, 0, 24.0, 0, 2048);
        ledcWrite(channel0, p);                    //analogWrite(enf, p);
        }
      else if (u < 0 && u >= -24) {
        digitalWrite(In1, LOW);
        digitalWrite(In2, HIGH);
        //float p = mapf(-u/3, 0, 24.0, 0, 2048);
        //ledcWrite(channel0, p);
        //float p = mapf(u, -24.0, 0, 2048, 0);
        //ledcWrite(channel0, p);                    //analogWrite(enf, p);
        //  Serial.println("4");
        }

        union {
          float dat[7] = {0, 0, 0, 0, 0, 0, 0};
          byte b[28];
        };

        if (Serial2.available() > 0)
        {
          for (int i = 0; i < 20; i++)
          {
            b[i] = Serial2.read();
          }
        }
        dat[5] = Velocity;
        dat[6] = millis()/1000.0;

        for (int i = 0; i < 7; i++)
        {
          Serial.print(dat[i]);
          Serial.print(" ");
          client.print(dat[i]);
          client.print("\t");
        }
        Serial.print(" ");
        Serial.print(u);
        Serial.println();
        client.println();
        //client.stop();
        //Serial.println("Client disconnected");
      }

      while (digitalRead(stp))
      {
        ledcWrite(channel0, 0); //analogWrite(en, 0);
        Serial.println("Process halted by user!");
        cumErr = 0;
        rateErr = 0;
        prevTime = millis();
      }
    }
    digitalWrite(In1, LOW);
    digitalWrite(In2, LOW);
    client.stop();
    Serial.println("Client disconnected");
  }
}

//Utility functions

// map function to map between double precision limits
double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Signum function
static inline int8_t sgn(double val)
{
  if (val < 0)
    return -1;
  if (val == 0)
    return 0;
  return 1;
}

void updateSpeed()
{
  speedPulse += 1;
}