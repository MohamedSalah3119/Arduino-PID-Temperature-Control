#include <PID_v1.h>
#include "DHT.h"

#define DHTPIN 2     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 11

DHT dht(DHTPIN, DHTTYPE);

double Temperature;
double Setpoint;
double OutPut;
double Kp = 0, Ki = 10, Kd = 0;

#define Fan 10
#define Lamp 9
double Tem;
PID myPID(&Temperature, &OutPut, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  pinMode(Lamp,OUTPUT );
  pinMode(Fan, OUTPUT);

  Serial.begin(115200);

  Setpoint = 25.0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(Kp, Ki, Kd);
}
void loop() {
  dht.begin();
  // Wait a few seconds between measurements.
  delay(300);
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  // Read temperature as Celsius (the default)
   Tem = map(Temperature, 20,30,0,255);
Temperature = dht.readTemperature();
  Serial.println(Temperature);
  // Check if any reads failed and exit early (to try again).
  if (isnan(Tem) ) {
    //  Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
 //  Serial.print(F("Temperature: "));
 //  Serial.println(Temperature);

  myPID.Compute();
  Serial.print(Tem);
  Serial.print(" ");
  Serial.println(OutPut);
 // Serial.print(" ");
 // Serial.println(SetPoint);
  if (Temperature < Setpoint) {
    analogWrite(Lamp, OutPut);
    analogWrite(Fan, 255);
  //  Serial.println("first else");
  }
  else if (Temperature > Setpoint) {
    analogWrite(Fan, OutPut);
    analogWrite(Lamp, 255);
  //  Serial.println("Second");
  }
  else {
    analogWrite(Fan, 255);
    analogWrite(Lamp, 255);
  //  Serial.println("Third");
  }
}
