/***GloveADCBoard***/
#include <math.h>

#include <MPU9250_asukiaaa.h>

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif

#define RXD2 16
#define TXD2 17

MPU9250_asukiaaa myIMU;


hw_timer_t *Timer10 = NULL;     //10 millisecond tick
hw_timer_t *Timer2s = NULL;     //10 millisecond tick
int ThumbFlex = 12;
int IndexFlex = 33;
int MiddleFlex = 27;
int RingFlex = 26;
int PinkyFlex = 25;
int ModeSwitch = 23;

int ReadVal[5] = {0, 0, 0, 0, 0};
int FlexMax[5] = {0, 0, 0, 0, 0};
int FlexMin[5] = {0, 0, 0, 0, 0};
uint8_t PercentageFlex[7] = {0, 0, 0, 0, 0, 0, 0};
int Value = 0;
int Transmit = 0;
int ActiveMode = 0;

double Aref[3][1] = {{0}, {0}, {0}};
double Mref[3][1] = {{0}, {0}, {0}};
double Areal[3][1] = {{0}, {0}, {0}};
double Mreal[3][1] = {{0}, {0}, {0}};

float phi;
float phiref;
float theta;
float thetaref;

////////////////////////////////////////////////////////

float Norm (double X[3][1]) {
  float c = sqrt(X[0][0] * X[0][0] + X[1][0] * X[1][0] + X[2][0] * X[2][0]);
  return c;
}

void Cross (double X[3][1], double Y[3][1], double Z[3][1]) {
  Z[0][0] = (X[1][0] * Y[2][0]) - (X[2][0] * Y[1][0]);
  Z[1][0] = (X[2][0] * Y[0][0]) - (X[0][0] * Y[2][0]);
  Z[2][0] = (X[0][0] * Y[1][0]) - (X[1][0] * Y[0][0]);
}

void Multiply (double X[3][3], double Y[3][3], double Z[3][3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      Z[i][j] = (X[i][0] * Y[0][j]) + (X[i][1] * Y[1][j]) + (X[i][2] * Y[2][j]);
    }
  }
}

float TRIAD (double X0[3][1], double X1[3][1], double Y0[3][1], double Y1[3][1], int Mode) { //Aref, Areal, Mref, Mreal
  double Ir[3][1] = {{X0[0][0] / Norm(X0)}, {X0[1][0] / Norm(X0)}, {X0[2][0] / Norm(X0)}};
  double Ib[3][1] = {{X1[0][0] / Norm(X1)}, {X1[1][0] / Norm(X1)}, {X1[2][0] / Norm(X1)}};

  double CrossJr[3][1];
  Cross(Ir, Y0, CrossJr);
  float NormJr = Norm(CrossJr);
  double Jr[3][1] = {{CrossJr[0][0] / NormJr}, {CrossJr[1][0] / NormJr}, {CrossJr[2][0] / NormJr}};

  double CrossJb[3][1];
  Cross(Ib, Y1, CrossJb);
  float NormJb = Norm(CrossJb);
  double Jb[3][1] = {{CrossJb[0][0] / NormJb}, {CrossJb[1][0] / NormJb}, {CrossJb[2][0] / NormJb}};

  double Kr[3][1];
  Cross(Ir, Jr, Kr);
  double Kb[3][1];
  Cross(Ib, Jb, Kb);

  double R[3][3] = {{Ir[0][0], Ir[1][0], Ir[2][0]}, {Jr[0][0], Jr[1][0], Jr[2][0]}, {Kr[0][0], Kr[1][0], Kr[2][0]}};
  double B[3][3] = {{Ib[0][0], Jb[0][0], Kb[0][0]}, {Ib[1][0], Jb[1][0], Kb[1][0]}, {Ib[2][0], Jb[2][0], Kb[2][0]}};

  double Cbn[3][3];
  Multiply(B, R, Cbn);

  float phi = atan(Cbn[1][2] / Cbn[2][2]);
  float theta = -asin(Cbn[0][2]);

  if (Mode == 0) {
    return phi;
  }
  if (Mode == 1) {
    return theta;
  }

}



void GetMax() {
  Value = 3;
  Serial2.write(Value);
  delay(1500);
  FlexMax[0] = analogRead(PinkyFlex);
  FlexMax[1] = analogRead(RingFlex);
  FlexMax[2] = analogRead(MiddleFlex);
  FlexMax[3] = analogRead(IndexFlex);
  FlexMax[4] = analogRead(ThumbFlex);
  Value = 4;
  Serial2.write(Value);

}


void GetMin() {
  Value = 1;
  Serial2.write(Value);
  delay(1500);
  FlexMin[0] =  analogRead(PinkyFlex) + 2048;
  FlexMin[1] =  analogRead(RingFlex) + 2048;
  FlexMin[2] =  analogRead(MiddleFlex) + 2048;
  FlexMin[3] =  analogRead(IndexFlex) + 2048;
  FlexMin[4] =  analogRead(ThumbFlex) + 2048;
  Value = 2;
  Serial2.write(Value);
}

void IMUcalibrate() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1500);
  myIMU.accelUpdate();
  myIMU.magUpdate();
  Aref[0][0] = myIMU.accelX();
  Aref[1][0] = myIMU.accelY();
  Aref[2][0] = myIMU.accelZ();
  Mref[0][0] = myIMU.magX();
  Mref[1][0] = myIMU.magY();
  Mref[2][0] = myIMU.magZ();
  digitalWrite(LED_BUILTIN, LOW);
  delay(2000);


  digitalWrite(LED_BUILTIN, HIGH);
  delay(1500);
  myIMU.accelUpdate();
  myIMU.magUpdate();
  Areal[0][0] = myIMU.accelX();
  Areal[1][0] = myIMU.accelY();
  Areal[2][0] = myIMU.accelZ();
  Mreal[0][0] = myIMU.magX();
  Mreal[1][0] = myIMU.magY();
  Mreal[2][0] = myIMU.magZ();
  digitalWrite(LED_BUILTIN, LOW);
  phiref = TRIAD(Aref, Areal, Mref, Mreal, 0);
  phiref = abs((phiref * 180) / PI);
  delay(2000);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(1500);
  myIMU.accelUpdate();
  myIMU.magUpdate();
  Areal[0][0] = myIMU.accelX();
  Areal[1][0] = myIMU.accelY();
  Areal[2][0] = myIMU.accelZ();
  Mreal[0][0] = myIMU.magX();
  Mreal[1][0] = myIMU.magY();
  Mreal[2][0] = myIMU.magZ();
  digitalWrite(LED_BUILTIN, LOW);
  thetaref = TRIAD(Aref, Areal, Mref, Mreal, 1);
  thetaref = abs((thetaref * 180) / PI);
}

void IRAM_ATTR OnTimer10() {
  ActiveMode = digitalRead(ModeSwitch);

  if (ActiveMode == 0) {
    //Read in value of flex sensor for each digit.
    ReadVal[0] =  analogRead(PinkyFlex);
    ReadVal[1] =  analogRead(RingFlex);
    ReadVal[2] =  analogRead(MiddleFlex);
    ReadVal[3] =  analogRead(IndexFlex);
    ReadVal[4] =  analogRead(ThumbFlex);

    for (int i = 0; i < 5; i++) {
      //normalise flex sensors readings into range
      if (ReadVal[i] < FlexMin[i]) {
        ReadVal[i] = FlexMin[i];
      }
      if (ReadVal[i] > FlexMax[i]) {
        ReadVal[i] = FlexMax[i];
      }
      //find percentage of total flex range that each digit is bent
      PercentageFlex[i] = (((ReadVal[i] - FlexMin[i]) * 100) / (FlexMax[i] - FlexMin[i]));

    }

  }
  Transmit = 1;
}

void IRAM_ATTR OnTimer2s() {
  if(ActiveMode==1){
    Transmit = 2;
  }
}

void serialTrans() {
  //Serial2 Transmit
  Serial2.write(PercentageFlex, sizeof(PercentageFlex) );
  Transmit = 0;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  analogReadResolution(12);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Wire.begin(SDA_PIN, SCL_PIN); //sda, scl


  myIMU.setWire(&Wire);

  myIMU.beginAccel();
  myIMU.beginMag();
  myIMU.magXOffset = -10;
  myIMU.magYOffset = -20;
  myIMU.magZOffset = 15;

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(ModeSwitch, INPUT);

  delay(2000);
  GetMin();
  delay(2000);
  GetMax();
  delay(2000);
  IMUcalibrate();

  Timer10 = timerBegin(0, 80, true);                   //attach timer10 to 1st timer of 4, 1 tick = 1us
  timerAttachInterrupt(Timer10, &OnTimer10, true);     //attach interrupt function to timer10
  timerAlarmWrite(Timer10, 100000, true);               //Set timer10 alarm to call interrupt every 10ms
  timerAlarmEnable(Timer10);                            //enable timer10

Timer2s = timerBegin(1, 80, true);                   //attach timer10 to 1st timer of 4, 1 tick = 1us
  timerAttachInterrupt(Timer2s, &OnTimer2s, true);     //attach interrupt function to timer10
  timerAlarmWrite(Timer2s,1000000, true);               //Set timer10 alarm to call interrupt every 10ms
  timerAlarmEnable(Timer2s);                            //enable timer10


}

void loop() {
  // put your main code here, to run repeatedly:

  if (Transmit == 1) {
    serialTrans();
  }
  if (ActiveMode == 1&&Transmit==2){
      myIMU.accelUpdate();
      myIMU.magUpdate();
      Areal[0][0] = myIMU.accelX();
      Areal[1][0] = myIMU.accelY();
      Areal[2][0] = myIMU.accelZ();
      Mreal[0][0] = myIMU.magX();
      Mreal[1][0] = myIMU.magY();
      Mreal[2][0] = myIMU.magZ();
      phi = TRIAD(Aref, Areal, Mref, Mreal, 0);
      theta = TRIAD(Aref, Areal, Mref, Mreal, 1);
      if(phi > 0){
      phi = abs((phi * 180) / PI);
      }
      else{
        phi = 0;
      }
      if(theta > 0){
      theta = abs((theta * 180) / PI);
      }
      else{
        theta = 0;
      }
      PercentageFlex[6] = (phi / phiref) * 100;
      PercentageFlex[5] = (theta / thetaref) * 100;
      Transmit = 0;
    }
}
