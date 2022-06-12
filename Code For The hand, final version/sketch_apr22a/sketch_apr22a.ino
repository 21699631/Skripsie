#include <math.h>

#include <MPU9250_asukiaaa.h>

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif

MPU9250_asukiaaa myIMU;



double Aref[3][1] = {{0},{0},{0}};
double Mref[3][1] = {{0},{0},{0}};
double Areal[3][1] = {{0},{0},{0}};
double Mreal[3][1] = {{0},{0},{0}};

float phi;
float phiref;
float theta;
float thetaref;


float Norm (double X[3][1]){
  float c = sqrt(X[0][0]*X[0][0]+X[1][0]*X[1][0]+X[2][0]*X[2][0]);
  return c;
}

void Cross (double X[3][1], double Y[3][1], double Z[3][1]){
  Z[0][0] = (X[1][0]*Y[2][0])-(X[2][0]*Y[1][0]);
  Z[1][0] = (X[2][0]*Y[0][0])-(X[0][0]*Y[2][0]);
  Z[2][0] = (X[0][0]*Y[1][0])-(X[1][0]*Y[0][0]);
}

void Multiply (double X[3][3], double Y[3][3], double Z[3][3]){
  for(int i = 0; i<3; i++){
    for(int j = 0; j<3; j++){
        Z[i][j] = (X[i][0]*Y[0][j])+(X[i][1]*Y[1][j])+(X[i][2]*Y[2][j]);
    }
  }
}

float TRIAD (double X0[3][1], double X1[3][1], double Y0[3][1], double Y1[3][1], int Mode){ //Aref, Areal, Mref, Mreal
  double Ir[3][1] = {{X0[0][0]/Norm(X0)},{X0[1][0]/Norm(X0)},{X0[2][0]/Norm(X0)}};
  double Ib[3][1] = {{X1[0][0]/Norm(X1)},{X1[1][0]/Norm(X1)},{X1[2][0]/Norm(X1)}};

  double CrossJr[3][1];
  Cross(Ir, Y0, CrossJr);
  float NormJr = Norm(CrossJr);
  double Jr[3][1] = {{CrossJr[0][0]/NormJr},{CrossJr[1][0]/NormJr},{CrossJr[2][0]/NormJr}};
  
  double CrossJb[3][1];
  Cross(Ib, Y1, CrossJb);
  float NormJb = Norm(CrossJb);
  double Jb[3][1] = {{CrossJb[0][0]/NormJb},{CrossJb[1][0]/NormJb},{CrossJb[2][0]/NormJb}};

  double Kr[3][1];
  Cross(Ir, Jr, Kr);
  double Kb[3][1];
  Cross(Ib, Jb, Kb);

  double R[3][3] = {{Ir[0][0], Ir[1][0], Ir[2][0]}, {Jr[0][0], Jr[1][0], Jr[2][0]}, {Kr[0][0], Kr[1][0], Kr[2][0]}};
  double B[3][3] = {{Ib[0][0], Jb[0][0], Kb[0][0]}, {Ib[1][0], Jb[1][0], Kb[1][0]}, {Ib[2][0], Jb[2][0], Kb[2][0]}};

  double Cbn[3][3];
  Multiply(B, R, Cbn);

  float phi = atan(Cbn[1][2]/Cbn[2][2]);
  float theta = -asin(Cbn[0][2]);
  
  if(Mode == 0){
    return phi;
  }
  if(Mode == 1){
    return theta;
  }

}

void setup () {
  Serial.begin(115200);
Wire.begin(SDA_PIN, SCL_PIN); //sda, scl


  myIMU.setWire(&Wire);

  myIMU.beginAccel();
  myIMU.beginMag();

  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(5000);
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

/*
  digitalWrite(LED_BUILTIN, HIGH);
  delay(5000);
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
  phiref = (phiref*180)/PI;
  delay(2000);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(5000);
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
  thetaref = -(thetaref*180)/PI;
*/

}

void loop() {
  
  digitalWrite(LED_BUILTIN, HIGH);
  delay(5000);
  myIMU.accelUpdate();
  myIMU.magUpdate();
  Areal[0][0] = myIMU.accelX();
  Areal[1][0] = myIMU.accelY();
  Areal[2][0] = myIMU.accelZ();
  Mreal[0][0] = myIMU.magX();
  Mreal[1][0] = myIMU.magY();
  Mreal[2][0] = myIMU.magZ();
  digitalWrite(LED_BUILTIN, LOW);
  phi = TRIAD(Aref, Areal, Mref, Mreal, 0);
  theta = TRIAD(Aref, Areal, Mref, Mreal, 1);

  phi = (phi*180)/PI;
  theta = (theta*180)/PI;

  Serial.println(phi);
  Serial.println(theta);
  Serial.println();
  delay(2000);
  Aref[0][0] = Areal[0][0];
  Aref[1][0] = Areal[1][0];
  Aref[2][0] = Areal[2][0];
  Mref[0][0] = Mreal[0][0];
  Mref[1][0] = Mreal[1][0];
  Mref[2][0] = Mreal[2][0];
}
