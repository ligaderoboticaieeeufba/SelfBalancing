
#include <PID_v1.h>
#include <Wire.h>

#define M1F 10 //pra frente com roda 1
#define M1T 9 // pra trás com roda 1
#define M2F 11  //pra frente com roda 2
#define M2T 3   // pra trás com roda 2

//=============================== Giroscopio
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int minVal = 265; int maxVal = 402;
double x; double y; double z;
double GyYant = 0;
//=============================== end Giroscopio
double IGyX =0;
double IGyY = 0;
double IGyZ = 0;

int funciona = 1;

//======================================
//Variaveis que vamos usar.
double Setpoint, Input, Output;
//======================================
//Ganhos e configuração do PID

double Kp = 2;//0,075  0,09  //0,1
double Ki = 0.1; //0
double Kd = 0; //0,0024  0,002 0,0036//0,0036

//kp = 0.1 ki =0.001 kd = 0.002

//velocidade 90 => kp =0.075 ki =0 kd=0.002
//adaptativo => velocidade 200, kpaggr=0.17,kiaggr=0,kdaggr=0.0055 kpsuave =0.1 kisuave = 0, kdsuave=0.004
PID superPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
//======================================

void setup() {

  Serial.begin(9600);
  
  //============================= Giroscopio
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  //============================= end Giroscopio


  //pinMode(2,INPUT); // Botao Calibração
  //pinMode(4,INPUT); //Botão Rodar o Carro

  
  int x = 0;
  
    //Determina SetPoint
    Setpoint = 0;
  //Define range na saída do controlador
  superPID.SetOutputLimits(-250, 250);
  //Determina tempo de execução do PID - 10ms ou 100 execuções por segundo
  superPID.SetSampleTime(100); //em ms
  //Liga o PID
  superPID.SetMode(AUTOMATIC);
  superPID.SetTunings(Kp, Ki, Kd);
}

void loop() {


  //=============== Giroscopio ==========

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);

  //--------------------------------------
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read()<<8|Wire.read(); 
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read(); 
  //---------------------------------------
  double IGyX = IGyX + GyX;
  double IGyY = ((GyY - GyYant)*0.01) + IGyY;
  double IGyZ = IGyZ + GyZ;
  GyYant = GyY;
  
  double xMed = (0.93*IGyX) + (0.07*AcX);
  double yMed = (0.98*IGyY) + (0.02*AcY);
  double zMed = (0.93*IGyZ) + (0.07*AcZ);
  
double  xAng = map(xMed, minVal, maxVal, -90, 90);
double yAng = map(yMed, minVal, maxVal, 0, 180);
double zAng = map(zMed, minVal, maxVal, -90, 90);
 

  //----------------------------------------
  x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
  //----------------------------------------


  //-------------------------------------------
  //Serial.print("AngleX= "); Serial.println(x);
  Serial.print("AngleY= "); 
  Serial.println(yMed);
  //Serial.print("AngleZ= "); Serial.println(z);
  Serial.println("-----------------------------------------");
  //============= end Giroscopio ==============







  Input = yMed;
 Serial.println(Output);
//  delay(10);

if(y>130 ||y<70){
  funciona = 0;
}
else{
  funciona = 1;
}

superPID.Compute();


  if (Output > 0 && funciona == 1)
  {

     int comando= -Output;
    analogWrite(M1T,  comando);
    analogWrite(M2T, comando);

  }
  else if (Output < 0 && funciona ==1)
  {
   
    analogWrite(M1F, Output);
    analogWrite(M2F, Output);

  }
  else
  {
   //analogWrite(M1F, velocidade);
    //analogWrite(M2F, velocidade);
  }






  //  unsigned long currentMillis = millis(); // grab current time

  // check if "interval" time has passed (1000 milliseconds)
  /*
    if ((unsigned long)(currentMillis - previousMillis) >= interval) {
    analogWrite(Motor1[m1],vel_calibracao);
        analogWrite(Motor2[m2],vel_calibracao);
    Serial.print(Output);
    Serial.print("  ");
    Serial.println(Input);

    // save the "current" time
    previousMillis = millis();
    }
  */
}


