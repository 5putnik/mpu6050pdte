#include<Wire.h> // Biblioteca da comunicacao I2C

const int MPU_addr=0x68;  // I2C address of the MPU-6050
const int MPU1=0x68;  // Endereco I2C do MPU-6050 numero 1
const int MPU2=0x69;  // Endereco I2C do MPU-6050 numero 2

int16_t AcX1, AcY1, AcZ1, Tmp1, GyX1, GyY1, GyZ1; // Leituras do MPU-6050 1
int16_t AcX2, AcY2, AcZ2, Tmp2, GyX2, GyY2, GyZ2; // Leituras do MPU-6050 2
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; // Variaveis finais, que serao enviadas ao programa
short p;
unsigned long dt;
#define BOT A0

void setup()
{
  pinMode(BOT,INPUT);
  
  Wire.begin();
  
  Wire.beginTransmission(MPU1);
  Wire.write(0x6B);  // registrador PWR_MGMT_1
  Wire.write(0);     // zera registrador, acordando o MPU-6050
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU2);
  Wire.write(0x6B);  // registrador PWR_MGMT_1
  Wire.write(0);     // zera registrador, acordando o MPU-6050
  Wire.endTransmission(true);
  
  //Serial.begin(9600); // Inicia o processo serial
  Serial.begin(115200); // Inicia o processo serial (alta velocidade)
}
void loop()
{
  dt = millis();
  Wire.beginTransmission(MPU1);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU1,14,true);  // request a total of 14 registers
  AcX1 = Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY1 = Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ1 = Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp1 = Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX1 = Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY1 = Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ1 = Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU2);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU2,14,true);  // request a total of 14 registers
  AcX2 = Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY2 = Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ2 = Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp2 = Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX2 = Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY2 = Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ2 = Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  Wire.endTransmission(true);
  
  AcX = AcX1 - AcX2;
  AcY = AcY1 - AcY2;
  AcZ = AcZ1 - AcZ2;
  
  // Tmp = Tmp1>Tmp2?Tmp1:Tmp2
  Tmp = Tmp1;
  if(Tmp < Tmp2)
    Tmp = Tmp2;
  
  GyX = GyX1 - GyX2;
  GyY = GyY1 - GyY2;
  GyZ = GyZ1 - GyZ2;
  
  p = digitalRead(BOT);
  Serial.print("#");
  Serial.print(AcX);
  Serial.print(":");
  Serial.print(AcY);
  Serial.print(":");
  Serial.print(AcZ);
  Serial.print(":");
  Serial.print(Tmp/340.00+36.53);
  Serial.print(":");
  Serial.print(GyX);
  Serial.print(":");
  Serial.print(GyY);
  Serial.print(":");
  Serial.print(GyZ);
  Serial.print(":");
  Serial.println(p);
  /*Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);*/
  while(millis()-dt < 10)
    ;
}
