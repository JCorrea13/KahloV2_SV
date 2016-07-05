/**
* Este es el programa principal de softare de vuelo de kahlo v2
**/

#include<Wire.h>

#define MPU_addr 0x68  // I2C address of the MPU-6050
#define BMP_addr 0x77  // I2C address of the BMP180

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int8_t presion_barometrica, tmp_b;
int contador = 0;

void setup(){

  //Inicializacion de MPU6050 - Giroscopio Acelerometro
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  //Fin de inicilizacion MPU6050
  
  Serial.begin(9600);
}

void imprime_datos(){
  Serial.print(" | cont = "); Serial.print(contador++);
  Serial.print(" | AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.print(GyZ);
  Serial.print(" | Pb = "); Serial.print(presion_barometrica);
  Serial.print(" | tmp_b = "); Serial.print(tmp_b);
  Serial.println("");
}
void loop(){

  //Lectura de MPU650 --------------------------------------------- GIRSOCOPIO ACELEROMETRO ------------------------------------------------------------------------
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  //lectura de 14 bytes a partir de la direccion 0x3B
    AcX=Wire.read()<<8|Wire.read();  //Lectura acelerometro eje X
    AcY=Wire.read()<<8|Wire.read();  //Lectura acelerometro eje Y
    AcZ=Wire.read()<<8|Wire.read();  //Lectura acelerometro eje Z
    Tmp=(Wire.read()<<8|Wire.read())/340.00+36.53;  //Lectura temperatura
    GyX=Wire.read()<<8|Wire.read();  //Lectura giroscopio eje X
    GyY=Wire.read()<<8|Wire.read();  //Lectura giroscopio eje Y
    GyZ=Wire.read()<<8|Wire.read();  //Lectura giroscopio eje Z
  //Fin lectura MPU605 ----------------------------------------------------------------------------------------------------------------------------------------------

  //Lectura de BPM180 --------------------------------------------- Presion Barometrica ------------------------------------------------------------------------
    Wire.beginTransmission(BMP_addr);
    Wire.write(0x34);
    Wire.endTransmission(false);
    Wire.requestFrom(BMP_addr,1,true);
    presion_barometrica = Wire.read();  //Lectura presion barometrica 

    Wire.beginTransmission(BMP_addr);
    Wire.write(0x2E);
    Wire.endTransmission(false);
    Wire.requestFrom(BMP_addr,1,true);
    tmp_b = Wire.read();  //Lectura temperatura
  //Fin lectura BPM180 ----------------------------------------------------------------------------------------------------------------------------------------------
  
  imprime_datos();
  delay(333);
}
