/**
* Este es el programa principal de softare de vuelo de kahlo v2
**/
#include <SFE_BMP180.h>
#include<Wire.h>

#define MPU_addr 0x68  // I2C address of the MPU-6050
#define BMP_addr 0x77  // I2C address of the BMP180
SFE_BMP180 pressure;

//--------------------------Variables de Transmision --------------------------------------
int contador = 0;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
double presion_barometrica, altitud_b, tmp_b;
//-----------------------------------------------------------------------------------------

double presion_base;

void setup(){

  //Inicializacion de MPU6050 - Giroscopio Acelerometro
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
  //Fin de inicilizacion MPU6050

  //Inicializacion de BMP180 - Barometro ------------------------------
    presion_base = getPressure();  //Lectura presion barometrica 
  //-------------------------------------------------------------------
  
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

double getPressure()
{
  char status;
  double p0,a;

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(tmp_b);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(presion_barometrica,tmp_b);
        if (status != 0)
        {
          return(presion_barometrica);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
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
    presion_barometrica = getPressure();  //Lectura presion barometrica 
    altitud_b = pressure.altitude(presion_barometrica,presion_base);
  //Fin lectura BPM180 ----------------------------------------------------------------------------------------------------------------------------------------------
  
  imprime_datos();
  delay(333);
}
