/**
* Este es el programa principal de softare de vuelo de kahlo v2
**/
#include <SFE_BMP180.h>
#include <TimerOne.h>
#include <Wire.h>

#define MPU_addr 0x68  // I2C address of the MPU-6050

//--------------------------Variables de Transmision --------------------------------------
char k = 'k';
int contador = 1;
long AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
double fXg = 0;
double fYg = 0;
double fZg = 0;
long pitch_t, roll_t, yaw_t;
const float alpha = 0.5;
double presion_barometrica, altitud_b, tmp_b;
//-----------------------------------------------------------------------------------------


#define A_R 16384.0  //Ratios de conversion
#define G_R 131.0    //Ratios de conversion
#define RAD_A_DEG = 57.295779      //Conversion de radianes a grados 180/PI
//ANGULOS
float Acc[2];
float Gy[2];
float pitch, roll, yaw;

SFE_BMP180 pressure;
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
  if (pressure.begin())
      Serial.println("BMP180 init success");
    else
    {
      // Oops, something went wrong, this is usually a connection problem,
      // see the comments at the top of this sketch for the proper connections.

      Serial.println("BMP180 init fail (disconnected?)\n\n");
      while(1); // Pause forever.
    }
    presion_base = getPressure();  //Lectura presion barometrica 
  //-------------------------------------------------------------------

  //Inicializamos Interrupcion para el envio de datos------------------
    Timer1.initialize(200000);         // Dispara cada 200 ms
    Timer1.attachInterrupt(ISR_envia_datos); // Activa la interrupcion y la asocia a imprime_datos
  //-------------------------------------------------------------------
  
  Serial.begin(9600);
}

void ISR_imprime_datos(){
  Serial.print(" | cont = "); Serial.print(contador++);
  Serial.print(" | Roll = "); Serial.print(roll);
  Serial.print(" | Pitch = "); Serial.print(pitch);
  Serial.print(" | Pb = "); Serial.print(presion_barometrica);
  Serial.print(" | tmp_b = "); Serial.print(tmp_b);
  Serial.println("");
}

byte b1, b2, b3, b4;
void ISR_envia_datos(){

  Serial.write(k);                        //Enviamos el inicion de la cadena
  
  b1 = (byte)contador;                    //Enviamos el contador
  b2 = (byte)(contador >> 8);
  Serial.write(b2);
  Serial.write(b1);

  roll_t = roll * 100;
  b1 = (byte)roll_t;                        //Enviamos Roll
  b2 = (byte)(roll_t >> 8);
  b3 = (byte)(roll_t >> 16);
  b4 = (byte)(roll_t >> 24);
  Serial.write(b4);
  Serial.write(b3);
  Serial.write(b2);
  Serial.write(b1);

  pitch_t = pitch * 100;
  b1 = (byte)pitch_t;                       //Enviamos Pitch
  b2 = (byte)(pitch_t >> 8);
  b3 = (byte)(pitch_t >> 16);
  b4 = (byte)(pitch_t >> 24);
  Serial.write(b4);
  Serial.write(b3);
  Serial.write(b2);
  Serial.write(b1);

  Serial.write(0);                        //Enviamos Yaw
  
  contador ++;
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

    Acc[0] = atan((AcY / A_R) / sqrt(pow((AcX / A_R), 2) + pow((AcZ / A_R), 2))) * RAD_TO_DEG; //Calculo del agulo en X [0]
    Acc[1] = atan(-1 * (AcX / A_R) / sqrt(pow((AcY / A_R), 2) + pow((AcZ / A_R), 2))) * RAD_TO_DEG; //Calculo del agulo en Y [1]
    Gy[0] = GyX / G_R;  //Calculo del angulo en X[0]
    Gy[1] = GyY / G_R;  //Calculo del angulo en Y[1]

    //Roll & Pitch Equations
    roll  =  (0.98 * (roll + Gy[0] * 0.010) + 0.02 * Acc[0]); //Filtro en X[0]
    pitch =  (0.98 * (pitch + Gy[1] * 0.010) + 0.02 * Acc[1]); //Filtro en Y[1]
    yaw = 0;
  
  //Fin lectura MPU605 ----------------------------------------------------------------------------------------------------------------------------------------------

  //Lectura de BPM180 --------------------------------------------- Presion Barometrica ------------------------------------------------------------------------
    presion_barometrica = getPressure();
  //Fin lectura BPM180 ----------------------------------------------------------------------------------------------------------------------------------------------
  
  delay(333);
}

double getPressure()
{
  char status;
  double P,p0,a;

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

        status = pressure.getPressure(P,tmp_b);
        if (status != 0)
        {
          return(P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}
