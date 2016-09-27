/**
  Este es el programa principal de softare de vuelo de kahlo v2
**/

#include "TinyGPS.h"
#include <SoftwareSerial.h>
#include <SFE_BMP180.h>
#include <TimerOne.h>
#include <Servo.h>
#include <Wire.h>
#include "DHT.h"

#define MPU_addr 0x68  // I2C address of the MPU-6050
unsigned long time;
unsigned long last_update_dht = 0;

//Serial de transmision de datos
#define PSERIAL_RX 13
#define PSERIAL_TX 6

//--------------------------Variables de Transmision --------------------------------------
char k = 'k';
int contador = 1;
long pitch, roll, yaw;
long lati, longi, alti;
long presion_barometrica, altitud_b, tmp_b;
long lat, lon, altitude;
long tmp_dht, humedad;
long voltaje;
//-----------------------------------------------------------------------------------------

//Variables giroscopio
#define A_R 16384.0  //Ratios de conversion
#define G_R 131.0    //Ratios de conversion
#define RAD_A_DEG = 57.295779      //Conversion de radianes a grados 180/PI
long AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
//ANGULOS
float Acc[2];
float Gy[2];
float Angle[2];

//Variables Barometro
SFE_BMP180 pressure;
double presion_base;

//Variables GPS
#define P_GPS_RX 5
#define P_GPS_TX 12
SoftwareSerial gpsSerial(P_GPS_RX , P_GPS_TX);
TinyGPS gps; //creamos el objeto TinyGPS

//variables dht
#define DHTPIN 11
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);


//servos
#define PIN_SAZUL 9
#define PIN_SNEGRO 10

//pin sensor de voltaje
#define PIN_VOLTAJE A7

//pin buzzer
#define PIN_BUZZER A2

SoftwareSerial serial(PSERIAL_RX , PSERIAL_TX);
void setup() {

  //quitamos el buffer del giroscopio
  Wire.begin();
  Wire.beginTransmission(MPU_addr); // transmit to device #8
  Wire.write(0x23);        // sends five bytes
  Wire.write(0x07);              // sends one byte
  Wire.endTransmission();
  //Inicializacion de MPU6050 - Giroscopio Acelerometro
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  //Fin de inicilizacion MPU6050

  //Inicializacion de BMP180 - Barometro ------------------------------
  pressure.begin();
  presion_base = getPressure();  //Lectura presion barometrica
  //-------------------------------------------------------------------


  //Inicializamos serial de comunicacion
  //NOTA: El serial de comunicacion se debe inicializar antes que el serial de GPS
  serial.begin(115200);
  //------------------------Iniicializamos GPS -----------------------
  gpsSerial.begin(4800);
  digitalWrite(P_GPS_TX, HIGH);
  //------------------------------------------------------------------

  //Inicializamos DHT
  dht.begin();
  
  //Inicializamos servos
  pinMode(PIN_SAZUL, OUTPUT);
  digitalWrite(PIN_SAZUL, LOW);
  pinMode(PIN_SNEGRO, OUTPUT);
  digitalWrite(PIN_SNEGRO, LOW);

    //Inicializa buzzer
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  
  //Procedimiento de inicializacion, tiempo para acomodar paracaidas y sistema de liberacion
  inicializacion();


  //Inicializamos Interrupcion para el envio de datos------------------
  Timer1.initialize(350000);         // Dispara cada 100 ms
  Timer1.attachInterrupt(ISR_envia_datos); // Activa la interrupcion y la asocia a imprime_datos
  //-------------------------------------------------------------------

  //Serial.begin(57600);
}

void ISR_imprime_datos() {
  /*Serial.print(" | cont = "); Serial.print(contador++);
    Serial.print(" | Roll = "); Serial.print(roll);
    Serial.print(" | Pitch = "); Serial.print(pitch);
    Serial.print(" | Pb = "); Serial.print(presion_barometrica);
    Serial.print(" | tmp_b = "); Serial.print(tmp_b);
    Serial.print(" | Lat = "); Serial.print(lat);
    Serial.print(" | Long = "); Serial.print(lon);
    Serial.print(" | Alt = "); Serial.print(altitude);
    Serial.print(" | Temperatura_dht = "); Serial.print(tmp_dht);
    Serial.print(" | Humedad = "); Serial.print(humedad);
    Serial.println("");
    lat = 0;
    lon = 0;*/
}

byte b1, b2, b3, b4;
void ISR_envia_datos() {

  serial.write(k);                        //Enviamos el inicion de la cadena

  b1 = (byte)contador;                    //Enviamos el contador
  b2 = (byte)(contador >> 8);
  serial.write(b2);
  serial.write(b1);

  b1 = (byte)roll;                        //Enviamos Roll
  b2 = (byte)(roll >> 8);
  b3 = (byte)(roll >> 16);
  b4 = (byte)(roll >> 24);
  serial.write(b4);
  serial.write(b3);
  serial.write(b2);
  serial.write(b1);

  b1 = (byte)yaw;                       //Enviamos Yaw
  b2 = (byte)(yaw >> 8);
  b3 = (byte)(yaw >> 16);
  b4 = (byte)(yaw >> 24);
  serial.write(b4);
  serial.write(b3);
  serial.write(b2);
  serial.write(b1);

  serial.write((byte)0);                        //Enviamos Pitch

  b1 = (byte)lat;                       //Enviamos Latitud
  b2 = (byte)(lat >> 8);
  b3 = (byte)(lat >> 16);
  b4 = (byte)(lat >> 24);
  serial.write(b4);
  serial.write(b3);
  serial.write(b2);
  serial.write(b1);


  b1 = (byte)lon;                       //Enviamos Longitud
  b2 = (byte)(lon >> 8);
  b3 = (byte)(lon >> 16);
  b4 = (byte)(lon >> 24);
  serial.write(b4);
  serial.write(b3);
  serial.write(b2);
  serial.write(b1);


  b1 = (byte)altitude;                       //Enviamos Altitud
  b2 = (byte)(altitude >> 8);
  b3 = (byte)(altitude >> 16);
  b4 = (byte)(altitude >> 24);
  serial.write(b4);
  serial.write(b3);
  serial.write(b2);
  serial.write(b1);

  b1 = (byte)presion_barometrica;                       //Enviamos Presion Barometrica
  b2 = (byte)(presion_barometrica >> 8);
  b3 = (byte)(presion_barometrica >> 16);
  b4 = (byte)(presion_barometrica >> 24);
  serial.write(b4);
  serial.write(b3);
  serial.write(b2);
  serial.write(b1);

  b1 = (byte)tmp_b;                       //Enviamos Temperatura Barometro
  b2 = (byte)(tmp_b >> 8);
  b3 = (byte)(tmp_b >> 16);
  b4 = (byte)(tmp_b >> 24);
  serial.write(b4);
  serial.write(b3);
  serial.write(b2);
  serial.write(b1);

  b1 = (byte)tmp_dht;                       //Enviamos Temperatura DHT
  b2 = (byte)(tmp_dht >> 8);
  b3 = (byte)(tmp_dht >> 16);
  b4 = (byte)(tmp_dht >> 24);
  serial.write(b4);
  serial.write(b3);
  serial.write(b2);
  serial.write(b1);

  b1 = (byte)humedad;                       //Enviamos Humedad
  b2 = (byte)(humedad >> 8);
  b3 = (byte)(humedad >> 16);
  b4 = (byte)(humedad >> 24);
  serial.write(b4);
  serial.write(b3);
  serial.write(b2);
  serial.write(b1);

  b1 = (byte)voltaje;                       //Enviamos Humedad
  b2 = (byte)(voltaje >> 8);
  b3 = (byte)(voltaje >> 16);
  b4 = (byte)(voltaje >> 24);
  serial.write(b4);
  serial.write(b3);
  serial.write(b2);
  serial.write(b1);

  contador ++;
}

boolean bandera_alt_max = true;
int contador_servo_lib = 0;
int contador_servo_par = 0;

void loop() {

  //Lectura de MPU650 --------------------------------------------- GIRSOCOPIO ACELEROMETRO ------------------------------------------------------------------------
  getDatosGiroscopio();
  //Lectura de BPM180 --------------------------------------------- Presion Barometrica ------------------------------------------------------------------------
  presion_barometrica = (getPressure() * 100);
  //Lectura GPS
  getDatosGPS();
  //Lectra datos DHT
  getDatosDHT();
  //Lectura voltaje
  getVoltaje();
  
  //movemos el servo negro
  if(altitude > 7500 && bandera_alt_max){
    contador_servo_lib = 10;
    bandera_alt_max = false;
  }
  if(contador_servo_lib > 0){
    moverServoAzul(PIN_SNEGRO ,170);
    contador_servo_lib --;
  }

    //movemos el servo negro
  if(altitude < 5000 && !bandera_alt_max){
    contador_servo_par = 10;
  }
  if(contador_servo_par > 0){
    moverServoAzul(PIN_SAZUL ,170);
    contador_servo_par --;
  }
}

void getVoltaje() {
  voltaje = (analogRead(PIN_VOLTAJE)) * (0.0450450450);
}

void inicializacion(){
  
  //Encendemos el buzzer por un segundo indicando el inicio de la configuracion
  digitalWrite(PIN_BUZZER, HIGH);
  delay(1000);
  digitalWrite(PIN_BUZZER, LOW);

  for(int i = 0; i < 10; i++)
    moverServoAzul(PIN_SNEGRO, 10);

  delay(5000);

  //Encendemos el buzzer por un segundo indicando el fin de la configuracion
  digitalWrite(PIN_BUZZER, HIGH);
  delay(1000);
  digitalWrite(PIN_BUZZER, LOW);
}

void moverServoAzul(int pin, int angulo)    // Recogemos las variables PIN y ANGULO en cada llamada
{
  float pausa;                         // Declaramos la variable float para recoger los resultados de la regla de tres
  pausa = angulo * 2000.0 / 180.0 + 700; // Calculamos el ancho del pulso aplicando la regla de tres
  digitalWrite(pin, HIGH);             // Ponemos el pin en HIGH
  delayMicroseconds(pausa);            // Esperamos con el pin en HIGH durante el resultado de la regla de tres
  digitalWrite(pin, LOW);              // Y ponemos de nuevo el pin en LOW
  delayMicroseconds(25000 - pausa);    // Completamos el ciclo de y empezamos uno nuevo para crear asi el tren de pulsos
}

void moverServoNegro(int pin, int angulo)    // Recogemos las variables PIN y ANGULO en cada llamada
{
  float pausa;
  pausa = (angulo * (1200 / 180.0)) + 900; // Calculamos el ancho del pulso aplicando la regla de tres
  digitalWrite(pin, HIGH);             // Ponemos el pin en HIGH
  delayMicroseconds(pausa);            // Esperamos con el pin en HIGH durante el resultado de la regla de tres
  digitalWrite(pin, LOW);              // Y ponemos de nuevo el pin en LOW
  delayMicroseconds(20000 - pausa);    // Completamos el ciclo de y empezamos uno nuevo para crear asi el tren de pulsos
}

void getDatosDHT() {
  time = millis();
  if ((time - last_update_dht) > 100) { //validamos el tiempo necesario para la actualizacion del sensor
    humedad = dht.readHumidity() * 100;
    tmp_dht =  dht.readTemperature() * 100;
    last_update_dht = time;
  }
}

void getDatosGPS() {

  while (gpsSerial.available() > 0)
  {

    int c = gpsSerial.read();
    // Encode() each byte
    // Check for new position if encode() returns "True"
    if (gps.encode(c))
    {
      gps.get_position(&lat, &lon);
      altitude = gps.f_altitude();
    }
  }
}


void getDatosGiroscopio() {

  //Leer los valores del Acelerometro de la IMU
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true); //A partir del 0x3B, se piden 6 registros
  AcX = Wire.read() << 8 | Wire.read(); //Cada valor ocupa 2 registros
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  //A partir de los valores del acelerometro, se calculan los angulos Y, X
  //respectivamente, con la formula de la tangente.
  Acc[1] = atan(-1 * (AcX / A_R) / sqrt(pow((AcY / A_R), 2) + pow((AcZ / A_R), 2))) * RAD_TO_DEG;
  Acc[0] = atan((AcY / A_R) / sqrt(pow((AcX / A_R), 2) + pow((AcZ / A_R), 2))) * RAD_TO_DEG;

  //Leer los valores del Giroscopio
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 4, true); //A diferencia del Acelerometro, solo se piden 4 registros
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();

  //Calculo del angulo del Giroscopio
  Gy[0] = GyX / G_R;
  Gy[1] = GyY / G_R;

  //Aplicar el Filtro Complementario
  Angle[0] = 0.98 * (Angle[0] + Gy[0] * 0.010) + 0.02 * Acc[0];
  Angle[1] = 0.98 * (Angle[1] + Gy[1] * 0.010) + 0.02 * Acc[1];



  //Roll & Pitch Equations
  roll  = Angle[0] * 100;
  yaw   = Angle[1] * 100;
  pitch = 0;
}

double getPressure()
{
  char status;
  double P, p0, a;
  double tmp;
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

    status = pressure.getTemperature(tmp);
    tmp_b = tmp * 100;
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

        status = pressure.getPressure(P, tmp);
        if (status != 0)
        {
          return (P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}
