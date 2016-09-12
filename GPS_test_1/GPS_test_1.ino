// A simple sketch to detect the Southern Hemisphere
// Assumes: LED on pin 13, GPS connected to Hardware Serial pins 0/1
#include "TinyGPS.h"
#include <SoftwareSerial.h>

TinyGPS gps; // create a TinyGPS object

#define HEMISPHERE_PIN 13

char k = 'k';
int c;
long lat, lon;
long altitude; 
byte b1, b2, b3, b4;
SoftwareSerial gpsSerial(5 , 12); //6 -> rx, 5 -> tx

void setup()
{
  Serial.begin(9600);                 // Abre el puerto serial, GPS devices frequently operate at 4800 baud,
  pinMode(HEMISPHERE_PIN, OUTPUT);
  digitalWrite(HEMISPHERE_PIN, LOW); // turn off LED to start
  //pinMode(LED1,OUTPUT);             //Declaracion de analogo 1 Sensor de Voltaje
  Serial.println("Sky-Ranger 'Serial.print'");
  gpsSerial.begin(4800);
  digitalWrite(5, HIGH);
}
void loop()
{

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

  /*
  Serial.print("Lat: ");
  Serial.println(lat);
  Serial.print("Lon: ");
  Serial.println(lon);
  Serial.print("Altitud (mts): ");
  Serial.println(altitude);
  
  Serial.println("-------------------------------------------");
  */

 
  Serial.write(k);                        //Enviamos el inicion de la cadena
  
  b1 = (byte)lat;                       //Enviamos Latitud
  b2 = (byte)(lat>> 8);
  b3 = (byte)(lat>> 16);
  b4 = (byte)(lat>> 24);
  Serial.write(b4);
  Serial.write(b3);
  Serial.write(b2);
  Serial.write(b1);

  
  b1 = (byte)lon;                       //Enviamos Longitud
  b2 = (byte)(lon>> 8);
  b3 = (byte)(lon>> 16);
  b4 = (byte)(lon>> 24);
  Serial.write(b4);
  Serial.write(b3);
  Serial.write(b2);
  Serial.write(b1);

  
  b1 = (byte)altitude;                       //Enviamos Altitud
  b2 = (byte)(altitude >> 8);
  b3 = (byte)(altitude >> 16);
  b4 = (byte)(altitude >> 24);
  Serial.write(b4);
  Serial.write(b3);
  Serial.write(b2);
  Serial.write(b1);
 
  
  delay (300);


}
