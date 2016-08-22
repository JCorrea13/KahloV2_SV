/**
 * TIPOS DE DATOS ARDUINO
 * 
 * char - 1 byte
 * int - 2 bytes
 * float - 4 bytes
 */


char k = 'k';
int contador = 10;
int presion = 500;
int tmp = 228;
long longitud = 206376856;
long latitud = -1034019972;
long yaw = 374170;
long pitch = -6789284;
long roll = 1237722;

void setup() {
  Serial.begin(9600);
}

byte b1;
byte b2;
byte b3;
byte b4;

void loop() {
  Serial.write(k);

  b1 = (byte)contador;
  b2 = (byte)(contador >> 8);
  Serial.write(b2);
  Serial.write(b1);
  b1 = (byte)tmp;
  b2 = (byte)(tmp >> 8);
  Serial.write(b2);
  Serial.write(b1);

  b1 = (byte)latitud;
  b2 = (byte)(latitud >> 8);
  b3 = (byte)(latitud >> 16);
  b4 = (byte)(latitud >> 24);
  Serial.write(b4);
  Serial.write(b3);
  Serial.write(b2);
  Serial.write(b1);

  b1 = (byte)longitud;
  b2 = (byte)(longitud >> 8);
  b3 = (byte)(longitud >> 16);
  b4 = (byte)(longitud >> 24);
  Serial.write(b4);
  Serial.write(b3);
  Serial.write(b2);
  Serial.write(b1);

  /*b1 = (byte)yaw;
  b2 = (byte)(yaw >> 8);
  b3 = (byte)(yaw >> 16);
  b4 = (byte)(yaw >> 24);
  Serial.write(b4);
  Serial.write(b3);
  Serial.write(b2);
  Serial.write(b1);

  b1 = (byte)pitch;
  b2 = (byte)(pitch >> 8);
  b3 = (byte)(pitch >> 16);
  b4 = (byte)(pitch >> 24);
  Serial.write(b4);
  Serial.write(b3);
  Serial.write(b2);
  Serial.write(b1);

  b1 = (byte)roll;
  b2 = (byte)(roll >> 8);
  b3 = (byte)(roll >> 16);
  b4 = (byte)(roll >> 24);
  Serial.write(b4);
  Serial.write(b3);
  Serial.write(b2);
  Serial.write(b1);*/
  /*Serial.print(presion);
  Serial.print(tmp);
  Serial.print(humedad);
  Serial.print(tmp_e);*/
  contador ++;
  delay(200);
}

