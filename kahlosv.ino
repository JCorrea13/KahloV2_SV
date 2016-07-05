/**
* Este es el programa principal de softare de vuelo de kahlo v2
**/


void setup() {
  Wire.begin();        //inicialozacion de interfaz para lecutura de I2C
  Serial.begin(9600);  //Innicializacion de serial
}

void loop() {
  Wire.requestFrom(0xEF,1);    // request 1 bytes from slave device #0xEF

  int 
  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    Serial.print(c);         // print the character
  }

  delay(500);
}
