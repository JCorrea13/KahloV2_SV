int servoPin = 10;  //Conectamos el servo al pin digital 9
 
void setup()
{
 pinMode(servoPin, OUTPUT);     // Declaramos el pin digital 9 como salida
 digitalWrite(servoPin, LOW);   // Ponemos el pin digital 9 en LOW
}
 
void loop() {
// Llamamos a la funcion "moverServo(pin, grados);" en bucle para crear 
// el tren de pulsos, ya que cada ciclo solo enviará un pulso 
moverServo(servoPin, 180);      
}
 
// Funcion para simplificar la regla de tres (modificada para un servomotor Tower PRO SG90)
void moverServo(int pin, int angulo)    // Recogemos las variables PIN y ANGULO en cada llamada 
{
   float pausa;                         // Declaramos la variable float para recoger los resultados de la regla de tres
   pausa = angulo*2000.0/180.0 + 700;   // Calculamos el ancho del pulso aplicando la regla de tres
   digitalWrite(pin, HIGH);             // Ponemos el pin en HIGH 
   delayMicroseconds(pausa);            // Esperamos con el pin en HIGH durante el resultado de la regla de tres
   digitalWrite(pin, LOW);              // Y ponemos de nuevo el pin en LOW
   delayMicroseconds(25000-pausa);      // Completamos el ciclo de y empezamos uno nuevo para crear asi el tren de pulsos
}
