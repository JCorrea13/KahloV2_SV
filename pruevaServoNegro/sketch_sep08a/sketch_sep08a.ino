int servoPin = 9;  //Conectamos el servo al pin digital 9
 
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
   float pausa;                   
   digitalWrite(pin, LOW);              // Y ponemos de nuevo el pin en LOW
   delayMicroseconds(900);
   
   pausa = (angulo*(1200/180.0))+ 900;   // Calculamos el ancho del pulso aplicando la regla de tres
   digitalWrite(pin, HIGH);             // Ponemos el pin en HIGH 
   delayMicroseconds(pausa);            // Esperamos con el pin en HIGH durante el resultado de la regla de tres
   
   digitalWrite(pin, LOW);              // Y ponemos de nuevo el pin en LOW
   delayMicroseconds(20000-pausa);      // Completamos el ciclo de y empezamos uno nuevo para crear asi el tren de pulsos
}
