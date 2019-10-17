B.1 PID librería Arduino
#include <PID_v1.h> //librería PID
#include <Servo.h>  //librería Servo
#define MAX_SIGNAL 2000 //máxima salida del brushless
#define MIN_SIGNAL 1000 //mínima salida del brushless
#define MOTOR_PIN 9    //pin para el brushless

int lectura=0;//lectura serial
double lecturaPotenciometro=0;//lectura del potenciómetro

Servo motor;

int T=20;//tiempo de muestreo de 20 ms

//parametros del controlador
double Kp=5;
double Ki=8.5;
double Kd=0.03;
double setPoint=45;
double salida;

//Se declara un nuevo elemento PID con sus respectivas variables
PID myPID(&lecturaPotenciometro, &salida,&setPoint,Kp,Ki,Kd, DIRECT);

void setup() {

  Serial.begin(9600);
  delay(1500);
  motor.attach(MOTOR_PIN); // Se calibra el brushless
  motor.writeMicroseconds(MAX_SIGNAL);
  motor.writeMicroseconds(MIN_SIGNAL);
  delay(1500);
    //se declara el PID en modo automático
  myPID.SetMode(AUTOMATIC);

  while (!Serial.available()){}

}
void loop() {

   //lectura serial
  if (Serial.available() > 0)
  {
    lectura = Serial.parseInt();
    if (lectura > 999 and lectura < 1500) // cambia el valor en lazo abierto del PWM 
    {
      motor.writeMicroseconds(lectura);
    }     
        if (lectura > 0 and lectura < 90) // cambia el setpoint
    {
      setPoint=lectura;
    }    
  }

    lecturaPotenciometro = analogRead(0); // lee el potenciómetro
    // convierte la lectura del potenciómetro en el ángulo
    lecturaPotenciometro = map(lecturaPotenciometro,419,943,113,-25);
    Serial.println(lecturaPotenciometro); // imprime la lectura por el serial
    myPID.Compute(); // calcula la salida del PID
    motor.writeMicroseconds(salida); // envía la señal de control  al brushless
    delay(T); // tiempo de muestreo
}
