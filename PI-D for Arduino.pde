#include <Servo.h> //librería servo
#define MAX_SIGNAL 2000 //máxima salida del brushless
#define MIN_SIGNAL 1000 //mínima salida del brushless
#define MOTOR_PIN 9 //pin del brushless
int lectura=0;// lectura serial
double lecturaPotenciometro=0; //lectura del potenciómetro
Servo motor; 
int T=20; // tiempo de muestreo
// parámetros del controlador
double Kp=5;
double Ki=8.5;
double Kd=0.03;
double setPoint=45;
// parámetros para la ecuación de diferencias
float ei;
float e[3];
float u;
float uAnterior=1000;
float Tm=0.02;

float b0= Kp + Ki*Tm;
float b1= -(Kp);
float b2= -(Kp + Ki*Tm + Kd/Tm);
float b3= Kp + 2*Kd/Tm;
float b4= -Kd/Tm;

int Yik=0;
int YiK1=0;
int Yik1=0;
int Yk=0;
int Yk1=0;
int Yk2=0;

void setup() {
  Serial.begin(9600); // velocidad de la conexión serial
  delay(1500);
  motor.attach(MOTOR_PIN);// calibra el brushless
  motor.writeMicroseconds(MAX_SIGNAL);
  motor.writeMicroseconds(MIN_SIGNAL);
  while (!Serial.available()){} // espera la llegada de algún dato por serial
}
void loop() {
  if (Serial.available() > 0)  //lectura serial
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
    lecturaPotenciometro = analogRead(0); //lectura del potenciómetro 
    lecturaPotenciometro = map(lecturaPotenciometro,419,943,113,-25); // convierte la lectura en un angulo
    Serial.println(lecturaPotenciometro); // imprime por serial el ángulo
    ei=setPoint-lecturaPotenciometro; // calcula el error 
    u[0]= b0*Yik +b1*Yik1 + b2*Yk + b3*Yk1 + b4*Yk2 + uAnterior;// calcula la salida del controlador
    // actualiza valores para la próxima iteración 
    e[1]=e[0];
    e[0]=ei;
    uAnterior=u;
    motor.writeMicroseconds(u); // envía la señal de control al brushless
    delay(T); // tiempo de muestreo
}
