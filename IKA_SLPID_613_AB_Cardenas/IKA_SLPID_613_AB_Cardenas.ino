#include "Arduino.h"
// Sensors and motors variable
#include <QTRSensors.h>
#include <Arduino.h>
//#include <Servo.h>  
//Ya no se usará la libreria de servo, solo PWM

#define RDY 10 //en lugar de poner "define" se puede usar una variable tipo entera
#define GO 11

float KP = 1.2; // Ganancia proporcional con 1.2 }
float KI = 1.3; // Ganancia integral     con 1.3  } ya medio jalan
float KD = 2.0;  // Ganancia derivativa  con 2.0 }

int integral = 0;       // Suma de los errores anteriores (para el componente integral)
int previousError = 0;  // Error de la iteración anterior (para el componente derivativo)
int pidOutput = 0;      // Salida del PID que ajustará la velocidad de los motores

int baseSpeed = 50;  // Velocidad base (0-255)
int maxSpeed = 170;   // Velocidad máxima

void setMotor(int pwmPin, int dir1, int dir2, int speed);
void init_pines();


      int pwma = 3;                        //Desde el 3, hasta el 9 es el arreglo original, el orden de las varbles es igual al orden del puente H
      int ain2 = 5;                         
      int ain1 = 4;                      //Mantener esta configuracion si es el de sin turbina
      int stby = 6;                     //11 con la placa de Cárdenas
      int bin1 = 8;
      int bin2 = 7;
      int pwmb = 9;                  //Este irá conectado con un diablito si se usa la placa de Cárdenas
   // int pwmb = 11;                //Descomment the pin if use turbine, and comment the superior line

     // int IR = 12;
      int IR = 12;
      int reading;
      int mistake;            //Error a usar
      QTRSensors qtr;

const uint8_t SensorCount = 8; // Use 8 sensors
uint16_t sensorValues[SensorCount];

void setup()
{
  
  init_pines();


  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Turn on Arduino´s led to show that are in calibration mode

  // Sensors calibration
  //Documentation recomends this
       for (int16_t i = 0; i < 300; i++) {
         qtr.calibrate();
       }

       digitalWrite(LED_BUILTIN, LOW); // Turn off Arduino´s led to show that are out of calibration mode

  // Print the minimun and maximun sensor´s values
  Serial.begin(9600);
  Serial.println("Minimos");
  for (int8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  Serial.println("Maximos");
       for (int8_t i = 0; i < SensorCount; i++) {
         Serial.print(qtr.calibrationOn.maximum[i]);
         Serial.print(' ');
       }
       Serial.println();
       Serial.println();
       delay(1000);
       digitalWrite(stby, HIGH); //Mantener activado para prender el puente h
     }
int num=0;
void loop()
{
  //if(digitalRead(GO) == HIGH) {                   // Only descoment to use the judge control, while all is good :)
   // digitalWrite(stby, HIGH);
   // // Read sensor values and determine line´s position
   uint16_t position = qtr.readLineBlack(sensorValues); //Use (White) if the line is white instead


          for (uint8_t i = 0; i < SensorCount; i++) {     // It commented because bring the sensor values, that´s unnecesary for the moment
           Serial.print(sensorValues[i]);
           Serial.print("    ");
         }          
         Serial.println();
         Serial.print("Posicion: ");
         Serial.print(position);
         Serial.println();
         delay(1000);                                      // It commented because bring the sensor values, that´s unnecesary for the moment
    // Calcular el error proporcional

    //Aparentemente no es necesario este arreglo

   /* reading = (sensorValues[0] * -4) +
              (sensorValues[1] * -3) +
              (sensorValues[2] * -2) +
              (sensorValues[3] * -1) +
              (sensorValues[4] * 1) +
              (sensorValues[5] * 2) +
              (sensorValues[6] * 3) +
              (sensorValues[7] * 4);
    mistake = reading;*/

       int error = position - 3500;
       integral += error;
       int derivative = error - previousError;
       int output = KP * error + KI * integral + KD * derivative;
       previousError = error;
  
  // Ajustar velocidades de los motores
  int leftSpeed = baseSpeed - output;
  int rightSpeed = baseSpeed + output;
  
  // Limitar velocidades
  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);
  Serial.println(num);

  Serial.print("Izq: ");
  setMotor(pwma, ain1,ain2, leftSpeed);
  Serial.print("Der: ");
  setMotor(pwmb, bin1, bin2, rightSpeed);
  //delay(100);
  num+=1;
    
}

/*Explicación del código con PID:
Variables PID:

KP, KI, y KD: Son las constantes de proporcionalidad, integral y derivativa respectivamente. Estas definen cómo responde el controlador PID a los errores de seguimiento.
integral: Almacena la suma acumulativa de los errores pasados, lo que ayuda a corregir el error a largo plazo.
previousError: Guarda el error de la iteración anterior, que se utiliza para calcular la derivada del error y responder a cambios rápidos.
pidOutput: Es la salida total del controlador PID que se usa para ajustar la velocidad de los motores.
Cálculo del PID:

Proporcional (KP * error): Responde a la diferencia actual entre la línea y el robot.
Integral (KI * integral): Ayuda a eliminar el error acumulado a lo largo del tiempo.
Derivativa (KD * derivative): Ayuda a prever futuros cambios en el error para responder de manera más precisa.
Ajuste de los motores:

Se usa constrain() para asegurarse de que las velocidades de los motores no superen el rango válido (0-255).
Interacción entre las variables:
error se calcula usando la lectura de los sensores, y este error es corregido por el PID.
La salida del PID (pidOutput) ajusta las velocidades de los motores, de modo que el robot siga la línea de manera más precisa.
El integral ayuda a corregir pequeños errores persistentes, mientras que la derivada responde a cambios bruscos. */
void setMotor(int pwmPin, int dir1, int dir2, int speed) {

     Serial.print(pwmPin);
       Serial.print(",");
       Serial.print(dir1);
       Serial.print(",");
       Serial.print(dir2);
       Serial.print(",");
       Serial.print(speed);
       Serial.println();
            digitalWrite(dir1, speed > 0 ? HIGH : LOW);
            digitalWrite(dir2, speed > 0 ? LOW : HIGH);
            analogWrite(pwmPin, abs(speed));
}

void init_pines(){                        //Declarados e iniciados, más arriba se inicia y da claridad
  Serial.begin(9600);
  
  pinMode(pwma, OUTPUT);
  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(pwmb, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);
  pinMode(stby, OUTPUT);

  pinMode(RDY, INPUT);
  pinMode(GO, INPUT);
    // Arreglo para los sensores
  qtr.setTypeAnalog();
  qtr.setSensorPins((const int8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(13);
}
