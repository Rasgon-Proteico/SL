#include <QTRSensors.h>
#include <Arduino.h>

#define PWMI 3       // PWM IZQUIERDO
#define AIZQ 5      // MOTOR IZQUIERDO 1
#define BIZQ 4     // MOTOR IZQUIERDO 2
#define PWMD 9    // PWM DERECHO
#define ADER 7   // MOTOR DERECHO 1
#define BDER 8  // MOTOR DERECHO 2
#define STBY 6 // stby

float KP = 0.01;    // Ganancia proporcional con 1.2 }
float KI = 0.0;    // Ganancia integral     con 1.3  } ya medio jalan
float KD = 0.00;  // Ganancia derivativa  con 2.0 }

int integral = 0;         // Suma de los errores anteriores (para el componente integral)
int pE = 0;              // Error de la iteración anterior (para el componente derivativo)
int pidOutput = 0;      // Salida del PID que ajustará la velocidad de los motores

int baseSpeed = 50;     // Velocidad base (0-255)
int maxSpeed = 150;    // Velocidad máxima


int error = 0;
uint16_t  position=0;
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup() {

entradasTB66();
calibracion(); 
}

void loop() {
     position = qtr.readLineBlack(sensorValues); //Use (White) if the line is white instead


          for (uint8_t i = 0; i < SensorCount; i++) {     // It commented because bring the sensor values, that´s unnecesary for the moment
           Serial.print(sensorValues[i]);
           Serial.print("    ");
         }          
         Serial.println();
         Serial.print("Posicion: ");
         Serial.print(position);
         Serial.println();
         delay(500);
         int P=(position/10);  
error=P-350;
int E=error/100;

Serial.print("P: ");
         Serial.print(P);
           Serial.println();
         Serial.print("Error: ");
       
Serial.print(error);

Serial.println();
 Serial.print("Error 2: ");
Serial.print(E);
Serial.println();

integral += error;
       int derivative = error - pE;
       int output = (KP * error) + (KI * integral) + (KD * derivative);
       int RE=(output);
       pE = error;

Serial.print("PE: ");
Serial.print(pE);
Serial.println();
Serial.print("Output: ");
Serial.print(output);
Serial.println();
Serial.print("RE: ");
Serial.print(RE);
Serial.println();

 int IZQS = constrain (baseSpeed +RE,0,maxSpeed);
  int DERS = constrain (baseSpeed - RE,0,maxSpeed);
  
  Serial.print("VeIZ: ");
Serial.print(IZQS);
Serial.println();
Serial.print("VeDER: ");
Serial.print(DERS);
Serial.println();

motores(IZQS,DERS);

}
  
void motores (int izq, int der){
  if (izq==0){
    digitalWrite(AIZQ,HIGH);
    digitalWrite(BIZQ,LOW);}
    else{
    digitalWrite(AIZQ,LOW);
    digitalWrite(BIZQ,HIGH);
  izq= -izq;
  
  }
  analogWrite(PWMI, izq);
  if (der==0){
    digitalWrite(ADER,HIGH);
    digitalWrite(BDER,LOW);}
    else{
    digitalWrite(ADER,LOW);
    digitalWrite(BDER,HIGH);
  der= -der;
  
  }
  analogWrite(PWMD, der);
 
  }
  
void calibracion () {
    // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2, A1, A0}, SensorCount);
  qtr.setEmitterPin(13);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 100; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
  }   
void entradasTB66(){
      pinMode(PWMI,OUTPUT);
      pinMode(AIZQ,OUTPUT);
      pinMode(BIZQ,OUTPUT);
      pinMode(PWMD,OUTPUT);
      pinMode(ADER,OUTPUT);
      pinMode(BDER,OUTPUT);
      pinMode(STBY,OUTPUT);
}