#define PWMI 3 // PWM IZQUIERDO
#define AIZQ 5 // MOTOR IZQUIERDO 1
#define BIZQ 4 // MOTOR IZQUIERDO 2
#define PWMD 9 // PWM DERECHO
#define ADER 7 // MOTOR DERECHO 1
#define BDER 8 // MOTOR DERECHO 2
#define STBY 6 // stby
int izq;
int der;
float KP = 0.25; //.049  A 40 BASE 
float KI = 0.00;//0.0067 A 40 BASE  
float KD = 0.5; 

int integral = 0;       // Suma de los errores anteriores (para el componente integral)
int pE = 0;  // Error de la iteración anterior (para el componente derivativo)
int pidOutput = 0;      // Salida del PID que ajustará la velocidad de los motores

int baseSpeed = 120;  // Velocidad base (0-255)
int maxSpeed = 200;   // Velocidad máxima

#include <QTRSensors.h>
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
         delay(100);
         int P=(position);  
error=(P-3500)/10;


Serial.print("P: ");
         Serial.print(P);
           Serial.println();
         Serial.print("Error: ");
       
Serial.print(error);

Serial.println();
 Serial.print("Error 2: ");

Serial.println();

integral += error;
integral =constrain (integral,-500,500);
       int derivative = error - pE;
       int output = (KP * error) + (KI * integral) + (KD * derivative);
       float RE=(output);
       pE = error;

/*Serial.print("PE: ");
Serial.print(pE);
Serial.println();
Serial.print("Output: ");
Serial.print(output);
Serial.println();
Serial.print("RE: ");
Serial.print(RE);
Serial.println();*/

 int IZQS = constrain (baseSpeed + RE ,0,maxSpeed);
  int DERS = constrain (baseSpeed - RE ,0,maxSpeed);
  
  Serial.print("VeIZ: ");
Serial.print(IZQS);
Serial.println();
Serial.print("VeDER: ");
Serial.print(DERS);
Serial.println();

if (IZQS==0){
    digitalWrite(AIZQ,HIGH);
    digitalWrite(BIZQ,LOW);}
    else{
    digitalWrite(AIZQ,LOW);
    digitalWrite(BIZQ,HIGH);
    
  }
  
  if (DERS==0){
    digitalWrite(ADER,HIGH);
    digitalWrite(BDER,LOW);}
    else{
    digitalWrite(ADER,LOW);
    digitalWrite(BDER,HIGH);
  

  }
  izq=IZQS;
  analogWrite(PWMI, izq);
  der=DERS;
  analogWrite(PWMD, der);
/*     Serial.print("PWMI");
Serial.print(PWMI);
Serial.println();
Serial.print("PWMD: ");
Serial.print(PWMD);
Serial.println();*/

}
  
/*void motores (int izq, int der){
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
    Serial.print("PWMI");
Serial.print(PWMI);
Serial.println();
Serial.print("PWMD: ");
Serial.print(PWMD);
Serial.println();
  }
  analogWrite(PWMD, der);
 
  }*/
  
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
  pinMode(AIZQ, OUTPUT);
  pinMode(BIZQ, OUTPUT);
  pinMode(ADER, OUTPUT);
  pinMode(BDER, OUTPUT);
  pinMode(PWMI, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); 

  }
