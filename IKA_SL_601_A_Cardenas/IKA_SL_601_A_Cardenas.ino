//Variables para PID
float KP = 0.17; //Variable porporcional
float KI = 0.05; //Variable integral 
float KD = 2.2;  //Variable derivativa
//Se mueven para callibrar
#include <QTRSensors.h>
  //  #include <Servo.h>  
#define RDY 10
#define GO 11

int pwma= 3;
int ain2= 4;
int ain1= 5;
int stby=6;
int bin1=7;
int bin2=8;
int pwmb=9;
int IR=12;
int reading;
int error;
QTRSensors qtr;

const uint8_t SensorCount = 8; //Usaremos los 8 sensores
uint16_t sensorValues[SensorCount];
          void front(){
            digitalWrite(ain1,HIGH);
            digitalWrite(bin1,HIGH);
            digitalWrite(ain2,LOW);
            digitalWrite(bin2,LOW);
            digitalWrite(pwma,255);
            digitalWrite(pwmb,255);
           }
          void right(){
            digitalWrite(ain1,HIGH);
            digitalWrite(bin1,LOW);
            digitalWrite(ain2,LOW);
            digitalWrite(bin2,HIGH);
            digitalWrite(pwma,255);
            digitalWrite(pwmb,160);
           }
          void left(){
            digitalWrite(ain1,LOW);
            digitalWrite(bin1,HIGH);
            digitalWrite(ain2,LOW);
            digitalWrite(bin2,HIGH);
            digitalWrite(pwma,255);
            digitalWrite(pwmb,160);
           }
void setup()
{
  pinMode(pwma,OUTPUT);
  pinMode(ain1,OUTPUT);
  pinMode(ain2,OUTPUT);
  pinMode(pwmb,OUTPUT);
  pinMode(bin1,OUTPUT);
  pinMode(bin2,OUTPUT);
  pinMode(stby,OUTPUT);
  
  pinMode(RDY,INPUT);
  pinMode(GO,INPUT);

  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const int8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(12);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  /* analogRead() takes about 0.1 ms on an AVR.
   0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
     10 reads per calibrate() call = ~24 ms per calibrate() call.                 
    Call calibrate() 400 times to make calibration take about 10 seconds.*/

            for (int16_t i = 0; i < 200; i++)
            {
              qtr.calibrate();
            }
            digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
            for (int8_t i = 0; i < SensorCount; i++)
            {
              Serial.print(qtr.calibrationOn.minimum[i]);
              Serial.print(' ');
            }
            Serial.println();

  // print the calibration maximum values measured when emitters were on
            for (int8_t i = 0; i < SensorCount; i++)
            {
              Serial.print(qtr.calibrationOn.maximum[i]);
              Serial.print(' ');
            }
  Serial.println();
  Serial.println();
  delay(1000);
  digitalWrite(stby,HIGH);
}

void loop()
{
  if(digitalRead(GO)==HIGH){
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
              Serial.println(position);
              reading = (sensorValues[0] * -4)  + 
              (sensorValues[1] * -3)  + 
              (sensorValues[2] * -2)  + 
              (sensorValues[3] * -1)  + 
              (sensorValues[4] * 1)   + 
              (sensorValues[5] * 2)   + 
              (sensorValues[6] * 3)   + 
              (sensorValues[7] * 4);
      /*La variable reading por si sola nos arroja mucho ruido en lo que muestra la pantalla, por ello
      se encierra en la variable error que es algo más depurado y visualmente limpio*/

               error=reading;

Serial.println(error);

     if(error=>)

  delay(20);
}}
