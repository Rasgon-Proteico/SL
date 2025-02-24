//Fin de competencia 21/02/2025 en la UVM guadalajara y el código quedó así:

#include <ServoTimer2.h> 
#include <QTRSensors.h>

#define rdy 11
#define go 3

#define PWMI 10       // PWM IZQUIERDO
#define AIZQ 4       // MOTOR IZQUIERDO 1
#define BIZQ 5      // MOTOR IZQUIERDO 2
#define PWMD 9     // PWM DERECHO
#define ADER 8    // MOTOR DERECHO 1
#define BDER 7   // MOTOR DERECHO 2
#define STBY 6  // stby

#define ESC_PIN 12  // Pin del ESC

ServoTimer2 esc;            // Objeto para el ESC
bool turbina=false;        //Bool para encender la turbina primero y calibrar
int izq;
int der;
float KP = .0015;             //.049  A 40 BASE; 0.088 a 45; 0.098 a 50
float KI = 0.005;           //0.0067 A 40 BASE;  0.0068 a 45; 0.0082 a 50
float KD = 1.2;            //0.0085 a 45; 0.0144 a 50;
int integral = 0;         // Suma de los errores anteriores (para el componente integral)
int pE = 0;              // Error de la iteración anterior (para el componente derivativo)
int pidOutput = 0;      // Salida del PID que ajustará la velocidad de los motores

int baseSpeed = 180;  // Velocidad base (0-255)
int maxSpeed = 255;  // Velocidad máxima
int error = 0;
uint16_t  position=0;

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup() {
  turbina=false;
  Serial.begin(9600);

 // TCCR1B = TCCR1B  & B11111000 | B00000010; // para frecuencia PWM de 3921,16 el original, usando timer 1
  TCCR2B = (TCCR2B & B11111000) | B00000010; // Configura el prescaler para obtener 3921,16 Hz, en timer 2 por la lbrería


    entradasTB66();          // Método para los pines 
     esc.attach(ESC_PIN);    // Conectar el ESC al pin 12
      arrancarESC();         // Iniciar ESC en estado seguro

  calibracion(); 

    }

void loop() {

if(!turbina){
  turbina=true;
 // if(digitalRead (rdy)==HIGH){   //original 1, CJ
  init_turbina();               //en el original el método es escrito sin el ciclo for
 //}
  //if(digitalRead(go)==LOW){
    //    while(digitalRead(go)==LOW){    //original 0
      
    setESC(110); // Ajusta la velocidad (1000 = mínimo, 2000 = máximo)
    delay(20);
    
     position = qtr.readLineBlack(sensorValues);           //Use (White) if the line is white instead
          for (uint8_t i = 0; i < SensorCount; i++) {     // It commented because bring the sensor values, that´s unnecesary for the moment
           Serial.print(sensorValues[i]);
           Serial.print("    ");
         }         
         int P=(position);  
         error=(P-3500)/10;

    if(error< - 350){
  digitalWrite(AIZQ,LOW);
   digitalWrite(BIZQ,HIGH);
    analogWrite(PWMI,120);
    
  digitalWrite(ADER,LOW);
   digitalWrite(BDER,HIGH);
    analogWrite(PWMD,180);

    } else if (error > 350) {
  digitalWrite(AIZQ,HIGH);
   digitalWrite(BIZQ,LOW);
    analogWrite(PWMI,135);
    
  digitalWrite(ADER,HIGH);
   digitalWrite(BDER,LOW);
    analogWrite(PWMD,90);

    }else { 

      int integral =(error+pE);
      integral =constrain (integral,-100,100);
       int derivative =( error - pE);
       int output = (KP * error) + (KI * integral) + (KD * derivative);
       float RE=(output);
       pE = error;
          

      int IZQS = constrain (baseSpeed +RE ,30,maxSpeed);
      int DERS = constrain (baseSpeed -RE ,30,maxSpeed);
     
    digitalWrite(AIZQ,HIGH);
    digitalWrite(BIZQ,LOW);
    
    digitalWrite(ADER,HIGH);
    digitalWrite(BDER,LOW);

  
  izq=IZQS;
  analogWrite(PWMI, izq);
  der=DERS;
  analogWrite(PWMD, der);

  //  }
  //break;             //Fin del while
  }
    }
      }
      //}
  
  
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
  // Call calibrate() 200 times to make calibration take about 4.8 seconds.
  for (uint16_t i = 0; i < 200; i++)
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

  pinMode(rdy, INPUT);
  pinMode(go, INPUT);

  digitalWrite(STBY, HIGH); 

    }

void arrancarESC() {
    setESC(1000);    // Señal de apagado para iniciar correctamente
     delay(2000);   // Esperar a que el ESC se estabilice
  }

                 // ⚡ Controlar la velocidad del ESC
void setESC(int throttle) {
    throttle = constrain(throttle, 1000, 2000); // Asegurar valores correctos
    esc.write(throttle);
    
  }

  void init_turbina(){
    arrancarESC();
        setESC(1000); // Ajusta la velocidad (1000 = mínimo, 2000 = máximo)
   
    delay(20);
  
  }