#include <QTRSensors.h>
//program by gerardo fregoso jimenez
#define NUM_SENSORS             6  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  5  // average 4 analog samples per sensor reading
#define EMITTER_PIN             10  // emitter is controlled by digital pin 2
//Sensor qtra 
//----------------------------------------------------------------------
#define motorder1     5
#define motorder2     4
#define motorizq1     7
#define motorizq2     8
#define pwmizq        3
#define pwmder        9
#define standby       6
int velocidad_der1 = 0;
int velocidad_izq1 = 0;
//Motores para tbgfsabeque
//----------------------------------------------------------
int derivativo=0;
int proporcional=0;
int proporcional_pasado=0;
unsigned int position;
int pwm_salida=0;
//variables pid
//-------------------------------------------------------------
float kp=0.18;  //constante de proporcional
float kd=1.89;  //constante de derivativa
 
int max=180;   //velocidad maxima

//variables a cambiar del pid 
//----------------------------------------------------
//inisiacion de sensores
  
QTRSensorsAnalog qtra((unsigned char[]) 
  {A0, A1, A2, A3, A4, A5}
, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
//----------------------------------------------------------------
//void principal
//void priincipal(){
  
//}
//------------------------------------------------------------------
//void motores

void Motorder(int valor)
{
  if ( valor >= 0 )
  {
    // si valor positivo vamos hacia adelante
    digitalWrite(motorder1,HIGH);
    digitalWrite(motorder2,LOW);
  }
  else
  {
    // si valor ngativo vamos hacia atras
    digitalWrite(motorder1,LOW);
    digitalWrite(motorder2,HIGH);
    valor *= -1;
  }

  // Setea Velocidad
  analogWrite(pwmder,valor);
}

void Motorizq(int valor)
{  
  if ( valor >= 0 )
  {
    // si valor positivo vamos hacia adelante
    digitalWrite(motorizq1,HIGH);
    digitalWrite(motorizq2,LOW);
  }
  else
  {
    // si valor negativo vamos hacia atras
    digitalWrite(motorizq1,LOW);
    digitalWrite(motorizq2,HIGH);
    valor *= -1;
  }    

  // Setea Velocidad
  analogWrite(pwmizq,valor);
}

void Velmotores(int izquierda, int derecha)// accion para mover los dos motores con Velmotores(velocidad del izquierdo, velocidad del derecho)
{
  digitalWrite(standby,HIGH); 
  Motorder(izquierda);
  Motorizq(derecha);
}


void setup() {
pinMode(motorder1, OUTPUT);
pinMode(motorizq1, OUTPUT);
pinMode(motorder2, OUTPUT);
pinMode(motorizq2, OUTPUT);
pinMode(pwmizq , OUTPUT);
pinMode(pwmder, OUTPUT);
pinMode(standby , OUTPUT);
pinMode(13, OUTPUT);
delay(1000);
//------------------------------------------------------------------
  //calibracion del sensor
  for ( int i=0; i<100; i++)      
  {
    digitalWrite(13, HIGH);
    delay(20);// parpadeo de les para indicar que inició
    
 digitalWrite(13, HIGH);
    qtra.calibrate();            // lee los 6 sensores 10 veces a 2.5 ms 
     digitalWrite(13, LOW);
    delay(20);
  }

  digitalWrite(13, LOW);  
  //------------------------------------------------------------------
  
  delay(2000);
}

void loop() {
 frenos(600);// activamos los frenos 
  position = qtra.readLine (sensorValues); // posicion es igual a la linea de valores que va de 0 A 5000 EN ESTE caso
  //------------------------------------------------------------------
  //proporcionales y derivativos
  proporcional = ((int)position) - 2500; // declaramos el proporcional que es la posicion menos 2500 asi nos queda de -2500 a 2500 teniendo como referencia de linea 0
 
   derivativo = proporcional - proporcional_pasado;// declaramos que el derivativo es el proporcional menos el proporcional pasado
    
     proporcional_pasado = proporcional;// decimos que proporcional_pasado es igual a proporcional lo que hace que cuando pase por esta linea de codigo guarde un valor anterior para el derivativo
     //------------------------------------------------------------------
     //UNION DE PD
    pwm_salida=( proporcional * kp ) + ( derivativo * kd );// decimos que nuestra salida para los motores(todavia no terminada) es esta formula s=(p*kp)+(d*kd)
     //------------------------------------------------------------------
     //deimitacion de velocidad
     if ( pwm_salida > max ) pwm_salida = max; //limitamos la salida de pwm diciendo que si es mayor la salida que nuestra velocidad maxima ponga la vlocidad maxima como salida
  else if ( pwm_salida < -max ) pwm_salida = -max;//limitamos la salida de pwm diciendo que si es mayor la salida que nuestra velocidad maxima ponga la vlocidad maxima como salida pero negativa 
 //------------------------------------------------------------------
 //Mandar velocidades a cada motor
 if( pwm_salida < 0 ) {   
 Velmotores(max+pwm_salida, max);  //comparacion de que si el valor de salida es negativo significa que se salio de la linea a la derecha entonces tiene que arreglarlo
 }
 else{
 Velmotores(max,max-pwm_salida);//comparacion de que si el valor de salida es positivo significa que se salio de la linea a la izquierda entonces tiene que arreglarlo o si es 0 ir recto
 }
 //------------------------------------------------------------------

}

void frenos(int flanco_comparacion)
{
    if (proporcional <=-2490) //si se salio por la parte derecha de la linea
    {
      while(true)
      { 
        digitalWrite(13 ,HIGH);
        Velmotores(165,-100);
        qtra.read(sensorValues); //lectura  de cada sensor 
        if ( sensorValues[0]>flanco_comparacion || sensorValues[1]>flanco_comparacion || sensorValues[2]>flanco_comparacion || sensorValues[3]>flanco_comparacion || sensorValues[4]>flanco_comparacion || sensorValues[5]>flanco_comparacion )
        {
          break;
        }
         
      }
    }
 
    if (proporcional>=2490) //si se salio por la parte izquierda de la linea
    {
      while(true)
      {
        digitalWrite(13,HIGH);
     Velmotores(-100,165); 
        qtra.read(sensorValues);
        if ( sensorValues[5]>flanco_comparacion || sensorValues[4]>flanco_comparacion || sensorValues[3]>flanco_comparacion || sensorValues[2]>flanco_comparacion || sensorValues[1]>flanco_comparacion|| sensorValues[0]>flanco_comparacion)
        {
          break;
        }
      }
  }
}
