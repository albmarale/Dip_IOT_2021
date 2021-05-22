/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "c:/Users/Albert/OneDrive/Escritorio/Proyecto_Final/Proyecto_Final/src/Proyecto_Final.ino"
/*
 *
 * Link de vídeo:
 * https://drive.google.com/file/d/1Jc203A7VdAOTYMdPyPDFuwGq9vnbUwuw/view?usp=sharing
 * Project Proyecto_Final
 * Description: Sensado continuo de la temperatura de un motor
 * Author: Albert Martínez Alegría
 * Date: 07-02-2021
 */
#include "Arduino.h" // Importamos esta libreria para el sensor de corriente
 
#include "adafruit-ina219.h" // Importamos la libreria del sensor de corriente

#include <Adafruit_DHT.h> //Librería para el sensor de temperatura
#include "Adafruit_DHT.h" //Librería para el sensor de temperatura

void setup();
void loop();
void parada_emergencia();
void cambio_estado_motor_boton();
#line 17 "c:/Users/Albert/OneDrive/Escritorio/Proyecto_Final/Proyecto_Final/src/Proyecto_Final.ino"
#define DHTPIN 7        //Definimos el pin 6 para el DHT
#define DHTTYPE DHT22   //Definimos el tipo de sensor que tenemos

#include "MPU6050.h" // Libreria del MPU

Adafruit_INA219 ina219;

int aux;        //Definimos una variable auxiliar
int vel_final;  //Definimos una variable para la velocidad final
int pwm = D4;   //Definimos que la salida PWM será el D3
int dato=0;     //Definimos que la variable dato, iniciará con el valor de 0
int comando;    //Definimos una variable comando
int s1 = D5;    //Definimos el Pin D4 como s1 para el sentido de giro
int s2 = D6;    //Definimos el Pin D5 como s2 para el sentido de giro  

int val_pwm = 255; //El valor de PWM para la demostración será de 255. 

int red = A0;
int green = A1;
int blue = A2;

int servo = D8;  //Definimos el Pin D7 para el servomotor 

int boton_reinicio_motor = D10;
int cont_reinicio_motor = 0;

int parada_hongo = D9; //Pin del hongo de seguridad, que en este caso será un boton
int cont_hongo = 0; // Contador del hongo, para definir el estado del mismo por cada pulso

int temperatura;  //Definimos la variable de la temperatura
int velocidad;    //Definimos la variable de la velocidad
int transporte;   //Definimos la variable del transporte

float temp_nube;  //Definimos la variable que almacenará los datos de temperatura para subirlos a la nube

int cont_tiempo_normal = 0;        //Contador para la publicación del mensaje de estado normal en SLACK
int cont_tiempo_medio = 0;         //Contador para la publicación del mensaje de estado de riesgo medio en SLACK
int cont_tiempo_alto = 0;          //Contador para la publicación del mensaje de estado critico alto en SLACK
int cont_tiempo_emergencia = 0;    //Contador para la publicación del mensaje de estado de parada de emergencia en SLACK
int cont_tiempo_nube = 0;          //Contador para la publicación del mensaje en la hoja de calculo en la nube

DHT dht(DHTPIN, DHTTYPE); //Se configura el tipo de sensor de temperatura y el pin del mismo

//Variables del IMU 
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

///////////////////////----------------SETUP---------------------////////////////////////////////////////


void setup() {

  pinMode(parada_hongo, INPUT_PULLDOWN); //Definimos como Pull Down la pulsación del boton de parada de emergencia 
  pinMode(boton_reinicio_motor, INPUT_PULLDOWN); 
  attachInterrupt(parada_hongo, parada_emergencia, RISING); //Interrupción será RISING, que es de LOW a HIGH. Para la pulsación del boton de emergencia
  attachInterrupt(boton_reinicio_motor, cambio_estado_motor_boton, RISING); 


  Particle.variable("Tempertatura Motor", &temperatura, INT); //Registramos la variable de temperatura como entero para particle
  Particle.variable("Velocidad Motor", &velocidad, INT);      //Registramos la variable de velocidad como entero para particle
  Particle.variable("Transporte Usado", &transporte, INT);    //Registramos la variable del transporte como entero para particle
        
	//Serial.begin(115200); //Iniciamos el comunicación serial a 9600 baudios para verla en el monitor

  Serial.begin(9600); //Iniciamos el comunicación serial a 9600 baudios para verla en el monitor

	//Serial.println("Temperatura del Motor!"); // 
	
	dht.begin(); //Iniciamos el sensor de temperatura 
	Wire.begin();

  accelgyro.initialize();


	pinMode(pwm, OUTPUT); //Definimos a PWM como salida
  pinMode(s1, OUTPUT);  //Definimos a s1 como salida
  pinMode(s2, OUTPUT);  //Definimos a s2 como salida
  
  pinMode(servo, OUTPUT); //Definimos a servo como salida

  pinMode(green, OUTPUT); //Definimos a Green como salida
  pinMode(red, OUTPUT); //Definimos a Red como salida
  pinMode(blue, OUTPUT); //Definimos a Blue como salida

  //uint32_t currentFrequency;
  ina219.begin();

}


///////////////////////----------------LOOP---------------------////////////////////////////////////////


void loop() {

  while (cont_hongo == 0)  //Usaremos el While, mientras el boton de emergencia no se haya presionado
  {
    delay(250); // Delay para tomar las mediciones de 2 segundos
    
    //TOMAMOS VALORES DEL IMU

    float promedio_acc_ax = 0;
    float promedio_acc_ay = 0;   
    float promedio_acc_az = 0;
    float promedio_acc_gx = 0;  
    float promedio_acc_gy = 0;
    float promedio_acc_gz = 0; 

    const float accScale = 2.0 * 9.81 / 32768.0;
    const float gyroScale = 250.0 / 32768.0;


    

    for (int cont_acc =0; cont_acc<500; cont_acc++)
      {
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        promedio_acc_ax = promedio_acc_ax + ax; //X
        promedio_acc_ay = promedio_acc_ay + ay; //Y   
        promedio_acc_az = promedio_acc_az + az; //Z   
        promedio_acc_gx = promedio_acc_gx + gx; //PITCH    
        promedio_acc_gy = promedio_acc_gy + gy; //YAW   
        promedio_acc_gz = promedio_acc_gz + gz; // ROLL      

      }

    float final_acc_ax = (promedio_acc_ax/500)*accScale;
    float final_acc_ay = (promedio_acc_ay/500)*accScale;   
    float final_acc_az = (promedio_acc_az/500)*accScale;
    float final_acc_gx = (promedio_acc_gx/500)*gyroScale;  
    float final_acc_gy = (promedio_acc_gy/500)*gyroScale;
    float final_acc_gz = (promedio_acc_gz/500)*gyroScale; 

    Serial.print("X: "); Serial.print(final_acc_ax); Serial.print(" G;  ");
    Serial.print("Y: "); Serial.print(final_acc_ay); Serial.print(" G;  ");
    Serial.print("Z: "); Serial.print(final_acc_az); Serial.println(" G  ");

    Serial.print("Pitch: "); Serial.print(final_acc_gx); Serial.print(" °/s;  ");
    Serial.print("Yaw: "); Serial.print(final_acc_gy); Serial.print(" °/s;  ");
    Serial.print("Roll: "); Serial.print(final_acc_gz); Serial.println(" °/s  ");



    //TOMAMOS LOS VALORES DE CORRIENTE

    float current_mA = 0;
    float potencia = 0;
  
    float promedio_I = 0;
    
    for (int cont_current =0; cont_current<500; cont_current++)
      {
        current_mA = ina219.getCurrent_mA();

        promedio_I = promedio_I + current_mA;        

      }

    float corriente_final = promedio_I/500;
    potencia = (corriente_final/1000)*5;  

    Serial.print("Corriente de motor:  "); Serial.print(corriente_final); Serial.println(" mA");
    Serial.print("Potencia de motor:   "); Serial.print(potencia); Serial.println(" Watts");
    
    //TOMAMOS VALORES DE HUMEDAD Y TEMPERATURA

    float h = dht.getHumidity();      // Leemos la humedad, y la ponemos en una variable float h
    float t = dht.getTempCelcius();   // Leemos la temperatura, y la ponemos en una variable float t

    if (isnan(h) || isnan(t)) {    // En caso de que el sensor mande datos erroneos nos saldrá esta advertencia.
      Serial.println("No se pudo leer datos!"); //Lo mandará por puerto serial
      return;
    }

    // Imprimimos los datos de temperatura y humedada en el puerto serial
    Serial.print("Humid: "); 
    Serial.print(h);
    Serial.print("% - ");
    Serial.print("Temp: "); 
    Serial.print(t);
    Serial.print("*C ");

    //Al igual que imprimimos la velocidad del motor por serial
    Serial.print(" Velocidad del Motor - PWM: ");
    Serial.print(vel_final);
    Serial.println("%");

    //A su vez, nos mostrará el estado del boton de emergencia, donded 0 es desactivado y 1 es activado.

    Serial.print(" BOTON EMERGENCIA (cont_hongo): ");
    Serial.println(cont_hongo);

    //Elegimos el sentido de giro con este juego, una señal debe ser HIGH y la otra LOW
    //Al intercambiar estos valores cambiamos el sentido de giro, pero como es una cinta de transporte solo debe ir en una dirección
    digitalWrite(s1,HIGH);
    digitalWrite(s2,LOW);

    //En caso de que la temperatura del motor no sea mayor a los 22grados Centigrados, el led verde se prenderá y el motor ira a su potencia normal
    if(corriente_final<=50.0)
    {
      //Encendemos el LED Verde, ya que se encuentra en los parametros normales. Los demás los apagamos.
      analogWrite(red,0);
      analogWrite(green,255);
      analogWrite(blue,0);
      
      //Damos la velocidad máxima al PWM, debido a que el motor no esta en riesgo de dañarse
      analogWrite(pwm, val_pwm);
      //Mantenemos el mismo carril, ya que el primero se encuentra en buen estado.
      analogWrite(servo, 40);
      transporte = 1;
      //Agregamos los valores de la velocidad a sus respectivas variables para su posterior visualización.
      velocidad = 100;
      vel_final = velocidad;

      cont_tiempo_normal++; //Aumentamos en uno el contador.

      if (cont_tiempo_normal ==  2)  //En caso de que el contador llegue al limite
      {
        Particle.publish("TEMP_NORMAL"); //Publicamos en SLACK que la temperatura es normal
        
        //Reseteamos los demás contadores.
        cont_tiempo_normal = 0;
        cont_tiempo_medio = 0;
        cont_tiempo_alto = 0;
        cont_tiempo_emergencia = 0;
      }
    }

    //En caso de que la temperatura del motor sea mayor a los 22grados Centigrados, el led amarillo se prenderá y el motor ira a la mitad de su potencia.
    if(corriente_final>50.0)
    {
      //Encendemos el LED Amarillo, ya que se encuentra en los parametros con cierto riesgo. Los demás los apagamos.
      analogWrite(red,255);
      analogWrite(green,34);
      analogWrite(blue,0);
      
      //Reducimos la velocidad a un 25% del PWM establecido, debido a que el motor esta en riesgo de dañarse por algun atasque 
      analogWrite(pwm, val_pwm*0.75);
      
      //Mantenemos el mismo carril, ya que este puede seguir trabajando aún que este en riesgo
      analogWrite(servo, 40);
      transporte = 1;
      //Agregamos los valores de la velocidad a sus respectivas variables para su posterior visualización.
      velocidad = 50;
      vel_final = velocidad;
      
      cont_tiempo_medio++; //Aumentamos en uno el contador.

      if (cont_tiempo_medio ==  2) //En caso de que el contador llegue al limite
      {
        Particle.publish("TEMP_MEDIO"); //Publicamos en SLACK que la temperatura del motor tiene riesgo medio.
        //Reseteamos los demás contadores.
        cont_tiempo_normal = 0;
        cont_tiempo_medio = 0;
        cont_tiempo_alto = 0;
        cont_tiempo_emergencia = 0;
      }
        
    }

    //En caso de que la temperatura del motor sea mayor a los 27grados Centigrados, el led rojo se prenderá y el motor se detendrá. Y el servomotor llevará al otro carril
    if(corriente_final>=100.0 || cont_reinicio_motor == 1)
    {

      cont_reinicio_motor = 1;

      //Encendemos el LED Rojo, ya que se encuentra en los parametros superen la temperatura segura. Los demás los apagamos.
      analogWrite(red,255);
      analogWrite(green,0);
      analogWrite(blue,0);
      
      //Apagamos el motor
      analogWrite(pwm, 0);
      
      //Cambiamos de carril
      analogWrite(servo, 200);
      transporte = 2; //Usamos la variable de transporte para definir que carril se esta usando.
      velocidad = 0; //Se usa la variable para mostrar que la velocidad es cero.
      
      vel_final = velocidad; //Ponemos el valor de la velocidad en la variable de vel_final
      
      cont_tiempo_alto++; //Aumentamos en uno el contador.

      if (cont_tiempo_alto ==  2) //bajamos el contador debido a que es importante que se avise cuando el motor esta parado
      {
        Particle.publish("TEMP_ALTO"); //Publicamos en SLACK que la temperatura del motor tiene riesgo alto. Por lo que se paro el motor.
        //Reseteamos los demás contadores.
        cont_tiempo_normal = 0;
        cont_tiempo_medio = 0;
        cont_tiempo_alto = 0;
        cont_tiempo_emergencia = 0;
      }

    }
    
    temp_nube = t; //Usamos este temporizador para que la publicacion de la informacion en slack y en el data set no sea tan seguida
    
    cont_tiempo_nube++; //Aumentamos en uno el contador de la variable anterior.

    if (cont_tiempo_nube == 4) //Contador para publicarlo en Google Sheets
    {
      
      //PUBLICAMOS LOS DATOS DE TEMPERATURA EN LA NUBE
      String temperature = String(temp_nube); // guardamos el dato de la temperatura en un string
      Particle.publish("temp_001", temperature, PRIVATE); // Publicamos en la nube, en una hoja de datos vinculada a Google.

      String corriente_str = String(corriente_final); // guardamos el dato de la corriente en un string
      Particle.publish("Corr", corriente_str, PRIVATE); // Publicamos en la nube la corriente, en una hoja de datos vinculada a Google.

      String nube_final_acc_gx = String(final_acc_gx); // guardamos el dato del giroscopio en x en un string
      Particle.publish("acc_gx_001", nube_final_acc_gx, PRIVATE); // Publicamos en la nube, en una hoja de datos vinculada a Google.


      //usamos el puerto serial para ver los datos, Tanto del contador, Como de la temperatura.
      Serial.print(" -------------- Se publico dato en la nube (contador_nube: ) "); 
      Serial.println(cont_tiempo_nube); 
      //Serial.print(" temperature: "); 
      //Serial.println(temperature);

      //Reseteamos el contador luego de que ol publicamos en la nube
      cont_tiempo_normal = 0;
      cont_tiempo_medio = 0;
      cont_tiempo_alto = 0;
      cont_tiempo_emergencia = 0;
      cont_tiempo_nube = 0;
    } 
    
  }
  

  //MIENTRAS EL BOTON DE PARADA DE EMERGENCIA ESTE ACTIVADO
  while (cont_hongo == 1)
  {
    //Publicamos por puerto serial el estado de la parada de emergencia, en caso de ser activado.
    Serial.print(" BOTON EMERGENCIA: ");
    Serial.println(cont_hongo);
    
    // Por motivos de seguridad para un tipo de mantenieminto o alguna otra emergencia, se parará el motor.
    analogWrite(pwm, 0);

    //Y encenderemos una baliza Azul en planta que se activo la parada de emergencia
    analogWrite(red,0);
    analogWrite(green,0);
    analogWrite(blue,255);
       
    //Ponemos en la variable de velodidad que esta es igual a cero.
    velocidad = 0;
    vel_final = velocidad;
    
    //Y aúmentamos el contador en 1
    cont_tiempo_emergencia++;

  
    if (cont_tiempo_emergencia ==  2) //bajamos el tiempo de aviso de que el motor esta parado por emergencia,debido a que es importante que se avise este evento
    {
      Particle.publish("PARADA_EMERGENCIA"); //Publicamos en slack que esta parado por emergencia.
      //Y reseteamos todos los contadores.
      cont_tiempo_normal = 0;
      cont_tiempo_medio = 0;
      cont_tiempo_alto = 0;
      cont_tiempo_emergencia = 0;
    }
  }     
}

//Creamos la funcion de parada de emergencia, con la interrupción
void parada_emergencia(){
  
  cont_hongo++; // Aumentamos el contador en 1, para que podamos verificar los estados del boton, donde 0 es desactivado y 1 es activado.

  if (cont_hongo == 2) //En caso de que llegue a un estado 2
  {
    cont_hongo = 0; //Reseteamos el contador, y será 0 nuevamente.
  }
   
}

void cambio_estado_motor_boton(){
  
  cont_reinicio_motor++; // Aumentamos el contador en 1, para que podamos verificar los estados del boton, donde 0 es desactivado y 1 es activado.

  if (cont_reinicio_motor == 2) //En caso de que llegue a un estado 2
  {
    cont_reinicio_motor = 0; //Reseteamos el contador, y será 0 nuevamente.
  }
   
}