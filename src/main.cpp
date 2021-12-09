#include <Arduino.h>
#include <WiFi.h>
#include <MQTT.h>
#include <stdlib.h>
#include <ctime>

//Red
const char ssid[] = "TPP055FC9";
const char pass[] = "16AZUL12";

WiFiClient net;
MQTTClient client;

#define cantMuestras 20

//FUNCIONES
void leerTemperatura(void);
void estadoLed(void);
void apagarLeds(void);
void limpiarVariables(void);
void publicar(void);
void pulsadorDeCorte(void);
//void connect(void);
float getTemp(void);

//Pines de los Leds
const int8_t ledInterno = 2;
const int8_t ledRojo = 33, ledAmarillo = 32, ledVerde = 25, button = 27; //sensor = 13;


//Variables de temperatura
float temperatura;

//Variable global String
String ESTADO;
String mensaje;

//Timer incial
unsigned long tiempoInicio = 0;
unsigned long timerDelay = 500;

//Variables auxiliares
static int8_t pulsador = 0, cont = 0;
static float suma = 0.0, promTemperatura;


void setup() {
   
  Serial.begin(115200);
  pinMode(ledInterno, OUTPUT); // Se utilizo para visualizar a simple vista si el pulsador fue accionado o si hay rebotes
  pinMode(ledRojo, OUTPUT);
  pinMode(ledVerde, OUTPUT);
  pinMode(ledAmarillo, OUTPUT);
  //pinMode(sensor, INPUT);
  pinMode(button, INPUT_PULLUP); //Al accionarse el pulsador se recibe un cero

  WiFi.begin(ssid, pass);   //NOMBRE DE LA RED WIFI Y CONTRASEÑA
  client.begin("192.168.11.170", 1883, net);
  connect();

  srand(time(NULL));

}

void loop() {


  while (digitalRead(button) == 1); //Cuando el pulsador es accionado sale del bucle while
  
  pulsador = !digitalRead(button);
  
  client.loop();        //Si o si tiene que estar Actualiza el estado de mi cliente MQTT

  if (!client.connected()) {  //Me aseguro que este conectado
    connect();
  }
   
  delay(300);

  while (pulsador == 1)
  {
    digitalWrite(ledInterno, HIGH);
    leerTemperatura();
    cont++;
    
    //cantMuestras es un valor de la cantidad de veces que se sumara la temperatura y se hara el promedio para obtener el valor de temperatura final
    while (cont == cantMuestras)
    {
      
      if ((millis() - tiempoInicio) == timerDelay || tiempoInicio == 0 ) //Se coloca la condicion "|| tiempoInicio==0" para que entre en el IF en la primer ejecucion del programa
      { //Cada 5 seg(5000ms) se muestra el valor de temperatura
        
        Serial.print(promTemperatura, 1);
        Serial.println();
        estadoLed();

        publicar();

        tiempoInicio = millis();
        limpiarVariables();
      }
      pulsadorDeCorte();

    }
  }

  limpiarVariables();
  apagarLeds();

  delay(500);

}


//==========ADQUISICION DE DATOS A TRAVES DEL ADC Y CONVERSION A °C=============

void leerTemperatura()
{
  //Se simula un sensor de temperatura

  temperatura = ((rand()%40+(260/113))+20+(260/113))/1.1; //Rango aproximado de 22 a 58°C de num decimales
  suma = suma + temperatura;
  promTemperatura = suma / cantMuestras; 

}


//*****DETECTA MEDIDAS FUERA DEL RANGO DE TEMP PERMITIDO, GENERANDO ESTADO DE ALERTA*******

void estadoLed()
{

  //Estado normal
  if (promTemperatura <= 25)
  {

    digitalWrite(ledRojo, LOW);
    digitalWrite(ledVerde, HIGH);
    digitalWrite(ledAmarillo, LOW);
    //Serial.print("Normal");
    char estado1[7] = "NORMAL";
    ESTADO = String(estado1);
    Serial.println();

  }

  //Estado alerta
  if (promTemperatura > 25 && promTemperatura <= 30)
  {
    digitalWrite(ledVerde, LOW);
    digitalWrite(ledRojo, LOW);
    digitalWrite(ledAmarillo, HIGH);
    //Serial.print("Precaución");
    char estado2[11] = "PRECAUCION";
    ESTADO = String(estado2);
    Serial.println();
  }

  //Estado critico
  if (promTemperatura >= 30)
  {
    digitalWrite(ledVerde, LOW);
    digitalWrite(ledRojo, HIGH);
    digitalWrite(ledAmarillo, LOW);
    //Serial.print("Critico");
    char estado3[8] = "CRITICO";
    ESTADO = String(estado3);
    Serial.println();
  }

}


//++++++APAGA TODO LOS LEDS LUEGO DE PULSAR POR 2da VEZ EL BUTTON+++++++

void apagarLeds()
{
  digitalWrite(ledInterno, LOW);

  digitalWrite(ledRojo, LOW);
  digitalWrite(ledVerde, LOW);
  digitalWrite(ledAmarillo, LOW);
}


//###########COLOCA LAS VARIABLES EN CERO###############

void limpiarVariables()
{
  suma = 0.0;
  promTemperatura = 0.0;
  cont = 0;
}


//---SE PRESIONA POR SEGUNDA VEZ EL BOTON Y SALE DEL BUCLE WHILE------
void pulsadorDeCorte()
{
  if (digitalRead(button) == 0)
  {
    pulsador = 0;
    tiempoInicio = 0;
    limpiarVariables();
  }
}

//______________PUBLICAR______________
void publicar()
{
  //Se envia el estado de temperatura y el valor de temperatura
  String mensaje = "{\"topico1\":" + ESTADO + ",\"topico2\":" + String(promTemperatura, 1) + "}";
  client.publish("Temp", mensaje);

}


//______________CONEXION__________________
void connect() {

  //Conectar wifi
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  //Conectar al brocker MQTT
  Serial.print("\nconnecting...");
  while (!client.connect("arduino")) { //, "public", "public")) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");

}