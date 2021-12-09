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
float leerTemperatura(void);
void estadoLed(void);
void apagarLeds(void);
void publicar(void);
void pulsadorDeCorte(void);
void connect(void);



// Pines de los Leds
const int8_t ledInterno = 2;
const int8_t ledRojo = 33, ledAmarillo = 32, ledVerde = 25, button = 27;

// Variables de temperatura
float temperatura;
float temp;

//Variable global String
String ESTADO;
String mensaje;

//Timer incial
unsigned long tiempoInicio = 0;
unsigned long timerDelay = 500;;

// Variables auxiliares
static int8_t pulsador = 0;


void setup() {
   
  Serial.begin(115200);
  pinMode(ledInterno, OUTPUT); // Se utilizo para visualizar a simple vista si el pulsador fue accionado o si hay rebotes
  pinMode(ledRojo, OUTPUT);
  pinMode(ledVerde, OUTPUT);
  pinMode(ledAmarillo, OUTPUT);
  pinMode(button, INPUT_PULLUP); //Al accionarse el pulsador se recibe un cero

  WiFi.begin(ssid, pass);   //NOMBRE DE LA RED WIFI Y CONTRASEÑA
  client.begin("192.168.11.170", 1883, net);
  connect();

  srand(time(NULL));

}

void loop()
{

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

    // Se coloca la condicion "|| tiempoInicio==0" para que entre en el IF en la primer ejecucion del programa
    // Cada cierto tiempo timerDelay
      if ((millis() - tiempoInicio) == timerDelay || tiempoInicio == 0)
      { 
        temp=LeerTemperatura();
        Serial.print(temp);
        Serial.println();
        estadoLed();
        publicar();
        tiempoInicio = millis();
        
      }
      pulsadorDeCorte();
    
  }
  
  apagarLeds();
  delay(500);

}


//*****DETECTA MEDIDAS FUERA DEL RANGO DE TEMP PERMITIDO, GENERANDO ESTADO DE ALERTA*******
void estadoLed()
{
  // Estado normal
  if (temp <= 25)
  {
    digitalWrite(ledRojo, LOW);
    digitalWrite(ledVerde, HIGH);
    digitalWrite(ledAmarillo, LOW);
    ESTADO = "NORMAL";
    Serial.println();
  }
  // Estado alerta
  if (temp > 25 && temp <= 30)
  {
    digitalWrite(ledVerde, LOW);
    digitalWrite(ledRojo, LOW);
    digitalWrite(ledAmarillo, HIGH);
    ESTADO="PRECAUCION";
    Serial.println();
  }
  // Estado critico
  if (temp >= 30)
  {
    digitalWrite(ledVerde, LOW);
    digitalWrite(ledRojo, HIGH);
    digitalWrite(ledAmarillo, LOW);
    ESTADO="CRITICO";
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

//Valores de temperatura
float LeerTemperatura()
{
  float vect[20] = {10.2,
                    13.2,
                    19.2,
                    25.3,
                    33.8,
                    27.9,
                    40.3,
                    31.5,
                    42.9,
                    57.6,
                    24.5,
                    31.3,
                    28.9,
                    33.3,
                    38.8,
                    26.4,
                    42.9,
                    21.5,
                    40.9,
                    61.6};
  static int i = 0;
  if (i < 19)
  {
    i++;
  }
  else
  {
    i = 0;
  }
  float aux = vect[i];
  return aux;
}

//---SE PRESIONA POR SEGUNDA VEZ EL BOTON Y SALE DEL BUCLE WHILE------
void pulsadorDeCorte()
{
  if (digitalRead(button) == 0)
  {
    pulsador = 0;
    tiempoInicio = 0;
   
  }
}

//______________PUBLICAR______________
void publicar()
{
  //Se envia el estado de temperatura y el valor de temperatura
  String mensaje = "{\"topico1\": \" " + ESTADO + " \" ,\"topico2\":" + String(temp) + "}";
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
