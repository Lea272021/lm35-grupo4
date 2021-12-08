#include <Arduino.h>
//#include "esp_adc_cal.h"
#include <WiFi.h>
#include <MQTT.h>
#include <stdlib.h>

// Red
const char ssid[] = "";
const char pass[] = "";

WiFiClient net;
MQTTClient client;

#define cantMuestras 20

// FUNCIONES
void leerTemperatura(void);
void estadoLed(void);
void apagarLeds(void);
void limpiarVariables(void);
void publicar(void);
void pulsadorDeCorte(void);
void connect(void);
// uint32_t readADC_Cal(int ADC_Raw);
float getTemp(void);

// Pines de los Leds
const int8_t ledInterno = 2;
const int8_t ledRojo = 33, ledAmarillo = 32, ledVerde = 25, button = 27, sensor = 13;

// Variables de temperatura
float temperatura;

// Variable global String
String ESTADO;
String mensaje;

// Timer incial
unsigned long tiempoInicio = 0;
unsigned long timerDelay = 500;

// Variables auxiliares
static int8_t pulsador = 0, cont = 0;
static float suma = 0.0, promTemperatura;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(ledInterno, OUTPUT); // Se utilizo para visualizar a simple vista si el pulsador fue accionado o si hay rebotes
  pinMode(ledRojo, OUTPUT);
  pinMode(ledVerde, OUTPUT);
  pinMode(ledAmarillo, OUTPUT);
  pinMode(sensor, INPUT);
  pinMode(button, INPUT_PULLUP); // Al accionarse el pulsador se recibe un cero
  WiFi.begin(ssid, pass);        // NOMBRE DE LA RED WIFI Y CONTRASEÑA
  client.begin("192.168.1.40", 1883, net);
  // client.onMessage(messageReceived);
  connect();
  srand(time(NULL));
}

void loop()
{
  client.loop(); // Si o si tiene que estar Actualiza el estado de mi cliente MQTT
  if (!client.connected())
  { // Me aseguro que este conectado
    connect();
  }
  while (digitalRead(button) == 1)
    ; // Cuando el pulsador es accionado sale del bucle while
  pulsador = !digitalRead(button);
  delay(300);
  while (pulsador == 1)
  {
    digitalWrite(ledInterno, HIGH);
    leerTemperatura();
    cont++;
    // cantMuestras es un valor de la cantidad de veces que se sumara la temperatura y se hara el promedio para obtener el valor de temperatura final
    while (cont == cantMuestras)
    {
      if ((millis() - tiempoInicio) == timerDelay || tiempoInicio == 0) // Se coloca la condicion "|| tiempoInicio==0" para que entre en el IF en la primer ejecucion del programa
      {                                                                 // Cada 5 seg(5000ms) se muestra el valor de temperatura

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
void leerTemperatura()
{
  // float AnalogLM35, Voltage;
  // Capturo datos de los 3 sensores
  // temperatura=(analogRead(sensor)*(5.0/4096))/0.01; //Sensor 1 de referencia
  /*AnalogLM35 = analogRead(sensor);
  Voltage = readADC_Cal(AnalogLM35);
  temperatura = (Voltage / 10) - 3.8; //3.8*/
  // Se realiza el promedio de las lecturas de los sensores
  temperatura = getTemp();
  suma = suma + temperatura;
  promTemperatura = suma / cantMuestras; // El ultimo valor se imprime
}
//__________FUNCION DE LECTURA Y CALIBRACION DEL ADC_________
// uint32_t readADC_Cal(int ADC_Raw)
//{
// esp_adc_cal_characteristics_t adc_chars;

// esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
// return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
//}
//*****DETECTA MEDIDAS FUERA DEL RANGO DE TEMP PERMITIDO, GENERANDO ESTADO DE ALERTA*******
void estadoLed()
{
  // Estado normal
  if (promTemperatura <= 25)
  {
    digitalWrite(ledRojo, LOW);
    digitalWrite(ledVerde, HIGH);
    digitalWrite(ledAmarillo, LOW);
    // Serial.print("Normal");
    char estado1[7] = "NORMAL";
    ESTADO = String(estado1);
    Serial.println();
  }
  // Estado alerta
  if (promTemperatura > 25 && promTemperatura <= 30)
  {
    digitalWrite(ledVerde, LOW);
    digitalWrite(ledRojo, LOW);
    digitalWrite(ledAmarillo, HIGH);
    // Serial.print("Precaución");
    char estado2[11] = "PRECAUCION";
    ESTADO = String(estado2);
    Serial.println();
  }
  // Estado critico
  if (promTemperatura >= 30)
  {
    digitalWrite(ledVerde, LOW);
    digitalWrite(ledRojo, HIGH);
    digitalWrite(ledAmarillo, LOW);
    // Serial.print("Critico");
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
  String mensaje = "{\"topico1\":" + ESTADO + ",\"topico2\":" + String(promTemperatura, 1) + "}";
  client.publish("Temp", mensaje);
}
//______________CONEXION__________________
void connect()
{
  // Conectar wifi
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
  // Conectar al brocker MQTT
  Serial.print("\nconnecting...");
  while (!client.connect("arduino"))
  { //, "public", "public")) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nconnected!");
}

float getTemp()
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