// Librerias estandar C
#include <stdio.h>
#include <stdint.h>
// Librerías para modulo IMU
#include "imu_manager.h"  //para acelerómetro y giroscopio
#include "Wire.h"
#include "compass_manager.h"  //para el magnometro
// Librerías GPS
#include <TinyGPS++.h>
#include "UART.h"            //Driver creado para la comunicación serial con GPS
// Librerías para módulo NRF24
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Declaraciones NRF24 transmisor
#define PERIODO_MUESTREO_NRF24 1000
#define PIN_CE 8    // Pin para habilitar el modulo NRF24
#define PIN_CSN 53  // Pin para seleccionar el dispositivo NRF24 durante la comunicación SPI.

const unsigned char direccionReceptor[5] = { 'T', 'L', 'M', 'T', 'R' };    //Dirección de recepción
const unsigned char direccionTransmisor[5] = { 'T', 'L', 'M', 'T', 'T' };  //Dirección de transmisión
float gpsLatitud = 0;
float gpsLongitud = 0;
RF24 transmisionDatos(PIN_CE, PIN_CSN);                                  // Se activa la comunicación con el MCU y se selecciona la interfaz SPI
float paquete[8] = { 23.44, 45.6, 12.2, 73.4, 85.6, 92.2, 42.3, 35.7 };  // Valores float a enviar
float paquete2[8] = { 11.2, 33.4, 55.6, 77.8, 91.1, 21.3, 47.5, 68.3 };
unsigned long int tiempoAnterior;
float verifFrame[4] = { 1234.88, 4321.88, 6789.99, 9876.99 };  //Array de flotantes necesarios para enviar como protocolo de comunicación
TinyGPSPlus gps;

void localizacion();
void transmitir();
void recibir();
void delayMillis(unsigned long int time) ;

void setup() {
  Serial.begin(115200);
  UART_begin();
  //Se agrega a cada paquete a transmitir el protocolo de comunicación.Consiste en un número flotante conocido al final y al inicio paquete para comprobar valides de los paquetes
  //además para ordenar la trama en el receptor
  paquete[0] = verifFrame[0];
  paquete[7] = verifFrame[1];
  paquete2[0] = verifFrame[2];
  paquete2[7] = verifFrame[3];

  //Inicialización de la Imu
  ImuManagerSetup();
  startCalibrationProcess();
  //Inicialización de magnetometro
  compassManagerSetup();
  compassManagerCalibrate(10);

  // NRF24
  //Se verifica si el módulo se inicio correctamente
  if (transmisionDatos.begin() != NULL) {
    //Se imprime por serial para visualizar los datos a enviar
    Serial.println("TRANSMISOR INICIALIZADO\n");
    Serial.println("Orden de valores para localización y telemtría ");
    Serial.println("|IP1        |aX        |aY        |aZ        |gX        |gY        |gZ        |FP1        |IP2        |mX        |mY        |mZ        |DirMag     |Lat      |Lng       |FP2        ");
    transmisionDatos.setDataRate(RF24_250KBPS);           //Se estable velocidad de transmisión
    transmisionDatos.setRetries(3, 5);                    // Número de reintentos y de tiempo entre reintentos
    transmisionDatos.setPALevel(RF24_PA_MAX);             // Potencia máxima de transmisión
    transmisionDatos.openWritingPipe(direccionReceptor);  //Se habilita la escritura en el canal
  } else{
    Serial.println("Error en la inicialización del módulo\n");}  //Se reporta si el modulo no se inicio correctamente

  transmisionDatos.setDataRate(RF24_250KBPS);                   //Se establece velocidad de transmisión
  transmisionDatos.openReadingPipe(1, direccionTransmisor);     //Se abre la tubería de comunicación
  transmisionDatos.startListening();                            //Se inicia la lectura del canal
}

void loop() {

  //Módulo GPS
  //Mientras en la UART hayan datos disponibles se decofican y se llama a la función localización para procesarlos.
  while (UART_available()) {
    if (gps.encode(UART_receive())) {
      localizacion();
    }
  }

  //Para módulo NRF24
  // Se espera determinado tiempo para actualizar valores y transmitir paquetes
  if (millis() - tiempoAnterior >= PERIODO_MUESTREO_NRF24) {
    transmitir();
    tiempoAnterior = millis();
  }
  //Verificamos que la hay datos disponibles para leer, de cumplirse se llama a la función recibir para iniciar el proceso
  if (transmisionDatos.available()) {
    recibir();
  }
}

// Función decodificar los valores de latitud y longitud entregados por el gps. Además se guardan los valores en el paquete2 para ser transmitidos posteriormente
//Return Nada.
void localizacion() {
  if (gps.location.isValid()) {
    // Guardado de datos de localización en el paquete2 a transmitir
    paquete2[5] = gps.location.lat();
    paquete2[6] = gps.location.lng();
  } else {
    Serial.print('\n');
    Serial.println(F("Esperando GPS"));
  }
}

// Función para transmitir por el canal y dirección configurados previamente. En primer lugar se transmiten dos paquetes cada uno de 32 bytes para poder
//enviar todas las variables medidas y que se guardan en paquete y paquete2 respectivamente. En segundo lugar se
//deshabilita el canal de transmisión para proceder a habilitar el canal de recepción.

// return Nada.
void transmitir() {

  uint8_t frame1 = 0;
  uint8_t frame2 = 0;
  delayMillis(50);
  //Se deshabilita el modo de recepción
  transmisionDatos.stopListening();
  //Se escribe el paquete 1 en el canal
  frame1 = transmisionDatos.write(paquete, sizeof(paquete));
  if (frame1)
    ;
  else{
    //Si falla la transmisión del paquete se reporta por consola
  Serial.print('\n');
  Serial.println("  Transmisión de la trama 1 falló\n");
  }
  delayMillis(100);
  //Se escribe el paquete 2 en el canal
  frame2 = transmisionDatos.write(paquete2, sizeof(paquete2));
  if (frame1 && frame2) {
    //Actualización de aceleraciones y valores de giroscopio
    updateAccelerometerData();
    updateGyroscopeData();
    //Se almacena acelX
    paquete[1] = ImuManagerGetAccX();
    //Se almacena acelY
    paquete[2] = ImuManagerGetAccY();
    //Se almacena acelZ
    paquete[3] = ImuManagerGetAccZ();
    //Se almacena Pitch
    paquete[4] = ImuManagerGetPitnch();
    //Se almacena ROLL
    paquete[5] = ImuManagerGetRoll();
    //Se almacena YAW
    paquete[6] = ImuManagerGetYaw();
    //En el paquete2 se alamcena los valores en x,y,z del magnetrometro
    paquete2[1] = compassManagerXfield();
    paquete2[2] = compassManagerYfield();
    paquete2[3] = compassManagerZfield();
    //También se almacena la dirección calculada a partir del magnetometro
    paquete2[4] = compassManagerGetHeading();

    //Para verificar los datos que seran transmitidos se imprimen por consola
    for (int i = 0; i < (sizeof(paquete) / 4 + sizeof(paquete2) / 4); i++) {
      if (i < sizeof(paquete) / 4) {
        Serial.print(paquete[i], 6);
        Serial.print(",");
      } else {
        Serial.print(paquete2[i - 8], 6);
        Serial.print(",");
      }
    }

  } else {
    //Si falla la transmisión del paquete2 se reporta por consola
    Serial.print('\n');
    Serial.println("  Transmisión del paquete 2 falló\n");
  }
  //Cuando los 2 paquetes se han enviado se deshabilita la transmisión y se enciende la recepción de datos, para recibir las señales de control(PWM)
  delayMillis(10);
  transmisionDatos.startListening();
}

//Función encargada de recibir las señales de control(PWM1, PWM2, PWM3, PWM4) para los 4 motores()
//return Nada
void recibir() {
  //Se crea un array de 4 números enteros sin signo, en donde se almacenan las señales de control(PWM) que el receptor envía para los 4 motores
  uint8_t pwm[4];
  transmisionDatos.read(pwm, sizeof(pwm));

  if (pwm[0] > 0 && pwm[1] > 0 && pwm[2] > 0 && pwm[3] > 0) {
    //Se muestra el PW1 recibido Motor 1
    Serial.print('\n');
    Serial.print("pwmM1:");
    Serial.print(pwm[0]);
    //Se muestra el PW2 recibido Motor 2
    Serial.print(",pwmM2:");
    Serial.print(pwm[1]);
    //Se muestra el PW3 recibido para Motor 3
    Serial.print(",pwmM3:");
    Serial.print(pwm[2]);
    //Se muestra el PW4 recibido para Motor 4
    Serial.print(",pwmM4:");
    Serial.println(pwm[3]);
  }
}

//Función para generar tiempos de espera
void delayMillis(unsigned long int time) {
  unsigned long int beforeTime = millis();
  unsigned long int currentTime = millis();
  while (currentTime - beforeTime < time) {
    currentTime = millis();
  }
}