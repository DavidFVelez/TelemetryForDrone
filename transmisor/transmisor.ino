// Transmisor

// Librerias estandar C
#include <stdio.h>
#include <stdint.h>
// Librerías para modulo IMU
//  #include "I2Cdev.h"
#include "MPU9250.h"
#include "Wire.h"
// Librerías GPS
#include <TinyGPS++.h>
// #include <SoftwareSerial.h>
// #include "UART.h"
// Librerías para módulo NRF24
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Declaraciones para la IMU

// Declaraciones NRF24 transmisor
#define PERIODO_MUESTREO_NRF24 1000
#define PIN_CE 8    // Pin para habilitar el modulo NRF24
#define PIN_CSN 53  // Pin para seleccionar el dispositivo NRF24 durante la comunicación SPI.
float gpsLatitud = 0;
float gpsLongitud = 0;
float sumaDatos = 0.0; 

// SoftwareSerial ss(18, pTX);

const unsigned char direccionReceptor[5] = { 'T', 'L', 'M', 'T', 'R' };
const unsigned char direccionTransmisor[5] = { 'T', 'L', 'M', 'T', 'T' };
RF24 transmisionDatos(PIN_CE, PIN_CSN);                                  // Se activa la comunicación con el MCU y se selecciona la interfaz SPI
float paquete[8] = { 23.44, 45.6, 12.2, 73.4, 85.6, 92.2, 42.3, 35.7 };  // Valores float a enviar
float paquete2[8] = { 11.2, 33.4, 55.6, 77.8, 91.1, 21.3, 47.5, 68.3 };

unsigned long tiempoAnterior;
float verifFrame[4] = { 1234.88, 4321.88, 6789.99, 9876.99 };


TinyGPSPlus gps;

void setup() {

  //Protocolo numero flotante conocido al final y al inicio paquete para comprobar valides de los paquetes
  // además para ordenar la trama en el receptor
  paquete[0] = verifFrame[0];
  paquete[7] = verifFrame[1];
  paquete2[0] = verifFrame[2];
  paquete2[7] = verifFrame[3];
 
  Serial.begin(115200);
  Serial1.begin(9600);

  // Setup para GPS
  //  UART_begin();
  //  ss.begin(baudGPS);
  //  uartDriverInit(pRX, pTX, baudGPS);
  //  sei();

  // Setup para el NRF24
  if (transmisionDatos.begin() != NULL) {
    Serial.println("TRANSMISOR INICIALIZADO\n");
    Serial.println("Orden de valores para localización y telemtría ");
    Serial.println("|Lat      |Lng       |aX        |aY        |aZ        |gX        |gY        |gZ        |mX        |mY        |mZ        |Otro     |");
    transmisionDatos.setDataRate(RF24_250KBPS);
    transmisionDatos.setRetries(3, 5);
    transmisionDatos.openWritingPipe(direccionReceptor);
  } else
    Serial.println("Error en la inicialización del módulo\n");

  // transmisionDatos.begin();
  transmisionDatos.setDataRate(RF24_250KBPS);
  transmisionDatos.openReadingPipe(1, direccionTransmisor);
  // transmisionDatos.stopListening();
  transmisionDatos.startListening();
}

void loop() {

  // Configuración para el modulo GPS
  while (Serial1.available() > 0) {
    // while (UART_read()> 0) {
    // while (uartDriverAvailable()){

    if (gps.encode(Serial1.read()))
      // if (gps.encode(UART_read()))

      // if (gps.encode(uartDriverRead()))
      localizacion();
  }
  // if (millis() > 5000 && gps.charsProcessed() < 10) {
  //   Serial.println(F("No GPS detected: check wiring."));
  //   while (true)
  //     ;
  // }

  // Configuración para el modulo NRF24

  if (millis() - tiempoAnterior >= PERIODO_MUESTREO_NRF24) {
    transmitir();
    tiempoAnterior = millis();
  }

  if (transmisionDatos.available()) {

    recibir();
  }

  // else {
  //   Serial.println( " no recibio ");
  // }

  // Configuración para el módulo
  // Configuración de la IMU
  // Configuración del módulo
}

// FUNCIONES PARA GPS
void localizacion() {
  // Serial.print(("Location: "));
  if (gps.location.isValid()) {

    // Guardado de datos de localización en el paquete2 a transmitir 
    paquete2[4] = gps.location.lat();
    paquete2[5] = gps.location.lng();
    Serial.print(paquete[0], 6);
    Serial.print(",");
    // Serial.println(paquete[1], 6);
  } else {
    Serial.println(F("Esperando GPS"));
  }
}

// FUNCIONES PARA NRF24
void transmitir() {

  bool frame1;
  bool frame2;
  delay(500);
  transmisionDatos.stopListening();

  //Para el protocolo de comunicación se crea un chekSum, y el valor se ingresa
  // en el paquete2 en la posicion [6]. Este valor debe coincidir en el receptor
  for(int i = 0; i<(sizeof(paquete) / 4 + sizeof(paquete2) / 4); i ++){
    sumaDatos += paquete[i] + paquete2[i]; 
  }
  paquete2[6] = sumaDatos;


  frame1 = transmisionDatos.write(paquete, sizeof(paquete));
  if (frame1)
    Serial.println();  // Serial.println("  Trama 1. Transmitida\n");
  else
    Serial.println("  Transmisión de la trama 1 falló\n");

  delay(1000);
  frame2 = transmisionDatos.write(paquete2, sizeof(paquete2));
  if (frame1 && frame2) {
    // Serial.println("  Trama 2. Transmitida\n");
    // Serial.println("Mensaje enviado: ");

    for (int i = 0; i < (sizeof(paquete) / 4 + sizeof(paquete2) / 4); i++) {
      if (i < sizeof(paquete) / 4) {
        Serial.print(paquete[i], 6);
        Serial.print(",");
        
      } else {
        Serial.print(paquete2[i - 6], 6);
        Serial.print(",");
        
      }
    }
    sumaDatos = 0; 
    // Serial.println();
  } else {
    Serial.println("  Transmisión del paquete 2 falló\n");
  }
  delay(100);
  transmisionDatos.startListening();
}

void recibir() {
  uint8_t pwm[4];
  transmisionDatos.read(pwm, sizeof(pwm));
  Serial.print(pwm[0]);
  Serial.print(pwm[1]);
  Serial.print(pwm[2]);
  Serial.println(pwm[3]);
}