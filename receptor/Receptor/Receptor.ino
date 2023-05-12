//Receptor

#include <SPI.h>
#include <nRF24L01.h>  //Librería para el modulo RF RF24L01
#include <RF24.h>
// #include "UART.h"  //Librería para la comunicación serial

#define PIN_CE 8    // Se ha habilita el pin 9
#define PIN_CSN 10  //

const unsigned char direccionReceptor[5] = { 'T', 'L', 'M', 'T', 'R' };
const unsigned char direccionTransmisor[5] = { 'T', 'L', 'M', 'T', 'T' };

RF24 recepcionDatos(PIN_CE, PIN_CSN);

uint8_t contTrama = 0;


bool recibido = false;
bool modoRx = true;
float datosRecibidos[8];
float auxRecibidos[8];
unsigned long tiempoAnterior = 0;
float verifFrame[4] = { 1234.88, 4321.88, 6789.99, 9876.99 };
//


void setup() {

  Serial.begin(115200);

  if (recepcionDatos.begin() != NULL) {
    // Serial.println("RECEPTOR INICIALIZADO\n");
    // Serial.println("|Lat      |Lng       |aX        |aY        |aZ        |gX        |gY        |gZ        |mX        |mY        |mZ        |Otro     |");
    recepcionDatos.setDataRate(RF24_250KBPS);              //Se define la velocidad de recepcion de datos
    recepcionDatos.openReadingPipe(1, direccionReceptor);  //Se definine canal y dirección para transmisión
    recepcionDatos.startListening();                       //Se establece el módulo como receptor
    // Serial.println("Escuchando canal: ");
  } else Serial.println("Error en la inicialización del módulo");

  // recepcionDatos.begin();
  recepcionDatos.setDataRate(RF24_250KBPS);
  recepcionDatos.setRetries(3, 5);
  recepcionDatos.openWritingPipe(direccionTransmisor);  //Se define canal para tansmisión
}

void loop() {
  // modoRx = false;
  if (modoRx) {
    if (millis() - tiempoAnterior > 1000) {
      obtenerDatos();
      // mostrarDatos();
      tiempoAnterior = millis();
    }
  } else {
    enviarDatos();
  }
}


void obtenerDatos() {
  while (1) {
    if (recepcionDatos.available()) {
      contTrama++;
      recepcionDatos.read(datosRecibidos, sizeof(datosRecibidos));
      if (contTrama == 1) recibido = true;
      else recibido = false;
      mostrarDatos();
      if(contTrama == 2) break;
    } else break;
  }
}

void mostrarDatos() {

  Serial.print("verifi ");
  Serial.println(verifFrame[0]);
  Serial.print("recibi ");
  Serial.println(datosRecibidos[0]);
  Serial.print("verifi 1 ");
  Serial.println(verifFrame[1]);
  Serial.print("recibi 1");
  Serial.println(datosRecibidos[7]);

  if (recibido && datosRecibidos[0] == verifFrame[0] && datosRecibidos[7] == verifFrame[1]) {
    // auxRecibidos[0] = datosRecibidos[0];
    // auxRecibidos[7] = datosRecibidos[7];
    for (int i = 0; i < sizeof(datosRecibidos) / 4; i++) {
      // if( i>0 && <7) auxRecibidos[i] = datosRecibidos[i];
      auxRecibidos[i] = datosRecibidos[i];
    }
  }
  if (contTrama == 2 && datosRecibidos[0] == verifFrame[2] && datosRecibidos[7] == verifFrame[3]) {
    for (int j = 0; j < sizeof(datosRecibidos) / 4 -2; j++) {
      Serial.print(auxRecibidos[j+1], 6);
      Serial.print(",");
    }
    for (int k = 0;k < sizeof(datosRecibidos) / 4 -2; k++) {
      Serial.print(datosRecibidos[k+1], 6);
      Serial.print(",");
    }

    Serial.println();
    delay(500);
    recepcionDatos.stopListening();
    modoRx = false;
    contTrama = 0;
  }
}

void enviarDatos() {

  uint8_t pwm[4];
  pwm[0] = random(100);
  pwm[1] = random(100);
  pwm[2] = random(100);
  pwm[3] = random(100);


  Serial.print(pwm[0]);
  Serial.print(pwm[1]);
  Serial.print(pwm[2]);
  Serial.print(pwm[3]);
  Serial.println();
  recepcionDatos.write(&pwm, sizeof(pwm));

  delay(100);
  modoRx = true;
  recepcionDatos.startListening();
}