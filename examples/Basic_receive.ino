// Uncomment the port you want to use BEFORE including DUERS485DMA
//#define USE_RS485_SERIAL1
//#define USE_RS485_SERIAL2
//#define USE_RS485_SERIAL3
//#define USE_RS485_USART2

#define USE_RS485_SERIAL1
#define RS485 RS485_SERIAL1
#include <DUERS485DMA.h>

#define BAUDRATE 9600

void setup() {
  RS485.begin(BAUDRATE);
}

void loop() {
  if (RS485.available()) {
    int b = RS485.read();
    SerialUSB.write(b); // echo to USB console
  }
}
