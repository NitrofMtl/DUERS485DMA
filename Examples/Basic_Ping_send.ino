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
  RS485.setDelays(50, 1000000/BAUDRATE*20);
}

void loop() {
  RS485.beginTransmission();
  RS485.write("Ping");
  RS485.endTransmission();
  delay(1000);
}
