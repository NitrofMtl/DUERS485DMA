// Uncomment the port you want to use BEFORE including DUERS485DMA
//#define USE_RS485_SERIAL1
//#define USE_RS485_SERIAL2
//#define USE_RS485_SERIAL3
//#define USE_RS485_USART2

#define USE_DUERS485DMA
#define USE_RS485_SERIAL3
#define RS485 RS485_SERIAL3
#define BAUDRATE 19200

#include <DUERS485DMA.h>

void onTxDone() {
  Serial.println("TX finished!");
}

void setup() {
  Serial.begin(115200);
  RS485.begin(BAUDRATE);
  RS485.onTxComplete(onTxDone);
}

void loop() {
  RS485.beginTransmission();
  RS485.print("Data frame");
  RS485.endTransmission();
  delay(2000);
}
