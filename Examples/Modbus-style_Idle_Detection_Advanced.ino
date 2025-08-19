// Uncomment the port you want to use BEFORE including DUERS485DMA
//#define USE_RS485_SERIAL1
//#define USE_RS485_SERIAL2
//#define USE_RS485_SERIAL3
//#define USE_RS485_USART2

#define USE_DUERS485DMA
#define USE_RS485_USART2
#define RS485 RS485_USART2
#define BAUDRATE 19200

#include <DUERS485DMA.h>

void setup() {
  Serial.begin(115200);
  RS485.begin(BAUDRATE);
  RS485.setRXIdleTime(3.5f); // ~3.5 chars (Modbus RTU spec)
}

void loop() {
  while (RS485.available()) {
    Serial.write(RS485.read());
  }
}
