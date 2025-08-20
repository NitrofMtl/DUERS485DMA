[![Arduino Library](https://www.ardu-badge.com/badge/DUERS485DMA.svg?version=1.0.0)](https://www.ardu-badge.com/DUERS485DMA)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/yourname/library/DUERS485DMA.svg)](https://registry.platformio.org/libraries/yourname/DUERS485DMA)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE.md)

# DUERS485DMA Library

A high-performance **RS485 driver with DMA support** for the Arduino Due.  
Designed as a drop-in replacement for the standard [`ArduinoRS485`](https://www.arduino.cc/en/Reference/ArduinoRS485) library, with optional integration into [`ArduinoModbus`](https://www.arduino.cc/en/Reference/ArduinoModbus).  

## ✨ Features
- DMA-based transmit for minimal CPU usage  
- Compatible with ArduinoRS485 API  
- Works with ArduinoModbus (with a slightly modified version, via compile-time switch)  
- Automatic DE/RE pin handling  
- Drop-in replacement for existing RS485 sketches  

---

## 📦 ⚠️ Important installation note ⚠️

Until the [pull request](link-to-PR) to the official [ArduinoCore-sam](https://github.com/arduino/ArduinoCore-sam) is accepted, you must manually patch your Arduino core:

⚠️ Required Core Patch

To use DUERS485DMA, you must make the `USARTx_Handler(void)` weak so the library can override it.

This patch does **not** change the normal functionality of the core:

- If DUERS485DMA is not used, the original functions behave as before.  
- If DUERS485DMA is included, it overrides them where needed.  

You can choose to make all predefined USART `USARTx_Handler(void)` functions weak, or only the ones you intend to use.  

**Note:** `USART2_Handler(void)` was not predefined in the core, so no patch is needed there.

📂 File locations

Arduino IDE (Windows):
```cpp
C:\Users\<YourName>\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.12\variants\arduino_due_x\variant.cpp
```

PlatformIO:

Linux / macOS:
```cpp
~/.platformio/packages/framework-arduinosam/variants/arduino_due_x/variant.cpp
```

Windows:
```cpp
C:\Users\<YourName>\.platformio\packages\framework-arduinosam\variants\arduino_due_x\variant.cpp
```

## Locate your board’s variant.cpp file and edit the IrqHandler() definition:

Handlers defined in ArduinoCore-sam

In variant.cpp for the Arduino Due, you will find:
```cpp
void USART0_Handler(void) { Serial1.IrqHandler(); } //Serial1
void USART1_Handler(void) { Serial2.IrqHandler(); } //serial2
void USART3_Handler(void) { Serial3.IrqHandler(); } //Serial3
// Note: USART2_Handler is not defined in the core, nothing to do
```

Before
```cpp
void USART0_Handler(void) {
  // original IRQ handler code
}
```

After
```cpp
__attribute__((weak)) void USART0_Handler(void) {
  // original IRQ handler code
}
```

## 📦 Installation & Configuration

### Enable RS485 Ports
The Arduino Due has **4 USART ports** available for RS485.  
Each must be enabled with a macro before including the library:

```cpp
#define USE_RS485_SERIAL1 // enable RS485 on Serial1
#define USE_RS485_SERIAL2 // enable RS485 on Serial2
#define USE_RS485_SERIAL3 // enable RS485 on Serial3
#define USE_RS485_USART2  // enable RS485 on USART2
```
---

Arduino IDE

1. Download or clone this repository into your libraries/ folder.

2. Restart the Arduino IDE.

3. In your sketch, enable DMA mode and define which RS485 ports you want to use before including the library:
```cpp
#define USE_DUERS485DMA        // MUST be defined before inclusion
#define USE_RS485_SERIAL1      // enable RS485 on Serial1
#include <DUERS485DMA.h>
```

---

⚠️ Note: Enabling RS485 on a USART port will disable the standard SerialX object for that port to prevent misusage.

---

### PlatformIO
1. install the library:
```
pio pkg install -l nitrofmtl/duers485dma
```
2. Add to your platformio.ini
```
lib_deps = 
    yourname/DUERS485DMA
build_flags = -DUSE_DUERS485DMA, -DUSE_RS485_SERIAL1
```
3. include the library:
```
#include <DUERS485DMA.h>
```

This ensures the library is compiled with DMA enabled.

✅ After this, Arduino IDE and PlatformIO usage is identical.

### Optional: Rename RS485 Port

You can define a default alias for convenience:
```
#define RS485 RS485_SERIAL1 //use RS485 Serial1 as default RS485 name
```


## Simple RS485 DMA Example:
```
#define BAUDRATE 9600

void setup() {
  RS485.begin(BAUDRATE);
  //preDealy should be very shrot, postDelay is good set as one char (10 bits in SERIAL_8N1, default mode)
  RS485.setDelays(50, 1000000/BAUDRATE*20);

}

void loop() {
  RS485.beginTransmission();
  RS485.write("Ping");
  RS485.endTransmission();
  delay(1000);
}
```

## 🔗 Using with ArduinoModbus

`DUERS485DMA` can also be used with [ArduinoModbus](https://www.arduino.cc/en/Reference/ArduinoModbus).  
However, the official library does not currently support external RS485 backends.  

➡️ To use with DMA, clone and install this fork:  
HAVE TO CHANGE FOR BRANCH UPTADED [DUERS485DMA-Compatible ArduinoModbus](https://github.com/NitrofMtl/ArduinoModbusDUE)  

Usage example:  
```cpp
#define USE_DUERS485DMA
#define USE_RS485_SERIAL1
#include <DUERS485DMA.h>
#include <ArduinoModbus.h>

ModbusRTUClientClass ModbusClient(RS485_SERIAL1);

void setup() {
  RS485.begin(9600);
  ModbusClient.begin(1, 9600); // nodeId = 1
}
```
