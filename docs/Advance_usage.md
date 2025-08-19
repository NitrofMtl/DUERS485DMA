# ⚡ Advanced Usage — DUERS485DMA

This document covers advanced features of the **DUERS485DMA** library that go beyond the basic `begin()`, `write()`, and `read()` usage.  

---

## 1. Custom RS485 Object Names
You can alias any enabled RS485 instance to a custom name for convenience:

```cpp
#define RS485 RS485_SERIAL3
#include <DUERS485DMA.h>

void setup() {
  RS485.begin(115200);
}
```

## 2. Fine-tuning DE/RE Timing

Some RS485 devices require precise timing for the driver enable (DE) and receiver enable (RE) pins.
You can adjust the delays as follows:
```cpp
#define BAUDRATE 9600

void setup() {
  RS485.begin(BAUDRATE);
  RS485.setDelays(
    10,                              // preDelay in µs
    1000000 / BAUDRATE * 15          // postDelay ≈ 1.5 character times
  );
}
```

## 3. Using Multiple Ports

You can enable and run multiple USART ports simultaneously:
```cpp
#define USE_RS485_SERIAL1
#define USE_RS485_SERIAL3
#include <DUERS485DMA.h>

void setup() {
  RS485_SERIAL1.begin(19200);
  RS485_SERIAL3.begin(38400);
}
```

## 4. Mixing With Standard Serial

DUERS485DMA only takes ownership of the ports you enable.
Other serial ports remain available for debugging or communication:
```cpp
#define USE_RS485_SERIAL1
#include <DUERS485DMA.h>

void setup() {
  Serial2.begin(115200);   // Debug output
  RS485_SERIAL1.begin(9600); // RS485 communication
}
```

## 5. Reading and Debugging

You can still use the standard Arduino available() and read() APIs:
```cpp
if (RS485.available()) {
  int b = RS485.read();
  Serial.print("RX: ");
  Serial.println(b, HEX);
}
```

## 6. Integration With ArduinoModbus

⚠️ Requires a patched ArduinoModbus library to recognize DUERS485DMA.

When enabled with:
```cpp
#define USE_DUERS485DMA
```

the ModbusMaster/ModbusSlave APIs can transparently use DMA-backed RS485 instances.

Details and examples are provided in the ArduinoModbus fork documentation.

## 7. Performance Notes

DMA offloads transmission from the CPU, freeing cycles for other tasks.

RX still uses interrupts, optimized for low latency.

Recommended to test pre/post delays for reliability in noisy environments.