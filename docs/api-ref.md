# 📖 API Reference — DUERS485DMA

This is the reference for all public classes, methods, macros, and predefined instances provided by the **DUERS485DMA** library.

---

## Classes

### `DUERS485DMAClass`
A drop-in replacement for `ArduinoRS485Class` with DMA support.

#### Methods
- **`bool begin(long baudrate, uint16_t config = SERIAL_8N1)`**  
  Initialize RS485 port with given baudrate and UART config.

- **`void end()`**  
  Stop RS485.

- **`bool beginTransmission()`**  
  Puts driver into transmit mode, enables DE.

- **`bool endTransmission()`**  
  Ends transmit, returns to receive mode.

- **`int available()`**  
  Number of bytes available to read.

- **`int read()`**  
  Read one byte (returns `-1` if none).

- **`size_t write(uint8_t b)`**  
  Send one byte.

- **`size_t write(const uint8_t* buffer, size_t size)`**  
  Send multiple bytes.

- **`void setDelays(unsigned preDelayUs, unsigned postDelayUs)`**  
  Configure DE/RE timing around transmissions.  
  - `preDelayUs`: microseconds before DE goes high  
  - `postDelayUs`: microseconds after last byte before DE goes low

- **`void flush()`**  
  Wait until all data is sent.

- **`peek()`**  
Look at the next byte in the receive buffer without removing it.

- **`setPins(int txPin, int dePin, int rePin)`**  

    Configures the TX, DE, and RE pins.

    Only the DE pin is actually used.

    txPin and rePin exist only for compatibility with the ArduinoRS485 API.

    If your RS485 transceiver requires RE control, tie RE and DE together. 

- **`receive()`**  

    Forces the interface into receive-only mode (DE low, RE enabled).

- **`noReceive()`**  

    Disables reception (ignores incoming data).

- **`setTxTimeout(int timeout)`**  

    Sets the maximum time (ms) allowed for a transmission.

    If DMA or hardware gets stuck, transmission will abort after this timeout.

    A safe starting value is ~10× the time required to send the largest expected frame.

- **`setRxIdleTime(float charMultiple = DEFAULT_CHAR_TIME)`**  

    Sets the idle time (in multiples of character duration) used to detect end-of-frame when receiving.

    Default = DEFAULT_CHAR_TIME (≈3.5 chars for Modbus RTU).

    Increase this if your bus has long idle gaps or slow turnaround.

- **`sendBreak(unsigned int duration)`**  
- **`sendBreakMicroseconds(unsigned int duration)`**  

    Forces the line low for a defined time, to send a break condition.

    duration in milliseconds or microseconds.

    Used mainly in special protocols or sync patterns.

- **`onTxComplete(TxCompleteCallback cb)`**  

    Registers a callback function that is called when transmission is complete (after DMA or interrupt).

- **`handleTxComplete()`**  

    Processes TX completion and executes the user callback (if any).

    Called automatically in endTransmission().

    May also be invoked manually from an ISR for asynchronous use.
    (Advanced usage only.)

---
## ⚡ Tip for tuning setTxTimeout() and setRxIdleTime():

- To avoid “phantom timeouts,” set setTxTimeout() slightly higher than the longest valid frame time.

- For Modbus RTU, use setRxIdleTime(3.5) (the protocol’s inter-frame delay requirement).
---

## Predefined Instances

Depending on enabled macros:

- **`RS485_SERIAL1`** → Uses `Serial1` (USART0)  
- **`RS485_SERIAL2`** → Uses `Serial2` (USART1)  
- **`RS485_SERIAL3`** → Uses `Serial3` (USART3)  
- **`RS485_USART2`**  → Uses `USART2` (extra UART, not mapped to Serial)

---

## Configuration Macros

- **`USE_DUERS485DMA`** → enable DMA backend  
- **`USE_RS485_SERIAL1`** → enable `RS485_SERIAL1` instance  
- **`USE_RS485_SERIAL2`** → enable `RS485_SERIAL2` instance  
- **`USE_RS485_SERIAL3`** → enable `RS485_SERIAL3` instance  
- **`USE_RS485_USART2`**  → enable `RS485_USART2` instance  

Optional alias:
```cpp
#define RS485 RS485_SERIAL1
```

## Notes:

- When enabling RS485 on a given port, the default SerialX object for that port is disabled to avoid conflicts.

- Always define configuration macros before including the library.

- Pre/post delays are critical for Modbus timing — tune them according to your baudrate and bus hardware.

## 🔹 Example Workflow
```cpp
#define USE_DUERS485DMA
#define USE_RS485_SERIAL1
#include <DUERS485DMA.h>

void setup() {
  RS485.begin(9600);
  RS485.setDelays(50, 1000000 / 9600 * 15);
}

void loop() {
  RS485.beginTransmission();
  RS485.write("Ping");
  RS485.endTransmission();

  delay(1000);

  if (RS485.available()) {
    SerialUSB.write(RS485.read());
  }
}
```

🔹 Advanced Notes — TX Handling

- Internally, DUERS485DMA uses endTransmission() to close a TX operation and automatically handle the DE/RE pins.

- If you need more control (e.g., when integrating with another library such as ArduinoModbus), you can use the lower-level API:

**`handleTxComplete()`**

Internal function called when a DMA TX completes. Normally you don’t need this.
Only useful if integrating with an API that manages TX completion externally.

**`onTxCallback(void (*callback)())`**

Register a user callback that will be invoked when a transmission is finished.
This is useful when implementing protocols that require precise timing between frames (like Modbus).

```cpp
void txDone() {
  digitalWrite(LED_BUILTIN, HIGH);
}

void setup() {
  RS485.begin(9600);
  RS485.onTxCallback(txDone);
}
```

⚠️ Not all APIs/libraries call onTxCallback(). It’s provided for compatibility if higher-level stacks support it.
