#pragma once
#include <Arduino.h>

#ifndef ARDUINO_ARCH_SAM 
#ERROR "This library is only for SAM3x Arduino DUE"
#else

#ifndef USE_DUERS485DMA
#warning "Compiling without DMA support. Define USE_DUERS485DMA to enable it."
#endif

//For use with arduino IDE
//#define USE_RS485_SERIAL1 //uncomment to use Serial1 for RS485
//#define USE_RS485_SERIAL2 //uncomment to use Serial2 for RS485
//#define USE_RS485_SERIAL3 //uncomment to use Serial3 for RS485
//#define USE_RS485_USART2  //uncomment to use USART2 for RS485

struct DUERS485Configs {
    Usart*      usart;       // pointer to USART registers (e.g., USART0)
    IRQn_Type   irq;         // interrupt number (e.g., USART0_IRQn)
    uint32_t    peripheralId;// peripheral ID (e.g., ID_USART0)
    RingBuffer* rxBuffer;    // pointer to RX ring buffer
    RingBuffer* txBuffer;    // pointer to TX ring buffer
};

extern RingBuffer rx_buffer2;
extern RingBuffer rx_buffer3;
extern RingBuffer rx_buffer4;
extern RingBuffer rx_buffer5;
extern RingBuffer tx_buffer2;
extern RingBuffer tx_buffer3;
extern RingBuffer tx_buffer4;
extern RingBuffer tx_buffer5;


constexpr DUERS485Configs RS485_SERIAL1_CONFIGS { USART0, USART0_IRQn, ID_USART0, &rx_buffer2, &tx_buffer2 };
constexpr DUERS485Configs RS485_SERIAL2_CONFIGS { USART1, USART1_IRQn, ID_USART1, &rx_buffer3, &tx_buffer3 };
constexpr DUERS485Configs RS485_SERIAL3_CONFIGS { USART3, USART3_IRQn, ID_USART3, &rx_buffer4, &tx_buffer4 };
constexpr DUERS485Configs RS485_USART2_CONFIGS { USART2, USART2_IRQn, ID_USART2, &rx_buffer5, &tx_buffer5 };

constexpr float DEFAULT_CHAR_TIME = 3.5f;

typedef void (*TxCompleteCallback)();

class DUERS485DMAClass : public UARTClass {
public:
    DUERS485DMAClass(DUERS485Configs cfg);

    void begin(unsigned long baudrate) override;
    void begin(unsigned long baudrate, int predelay, int postdelay);
    void begin(unsigned long baudrate, UARTClass::UARTModes config);
    void begin(unsigned long baudrate, UARTClass::UARTModes config, int predelay, int postdelay);
    void end();

    int available() override;
    int read() override;
    int peek() override;
    void flush() override;
    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    size_t readBytes(uint8_t *buffer, size_t length);
    using Print::write; // pull in write(str) and write(buf, size) from Print
    inline bool beginTransmission()
    {
        unsigned long start = micros();
        while (!_endRX) {
            if (micros() - start > _txTimeoutUs){
                return false; // RX not done
            } 
            yield();
        }

        if (_dePin >= 0) {
            digitalWrite(_dePin, HIGH);
            if (_preDelay > 0) delayMicroseconds(_preDelay);
        }
        _transmitting = true;
        return true;
    };
    inline bool endTransmission()
    {
        unsigned long start = micros();  
        // blocking until DMA done
        while (_txBusy && (micros() - start < _txTimeoutUs)) yield();

        if (_txBusy) return false;
        flush();
        if (_postDelay > 0) delayMicroseconds(_postDelay);
        handleTxComplete();
        return true;
    };
    inline void handleTxComplete()
    {
        if (_dePin >= 0) {
                digitalWrite(_dePin, LOW);
            }
            _transmitting = false;
    };

    
    void setPins(int txPin, int dePin, int rePin);
    void setDelays(int predelay, int postdelay);
    void setTxTimeout(int timeout);
    void setRXIdleTime(float charMultiple = DEFAULT_CHAR_TIME);

    void sendBreak(unsigned int duration);
    void sendBreakMicroseconds(unsigned int duration);

    
    void onTxComplete(TxCompleteCallback cb);
    
    void receive();
    void noReceive();
    void IrqHandler();

private:
    
    void configurePins();
    inline void updateRXBuffer();
    inline void triggerDMATXFromBuffer();
    uint32_t ring_buffer_size(const RingBuffer* rb) const;

    static const uint32_t DMA_RX_BUFFER_SIZE = 128;
    
    __attribute__((aligned(4))) volatile uint8_t _dma_rx_buffer[DMA_RX_BUFFER_SIZE];
    Usart* _pUsart;
    // Called inside TX complete interrupt context — keep handlers short & non-blocking
    TxCompleteCallback _txCallback = nullptr;
    volatile uint32_t _dma_rx_pos = 0;
    volatile uint32_t _txLen = 0;
    // End of RX flag: true when DMA RX has completed or timeout triggered
    volatile bool _endRX = true;
    // Transmission busy flag: true when DMA TX in progress
    volatile bool _txBusy = false;
    
    bool _transmitting = false;
    int _dePin = -1;
    uint32_t _preDelay = 0;
    uint32_t _postDelay = 0;
    uint32_t _txTimeoutUs = 0;
};

#ifdef USE_RS485_SERIAL1
// Completely disable Serial1
#ifdef Serial1
#undef Serial1
#endif
#define Serial1 __error__You_cannot_use_Serial1_with_RS485_DMA

extern void serialEvent1();
extern DUERS485DMAClass RS485_SERIAL1;
#endif

#ifdef USE_RS485_SERIAL2
// Completely disable Serial2
#ifdef Serial2
#undef Serial2
#endif
#define Serial2 __error__You_cannot_use_Serial2_with_RS485_DMA

extern void serialEvent2();
extern DUERS485DMAClass RS485_SERIAL2;
#endif

#ifdef USE_RS485_SERIAL3
// Completely disable Serial3
#ifdef Serial3
#undef Serial3
#endif
#define Serial3 __error__You_cannot_use_Serial3_with_RS485_DMA

extern void serialEvent3();
extern DUERS485DMAClass RS485_SERIAL3;
#endif

#ifdef USE_RS485_USART2
extern DUERS485DMAClass RS485_USART2;
#endif

#endif //ARDUINO_ARCH_SAM
