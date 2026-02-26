#include "DUERS485DMA.h"


DUERS485DMAClass::DUERS485DMAClass(DUERS485Configs cfg)
    //: UARTClass((Uart*)pUsart, dwIrq, dwId, pRx_buffer, pTx_buffer),
    : UARTClass((Uart*)cfg.usart, cfg.irq, cfg.peripheralId, cfg.rxBuffer, cfg.txBuffer),
    _pUsart(cfg.usart)
{
    if (!cfg.usart || !cfg.rxBuffer || !cfg.txBuffer) {
        Serial.println("ERROR: DUERS485DMAClass received null pointer!");
        while (1); // halt MCU, optional for debug
    }
}


void DUERS485DMAClass::configurePins()
{
    switch (_dwId) {
        case ID_USART0:
            // RXD0: PA10, TXD0: PA11, RTS0: PB25 (optional)
            PIOA->PIO_PDR = PIO_PA10 | PIO_PA11;
            PIOA->PIO_ABSR &= ~(PIO_PA10 | PIO_PA11);  // Peripheral A
            PIOA->PIO_PUDR = PIO_PA10 | PIO_PA11;      // Disable pull-ups
            break;

        case ID_USART1:
            // RXD1: PA12, TXD1: PA13
            PIOA->PIO_PDR = PIO_PA12 | PIO_PA13;
            PIOA->PIO_ABSR &= ~(PIO_PA12 | PIO_PA13);  // Peripheral A
            PIOA->PIO_PUDR = PIO_PA12 | PIO_PA13;      // Disable pull-ups
            break;

        case ID_USART2:
            // RXD2: PB21, TXD2: PB20
            PIOB->PIO_PDR = PIO_PB21 | PIO_PB20;
            PIOB->PIO_ABSR &= ~(PIO_PB21 | PIO_PB20);  // Peripheral A
            PIOB->PIO_PUDR = PIO_PB21 | PIO_PB20;      // Disable pull-ups
            break;

        case ID_USART3:
            // RXD3: PD5, TXD3: PD4
            PIOD->PIO_PDR = PIO_PD5 | PIO_PD4;
            PIOD->PIO_ABSR &= ~(PIO_PD5 | PIO_PD4);    // Peripheral A
            PIOD->PIO_PUDR = PIO_PD5 | PIO_PD4;        // Disable pull-ups
            break;

        default:
            // Optionally fail here or leave pins untouched
            break;
    }
}


void DUERS485DMAClass::begin(unsigned long baudrate)
{
    begin(baudrate, SERIAL_8N1, 0, 0);
}


void DUERS485DMAClass::begin(unsigned long baudrate, int predelay, int postdelay)
{
    begin(baudrate, SERIAL_8N1, predelay, postdelay);
}


void DUERS485DMAClass::begin(unsigned long baudrate, UARTClass::UARTModes config)
{
    begin(baudrate, config, 0, 0);
}


void DUERS485DMAClass::begin(unsigned long baudrate, UARTClass::UARTModes config, int predelay, int postdelay)
{
    _preDelay = predelay;
    _postDelay = postdelay;
    _txTimeoutGuardUs = (uint32_t)((10UL * SERIAL_BUFFER_SIZE * 1000000UL) / baudrate);
    _txTimeoutGuardUs = _txTimeoutGuardUs + _txTimeoutGuardUs / 5;  // Add ~20% margin

    configurePins();

    PMC->PMC_PCER0 = (1 << _dwId);
    _pUsart->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS;
    _pUsart->US_BRGR = SystemCoreClock / (16 * baudrate);
    _pUsart->US_MR = config | US_MR_USCLKS_MCK | US_MR_CHMODE_NORMAL;
    _pUsart->US_CR = US_CR_RXEN | US_CR_TXEN;

    if (_dePin >= 0) {
        pinMode(_dePin, OUTPUT);
        digitalWrite(_dePin, LOW);
    }

    // Make sure both ring buffers are initialized back to empty.
    _rx_buffer->_iHead = _rx_buffer->_iTail = 0;
    _tx_buffer->_iHead = _tx_buffer->_iTail = 0;

    //set timeout rx
    setRxTimeout();
    uint32_t idleTime = getUsecForNChar(RS485DEFAULT_RX_IDLE_CHARS);
    setRXIdleTime(idleTime);

    receive();
    NVIC_EnableIRQ(_dwIrq);
}


//tx and re pins are here to match arduinoRS485 library
void DUERS485DMAClass::setPins(int txPin, int dePin, int rePin)
{
    _dePin = dePin;
    if (_dePin >= 0) {
        pinMode(_dePin, OUTPUT);
        digitalWrite(_dePin, LOW);
    }
}


void DUERS485DMAClass::setDelays(int predelay, int postdelay)
{
    _preDelay = predelay;
    _postDelay = postdelay;
}


void DUERS485DMAClass::setTxTimeoutGuard(int timeout)
{
    _txTimeoutGuardUs = timeout;
}


int DUERS485DMAClass::getBitsPerChar() const
{
    uint32_t mr = _pUsart->US_MR; // Mode Register

    // --------------------------
    // Data bits
    // CHRL bits (Character Length) 00=5 bits, 01=6 bits, 10=7 bits, 11=8 bits
    uint8_t chrl = (mr & US_MR_CHRL_Msk) >> US_MR_CHRL_Pos;
    uint8_t dataBits = chrl + 5; // since 0=>5 bits, 3=>8 bits

    // --------------------------
    // Parity bits
    uint8_t parityBits = 0;
    uint8_t par = (mr & US_MR_PAR_Msk) >> US_MR_PAR_Pos;
    if (par != US_MR_PAR_NO) parityBits = 1;

    // --------------------------
    // Stop bits
    uint8_t stopBits = 1;
    uint8_t nbstop = (mr & US_MR_NBSTOP_Msk) >> US_MR_NBSTOP_Pos;
    if (nbstop == US_MR_NBSTOP_1_5_BIT) stopBits = 2;  // treat 1.5 as 2
    else if (nbstop == US_MR_NBSTOP_2_BIT) stopBits = 2;

    // --------------------------
    // Bits per char
    return 1 + dataBits + parityBits + stopBits;
}


uint32_t DUERS485DMAClass::getUsecForNChar(float nChar)
{
    // Baud rate
    // On Due core, baud is stored in BRGR
    uint32_t cd = (_pUsart->US_BRGR & US_BRGR_CD_Msk) >> US_BRGR_CD_Pos;
    if (cd == 0) cd = 1; // avoid division by zero
    uint32_t baud = SystemCoreClock / (16 * cd);

    // Character time in microseconds
    float charTimeUs = (getBitsPerChar() * 1000000.0f) / baud;

    // time
    return (unsigned long)(charTimeUs * nChar);
}


void DUERS485DMAClass::setRXIdleTime(uint32_t idleTimeUs)
{
    uint32_t oneCharTimeUs = getUsecForNChar(1);
    _rxIdleTimeUs = (idleTimeUs > oneCharTimeUs) ? idleTimeUs-oneCharTimeUs : 0;
}


void DUERS485DMAClass::setRxTimeout()
{
    // RTOR counts in bit times, not microseconds
    int rtor = getBitsPerChar() +1; //set timeout 1 char time + 1 bit margin
    _pUsart->US_RTOR = US_RTOR_TO(rtor);
    _pUsart->US_CR = US_CR_STTTO; // restart timeout counter
}


void DUERS485DMAClass::end()
{
    _pUsart->US_CR = US_CR_RXDIS | US_CR_TXDIS;
    UARTClass::end();
}


void DUERS485DMAClass::flush()
{
   uint32_t start = micros();
   while (!_txPhysDone) { //wait usart to emty
    if (micros() - start > _txTimeoutGuardUs) {
        break;
    }
    yield();
   }
}


int DUERS485DMAClass::peek()
{
    updateRXBuffer();
    return UARTClass::peek();
}


int DUERS485DMAClass::available()
{
    updateRXBuffer();// Copy new DMA-received bytes to ring buffer
    return UARTClass::available();
}


int DUERS485DMAClass::read()
{
    //updateRXBuffer();// Copy new DMA-received bytes to ring buffer
    return UARTClass::read();
}


size_t DUERS485DMAClass::write(uint8_t c)
{
    size_t written = UARTClass::write(c);
    triggerDMATXFromBuffer();
    return written;
}


size_t DUERS485DMAClass::write(const uint8_t *buffer, size_t size)
{
    size_t written = 0;
    for (size_t i = 0; i < size; i++) {
        
        if (UARTClass::write(buffer[i])) {
            written++;
        } else {
            break;
        }
    }
    
    // Trigger DMA only once for entire string
    triggerDMATXFromBuffer();

    return written;
}


size_t DUERS485DMAClass::readBytes(uint8_t *buffer, size_t length)
{
    size_t count = 0;
    unsigned long start = millis();

    while (count < length) {
        //updateRXBuffer();

        size_t avail = available();
        if (avail) {
            size_t toRead = min(avail, length - count);

            size_t tail = _rx_buffer->_iTail;
            if (tail + toRead <= DMA_RX_BUFFER_SIZE) {
                memcpy(buffer + count, rxBufferPtr(tail), toRead);
            } else {
                size_t first = DMA_RX_BUFFER_SIZE - tail;
                memcpy(buffer + count, rxBufferPtr(tail), first);
                memcpy(buffer + count + first, rxBufferPtr(0), toRead - first);
            }

            _rx_buffer->_iTail = (tail + toRead) % DMA_RX_BUFFER_SIZE;
            count += toRead;
            start = millis();  // reset timeout on progress
        }

        if (millis() - start >= _timeout) {
            break;
        }
        yield();
    }
    return count;
}


size_t DUERS485DMAClass::readFrame(uint8_t *buffer, size_t length)
{ 
    if (!_frame.armed) return 0; // No complete frame available

    if (micros() - _frame.idleTimeStamp < _rxIdleTimeUs) return 0; // still in post-idle guard

    size_t len = min(length, _frame.len);

    size_t tail = (_frame.head + DMA_RX_BUFFER_SIZE - _frame.len) % DMA_RX_BUFFER_SIZE;

    if (len + tail > DMA_RX_BUFFER_SIZE) {
        // Wrap around case
        size_t firstPart = DMA_RX_BUFFER_SIZE - tail;
        memcpy(buffer, rxBufferPtr(tail), firstPart);
        memcpy(buffer + firstPart, rxBufferPtr(0), len - firstPart);
    } else {
        // Contiguous case
        memcpy(buffer, rxBufferPtr(tail), len);
    }
    _rx_buffer->_iTail = (_rx_buffer->_iTail + len) % DMA_RX_BUFFER_SIZE;
    _frame.len -= len;

     // If we've read the entire frame, disarm it
    if (_frame.len == 0) {
        _frame.armed = false;
        if (_frame.overflow) {
            _rx_buffer->_iTail = _rx_buffer->_iHead; // Drop remaining data in ring buffer
            _frame.overflow = false; // reset overflow flag for next frame
            Serial.println("[[[OVERFLOW!!!]]]");
        }
    }

    return len;
}


void DUERS485DMAClass::receive() {
    // --- Ensure receiver is stopped before reconfiguring ---
    _pUsart->US_PTCR = US_PTCR_RXTDIS;  // Disable DMA RX
    _pUsart->US_IDR  = US_IDR_TXRDY | US_IDR_TXEMPTY | US_IDR_TXBUFE; // Disable all TX interrupts
    _pUsart->US_CR   = US_CR_RXDIS;     // Disable USART RX 
        // Clear any pending error/status flags (stale RXRDY, etc.)
    _pUsart->US_CR = US_CR_RSTSTA;

    // Reset DMA buffer pointer & count
    _pUsart->US_RPR = (uint32_t)_dma_rx_buffer;
    _pUsart->US_RCR = DMA_RX_BUFFER_SIZE;
    _lastRcrSnapshot = _pUsart->US_RCR; //set for activity detection
    _dma_rx_pos = 0; // Reset our tracking index as well
    // Re-enable receiver
    _pUsart->US_CR = US_CR_RXEN;
    // Enable PDC (DMA) reception
    _pUsart->US_PTCR = US_PTCR_RXTEN;
    // Timeout setup
    _pUsart->US_CR = US_CR_STTTO;
    _pUsart->US_IER = US_IER_TIMEOUT;
}


void DUERS485DMAClass::noReceive() {
    uint32_t start = micros();
    while (!isRxIdle()){
        yield();
        if (micros() - start > RS485_RX_STALL_TIMEOUT_US) {
            break; // Exit if we've waited significantly past idle time
        }
    }

    _pUsart->US_PTCR = US_PTCR_RXTDIS; // Disable PDC (DMA) receiver
    _pUsart->US_CR = US_CR_RXDIS; // Disable USART RX
    _pUsart->US_IDR = US_IDR_ENDRX //disable interrupts
                    //| US_IDR_RXRDY
                    | US_IDR_TIMEOUT;

    // Clear any pending error/status flags
    _pUsart->US_CR = US_CR_RSTSTA;

    // NOTE: We intentionally do NOT reset _rx_buffer here,
    // so the caller can still read any pending bytes after stopping reception.
}


void DUERS485DMAClass::sendBreak(unsigned int duration)
{
    if (duration > 1000) duration = 1000; // Cap at 1 second break
    sendBreakMicroseconds(duration*1000);
}


void DUERS485DMAClass::sendBreakMicroseconds(unsigned int duration)
{
    beginTransmission();           // Assert DE pin
    _pUsart->US_CR = US_CR_STTBRK;
    delayMicroseconds(duration);  // Duration of break
    _pUsart->US_CR = US_CR_STPBRK;
    flush();                       // Wait until break completes
    endTransmission();            // De-assert DE pin
}


void DUERS485DMAClass::onTxComplete(TxCompleteCallback cb)
{
    _txCallback = cb;
}


bool DUERS485DMAClass::isRxIdle()
{
    if (rxActivity()) return false; // Not idle if RX not marked complete
    return micros() - _rxIdleStamp >= _rxIdleTimeUs;
}


inline uint32_t DUERS485DMAClass::ring_buffer_size(const RingBuffer* rb) const
{
    return (rb->_iHead + SERIAL_BUFFER_SIZE - rb->_iTail) % SERIAL_BUFFER_SIZE;
}


void DUERS485DMAClass::updateRXBuffer()
{
    // How many bytes DMA has written into _dma_rx_buffer so far
    uint32_t received = DMA_RX_BUFFER_SIZE - _pUsart->US_RCR;

    // Copy from last read position to current DMA write position
    unsigned int start = micros();
    while (_dma_rx_pos < received) {
        if(micros() - start > 10000) return;
        _rx_buffer->store_char(_dma_rx_buffer[_dma_rx_pos]);
        _dma_rx_pos++;
        
    }
}


void DUERS485DMAClass::triggerDMATXFromBuffer()
{
    if (_txBusy) return;

    uint32_t head = _tx_buffer->_iHead;
    uint32_t tail = _tx_buffer->_iTail;

    _txLen = (head >= tail) ? (head - tail) : (SERIAL_BUFFER_SIZE - tail);
    if (_txLen == 0) return;

    // Prepare linear section for DMA
    uint8_t* src = const_cast<uint8_t*>(&_tx_buffer->_aucBuffer[tail]);

    _txBusy = true;
    _txPhysDone = false;

    _pUsart->US_TPR = (uint32_t)src;
    _pUsart->US_TCR = _txLen;
    _pUsart->US_PTCR = US_PTCR_TXTEN;
    // Enable interruptS when buffer is empty and usart empty
    _pUsart->US_IER = US_IER_TXBUFE;// | US_CSR_TXEMPTY;
}


bool DUERS485DMAClass::rxActivity()
{
    return _lastRcrSnapshot != _pUsart->US_RCR;
}


void DUERS485DMAClass::IrqHandler()
{    
    uint32_t status = _pUsart->US_CSR;
    if ( _txBusy && (status & US_CSR_TXBUFE)) {
        //US_CSR_TXEMPTY
        _pUsart->US_IDR = US_IDR_TXBUFE;
        _tx_buffer->_iTail = (_tx_buffer->_iTail + _txLen) % SERIAL_BUFFER_SIZE;
        
        _txLen = 0;
        _txBusy = false;
        
        if (ring_buffer_size(_tx_buffer) == 0) {
            _pUsart->US_PTCR = US_PTCR_TXTDIS;
            _pUsart->US_TCR = 0;
            _pUsart->US_TPR = 0;
            // Kill all USART TX interrupts
            _pUsart->US_IDR = US_IDR_TXRDY | US_IDR_TXBUFE;
            _pUsart->US_IER = US_IER_TXEMPTY;

        } else {// Immediately restart if there’s more to send
            triggerDMATXFromBuffer();
        }
    }

    if (status & US_CSR_TXEMPTY) {
        _pUsart->US_IDR = US_IDR_TXEMPTY;

        _txPhysDone   = true;
        _txEndStamp   = micros();
        if (_txCallback) {
            _txCallback();
        }
    }

    if (status & US_CSR_TIMEOUT) {
        updateRXBuffer();  // Push remaining bytes
        onRxIdleIRQ();
    }
    
}


void DUERS485DMAClass::onRxIdleIRQ()
{
    _pUsart->US_CR = US_CR_STTTO; // Clear timeout condition (restart timer)
    _rxIdleStamp = micros(); // Update idle timestamp
    //_endRX = true;                // Mark RX as complete
    _lastRcrSnapshot = _pUsart->US_RCR; //set for activity detection

    if (_frame.armed && !( _rxIdleStamp - _frame.idleTimeStamp > _rxIdleTimeUs)) {
        _frame.overflow = true;
        return;
    }

    _frame.head = _rx_buffer->_iHead;
    size_t len = (_frame.head - _rx_buffer->_iTail + DMA_RX_BUFFER_SIZE) % DMA_RX_BUFFER_SIZE;
    if (len == 0) return;   // ignore spurious IDLE
    _frame.len = len;
    _frame.idleTimeStamp = _rxIdleStamp;
    _frame.armed = true;
}


//You must make the default USART1_Handler(void) --> __attribute__((weak)) void USART1_Handler(void) in
//platformIO:
//.pio/packages/framework-arduino-sam/variants/arduino_due_x/variant.cpp
//Arduino IDE 
//..\Arduino15\packages\arduino\hardware\sam\1.6.12\variants\arduino_due_x

#ifdef USE_RS485_SERIAL1

extern RingBuffer rx_buffer2;
extern RingBuffer tx_buffer2;

// DO NOT REMOVE — ties RS485 to USART0 IRQ
extern "C" void USART0_Handler(void)
{
    RS485_SERIAL1.IrqHandler();
}

// Override serialEvent1 to neutralize serialEventRun
void serialEvent1() {
  // Do nothing; prevents USART0 conflicts
}

DUERS485DMAClass RS485_SERIAL1(RS485_SERIAL1_CONFIGS);

#endif

#ifdef USE_RS485_SERIAL2

extern RingBuffer rx_buffer3;
extern RingBuffer tx_buffer3;

// DO NOT REMOVE — ties RS485 to USART1 IRQ
extern "C" void USART1_Handler(void)
{
    RS485_SERIAL2.IrqHandler();
}

// Override serialEvent2 to neutralize serialEventRun
void serialEvent2() {
  // Do nothing; prevents USART1 conflicts
}

DUERS485DMAClass RS485_SERIAL2(RS485_SERIAL2_CONFIGS);

#endif

#ifdef USE_RS485_SERIAL3

extern RingBuffer rx_buffer4;
extern RingBuffer tx_buffer4;

// DO NOT REMOVE — ties RS485 to USART3 IRQ
extern "C" void USART3_Handler(void)
{
    RS485_SERIAL3.IrqHandler();
}

// Override serialEvent3 to neutralize serialEventRun
void serialEvent3() {
  // Do nothing; prevents USART3 conflicts
}

DUERS485DMAClass RS485_SERIAL3(RS485_SERIAL3_CONFIGS);

#endif

#ifdef USE_RS485_USART2
RingBuffer rx_buffer5;
RingBuffer tx_buffer5;
DUERS485DMAClass RS485_USART2(RS485_USART2_CONFIGS);

extern "C" void USART2_Handler(void)
{
    RS485_USART2.IrqHandler();
}
#endif
