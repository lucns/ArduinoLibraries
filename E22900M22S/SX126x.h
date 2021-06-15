#include "Definitions.h"
#include <stdint.h>

class SX126x {
  public:
    SX126x();

    int16_t begin(int nss, int nRst, int busy);
    int8_t  configure(uint8_t packetType, uint32_t frequencyInHz, int8_t txPowerInDbm, uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate, uint16_t preambleLength, uint8_t payloadLen, bool crcOn, bool invertIrq);
    bool    checkDevice();
    uint8_t receivePacket(uint8_t *buff, uint16_t len);
    bool    sendPacket(uint8_t *buff, uint8_t len, uint8_t mode = SX126x_TXMODE_SYNC);
    bool    receiveMode(void);
    int8_t  getRssi();
    void    setTxPower(int8_t txPowerInDbm);
    void    sleep(uint8_t config);
    void    wakeup();

  private:
    uint8_t packetParams[6];
    bool    txActive;

    int     _NSS;
    int     _RESET;
    int     _BUSY;

    void    writeCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes, bool waitForBusy = true);
    void    readCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes, bool waitForBusy = true);
    void    SPItransfer(uint8_t cmd, bool write, uint8_t* dataOut, uint8_t* dataIn, uint8_t numBytes, bool waitForBusy);

    void    setDio3AsTcxoCtrl(uint8_t tcxoVoltage, uint32_t timeout);
    void    setDio2AsRfSwitchCtrl(uint8_t enable);
    void    reset(void);
    uint8_t getStatus(void);
    void    setStandby(uint8_t mode);
    void    waitOnBusy(void);
    void    setRfFrequency(uint32_t frequency);
    void    calibrateParameter(uint8_t calibParam);
    void    calibrateImage(uint32_t frequency);
    void    setRegulatorMode(uint8_t mode);
    void    setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress);
    void    setPowerConfig(int8_t power, uint8_t rampTime);
    void    setOvercurrentProtection(uint8_t value);
    void    setPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut);
    void    setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask);
    void    setStopRxTimerOnPreambleDetect(bool enable);
    void    setLoRaSymbNumTimeout(uint8_t SymbNum);
    void    setPacketType(uint8_t packetType);
    void    setModulationParams(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate, uint8_t lowDataRateOptimize);
    uint16_t getIrqStatus(void);
    void    clearIrqStatus(uint16_t irq);
    void    setRx(uint32_t timeout);
    void    setTx(uint32_t timeoutInMs);
    void    getRxBufferStatus(uint8_t *payloadLength, uint8_t *rxStartBufferPointer);
    uint8_t readBuffer(uint8_t *rxData, uint8_t *rxDataLen,  uint8_t maxLen);
    uint8_t writeBuffer(uint8_t *txData, uint8_t txDataLen);
    void    spiTranscept(uint8_t cmd, bool write, uint8_t* dataOut, uint8_t* dataIn, uint8_t numBytes, bool waitForBusy);
};
