#include "Arduino.h"
#include <SPI.h>
#include "SX126x.h"

SX126x::SX126x() {
}

int16_t SX126x::begin(int nss, int nRst, int busy) {
  _NSS       = nss;
  _RESET      = nRst;
  _BUSY       = busy;
  txActive    = false;

  pinMode(_NSS, OUTPUT);
  pinMode(_RESET, OUTPUT);
  pinMode(_BUSY, INPUT);

  SPI.begin();  
  //SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0));
}


int8_t SX126x::configure(uint8_t packetType, uint32_t frequencyInHz, int8_t txPowerInDbm, uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate, uint16_t preambleLength, uint8_t payloadLen, bool crcOn, bool invertIrq) {
  if ( txPowerInDbm > 22 ) txPowerInDbm = 22;
  if ( txPowerInDbm < -3 ) txPowerInDbm = -3;
  
  reset();

  if (getStatus() != SX126X_STATUS_IDLE) return 0;
  setStandby(SX126X_STANDBY_RC);

  setDio3AsTcxoCtrl(SX126X_DIO3_OUTPUT_3_3, RADIO_TCXO_SETUP_TIME << 6); // convert from ms to SX126x time base

  calibrateParameter(  SX126X_CALIBRATE_IMAGE_ON
                       | SX126X_CALIBRATE_ADC_BULK_P_ON
                       | SX126X_CALIBRATE_ADC_BULK_N_ON
                       | SX126X_CALIBRATE_ADC_PULSE_ON
                       | SX126X_CALIBRATE_PLL_ON
                       | SX126X_CALIBRATE_RC13M_ON
                       | SX126X_CALIBRATE_RC64K_ON
                    );

  setDio2AsRfSwitchCtrl(true);

  setStandby(SX126X_STANDBY_RC);
  setRegulatorMode(SX126X_REGULATOR_DC_DC);
  setBufferBaseAddress(0, 0);
  setPaConfig(0x04, SX126X_PA_CONFIG_HP_MAX, 0x00, 0x01);
  // setOvercurrentProtection(0x38);  // current max 30mA for the whole device
  setPowerConfig(txPowerInDbm, SX126X_PA_RAMP_10U);
  setDioIrqParams(SX126X_IRQ_ALL,  //all interrupts enabled
                  (SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT), //interrupts on DIO1
                  SX126X_IRQ_NONE,  //interrupts on DIO2
                  SX126X_IRQ_NONE); //interrupts on DIO3

  setRfFrequency(frequencyInHz);

  uint8_t ldro; //LowDataRateOptimize

  setStopRxTimerOnPreambleDetect(false);
  setLoRaSymbNumTimeout(0);
  setPacketType(SX126X_PACKET_TYPE_LORA); //RadioSetModem( ( SX126x.ModulationParams.PacketType == PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
  setModulationParams(spreadingFactor, bandwidth, codingRate, ldro);

  packetParams[0] = (preambleLength >> 8) & 0xFF;
  packetParams[1] = preambleLength;
  if ( payloadLen ) {
    //fixed payload length
    packetParams[2] = 0x01;
    packetParams[3] = payloadLen;
  } else {
    packetParams[2] = 0x00;
    packetParams[3] = 0xFF;
  }

  if ( crcOn ) packetParams[4] = 0x01;
  else packetParams[4] = 0x00;

  if (invertIrq) packetParams[5] = 0x01;
  else packetParams[5] = 0x00;

  writeCommand(SX126X_CMD_SET_PACKET_PARAMS, packetParams, 6);
  setDioIrqParams(SX126X_IRQ_ALL,  //all interrupts enabled
                  (SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE, SX126X_IRQ_TIMEOUT), //interrupts on DIO1
                  SX126X_IRQ_NONE,  //interrupts on DIO2
                  SX126X_IRQ_NONE);
  //receive state no receive timeoout
  setRx(0xFFFFFF);
  return 1;
}


uint8_t SX126x::receivePacket(uint8_t *buff, uint16_t len) {
  uint8_t rxLen = 0;
  uint16_t irqRegs = getIrqStatus();

  if ( irqRegs & SX126X_IRQ_RX_DONE ) {
    clearIrqStatus(SX126X_IRQ_RX_DONE);
    readBuffer(buff, &rxLen, len);
  }

  return rxLen;
}


bool SX126x::sendPacket(uint8_t *buff, uint8_t len, uint8_t mode) {
  uint16_t irq;
  bool rv = false;

  if ( txActive == false ) {
    txActive = true;
    packetParams[2] = 0x00; //Variable length packet (explicit header)
    packetParams[3] = len;
    writeCommand(SX126X_CMD_SET_PACKET_PARAMS, packetParams, 6);

    clearIrqStatus(SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT);

    writeBuffer(buff, len);
    setTx(500);

    if ( mode & SX126x_TXMODE_SYNC ) {
      irq = getIrqStatus();
      while ( (!(irq & SX126X_IRQ_TX_DONE)) && (!(irq & SX126X_IRQ_TIMEOUT)) ) {
        irq = getIrqStatus();
      }
      txActive = false;

      setRx(0xFFFFFF);

      if ( irq != SX126X_IRQ_TIMEOUT ) rv = true;
    } else {
      rv = true;
    }
  }
  return rv;
}


bool SX126x::receiveMode(void) {
  uint16_t irq;
  bool rv = false;

  if ( txActive == false )  {
    rv = true;
  } else {
    irq = getIrqStatus();
    if ( irq & (SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT) ) {
      setRx(0xFFFFFF);
      txActive = false;
      rv = true;
    }
  }

  return rv;
}


int8_t SX126x::getRssi() {  
  uint8_t status[5];

  readCommand(SX126X_CMD_GET_PACKET_STATUS, status, 5);
  int8_t _packetRSSI = -status[0] / 2;
  int8_t _packetSNR;

  if ( status[1] < 128 ) _packetSNR = status[1] / 4;
  else _packetSNR = (( status[1] - 256 ) / 4);
  return _packetRSSI;
}


void SX126x::setTxPower(int8_t txPowerInDbm) {
  setPowerConfig(txPowerInDbm, SX126X_PA_RAMP_200U);
}


void SX126x::sleep(uint8_t config) {  
  digitalWrite(_NSS, LOW);
  
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0));
  SPI.transfer(SX126X_CMD_SET_SLEEP);
  SPI.transfer(config);
  SPI.endTransaction();
  
  digitalWrite(_NSS, HIGH);  
  delay(5);
}

void SX126x::wakeup(void) {
  digitalWrite(_NSS, LOW); 
  getStatus();
  digitalWrite(_NSS, HIGH); 
  /*
  
  digitalWrite(_NSS, LOW);
  delay(1);
  digitalWrite(_NSS, HIGH);
  delay(1);
  while (digitalRead(_BUSY));
  */
}


void SX126x::reset(void) {
  delay(10);
  digitalWrite(_RESET, LOW);
  delay(20);
  digitalWrite(_RESET, HIGH);
  delay(10);
  while (digitalRead(_BUSY));
}

//----------------------------------------------------------------------------------------------------------------------------
//  The command setStandby(...) is used to set the device in a configuration mode which is at an intermediate level of
//  consumption. In this mode, the chip is placed in halt mode waiting for instructions via SPI. This mode is dedicated to chip
//  configuration using high level commands such as setPacketType(...).
//  By default, after battery insertion or reset operation (pin NRESET goes low), the chip will enter in STDBY_RC mode running
//  with a 13 MHz RC clock
//
//  Parameters
//  ----------
//  0: Device running on RC13M, set STDBY_RC mode
//  1: Device running on XTAL 32MHz, set STDBY_XOSC mode
//
//  Return value
//  ------------
//  none
//
//----------------------------------------------------------------------------------------------------------------------------
void SX126x::setStandby(uint8_t mode) {
  uint8_t data = mode;
  writeCommand(SX126X_CMD_SET_STANDBY, &data, 1);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The host can retrieve chip status directly through the command getStatus() : this command can be issued at any time and
//  the device returns the status of the device. The command getStatus() is not strictly necessary since device returns status
//  information also on command bytes.
//
//  Parameters:
//  none
//
//  Return value:
//  Bit 6:4 Chipmode:0x0: Unused
//  Bit 3:1 Command Status
//  Bit 0: unused
//  Bit 7: unused
//----------------------------------------------------------------------------------------------------------------------------
uint8_t SX126x::getStatus(void) {
  uint8_t rv;
  readCommand(SX126X_CMD_GET_STATUS, &rv, 1);
  return rv;
}

//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//
//----------------------------------------------------------------------------------------------------------------------------
void SX126x::setDio3AsTcxoCtrl(uint8_t tcxoVoltage, uint32_t timeout) {
  uint8_t buf[4];

  buf[0] = tcxoVoltage & 0x07;
  buf[1] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
  buf[2] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
  buf[3] = ( uint8_t )( timeout & 0xFF );

  writeCommand(SX126X_CMD_SET_DIO3_AS_TCXO_CTRL, buf, 4);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//
//----------------------------------------------------------------------------------------------------------------------------
void SX126x::calibrateParameter(uint8_t calibParam) {
  uint8_t data = calibParam;
  writeCommand(SX126X_CMD_CALIBRATE, &data, 1);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//
//----------------------------------------------------------------------------------------------------------------------------
void SX126x::setDio2AsRfSwitchCtrl(uint8_t enable) {
  uint8_t data = enable;
  writeCommand(SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, &data, 1);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//
//----------------------------------------------------------------------------------------------------------------------------
void SX126x::setRfFrequency(uint32_t frequency) {
  uint8_t buf[4];
  uint32_t freq = 0;

  calibrateImage(frequency);

  freq = (uint32_t)((double)frequency / (double)FREQ_STEP);
  buf[0] = (uint8_t)((freq >> 24) & 0xFF);
  buf[1] = (uint8_t)((freq >> 16) & 0xFF);
  buf[2] = (uint8_t)((freq >> 8) & 0xFF);
  buf[3] = (uint8_t)(freq & 0xFF);
  writeCommand(SX126X_CMD_SET_RF_FREQUENCY, buf, 4);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//
//----------------------------------------------------------------------------------------------------------------------------
void SX126x::calibrateImage(uint32_t frequency) {
  uint8_t calFreq[2];

  if ( frequency > 900000000 )   {
    calFreq[0] = 0xE1;
    calFreq[1] = 0xE9;
  } else if ( frequency > 850000000 ) {
    calFreq[0] = 0xD7;
    calFreq[1] = 0xD8;
  } else if ( frequency > 770000000 ) {
    calFreq[0] = 0xC1;
    calFreq[1] = 0xC5;
  } else if ( frequency > 460000000 ) {
    calFreq[0] = 0x75;
    calFreq[1] = 0x81;
  } else if ( frequency > 425000000 ) {
    calFreq[0] = 0x6B;
    calFreq[1] = 0x6F;
  }
  writeCommand(SX126X_CMD_CALIBRATE_IMAGE, calFreq, 2);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//
//----------------------------------------------------------------------------------------------------------------------------
void SX126x::setRegulatorMode(uint8_t mode) {
  uint8_t data = mode;
  writeCommand(SX126X_CMD_SET_REGULATOR_MODE, &data, 1);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//
//----------------------------------------------------------------------------------------------------------------------------
void SX126x::setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress) {
  uint8_t buf[2];

  buf[0] = txBaseAddress;
  buf[1] = rxBaseAddress;
  writeCommand(SX126X_CMD_SET_BUFFER_BASE_ADDRESS, buf, 2);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//
//----------------------------------------------------------------------------------------------------------------------------
void SX126x::setPowerConfig(int8_t power, uint8_t rampTime) {
  uint8_t buf[2];

  if ( power > 22 ) {
    power = 22;
  } else if ( power < -3 ) {
    power = -3;
  }

  buf[0] = power;
  buf[1] = ( uint8_t )rampTime;
  writeCommand(SX126X_CMD_SET_TX_PARAMS, buf, 2);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//
//----------------------------------------------------------------------------------------------------------------------------
void SX126x::setPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut) {
  uint8_t buf[4];

  buf[0] = paDutyCycle;
  buf[1] = hpMax;
  buf[2] = deviceSel;
  buf[3] = paLut;
  writeCommand(SX126X_CMD_SET_PA_CONFIG, buf, 4);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The OCP is configurable by steps of 2.5 mA and the default value is re-configured automatically each time the function
//  setPaConfig(...) is called. If the user wants to adjust the OCP value, it is necessary to change the register as a second
//  step after calling the function setPaConfig.
//
//  Parameters:
//  value: steps of 2,5mA (0x18 = 60mA)
//
//
//  Return value:
//  none
//
//----------------------------------------------------------------------------------------------------------------------------
void SX126x::setOvercurrentProtection(uint8_t value) {
  uint8_t buf[3];

  buf[0] = ((SX126X_REG_OCP_CONFIGURATION & 0xFF00) >> 8);
  buf[1] = (SX126X_REG_OCP_CONFIGURATION & 0x00FF);
  buf[2] = value;
  writeCommand(SX126X_CMD_WRITE_REGISTER, buf, 3);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//
//----------------------------------------------------------------------------------------------------------------------------
void SX126x::setDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask ) {
  uint8_t buf[8];

  buf[0] = (uint8_t)((irqMask >> 8) & 0x00FF);
  buf[1] = (uint8_t)(irqMask & 0x00FF);
  buf[2] = (uint8_t)((dio1Mask >> 8) & 0x00FF);
  buf[3] = (uint8_t)(dio1Mask & 0x00FF);
  buf[4] = (uint8_t)((dio2Mask >> 8) & 0x00FF);
  buf[5] = (uint8_t)(dio2Mask & 0x00FF);
  buf[6] = (uint8_t)((dio3Mask >> 8) & 0x00FF);
  buf[7] = (uint8_t)(dio3Mask & 0x00FF);
  writeCommand(SX126X_CMD_SET_DIO_IRQ_PARAMS, buf, 8);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//
//----------------------------------------------------------------------------------------------------------------------------
void SX126x::setStopRxTimerOnPreambleDetect( bool enable ) {
  uint8_t data = (uint8_t)enable;
  writeCommand(SX126X_CMD_STOP_TIMER_ON_PREAMBLE, &data, 1);
}


//----------------------------------------------------------------------------------------------------------------------------
//  In LoRa mode, when going into Rx, the modem will lock as soon as a LoRa® symbol has been detected which may lead to
//  false detection. This phenomena is quite rare but nevertheless possible. To avoid this, the command
//  setLoRaSymbNumTimeout can be used to define the number of symbols which will be used to validate the correct
//  reception of a packet.
//
//  Parameters:
//  0:      validate the reception as soon as a LoRa® Symbol has been detected
//  1..255: When SymbNum is different from 0, the modem will wait for a total of SymbNum LoRa® symbol to validate, or not, the
//          correct detection of a LoRa packet. If the various states of the demodulator are not locked at this moment, the radio will
//          generate the RxTimeout IRQ.
//
//
//  Return value:
//  none
//
//----------------------------------------------------------------------------------------------------------------------------
void SX126x::setLoRaSymbNumTimeout(uint8_t SymbNum) {
  uint8_t data = SymbNum;
  writeCommand(SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT, &data, 1);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//
//----------------------------------------------------------------------------------------------------------------------------
void SX126x::setPacketType(uint8_t packetType) {
  uint8_t data = packetType;
  writeCommand(SX126X_CMD_SET_PACKET_TYPE, &data, 1);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//
//----------------------------------------------------------------------------------------------------------------------------
void SX126x::setModulationParams(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate, uint8_t lowDataRateOptimize) {
  uint8_t data[4];
  //currently only LoRa supported
  data[0] = spreadingFactor;
  data[1] = bandwidth;
  data[2] = codingRate;
  data[3] = lowDataRateOptimize;
  writeCommand(SX126X_CMD_SET_MODULATION_PARAMS, data, 4);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//
//----------------------------------------------------------------------------------------------------------------------------
uint16_t SX126x::getIrqStatus( void ) {
  uint8_t data[2];
  readCommand(SX126X_CMD_GET_IRQ_STATUS, data, 2);
  return (data[0] << 8) | data[1];
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//
//----------------------------------------------------------------------------------------------------------------------------
void SX126x::clearIrqStatus(uint16_t irq) {
  uint8_t buf[2];

  buf[0] = (uint8_t)(((uint16_t)irq >> 8) & 0x00FF);
  buf[1] = (uint8_t)((uint16_t)irq & 0x00FF);
  writeCommand(SX126X_CMD_CLEAR_IRQ_STATUS, buf, 2);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//
//----------------------------------------------------------------------------------------------------------------------------
void SX126x::setRx(uint32_t timeout) {
  uint8_t buf[3];

  buf[0] = (uint8_t)((timeout >> 16) & 0xFF);
  buf[1] = (uint8_t)((timeout >> 8) & 0xFF);
  buf[2] = (uint8_t )(timeout & 0xFF);
  writeCommand(SX126X_CMD_SET_RX, buf, 3);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command setTx() sets the device in transmit mode. When the last bit of the packet has been sent, an IRQ TX_DONE
//  is generated. A TIMEOUT IRQ is triggered if the TX_DONE IRQ is not generated within the given timeout period.
//  The chip goes back to STBY_RC mode after a TIMEOUT IRQ or a TX_DONE IRQ.
//  he timeout duration can be computed with the formula: Timeout duration = Timeout * 15.625 μs
//
//  Parameters:
//  0: Timeout disable, Tx Single mode, the device will stay in TX Mode until the packet is transmitted
//  other: Timeout in milliseconds, timeout active, the device remains in TX mode. The maximum timeout is then 262 s.
//
//
//
//  Return value:
//  none
//
//----------------------------------------------------------------------------------------------------------------------------
void SX126x::setTx(uint32_t timeoutInMs) {
  uint8_t buf[3];
  uint32_t tout = (uint32_t)(timeoutInMs * 0, 015625);
  buf[0] = (uint8_t)((tout >> 16) & 0xFF);
  buf[1] = (uint8_t)((tout >> 8) & 0xFF);
  buf[2] = (uint8_t )(tout & 0xFF);
  writeCommand(SX126X_CMD_SET_TX, buf, 3);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//
//----------------------------------------------------------------------------------------------------------------------------
void SX126x::getRxBufferStatus(uint8_t *payloadLength, uint8_t *rxStartBufferPointer) {
  uint8_t buf[2];

  readCommand( SX126X_CMD_GET_RX_BUFFER_STATUS, buf, 2 );

  *payloadLength = buf[0];
  *rxStartBufferPointer = buf[1];
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//
//----------------------------------------------------------------------------------------------------------------------------
uint8_t SX126x::readBuffer(uint8_t *rxData, uint8_t *rxDataLen,  uint8_t maxLen) {
  uint8_t offset = 0;

  getRxBufferStatus(rxDataLen, &offset);
  if ( *rxDataLen > maxLen ) return 1;

  while (digitalRead(_BUSY));

  digitalWrite(_NSS, LOW);
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0));
  SPI.transfer(SX126X_CMD_READ_BUFFER);
  SPI.transfer(offset);
  SPI.transfer(SX126X_CMD_NOP);
  for ( uint16_t i = 0; i < *rxDataLen; i++ ) {
    rxData[i] = SPI.transfer(SX126X_CMD_NOP);
  }
  SPI.endTransaction();
  digitalWrite(_NSS, HIGH);

  while (digitalRead(_BUSY));

  return 0;
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//
//----------------------------------------------------------------------------------------------------------------------------
uint8_t SX126x::writeBuffer(uint8_t *txData, uint8_t txDataLen) {
  //Serial.print("SPI write: CMD=0x");
  //Serial.print(SX126X_CMD_WRITE_BUFFER, HEX);
  //Serial.print(" DataOut: ");
  digitalWrite(_NSS, LOW);
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0));
  SPI.transfer(SX126X_CMD_WRITE_BUFFER);
  SPI.transfer(0); //offset in tx fifo
  //Serial.print(" 0 ");
  for ( uint16_t i = 0; i < txDataLen; i++ ) {
    //Serial.print(txData[i]);
    //Serial.print(" ");
    SPI.transfer( txData[i]);
  }
  SPI.endTransaction();
  digitalWrite(_NSS, HIGH);
  //Serial.println("");
  while (digitalRead(_BUSY));

  return 0;
}


void SX126x::writeCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes, bool waitForBusy) {
  spiTranscept(cmd, true, data, NULL, numBytes, waitForBusy);
}


void SX126x::readCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes, bool waitForBusy) {
  spiTranscept(cmd, false, NULL, data, numBytes, waitForBusy);
}


void SX126x::spiTranscept(uint8_t cmd, bool write, uint8_t* dataOut, uint8_t* dataIn, uint8_t numBytes, bool waitForBusy) {

  // ensure BUSY is low (state meachine ready)
  // TODO timeout
  while (digitalRead(_BUSY));

  // start transfer
  digitalWrite(_NSS, LOW);
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0));

  // send command byte
  SPI.transfer(cmd);

  // send/receive all bytes
  if (write) {
    //Serial.print("SPI write: CMD=0x");
    //Serial.print(cmd, HEX);
    //Serial.print(" DataOut: ");
    for (uint8_t n = 0; n < numBytes; n++) {
      uint8_t in = SPI.transfer(dataOut[n]);
      //Serial.print(dataOut[n], HEX);
      //Serial.print(" ");
    }
    //Serial.println();
  } else {
    //Serial.print("SPI read:  CMD=0x");
    //Serial.print(cmd, HEX);
    // skip the first byte for read-type commands (status-only)
    uint8_t in = SPI.transfer(SX126X_CMD_NOP);
    ////Serial.println((SX126X_CMD_NOP, HEX));
    //Serial.print(" DataIn: ");

    for (uint8_t n = 0; n < numBytes; n++) {
      dataIn[n] = SPI.transfer(SX126X_CMD_NOP);
      ////Serial.println((SX126X_CMD_NOP, HEX));
      //Serial.print(dataIn[n], HEX);
      //Serial.print(" ");
    }
    //Serial.println();
  }

  // stop transfer
  SPI.endTransaction();
  digitalWrite(_NSS, HIGH);

  // wait for BUSY to go high and then low
  // TODO timeout
  if (waitForBusy) {
    delayMicroseconds(1);
    while (digitalRead(_BUSY));
  }
}
