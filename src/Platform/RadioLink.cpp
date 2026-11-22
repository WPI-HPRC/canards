#if defined(ARDUINO)

#include "Platform/RadioLink.h"

#include <HardwareSerial.h>

#include "RadioConfigs.h"

namespace {

HardwareSerial gRadioSerial(RADIO_SERIAL_RX, RADIO_SERIAL_TX);
const char* const kCallsign = "KV0R";
LoRaE22 gRadio(&gRadioSerial, RADIO_M0, RADIO_M1, RADIO_AUX, kCallsign);

// The E22's baud rates follow no pattern, hence the switch.
bool changeSerialPortConfig(RadioConfigTypes::SerialSpeeds baudRate, RadioConfigTypes::ParityConfig parity) {
  gRadioSerial.end();  // safe even when the port is not open
  uint32_t baud = 0;
  uint16_t parityConfig = 0;
  switch (baudRate) {
    case RadioConfigTypes::SerialSpeeds::BAUD_1200: baud = 1200; break;
    case RadioConfigTypes::SerialSpeeds::BAUD_2400: baud = 2400; break;
    case RadioConfigTypes::SerialSpeeds::BAUD_4800: baud = 4800; break;
    case RadioConfigTypes::SerialSpeeds::BAUD_9600: baud = 9600; break;
    case RadioConfigTypes::SerialSpeeds::BAUD_19200: baud = 19200; break;
    case RadioConfigTypes::SerialSpeeds::BAUD_38400: baud = 38400; break;
    case RadioConfigTypes::SerialSpeeds::BAUD_57600: baud = 57600; break;
    case RadioConfigTypes::SerialSpeeds::BAUD_115200: baud = 115200; break;
  }
  switch (parity) {
    case RadioConfigTypes::ParityConfig::Parity_8N1: parityConfig = SERIAL_8N1; break;
    case RadioConfigTypes::ParityConfig::Parity_8E1: parityConfig = SERIAL_8E1; break;
    case RadioConfigTypes::ParityConfig::Parity_8O1: parityConfig = SERIAL_8O1; break;
  }
  gRadioSerial.begin(baud, parityConfig);
  return true;
}

}  // namespace

LoRaE22& radioLink() { return gRadio; }

int radioTxRoom() { return gRadioSerial.availableForWrite(); }

bool radioLinkBegin() {
  RadioConfig config;
  config.address = ADDRESS;
  config.networkId = NETWORKID;
  config.encryptionKey = ENCRYPTIONKEY;
  config.parityConfig = PARITYCONFIG;
  config.serialSpeed = SERIALSPEED;
  config.airDataRate = AIRDATARATE;
  config.packetSize = PACKETSIZE;
  config.worMode = WORMODE;
  config.worPeriod = WORPERIOD;
  config.relayMode = RELAYMODE;
  config.destination = DESTINATIONMODE;
  config.txPower = dBm33;
  config.ambientRSSIEnabled = AMBIENTRSSI;
  config.rssiReadingsEnabled = RSSIREADINGS;
  config.listenBeforeTxEnable = LISTENBEFORETX;
  gRadio.setConfig(config);
  gRadio.setFrequency(FREQUENCY);
  gRadio.changeSerialPortCallback(changeSerialPortConfig);
  gRadio.setTimeout(2000);
  const int8_t code = gRadio.init(3);
  gRadio.setMode(RadioMode::Normal);
  return code >= 0;
}

#endif  // ARDUINO
