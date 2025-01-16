/**************************************************************************
 * Attention! This file has hard links to both Delivery Box Reporter LoRa *
 * and LoRa-to-MQTT projects! A change in this file will be reflected in  *
 * both of these projects, and possibly others!                           *
 **************************************************************************/ 
 

#ifndef RYLR998_H
#define RYLR998_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

class RYLR998 {
public:
    RYLR998(int rx, int tx);
    void begin(long baudRate);
    void setJsonDocument(StaticJsonDocument<250>& doc);
    bool handleIncoming();
    bool send(uint16_t address, const String& data);
    bool setMode(uint8_t mode, uint16_t rxTime = 0, uint16_t lowSpeedTime = 0);
    bool setBand(uint32_t frequency);
    bool setParameter(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t preamble);
    bool setAddress(uint16_t address);
    bool setNetworkID(uint8_t id);
    bool setCPIN(const String& password);
    bool setRFPower(uint8_t power);
    bool setBaudRate(uint32_t baudrate);
    bool setdebug(bool debugMode);
    bool testComm();
    String getMode();
    String getBand();
    String getParameter();
    String getAddress();
    String getNetworkID();
    String getCPIN();
    String getRFPower();
    String getBaudRate();

private:
    SoftwareSerial _serial;
    int8_t _rxPin;
    int8_t _txPin;
    bool _debug=false;
    StaticJsonDocument<250>* _doc;
    String _sendCommand(const String& command, unsigned long timeout = 1000);
    void _parseRcvString(const String& input, String& address, String& length, String& jsonData, String& rssi, String& snr);

};

#endif // RYLR998_H
