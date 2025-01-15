/**************************************************************************
 * Attention! This file has hard links to both Delivery Box Reporter LoRa *
 * and LoRa-to-MQTT projects! A change in this file will be reflected in  *
 * both of these projects, and possibly others!                           *
 **************************************************************************/ 
 

#include "RYLR998.h"

RYLR998::RYLR998(int rx, int tx) : _serial(rx, tx), _doc(nullptr) {}

void RYLR998::begin(long baudRate)
    {
    Serial.println("Setting LoRa serial baud rate to "+String(baudRate));
    _serial.begin(baudRate);
    }

void RYLR998::setJsonDocument(StaticJsonDocument<250> &doc)
    {   
    _doc = &doc;
    }

bool RYLR998::handleIncoming()
    {
    bool ok=false;
    
    if (_serial.available())
        {
        String response = _serial.readStringUntil('\n');
        Serial.println("Received from LoRa:"+response);

        if (response.startsWith("+RCV="))
            {
            int equalSignIndex = response.indexOf('=');
            String result = response.substring(equalSignIndex + 1);

            String address, length, jsonData, rssi, snr;
            _parseRcvString(result, address, length, jsonData, rssi, snr);

            if (_doc && jsonData)
                {
                DeserializationError error = deserializeJson(*_doc, jsonData);
                if (error)
                    {
                    Serial.println(F("deserializeJson() failed. Error is "));
                    Serial.println(error.c_str());
                    }
                (*_doc)["address"]=atoi(address.c_str());
                (*_doc)["length"]=atoi(length.c_str());
                (*_doc)["rssi"]=atoi(rssi.c_str());
                (*_doc)["snr"]=atoi(snr.c_str());
                ok=true;
                }
            }
        }
    return ok;
    }

bool RYLR998::send(uint16_t address, const String &data)
    {
    String command = "AT+SEND=" + 
                        String(address) + "," + 
                        String(data.length()) + "," + 
                        data;
    String response = _sendCommand(command);
    return response == "+OK";
    }

bool RYLR998::setMode(uint8_t mode, uint16_t rxTime, uint16_t lowSpeedTime)
    {
    String command = "AT+MODE=" + String(mode);
    if (mode == 2)
    {
        command += "," + String(rxTime) + "," + String(lowSpeedTime);
    }
    String response = _sendCommand(command);
    return response == "+OK";
    }   

bool RYLR998::setBand(uint32_t frequency)
    {
    String command = "AT+BAND=" + String(frequency);
    String response = _sendCommand(command);
    return response == "+OK";
    }

bool RYLR998::setParameter(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t preamble)
    {
    String command = "AT+PARAMETER=" + String(sf) + "," + String(bw) + "," + String(cr) + "," + String(preamble);
    String response = _sendCommand(command);
    return response == "+OK";
    }

bool RYLR998::setAddress(uint16_t address)
    {
    String command = "AT+ADDRESS=" + String(address);
    String response = _sendCommand(command);
    return response == "+OK";
    }   

bool RYLR998::setNetworkID(uint8_t id)
    {
    String command = "AT+NETWORKID=" + String(id);
    String response = _sendCommand(command);
    return response == "+OK";
    }

bool RYLR998::setCPIN(const String &password)
    {
    String command = "AT+CPIN=" + password;
    String response = _sendCommand(command);
    return response == "+OK";
    }

bool RYLR998::setRFPower(uint8_t power)
    {
    String command = "AT+CRFOP=" + String(power);
    String response = _sendCommand(command);
    return response == "+OK";
    }

bool RYLR998::setBaudRate(uint32_t baudrate)
    {
    String command = "AT+IPR=" + String(baudrate);
    String response = _sendCommand(command);
    return response == "+OK";
    }

bool RYLR998::testComm()
    {
    String command = "AT";
    String response = _sendCommand(command);
    return response == "+OK";
    }

String RYLR998::getMode()
    {
    return _sendCommand("AT+MODE?");
    }  
     
String RYLR998::getBand()
    {
    return _sendCommand("AT+BAND?");
    }

String RYLR998::getParameter()
    {
    return _sendCommand("AT+PARAMETER?");
    }

String RYLR998::getAddress()
    {
    return _sendCommand("AT+ADDRESS?");
    }   

String RYLR998::getNetworkID()
    {
    return _sendCommand("AT+NETWORKID?");
    }

String RYLR998::getCPIN()
    {
    return _sendCommand("AT+CPIN?");
    }

String RYLR998::getRFPower()
    {
    return _sendCommand("AT+CRFOP?");
    }

String RYLR998::getBaudRate()
    {
    return _sendCommand("AT+IPR?");
    }


String RYLR998::_sendCommand(const String &command, unsigned long timeout)
    {
    Serial.println("sending lora command: "+command);
    _serial.println(command);
    unsigned long start = millis();
    while (millis() - start < timeout)
        {
        if (_serial.available())
            {
            String response = _serial.readStringUntil('\n');
            response.trim();
            return response;
            }
        }
    return "";
    }

void RYLR998::_parseRcvString(const String& input, String& address, String& length, String& jsonData, String& rssi, String& snr) 
    {
    int start = 0;
    int end = input.indexOf(',', start);
    address = input.substring(start, end);
    
    start = end + 1;
    end = input.indexOf(',', start);
    length = input.substring(start, end);
    
    start = end + 1;
    end = input.lastIndexOf(',', input.lastIndexOf(',') - 1);
    jsonData = input.substring(start, end);
    
    start = end + 1;
    end = input.lastIndexOf(',');
    rssi = input.substring(start, end);
    
    snr = input.substring(end + 1);
    }