/**************************************************************************
 * Attention! This file has hard links to both Delivery Box Reporter LoRa *
 * and LoRa-to-MQTT projects! A change in this file will be reflected in  *
 * both of these projects, and possibly others!                           *
 **************************************************************************/ 
/*  ****** RYLR998 Commands ********
 * 
 *  This is a comprehensive list of AT commands for the RYLR998 LoRa module:
 * 
 * - Basic Commands
 * AT: Test if the module can respond to commands
 * AT+RESET: Perform a software reset of the module
 *
 * Configuration Commands
 * AT+MODE=<Parameter>[,<RX time>,<Low speed time>]: Set the wireless work mode
 *   Where:
 *   <Parameter> ranges from 0 to 2:
 *     0: Transceiver mode (default)
 *     1: Sleep mode
 *     2: Smart receiving mode for power saving
 *   <RX time>: 30ms to 60000ms (default is 1000ms)
 *     Used in Smart receiving mode to set the active receiving time
 *   <Low speed time>: 30ms to 60000ms
 *     Used in Smart receiving mode to set the low-power state duration
 * AT+IPR: Set the UART baud rate
 * AT+BAND: Set RF frequency in Hz
 * AT+PARAMETER=<Spreading Factor>,<Bandwidth>,<Coding Rate>,<Programmed Preamble>: Set the RF parameters 
 *   (spreading factor, bandwidth, coding rate, preamble)
 *   - Parameters
 *     - Spreading Factor (SF): Range 5-11 (default 9)
 *       Higher SF improves sensitivity but increases transmission time
 *       SF7 to SF9 at 125kHz, SF7 to SF10 at 250kHz, and SF7 to SF11 at 500kHz
 *     - Bandwidth (BW): 7-9
 *       7: 125 KHz (default)
 *       8: 250 KHz
 *       9: 500 KHz
 *       Smaller bandwidth improves sensitivity but increases transmission time
 *     - Coding Rate (CR): 1-4 (default 1)
 *       1: 4/5
 *       2: 4/6
 *       3: 4/7
 *       4: 4/8
 *       Lower coding rate is faster
 *     - Programmed Preamble: Default 12
 *       When NETWORKID=18, can be set from 4 to 24
 *       For other NETWORKID values, must be set to 12
 *       Larger preamble reduces data loss but increases transmission time
 * AT+ADDRESS: Set the ADDRESS ID of the module (0 to 65535)
 * AT+NETWORKID: Set the network ID (3 to 15, or 18 (default))
 * AT+CPIN: Set the domain password
 *   - The password must be exactly 8 characters long
 *   - Valid range: 00000001 to FFFFFFFF (hexadecimal)
 *   - Only modules using the same password can recognize and communicate with each other
 * AT+CRFOP: Set the RF output power (0 to 22 dBm)
 *
 * - Communication Commands
 * AT+SEND=<Address>,<Payload Length>,<Data>: Send data to the appointed address (250 bytes max)
 *   Use address 0 to broadcast to all addresses.
 *   For payloads greater than 100 bytes, the suggested setting is "AT+PARAMETER=8,7,1,12"
 *
 * - Query Commands
 * AT+UID?: Inquire module ID
 * AT+VER?: Inquire the firmware version
 *
 * - Other Commands
 * AT+FACTORY: Reset all parameters to manufacturer defaults
 *
 * - Response Formats
 * +RCV=<Address>,<Length>,<Data>,<RSSI>,<SNR>: Shows received data actively
 * +OK: Indicates successful command execution
 * +ERR: Indicates an error, followed by an error code
 * +READY: Reset was successful and waiting for a command
 *
 * You can also add a question mark at the end of most commands
 * to query their current settings. 
 *
 * - The Error Codes that can be returned are:
 * +ERR=1: There is no "enter" or 0x0D 0x0A (carriage return and line feed) at the end of the AT Command.
 * +ERR=2: The head of the AT command is not an "AT" string.
 * +ERR=4: Unknown command or the data to be sent does not match the actual length.
 * +ERR=5: The data to be sent does not match the actual length.
 * +ERR=10: TX is over time (transmission timeout).
 * +ERR=12: CRC error.
 * +ERR=13: TX data exceeds 240 bytes.
 * +ERR=14: Failed to write flash memory.
 * +ERR=15: Unknown failure.
 * +ERR=17: Last TX was not completed.
 * +ERR=18: Preamble value is not allowed.
 * +ERR=19: RX failed, Header error.
 * **** NOTE that I have seen many bogus "+ERR=2" codes. Probably a bug in the RYLR998.
 */


#include "RYLR998.h"

RYLR998::RYLR998(int rx, int tx) : _serial(rx, tx), _doc(nullptr) 
    {
    _rxPin=rx;
    _txPin=tx;
    }

void RYLR998::begin(long baudRate)
    {
    if (_debug)
        Serial.println("LORA:Setting softwareSerial baud rate to "+String(baudRate));
    _serial.begin(baudRate, SWSERIAL_8N1, _rxPin, _txPin, false, 120,1200);
    
    //clear out any lingering buffer contents
    _serial.flush();
    while(_serial.available())
      {
      _serial.read(); 
      }
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
        
        if (_debug)
            Serial.println("LORA:Received from LoRa:"+response);

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
                    Serial.print(F("LORA:deserializeJson() failed. Error is: "));
                    Serial.println(error.c_str());
                    }
                //These are the standard data that go with all messages
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
    if (response != "+OK")
        Serial.println("LORA:Response from RYLR998: "+response);
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

bool RYLR998::setdebug(bool debugMode)
    {
    _debug=debugMode;
    return true;
    }

String RYLR998::getMode()
    {
    String response=_sendCommand("AT+MODE?");
    int equalSignIndex = response.indexOf('=');
    return response.substring(equalSignIndex + 1);
    }  
     
String RYLR998::getBand()
    {
    String response= _sendCommand("AT+BAND?");
    int equalSignIndex = response.indexOf('=');
    return response.substring(equalSignIndex + 1);
    }

String RYLR998::getParameter()
    {
    String response= _sendCommand("AT+PARAMETER?");
    int equalSignIndex = response.indexOf('=');
    return response.substring(equalSignIndex + 1);
    }

String RYLR998::getAddress()
    {
    String response= _sendCommand("AT+ADDRESS?");
    int equalSignIndex = response.indexOf('=');
    return response.substring(equalSignIndex + 1);
    }   

String RYLR998::getNetworkID()
    {
    String response= _sendCommand("AT+NETWORKID?");
    int equalSignIndex = response.indexOf('=');
    return response.substring(equalSignIndex + 1);
    }

String RYLR998::getCPIN()
    {
    String response= _sendCommand("AT+CPIN?");
    int equalSignIndex = response.indexOf('=');
    return response.substring(equalSignIndex + 1);
    }

String RYLR998::getRFPower()
    {
    String response= _sendCommand("AT+CRFOP?");
    int equalSignIndex = response.indexOf('=');
    return response.substring(equalSignIndex + 1);
    }

String RYLR998::getBaudRate()
    {
    String response= _sendCommand("AT+IPR?");
    int equalSignIndex = response.indexOf('=');
    return response.substring(equalSignIndex + 1);
    }

bool RYLR998::testComm()
    {
    String command = "AT";
    String response = _sendCommand(command);
    return response == "+OK";
    }


String RYLR998::_sendCommand(const String &command, unsigned long timeout)
    {
    if (_debug)
        Serial.println("LORA:Sending lora command:"+command);

    _serial.println(command);
    unsigned long start = millis();
    String response = "";
    while (millis() - start < timeout)
      {
      if (_serial.available())
        {
        response = _serial.readStringUntil('\n');
        response.trim();
        if (_debug)
            Serial.println("LORA:"+response);
        break;
        }
      }
    return response;
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