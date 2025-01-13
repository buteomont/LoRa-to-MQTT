/* A program to receive data from a RYLR998 LoRa receiver and publish it to MQTT.

    This is a comprehensive list of AT commands for the RYLR998 LoRa module:

  - Basic Commands
  AT: Test if the module can respond to commands
  AT+RESET: Perform a software reset of the module

  Configuration Commands
  AT+MODE=<Parameter>[,<RX time>,<Low speed time>]: Set the wireless work mode
    Where:
    <Parameter> ranges from 0 to 2:
      0: Transceiver mode (default)
      1: Sleep mode
      2: Smart receiving mode for power saving
    <RX time>: 30ms to 60000ms (default is 1000ms)
      Used in Smart receiving mode to set the active receiving time
    <Low speed time>: 30ms to 60000ms
      Used in Smart receiving mode to set the low-power state duration
  AT+IPR: Set the UART baud rate
  AT+BAND: Set RF frequency in Hz
  AT+PARAMETER=<Spreading Factor>,<Bandwidth>,<Coding Rate>,<Programmed Preamble>: Set the RF parameters 
    (spreading factor, bandwidth, coding rate, preamble)
    - Parameters
      - Spreading Factor (SF): Range 5-11 (default 9)
        Higher SF improves sensitivity but increases transmission time
        SF7 to SF9 at 125kHz, SF7 to SF10 at 250kHz, and SF7 to SF11 at 500kHz
      - Bandwidth (BW): 7-9
        7: 125 KHz (default)
        8: 250 KHz
        9: 500 KHz
        Smaller bandwidth improves sensitivity but increases transmission time
      - Coding Rate (CR): 1-4 (default 1)
        1: 4/5
        2: 4/6
        3: 4/7
        4: 4/8
        Lower coding rate is faster
      - Programmed Preamble: Default 12
        When NETWORKID=18, can be set from 4 to 24
        For other NETWORKID values, must be set to 12
        Larger preamble reduces data loss but increases transmission time
  AT+ADDRESS: Set the ADDRESS ID of the module (0 to 65535)
  AT+NETWORKID: Set the network ID (3 to 15, or 18 (default))
  AT+CPIN: Set the domain password
    - The password must be exactly 8 characters long
    - Valid range: 00000001 to FFFFFFFF (hexadecimal)
    - Only modules using the same password can recognize and communicate with each other
  AT+CRFOP: Set the RF output power (0 to 22 dBm)

  - Communication Commands
  AT+SEND=<Address>,<Payload Length>,<Data>: Send data to the appointed address (250 bytes max)
    Use address 0 to broadcast to all addresses.
    For payloads greater than 100 bytes, the suggested setting is "AT+PARAMETER=8,7,1,12"

  - Query Commands
  AT+UID?: Inquire module ID
  AT+VER?: Inquire the firmware version

  - Other Commands
  AT+FACTORY: Reset all parameters to manufacturer defaults

  - Response Formats
  +RCV=<Address>,<Length>,<Data>,<RSSI>,<SNR>: Shows received data actively
  +OK: Indicates successful command execution
  +ERR: Indicates an error, followed by an error code
  +READY: Reset was successful and waiting for a command

  You can also add a question mark at the end of most commands
  to query their current settings. 

  - The Error Codes that can be returned are:
  +ERR=1: There is no "enter" or 0x0D 0x0A (carriage return and line feed) at the end of the AT Command.
  +ERR=2: The head of the AT command is not an "AT" string.
  +ERR=4: Unknown command or the data to be sent does not match the actual length.
  +ERR=5: The data to be sent does not match the actual length.
  +ERR=10: TX is over time (transmission timeout).
  +ERR=12: CRC error.
  +ERR=13: TX data exceeds 240 bytes.
  +ERR=14: Failed to write flash memory.
  +ERR=15: Unknown failure.
  +ERR=17: Last TX was not completed.
  +ERR=18: Preamble value is not allowed.
  +ERR=19: RX failed, Header error.
  
  The data from the RYLR998 before processing is like this:
  +RCV=3,46,{"DISTANCE":8123,"ISPRSENT":0,"BATTERY":3.41},-47,12
       a b  c                                              d   e
  a=source address
  b=data length
  c=raw data
  d=RSSI
  e=SNR

  This program will receive a JSON object from the RYLR998 object that looks something like this:
  {"address":2,"rssi":-23,"snr":3,"data":{"distance":8123,"ispresent":0,"battery":3.41}}
  and send each field to the MQTT broker over WiFi.
  */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include "RYLR998.h"
#include "lora2mqtt.h"

#define VERSION "25.08.00.0"  //remember to update this after every change! YY.MM.DD.REV

WiFiClient wifiClient;
RYLR998 lora(LORA_RX_PIN, LORA_TX_PIN);
PubSubClient mqttClient(wifiClient);
StaticJsonDocument<250> doc;

String commandString = "";     // a String to hold incoming commands from serial
bool commandComplete = false;  // goes true when enter is pressed


// These are the settings that get stored in EEPROM.  They are all in one struct which
// makes it easier to store and retrieve.
typedef struct 
  {
  unsigned int validConfig=0; 
  char ssid[SSID_SIZE] = "";
  char wifiPassword[PASSWORD_SIZE] = "";
  char mqttBrokerAddress[ADDRESS_SIZE]=""; //default
  int mqttBrokerPort=1883;
  char mqttUsername[USERNAME_SIZE]="";
  char mqttPassword[PASSWORD_SIZE]="";
  char mqttTopicRoot[MQTT_TOPIC_SIZE]="";
  char mqttClientId[MQTT_CLIENTID_SIZE]=""; //will be the same across reboots
  bool debug=false;
  char address[ADDRESS_SIZE]=""; //static address for this device
  char netmask[ADDRESS_SIZE]=""; //size of network
  int loRaAddress=DEFAULT_LORA_ADDRESS;
  int loRaNetworkID=DEFAULT_LORA_NETWORK_ID;
  uint32_t loRaBand=DEFAULT_LORA_BAND; //(915000000);
  byte loRaSpreadingFactor=DEFAULT_LORA_SPREADING_FACTOR;
  byte loRaBandwidth=DEFAULT_LORA_BANDWIDTH;
  byte loRaCodingRate=DEFAULT_LORA_CODING_RATE;
  byte loRaPreamble=DEFAULT_LORA_PREAMBLE;
  uint32_t loRaBaudRate=DEFAULT_LORA_BAUD_RATE; //both for RF and serial comms
  } conf;
conf settings; //all settings in one struct makes it easier to store in EEPROM
boolean settingsAreValid=false;

IPAddress ip;
IPAddress mask;

typedef struct
  {
  uint16_t distance;
  bool isPresent;
  float_t battery;
  int8_t rssi;
  int8_t snr;
  uint8_t address;
  } box;
box boxStatus; 


void showSettings()
  {
  Serial.print("broker=<MQTT broker host name or address> (");
  Serial.print(settings.mqttBrokerAddress);
  Serial.println(")");
  Serial.print("port=<port number>   (");
  Serial.print(settings.mqttBrokerPort);
  Serial.println(")");
  Serial.print("topicroot=<topic root> (");
  Serial.print(settings.mqttTopicRoot);
  Serial.println(")  Note: must end with \"/\"");  
  Serial.print("user=<mqtt user> (");
  Serial.print(settings.mqttUsername);
  Serial.println(")");
  Serial.print("pass=<mqtt password> (");
  Serial.print(settings.mqttPassword);
  Serial.println(")");
  Serial.print("ssid=<wifi ssid> (");
  Serial.print(settings.ssid);
  Serial.println(")");
  Serial.print("wifipass=<wifi password> (");
  Serial.print(settings.wifiPassword);
  Serial.println(")");
  Serial.print("address=<Static IP address if so desired> (");
  Serial.print(settings.address);
  Serial.println(")");
  Serial.print("netmask=<Network mask to be used with static IP> (");
  Serial.print(settings.netmask);
  Serial.println(")");
  Serial.print("debug=1|0 (");
  Serial.print(settings.debug);
  Serial.println(")");
  Serial.print("loRaAddress=<LoRa module's address 0-65535> (");
  Serial.print(settings.loRaAddress);
  Serial.println(")");
  Serial.print("loRaBand=<Freq in Hz> (");
  Serial.print(settings.loRaBand);
  Serial.println(")");
  Serial.print("loRaBandwidth=<bandwidth code 7-9> (");
  Serial.print(settings.loRaBandwidth);
  Serial.println(")");
  Serial.print("loRaCodingRate=<Coding rate code 1-4> (");
  Serial.print(settings.loRaCodingRate);
  Serial.println(")");
  Serial.print("loRaNetworkID=<Network ID 3-15 or 18> (");
  Serial.print(settings.loRaNetworkID);
  Serial.println(")");
  Serial.print("loRaSpreadingFactor=<Spreading Factor 5-11> (");
  Serial.print(settings.loRaSpreadingFactor);
  Serial.println(")");
  Serial.print("loRaPreamble=<4-24, see docs> (");
  Serial.print(settings.loRaPreamble);
  Serial.println(")");
  Serial.print("loRaBaudRate=<baud rate> (");
  Serial.print(settings.loRaBaudRate);
  Serial.println(")");

  Serial.print("MQTT Client ID is ");
  Serial.println(settings.mqttClientId);
  Serial.print("Address is ");
  Serial.println(wifiClient.localIP());
  Serial.println("\n*** Use NULL to reset a setting to its default value ***");
  Serial.println("*** Use \"factorydefaults=yes\" to reset all settings  ***\n");
  
  Serial.print("\nSettings are ");
  Serial.println(settingsAreValid?"valid.":"incomplete.");
  }

// Configure LoRa module
void configureLoRa()
  {
  if (settingsAreValid)
    {
    lora.setAddress(settings.loRaAddress);
    lora.setNetworkID(settings.loRaNetworkID);
    lora.setBand(settings.loRaBand);
    lora.setBaudRate(settings.loRaBaudRate);
    lora.setParameter(settings.loRaSpreadingFactor, 
                      settings.loRaBandwidth, 
                      settings.loRaCodingRate, 
                      settings.loRaPreamble);
    }
  }

  
/*
 * Check for configuration input via the serial port.  Return a null string 
 * if no input is available or return the complete line otherwise.
 */
String getConfigCommand()
  {
  if (commandComplete) 
    {
    Serial.println(commandString);
    String newCommand=commandString;

    commandString = "";
    commandComplete = false;
    return newCommand;
    }
  else return "";
  }

bool processCommand(String cmd)
  {
  bool commandFound=true; //saves a lot of code

  const char *str=cmd.c_str();
  char *val=NULL;
  char *nme=strtok((char *)str,"=");
  if (nme!=NULL)
    val=strtok(NULL,"=");

  if (nme[0]==13) //a single cr means show current settings
    {
    showSettings();
    commandFound=false; //command not found
    }
  else
    {
    //Get rid of the carriage return
    if (val!=NULL && strlen(val)>0 && val[strlen(val)-1]==13)
      val[strlen(val)-1]=0; 

    if (val!=NULL)
      {
      if (strcmp(val,"NULL")==0) //to nullify a value, you have to really mean it
        {
        strcpy(val,"");
        }
      
      if (strcmp(nme,"broker")==0)
        {
        strcpy(settings.mqttBrokerAddress,val);
        saveSettings();
        }
      else if (strcmp(nme,"port")==0)
        {
        if (!val)
          strcpy(val,"0");
        settings.mqttBrokerPort=atoi(val);
        saveSettings();
        }
      else if (strcmp(nme,"topicroot")==0)
        {
        strcpy(settings.mqttTopicRoot,val);
        saveSettings();
        }
      else if (strcmp(nme,"user")==0)
        {
        strcpy(settings.mqttUsername,val);
        saveSettings();
        }
      else if (strcmp(nme,"pass")==0)
        {
        strcpy(settings.mqttPassword,val);
        saveSettings();
        }
      else if (strcmp(nme,"ssid")==0)
        {
        strcpy(settings.ssid,val);
        saveSettings();
        }
      else if (strcmp(nme,"wifipass")==0)
        {
        strcpy(settings.wifiPassword,val);
        saveSettings();
        }
      else if (strcmp(nme,"address")==0)
        {
        strcpy(settings.address,val);
        saveSettings();
        }
      else if (strcmp(nme,"netmask")==0)
        {
        strcpy(settings.netmask,val);
        saveSettings();
        }
      else if (strcmp(nme,"loRaAddress")==0)
        {
        if (!val)
          strcpy(val,"0");
        settings.loRaAddress=atoi(val);
        configureLoRa();
        saveSettings();
        }
      else if (strcmp(nme,"loRaBand")==0)
        {
        if (!val)
          strcpy(val,"0");
        settings.loRaBand=atoi(val);
        configureLoRa();
        saveSettings();
        }
      else if (strcmp(nme,"loRaBandwidth")==0)
        {
        if (!val)
          strcpy(val,"0");
        settings.loRaBandwidth=atoi(val);
        configureLoRa();
        saveSettings();
        }
      else if (strcmp(nme,"loRaCodingRate")==0)
        {
        if (!val)
          strcpy(val,"0");
        settings.loRaCodingRate=atoi(val);
        configureLoRa();
        saveSettings();
        }
      else if (strcmp(nme,"loRaNetworkID")==0)
        {
        if (!val)
          strcpy(val,"0");
        settings.loRaNetworkID=atoi(val);
        configureLoRa();
        saveSettings();
        }
      else if (strcmp(nme,"loRaSpreadingFactor")==0)
        {
        if (!val)
          strcpy(val,"0");
        settings.loRaSpreadingFactor=atoi(val);
        configureLoRa();
        saveSettings();
        }
      else if (strcmp(nme,"loRaPreamble")==0)
        {
        if (!val)
          strcpy(val,"0");
        settings.loRaPreamble=atoi(val);
        configureLoRa();
        saveSettings();
        }
      else if (strcmp(nme,"loRaBaudRate")==0)
        {
        if (!val)
          strcpy(val,"0");
        settings.loRaBaudRate=atoi(val);
        configureLoRa();
        saveSettings();
        }
      else if (strcmp(nme,"debug")==0)
        {
        if (!val)
          strcpy(val,"0");
        settings.debug=atoi(val)==1?true:false;
        saveSettings();
        }
      else if ((strcmp(nme,"resetmqttid")==0)&& (strcmp(val,"yes")==0))
        {
        generateMqttClientId(settings.mqttClientId);
        saveSettings();
        }
      else if ((strcmp(nme,"factorydefaults")==0) && (strcmp(val,"yes")==0)) //reset all eeprom settings
        {
        Serial.println("\n*********************** Resetting EEPROM Values ************************");
        initializeSettings();
        saveSettings();
        delay(2000);
        ESP.restart();
        }
      else
        {
        showSettings();
        commandFound=false; //command not found
        }
      }
    }
  return commandFound;
  }

void initializeSettings()
  {
  settings.validConfig=0; 
  strcpy(settings.ssid,"");
  strcpy(settings.wifiPassword,"");
  strcpy(settings.mqttBrokerAddress,""); //default
  settings.mqttBrokerPort=1883;
  strcpy(settings.mqttUsername,"");
  strcpy(settings.mqttPassword,"");
  strcpy(settings.mqttTopicRoot,"");
  strcpy(settings.address,"");
  strcpy(settings.netmask,"255.255.255.0");
  settings.loRaAddress=DEFAULT_LORA_ADDRESS;
  settings.loRaNetworkID=DEFAULT_LORA_NETWORK_ID;
  settings.loRaBand=DEFAULT_LORA_BAND;
  settings.loRaSpreadingFactor=DEFAULT_LORA_SPREADING_FACTOR;
  settings.loRaBandwidth=DEFAULT_LORA_BANDWIDTH;
  settings.loRaCodingRate=DEFAULT_LORA_CODING_RATE;
  settings.loRaPreamble=DEFAULT_LORA_PREAMBLE;
  settings.loRaBaudRate=DEFAULT_LORA_BAUD_RATE;
  generateMqttClientId(settings.mqttClientId);
  }

void checkForCommand()
  {
  if (Serial.available())
    {
    incomingSerialData();
    String cmd=getConfigCommand();
    if (cmd.length()>0)
      {
      processCommand(cmd);
      }
    }
  }

/************************
 * Do the MQTT thing
 ************************/
void report()
  {
  if (strlen(settings.mqttBrokerAddress)>0)
    {
    char topic[MQTT_TOPIC_SIZE];
    char reading[18];
    boolean success=false;
    
    Serial.println();
    serializeJson(doc, Serial); //print it to the console
    Serial.println();

    //publish the radio signal/noise ratio
    strcpy(topic,settings.mqttTopicRoot);
    strcat(topic,MQTT_TOPIC_SNR);
    sprintf(reading,"%d",(int)doc["snr"]); 
    success=publish(topic,reading,true); //retain
    if (!success)
      Serial.println("************ Failed publishing rssi!");
     
    //publish the radio strength reading
    strcpy(topic,settings.mqttTopicRoot);
    strcat(topic,MQTT_TOPIC_RSSI);
    sprintf(reading,"%d",(int)doc["rssi"]); 
    success=publish(topic,reading,true); //retain
    if (!success)
      Serial.println("************ Failed publishing rssi!");
   
    //publish the battery voltage
    strcpy(topic,settings.mqttTopicRoot);
    strcat(topic,MQTT_TOPIC_BATTERY);
    sprintf(reading,"%.2f",(float)doc["battery"]); 
    success=publish(topic,reading,true); //retain
    if (!success)
      Serial.println("************ Failed publishing battery voltage!");

    //publish the distance measurement
    strcpy(topic,settings.mqttTopicRoot);
    strcat(topic,MQTT_TOPIC_DISTANCE);
    sprintf(reading,"%d",(int)doc["distance"]); 
    success=publish(topic,reading,true); //retain
    if (!success)
      Serial.println("************ Failed publishing distance measurement!");

    //publish the object detection state
    strcpy(topic,settings.mqttTopicRoot);
    strcat(topic,MQTT_TOPIC_STATE);
    sprintf(reading,"%s",doc["isPresent"]?"YES":"NO"); //item within range window
    success=publish(topic,reading,true); //retain
    if (!success)
      Serial.println("************ Failed publishing sensor state!");
    }
  }

boolean publish(char* topic, const char* reading, boolean retain)
  {
  Serial.print(topic);
  Serial.print(" ");
  Serial.println(reading);
  boolean ok=false;

  if (mqttClient.connected() && settings.mqttTopicRoot)
    {
    ok=mqttClient.publish(topic,reading,retain); 
    }
  return ok;
  }



/**
 * Handler for incoming MQTT messages.  The payload is the command to perform. 
 * The MQTT message topic sent is the topic root plus the command.
 * Implemented commands are: 
 * MQTT_PAYLOAD_SETTINGS_COMMAND: sends a JSON payload of all user-specified settings
 * MQTT_PAYLOAD_REBOOT_COMMAND: Reboot the controller
 * MQTT_PAYLOAD_VERSION_COMMAND Show the version number
 * MQTT_PAYLOAD_STATUS_COMMAND Show the most recent flow values
 */
void incomingMqttHandler(char* reqTopic, byte* payload, unsigned int length) 
  {
  if (settings.debug)
    {
    Serial.println("====================================> Callback works.");
    }
  payload[length]='\0'; //this should have been done in the calling code, shouldn't have to do it here
  boolean rebootScheduled=false; //so we can reboot after sending the reboot response
  char charbuf[100];
  sprintf(charbuf,"%s",payload);
  const char* response;
  
  
  //if the command is MQTT_PAYLOAD_SETTINGS_COMMAND, send all of the settings
  if (strcmp(charbuf,MQTT_PAYLOAD_SETTINGS_COMMAND)==0)
    {
    char tempbuf[35]; //for converting numbers to strings
    char jsonStatus[JSON_STATUS_SIZE];
    
    strcpy(jsonStatus,"{");
    strcat(jsonStatus,"\"broker\":\"");
    strcat(jsonStatus,settings.mqttBrokerAddress);
    strcat(jsonStatus,"\", \"port\":");
    sprintf(tempbuf,"%d",settings.mqttBrokerPort);
    strcat(jsonStatus,tempbuf);
    strcat(jsonStatus,", \"topicroot\":\"");
    strcat(jsonStatus,settings.mqttTopicRoot);
    strcat(jsonStatus,"\", \"user\":\"");
    strcat(jsonStatus,settings.mqttUsername);
    strcat(jsonStatus,"\", \"pass\":\"");
    strcat(jsonStatus,settings.mqttPassword);
    strcat(jsonStatus,"\", \"ssid\":\"");
    strcat(jsonStatus,settings.ssid);
    strcat(jsonStatus,"\", \"wifipass\":\"");
    strcat(jsonStatus,settings.wifiPassword);
    strcat(jsonStatus,"\", \"mqttClientId\":\"");
    strcat(jsonStatus,settings.mqttClientId);
    strcat(jsonStatus,"\", \"address\":\"");
    strcat(jsonStatus,settings.address);
    strcat(jsonStatus,"\", \"netmask\":\"");
    strcat(jsonStatus,settings.netmask);

    strcat(jsonStatus,"\", \"loRaAddress\":");
    sprintf(tempbuf,"%d",settings.loRaAddress);
    strcat(jsonStatus,tempbuf);
    strcat(jsonStatus,"\", \"loRaBand\":");
    sprintf(tempbuf,"%d",settings.loRaBand);
    strcat(jsonStatus,tempbuf);
    strcat(jsonStatus,"\", \"loRaBandwidth\":");
    sprintf(tempbuf,"%d",settings.loRaBandwidth);
    strcat(jsonStatus,tempbuf);
    strcat(jsonStatus,"\", \"loRaCodingRate\":");
    sprintf(tempbuf,"%d",settings.loRaCodingRate);
    strcat(jsonStatus,tempbuf);
    strcat(jsonStatus,"\", \"loRaNetworkID\":");
    sprintf(tempbuf,"%d",settings.loRaNetworkID);
    strcat(jsonStatus,tempbuf);
    strcat(jsonStatus,"\", \"loRaSpreadingFactor\":");
    sprintf(tempbuf,"%d",settings.loRaSpreadingFactor);
    strcat(jsonStatus,tempbuf);
    strcat(jsonStatus,"\", \"loRaPreamble\":");
    sprintf(tempbuf,"%d",settings.loRaPreamble);
    strcat(jsonStatus,tempbuf);
    strcat(jsonStatus,"\", \"loRaBaudRate\":");
    sprintf(tempbuf,"%d",settings.loRaBaudRate);
    strcat(jsonStatus,tempbuf);
   
    strcat(jsonStatus,"\", \"debug\":\"");
    strcat(jsonStatus,settings.debug?"true":"false");
    strcat(jsonStatus,"\", \"IPAddress\":\"");
    strcat(jsonStatus,wifiClient.localIP().toString().c_str());
    
    strcat(jsonStatus,"\"}");
    response=jsonStatus;
    }
  else if (strcmp(charbuf,MQTT_PAYLOAD_VERSION_COMMAND)==0) //show the version number
    {
    char tmp[15];
    strcpy(tmp,VERSION);
    response=tmp;
    }
  else if (strcmp(charbuf,MQTT_PAYLOAD_STATUS_COMMAND)==0) //show the latest value
    {
    report();
    
    char tmp[25];
    strcpy(tmp,"Status report complete");
    response=tmp;
    }
  else if (strcmp(charbuf,MQTT_PAYLOAD_REBOOT_COMMAND)==0) //reboot the controller
    {
    char tmp[10];
    strcpy(tmp,"REBOOTING");
    response=tmp;
    rebootScheduled=true;
    }
  else if (processCommand(charbuf))
    {
    response="OK";
    }
  else
    {
    char badCmd[18];
    strcpy(badCmd,"(empty)");
    response=badCmd;
    }
    
  char topic[MQTT_TOPIC_SIZE];
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,charbuf); //the incoming command becomes the topic suffix

  if (!publish(topic,response,false)) //do not retain
    Serial.println("************ Failure when publishing status response!");
    
  delay(2000); //give publish time to complete
  
  if (rebootScheduled)
    {
    ESP.restart();
    }
  }


//Generate an MQTT client ID.  This should not be necessary very often
char* generateMqttClientId(char* mqttId)
  {
  strcpy(mqttId,MQTT_CLIENT_ID_ROOT);
  strcat(mqttId, String(random(0xffff), HEX).c_str());
  if (settings.debug)
    {
    Serial.print("New MQTT userid is ");
    Serial.println(mqttId);
    }
  return mqttId;
  }


void setup_wifi()
  {
  // WiFi connection setup code here
  if (WiFi.status() != WL_CONNECTED)
    {
    Serial.print("Attempting to connect to WPA SSID \"");
    Serial.print(settings.ssid);
    Serial.println("\"");

    WiFi.mode(WIFI_STA); //station mode, we are only a client in the wifi world

    if (ip.isSet()) //Go with a dynamic address if no valid IP has been entered
      {
      if (!WiFi.config(ip,ip,mask))
        {
        Serial.println("STA Failed to configure");
        }
      }

    unsigned long connectTimeout = millis() + WIFI_TIMEOUT_SECONDS*1000; // 10 second timeout
    WiFi.begin(settings.ssid, settings.wifiPassword);
    while (WiFi.status() != WL_CONNECTED && millis() < connectTimeout) 
      {
      // not yet connected
      // Serial.print(".");
      // checkForCommand(); // Check for input in case something needs to be changed to work
      delay(100);
      }
    
    checkForCommand(); // Check for input in case something needs to be changed to work

    if (WiFi.status() != WL_CONNECTED)
      {
      Serial.println("Connection to network failed. ");
      Serial.println();
//      show("Wifi failed to \nconnect");
      delay(3000);
      }
    else 
      {
      Serial.print("Connected to network with address ");
      Serial.println(WiFi.localIP());
      Serial.println();
      }
    }
  else if (settings.debug)
    {
    Serial.print("Actual network address is ");
    Serial.println(WiFi.localIP());
    }
  } 

/*
 * Reconnect to the MQTT broker
 */
void reconnect() 
  {
  if (strlen(settings.mqttBrokerAddress)>0)
    {
    if (WiFi.status() != WL_CONNECTED)
      {
      Serial.println("WiFi not ready, skipping MQTT connection");
      }
    else
      {
      // Loop until we're reconnected
      while (!mqttClient.connected()) 
        {      
        Serial.print("Attempting MQTT connection...");

        mqttClient.setBufferSize(JSON_STATUS_SIZE); //default (256) isn't big enough
        mqttClient.setServer(settings.mqttBrokerAddress, settings.mqttBrokerPort);
        mqttClient.setCallback(incomingMqttHandler);
        
        // Attempt to connect
        if (mqttClient.connect(settings.mqttClientId,settings.mqttUsername,settings.mqttPassword))
          {
          Serial.println("connected to MQTT broker.");

          //resubscribe to the incoming message topic
          char topic[MQTT_TOPIC_SIZE];
          strcpy(topic,settings.mqttTopicRoot);
          strcat(topic,MQTT_TOPIC_COMMAND_REQUEST);
          bool subgood=mqttClient.subscribe(topic);
          showSub(topic,subgood);
          }
        else 
          {
          Serial.print("failed, rc=");
          Serial.println(mqttClient.state());
          Serial.println("Will try again in a second");
          
          // Wait a second before retrying
          // In the meantime check for input in case something needs to be changed to make it work
        //  checkForCommand(); 
          
          delay(1000);
          }
        checkForCommand();
        }
      mqttClient.loop(); //This has to happen every so often or we get disconnected for some reason
      }
    }
  else if (settings.debug)
    {
    Serial.println("Broker address not set, ignoring MQTT");
    }
  }

void showSub(char* topic, bool subgood)
  {
  if (settings.debug)
    {
    Serial.print("++++++Subscribing to ");
    Serial.print(topic);
    Serial.print(":");
    Serial.println(subgood);
    }
  }

/*
 * Save the settings to EEPROM. Set the valid flag if everything is filled in.
 */
boolean saveSettings()
  {
  if (strlen(settings.ssid)>0 &&
      strlen(settings.wifiPassword)>0 &&
      // strlen(settings.mqttBrokerAddress)>0 &&
      // settings.mqttBrokerPort!=0 &&
      strlen(settings.mqttTopicRoot)>0 &&
      strlen(settings.mqttClientId)>0)
    {
    Serial.println("Settings deemed complete");
    settings.validConfig=VALID_SETTINGS_FLAG;
    settingsAreValid=true;
    }
  else
    {
    Serial.println("Settings still incomplete");
    settings.validConfig=0;
    settingsAreValid=false;
    }
    
  //The mqttClientId is not set by the user, but we need to make sure it's set  
  if (strlen(settings.mqttClientId)==0)
    {
    generateMqttClientId(settings.mqttClientId);
    }
    
  EEPROM.put(0,settings);
  if (settings.debug)
    Serial.println("Committing settings to eeprom");
  return EEPROM.commit();
  }

// populate the box struct from the received json
void deserialize(StaticJsonDocument<250> &doc)
  {
  boxStatus.address=doc["address"];
  boxStatus.battery=doc["battery"];
  boxStatus.distance=doc["distance"];
  }

void initLoRa()
  {
  //loraSerial.begin(settings.loRaBaudRate);
  lora.begin(settings.loRaBaudRate);
  lora.setJsonDocument(doc);
  configureLoRa();
 
  Serial.println(lora.getMode());
  Serial.println(lora.getBand());
  Serial.println(lora.getParameter());
  Serial.println(lora.getAddress());
  Serial.println(lora.getNetworkID());
  Serial.println(lora.getCPIN());
  Serial.println(lora.getRFPower());
  Serial.println(lora.getBaudRate()); 
 }


void initSerial()
  {
  Serial.begin(115200);
  Serial.setTimeout(10000);
  
  while (!Serial); // wait here for serial port to connect.
  Serial.println();
  Serial.println("Serial communications established.");
  }

/*
*  Initialize the settings from eeprom and determine if they are valid
*/
void loadSettings()
  {
  EEPROM.get(0,settings);
  if (settings.validConfig==VALID_SETTINGS_FLAG)    //skip loading stuff if it's never been written
    {
    settingsAreValid=true;
    if (settings.debug)
      {
      Serial.println("\nLoaded configuration values from EEPROM");
      }
    }
  else
    {
    Serial.println("Skipping load from EEPROM, device not configured.");    
    settingsAreValid=false;
    }
    showSettings();
  }


void initSettings()
  {
  EEPROM.begin(sizeof(settings)); //fire up the eeprom section of flash
  commandString.reserve(200); // reserve 200 bytes of serial buffer space for incoming command string

  loadSettings(); //set the values from eeprom 

  if (settings.mqttBrokerPort < 0) //then this must be the first powerup
    {
    Serial.println("\n*********************** Resetting All EEPROM Values ************************");
    initializeSettings();
    saveSettings();
    delay(2000);
    ESP.restart();
    }
  }

/*
 * If not connected to wifi, connect.
 */
void connectToWiFi()
  {
  if (settingsAreValid && WiFi.status() != WL_CONNECTED)
    {
    Serial.print("Attempting to connect to WPA SSID \"");
    Serial.print(settings.ssid);
    Serial.println("\"");

//    WiFi.forceSleepWake(); //turn on the radio
//    delay(1);              //return control to let it come on
    
    WiFi.mode(WIFI_STA); //station mode, we are only a client in the wifi world

    if (ip.isSet()) //Go with a dynamic address if no valid IP has been entered
      {
      if (!WiFi.config(ip,ip,mask))
        {
        Serial.println("STA Failed to configure");
        }
      }

    unsigned long connectTimeout = millis() + WIFI_TIMEOUT_SECONDS*1000; // 10 second timeout
    WiFi.begin(settings.ssid, settings.wifiPassword);
    delay(1000);
    while (WiFi.status() != WL_CONNECTED && millis() < connectTimeout) 
      {
      // not yet connected
      // Serial.print(".");
      // checkForCommand(); // Check for input in case something needs to be changed to work
      Serial.print(".");
      checkForCommand();
      delay(500);
      }
    
    checkForCommand(); // Check for input in case something needs to be changed to work

    if (WiFi.status() != WL_CONNECTED)
      {
      Serial.println("\nConnection to network failed. ");
      delay(3000);
      }
    else 
      {
      Serial.print("\nConnected to network with address ");
      Serial.println(WiFi.localIP());
      Serial.println();
      }
    }
  else if (settings.debug)
    {
    Serial.print("Actual network address is ");
    Serial.println(WiFi.localIP());
    }
  }


void setup()
  {
  initSerial();

  initSettings();

  if (settingsAreValid)
    {      
    //initialize everything
    Serial.println("Initializing LoRa module");
    initLoRa();

    if (settings.debug)
      {
      if (!ip.fromString(settings.address))
        {
        Serial.println("IP Address "+String(settings.address)+" is not valid. Using dynamic addressing.");
        // settingsAreValid=false;
        // settings.validConfig=false;
        }
      else if (!mask.fromString(settings.netmask))
        {
        Serial.println("Network mask "+String(settings.netmask)+" is not valid.");
        // settingsAreValid=false;
        // settings.validConfig=false;
        }
      }
    }
  //showSettings();
  if (lora.testComm())
    Serial.println("RYLR998 is working.");
  else
    Serial.println("No response from RYLR998");
  }

void loop()
  {
  if (settingsAreValid)
    {      
    if (WiFi.status() != WL_CONNECTED)
      {
      connectToWiFi();
      }
    if (!mqttClient.connected() && WiFi.status() == WL_CONNECTED)
     {
      reconnect();
      }

    if (lora.handleIncoming()) 
      {
      report();
      }
    mqttClient.loop();
    }
  yield();
  checkForCommand();
  }

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void incomingSerialData() 
  {
  while (Serial.available()) 
    {
    // get the new byte
    char inChar = (char)Serial.read();
    Serial.print(inChar); //echo it back to the terminal

    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it 
    if (inChar == '\n') 
      {
      commandComplete = true;
      }
    else
      {
      // add it to the inputString 
      commandString += inChar;
      }
    }
  }