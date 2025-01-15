#define MAX_HARDWARE_FAILURES 20
#define VALID_SETTINGS_FLAG 0xDAB0
#define LED_ON LOW
#define LED_OFF HIGH
#define SSID_SIZE 100
#define PASSWORD_SIZE 50
#define ADDRESS_SIZE 30
#define USERNAME_SIZE 50
#define MQTT_CLIENTID_SIZE 25
#define MQTT_TOPIC_SIZE 150
#define MQTT_TOPIC_DISTANCE "distance"
#define MQTT_TOPIC_STATE "present"
#define MQTT_TOPIC_BATTERY "battery"
#define MQTT_TOPIC_ANALOG "analog"
#define MQTT_TOPIC_RSSI "rssi"
#define MQTT_TOPIC_SNR "snr"
#define MQTT_CLIENT_ID_ROOT "DeliveryReporter"
#define MQTT_TOPIC_COMMAND_REQUEST "command"
#define MQTT_PAYLOAD_SETTINGS_COMMAND "settings" //show all user accessable settings
#define MQTT_PAYLOAD_RESET_PULSE_COMMAND "resetPulseCounter" //reset the pulse counter to zero
#define MQTT_PAYLOAD_REBOOT_COMMAND "reboot" //reboot the controller
#define MQTT_PAYLOAD_VERSION_COMMAND "version" //show the version number
#define MQTT_PAYLOAD_STATUS_COMMAND "status" //show the most recent flow values
#define JSON_STATUS_SIZE SSID_SIZE+PASSWORD_SIZE+USERNAME_SIZE+MQTT_TOPIC_SIZE+150 //+150 for associated field names, etc
#define PUBLISH_DELAY 400 //milliseconds to wait after publishing to MQTT to allow transaction to finish
#define WIFI_TIMEOUT_SECONDS 20 // give up on wifi after this long
#define FULL_BATTERY_COUNT 3686 //raw A0 count with a freshly charged 18650 lithium battery 
#define FULL_BATTERY_VOLTS 412 //4.12 volts for a fully charged 18650 lithium battery 
#define ONE_HOUR 3600000 //milliseconds
#define LORA_RX_PIN D5
#define LORA_TX_PIN D6
#define DEFAULT_LORA_ADDRESS 1
#define DEFAULT_LORA_NETWORK_ID 18
#define DEFAULT_LORA_BAND 915000000
#define DEFAULT_LORA_SPREADING_FACTOR 8
#define DEFAULT_LORA_BANDWIDTH 7
#define DEFAULT_LORA_CODING_RATE 1
#define DEFAULT_LORA_PREAMBLE 12
#define DEFAULT_LORA_BAUD_RATE 115200
void showSettings();
String getConfigCommand();
bool processCommand(String cmd);
void checkForCommand();
void report();
boolean publish(char* topic, const char* reading, boolean retain);
void incomingMqttHandler(char* reqTopic, byte* payload, unsigned int length) ;
void setup_wifi();
void connectToWiFi();
void reconnect();
void showSub(char* topic, bool subgood);
void initializeSettings();
boolean saveSettings();
void deserialize(StaticJsonDocument<250> &doc);
void setup();
void loop();
void incomingSerialData();
char* generateMqttClientId(char* mqttId);
