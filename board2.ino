/**
 * A simple Azure IoT example for sending telemetry to Iot Hub.
 */

#include <WiFi.h>
#include "Esp32MQTTClient.h"
#include <WiFiManager.h> 
#include <ArduinoJson.h>
#include <ezTime.h>     
#include "esp_adc_cal.h"
#include "math.h"

#define INTERVAL 10000  // IoT message sending interval in ms
#define MESSAGE_MAX_LEN 256


//// Actuators status////
int PC1_status = 0;                                        //status of the recirculation pump (0: OFF, 1: ON)
int EV1_status = 0;                                        //status of the electrovalve 1 (0: OFF, 1: ON)
int R1_status = 0;                                        //status of the boiler resistor (0: OFF, 1: ON)
//// firmware version of the device  ////
char sw_version[] = "0.1";    
//// Other handy variables ////
volatile int new_request = 0;                             // flag that tells if a new request has arrived from the hub
int received_msg_id = 0;                                  // used for ack mechanism
int received_msg_type = -1;                             // if 0 the HUB wants to know the status of the device
                                                          // if 1 the HUB wants to change the status of the device (with thw values passed in the message)
                                                          // if 2 the HUB ACKs the device
// defines for message type 
#define STATUS 0
#define SET_VALUES 1
#define HUB_ACK 2
// defines for voltages corresponding to temperature ranges, for PT1000:
//1.25V   corresponds to 0 deg C
//1.4518V  corresponds to 100 deg C
float mv = 0.816326;        // Vin = mv * ADC + qv
float qv = 121.836734;
float mt =  0.5;      // Temperature = mt * Vin + qt
float qt = -625;
////  MICROSOFT AZURE IOT DEFINITIONS   ////
static const char* connectionString = "HostName=geniale-iothub.azure-devices.net;DeviceId=00000001;SharedAccessKey=Cn4UylzZVDZD8UGzCTJazR3A9lRLnB+CbK6NkHxCIMk=";
static bool hasIoTHub = false;
static bool hasWifi = false;
//int messageCount = 1;
//static bool messageSending = true;
//static uint64_t send_interval_ms;

////  I/Os definitions    ////
#define EV1_GPIO   18               // Electrovalve 1 connected to GPIO18
#define PC1_GPIO   17               // Recirculation pump PC1 connected to GPIO17
#define R1_GPIO   19                // Boiler R1 connected to GPIO19
#define ST1_FORCE_GPIO   32         // Voltage force to temperature sensor 1 connected to GPIO32
#define SL2_GPIO   34               // Level sensor 2 connected to GPIO34
#define SL3_GPIO   35               // Level sensor 3 connected to GPIO35
#define ST1_MEASURE_GPIO   39       // Voltage measure temperature sensor 1 connected to GPIO39 (VN)
#define LED   5                     // Status led connected to GPIO5

static void SendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result)
{
  if (result == IOTHUB_CLIENT_CONFIRMATION_OK)
  {
    //Serial.println("Send Confirmation Callback finished.");
  }
}

static void MessageCallback(const char* payLoad, int size)
{
  Serial.println("Received message from HUB");
  if (size < 256) { 
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, payLoad);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
    }
    else {  
    new_request = 1;
    received_msg_id = doc["message_id"];
    received_msg_type = doc["message_type"];
      if(received_msg_type == SET_VALUES) {
          PC1_status = doc["PC1"];
          R1_status = doc["R1"];
          EV1_status = doc["EV1"];
      }
    }
  }
  else Serial.println("Cannot parse message, too long!");
}

/* NOT USED - DEVICE TWIN CALLBACK
static void DeviceTwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char *payLoad, int size)
{
  char *temp = (char *)malloc(size + 1);
  if (temp == NULL)
  {
    return;
  }
  memcpy(temp, payLoad, size);
  temp[size] = '\0';
  // Display Twin message.
  Serial.println(temp);
  free(temp);
}
*/
/* NOT USED - DEVICE METHOD CALLBACK
  static int  DeviceMethodCallback(const char *methodName, const unsigned char *payload, int size, unsigned char **response, int *response_size)
  {
    LogInfo("Try to invoke method %s", methodName);
    const char *responseMessage = "\"Successfully invoke device method\"";
    int result = 200;

    if (strcmp(methodName, "start") == 0)
    {
      LogInfo("Start sending temperature and humidity data");
      messageSending = true;
    }
    else if (strcmp(methodName, "stop") == 0)
    {
      LogInfo("Stop sending temperature and humidity data");
      messageSending = false;
    }
    else
    {
      LogInfo("No method %s found", methodName);
      responseMessage = "\"No method found\"";
      result = 404;
    }

    *response_size = strlen(responseMessage) + 1;
    *response = (unsigned char *)strdup(responseMessage);

    return result;
  }
*/

float read_temperature() {
  digitalWrite(ST1_FORCE_GPIO, HIGH);
  float mean = 0;
    for(int i = 0; i < 100; i++)  {
        float val = analogRead(ST1_MEASURE_GPIO);
        float vin = (mv * val + qv);
        float temperaturec = (mt * vin + qt);
        mean += temperaturec;
        delay(10); 
      }
  digitalWrite(ST1_FORCE_GPIO, LOW);
  Serial.println(String("Temperature in deg C: ") + String(mean/100, 2));
  return roundf(mean) / 100; 
}

void setup() {
  pinMode(EV1_GPIO, OUTPUT);
  pinMode(PC1_GPIO, OUTPUT);     
  pinMode(R1_GPIO, OUTPUT);
  pinMode(SL2_GPIO, INPUT);
  pinMode(SL3_GPIO, INPUT);
  pinMode(ST1_FORCE_GPIO, OUTPUT);
  pinMode(LED, OUTPUT);                              // Status LED
  pinMode(ST1_MEASURE_GPIO, INPUT);                  // ST1 voltage measurement is done by ADC1_3
  digitalWrite(EV1_GPIO, LOW);                       // Electrovalve is normally closed
  digitalWrite(PC1_GPIO, LOW);
  digitalWrite(R1_GPIO, LOW);
  digitalWrite(ST1_FORCE_GPIO, LOW);
  digitalWrite(LED, LOW);
  Serial.begin(115200);
  Serial.println("ESP32 Device");
  Serial.println("Initializing...");
  Serial.println(" > WiFi");
  Serial.println("Starting connecting WiFi.");

  delay(10);

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  // it is a good practice to make sure your code sets wifi mode how you want it.
  WiFiManager wm;
  //wm.resetSettings();  // reset settings - wipe stored credentials for testing
  bool res;
  // res = wm.autoConnect(); // auto generated AP name from chipid
  res = wm.autoConnect("GENIALE"); // anonymous ap
  //res = wm.autoConnect("AutoConnectAP","password"); // password protected ap

  if(!res) {
      Serial.println("Failed to connect to wifi");
      // ESP.restart();
  } 
  else {
      //if you get here you have connected to the WiFi    
      Serial.println("Connected to wifi...");
      // Wait for ezTime to get its time synchronized
	    waitForSync();
      Serial.println("UTC Time in ISO8601: " + UTC.dateTime(ISO8601));
      hasWifi = true;
      digitalWrite(LED, HIGH);
    }
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println(" > IoT Hub");
  if (!Esp32MQTTClient_Init((const uint8_t*)connectionString, true))
  {
    hasIoTHub = false;
    Serial.println("Initializing IoT hub failed.");
    return;
  }
  hasIoTHub = true;
  Esp32MQTTClient_SetSendConfirmationCallback(SendConfirmationCallback);
  Esp32MQTTClient_SetMessageCallback(MessageCallback);
  //Esp32MQTTClient_SetDeviceTwinCallback(DeviceTwinCallback);
  //Esp32MQTTClient_SetDeviceMethodCallback(DeviceMethodCallback);
  Serial.println("Waiting for messages from HUB...");
  randomSeed(analogRead(0));
  //send_interval_ms = millis();
}

void send_reply(int reply_type) {
if (hasWifi && hasIoTHub)
  {
      StaticJsonDocument<256> msgtosend;  // pre-allocate 256 bytes of memory for the json message
      msgtosend["message_id"] = received_msg_id;
      msgtosend["timestamp"] = UTC.dateTime(ISO8601);
      msgtosend["message_type"] = reply_type;
      msgtosend["device_id"] = "geniale board 2";
      msgtosend["iot_module_software_version"] = "0.1";
      msgtosend["SL2"] = digitalRead(SL2_GPIO);
      msgtosend["SL3"] = digitalRead(SL3_GPIO);
      msgtosend["ST1"] = read_temperature();
      msgtosend["EV1"] = EV1_status;      
      msgtosend["R1"] = R1_status;
      msgtosend["PC1"] = PC1_status;

      char out[256];
      int msgsize =serializeJson(msgtosend, out);
      //Serial.println(msgsize);
      Serial.println("Replying to HUB with:");
      Serial.println(out);
      EVENT_INSTANCE* message = Esp32MQTTClient_Event_Generate(out, MESSAGE);
      Esp32MQTTClient_SendEventInstance(message);
  }
}


void loop() {
  Esp32MQTTClient_Check();
  if(new_request){
    new_request = 0;
    switch (received_msg_type)  {
      case SET_VALUES: 
        digitalWrite(EV1_GPIO, EV1_status);                         //Set contact state depending on messages
        digitalWrite(R1_GPIO, R1_status);
        send_reply(HUB_ACK);
        break;
      case STATUS:
        send_reply(STATUS);
        break;
      default:
        Serial.println("Invalid message type!");
        break;
    }
  }
}