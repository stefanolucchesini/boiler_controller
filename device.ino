/**
 * A simple Azure IoT example for sending telemetry to Iot Hub.
 */

#include <WiFi.h>
#include "Esp32MQTTClient.h"
#include <WiFiManager.h> 
#include "driver/pcnt.h"   //Pulse counter library
#include <ArduinoJson.h>
#include <ezTime.h>     

#define INTERVAL 10000  // IoT message sending interval in ms
#define MESSAGE_MAX_LEN 256

//// PULSE COUNTER MODULE ////
#define PCNT_FREQ_UNIT      PCNT_UNIT_0     // select ESP32 pulse counter unit 0 (out of 0 to 7 indipendent counting units)
                                            // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/pcnt.html
int16_t PulseCounter =     0;                                // pulse counter, max. value is 65536
int OverflowCounter =      0;                                // pulse counter overflow counter
int PCNT_H_LIM_VAL =       30000;                            // upper limit of counting  max. 32767, write +1 to overflow counter, when reached 
uint16_t PCNT_FILTER_VAL=  0;                             // filter (damping, inertia) value for avoiding glitches in the count, max. 1023
pcnt_isr_handle_t user_isr_handle = NULL;                 // user interrupt handler (not used)
int liters = 0;                                           // number of liters that pass through the flux sensor
#define PULSES_PER_LITER 165                              // 165 pulses from the flux sensor tell that a liter has passed

//// Sanitizer level pumps status////
int P1_status = 0;                                        //status of the sanitizer pump 1 (0: OFF, 1: ON)
int P2_status = 0;                                        //status of the sanitizer pump 2 (0: OFF, 1: ON)
//// Chlorine sensor reading variable ////
float chlorine_concentration = 0.1;                         // concentration of chlorine given by crs1 (range: 0.1 ppm - 20 ppm)
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

////  MICROSOFT AZURE IOT DEFINITIONS   ////
static const char* connectionString = "HostName=geniale-iothub.azure-devices.net;DeviceId=00000001;SharedAccessKey=Cn4UylzZVDZD8UGzCTJazR3A9lRLnB+CbK6NkHxCIMk=";
static bool hasIoTHub = false;
static bool hasWifi = false;
//int messageCount = 1;
//static bool messageSending = true;
//static uint64_t send_interval_ms;

////  I/Os definitions    ////
#define PCNT_INPUT_SIG_IO   34       // Flow sensor connected to GPIO34
#define P2_GPIO   33       // Sanitizer pump P2 contact connected to GPIO33
#define P1_GPIO   32       // Sanitizer pump P1 contact connected to GPIO32
#define SL1_GPIO  35       // Level sensor connected to GPIO35
#define LED   5           // Status led connected to GPIO5

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
          P1_status = doc["P1"];
          P2_status = doc["P2"];
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

//// PULSE COUNTER OVERFLOW ISR ////
  void IRAM_ATTR CounterOverflow(void *arg) {                  // Interrupt for overflow of pulse counter
    OverflowCounter = OverflowCounter + 1;                     // increase overflow counter
    PCNT.int_clr.val = BIT(PCNT_FREQ_UNIT);                    // clean overflow flag
    pcnt_counter_clear(PCNT_FREQ_UNIT);                        // zero and reset of pulse counter unit
  }

  void initPulseCounter (){                                    // initialise pulse counter
    pcnt_config_t pcntFreqConfig = { };                        // Instance of pulse counter
    pcntFreqConfig.pulse_gpio_num = PCNT_INPUT_SIG_IO;         // pin assignment for pulse counter
    pcntFreqConfig.pos_mode = PCNT_COUNT_INC;                  // count rising edges (=change from low to high logical level) as pulses
    pcntFreqConfig.neg_mode = PCNT_COUNT_DIS;                  // do nothing on falling edges
    pcntFreqConfig.counter_h_lim = PCNT_H_LIM_VAL;             // set upper limit of counting 
    pcntFreqConfig.unit = PCNT_FREQ_UNIT;                      // select ESP32 pulse counter unit 0
    pcntFreqConfig.channel = PCNT_CHANNEL_0;                   // select channel 0 of pulse counter unit 0
    pcnt_unit_config(&pcntFreqConfig);                         // configure rigisters of the pulse counter
  
    pcnt_counter_pause(PCNT_FREQ_UNIT);                        // pause pulse counter unit
    pcnt_counter_clear(PCNT_FREQ_UNIT);                        // zero and reset of pulse counter unit
  
    pcnt_event_enable(PCNT_FREQ_UNIT, PCNT_EVT_H_LIM);         // enable event for interrupt on reaching upper limit of counting
    pcnt_isr_register(CounterOverflow, NULL, 0, &user_isr_handle);  // configure register overflow interrupt handler
    pcnt_intr_enable(PCNT_FREQ_UNIT);                          // enable overflow interrupt

    pcnt_set_filter_value(PCNT_FREQ_UNIT, PCNT_FILTER_VAL);    // set damping, inertia 
    pcnt_filter_enable(PCNT_FREQ_UNIT);                        // enable counter glitch filter (damping)
  
    pcnt_counter_resume(PCNT_FREQ_UNIT);                       // resume counting on pulse counter unit
  }
   
  void Reset_PCNT() {                                          // function resetting counter 
    OverflowCounter = 0;                                       // set overflow counter to zero
    pcnt_counter_clear(PCNT_FREQ_UNIT);                        // zero and reset of pulse counter unit
  }

  void get_liters(){                                    // converts the pulses received from fl1 to liters
      pcnt_get_counter_value(PCNT_FREQ_UNIT, &PulseCounter);     // get pulse counter value - maximum value is 16 bit
      liters = ( OverflowCounter*PCNT_EVT_H_LIM + PulseCounter ) / PULSES_PER_LITER;
  }

void setup() {
  pinMode(PCNT_INPUT_SIG_IO, INPUT);                  // the output of the flow sensor is open collector (MUST USE EXTERNAL PULL UP!!)
  pinMode(SL1_GPIO, INPUT);
  pinMode(P1_GPIO, OUTPUT);     
  pinMode(P2_GPIO, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(P1_GPIO, LOW);                         //Set contacts initially open
  digitalWrite(P2_GPIO, LOW);
  digitalWrite(LED, LOW);
  initPulseCounter();
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
      msgtosend["device_id"] = "geniale board 1";
      msgtosend["iot_module_software_version"] = "0.1";
      msgtosend["SL1"] = digitalRead(SL1_GPIO);
      msgtosend["CRS1"] = chlorine_concentration;
      get_liters();
      msgtosend["FL1"] = liters;
      msgtosend["P1"] = P1_status;
      msgtosend["P2"] = P2_status;
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
        digitalWrite(P1_GPIO, P1_status);                         //Set contact state depending on messages
        digitalWrite(P2_GPIO, P2_status);
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