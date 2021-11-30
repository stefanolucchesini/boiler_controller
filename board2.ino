#include <WiFi.h>
#include "Esp32MQTTClient.h"
#include <WiFiManager.h> 
#include <ArduinoJson.h>
#include <ezTime.h>     
#include "esp_adc_cal.h"
#include "math.h"

//// Actuators status////
int PC1_status = 0;                                        //status of the recirculation pump (0: OFF, 1: ON)
int EV1_status = 0;                                        //status of the electrovalve 1 (0: OFF, 1: ON)
int R1_status = 0;                                         //status of the boiler resistor (0: OFF, 1: ON)
//// Sensors status ////
volatile int SL2_status;
volatile int old_SL2_status;
volatile int SL3_status;
volatile int old_SL3_status;
volatile float ST1_temp;
volatile float old_ST1_temp;
//// firmware version of the device and device id ////
#define SW_VERSION "0.1"
#define DEVICE_ID "geniale board 2"   
//// Other handy variables ////
#define TEMP_SAMPLES 200                                  // Number of samples taken to give a temperature value
#define TEMP_INTERVAL 10                                   // Interval of time in ms between two successive samples
volatile int new_request = 0;                             // flag that tells if a new request has arrived from the hub
volatile int received_msg_id = 0;                         // used for ack mechanism
volatile int received_msg_type = -1;                      // if 0 the HUB wants to know the status of the device
                                                          // if 1 the HUB wants to change the status of the device (with thw values passed in the message)
                                                          // if 2 the device ACKs the HUB
// defines for message type 
#define STATUS 0
#define SET_VALUES 1
#define ACK_HUB 2
// Variables for voltages corresponding to temperature ranges, for PT1000:
//1.25V   corresponds to 0 deg C
//1.4518V  corresponds to 100 deg C
// Temperature = M * ADC + Q
float M = 0.408163;
float Q = -564.081633;
// STATUS LED HANDLING
#define LED_CHANNEL 0
#define RESOLUTION 8
#define LED_PWM_FREQ 10
#define OFF 0
#define BLINK_5HZ 128
#define ON 255
////  MICROSOFT AZURE IOT DEFINITIONS   ////
static const char* connectionString = "HostName=geniale-iothub.azure-devices.net;DeviceId=00000001;SharedAccessKey=Cn4UylzZVDZD8UGzCTJazR3A9lRLnB+CbK6NkHxCIMk=";
static bool hasIoTHub = false;
static bool hasWifi = false;
//#define INTERVAL 10000             // IoT message sending interval in ms
#define MESSAGE_MAX_LEN 256
int messageCount = 1;              // tells the number of the sent message
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

// Create a timer to generate an ISR at a defined frequency in order to sample the system
hw_timer_t * timer = NULL;
#define OVF_MS 5000                      // The timer interrupt fires every 5 second
bool new_status = false;       // When it's true a sensor has changed its value and it needs to be sent
volatile bool timetosample = false; 

void IRAM_ATTR onTimer(){            // Timer ISR, called on timer overflow every OVF_MS
  timetosample = true;
}

static void SendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result)
{
  if (result == IOTHUB_CLIENT_CONFIRMATION_OK)
  {
    //Serial.println("Send Confirmation Callback finished.");
  }
}

static void MessageCallback(const char* payLoad, int size)
{
  ledcWrite(LED_CHANNEL, ON);
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
    // acquire 100 samples and compute mean
    for(int i = 0; i < TEMP_SAMPLES; i++)  {
        float val = analogRead(ST1_MEASURE_GPIO);
        float temperaturec = M * val + Q;
        mean += temperaturec;
        delay(TEMP_INTERVAL); 
      }
  digitalWrite(ST1_FORCE_GPIO, LOW);
  //Serial.println(String("Temperature in deg C: ") + String(mean/100, 2));
  return roundf(mean/(TEMP_SAMPLES/10)) / 10;   //return the temperature with a single decimal place
}

void setup() {
  pinMode(EV1_GPIO, OUTPUT);
  pinMode(PC1_GPIO, OUTPUT);     
  pinMode(R1_GPIO, OUTPUT);
  pinMode(SL2_GPIO, INPUT);
  pinMode(SL3_GPIO, INPUT);
  pinMode(ST1_FORCE_GPIO, OUTPUT);
  pinMode(ST1_MEASURE_GPIO, INPUT);                  // ST1 voltage measurement is done by ADC1_3
  digitalWrite(EV1_GPIO, LOW);                       // Electrovalve is normally closed
  digitalWrite(PC1_GPIO, LOW);
  digitalWrite(R1_GPIO, LOW);
  digitalWrite(ST1_FORCE_GPIO, LOW);
   // configure LED PWM functionalitites
  ledcSetup(LED_CHANNEL, LED_PWM_FREQ, RESOLUTION);
  ledcAttachPin(LED, LED_CHANNEL);                              // Attach PWM module to status LED
  ledcWrite(LED_CHANNEL, BLINK_5HZ);                            // LED initially blinks at 5Hz
  
  Serial.begin(115200);
  Serial.println("ESP32 Device");
  Serial.println("Initializing...");
  Serial.println(" > WiFi");
  Serial.println("Starting connecting WiFi.");

  delay(10);

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  WiFiManager wm;
  //wm.resetSettings();  // reset settings - wipe stored credentials for testing
  bool res;
  res = wm.autoConnect("GENIALE brd2 setup"); // Generates a pwd-free ap for the user to connect and tell Wi-Fi credentials
  //res = wm.autoConnect("AutoConnectAP","password"); // Generates a pwd-protected ap for the user to connect and tell Wi-Fi credentials

  if(!res) {
      Serial.println("Failed to connect to wifi");
      delay(10000);
      ESP.restart();
  } 
  else {
      //if you get here you have connected to the WiFi    
      Serial.println("Connected to wifi!");
      ledcWrite(LED_CHANNEL, ON);
      // Wait for ezTime to get its time synchronized
	    waitForSync();
      Serial.println("UTC Time in ISO8601: " + UTC.dateTime(ISO8601));
      hasWifi = true;
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
  randomSeed(analogRead(0));
  //send_interval_ms = millis();
  /* Use 1st timer of 4 */
  /* 1 tick take 1/(80MHZ/80) = 1us so we set divider 80 and count up */
  timer = timerBegin(0, 80, true);
  /* Attach onTimer function to our timer */
  timerAttachInterrupt(timer, &onTimer, true);
  /* Set alarm to call onTimer function every OVF_MS milliseconds. 
  1 tick is 1us*/
  /* Repeat the alarm (third parameter) */
  timerAlarmWrite(timer, 1000*OVF_MS, true);
  /* Start an alarm */
  timerAlarmEnable(timer);
  Serial.println("ISR Timer started");
  ledcWrite(LED_CHANNEL, OFF);
  Serial.println("Waiting for messages from HUB...");
}

void send_message(int reply_type, int msgid) {
if (hasWifi && hasIoTHub)
  {
      StaticJsonDocument<256> msgtosend;  // pre-allocate 256 bytes of memory for the json message
      msgtosend["message_id"] = msgid;
      msgtosend["timestamp"] = UTC.dateTime(ISO8601);
      msgtosend["message_type"] = reply_type;
      msgtosend["device_id"] = DEVICE_ID;
      msgtosend["iot_module_software_version"] = SW_VERSION;
      msgtosend["SL2"] = SL2_status;
      msgtosend["SL3"] = SL3_status;
      msgtosend["ST1"] = ST1_temp;
      msgtosend["EV1"] = EV1_status;      
      msgtosend["R1"] = R1_status;
      msgtosend["PC1"] = PC1_status;

      char out[256];
      int msgsize =serializeJson(msgtosend, out);
      //Serial.println(msgsize);
      Serial.println("Sending message to HUB:");
      Serial.println(out);
      EVENT_INSTANCE* message = Esp32MQTTClient_Event_Generate(out, MESSAGE);
      Esp32MQTTClient_SendEventInstance(message);
      ledcWrite(LED_CHANNEL, OFF);
  }
}


void loop() {
  Esp32MQTTClient_Check();
  if(new_request){
    new_request = 0;
    switch (received_msg_type)  {
      case SET_VALUES: 
        send_message(ACK_HUB, received_msg_id);
        break;
      case STATUS:
        send_message(STATUS, received_msg_id);
        break;
      default:
        Serial.println("Invalid message type!");
        ledcWrite(LED_CHANNEL, OFF);
        break;
    }
  }
    if(new_status == true) {
    new_status = false;
    digitalWrite(EV1_GPIO, EV1_status);                         //Set contact state depending on messages
    digitalWrite(R1_GPIO, R1_status);
    digitalWrite(PC1_GPIO, PC1_status);
    send_message(STATUS, messageCount);
    messageCount++;
    }
    if(timetosample){
      timetosample = 0;
        // Read status of sensors  //
      SL2_status = digitalRead(SL2_GPIO);
      SL3_status = digitalRead(SL3_GPIO);
      ST1_temp = read_temperature();
      if( SL2_status != old_SL2_status || SL3_status != old_SL3_status || ST1_temp != old_ST1_temp ) 
        new_status = true;

      old_SL2_status = SL2_status;
      old_SL3_status = SL3_status;
      old_ST1_temp = ST1_temp;
    }
}