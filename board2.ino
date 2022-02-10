#include <WiFi.h>
#include "Esp32MQTTClient.h"
#include <WiFiManager.h> 
#include <ArduinoJson.h>
#include <ezTime.h>     
#include "math.h"

#define DEBUG true // flag to turn on/off debugging over serial monitor
#define DEBUG_SERIAL if(DEBUG)Serial

//// Actuators status////
int PC1_status = 0;                                        //status of the recirculation pump (0: OFF, 1: ON)
int EV1_status = 0;                                        //status of the electrovalve 1 (0: OFF, 1: ON)
int R1_status = 0;                                         //status of the boiler resistor (0: OFF, 1: ON)
//// Sensors status ////
int SL2_status, old_SL2_status;
int SL3_status, old_SL3_status;
//// firmware version of the device and device id ////
#define SW_VERSION "0.4"
#define DEVICE_TYPE "SC2"     
#define DEVICE_ID "00000002"
//// Temperature variables and defines ////
#define TEMP_SAMPLES 200                                  // Number of samples taken to give a temperature value
#define TEMP_INTERVAL 10                                  // Interval of time in ms between two successive samples
float ST1_temp, old_ST1_temp;
float legio_temp, old_legio_temp;
int target_loop_temperature = 70;                        // Target temperature to reach 
#define TOLERANCE 0.5                                      // Tolerance for which target temperature is considered reached 
#define WAIT_TIME 30                                     // Time in seconds to wait in order to reach temperature regime
bool flag_alarm_fw = false, flag_alarm_rev = false;      // Flags that disable motor control in case of fully open/closed valve
bool boiler_overtemperature = false;                     // Boiler overtemperature flag (if Temperature is >90 it goes true and R1 is disabled)
bool boiler_too_full = false;                            // If SL2 level is high the electrovalve opening is disabled
bool boiler_empty = false;                               // If boiler is empty, disable heating
int overtemp_warning_counter = 0;
int empty_warning_counter = 0;
int full_warning_counter = 0;
// Steinhart-Hart model coefficients for NTC sensors
// https://www.thinksrs.com/downloads/programs/therm%20calc/ntccalibrator/ntccalculator.html
float A = 1.107430505e-03;
float B = 2.382284132e-04;
float C = 0.6743610533e-07;
// Temperature in K = 1 / (A + B*ln(R_ntc) + C*(ln(R_ntc))^3)
#define R_UP_TEMP 33000.0                                  // Upper resistance of the voltage partitioner used to measure the NTC resistance     

volatile bool new_request = false;                        // flag that tells if a new request has arrived from the hub
volatile int received_msg_id = 0;                         // used for ack mechanism
volatile int received_msg_type = -1;                      // if 0 the HUB wants to know the status of the device
                                                          // if 1 the HUB wants to change the status of the device (with thw values passed in the message)
                                                          // if 2 the device ACKs the HUB
volatile int received_device_id;
// defines for message type 
#define STATUS 0
#define SET_VALUES 1
#define ACK_HUB 2
// STATUS LED HANDLING
#define LED_CHANNEL 0
#define RESOLUTION 8
#define LED_PWM_FREQ 10
#define OFF 0
#define BLINK_5HZ 128
#define ON 255
// MOTOR ROTATION STATES
#define STOP    0
#define FORWARD 1                             // Tends to close the 3-way valve
#define REVERSE 2                             // Tends to open the 3-way valve
// MOTOR CONTROL VARIABLES
int pulses_FWD = 0;                           // used to count the pulses needed to open the valve from initial condition
int pulses_REV = 0;                           // used to count the pulses needed to close the valve from initial condition
#define SAFETY_LIMIT 1.5                      // After 1.5 seconds of no encoder transitions the motor is stopped 
int old_encstatus, encstatus;                 // variables that contain the digital value read from the encoder pin
volatile int motor_counter = 0;               // it is increased at ISR frequency, used for safety limit
////  MICROSOFT AZURE IOT DEFINITIONS   ////
static const char* connectionString = "HostName=geniale-iothub.azure-devices.net;DeviceId=00000002;SharedAccessKey=t3dpZBvo1oW87B29lfbGj7ywM0PH+UhpyyEGFFy8I/c=";
static bool hasIoTHub = false;
static bool hasWifi = false;
#define MESSAGE_MAX_LEN 256
int messageCount = 1;              // tells the number of the sent message
////  I/Os definitions    ////
#define EV1_GPIO   18               // Electrovalve 1 connected to GPIO18
#define PC1_GPIO   17               // Recirculation pump PC1 connected to GPIO17
#define R1_GPIO   19                // Boiler R1 connected to GPIO19
#define ST1_FORCE_GPIO   32         // Voltage force to temperature sensor 1 connected to GPIO32
#define SL2_GPIO   34               // Level sensor 2 connected to GPIO34
#define SL3_GPIO   35               // Level sensor 3 connected to GPIO35
#define ST1_MEASURE_GPIO   39       // Voltage measure temperature sensor 1 (thermistor) connected to GPIO39 (VN)
#define TEMPSENS_LEGIO_GPIO   36    // Voltage measure legiomix's thermistor connected to GPIO36 (VP)
#define CONTACT_LEGIO_GPIO    33    // Legiomix contact (NOT USED)
#define MOTOR_CTRL_IN1_GPIO   25    // IN1 of BD62105 H-bridge motor driver
#define MOTOR_CTRL_IN2_GPIO   26    // IN2 of BD62105 H-bridge motor driver
#define MOTOR_ENCODER_GPIO    16    // Encoder GPIO
#define LED   5                     // Status led connected to GPIO5

// Create a timer to generate an ISR at a defined frequency in order to sample the system
hw_timer_t * timer = NULL;
#define OVF_MS 100                       // The timer interrupt fires every 100 milliseconds
volatile int time2sample_counter = 0; 
volatile int wait_for_regime_counter = 0;// used to implement wait time to win thermal inertia      
volatile int too_full_counter = 0;       // used to sample SL2 sensor every second
#define SAMPLING_TIME 5                  // Sample the sensors every SAMPLING_TIME seconds
bool new_status = false;                 // When it's true a sensor has changed its value and it needs to be sent
volatile bool timetosample = false; 

void IRAM_ATTR onTimer(){            // Timer ISR, called on timer overflow every OVF_MS
  time2sample_counter++;
  motor_counter++;
  wait_for_regime_counter++;
  too_full_counter++;
  // keep electrovalve closed if boiler is too full (checked every second instead of every 5)
  if(too_full_counter >= OVF_MS/10)  
  { 
    too_full_counter = 0;
    if(digitalRead(SL2_GPIO) == HIGH) {             
    EV1_status = 0;                         
    digitalWrite(EV1_GPIO, EV1_status); 
    } 
  }
  if(time2sample_counter >= SAMPLING_TIME*10){
    timetosample = true;
    time2sample_counter = 0;
  } 
}

static void SendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result)
{
  if (result == IOTHUB_CLIENT_CONFIRMATION_OK)
  {
    //DEBUG_SERIAL.println("Send Confirmation Callback finished.");
  }
}

static void MessageCallback(const char* payLoad, int size)
{
  ledcWrite(LED_CHANNEL, ON);
  DEBUG_SERIAL.println("Received message from HUB");
  //DEBUG_SERIAL.println(payLoad);
  if (size < 256) { 
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, payLoad);
    if (error) {
      DEBUG_SERIAL.print(F("deserializeJson() failed: "));
      DEBUG_SERIAL.println(error.f_str());
    }
    else {  
    new_request = true;  
    received_msg_id = doc["message_id"];
    received_msg_type = doc["message_type"];
    received_device_id = doc["device_id"];
      if(received_msg_type == SET_VALUES) {
          PC1_status = doc["PC1"];
          R1_status = doc["R1"];
          EV1_status = doc["EV1"];
          target_loop_temperature = doc["STloop_target"];
      }
    }
  }
  else DEBUG_SERIAL.println("Cannot parse message, too long!");
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
  DEBUG_SERIAL.println(temp);
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

float read_NTC_temperature(int channel){
  digitalWrite(ST1_FORCE_GPIO, HIGH);
  float ADC_val = 0; 
  float NTC_resistance, vin, temperatureK;
  //https://w4krl.com/esp32-analog-to-digital-conversion-accuracy/
  for(int i = 0; i < TEMP_SAMPLES; i++)  { 
      ADC_val += analogRead(channel);
      delay(TEMP_INTERVAL);
  }  
  digitalWrite(ST1_FORCE_GPIO, LOW);
  ADC_val /= TEMP_SAMPLES;
  if(ADC_val>3000)
    vin = (0.5*ADC_val + 1.0874);  
  else
    vin = (0.8*ADC_val + 0.1372);  
  //DEBUG_SERIAL.println(String("GPIO num: ") + String(channel));
  //DEBUG_SERIAL.println(String("Computed ADCn  voltage (mV): ") + String(vin, 2));
  NTC_resistance = - (R_UP_TEMP*vin) / (vin-5000.0);
  //DEBUG_SERIAL.println(String("Computed NTC resistance: ") + String(NTC_resistance, 2));
  float logNTC = log(NTC_resistance);
  temperatureK = 1.0 / (A + B*logNTC + C*logNTC*logNTC*logNTC);
  float temperatureC = temperatureK - 273.15;
  //DEBUG_SERIAL.println(String("NTC Temperature in deg C: ") + String(temperatureC, 2));
  return roundf(temperatureC*10) / 10;
}

void set_motor_direction(int dir){
  switch(dir){
    case STOP:
    digitalWrite(MOTOR_CTRL_IN1_GPIO, LOW);
    digitalWrite(MOTOR_CTRL_IN2_GPIO, LOW);
    break;
    case FORWARD:
    digitalWrite(MOTOR_CTRL_IN1_GPIO, HIGH);
    digitalWrite(MOTOR_CTRL_IN2_GPIO, LOW);
    break;
    case REVERSE:
    digitalWrite(MOTOR_CTRL_IN1_GPIO, LOW);
    digitalWrite(MOTOR_CTRL_IN2_GPIO, HIGH);
    break;
  }
}

void setup() {
  pinMode(EV1_GPIO, OUTPUT);
  pinMode(PC1_GPIO, OUTPUT);     
  pinMode(R1_GPIO, OUTPUT);
  pinMode(ST1_FORCE_GPIO, OUTPUT);
  pinMode(CONTACT_LEGIO_GPIO, OUTPUT);
  pinMode(MOTOR_CTRL_IN1_GPIO, OUTPUT);
  pinMode(MOTOR_CTRL_IN2_GPIO, OUTPUT);
  set_motor_direction(STOP);
  digitalWrite(EV1_GPIO, LOW);                       // Electrovalve is normally closed
  digitalWrite(PC1_GPIO, LOW);
  digitalWrite(R1_GPIO, LOW);
  digitalWrite(CONTACT_LEGIO_GPIO, LOW);
  digitalWrite(ST1_FORCE_GPIO, LOW);
  pinMode(SL2_GPIO, INPUT);
  pinMode(SL3_GPIO, INPUT);
  pinMode(TEMPSENS_LEGIO_GPIO, INPUT);
  pinMode(MOTOR_ENCODER_GPIO, INPUT);
  pinMode(ST1_MEASURE_GPIO, INPUT);                  // ST1 voltage measurement is done by ADC1_3
  old_encstatus = digitalRead(MOTOR_ENCODER_GPIO);
   // configure LED PWM functionalitites
  ledcSetup(LED_CHANNEL, LED_PWM_FREQ, RESOLUTION);
  ledcAttachPin(LED, LED_CHANNEL);                              // Attach PWM module to status LED
  ledcWrite(LED_CHANNEL, BLINK_5HZ);                            // LED initially blinks at 5Hz
  DEBUG_SERIAL.begin(115200);
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
  DEBUG_SERIAL.println("ISR Timer started");
  goto_central_pos_of_3wayvalve();
  DEBUG_SERIAL.println("Starting connecting WiFi.");
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  WiFiManager wm;
  //wm.resetSettings();  // reset settings - wipe stored credentials for testing
  bool res;
  res = wm.autoConnect("GENIALE brd2 setup"); // Generates a pwd-free ap for the user to connect and tell Wi-Fi credentials
  //res = wm.autoConnect("AutoConnectAP","password"); // Generates a pwd-protected ap for the user to connect and tell Wi-Fi credentials

  if(!res) {
      DEBUG_SERIAL.println("Failed to connect to wifi");
      delay(10000);
      ESP.restart();
  } 
  else {
      //if you get here you have connected to the WiFi    
      DEBUG_SERIAL.println("Connected to wifi!");
      ledcWrite(LED_CHANNEL, ON);
      DEBUG_SERIAL.println("Wait for ezTime to get its time synchronized");
	    waitForSync();
      DEBUG_SERIAL.println("UTC Time in ISO8601: " + UTC.dateTime(ISO8601));
      hasWifi = true;
    }
  DEBUG_SERIAL.println("IP address: ");
  DEBUG_SERIAL.println(WiFi.localIP());
  DEBUG_SERIAL.println("IoT Hub init");
  if (!Esp32MQTTClient_Init((const uint8_t*)connectionString, true))
  {
    hasIoTHub = false;
    DEBUG_SERIAL.println("Initializing IoT hub failed.");
    return;
  }
  hasIoTHub = true;
  Esp32MQTTClient_SetSendConfirmationCallback(SendConfirmationCallback);
  Esp32MQTTClient_SetMessageCallback(MessageCallback);
  //Esp32MQTTClient_SetDeviceTwinCallback(DeviceTwinCallback);
  //Esp32MQTTClient_SetDeviceMethodCallback(DeviceMethodCallback);
  randomSeed(analogRead(0));
  //send_interval_ms = millis();
  ledcWrite(LED_CHANNEL, OFF);
  DEBUG_SERIAL.println("Waiting for messages from HUB...");
}

// Search the number of pulses needed to open totally the valve and close totally the valve
void goto_central_pos_of_3wayvalve(){
    DEBUG_SERIAL.println("Searching number of impulses needed to close the valve");
    set_motor_direction(FORWARD);
    delay(500);                                // wait for electromechanical transient to end
    motor_counter = 0;
    while(motor_counter <= 10*SAFETY_LIMIT){
      encstatus = digitalRead(MOTOR_ENCODER_GPIO);
      if(encstatus != old_encstatus){
        pulses_FWD++;
        delay(100);
        motor_counter = 0;
      }
      old_encstatus = encstatus;
    }
    pulses_FWD /= 2;
    DEBUG_SERIAL.println(String("Pulses needed to close: ")+String(pulses_FWD));
    if(motor_counter >= 10*SAFETY_LIMIT)
      DEBUG_SERIAL.println("Valve is fully closed");
    /*  
    DEBUG_SERIAL.println("Searching number of impulses needed to open the valve");
    set_motor_direction(STOP);
    delay(1000);
    set_motor_direction(REVERSE);
    delay(500);
    motor_counter = 0;
    while(motor_counter <= 10*SAFETY_LIMIT){
      encstatus = digitalRead(MOTOR_ENCODER_GPIO);
      if(encstatus != old_encstatus){
        pulses_REV++;
        delay(100);
        motor_counter = 0;
      }
      old_encstatus = encstatus;
    }
    pulses_REV /= 2;                            // HtoL AND LtoH transitions are counted
    DEBUG_SERIAL.println(String("Pulses needed to open: ")+String(pulses_REV));
    if(motor_counter >= 10*SAFETY_LIMIT)
      DEBUG_SERIAL.println("Valve fully open");
    int mean_pulses = pulses_REV / 2;
    */
    int mean_pulses = 15;  // impulses from open to close are 30
    set_motor_direction(STOP);
    delay(1000);
    //set_motor_direction(FORWARD);
    DEBUG_SERIAL.println(String("Moving to half closed half open position ") + String(mean_pulses));
    set_motor_direction(REVERSE);
    delay(1000);
    int index = 0;
    motor_counter = 0;
    while(motor_counter <= 10*SAFETY_LIMIT){
      encstatus = digitalRead(MOTOR_ENCODER_GPIO);
      if(encstatus != old_encstatus){
        index++;
        delay(100);
        motor_counter = 0;
      }
      if(index >= 2*mean_pulses) {
        DEBUG_SERIAL.println(String("Center reached at ")+String(index/2));
        break;
      }
      old_encstatus = encstatus;
    }
    set_motor_direction(STOP);
    if(motor_counter >= 10*SAFETY_LIMIT)
      DEBUG_SERIAL.println("Encoder error!");
}

void move_motor_by_steps(int dir, int steps){
  DEBUG_SERIAL.println(String("Moving motor by ")+String(steps)+String(" steps and in direction ")+String(dir));
    set_motor_direction(dir);
    delay(500);
    int count = 0;
    motor_counter = 0;
    while(motor_counter <= 10*SAFETY_LIMIT && count<steps){
      encstatus = digitalRead(MOTOR_ENCODER_GPIO);
      if(encstatus != old_encstatus){
        count++;
        motor_counter = 0;
        delay(100);
      }
      old_encstatus = encstatus;
    }
    if(motor_counter >= 10*SAFETY_LIMIT) {
      set_motor_direction(STOP);
      switch(dir) {
        case FORWARD:
        flag_alarm_fw = true;               // don't allow the motor to move further forward
        DEBUG_SERIAL.println("Safety limit reached, the valve is fully open");
        break;
        case REVERSE:
        flag_alarm_rev = true;              // don't allow the motor to move further in reverse
        DEBUG_SERIAL.println("Safety limit reached, the valve is fully closed");
        break;
      }        
    }
  wait_for_regime_counter = 0;
}

void send_message(int reply_type, int msgid) {
if (hasWifi && hasIoTHub)
  {
      StaticJsonDocument<256> msgtosend;  // pre-allocate 256 bytes of memory for the json message
      msgtosend["message_id"] = msgid;
      msgtosend["timestamp"] = UTC.dateTime(ISO8601);
      msgtosend["message_type"] = reply_type;
      msgtosend["device_type"] = DEVICE_TYPE;
      msgtosend["device_id"] = DEVICE_ID;
      msgtosend["iot_module_software_version"] = SW_VERSION;
      msgtosend["SL2"] = SL2_status;
      msgtosend["SL3"] = SL3_status;
      msgtosend["ST1"] = ST1_temp;
      msgtosend["STloop"] = legio_temp;
      msgtosend["STloop_target"] = target_loop_temperature;
      msgtosend["EV1"] = EV1_status;      
      msgtosend["R1"] = R1_status;
      msgtosend["PC1"] = PC1_status;

      char out[256];
      int msgsize =serializeJson(msgtosend, out);
      //DEBUG_SERIAL.println(msgsize);

      EVENT_INSTANCE* message = Esp32MQTTClient_Event_Generate(out, MESSAGE);
      Esp32MQTTClient_SendEventInstance(message);      
      DEBUG_SERIAL.println("Message sent to HUB:");
      DEBUG_SERIAL.println(out);
      ledcWrite(LED_CHANNEL, OFF);
  }
}


void loop() {
  Esp32MQTTClient_Check();
  if(new_request == true){            // received message from HUB
    new_request = false;
    switch (received_msg_type) {
      case SET_VALUES:                    
        digitalWrite(EV1_GPIO, EV1_status);        
        digitalWrite(R1_GPIO, R1_status);  
        digitalWrite(PC1_GPIO, PC1_status);
        send_message(ACK_HUB, received_msg_id);
        break;
      case STATUS:
        send_message(STATUS, received_msg_id);
        break;
      default:
        DEBUG_SERIAL.println("Invalid message type!");
        ledcWrite(LED_CHANNEL, OFF);
        break;
    }
  }
  if(timetosample == true) {             // sensor values are sampled every SAMPLING_TIME seconds
    timetosample = false;
    // Read status of sensors  //
    SL2_status = digitalRead(SL2_GPIO);
    boiler_too_full = (SL2_status == HIGH) ? true : false;
    if(boiler_too_full == true)                // keep electrovalve closed if boiler is too full    
    {
        EV1_status = 0;                         
        digitalWrite(EV1_GPIO, EV1_status);
        full_warning_counter++;
        if(full_warning_counter > 10){
          full_warning_counter = 0;
          DEBUG_SERIAL.println("Boiler is too full! EV1 is disabled");
        }  
    }
    SL3_status = digitalRead(SL3_GPIO);
    boiler_empty = (SL3_status == HIGH) ? false : true;
    if(boiler_empty == true)
    {
      R1_status = 0;   
      digitalWrite(R1_GPIO, R1_status);  
      empty_warning_counter++;
      if(empty_warning_counter > 10){
        empty_warning_counter = 0;
        DEBUG_SERIAL.println("Boiler is empty, disabling heating");
      }
      
    }
    ST1_temp = read_NTC_temperature(ST1_MEASURE_GPIO); 
    boiler_overtemperature = (ST1_temp >= 85) ? true : false; // Maximum input temperature of legiomix's 3-way valve is 90 degC
    if(boiler_overtemperature == true)         // disable boiler heater if temperature is too hot
    {
        R1_status = 0;   
        digitalWrite(R1_GPIO, R1_status); 
        overtemp_warning_counter++;
        if(overtemp_warning_counter > 10){
          overtemp_warning_counter = 0;
          DEBUG_SERIAL.println("Water in boiler is too hot, disabling heating");
        }   
    }
    legio_temp = read_NTC_temperature(TEMPSENS_LEGIO_GPIO);    // read temperature of mixed water exiting from legiomix
    if( SL2_status != old_SL2_status || SL3_status != old_SL3_status || ST1_temp != old_ST1_temp || legio_temp != old_legio_temp)  new_status = true;
    old_SL2_status = SL2_status;
    old_SL3_status = SL3_status;
    old_ST1_temp = ST1_temp;
    old_legio_temp = legio_temp;
  }
  if(new_status == true) {                 // the value read from one or more sensors is changed, notify the HUB
  new_status = false;
  send_message(STATUS, messageCount);
  messageCount++;
  }
  // Legiomix control routine 
  // Open valve ---> HOT water. REVERSE tends to open the valve
  // Closed valve ---> COLD water. FORWARD tends to close the valve
  if(legio_temp > float(target_loop_temperature+TOLERANCE) && wait_for_regime_counter >= 10*WAIT_TIME && flag_alarm_fw == false && PC1_status == 1){
    move_motor_by_steps(FORWARD, 1);   // Arguments: direcion, number of steps in that direction   
    }
  else if(legio_temp < float(target_loop_temperature-TOLERANCE) && wait_for_regime_counter >= 10*WAIT_TIME && flag_alarm_rev == false && PC1_status == 1){
    move_motor_by_steps(REVERSE, 1);
  }
  else 
    set_motor_direction(STOP);    // temperature is ok, don't move the motor
}