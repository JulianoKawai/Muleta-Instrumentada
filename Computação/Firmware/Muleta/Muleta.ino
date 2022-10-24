#include <I2Cdev.h>
//#include <MPU6050_6Axis_MotionApps20.h> 
#include <MPU6050_6Axis_MotionApps_V6_12.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>

#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

// Variables for test data
int int_value;
float float_value;
bool bool_value = true;
 
// MAC Address of responder - edit as requiredcy
uint8_t broadcastAddress[] = {0x24, 0xD7, 0xEB, 0x11, 0xC7, 0x88};

// Define a data structure
//typedef struct struct_message {
//  char a[32];
//  int b;
//  float c;
//  bool d;
//} struct_message;

typedef struct struct_message {
  float lc1;
  float lc2;
  float lc3;
  float omgx;
  float omgy;
  float omgz;
}struct_message;

// Create a structured object
struct struct_message infos ={.lc1=0.0,.lc2=0.0 ,.lc3=0.0,.omgx=0.0,.omgy=0.0, .omgz=0.0};
 
// Peer info
esp_now_peer_info_t peerInfo;
 
// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//  Serial.print("\r\nLast Packet Send Status:\t");
//  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

byte display_pins[] = {27,26,25,33,32,14,13};

int count = 0;

void TaskBlink( void *pvParameters);

void TaskIMU(void *pvParameters);

void TaskSend(void *pvParameters);

void display_num(int num);

void TaskStrains(void *pvParameters);

void setup() {
  
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  IMU_setup();
  Strain_Gauge_setup();
  
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the send callback
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(33, OUTPUT);  
  pinMode(32, OUTPUT); 
  
  
  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskBlink
    ,  "TaskBlink"   // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskSend
    ,  "Send"
    ,  1024  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskIMU
    ,  "IMU"
    ,  2048  // Stack size
    ,  NULL
    ,  3  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(
    TaskStrains
    ,  "Strain"
    ,  2048  // Stack size
    ,  NULL
    ,  4  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskBlink(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    vTaskDelay(1000);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskSend(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  for (;;)
  {    
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &infos, sizeof(infos));
     
    if (result == ESP_OK) {
      //Serial.println("Sending confirmed");
    }
    else {
      Serial.println("Sending error");
    }
    vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskIMU(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  for (;;) // A Task shall never return or exit.
  {
    IMU_loop();
    vTaskDelay(20);
  }
}

void TaskStrains(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  for (;;) // A Task shall never return or exit.
  {
    Strain_Gauge_loop();
    vTaskDelay(20);
  }
}
