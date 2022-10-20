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
 
// MAC Address of responder - edit as required
uint8_t broadcastAddress[] = {0x08, 0x3A, 0xF2, 0xA9, 0x80, 0xDC};

// Define a data structure
typedef struct struct_message {
  char a[32];
  int b;
  float c;
  bool d;
} struct_message;
 
// Create a structured object
struct_message myData;
 
// Peer info
esp_now_peer_info_t peerInfo;
 
// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//  Serial.print("\r\nLast Packet Send Status:\t");
//  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

struct Button {
    const uint8_t PIN;
    uint32_t numberKeyPresses;
    bool pressed;
};

Button button1 = {34, 0, false};
Button button2 = {35, 0, false};

byte display_map[10][7] = {
  {0,0,0,0,0,0,1},  //0
  {1,0,0,1,1,1,1},  //1
  {0,0,1,0,0,1,0},  //2
  {0,0,0,0,1,1,0},  //3
  {1,0,0,1,1,0,0},  //4
  {0,1,0,0,1,0,0},  //5
  {0,1,0,0,0,0,0},  //6
  {0,0,0,1,1,1,1},  //7
  {0,0,0,0,0,0,0},  //8
  {0,0,0,0,1,0,0}   //9
};

byte display_pins[] = {27,26,25,33,32,14,13};

int count = 0;

void display_num(int num)
{
  for (int i = 0;i < 7;i++) 
  {
     digitalWrite(display_pins[i], display_map[num][i]); 
  }
}

void TaskBlink( void *pvParameters );

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
    // Generate a random integer
    int_value = random(1,20);
   
    // Use integer to make a new float
    float_value = 1.3 * int_value;
   
    // Invert the boolean value
    bool_value = !bool_value;
    
    // Format structured data
    strcpy(myData.a, "Welcome to the Workshop!");
    myData.b = int_value;
    myData.c = float_value;
    myData.d = bool_value;
    
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
     
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
