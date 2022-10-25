#include <I2Cdev.h>
//#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050_6Axis_MotionApps_V6_12.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>

#include <HX711_ADC.h>
#if defined(ESP8266) || defined(ESP32) || defined(AVR)
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

int count = 0;

void TaskBlink(void *pvParameters);

void TaskIMU(void *pvParameters);

void TaskSend(void *pvParameters);

void display_num(int num);

void TaskStrains(void *pvParameters);

void setup() {

  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  IMU_setup();
  Strain_Gauge_setup();
  ESP_NOW_Setup();
  Interface_Setup();

  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskInterface, "TaskInterface"  // A name just for humans
    ,
    1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,
    NULL, 1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,
    NULL, ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskSend, "Send", 1024  // Stack size
    ,
    NULL, 2  // Priority
    ,
    NULL, ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskIMU, "IMU", 2048  // Stack size
    ,
    NULL, 4  // Priority
    ,
    NULL, ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskStrains, "Strain", 2048  // Stack size
    ,
    NULL, 3  // Priority
    ,
    NULL, ARDUINO_RUNNING_CORE);
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop() {
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskInterface(void *pvParameters)  // This is a task.
{
  (void)pvParameters;
  for (;;)  // A Task shall never return or exit.
  {
    Interface_Loop();
    vTaskDelay(250);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskSend(void *pvParameters)  // This is a task.
{
  (void)pvParameters;
  for (;;) {
    Send_Message();
    vTaskDelay(100);
  }
}

void TaskIMU(void *pvParameters)  // This is a task.
{
  (void)pvParameters;
  for (;;)  // A Task shall never return or exit.
  {
    IMU_loop();
    vTaskDelay(20);
  }
}

void TaskStrains(void *pvParameters)  // This is a task.
{
  (void)pvParameters;
  for (;;)  // A Task shall never return or exit.
  {
    Strain_Gauge_loop();
    vTaskDelay(20);
  }
}