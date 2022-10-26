//-------------------------------------------------------------------------------------
// HX711_ADC.h
// Arduino master library for HX711 24-Bit Analog-to-Digital Converter for Weigh Scales
// Olav Kallhovd sept2017
// Tested with      : HX711 asian module on channel A and YZC-133 3kg load cell
// Tested with MCU  : Arduino Nano
//-------------------------------------------------------------------------------------
// This is an example sketch on how to use this library for two ore more HX711 modules
// Settling time (number of samples) and data filtering can be adjusted in the config.h file

//pins:
const int HX711_dout_1 = 15;  //mcu > HX711 no 1 dout pin
const int HX711_sck_1 = 4;    //mcu > HX711 no 1 sck pin
const int HX711_dout_2 = 5;   //mcu > HX711 no 2 dout pin
const int HX711_sck_2 = 23;   //mcu > HX711 no 2 sck pin
const int HX711_dout_3 = 18;  //mcu > HX711 no 2 dout pin
const int HX711_sck_3 = 19;   //mcu > HX711 no 2 sck pin

const int samples = 1;

//HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1);  //HX711 1
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2);  //HX711 2
HX711_ADC LoadCell_3(HX711_dout_3, HX711_sck_3);  //HX711 3

unsigned long t = 0;

void Strain_Gauge_setup() {
  Serial.println();
  Serial.println("Starting...");

  float calibrationValue_1;  // calibration value load cell 1
  float calibrationValue_2;  // calibration value load cell 2
  float calibrationValue_3;  // calibration value load cell 3

  calibrationValue_1 = 696.0;
  calibrationValue_2 = 733.0;
  calibrationValue_3 = 733.0;


  LoadCell_1.begin();
  LoadCell_2.begin();
  LoadCell_3.begin();

  LoadCell_1.setSamplesInUse(samples);
  LoadCell_2.setSamplesInUse(samples);
  LoadCell_3.setSamplesInUse(samples);

  //LoadCell_1.setReverseOutput();
  //LoadCell_2.setReverseOutput();
  //LoadCell_3.setReverseOutput();

  unsigned long stabilizingtime = 2000;  // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                  //set this to false if you don't want tare to be performed in the next step
  byte loadcell_1_rdy = 0;
  byte loadcell_2_rdy = 0;
  byte loadcell_3_rdy = 0;
  while ((loadcell_1_rdy + loadcell_2_rdy + loadcell_3_rdy) < 3) {  //run startup, stabilization and tare, both modules simultaniously
    if (!loadcell_1_rdy) loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
    if (!loadcell_2_rdy) loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
    if (!loadcell_3_rdy) loadcell_3_rdy = LoadCell_3.startMultiple(stabilizingtime, _tare);
  }
  if (LoadCell_1.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations");
  }
  if (LoadCell_2.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations");
  }
  if (LoadCell_3.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.3 wiring and pin designations");
  }
  LoadCell_1.setCalFactor(calibrationValue_1);  // user set calibration value (float)
  LoadCell_2.setCalFactor(calibrationValue_2);  // user set calibration value (float)
  LoadCell_3.setCalFactor(calibrationValue_3);  // user set calibration value (float)
  Serial.println("Startup is complete");
}

void Strain_Gauge_loop() {
  static boolean newDataReady = 0;

  // check for new data/start next conversion:
  if (LoadCell_1.update()) newDataReady = true;
  LoadCell_1.update();
  LoadCell_2.update();
  LoadCell_3.update();

  //get smoothed value from data set
  if ((newDataReady)) {
    float a = LoadCell_1.getData();
    float b = LoadCell_2.getData();
    float c = LoadCell_3.getData();
    Serial.print("Load_cell 1 output val: ");
    Serial.print(a);
    infos.lc1 = (float)a;
    Serial.print("    Load_cell 2 output val: ");
    Serial.print(b);
    infos.lc2 = (float)b;
    Serial.print("    Load_cell 3 output val: ");
    Serial.println(c);
    infos.lc3 = (float)c;
    newDataReady = 0;
  }
  
  // receive command from serial terminal, send 't' to initiate tare operation:
  // if (Serial.available() > 0) {
  //   char inByte = Serial.read();
  //   if (inByte == 't') {
  //     LoadCell_1.tareNoDelay();
  //     LoadCell_2.tareNoDelay();
  //     LoadCell_3.tareNoDelay();
  //   }
  // }

  // check if last tare operation is complete
  // if (LoadCell_1.getTareStatus() == true) {
  //   Serial.println("Tare load cell 1 complete");
  // }
  // if (LoadCell_2.getTareStatus() == true) {
  //   Serial.println("Tare load cell 2 complete");
  // }
  // if (LoadCell_3.getTareStatus() == true) {
  //   Serial.println("Tare load cell 3 complete");
  // }
}