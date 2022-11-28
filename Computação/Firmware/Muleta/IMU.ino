#define OUTPUT_READABLE_WORLDACCEL
MPU6050 mpu;

uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
unsigned long dt;
unsigned long old_time;
bool first_run = true;
float old_ypr[3];
float imu_omega[3];
double tip_position[3];
double imu_position[3];
float tip_velocity[3];
float *tip_relposition = new float[3];
double aaWorld_offset[3];
double aaWorld_new[3];
double aaWorld_old[3];
double imu_velocity[3];
double imu_velocity_old[3];

double vx[3];
double vy[3];
double vz[3];

// MEDIR VALORES

float min_lz = 1;
float lx = -0.03;
float ly = -0.01;

float acc_threshold = 30000;
float grf_threshold = 2000;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

void IMU_setup() {
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  delay(100);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(39);
  mpu.setYGyroOffset(56);
  mpu.setZGyroOffset(28);
  mpu.setXAccelOffset(-2478);  // 1788 factory default for my test chip
  mpu.setYAccelOffset(-565);
  mpu.setZAccelOffset(836);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    //mpu.CalibrateAccel(6);
    //mpu.CalibrateGyro(6);
    //mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    mpuIntStatus = mpu.getIntStatus();

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void IMU_loop() {
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet

#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    aa.x = aa.x/2;   
    aa.y = aa.y/2;  
    aa.z = aa.z/2;
    mpu.dmpGetGravity(&gravity, &q); 
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //Serial.print("ypr\t");
    //Serial.print(ypr[0] * 180 / M_PI);
    //Serial.print("\t");
    //Serial.print(ypr[1] * 180 / M_PI);
    //Serial.print("\t");
    //Serial.println(ypr[2] * 180 / M_PI);
    infos.omgx = (float) ypr[0]; //* 180 / M_PI;
    infos.omgy = (float) ypr[1]; //* 180 / M_PI;
    infos.omgz = (float) ypr[2]; //* 180 / M_PI;
    // Serial.print("aworld\t");
    // Serial.print(aaWorld.x);
    // Serial.print("\t");
    // Serial.print(aaWorld.y);
    // Serial.print("\t");
    // Serial.println(aaWorld.z);   
#endif
  }
}

void get_tip_position() {
  
  if (first_run == true) {    
    old_time = millis();
    old_ypr[0] = ypr[0];
    old_ypr[1] = ypr[1];
    old_ypr[2] = ypr[2]; 
      
    imu_velocity[0] = 0;
    imu_velocity[1] = 0;
    imu_velocity[2] = 0;
    
    tip_position[0] = 0;
    tip_position[1] = 0;
    tip_position[2] = 0;
    
    aaWorld_offset[0] = 0;
    aaWorld_offset[1] = 0;
    aaWorld_offset[2] = 0;    
    
    vx[0] = 0.0;
    vx[1] = 0.0;
    vx[2] = 0.0;
    vy[0] = 0.0;
    vy[1] = 0.0;
    vy[2] = 0.0;
    vz[0] = 0.0;
    vz[1] = 0.0;
    vz[2] = 0.0;
    first_run = false;    
  }

  else {
    dt = millis() - old_time; 
    old_time = millis();   
    
    aaWorld_old[0] = aaWorld_new[0];
    aaWorld_old[1] = aaWorld_new[1];
    aaWorld_old[2] = aaWorld_new[2];   

    imu_velocity_old[0] = imu_velocity[0];
    imu_velocity_old[1] = imu_velocity[1];
    imu_velocity_old[2] = imu_velocity[2];
    
    // aaWorld_new[0] = butterworth_x(aaWorld.x);
    // aaWorld_new[1] = butterworth_y(aaWorld.y);
    // aaWorld_new[2] = butterworth_z(aaWorld.z);

    if (gravity.x>0)
      aaWorld_new[0] = butterworth_x(0.5*aa.x - gravity.x*8450);
    else
      aaWorld_new[0] = butterworth_x(0.5*aa.x - gravity.x*7860);
    if (gravity.y>0)
      aaWorld_new[1] = butterworth_y(0.5*aa.y - gravity.y*8250);
    else
      aaWorld_new[1] = butterworth_y(0.5*aa.y - gravity.y*8080);
    aaWorld_new[2] = butterworth_z(0.5*aa.z - gravity.z*8295);

    imu_velocity[0] = imu_velocity[0] + (((aaWorld_new[0] + aaWorld_old[0]) / 2) * dt)/1000000;
    imu_velocity[1] = imu_velocity[1] + (((aaWorld_new[1] + aaWorld_old[1]) / 2) * dt)/1000000;
    imu_velocity[2] = imu_velocity[2] + (((aaWorld_new[2] + aaWorld_old[2]) / 2) * dt)/1000000; 

    imu_position[0] = imu_position[0] + (imu_velocity[0] + imu_velocity_old[0])/2 * dt;
    imu_position[1] = imu_position[1] + (imu_velocity[1] + imu_velocity_old[1])/2 * dt;
    imu_position[2] = imu_position[2] + (imu_velocity[2] + imu_velocity_old[2])/2 * dt;

    // imu_omega[0] = (ypr[2] - old_ypr[2]) / dt;
    // imu_omega[1] = (ypr[1] - old_ypr[1]) / dt;
    // imu_omega[2] = (ypr[0] - old_ypr[0]) / dt;

    // tip_relposition = get_tip_relposition(lx,ly,len_select*L_furos + min_lz);

    // tip_velocity[0] = imu_velocity[0] + imu_omega[1]*tip_relposition[2] - imu_omega[2]*tip_relposition[1];
    // tip_velocity[1] = imu_velocity[1] - imu_omega[0]*tip_relposition[2] + imu_omega[2]*tip_relposition[0];
    // tip_velocity[2] = imu_velocity[2] + imu_omega[0]*tip_relposition[1] - imu_omega[1]*tip_relposition[0];  
      
    // tip_position[0] = tip_position[0] + tip_velocity[0] * dt;
    // tip_position[1] = tip_position[1] + tip_velocity[1] * dt;
    // tip_position[2] = tip_position[2] + tip_velocity[2] * dt;
    
    if (aa.z> acc_threshold) { //aaWorld.z < acc_threshold && infos.lc3 < grf_threshold
      // tip_position[0] = 0;
      // tip_position[1] = 0;
      // tip_position[2] = 0;
      imu_position[0] = 0;
      imu_position[1] = 0;
      imu_position[2] = 0;
      // tip_velocity[0] = 0;
      // tip_velocity[1] = 0;
      // tip_velocity[2] = 0;
      // imu_velocity[0] = 0;
      // imu_velocity[1] = 0; 
      // imu_velocity[2] = 0;   
      // vx[0] = 0;
      // vx[1] = 0;
      // vy[0] = 0;
      // vy[1] = 0;   
      // vz[0] = 0;
      // vz[1] = 0;         
    }
    
    old_ypr[0] = ypr[0];
    old_ypr[1] = ypr[1];
    old_ypr[2] = ypr[2];

    // Serial.print(imu_position[0]);
    // Serial.print("\t");
    // Serial.print(imu_position[1]);
    // Serial.print("\t");
    // Serial.println(imu_position[2]);   

    // Serial.print(imu_velocity[0]);
    // Serial.print("\t");
    // Serial.print(imu_velocity[1]);
    // Serial.print("\t");
    // Serial.println(imu_velocity[2]);    

    // Serial.print(aaReal.x);
    // Serial.print("\t");
    // Serial.print(aaReal.y);
    // Serial.print("\t");
    // Serial.println(aaReal.z); 

    // Serial.print(aaWorld_new[0]);
    // Serial.print("\t");
    // Serial.print(aaWorld_new[1]);
    // Serial.print("\t");
    // Serial.println(aaWorld_new[2]);    

  }
}

float *get_tip_relposition(float dx, float dy, float dz) {
  return get_grf_fixed_yaw(dx,dy,dz);
}

float butterworth_x(float x) {
  vx[0] = vx[1];
  vx[1] = vx[2];
  vx[2] = (9.944617889581948145e-1 * x)
				 + (-0.98895424993312641693 * vx[0])
				 + (1.98889290589965295197 * vx[1]);
  return (vx[0] + vx[2]) - 2 * vx[1];
}

float butterworth_y(float x) {
  vy[0] = vy[1];
  vy[1] = vy[2];
  vy[2] = (9.944617889581948145e-1 * x)
				 + (-0.98895424993312641693 * vy[0])
				 + (1.98889290589965295197 * vy[1]);
  return (vy[0] + vy[2]) - 2 * vy[1];
}

float butterworth_z(float x) {
  vz[0] = vz[1];
  vz[1] = vz[2];
  vz[2] = (9.944617889581948145e-1 * x)
				 + (-0.98895424993312641693 * vz[0])
				 + (1.98889290589965295197 * vz[1]);
  return (vz[0] + vz[2]) - 2 * vz[1];
}
