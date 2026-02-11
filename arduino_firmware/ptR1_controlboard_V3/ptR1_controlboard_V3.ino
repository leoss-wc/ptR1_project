#include <Arduino.h>
#include <PCF8575.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>  
#include <math.h>                          
#include <PID_v1.h>
#include <stdlib.h>             // IMU normalized
#include <Wire.h>               // ไลบรารีสำหรับ I2C communication
#include <MPU6050_light.h>           // Library MPU6050
#include <QMC5883LCompass.h>         // Library QMC5883L

#include <ros.h>                            // ไลบรารีสำหรับ ROS communication
#include <tf/transform_broadcaster.h> 
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>            // ใช้สำหรับรับคำสั่งความเร็ว (cmd_vel) จาก ROS
#include <sensor_msgs/Imu.h>                // ใช้สำหรับส่งข้อมูลจาก IMU ไปยัง ROS
#include <sensor_msgs/MagneticField.h>      // เพิ่ม Header Mag
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>                // ใช้สำหรับส่งข้อมูลเป็นข้อความ
#include <std_msgs/UInt32.h>                // ใช้สำหรับส่งค่าประเภท UInt32 ใน ROS
#include <std_msgs/UInt16.h>                // ใช้สำหรับส่งค่าประเภท UInt16 ใน ROS
#include <std_msgs/UInt8.h>                 // ใช้สำหรับส่งค่าประเภท UInt8 ใน ROS
#include <std_msgs/Int16.h>

#define I2C_SDA 8
#define I2C_SCL 9

bool debug_mode = false;
bool manual_mode = false;

// พินของมอเตอร์ไดรเวอร์ (Motor Driver)
  const uint8_t md1_PWMA  = 6; // to esp32 pin
  const uint8_t md1_AIN2  = 12;   // PCF8575 pin
  const uint8_t md1_AIN1  = 11;   // PCF8575 pin
  const uint8_t md1_STBY  = 2;    // PCF8575 pin
  const uint8_t md1_BIN1  = 3;    // PCF8575 pin
  const uint8_t md1_BIN2  = 4;    // PCF8575 pin
  const uint8_t md1_PWMB  = 7; // to esp32 pin

                      
  const uint8_t md2_PWMA = 10; //esp32 pin
  const uint8_t md2_AIN2 = 5;   // PCF8575 pin
  const uint8_t md2_AIN1 = 6;   // PCF8575 pin
  const uint8_t md2_STBY = 7;   // PCF8575 pin
  const uint8_t md2_BIN1 = 8;   // PCF8575 pin
  const uint8_t md2_BIN2 = 9;   //PCF8575 pin
  const uint8_t md2_PWMB = 21; //esp32 pin

// PCF8575 Pins (0-15)
  const uint8_t FL_IN1_PCF = md1_AIN1; const uint8_t FL_IN2_PCF = md1_AIN2;
  const uint8_t FR_IN1_PCF = md1_BIN1;  const uint8_t FR_IN2_PCF = md1_BIN2;
  const uint8_t RL_IN1_PCF = md2_AIN1;  const uint8_t RL_IN2_PCF = md2_AIN2;
  const uint8_t RR_IN1_PCF = md2_BIN1;  const uint8_t RR_IN2_PCF = md2_BIN2;
  const uint8_t STBY_PCF_1 = md1_STBY;  const uint8_t STBY_PCF_2 = md2_STBY;

  const uint8_t FL_PWM_PIN = md1_PWMA;
  const uint8_t FR_PWM_PIN = md1_PWMB;
  const uint8_t RL_PWM_PIN = md2_PWMA;
  const uint8_t RR_PWM_PIN = md2_PWMB;

//Encoder pin
  // Motor 1: Front-Left (FL)
    const uint8_t ENCODER_FL_A = 13;
    const uint8_t ENCODER_FL_B = 14;

    // Motor 3: Front-Right (FR)
    const uint8_t ENCODER_FR_A = 12;
    const uint8_t ENCODER_FR_B = 11;

    // Motor 4: Rear-Left (RL)
    const uint8_t ENCODER_RL_A = 17;
    const uint8_t ENCODER_RL_B = 18;

    // Motor 2: Rear-Right (RR)
    const uint8_t ENCODER_RR_A = 16;
    const uint8_t ENCODER_RR_B = 15;

// --- Robot Parameters ---
  const float WHEEL_RADIUS = 0.04; // m
  const float LX = 0.105; // L1 (Half length) m
  const float LY = 0.0825; // L2 (Half width) m
  const float ROBOT_GEOMETRY = LX + LY;
  const float TICKS_PER_REV = 1320.0;
  const float RADS_PER_TICK = (2.0 * PI) / TICKS_PER_REV;

  const int MIN_PWM = 45;

// --- Power variable ---
  const uint8_t CURRENT_SENSOR_PIN = 4; // GPIO ที่ต่อ Current sensor
  const uint8_t VOLTAGE_SENSOR_PIN = 5; // GPIO ที่ต่อ Voltage sensor
  // --- Power sensor calibration Values and variable ---
    // ESP32 ADC 12-bit = 4095
    // V_REF ของ ESP32 ประมาณ 3.3V
    const float ADC_VREF = 3.2; 
    const float ADC_RES = 4095.0;
    const float VOLTAGE_MULTIPLIER = 12.5;
    int current_zero_point = 0;
  //Exponential Moving Average (EMA) 
  float filter_volt = 0; //ของ current sensor
  float filter_amp = 0;  //ของ current sensor
  const float alpha = 0.1; //ของ current sensor
  float filter_amp_adc = 0; // เก็บค่า ADC ที่กรองแล้ว ของ current sensor

// --- PID object and variables ---
  double Kp = 1.5, Ki = 5.0, Kd = 0.05;
  // PID Objects  setpoint(ROS) input(Encoder) output(PID)
  double sp_FL=0, in_FL=0, out_FL=0;
  double sp_FR=0, in_FR=0, out_FR=0;
  double sp_RL=0, in_RL=0, out_RL=0;
  double sp_RR=0, in_RR=0, out_RR=0;

  PID pidFL(&in_FL, &out_FL, &sp_FL, Kp, Ki, Kd, DIRECT);
  PID pidFR(&in_FR, &out_FR, &sp_FR, Kp, Ki, Kd, DIRECT);
  PID pidRL(&in_RL, &out_RL, &sp_RL, Kp, Ki, Kd, DIRECT);
  PID pidRR(&in_RR, &out_RR, &sp_RR, Kp, Ki, Kd, DIRECT);
  // --- Timing(PID Cycle) ---
  unsigned long prevPIDTime = 0;
  //Ramp Filter
  double target_FL = 0, target_FR = 0, target_RL = 0, target_RR = 0;
  const double RAMP_STEP = 5.0; // ค่าความชันการเร่ง (Rad/s ต่อ Loop) ยิ่งน้อยยิ่งนุ่ม


//Encoder variable
  ESP32Encoder encoderFL;
  ESP32Encoder encoderFR;
  ESP32Encoder encoderRL;
  ESP32Encoder encoderRR;
// Servo object and variables
  Servo servoPan;
  Servo servoTilt;
  const uint8_t SERVO_PAN_PIN  = 2;    //X
  const uint8_t SERVO_TILT_PIN = 1;    //Y

  int pos_pan = 1500;  // 1500us = 90 degrees (Center)
  int pos_tilt = 1500;
  int step_servo_x = 50; // Step การขยับ
  int step_servo_y = 50;
  const int SERVO_MIN = 500;
  const int SERVO_MAX = 2500;
// Relay variable
  const uint8_t RELAY1_PCF = 13; 
  const uint8_t RELAY2_PCF = 14;

// --- Global Variables for Odometry ---
  double x_pos = 0.0;
  double y_pos = 0.0;
  double theta = 0.0;
  double linear_x = 0;
  double linear_y = 0;
  double gyro_z = 0;
  geometry_msgs::Quaternion odom_quat;

// --- ROS Globals ---
  ros::NodeHandle nh;
  nav_msgs::Odometry odom_msg;
  sensor_msgs::Imu imu_msg;
  geometry_msgs::TransformStamped t;
  tf::TransformBroadcaster broadcaster;
  sensor_msgs::BatteryState bat_msg;
  std_msgs::String status_msg;
  char base_link[] = "base_link";
  char odom_frame[] = "odom";
  // sensor_msgs::MagneticField mag_msg; // Mag Message

  ros::Publisher odom_pub("/odom", &odom_msg);
  ros::Publisher imu_pub("/imu/data", &imu_msg);
  ros::Publisher battery_pub("/sensor/battery", &bat_msg);
  ros::Publisher status_pub("/robot/status", &status_msg);
  // ros::Publisher mag_pub("/imu/mag", &mag_msg);

  char status_buffer[150]; //ใช้สำหรับ Topic /robot/status

//Sensor Object
  PCF8575 pcf(0x20);
  uint16_t pcf_buffer = 0xFFFF;

  MPU6050 mpu(Wire);
  QMC5883LCompass compass;

//Heading Hold variable
  bool enable_heading_hold = true; // เปิด/ปิด ระบบนี้
  float heading_kp = 2.0;          // ค่า Kp: ยิ่งมาก ยิ่งสู้แรงไถล (ลองเริ่มที่ 2.0 - 5.0)
  double target_heading = 0.0;     // มุมที่เราต้องการล็อกไว้
  bool is_turning = false;         // เช็คว่าตอนนี้กำลังตั้งใจหมุนอยู่ไหม

void setPCFBit(uint8_t pin, bool state);
void setMotorPWM(float pwm, int pin1, int pin2, int pwmPin);
void configPWMPin(uint8_t pin);
void updateOdometryAndIMU(float dt, double v_fl, double v_fr, double v_rl, double v_rr);
void panCallback(const std_msgs::Int16& msg);
void tiltCallback(const std_msgs::Int16& msg);
void pidCallback(const geometry_msgs::Vector3& msg);
void sysCommandCallback(const std_msgs::String& msg);
void cmdVelCallbackAuto(const geometry_msgs::Twist& msg);
void cmdVelCallbackManual(const geometry_msgs::Twist& msg);
// Ros Subscriber
  ros::Subscriber<std_msgs::Int16> subPan("/camera/pan", panCallback);
  ros::Subscriber<std_msgs::Int16> subTilt("/camera/tilt", tiltCallback);
  ros::Subscriber<geometry_msgs::Vector3> subPID("/config/pid", pidCallback);
  ros::Subscriber<std_msgs::String> subSys("/robot/cmd", sysCommandCallback);
  ros::Subscriber<geometry_msgs::Twist> subCmdVelManual("/robot/cmdvel_manual", cmdVelCallbackManual);
  ros::Subscriber<geometry_msgs::Twist> subCmdVelAuto("/cmd_vel", cmdVelCallbackAuto);

void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(20);
  setupSensors();
  
  //Setup PCF8575
  pcf.begin();
  pcf_buffer = 0x0000;
  setPCFBit(STBY_PCF_1, HIGH);
  setPCFBit(STBY_PCF_2, HIGH);
    //Setup Relays (PCF8575)
  setPCFBit(RELAY1_PCF, HIGH); // Default OFF
  setPCFBit(RELAY2_PCF, HIGH);
  pcf.write16(pcf_buffer);

  servoPan.setPeriodHertz(50);
  servoTilt.setPeriodHertz(50);

  servoPan.attach(SERVO_PAN_PIN, SERVO_MIN, SERVO_MAX);
  servoTilt.attach(SERVO_TILT_PIN, SERVO_MIN, SERVO_MAX);

  //สั่งให้ไปที่จุดกึ่งกลาง 
  servoPan.writeMicroseconds(pos_pan);
  servoTilt.writeMicroseconds(pos_tilt);


  //Setup IMU (MPU6050)
  byte status = mpu.begin();
  if(status != 0){ /* MPU not connected */ }
  mpu.setGyroOffsets(-4.43, 0.99, -0.02);
  delay(3000);
  
  //Setup Compass (QMC5883L)
  compass.init();
  compass.setCalibration(-968, 1291, -2735, -292, -2481, -177);

  //Setup Motors & Encoders
  configPWMPin(FL_PWM_PIN);
  configPWMPin(FR_PWM_PIN);
  configPWMPin(RL_PWM_PIN);
  configPWMPin(RR_PWM_PIN);

  pinMode(ENCODER_FL_A, INPUT_PULLUP); pinMode(ENCODER_FL_B, INPUT_PULLUP);
  pinMode(ENCODER_FR_A, INPUT_PULLUP); pinMode(ENCODER_FR_B, INPUT_PULLUP);
  pinMode(ENCODER_RL_A, INPUT_PULLUP); pinMode(ENCODER_RL_B, INPUT_PULLUP);
  pinMode(ENCODER_RR_A, INPUT_PULLUP); pinMode(ENCODER_RR_B, INPUT_PULLUP);


  // Setup Encoder FL
  encoderFL.attachFullQuad(ENCODER_FL_A, ENCODER_FL_B); // โหมด Quadrature (x4) ละเอียดสุด
  encoderFL.setCount(0); // รีเซ็ตค่าเริ่มต้น
  // Setup Encoder FR
  encoderFR.attachFullQuad(ENCODER_FR_A, ENCODER_FR_B);
  encoderFR.setCount(0);
  // Setup Encoder RL
  encoderRL.attachFullQuad(ENCODER_RL_A, ENCODER_RL_B);
  encoderRL.setCount(0);
  // Setup Encoder RR
  encoderRR.attachFullQuad(ENCODER_RR_A, ENCODER_RR_B);
  encoderRR.setCount(0);

  //Setup PID
  pidFL.SetMode(AUTOMATIC); pidFL.SetOutputLimits(-255, 255);
  pidFR.SetMode(AUTOMATIC); pidFR.SetOutputLimits(-255, 255);
  pidRL.SetMode(AUTOMATIC); pidRL.SetOutputLimits(-255, 255);
  pidRR.SetMode(AUTOMATIC); pidRR.SetOutputLimits(-255, 255);

  nh.getHardware()->setBaud(250000);
  nh.initNode();
  broadcaster.init(nh);
  nh.subscribe(subCmdVelManual);
  nh.subscribe(subCmdVelAuto);
  nh.subscribe(subPan);
  nh.subscribe(subTilt);
  nh.subscribe(subPID);
  nh.subscribe(subSys);
  nh.advertise(odom_pub);
  nh.advertise(imu_pub);
  nh.advertise(battery_pub);
  nh.advertise(status_pub);
}

// Global variables หรือ static ภายใน loop
  unsigned long prevPIDMicros = 0;
  const unsigned long PID_INTERVAL_US = 5000; // 5ms = 200Hz

  unsigned long prevPubMillis = 0;
  const unsigned long PUB_INTERVAL_MS = 50; // 50ms = 20Hz

  unsigned long prevHeartbeat = 0;
  const unsigned long HEARTBEAT_INTERVAL = 5000;

  unsigned long lastCmdTime = 0;
  const unsigned long CMD_TIMEOUT = 200; // watchdog

  int compassCounter = 0;

void loop() {
  unsigned long currentMillis = millis();
  bool is_timeout = (currentMillis - lastCmdTime > CMD_TIMEOUT);
  bool is_disconnected = !nh.connected();

  if (is_timeout || is_disconnected) {
    // Force Stop ทันที (บายพาส PID เพื่อความปลอดภัยสูงสุด)
    sp_FL = 0; sp_FR = 0; sp_RL = 0; sp_RR = 0;
    out_FL = 0; out_FR = 0; out_RL = 0; out_RR = 0; // Clear PID output
    
    // Reset PID error accumulation (ถ้า library รองรับ หรือสั่ง re-init)
    pidFL.SetMode(MANUAL); pidFR.SetMode(MANUAL); pidRL.SetMode(MANUAL); pidRR.SetMode(MANUAL);// ปิด PID ชั่วคราว
    
    // สั่ง Hardware ให้หยุด
    setMotorPWM(0, FL_IN1_PCF, FL_IN2_PCF, FL_PWM_PIN);
    setMotorPWM(0, FR_IN1_PCF, FR_IN2_PCF, FR_PWM_PIN);
    setMotorPWM(0, RL_IN1_PCF, RL_IN2_PCF, RL_PWM_PIN);
    setMotorPWM(0, RR_IN1_PCF, RR_IN2_PCF, RR_PWM_PIN);
    
    // Update PCF (สำคัญมาก: ต้องสั่ง PCF ให้ตัดไฟ Driver)
    setPCFBit(STBY_PCF_1, LOW); // Disable motor driver 1
    setPCFBit(STBY_PCF_2, LOW); // // Disable motor driver 2

    pcf.write16(pcf_buffer); 
    
    // ถ้า ROS หลุด ให้พยายามต่อใหม่ แล้วจบ Loop รอบนี้
    if (is_disconnected) {
      nh.spinOnce();
      delay(50); // รอหน่อยกัน Loop เร็วเกินไปตอนหลุด;
      return; 
    }
    } else {
      // ถ้าปกติดี ให้เปิด PID กลับมา (กรณีที่เพิ่งหายจาก Timeout)
      if(pidFL.GetMode() == MANUAL) { 
          pidFL.SetMode(AUTOMATIC); pidFR.SetMode(AUTOMATIC);
          pidRL.SetMode(AUTOMATIC); pidRR.SetMode(AUTOMATIC);
      }
    }

  //PID Loop 
  unsigned long currentMicros = micros();
  if (currentMicros - prevPIDMicros >= PID_INTERVAL_US) {
    float dt = (currentMicros - prevPIDMicros) / 1000000.0; // แปลง us เป็น seconds
    prevPIDMicros = currentMicros;

    // --- 1. อ่าน Encoder ---
    long curFL = encoderFL.getCount();
    long curFR = encoderFR.getCount();
    long curRL = encoderRL.getCount();
    long curRR = encoderRR.getCount();

    static long oldFL=0, oldFR=0, oldRL=0, oldRR=0;
    
    double ticks_to_rads = RADS_PER_TICK / dt;

    in_FL = (curFL - oldFL) * ticks_to_rads;
    in_FR = (curFR - oldFR) * ticks_to_rads;
    in_RL = (curRL - oldRL) * ticks_to_rads;
    in_RR = (curRR - oldRR) * ticks_to_rads;
    oldFL = curFL; oldFR = curFR; oldRL = curRL; oldRR = curRR;

    //Odometry calculate and update
    calculateOdometry(dt, in_FL, in_FR, in_RL, in_RR);
    applyRampFilter(); 
    //PID Compute
    pidFL.Compute(); pidFR.Compute(); pidRL.Compute(); pidRR.Compute();

    //Motor Output
    setMotorPWM(out_FL, FL_IN1_PCF, FL_IN2_PCF, FL_PWM_PIN);
    setMotorPWM(out_FR, FR_IN1_PCF, FR_IN2_PCF, FR_PWM_PIN);
    setMotorPWM(out_RL, RL_IN1_PCF, RL_IN2_PCF, RL_PWM_PIN);
    setMotorPWM(out_RR, RR_IN1_PCF, RR_IN2_PCF, RR_PWM_PIN);

    // Send I2C Batch
    setPCFBit(STBY_PCF_1, HIGH);
    setPCFBit(STBY_PCF_2, HIGH);
    pcf.write16(pcf_buffer);
  }
  if (currentMillis - prevPubMillis >= PUB_INTERVAL_MS) {
    prevPubMillis = currentMillis;
    publishOdometryAndTF(); 
  }
  if (currentMillis - prevHeartbeat >= HEARTBEAT_INTERVAL) {
      prevHeartbeat = currentMillis;
      publishRobotStatus(); // ส่งสถานะรวม (แบต + ระบบ)
    }
  nh.spinOnce();
}

// ฟังก์ชันช่วยคำนวณ Inverse Kinematics (ใช้ร่วมกันทั้ง Auto และ Manual)
void computeWheelSpeeds(float vx, float vy, float w) {
  lastCmdTime = millis(); // อัปเดตเวลาล่าสุดที่ได้รับคำสั่ง (ป้องกัน Watchdog ตัด)

  double w_final = w;

  if (enable_heading_hold) {
    // กรณีที่ 1: เราสั่งให้หยุดหมุน (w = 0) -> เข้าโหมด "ล็อกทิศ"
    if (abs(w) < 0.01) {
      if (is_turning) {
        // เพิ่งหยุดหมุนเมื่อกี้ -> ให้จำมุมปัจจุบันเป็นเป้าหมายใหม่ทันที
        target_heading = theta; 
        is_turning = false;
      }

      // คำนวณ Error (เป้าหมาย - มุมจริง)
      double heading_error = target_heading - theta;

      // แก้ปัญหา Wrap Around (เช่น เป้า 3.14 แต่มุมจริง -3.14 -> error ควรเป็นนิดเดียว)
      if (heading_error > PI)  heading_error -= TWO_PI;
      if (heading_error < -PI) heading_error += TWO_PI;

      // คำนวณค่าชดเชย (P-Controller)
      // ถ้า Error เป็นบวก (หุ่นหันซ้ายเกิน) -> ต้องสั่งลบ (หมุนขวา)
      double w_correction = heading_error * heading_kp;
      
      // เอาค่าชดเชยไปรวมกับ w (ซึ่งตอนนี้เป็น 0)
      w_final = w_correction;

    } 
    // กรณีที่ 2: เราสั่งหมุน (w != 0) -> ปล่อยให้หมุน
    else {
      is_turning = true;
      target_heading = theta; // อัปเดตเป้าหมายตามตัวหุ่นไปเรื่อยๆ
      w_final = w;
    }
  }

  // Inverse Kinematics Mecanum X-Config
  float v_fl = vx - vy - (ROBOT_GEOMETRY * w);
  float v_fr = vx + vy + (ROBOT_GEOMETRY * w);
  float v_rl = vx + vy - (ROBOT_GEOMETRY * w);
  float v_rr = vx - vy + (ROBOT_GEOMETRY * w);

  target_FL = v_fl / WHEEL_RADIUS;
  target_FR = v_fr / WHEEL_RADIUS; 
  target_RL = v_rl / WHEEL_RADIUS;
  target_RR = v_rr / WHEEL_RADIUS;
}

void cmdVelCallbackAuto(const geometry_msgs::Twist& msg) {
  if (manual_mode == true) return; 
  computeWheelSpeeds(msg.linear.x, msg.linear.y, msg.angular.z);
}

void cmdVelCallbackManual(const geometry_msgs::Twist& msg) {
  if (manual_mode == false) return;
  computeWheelSpeeds(msg.linear.x, msg.linear.y, msg.angular.z);
}

void panCallback(const std_msgs::Int16& msg) {
  int angle = constrain(msg.data, 0, 180);
  int us = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  servoPan.writeMicroseconds(us);
}

void tiltCallback(const std_msgs::Int16& msg) {
  int angle = constrain(msg.data, 0, 180);
  int us = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  servoTilt.writeMicroseconds(us);
}

void pidCallback(const geometry_msgs::Vector3& msg) {
  Kp = msg.x;
  Ki = msg.y;
  Kd = msg.z;
  updatePIDTunings(); // อัปเดตทันที
  
  // (Optional) ส่ง Log กลับไปบอกว่าเปลี่ยนแล้ว
  nh.loginfo("PID Updated");
}

void sysCommandCallback(const std_msgs::String& msg) {
  String cmd = msg.data;
  bool status_changed = false;
  if (cmd == "manual_on") {
    manual_mode = true;
    // Reset ความเร็วเป็น 0 เพื่อความปลอดภัยตอนสลับโหมด
    sp_FL = 0; sp_FR = 0; sp_RL = 0; sp_RR = 0;
    status_changed = true;
  } 
  else if (cmd == "manual_off" || cmd == "auto_on") { 
    manual_mode = false;
    // Reset ความเร็วเป็น 0
    sp_FL = 0; sp_FR = 0; sp_RL = 0; sp_RR = 0;
    status_changed = true;
  }
  else if (cmd == "r1_on") {
    setPCFBit(RELAY1_PCF, LOW); 
    pcf.write16(pcf_buffer);
    status_changed = true;
    } 
  else if (cmd == "r1_off") {
    setPCFBit(RELAY1_PCF, HIGH);
    pcf.write16(pcf_buffer);
    status_changed = true;
    }
  else if (cmd == "r2_on")  {
    setPCFBit(RELAY2_PCF, LOW);
    pcf.write16(pcf_buffer);
    status_changed = true;
  }
  else if (cmd == "r2_off") {
    setPCFBit(RELAY2_PCF, HIGH);
    pcf.write16(pcf_buffer);
    status_changed = true;
  }
  else if (cmd == "debug_on") {
    debug_mode = true;
    status_changed = true;
  }
  else if (cmd == "debug_off") {
    debug_mode = false;
    status_changed = true;
  }
  else if (cmd.startsWith("set_pid:")) {
    // ตัดเอาเฉพาะส่วนตัวเลขหลังเครื่องหมาย :
    String values = cmd.substring(8); 
    float p, i, d;
    // ใช้ sscanf เพื่อแยกตัวเลขออกจาก comma (,)
    if (sscanf(values.c_str(), "%f,%f,%f", &p, &i, &d) == 3) {
       Kp = p;
       Ki = i;
       Kd = d;
       updatePIDTunings();
       status_changed = true;
    }   
  }
  else if (cmd == "report" || cmd == "status") {
      status_changed = true;
  }
  if (status_changed) {
    publishRobotStatus();
  }
}

// --- Helper: Control Motor via PCF8575 & PWM ---
void setMotorPWM(float pwm, int pin1, int pin2, int pwmPin) {
  int speed = abs((int)pwm);
  if (speed > 0 && speed < MIN_PWM) {
    speed = MIN_PWM;
  }

  if (speed > 255) speed = 255;

  // Update แค่ใน Buffer (ยังไม่ส่ง I2C)
  if (pwm > 0) {
    setPCFBit(pin1, HIGH);
    setPCFBit(pin2, LOW);
  } else if (pwm < 0) {
    setPCFBit(pin1, LOW);
    setPCFBit(pin2, HIGH);
  } else {
    setPCFBit(pin1, LOW);
    setPCFBit(pin2, LOW);
    speed = 0;
  }
  
  // เขียน PWM Direct to ESP32 Pin
  analogWrite(pwmPin, speed);
}

void updatePIDTunings() {
  pidFL.SetTunings(Kp, Ki, Kd);
  pidFR.SetTunings(Kp, Ki, Kd);
  pidRL.SetTunings(Kp, Ki, Kd);
  pidRR.SetTunings(Kp, Ki, Kd);
}

// ฟังก์ชันสำหรับแก้ค่า Bit ในตัวแปร buffer
void setPCFBit(uint8_t pin, bool state) {
  if (state) {
    pcf_buffer |= (1 << pin);  // Set Bit (ให้เป็น 1)
  } else {
    pcf_buffer &= ~(1 << pin); // Clear Bit (ให้เป็น 0)
  }
}

// ฟังก์ชันตั้งค่า PWM สำหรับ ESP32 Core 3.0+
void configPWMPin(uint8_t pin) {
  // ตั้งค่า Resolution และ Frequency ต่อขา
  analogWriteResolution(pin, 8);   // 8-bit (0-255)
  analogWriteFrequency(pin, 20000); // 20kHz
}

void publishOdometryAndTF() {
    ros::Time now = nh.now();

    // A. Odometry Message
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = odom_frame;
    odom_msg.child_frame_id = base_link;
    
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.position.y = y_pos;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.twist.twist.linear.x = linear_x;
    odom_msg.twist.twist.linear.y = linear_y;
    odom_msg.twist.twist.angular.z = gyro_z;

    odom_msg.pose.covariance[0] = 0.01; 
    odom_msg.pose.covariance[7] = 0.01;
    odom_msg.pose.covariance[35] = 0.1; 

    odom_pub.publish(&odom_msg);

    // B. IMU Message
    imu_msg.header.stamp = now;
    imu_msg.header.frame_id = "imu_link";
    imu_msg.orientation = odom_quat;
    
    imu_msg.angular_velocity.x = mpu.getGyroX() * DEG_TO_RAD;
    imu_msg.angular_velocity.y = mpu.getGyroY() * DEG_TO_RAD;
    imu_msg.angular_velocity.z = gyro_z;

    imu_msg.linear_acceleration.x = mpu.getAccX() * 9.81;
    imu_msg.linear_acceleration.y = mpu.getAccY() * 9.81;
    imu_msg.linear_acceleration.z = mpu.getAccZ() * 9.81;

    imu_msg.orientation_covariance[0] = 0.01;
    imu_msg.orientation_covariance[8] = 0.01; 
    
    imu_pub.publish(&imu_msg);

    // C. TF Broadcast
    t.header.stamp = now;
    t.header.frame_id = odom_frame;
    t.child_frame_id = base_link;
    
    t.transform.translation.x = x_pos;
    t.transform.translation.y = y_pos;
    t.transform.translation.z = 0.0;
    t.transform.rotation = odom_quat;
    
    broadcaster.sendTransform(t);
}

double mag_heading = 0;
const float MAGNETIC_DECLINATION = 0.0 * DEG_TO_RAD;
void calculateOdometry(float dt, double v_fl, double v_fr, double v_rl, double v_rr) {
    // --- 1. Wheel Odometry ---
    linear_x  = (v_fl + v_fr + v_rl + v_rr) * (WHEEL_RADIUS / 4.0);
    linear_y  = (-v_fl + v_fr + v_rl - v_rr) * (WHEEL_RADIUS / 4.0);

    // --- 2. Gyro Integration (ทำทุกรอบ เพื่อความต่อเนื่อง) ---
    mpu.update();
    gyro_z = mpu.getGyroZ() * DEG_TO_RAD;
    
    // บวกค่า Gyro เข้าไปก่อนเลย (Prediction Step)
    theta += gyro_z * dt; 

    // Normalize Theta (-PI to +PI) กันค่าล้น
    if (theta > PI)  theta -= TWO_PI;
    if (theta < -PI) theta += TWO_PI;

    // --- 3. Compass Correction (ทำเฉพาะรอบที่มีข้อมูลใหม่) ---
    compassCounter++;
    if (compassCounter >= 4) {
        compass.read(); 
        float mag_x = compass.getX();
        float mag_y = compass.getY();
        
        // คำนวณ Heading ใหม่
        mag_heading = atan2(mag_y, mag_x) + MAGNETIC_DECLINATION;
        
        // Normalize Compass Heading
        if (mag_heading > PI)  mag_heading -= TWO_PI;
        if (mag_heading < -PI) mag_heading += TWO_PI;

        // --- คำนวณ Error และ Fusion เฉพาะตรงนี้ ---
        double error = mag_heading - theta;

        // แก้ปัญหา Wrap Around (เช่น Compass 3.14, Theta -3.14 -> error ควรนิดเดียว)
        if (error > PI)  error -= TWO_PI;
        if (error < -PI) error += TWO_PI;

        // Apply Correction (เชื่อ Compass 2%)
        theta += (0.02 * error);

        // Normalize Theta อีกรอบหลังแก้
        if (theta > PI)  theta -= TWO_PI;
        if (theta < -PI) theta += TWO_PI;

        compassCounter = 0; // Reset counter
    }

    // --- 4. Position Integration ---
    // ใช้ Theta ล่าสุดที่ผ่านการ Fusion แล้วมาแตกแรง
    double delta_x = (linear_x * cos(theta) - linear_y * sin(theta)) * dt;
    double delta_y = (linear_x * sin(theta) + linear_y * cos(theta)) * dt;

    x_pos += delta_x;
    y_pos += delta_y;
    
    // อัปเดต Quaternion
    odom_quat.w = cos(theta / 2.0);
    odom_quat.z = sin(theta / 2.0);
    odom_quat.x = 0.0;
    odom_quat.y = 0.0;
}

void setupSensors() {
  analogReadResolution(12); // อ่านละเอียด 12-bit
  analogSetAttenuation(ADC_11db); // อ่านได้เต็มช่วง 0-3.3V
  
  // Calibrate Current Sensor Zero Point (ตอนเปิดเครื่องต้องไม่มีโหลด)
  long sum = 0;
  for(int i=0; i<50; i++) {
    sum += analogRead(CURRENT_SENSOR_PIN);
    delay(2);
  }
  current_zero_point = sum / 50;
}

float readVoltageEMA() {
    float raw = analogRead(VOLTAGE_SENSOR_PIN);
    // สูตร EMA: ใหม่ = (เดิม * 0.9) + (ใหม่ * 0.1)
    filter_volt = (filter_volt * (1.0 - alpha)) + (raw * alpha);
    
    // แปลง filter_volt เป็น Voltage ตามสูตรเดิมของคุณ
    float voltage = (filter_volt / ADC_RES) * ADC_VREF * VOLTAGE_MULTIPLIER;
    return voltage;
}

float readCurrentEMA() {
  int raw_adc = analogRead(CURRENT_SENSOR_PIN);

  //เข้าสูตร EMA (กรอง Noise)
  // alpha = 0.1 (เชื่อค่าใหม่ 10%, ค่าเดิม 90%) ช่วยลดการแกว่ง
  filter_amp_adc = (filter_amp_adc * 0.9) + (raw_adc * 0.1);

  //เอาค่าที่กรองแล้ว (filter_amp_adc) ไปคำนวณสูตรเดิม
  // ลบค่า Zero Point (Calibration)
  float delta_adc = filter_amp_adc - current_zero_point;
  
  // แปลง ADC -> Voltage ที่ขา ESP
  float delta_volts = (delta_adc / ADC_RES) * ADC_VREF;
  
  // ย้อนกลับ Voltage Divider
  float sensor_volts = delta_volts / 0.647; 
  
  // แปลงเป็น Amps (Sensitivity 0.100 V/A)
  float amps = sensor_volts / 0.100;
  
  return abs(amps); // ส่งกลับเป็น mA
}

// ฟังก์ชันช่วยคำนวณการขยับค่าทีละนิด
double stepTowards(double current, double target, double step) {
  if (current < target) return min(current + step, target);
  if (current > target) return max(current - step, target);
  return target;
}

void applyRampFilter() {
  sp_FL = stepTowards(sp_FL, target_FL, RAMP_STEP);
  sp_FR = stepTowards(sp_FR, target_FR, RAMP_STEP);
  sp_RL = stepTowards(sp_RL, target_RL, RAMP_STEP);
  sp_RR = stepTowards(sp_RR, target_RR, RAMP_STEP);
}

void publishRobotStatus() {
  //อ่านค่าแบตเตอรี่ล่าสุด (ผ่าน EMA Filter)
  float voltage = readVoltageEMA();
  float current = readCurrentEMA();

  //เช็คสถานะโหมด
  String modeStr = manual_mode ? "MAN" : "AUTO"; // ย่อให้สั้นลงนิดนึง
  if (debug_mode) modeStr += "+DBG";

  //เช็คสถานะ Watchdog
  unsigned long timeSinceLastCmd = millis() - lastCmdTime;
  String wdStr = (timeSinceLastCmd > CMD_TIMEOUT) ? "STOP" : "OK";

  //เช็คสถานะ Relay
  bool r1 = !((pcf_buffer >> RELAY1_PCF) & 1);
  bool r2 = !((pcf_buffer >> RELAY2_PCF) & 1);

  //อัดทุกอย่างลง Buffer
  // Format: [MODE] Bat:VV.V(A.AA) | WD:State | PID:P,I,D | Relay:1,2
  snprintf(status_buffer, sizeof(status_buffer), 
    "[%s] Bat:%.2fV(%.2fA) | WD:%s(%lums) | PID:%.1f,%.1f,%.2f | R:%d,%d", 
    modeStr.c_str(),
    voltage, current,
    wdStr.c_str(), timeSinceLastCmd,
    Kp, Ki, Kd,
    r1, r2
  );

  //ส่งข้อมูล
  status_msg.data = status_buffer;
  status_pub.publish(&status_msg);
}
