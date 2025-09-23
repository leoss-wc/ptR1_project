#include "I2Cdev.h"                        // ไลบรารีสำหรับ I2C communication
#include "MPU6050_6Axis_MotionApps20.h"     // ไลบรารีสำหรับใช้งาน Digital Motion Processing (DMP)
#include "MPU6050.h"                        // ไลบรารีสำหรับเซ็นเซอร์ IMU MPU6050

#include <ros.h>                            // ไลบรารีสำหรับ ROS communication
#include <std_msgs/UInt32.h>                // ใช้สำหรับส่งค่าประเภท UInt32 ใน ROS
#include <std_msgs/UInt16.h>                // ใช้สำหรับส่งค่าประเภท UInt16 ใน ROS
#include <std_msgs/UInt8.h>                 // ใช้สำหรับส่งค่าประเภท UInt8 ใน ROS
#include <std_msgs/Float32.h>               // ใช้สำหรับส่งค่าประเภท Float32 ใน ROS
#include <geometry_msgs/Twist.h>            // ใช้สำหรับรับคำสั่งความเร็ว (cmd_vel) จาก ROS
#include <sensor_msgs/Imu.h>                // ใช้สำหรับส่งข้อมูลจาก IMU ไปยัง ROS
#include <sensor_msgs/MagneticField.h>      // ใช้สำหรับส่งค่าของ Compass (สนามแม่เหล็ก)
#include <std_msgs/String.h>                // ใช้สำหรับส่งข้อมูลเป็นข้อความ

#include <avr/io.h>                         // ไลบรารีควบคุม I/O ของ ATmega
#include <avr/interrupt.h>                  // ไลบรารีสำหรับใช้ Interrupts
#include <Wire.h>                            // ไลบรารีสำหรับ I2C communication
#include <Servo.h>                          // ไลบรารีควบคุม Servo
#include <math.h>                           // IMU normalized


// ค่าคงที่ของ Timer
#define TIMER_INTERVAL_MS_sensor 50  // Sampling interval for sensors (50ms = 20Hz)
#define FREQ 16000000  // ความถี่ของ CPU (16MHz)
#define PRESCALER 1024  // ค่าแบ่งสัญญาณของ Timer

// การกำหนดพินของมอเตอร์ไดรเวอร์ (Motor Driver)
#define md1_AIN2 35
#define md1_AIN1 37
#define md1_BIN1 41
#define md1_BIN2 43
#define md1_PWMA 12 
#define md1_PWMB 13

#define md1_STBY 39
                    
#define md2_AIN2 40 
#define md2_AIN1 42 
#define md2_BIN1 34 
#define md2_BIN2 36 
#define md2_PWMA 10
#define md2_PWMB 11

#define md2_STBY 38  // ขาสำหรับเปิดใช้งานมอเตอร์ (STBY)

// Wheels mapping 
#define FL_IN1 md1_AIN1
#define FL_IN2 md1_AIN2
#define FL_PWM md1_PWMA

#define FR_IN1   md2_AIN1
#define FR_IN2   md2_AIN2
#define FR_PWM   md2_PWMA

#define RL_IN1   md2_BIN1
#define RL_IN2   md2_BIN2
#define RL_PWM   md2_PWMB

#define RR_IN1   md1_BIN1
#define RR_IN2   md1_BIN2
#define RR_PWM   md1_PWMB




#define RELAY1 33    // pin relay 1
#define RELAY2 32    // pin relay 2

// ค่า Baud Rate ของ Serial Communication
#define BAUD_RATE 250000

// ค่าคงที่ของ Compass Sensor (QMC5883L)
#define QMC5883L_ADDRESS 0x0D
#define MIN_X -1363
#define MAX_X  665
#define MIN_Y -1932
#define MAX_Y  32
#define MIN_Z -568
#define MAX_Z -265

// Odometry
  float vx = 0.0, vy = 0.0, omega = 0.0;

// MPU6050
uint8_t fifoBuffer[64]; 
bool use_imu = false;
bool use_imu_mag = false;


// การชดเชยค่าของ Compass Sensor หลังจาก Calibration
const float OFFSET_X = -349.0;  // Offset ตามแนวแกน X
const float OFFSET_Y = -950.0;  // Offset ตามแนวแกน Y
const float OFFSET_Z = -416.5;  // Offset ตามแนวแกน Z
const float SCALE_X = 0.8527;
const float SCALE_Y = 0.8284;
const float SCALE_Z = 1.6124;

// พินของเซ็นเซอร์วัดกระแสและแรงดันไฟฟ้า
#define CURRENT_SENSOR_PIN A9
#define VOLTAGE_SENSOR_PIN A8
// ค่าคงที่สำหรับเซ็นเซอร์วัดกระแสไฟฟ้า (ACS712-20A)
#define ACS712_SENSITIVITY 0.100  // 0.100 V/A
// ค่า offset ที่ได้จากการ calibrate (เช่น analogRead() ขณะไม่มีโหลด)
#define ACS712_ZERO_RAW 511     

#define ADC_RESOLUTION 1024.0
#define REFERENCE_VOLTAGE 5.0  // อ้างอิงแรงดันที่ 5V

// พินของ Servo Motor
const uint8_t SERVO1_PIN = 44;
const uint8_t SERVO2_PIN = 46;

// ค่าเริ่มต้นของ Servo
Servo servo1, servo2;
const uint16_t MIN_POSITION = 1000;   // ตำแหน่งต่ำสุดของ Servo (us)
const uint16_t MAX_POSITION = 2000;   // ตำแหน่งสูงสุดของ Servo (us)
uint16_t current_position_1 = 1500;  // ตำแหน่งเริ่มต้นของ Servo1 (90°)
uint16_t current_position_2 = 1500;  // ตำแหน่งเริ่มต้นของ Servo2 (90°)
uint16_t STEP_SIZE_X = 55;  // ขยับ 10 องศา ≈ 55us
uint16_t STEP_SIZE_Y = 28;  // ขยับ 5 องศา ≈ 28us

// Debugging
#define DEBUG_BUFFER_SIZE 128
char debug_buffer[DEBUG_BUFFER_SIZE];
bool DEBUG_MODE_S2 = 1;   // เปิดใช้งาน Debug Mode
bool debug_flag = 1;       // เปิดใช้งานการพิมพ์ข้อมูล Debug


// การควบคุมหุ่นยนต์
enum ControlMode { AUTO, MANUAL }; // โหมดควบคุม: อัตโนมัติ (AUTO) หรือ ควบคุมเอง (MANUAL)
ControlMode current_mode = MANUAL; // ตั้งค่าเริ่มต้นเป็น Manual

// ตัวแปรเก็บข้อมูลเซ็นเซอร์
MPU6050 mpu;          // IMU MPU6050
uint16_t current_mA;  // กระแสไฟฟ้า (mA)
uint16_t voltage_cV;  // แรงดันไฟฟ้า (cV)

void DriveCallback(const std_msgs::UInt16& msg);
void command_servo(const std_msgs::UInt8& msg);
void commandEdit(const std_msgs::UInt32& msg);
void cmdVelCallback(const geometry_msgs::Twist& cmd_vel_msg);

// ROS Node และข้อความที่ใช้ใน ROS
ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
sensor_msgs::Imu mag_msg;
std_msgs::String debug_msgs;
std_msgs::UInt32 sensor_data_msg;
std_msgs::Float32 temp_msg;
geometry_msgs::Twist cmd_vel_msg;

// ROS Publishers
ros::Publisher mag_pub("/imu/mag", &mag_msg);
ros::Publisher debug_pub("debug/data", &debug_msgs);
ros::Publisher sensor_data_pub("/sensor/data", &sensor_data_msg);
ros::Publisher imu_raw_pub("imu/data_raw", &imu_msg);
ros::Publisher temp_pub("imu/temperature", &temp_msg);

// ROS Subscribers
ros::Subscriber<std_msgs::UInt16> sub_drive("/rb/cm/dr", DriveCallback);
ros::Subscriber<std_msgs::UInt8> sub_servo("/rb/cm/sv", command_servo);
ros::Subscriber<std_msgs::UInt32> sub_edit("/rb/cm/ed", commandEdit);
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("/cmd_vel", cmdVelCallback);

// ตัวแปรเก็บสถานะของมอเตอร์
unsigned long current_time;
bool motor_running = false;
unsigned long last_cmd_time = 0;
uint16_t time_delay = 60;  // เวลาหน่วงสำหรับหยุดมอเตอร์อัตโนมัติ (ms)

/**
 * @brief ฟังก์ชัน setup() ใช้สำหรับกำหนดค่าเริ่มต้นของระบบก่อนเริ่ม loop()
 * 
 * @details 
 * - กำหนดค่า Serial และ I2C สำหรับการสื่อสาร
 * - กำหนดค่าเริ่มต้นสำหรับ ROS Node
 * - ตั้งค่าขา (Pin Mode) สำหรับมอเตอร์, เซ็นเซอร์ และ Servo
 * - ทดสอบการเชื่อมต่อกับเซ็นเซอร์ต่างๆ เช่น IMU (MPU6050) และ Compass (QMC5883L)
 * - กำหนดค่า Standby ของมอเตอร์ให้พร้อมใช้งาน
 */
void setup() {
    // ตั้งค่าการสื่อสาร Serial และ I2C
    Serial.begin(BAUD_RATE);     // Serial หลักสำหรับ rosserial 250000
    Serial1.begin(57600);        // Serial1 สำหรับการรับข้อมูลจาก Arduino อื่นๆ
    Serial2.begin(115200);       // Serial2 สำหรับ Debugging เพิ่มเติม
    Wire.begin();                // เริ่มใช้งาน I2C
    Wire.setClock(400000);       // ตั้งค่า I2C Speed เป็น 400kHz (Fast Mode)

    // ตั้งค่า ROS Node
    nh.getHardware()->setBaud(BAUD_RATE);
    nh.initNode();
    
    // กำหนด Publisher สำหรับส่งข้อมูลไปยัง ROS
    nh.advertise(mag_pub);
    nh.advertise(debug_pub);
    nh.advertise(sensor_data_pub);
    nh.advertise(imu_raw_pub);

    // กำหนด Subscriber สำหรับรับคำสั่งจาก ROS
    nh.subscribe(sub_drive);
    nh.subscribe(sub_servo);
    nh.subscribe(sub_edit);
    nh.subscribe(sub_cmd_vel);

    // ตั้งค่าขา (Pin Mode) สำหรับมอเตอร์
    pinMode(md1_AIN2, OUTPUT);
    pinMode(md1_AIN1, OUTPUT);
    pinMode(md1_STBY, OUTPUT);
    pinMode(md1_BIN1, OUTPUT);
    pinMode(md1_BIN2, OUTPUT);
    pinMode(md1_PWMA, OUTPUT);
    pinMode(md1_PWMB, OUTPUT);
    

    pinMode(md2_AIN2, OUTPUT);
    pinMode(md2_AIN1, OUTPUT);
    pinMode(md2_STBY, OUTPUT);
    pinMode(md2_BIN1, OUTPUT);
    pinMode(md2_BIN2, OUTPUT);
    pinMode(md2_PWMA, OUTPUT);
    pinMode(md2_PWMB, OUTPUT);


    //Relay pinout
    pinMode(RELAY1, OUTPUT);
    pinMode(RELAY2, OUTPUT);

    // เปิดใช้งาน STBY ของมอเตอร์เพื่อให้พร้อมทำงาน
    digitalWrite(md1_STBY, HIGH);
    digitalWrite(md2_STBY, HIGH);

    // ตั้งค่าและทดสอบเซ็นเซอร์ IMU (MPU6050)
    mpu.initialize();
    if (mpu.testConnection() == false) {
        Serial2.println("MPU6050 connection failed");
    } else {
        Serial2.println("MPU6050 connection successful");
    }

    // เปิดใช้งาน DMP (Digital Motion Processing) ของ MPU6050
    uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        Serial2.println("DMP Enabled");
        mpu.resetFIFO();
    } else {
        Serial2.print("DMP Initialization failed (code ");
        Serial2.print(devStatus);
        Serial2.println(")");
    }

    // ตั้งค่าและทดสอบ Compass (QMC5883L)
    Wire.beginTransmission(QMC5883L_ADDRESS);
    if (Wire.endTransmission() == 0) {
        Serial2.println("Compass detected");
    } else {
        Serial2.println("Compass NOT detected!");
    }

    // กำหนดค่าให้ Compass ทำงานในโหมด Continuous ที่ ODR 200Hz
    Wire.beginTransmission(QMC5883L_ADDRESS);
    Wire.write(0x09);      // Control register
    Wire.write(0x0D);      // Mode Continuous, ODR 1D = 200Hz, full-scale  , 0D = 10 Hz
    Wire.endTransmission();

    // ตั้งค่า Servo Motor
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    moveServo(SERVO1_PIN, 1500);  // ตั้งค่า Servo1 ไว้ที่ตำแหน่งกลาง (90°)
    moveServo(SERVO2_PIN, 1500);  // ตั้งค่า Servo2 ไว้ที่ตำแหน่งกลาง (90°)

    // กำหนดสถานะเริ่มต้นของมอเตอร์
    motor_running = false;

    Serial2.println("Setup complete");
}

/**
 * @brief ฟังก์ชันหลักของ Arduino ที่ทำงานซ้ำๆ ตลอดเวลา
 * 
 * @details
 * - อ่านค่าข้อมูลจาก Serial1 หากมีข้อมูลเข้ามา
 * - ตรวจสอบระยะเวลาที่เหมาะสมในการอ่านข้อมูลจาก Compass, IMU, และเซ็นเซอร์ไฟฟ้า
 * - ตรวจสอบว่ามีคำสั่งควบคุมมอเตอร์ล่าสุดหรือไม่ หากไม่มีให้หยุดการทำงานของมอเตอร์
 * - เรียก `nh.spinOnce()` เพื่อให้ ROS สามารถอัปเดตสถานะและรับ-ส่งข้อมูลได้
 * - ใช้ `delay(1)` เพื่อลดภาระ CPU เล็กน้อย (อาจปรับปรุงให้ใช้ `millis()` แทน)
 */
void loop() {
    // ใช้ตัวแปร static เพื่อติดตามเวลาที่ผ่านไปของแต่ละเซ็นเซอร์
    static unsigned long lastCompassTime = 0;
    static unsigned long lastIMUTime = 0;
    static unsigned long lastPowerSensorTime = 0;
    unsigned long now = millis(); // อ่านเวลาปัจจุบัน

    while (Serial1.available()) {
    char c = Serial1.read();
    processSerialData(c);
    }

    // อ่านค่าจาก Compass ทุกๆ 100ms (10Hz)
    if ((now - lastCompassTime >= 10)&& use_imu_mag) {  
        readCompass();
        lastCompassTime = now;
    }

    // อ่านค่าจาก IMU ทุกๆ 20ms (50Hz)
    if ((now - lastIMUTime >= 50)&& use_imu) {  
        readIMU();
        lastIMUTime = now;
    }
    // อ่านค่ากระแสไฟฟ้าและแรงดันไฟฟ้าทุกๆ 1000ms (1Hz)
    if (now - lastPowerSensorTime >= 1000) {  
        current_mA = readCurrent();
        voltage_cV = readVoltage();
        sensor_data_msg.data = ((uint32_t)current_mA << 16) | voltage_cV;
        sensor_data_pub.publish(&sensor_data_msg);
        lastPowerSensorTime = now;      
    }

    // ตรวจสอบสถานะของมอเตอร์
    if (motor_running) {  
        current_time = millis();
        if (current_time - last_cmd_time > time_delay) {
            // หากไม่มีคำสั่งใหม่ในช่วงเวลาที่กำหนด ให้หยุดมอเตอร์
            MotorCoastMode(); // อาจใช้โหมด coast mode เพื่อปล่อยมอเตอร์ให้หมุนอิสระ
            motor_running = false;  
            if (debug_flag) {
                snprintf(debug_buffer, sizeof(debug_buffer),
                        "Auto-stop: No cmd received");
                DebugPublish(debug_buffer);
            }

        }
    }

    // อัปเดต ROS Node และรอรับคำสั่งใหม่
    static unsigned long lastSpinTime = 0;
    if (now - lastSpinTime >= 5) {  // 200Hz
      nh.spinOnce();
      lastSpinTime = now;
    }
}

/**
 * @brief หยุดการทำงานแบบปล่อยอิสระของมอเตอร์ทั้งหมดโดยตรง
 * หยุดโดยไม่ต้องผ่านการเปรียบเทียบเงื่อนไข
 */
void MotorCoastMode(){
  // หยุดมอเตอร์ MD1
  digitalWrite(FL_IN1, LOW);
  digitalWrite(FL_IN2, LOW);
  analogWrite(FL_PWM,0);

  digitalWrite(FR_IN1, LOW);
  digitalWrite(FR_IN2, LOW);
  analogWrite(FR_PWM,0);


  // หยุดมอเตอร์ MD2
  digitalWrite(RL_IN1, LOW);
  digitalWrite(RL_IN2, LOW);
  analogWrite(RL_PWM,0);

  digitalWrite(RR_IN1, LOW);
  digitalWrite(RR_IN2, LOW);
  analogWrite(RR_PWM,0);
}

/**
 * @brief ควบคุมการเคลื่อนที่ของหุ่นยนต์ Mecanum
 * 
 * ฟังก์ชันนี้ใช้กำหนดทิศทางและความเร็วของหุ่นยนต์โดยอ้างอิงจาก
 * หมายเลขมอเตอร์ที่ตั้งค่าไว้ (FL(M1), FR(M2), RL(M3), RR(M4)) และปรับความเร็วโดยใช้ PWM
 *
 * @param direction ทิศทางการเคลื่อนที่ของหุ่นยนต์ (1-10)
 *  - 1: Forward (เดินหน้า)
 *  - 2: Left (ซ้าย)
 *  - 3: Right (ขวา)
 *  - 4: Backward (ถอยหลัง)
 *  - 5: Turn Left (หมุนซ้าย)
 *  - 6: Turn Right (หมุนขวา)
 *  - 7: Forward Left (เดินหน้าเฉียงซ้าย)
 *  - 8: Forward Right (เดินหน้าเฉียงขวา)
 *  - 9: Backward Left (ถอยหลังเฉียงซ้าย)
 *  - 10: Backward Right (ถอยหลังเฉียงขวา)
 * @param pwmMD1A ค่าความเร็วของมอเตอร์ FL (0-255)
 * @param pwmMD1B ค่าความเร็วของมอเตอร์ FR (0-255)
 * @param pwmMD2A ค่าความเร็วของมอเตอร์ RL (0-255)
 * @param pwmMD2B ค่าความเร็วของมอเตอร์ RR (0-255)
 */
void moveRobot(uint8_t direc, uint8_t pwmMD1A, uint8_t pwmMD1B, uint8_t pwmMD2A, uint8_t pwmMD2B) {
    const char* direction_str = "Unknown";

    switch (direc) {
        case 1:  // Forward
            setMotorDirection(1, 0); setMotorDirection(2, 0);
            setMotorDirection(3, 0); setMotorDirection(4, 0);
            direction_str = "Forward";
            break;

        case 2:  // Left
            setMotorDirection(1, 1); setMotorDirection(2, 0);
            setMotorDirection(3, 0); setMotorDirection(4, 1);
            direction_str = "Left";
            break;

        case 3:  // Right
            setMotorDirection(1, 0); setMotorDirection(2, 1);
            setMotorDirection(3, 1); setMotorDirection(4, 0);
            direction_str = "Right";
            break;

        case 4:  // Backward
            setMotorDirection(1, 1); setMotorDirection(2, 1);
            setMotorDirection(3, 1); setMotorDirection(4, 1);
            direction_str = "Backward";
            break;

        case 5:  // Turn Left
            setMotorDirection(1, 1); setMotorDirection(2, 0);
            setMotorDirection(3, 1); setMotorDirection(4, 0);
            direction_str = "Turn Left";
            break;

        case 6:  // Turn Right
            setMotorDirection(1, 0); setMotorDirection(2, 1);
            setMotorDirection(3, 0); setMotorDirection(4, 1);
            direction_str = "Turn Right";
            break;

        case 7:  // Forward Left
            setMotorCoastMode(1);    // FL
            setMotorDirection(2, 0); // FR
            setMotorDirection(3, 0); // RL
            setMotorCoastMode(4);    // RR
            direction_str = "Forward Left";
            pwmMD1A = 0;
            pwmMD2B = 0;
            break;

        case 8:  // Forward Right
            setMotorDirection(1, 0); // FL
            setMotorCoastMode(2);    // FR
            setMotorCoastMode(3);    // RL
            setMotorDirection(4, 0); // RR
            direction_str = "Forward Right";
            pwmMD1B = 0;
            pwmMD2A = 0;
            break;

        case 9:  // Backward Left
            setMotorCoastMode(1);    // FL
            setMotorDirection(2, 1); // FR
            setMotorDirection(3, 1); // RL
            setMotorCoastMode(4);    // RR
            direction_str = "Backward Left";
            pwmMD1A = 0;
            pwmMD2B = 0;
            break;

        case 10:  // Backward Right
            setMotorDirection(1, 1); // FL
            setMotorCoastMode(2);    // FR
            setMotorCoastMode(3);    // RL
            setMotorDirection(4, 1); // RR
            direction_str = "Backward Right";
            pwmMD1B = 0;
            pwmMD2A = 0;
            break;

        default:
            Serial2.println("Invalid direction command");
            return;
    }

    // ตั้งค่า PWM ให้กับมอเตอร์
    analogWrite(FL_PWM, pwmMD1A); // M1 = FL
    analogWrite(FR_PWM, pwmMD1B); // M2 = FR
    analogWrite(RL_PWM, pwmMD2A); // M3 = RL
    analogWrite(RR_PWM, pwmMD2B); // M4 = RR

    // Debug
    if (debug_flag) {
        snprintf(debug_buffer, sizeof(debug_buffer),
                 "Direc: %s (%d)  FL:%d  FR:%d  RL:%d  RR:%d",
                 direction_str, direc, pwmMD1A, pwmMD1B, pwmMD2A, pwmMD2B);
        DebugPublish(debug_buffer);
    }
}


/**
 * @brief ทำให้มอเตอร์หมุนอิสระ (Coast Mode)
 * @param motorNum หมายเลขมอเตอร์ (1-4)
 */
void setMotorCoastMode(uint8_t motorNum) {
    int in1, in2;
    
    switch (motorNum) {
        case 1: in1 = FL_IN1; in2 = FL_IN2; break;
        case 2: in1 = FR_IN1; in2 = FR_IN2; break;
        case 3: in1 = RL_IN1; in2 = RL_IN2; break;
        case 4: in1 = RR_IN1; in2 = RR_IN2; break;
        default: return;
    }

    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
}

/**
 * @brief ควบคุมการหมุนของมอเตอร์ M1-M4
 * @param motorNum หมายเลขมอเตอร์ (1-4)
 * @param direction ทิศทางของมอเตอร์ (0 = เดินหน้า, 1 = ถอยหลัง)
 */
void setMotorDirection(uint8_t motorNum, bool direction) {
    int in1, in2;
    
    switch (motorNum) {
        case 1: in1 = FL_IN1; in2 = FL_IN2; break;
        case 2: in1 = FR_IN1; in2 = FR_IN2; break;
        case 3: in1 = RL_IN1; in2 = RL_IN2; break;
        case 4: in1 = RR_IN1; in2 = RR_IN2; break;
        default: return;
    }

    if (direction == 0) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }
}

/**
 * @brief Callback function สำหรับรับคำสั่งความเร็วจาก `move_base` หรือแหล่งอื่นผ่าน `/cmd_vel`
 * 
 * @param cmd_vel_msg ข้อมูลที่ได้รับจาก ROS topic `/cmd_vel` (geometry_msgs::Twist)
 *                    - `linear.x` → ความเร็วในแนวแกน X (ไปหน้า/ถอยหลัง)
 *                    - `linear.y` → ความเร็วในแนวแกน Y (ซ้าย/ขวา)
 *                    - `angular.z` → ความเร็วเชิงมุม (หมุนซ้าย/ขวา)
 * 
 * @note ฟังก์ชันนี้จะทำงานเฉพาะเมื่อ `current_mode` อยู่ในโหมด `AUTO` เท่านั้น
 * 
 * @details 
 * - ค่าความเร็วที่ได้รับมาจะถูกปรับสเกลให้เหมาะสมกับ PWM โดยคูณด้วย 100 
 * - จากนั้นเรียก `controlMotors()` เพื่อควบคุมการทำงานของมอเตอร์
 * - อัปเดตตัวแปร `motor_running` เป็น `true` เพื่อบอกว่ามอเตอร์กำลังทำงาน
 * - รีเซ็ต `last_cmd_time` เป็นค่า `millis()` เพื่อป้องกันการหยุดมอเตอร์โดยไม่มีคำสั่งใหม่
 */
void cmdVelCallback(const geometry_msgs::Twist& cmd_vel_msg) {
    if (current_mode == AUTO) { // รับคำสั่งเฉพาะโหมดอัตโนมัติ
        float vx = cmd_vel_msg.linear.x * 100;    // ปรับสเกลให้เหมาะกับ PWM (ค่าต้องไม่เกิน 255)
        float vy = cmd_vel_msg.linear.y * 100;
        float omega = cmd_vel_msg.angular.z * 100;
        controlMotors(vx, vy, omega);

        // อัปเดตเวลาล่าสุดที่ได้รับคำสั่ง
        motor_running = true;
        last_cmd_time = millis();
    }
}

/**
 * @brief คำนวณและกำหนดค่าความเร็วของล้อแต่ละตัวของหุ่นยนต์ Mecanum
 * 
 * @param vx ความเร็วเชิงเส้นตามแกน X (ไปหน้า/ถอยหลัง)
 * @param vy ความเร็วเชิงเส้นตามแกน Y (เคลื่อนที่ซ้าย/ขวา)
 * @param omega ความเร็วเชิงมุม (หมุนซ้าย/ขวา)
 * 
 * @details 
 * - คำนวณค่าความเร็วสำหรับล้อทั้ง 4 ตามสูตรของล้อ Mecanum
 * - ปรับค่าสเกลให้อยู่ในช่วง -255 ถึง 255 (ค่าของ PWM)
 * - ส่งค่าความเร็วไปยัง `setMotorPWM()` เพื่อควบคุมมอเตอร์
 */
void controlMotors(float vx, float vy, float omega) {
    //  คำนวณความเร็วของล้อแต่ละตัว
    float wheel_FL = vx - vy - omega;  // ล้อหน้า-ซ้าย (Front-Left)
    float wheel_FR = vx + vy + omega;  // ล้อหน้า-ขวา (Front-Right)
    float wheel_RL = vx + vy - omega;  // ล้อหลัง-ซ้าย (Rear-Left)
    float wheel_RR = vx - vy + omega;  // ล้อหลัง-ขวา (Rear-Right)

    //  ปรับค่าสเกลให้ไม่เกิน -255 ถึง 255
    float maxVal = max(max(abs(wheel_FL), abs(wheel_FR)), 
                       max(abs(wheel_RL), abs(wheel_RR)));
    if (maxVal > 255) {
        wheel_FL *= 255.0 / maxVal;
        wheel_FR *= 255.0 / maxVal;
        wheel_RL *= 255.0 / maxVal;
        wheel_RR *= 255.0 / maxVal;
    }

    //  ส่งค่าควบคุมมอเตอร์ไปยัง `setMotorPWM()`
    // MD1 - ควบคุมมอเตอร์หน้า-ซ้าย (M1) และหน้า-ขวา (M2)
    setMotorPWM(wheel_FL, FL_IN1, FL_IN2, FL_PWM);  // M1 - FL
    setMotorPWM(wheel_FR, FR_IN1, FR_IN2, FR_PWM);  // M2 - FR
    
    // MD2 - ควบคุมมอเตอร์หลัง-ซ้าย (M3) และหลัง-ขวา (M4)
    setMotorPWM(wheel_RL, RL_IN1, RL_IN2, RL_PWM);  // M3 - RL
    setMotorPWM(wheel_RR, RR_IN1, RR_IN2, RR_PWM);  // M4 - RR
}

/**
 * @brief ตั้งค่าทิศทางและความเร็วของมอเตอร์ DC ผ่าน PWM
 * 
 * @param speed ค่า (-255 ถึง 255) ใช้กำหนดทิศทางและความเร็วของมอเตอร์
 *               - ค่า `> 0` → หมุนไปข้างหน้า
 *               - ค่า `< 0` → หมุนถอยหลัง
 *               - ค่า `0`  → หยุดมอเตอร์
 * @param in1 พินควบคุมทิศทางของมอเตอร์ (ต้องใช้ร่วมกับ in2)
 * @param in2 พินควบคุมทิศทางของมอเตอร์ (ต้องใช้ร่วมกับ in1)
 * @param pwmPin พินที่ใช้ส่งสัญญาณ PWM ไปยังมอเตอร์เพื่อควบคุมความเร็ว
 * 
 * @details 
 * - กำหนดค่าความเร็ว `speed` ให้อยู่ในช่วงที่ปลอดภัย (-255 ถึง 255)
 * - ควบคุมทิศทางของมอเตอร์โดยใช้ `digitalWrite()` กับ `in1` และ `in2`
 * - ส่งสัญญาณ PWM ไปยัง `pwmPin` เพื่อควบคุมความเร็วของมอเตอร์
 */
void setMotorPWM(float speed, int in1, int in2, int pwmPin) {
    //  คำนวณค่าความเร็วของ PWM (ปรับให้ไม่เกิน 255)
    int pwmVal = abs(speed);
    if (pwmVal > 255) pwmVal = 255;

    //  กำหนดทิศทางของมอเตอร์
    if (speed > 0) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);   // มอเตอร์หมุนไปข้างหน้า
    } else if (speed < 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);  // มอเตอร์หมุนถอยหลัง
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);   // หยุดมอเตอร์ (Brake)
    }

    //  ส่งค่า PWM ไปควบคุมความเร็วของมอเตอร์
    analogWrite(pwmPin, pwmVal);
}

/**
 * @brief ควบคุมการเคลื่อนที่ของเซอร์โวมอเตอร์ตามค่าที่ได้รับจาก ROS Topic
 * 
 * @param msg ข้อมูลที่ได้รับจาก ROS (`std_msgs::UInt8`) โดยค่าจะกำหนดการเคลื่อนที่ของเซอร์โว
 *            - `0x1` → ขยับเซอร์โวตัวที่ 1 (แนวนอน) ไปทางซ้าย (-STEP_SIZE_X)
 *            - `0x2` → ขยับเซอร์โวตัวที่ 1 (แนวนอน) ไปทางขวา (+STEP_SIZE_X)
 *            - `0x3` → ขยับเซอร์โวตัวที่ 2 (แนวตั้ง) ขึ้น (-STEP_SIZE_Y)
 *            - `0x4` → ขยับเซอร์โวตัวที่ 2 (แนวตั้ง) ลง (+STEP_SIZE_Y)
 *            - `0x5` → รีเซ็ตเซอร์โวตัวที่ 1 ไปตำแหน่งกลาง (1500 µs)
 *            - `0x6` → รีเซ็ตเซอร์โวตัวที่ 2 ไปตำแหน่งกลาง (1500 µs)
 *            - `0x7` → รีเซ็ตเซอร์โวทั้งสองตัวไปตำแหน่งกลาง (1500 µs)
 * 
 * @details
 * - ใช้ `ServoTimer2` ในการควบคุมเซอร์โว โดยค่าที่ใช้เป็นค่า `pulse width` (1000-2000 µs)
 * - จำกัดค่าการเคลื่อนที่ของเซอร์โวไม่ให้ออกนอกช่วงที่กำหนด (`MIN_POSITION` - `MAX_POSITION`)
 * - ใช้ฟังก์ชัน `moveServo(pin, value)` เพื่อกำหนดตำแหน่งของเซอร์โว
 * - ค่า `STEP_SIZE_X` และ `STEP_SIZE_Y` ใช้กำหนดระดับการเคลื่อนที่ของเซอร์โวในแต่ละคำสั่ง
 */
void command_servo(const std_msgs::UInt8& msg) {
    if (msg.data > 0x7) {
        serial2Print("Invalid servo value", msg.data);
        return;
    }
    switch (msg.data) {
        case 0x1: // ซ้าย
            current_position_1 = max(current_position_1 - STEP_SIZE_X, MIN_POSITION);
            servo1.writeMicroseconds(current_position_1);
            break;

        case 0x2: // ขวา
            current_position_1 = min(current_position_1 + STEP_SIZE_X, MAX_POSITION);
            servo1.writeMicroseconds(current_position_1);
            break;

        case 0x3: // ขึ้น
            current_position_2 = max(current_position_2 - STEP_SIZE_Y, MIN_POSITION);
            servo2.writeMicroseconds(current_position_2);
            break;

        case 0x4: // ลง
            current_position_2 = min(current_position_2 + STEP_SIZE_Y, MAX_POSITION);
            servo2.writeMicroseconds(current_position_2);
            break;

        case 0x5: // รีเซ็ต servo1
            current_position_1 = 1500;
            servo1.writeMicroseconds(current_position_1);
            break;

        case 0x6: // รีเซ็ต servo2
            current_position_2 = 1500;
            servo2.writeMicroseconds(current_position_2);
            break;

        case 0x7: // รีเซ็ตทั้งคู่
            current_position_1 = 1500;
            current_position_2 = 1500;
            servo1.writeMicroseconds(current_position_1);
            servo2.writeMicroseconds(current_position_2);
            break;

        default:
            serial2Print("Invalid servo value", msg.data);
            break;
    }
    // แสดงตำแหน่ง Pan / Tilt หลังสั่งงาน
    if (debug_flag) {
        int pan_deg  = map(current_position_1, MIN_POSITION, MAX_POSITION, 0, 180);
        int tilt_deg = map(current_position_2, MIN_POSITION, MAX_POSITION, 0, 180);
        snprintf(debug_buffer, sizeof(debug_buffer),
                 "Servo Pan: %d°  Tilt: %d°", pan_deg, tilt_deg);
        DebugPublish(debug_buffer);
    }
}

/**
 * @brief ฟังก์ชันสำหรับปรับค่าตัวแปรและโหมดการทำงานผ่าน ROS Message
 * @param msg ข้อมูลที่ได้รับเป็น `std_msgs::UInt32` ที่ประกอบด้วย:
 *        - 8 บิตแรก (MSB) ใช้เป็นรหัสตัวแปรที่ต้องการเปลี่ยนค่า
 *        - 24 บิตล่าง ใช้เก็บค่าของตัวแปรนั้น ๆ
 */
void commandEdit(const std_msgs::UInt32& msg) {
    // ดึงรหัสตัวแปรจาก 8 บิตแรกของข้อมูลที่ได้รับ
    uint8_t variable_id = (msg.data >> 24) & 0xFF;

    // ใช้ switch-case เพื่อตรวจสอบว่าต้องแก้ไขค่าตัวแปรใด
    switch (variable_id) {
        case 0x01: { // แก้ไขค่า time_delay
            time_delay = msg.data & 0xFFFFFF; // 24 บิตล่างคือค่าที่ต้องการเปลี่ยน
            snprintf(debug_buffer, sizeof(debug_buffer), "Updated time_delay to: %d", time_delay);
            DebugPublish(debug_buffer);
            break;
        }

        case 0x02: { // แก้ไขค่า STEP_SIZE_X (ขนาด step ของ Servo X)
            STEP_SIZE_X = msg.data & 0xFFFFFF;
            snprintf(debug_buffer, sizeof(debug_buffer), "Updated STEP_SIZE_X to: %d", STEP_SIZE_X);
            DebugPublish(debug_buffer);
            break;
        }

        case 0x03: { // แก้ไขค่า STEP_SIZE_Y (ขนาด step ของ Servo Y)
            STEP_SIZE_Y = msg.data & 0xFFFFFF;
            snprintf(debug_buffer, sizeof(debug_buffer), "Updated STEP_SIZE_Y to: %d", STEP_SIZE_Y);
            DebugPublish(debug_buffer);
            break;
        }

        case 0x05: { // เปลี่ยนโหมดควบคุมมอเตอร์ (AUTO / MANUAL)
            uint8_t flag_mode = msg.data & 0xFF;

            MotorCoastMode(); // หยุดมอเตอร์ก่อนเปลี่ยนโหมด

            if (flag_mode == 0) {
                current_mode = AUTO;
            } else if (flag_mode == 1) {
                current_mode = MANUAL;
            }

            snprintf(debug_buffer, sizeof(debug_buffer), "Switched to %s mode", flag_mode == 0 ? "AUTO" : "MANUAL");
            DebugPublish(debug_buffer);
            break;
        }

        case 0x06: { // เปิด / ปิดโหมด Debug
            uint8_t flag_debug = msg.data & 0xFF;
            debug_flag = flag_debug;
            break;
        }

        case 0x07: { // ขอข้อมูลค่าตัวแปรที่ใช้งานอยู่
            snprintf(debug_buffer, sizeof(debug_buffer),
                     "{ \"STEP_Servo_X\": %d, \"STEP_Servo_Y\": %d, \"debug_flag\": %d, \"time_delay\": %d }",
                     STEP_SIZE_X, STEP_SIZE_Y, debug_flag, time_delay);
            DebugPublish(debug_buffer);
            break;
        }

        case 0x08: { // ควบคุม Relay
            uint8_t flag_relay = msg.data & 0xFF;

            switch (flag_relay) {
                case 0x00:
                    digitalWrite(RELAY1, LOW);
                    snprintf(debug_buffer, sizeof(debug_buffer), "R1 0");
                    DebugPublish(debug_buffer);
                    break;

                case 0x01:
                    digitalWrite(RELAY1, HIGH);
                    snprintf(debug_buffer, sizeof(debug_buffer), "R1 1");
                    DebugPublish(debug_buffer);
                    break;

                case 0x02:
                    digitalWrite(RELAY2, LOW);
                    snprintf(debug_buffer, sizeof(debug_buffer), "R2 0");
                    DebugPublish(debug_buffer);
                    break;

                case 0x03:
                    digitalWrite(RELAY2, HIGH);
                    snprintf(debug_buffer, sizeof(debug_buffer), "R2 1");
                    DebugPublish(debug_buffer);
                    break;

                default:
                    snprintf(debug_buffer, sizeof(debug_buffer), "Invalid relay command");
                    DebugPublish(debug_buffer);
                    break;
            }

            break;
        }
        case 0x09: { // อับเดตค่า use_imu
            uint8_t flag_imu = msg.data & 0xFF;
            if (flag_imu == 0) {
                use_imu = false;
            } else if (flag_imu == 1) {
                use_imu = true;
            }
            snprintf(debug_buffer,  sizeof(debug_buffer), "use_imu %d",flag_imu);
            DebugPublish(debug_buffer);
            break;
        }
        case 0x0A: { // อับเดตค่า use_imu_mag
            uint8_t flag_imu_mag = msg.data & 0xFF;
            if (flag_imu_mag == 0) {
                use_imu_mag = false;
            } else if (flag_imu_mag == 1) {
                use_imu_mag = true;
            }
            snprintf(debug_buffer,  sizeof(debug_buffer), "use_imu_mag %d",use_imu_mag);

            DebugPublish(debug_buffer);
            break;
        }

        default: { // กรณีที่ได้รับคำสั่งที่ไม่รู้จัก
            snprintf(debug_buffer, sizeof(debug_buffer), "Invalid command edit ID: %d", variable_id);
            DebugPublish(debug_buffer);
            break;
        }
    }
}
/**
 * @brief รับคำสั่งควบคุมมอเตอร์จาก ROS และส่งให้หุ่นยนต์เคลื่อนที่ (เฉพาะโหมด Manual)
 *
 * ฟังก์ชันนี้ใช้สำหรับรับข้อมูลจาก ROS (ผ่าน `std_msgs::UInt16`)  
 * และแยกข้อมูลเป็น **ทิศทาง (`direction`) และ PWM (`pwm`)**  
 * จากนั้นเรียก `moveRobot()` เพื่อควบคุมมอเตอร์ แต่จะทำงาน **เฉพาะในโหมด MANUAL** เท่านั้น
 * ตัวอย่างค่าที่ส่งมาจาก ROS
 * msg.data (16-bit)	           ค่า Direction  (8-bit)	  ค่า PWM (8-bit)	ทิศทางที่ได้
 * 0x010F (0000 0001 0000 1111)	 0x01 (1)	    0x0F (15)	 เดินหน้า PWM=15
 * 0x040A (0000 0100 0000 1010)	 0x04 (4)	    0x0A (10)	 ถอยหลัง PWM=10
 * 0x0700 (0000 0111 0000 0000)	 0x07 (7)	    0x00 (0)	 เดินหน้าเฉียงซ้าย PWM=0  
 * 
 * @param msg ข้อมูลคำสั่งจาก ROS (`std_msgs::UInt16`)
 *  - **8 บิตแรก** (MSB) → ทิศทาง (`direction`) (1-10)
 *  - **8 บิตหลัง** (LSB) → ค่าความเร็ว PWM (0-255)
 *
 * @note ถ้า `current_mode != MANUAL` ฟังก์ชันนี้จะไม่ทำงาน
 */
void DriveCallback(const std_msgs::UInt16& dirve_msg) {
    // ตรวจสอบว่าอยู่ในโหมด Manual เท่านั้น
    if (current_mode == MANUAL) {
        // Debug แสดงข้อมูลที่ได้รับ

        // แยกข้อมูลเป็นทิศทาง (direction) และค่าความเร็ว (PWM)
        uint8_t direction = (dirve_msg.data >> 8) & 0xFF;  // ดึง 8 บิตแรก (MSB) เป็นทิศทาง
        uint8_t pwm = dirve_msg.data & 0xFF;               // ดึง 8 บิตหลัง (LSB) เป็นค่า PWM

        // ส่งคำสั่งไปยังฟังก์ชันควบคุมหุ่นยนต์
        moveRobot(direction, pwm,pwm,pwm,pwm);

        // อัปเดตสถานะว่ามอเตอร์กำลังทำงาน
        motor_running = true;

        // บันทึกเวลาปัจจุบัน เพื่อใช้ตรวจจับว่าไม่มีคำสั่งใหม่แล้วหรือไม่
        last_cmd_time = millis();
    }
}

void DebugPublish(const char* data){
  if(debug_flag){
    debug_msgs.data = (char *)data;
    debug_pub.publish(&debug_msgs);
    Serial2.println(debug_msgs.data);// and serial2 debug
  }
}

/**
 * @brief อ่านข้อมูลที่ได้รับจาก `Serial1` และประมวลผลคำสั่ง
 *
 * ฟังก์ชันนี้ทำหน้าที่อ่านข้อมูลที่ส่งมาผ่าน `Serial1` (UART)  
 * และนำไปประมวลผลหากเป็นคำสั่งที่ต้องการ โดยเฉพาะคำสั่ง `"D "` (D + space)  
 * ซึ่งใช้สำหรับอัปเดตค่าความเร็ว (`vx`, `vy`, `omega`) และเผยแพร่ Odometry
 *
 * @note ฟังก์ชันนี้จะตัดข้อความเมื่อเจอ '\n' และป้องกัน Buffer Overflow ด้วยขนาด 32 ไบต์
 */
void processSerialData(char c) {
    static String buffer = "";
    
    if (c == '\n') {  // จบข้อความเมื่อพบ '\n'
        processReceivedCommand(buffer.c_str());  // ส่งข้อความไปประมวลผล
        buffer = "";  // เคลียร์ buffer
    } else {
        buffer += c;  // เก็บข้อมูลเพิ่มเข้าไปใน buffer
    }
}


void processReceivedCommand(const char* received) {

    if (received[0] == 'D' && received[1] == ' ') {
        char* token = strtok(received + 2, " ");
        if (token) vx = atof(token);
        token = strtok(NULL, " ");
        if (token) vy = atof(token);
        token = strtok(NULL, " ");
        if (token) omega = atof(token);

    }
    else if(strncmp(received, "ACK ", 4) == 0){
        serial2Print("ACK ", received);
    }
}

/**
 * @brief อ่านข้อมูลจาก MPU6050 และเผยแพร่เป็น ROS IMU Message
 * 
 * @details 
 * - อ่านค่าจาก FIFO Buffer ของ MPU6050 (ใช้ DMP)
 * - ตรวจสอบ Buffer Overflow และรีเซ็ตหากเกินขีดจำกัด
 * - อ่านค่า Quaternion และคำนวณค่ามุม yaw, pitch, roll
 * - อ่านค่าความเร่ง (Accelerometer) และอัตราการหมุน (Gyroscope)
 * - แปลงค่าที่ได้ให้อยู่ในหน่วยที่เหมาะสม
 * - normalizeQuaternion
 * - เผยแพร่ข้อมูลไปยัง ROS Topic `imu/data_raw`
 */
void readIMU() {
    int fifoCount = mpu.getFIFOCount(); // ตรวจสอบจำนวนข้อมูลที่มีอยู่ใน FIFO
    if (fifoCount < 42) return;         // ถ้ายังไม่มีข้อมูลเพียงพอ ให้รอ

    // ตรวจสอบกรณี FIFO Buffer Overflow
    if (fifoCount >= 1024) {  
        mpu.resetFIFO();  // รีเซ็ต FIFO เพื่อป้องกันข้อมูลเสียหาย
        return;
    }

    // อ่านข้อมูลจาก FIFO ทีละ 42 ไบต์ (MPU DMP ใช้ขนาดนี้)
    while (fifoCount >= 42) {
        mpu.getFIFOBytes(fifoBuffer, 42);
        fifoCount -= 42;

        Quaternion q;
        VectorFloat gravity;
        float ypr[3]; 

        // ดึงข้อมูล Quaternion และคำนวณ yaw, pitch, roll
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // ตั้งค่า Header ของ ROS IMU Message
        imu_msg.header.stamp = nh.now();
        imu_msg.header.frame_id = "imu_link";

        // ใช้ Quaternion จาก DMP
        imu_msg.orientation.w = q.w;
        imu_msg.orientation.x = q.x;
        imu_msg.orientation.y = q.y;
        imu_msg.orientation.z = q.z;

        // อ่านค่าความเร่งและอัตราการหมุนจากเซ็นเซอร์
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // **แปลงหน่วยให้ถูกต้อง**
        float accelScale = 16384.0;  // MPU6050 ในโหมด ±2g (1g = 9.81 m/s²)
        float gyroScale = 131.0;     // MPU6050 ในโหมด ±250°/s 

        imu_msg.linear_acceleration.x = (ax / accelScale) * 9.81;
        imu_msg.linear_acceleration.y = (ay / accelScale) * 9.81;
        imu_msg.linear_acceleration.z = (az / accelScale) * 9.81;

        imu_msg.angular_velocity.x = (gx / gyroScale) * (PI / 180.0); // rad/s
        imu_msg.angular_velocity.y = (gy / gyroScale) * (PI / 180.0);
        imu_msg.angular_velocity.z = (gz / gyroScale) * (PI / 180.0);

        
        normalizeQuaternion(imu_msg);
        // ส่งข้อมูลไปยัง ROS Topic
        imu_raw_pub.publish(&imu_msg);
    }
}


/**
 * @brief Normalize Quaternion จาก MPU6050
 * 
 * @details 
 * - ทำให้ quaternion ใน Imu &msg มีความยาว (magnitude) เท่ากับ 1
 */
void normalizeQuaternion(sensor_msgs::Imu &msg) {
    float norm = sqrt(msg.orientation.x * msg.orientation.x +
                      msg.orientation.y * msg.orientation.y +
                      msg.orientation.z * msg.orientation.z +
                      msg.orientation.w * msg.orientation.w);

    if (norm > 0.0) {  // ป้องกันการหารด้วย 0
        msg.orientation.x /= norm;
        msg.orientation.y /= norm;
        msg.orientation.z /= norm;
        msg.orientation.w /= norm;
    }
}

void readCompass() {   
    Wire.beginTransmission(QMC5883L_ADDRESS);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) {
        Serial2.println("[ERROR] Compass not responding!");
        return;
    }

    Wire.requestFrom(QMC5883L_ADDRESS, 6);
    uint8_t timeout = 10;
    while (Wire.available() < 6 && timeout > 0) {
        delayMicroseconds(100);
        timeout--;
    }
    if (Wire.available() < 6) {
        Serial2.println("[ERROR] Compass read timeout!");
        return;
    }

    int16_t x = Wire.read(); x |= Wire.read() << 8;
    int16_t y = Wire.read(); y |= Wire.read() << 8;
    Wire.read(); Wire.read();  // skip Z

    float rawX = static_cast<float>(x) - OFFSET_X;
    float rawY = static_cast<float>(y) - OFFSET_Y;

    if (abs(rawX) > 5000 || abs(rawY) > 5000) {
        Serial2.println("[ERROR] Compass data out of range!");
        return;
    }

    // mag_msg.header.stamp = nh.now();
    // mag_msg.header.frame_id = "mag_link";
    // mag_msg.magnetic_field.x = rawX;
    // mag_msg.magnetic_field.y = rawY;
    // mag_msg.magnetic_field.z = 0.0;
    // mag_pub.publish(&mag_msg);

    // กรอง heading
    static float heading_prev = 0.0;
    const float alpha = 0.1;

    //คำนวณ heading
    float heading = atan2(rawY, rawX);
    if (heading < 0)
        heading += 2 * PI;

    heading = alpha * heading + (1 - alpha) * heading_prev;
    heading_prev = heading;

    float qx, qy, qz, qw;
    yawToQuaternion(heading, qx, qy, qz, qw);

    mag_msg.header.stamp = nh.now();
    mag_msg.header.frame_id = "mag_link";

    mag_msg.orientation.x = qx;
    mag_msg.orientation.y = qy;
    mag_msg.orientation.z = qz;
    mag_msg.orientation.w = qw;

    mag_msg.orientation_covariance[0] = -1.0;
    mag_msg.orientation_covariance[4] = -1.0;
    mag_msg.orientation_covariance[8] = 0.05;

    mag_pub.publish(&mag_msg);
}

void yawToQuaternion(float yaw, float& qx, float& qy, float& qz, float& qw) {
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);

    qx = 0.0;
    qy = 0.0;
    qz = sy;
    qw = cy;
}

/**
 * @brief ปรับตำแหน่งของเซอร์โวมอเตอร์ด้วยค่า PWM ที่กำหนด
 * 
 * @param pin พินที่เชื่อมต่อกับเซอร์โว (`SERVO1_PIN` หรือ `SERVO2_PIN`)
 * @param pwm_value ค่ามุมของเซอร์โวในรูปแบบของไมโครวินาที (us)
 * 
 * @details
 * - จำกัดค่าของ `pwm_value` ให้อยู่ในช่วงที่เซอร์โวรับได้ (`MIN_POSITION` ถึง `MAX_POSITION`)
 * - ตรวจสอบว่าค่าพินที่รับเข้ามาตรงกับเซอร์โวตัวที่ 1 (`SERVO1_PIN`) หรือ เซอร์โวตัวที่ 2 (`SERVO2_PIN`)
 * - ใช้ไลบรารี `ServoTimer2` ในการสั่งให้เซอร์โวเคลื่อนที่ไปยังตำแหน่งที่กำหนด
 */
void moveServo(int pin, int pwm_value) {
    // จำกัดค่าการเคลื่อนที่ของเซอร์โวให้อยู่ในช่วงที่กำหนด
    pwm_value = constrain(pwm_value, MIN_POSITION, MAX_POSITION); 

    // ตรวจสอบว่าพินตรงกับเซอร์โวตัวไหน และสั่งให้เคลื่อนที่
    if (pin == SERVO1_PIN) {
        servo1.write(pwm_value);
    } else if (pin == SERVO2_PIN) {
        servo2.write(pwm_value);
    }
}

/**
 * @brief อ่านค่ากระแสไฟฟ้าจากเซ็นเซอร์ ACS712-05A และแปลงเป็นหน่วยมิลลิแอมป์ (mA)
 * 
 * @return ค่าในหน่วยมิลลิแอมป์ (mA)
 * 
 * @details
 * - อ่านค่าจากพินอนาล็อกที่เชื่อมต่อกับ ACS712
 * - แปลงค่า ADC (0-1023) เป็นแรงดันไฟฟ้า (V)
 * - ใช้แรงดันไฟฟ้าที่อ่านได้คำนวณค่ากระแสไฟฟ้าโดยใช้ค่า Sensitivity ของเซ็นเซอร์
 * - ค่าที่ได้จาก ACS712 เป็นแรงดันอ้างอิงศูนย์ที่ 2.5V (สำหรับการวัดกระแส AC/DC)
 * - ค่ากระแสที่ได้จะถูกแปลงจากแอมป์ (A) เป็นมิลลิแอมป์ (mA) ก่อนส่งคืน
 */
uint16_t readCurrent() {
    int sensorValue = analogRead(CURRENT_SENSOR_PIN);

    // แปลงค่าดิบเป็นแรงดัน (V)
    float voltage = ((float)sensorValue / ADC_RESOLUTION) * REFERENCE_VOLTAGE;

    // แปลง offset ที่ calibrate มา (ACS712_ZERO_RAW) → เป็นแรงดัน (V)
    float zeroVoltage = ((float)ACS712_ZERO_RAW / ADC_RESOLUTION) * REFERENCE_VOLTAGE;

    // คำนวณกระแสจากแรงดันที่เบี่ยงเบนจากศูนย์
    float current = (voltage - zeroVoltage) / ACS712_SENSITIVITY;

    float currentmA = current * 1000.0 / 5.615;
    serial2Print("current_mA : ", currentmA);

    return (uint16_t)(abs(currentmA));  // แปลงเป็น unsigned 16-bit mA
}

/**
 * @brief อ่านค่าแรงดันไฟฟ้าจาก Voltage Sensor และแปลงเป็นเซนติโวลต์ (cV)
 * 
 * @return ค่าแรงดันไฟฟ้าในหน่วยเซนติโวลต์ (cV)
 * 
 * @details
 * - อ่านค่าจากพินอนาล็อกที่เชื่อมต่อกับ Voltage Sensor
 * - แปลงค่า ADC (0-1023) เป็นแรงดันไฟฟ้า (V) โดยใช้แรงดันอ้างอิง (`REFERENCE_VOLTAGE`)
 * - คำนวณแรงดันที่แท้จริงโดยคูณด้วยค่า **Voltage Divider Ratio**
 * - แปลงค่าโวลต์ (V) เป็นเซนติโวลต์ (cV) เพื่อความละเอียดที่มากขึ้น
 * - ค่า `VOLTAGE_DIVIDER_RATIO` คืออัตราการแบ่งแรงดันจากวงจรแบ่งแรงดัน (Voltage Divider)
 */
uint16_t readVoltage() {
    int val = analogRead(VOLTAGE_SENSOR_PIN);  // อ่านค่า ADC
    
    // แปลงค่า ADC → แรงดันจริงที่วัดได้จากโมดูล
    // โมดูลนี้ใช้วงจรแบ่งแรงดัน (Voltage Divider) ด้วย R1 = 30k, R2 = 7.5k
    // ทำให้แรงดันที่ Arduino เห็น = แรงดันจริง ÷ 5
    // ดังนั้นแรงดันจริง = ค่า ADC × (5V ÷ 1024) × 5
    //                      = ค่า ADC × 0.0048828125 × 5
    //                      ≈ ค่า ADC × 0.02443
    // 1.2578 (add offset)
    float voltage = val * 0.02443 *  1.2578 ;       // แปลงเป็นแรงดัน (V) จากโมดูล 25V
    serial2Print("Voltage : ", voltage); // พิมพ์ค่าแรงดัน
    return (uint16_t)(voltage * 100);    // แปลงเป็น centivolt (cV)
}

//debug serial2
  void serial2Print(const String& des, int data) {
      if (DEBUG_MODE_S2) {
          Serial2.print(des);
          Serial2.println(data);
      }
  }

  void serial2Print(const String& des, float data) {
      if (DEBUG_MODE_S2) {
          Serial2.print(des);
          Serial2.println(data);
      }
  }

  void serial2Print(const String& des, const String& data) {
      if (DEBUG_MODE_S2) {
          Serial2.print(des);
          Serial2.println(data);
      }
  }




