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
#include <tf/transform_broadcaster.h> 
#include <nav_msgs/Odometry.h>

#include <avr/io.h>                         // ไลบรารีควบคุม I/O ของ ATmega
#include <avr/interrupt.h>                  // ไลบรารีสำหรับใช้ Interrupts
#include <Wire.h>                            // ไลบรารีสำหรับ I2C communication
#include <Servo.h>                          // ไลบรารีควบคุม Servo
#include <math.h>                           // IMU normalized
#include <stdlib.h>

// การกำหนดพินของมอเตอร์ไดรเวอร์ (Motor Driver)
const uint8_t md1_AIN2  = 35;
const uint8_t md1_AIN1  = 37;
const uint8_t md1_BIN1  = 41;
const uint8_t md1_BIN2  = 43;
const uint8_t md1_PWMA  = 12;
const uint8_t md1_PWMB  = 13;

const uint8_t md1_STBY  = 39;
                    
const uint8_t md2_AIN2 = 40;
const uint8_t md2_AIN1 = 42;
const uint8_t md2_BIN1 = 34;
const uint8_t md2_BIN2 = 36;
const uint8_t md2_PWMA = 10;
const uint8_t md2_PWMB = 11;

const uint8_t md2_STBY = 38;  // ขาสำหรับเปิดใช้งานมอเตอร์ (STBY)

// Wheels mapping 
const uint8_t FL_IN1 = md1_AIN1;
const uint8_t FL_IN2 = md1_AIN2;
const uint8_t FL_PWM = md1_PWMA;

const uint8_t FR_IN1 = md2_AIN1;
const uint8_t FR_IN2 = md2_AIN2;
const uint8_t FR_PWM = md2_PWMA;

const uint8_t RL_IN1 = md2_BIN1;
const uint8_t RL_IN2 = md2_BIN2;
const uint8_t RL_PWM = md2_PWMB;

const uint8_t RR_IN1 = md1_BIN1;
const uint8_t RR_IN2 = md1_BIN2;
const uint8_t RR_PWM = md1_PWMB;


// Motor 1: Front-Left (FL)
const uint8_t ENCODER_FL_A = 2;   // Hardware Interrupt Pin (INT.4) -> Port E, Bit 4 (PE4)
const uint8_t ENCODER_FL_B = 4;  // Port G, Bit 5 (PG5)

// Motor 2: Rear-Right (RR)
const uint8_t ENCODER_RR_A = 3;  // Hardware Interrupt Pin (INT.5) -> Port E, Bit 5 (PE5)
const uint8_t ENCODER_RR_B = 5;  // Port E, Bit 3 (PE3)

// Motor 3: Front-Right (FR)
const uint8_t ENCODER_FR_A = 18;  // Hardware Interrupt Pin (INT.3) -> Port D, Bit 3 (PD3)
const uint8_t ENCODER_FR_B = 22; // Port A, Bit 0 (PA0)

// Motor 4: Rear-Left (RL)
const uint8_t ENCODER_RL_A = 19; // Hardware Interrupt Pin (INT.2) -> Port D, Bit 2 (PD2)
const uint8_t ENCODER_RL_B = 23; // Port A, Bit 1 (PA1)



const uint8_t RELAY1 = 33;    // pin relay 1
const uint8_t RELAY2 = 32;   // pin relay 2

//Robot Physical Parameters
const float WHEEL_RADIUS = 0.04;     // รัศมีล้อ (เมตร)
const float L1 = 0.105;              // ระยะครึ่งหนึ่งของความยาวหุ่นยนต์ (เมตร)
const float L2 = 0.0825;            // ระยะครึ่งหนึ่งของความกว้างหุ่นยนต์ (เมตร)
const float PPR = 660.0;             // Pulses Per Revolution ของ Encoder
const float MAX_LINEAR_SPEED = 0.5;   // ความเร็วไปข้างหน้า/ด้านข้างสูงสุด (m/s)
const float MAX_ANGULAR_SPEED = 1.0;  // ความเร็วในการหมุนสูงสุด (rad/s)

//Encoder Reset Thresholds
const long MAX_POSITION_ENC = 20000000;
const long ENCODER_MIDPOINT = 10000000;

// --- Main Loop Timings ---
const unsigned long ODOM_INTERVAL_MS = 40;         // คำนวณและส่ง Odometry ทุก 40ms (25 Hz)
const unsigned long COMPASS_INTERVAL_MS = 100;     // อ่าน Compass ทุก 100ms (10 Hz)
const unsigned long IMU_INTERVAL_MS = 50;          // อ่าน IMU ทุก 50ms (20 Hz)
const unsigned long POWER_INTERVAL_MS = 1000;      // อ่าน Power Sensor ทุก 1000ms (1 Hz)
const unsigned long DEBUG_INTERVAL_MS = 500;
const unsigned int  ENCODER_DEBOUNCE_DELAY_US = 800; // ใช้ unsigned int สำหรับค่าที่ไม่ใหญ่มาก

// ค่าคงที่ของ Compass Sensor (QMC5883L)
const uint8_t QMC5883L_ADDRESS = 0x0D; // uint8_t หรือ byte เหมาะสำหรับ I2C address
const int MIN_X = -1363;
const int MAX_X = 665;
const int MIN_Y = -1932;
const int MAX_Y = 32;
const int MIN_Z = -568;
const int MAX_Z = -265;

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

// --- พินของเซ็นเซอร์วัดกระแสและแรงดันไฟฟ้า ---
const uint8_t CURRENT_SENSOR_PIN = A9;
const uint8_t VOLTAGE_SENSOR_PIN = A8;

// --- ค่าคงที่สำหรับเซ็นเซอร์วัดกระแสไฟฟ้า (ACS712-20A) ---
const float ACS712_SENSITIVITY = 0.100;    // 0.100 V/A
const int   ACS712_ZERO_RAW    = 511;       // ค่า offset ที่ได้จากการ calibrate

// --- ค่าอ้างอิงของ ADC ---
const float ADC_RESOLUTION    = 1024.0;
const float REFERENCE_VOLTAGE = 5.0;       // อ้างอิงแรงดันที่ 5V


//Encoder Variables +++
volatile long counter_FL = ENCODER_MIDPOINT;
volatile long counter_FR = ENCODER_MIDPOINT;
volatile long counter_RL = ENCODER_MIDPOINT;
volatile long counter_RR = ENCODER_MIDPOINT;

volatile byte prevA_FL;
volatile byte prevA_FR;
volatile byte prevA_RL;
volatile byte prevA_RR;

long prevFL = ENCODER_MIDPOINT;
long prevFR = ENCODER_MIDPOINT;
long prevRL = ENCODER_MIDPOINT;
long prevRR = ENCODER_MIDPOINT;


volatile unsigned long last_interrupt_time_FL = 0;
volatile unsigned long last_interrupt_time_FR = 0;
volatile unsigned long last_interrupt_time_RL = 0;
volatile unsigned long last_interrupt_time_RR = 0;

// พินของ Servo Motor
const uint8_t SERVO1_PIN = 44;
const uint8_t SERVO2_PIN = 46;

// ค่าเริ่มต้นของ Servo
Servo servo1, servo2;
const uint16_t MIN_POSITION_SV = 1000;   // ตำแหน่งต่ำสุดของ Servo (us)
const uint16_t MAX_POSITION_SV = 2000;   // ตำแหน่งสูงสุดของ Servo (us)
uint16_t current_position_1 = 1500;  // ตำแหน่งเริ่มต้นของ Servo1 (90°)
uint16_t current_position_2 = 1500;  // ตำแหน่งเริ่มต้นของ Servo2 (90°)
uint16_t STEP_SIZE_X = 55;  // ขยับ 10 องศา ≈ 55us
uint16_t STEP_SIZE_Y = 28;  // ขยับ 5 องศา ≈ 28us

// Debugging
char debug_buffer[128];
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
std_msgs::String debug_msgs;
std_msgs::UInt32 sensor_data_msg;
geometry_msgs::Twist cmd_vel_msg;
nav_msgs::Odometry odom_msg;

// ROS Publishers
ros::Publisher odom_pub("/odom", &odom_msg);
ros::Publisher imu_pub("/imu/data", &imu_msg);
ros::Publisher debug_pub("debug/data", &debug_msgs);
ros::Publisher sensor_data_pub("/sensor/data", &sensor_data_msg);

// ROS Subscribers
ros::Subscriber<std_msgs::UInt16> sub_drive("/rb/cm/dr", DriveCallback);
ros::Subscriber<std_msgs::UInt8> sub_servo("/rb/cm/sv", command_servo);
ros::Subscriber<std_msgs::UInt32> sub_edit("/rb/cm/ed", commandEdit);
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("/cmd_vel", cmdVelCallback);

tf::TransformBroadcaster broadcaster;
geometry_msgs::TransformStamped odom_trans;
char odom_frame[] = "odom";
char base_link_frame[] = "base_link";
double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0; 

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
    Serial.begin(250000);     // Serial หลักสำหรับ rosserial 250000
    Serial2.begin(115200);       // Serial2 สำหรับ Debugging เพิ่มเติม
    Wire.begin();                // เริ่มใช้งาน I2C
    Wire.setClock(400000);       // ตั้งค่า I2C Speed เป็น 400kHz (Fast Mode)

    // ตั้งค่า ROS Node
    nh.getHardware()->setBaud(250000);
    nh.initNode();
    
    // กำหนด Publisher สำหรับส่งข้อมูลไปยัง ROS
    nh.advertise(imu_pub); 
    nh.advertise(debug_pub);
    nh.advertise(sensor_data_pub);
    nh.advertise(odom_pub);
    broadcaster.init(nh);

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

    //Encoder Pin Initialization
    pinMode(ENCODER_FL_A, INPUT_PULLUP);
    pinMode(ENCODER_FL_B, INPUT_PULLUP);
    pinMode(ENCODER_FR_A, INPUT_PULLUP);
    pinMode(ENCODER_FR_B, INPUT_PULLUP);
    pinMode(ENCODER_RL_A, INPUT_PULLUP);
    pinMode(ENCODER_RL_B, INPUT_PULLUP);
    pinMode(ENCODER_RR_A, INPUT_PULLUP);
    pinMode(ENCODER_RR_B, INPUT_PULLUP);
    //Attach Interrupts for Encoders
    attachInterrupt(digitalPinToInterrupt(ENCODER_FL_A), isr_FL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_FR_A), isr_FR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RL_A), isr_RL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RR_A), isr_RR, CHANGE);

    // อ่านค่าเริ่มต้นของขา A แต่ละตัวเก็บไว้ในตัวแปร prevA
    // เพื่อให้การทำงานใน ISR ครั้งแรกถูกต้อง
    prevA_FL = (PINE >> PINE4) & 1;
    prevA_FR = (PIND >> PIND3) & 1;
    prevA_RL = (PIND >> PIND2) & 1;
    prevA_RR = (PINE >> PINE5) & 1;


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
    static unsigned long lastOdomCalcTime = 0;
    static unsigned long lastCompassTime = 0;
    static unsigned long lastIMUTime = 0;
    static unsigned long lastPowerSensorTime = 0;
    static unsigned long lastDebugTime = 0;
    unsigned long now = millis();
    // Calculate and Publish Odometry Velocity
    if (now - lastOdomCalcTime >= ODOM_INTERVAL_MS) {
        calculateAndPublishOdometry();
        lastOdomCalcTime = now;

    }
    // Read MPU6050 and comapss
    if (now - lastIMUTime >= IMU_INTERVAL_MS) {
        publishFusedIMUData(); // <-- เรียกใช้ฟังก์ชันใหม่ที่รวมข้อมูล
        lastIMUTime = now;
    }

    // Read Power Sensors
    if (now - lastPowerSensorTime >= POWER_INTERVAL_MS) {  
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
            /* 
            if (debug_flag) {
                snprintf(debug_buffer, sizeof(debug_buffer),
                        "Auto-stop: No cmd received");
                DebugPublish(debug_buffer);
            }*/

        }
    }

    if (debug_flag && (now - lastDebugTime >= DEBUG_INTERVAL_MS)) {
        char encoder_debug_buffer[80];
        snprintf(encoder_debug_buffer, sizeof(encoder_debug_buffer),
                 "Encoders: FL:%ld FR:%ld RL:%ld RR:%ld",
                 counter_FL, counter_FR, counter_RL, counter_RR);
        DebugPublish(encoder_debug_buffer);
        lastDebugTime = now;
    }
    checkAndResetEncoder(counter_FL);
    checkAndResetEncoder(counter_FR);
    checkAndResetEncoder(counter_RL);
    checkAndResetEncoder(counter_RR);
    nh.spinOnce();
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
 * @brief (ล่าม) แปลคำสั่งควบคุมแบบ Manual (ทิศทาง + PWM)
 * ให้เป็นคำสั่งความเร็ว (vx, vy, omega) แล้วส่งให้ controlMotors() คำนวณต่อ
 * * @param direc ทิศทางการเคลื่อนที่ของหุ่นยนต์ (1-10)
 * @param pwm ค่าความเร็ว PWM ที่ได้รับ (0-255)
 */
void moveRobot(uint8_t direc, uint8_t pwm) {
    // แปลงค่า pwm (0-255) ให้เป็น "ขนาดของความเร็ว" ที่จะใช้ในการคำนวณ
    float speed_magnitude = (float)pwm;
    float vx = 0.0, vy = 0.0, omega = 0.0;

    // แปลงคำสั่งทิศทางให้เป็นค่า vx, vy, omega
    switch (direc) {
        case 1:  // Forward
            vx = speed_magnitude;
            break;
        case 2:  // Left (Strafe)
            vy = -speed_magnitude;
            break;
        case 3:  // Right (Strafe)
            vy = speed_magnitude;
            break;
        case 4:  // Backward
            vx = -speed_magnitude;
            break;
        case 5:  // Turn Left
            omega = speed_magnitude;
            break;
        case 6:  // Turn Right
            omega = -speed_magnitude;
            break;
        case 7:  // Forward-Left
            vx = speed_magnitude * 0.707;
            vy = -speed_magnitude * 0.707;
            break;
        case 8:  // Forward-Right
            vx = speed_magnitude * 0.707;
            vy = speed_magnitude * 0.707;
            break;
        case 9:  // Backward-Left
            vx = -speed_magnitude * 0.707;
            vy = -speed_magnitude * 0.707;
            break;
        case 10: // Backward-Right
            vx = -speed_magnitude * 0.707;
            vy = speed_magnitude * 0.707;
            break;
        default: // ถ้าคำสั่งผิดพลาด ให้หยุด
            vx = 0; vy = 0; omega = 0;
            break;
    }
    controlMotors(vx, vy, omega);

    // Debug
    if (debug_flag) {
        snprintf(debug_buffer, sizeof(debug_buffer),
                 "Manual CMD: Direc=%d PWM=%d -> vx=%.0f vy=%.0f w=%.0f",
                 direc, pwm, vx, vy, omega);
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
        float vx = map_float(cmd_vel_msg.linear.x, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED, -255.0, 255.0);
        float vy = map_float(cmd_vel_msg.linear.y, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED, -255.0, 255.0);
        float omega = map_float(cmd_vel_msg.angular.z, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED, -255.0, 255.0);
        
        controlMotors(vx, vy, omega);

        motor_running = true;
        last_cmd_time = millis();
    }
}

float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
    snprintf(debug_buffer, sizeof(debug_buffer),
                 "vx: %.4f  vy: %.4f  omega: %.4f", vx, vy,omega);
        DebugPublish(debug_buffer);
    
    //  คำนวณความเร็วของล้อแต่ละตัว
    float wheel_FL = vx - vy - (L1 + L2) * omega;
    float wheel_FR = vx + vy + (L1 + L2) * omega;
    float wheel_RL = vx + vy - (L1 + L2) * omega; // แก้ไข
    float wheel_RR = vx - vy + (L1 + L2) * omega; // แก้ไข

    //  ปรับค่าสเกลให้ไม่เกิน -255 ถึง 255
    float maxVal = max(max(abs(wheel_FL), abs(wheel_FR)), 
                       max(abs(wheel_RL), abs(wheel_RR)));
    if (maxVal > 255) {
        wheel_FL *= 255.0 / maxVal;
        wheel_FR *= 255.0 / maxVal;
        wheel_RL *= 255.0 / maxVal;
        wheel_RR *= 255.0 / maxVal;
    }

    setMotorPWM(wheel_FL, FL_IN1, FL_IN2, FL_PWM); // M1: Front-Left
    setMotorPWM(wheel_FR, FR_IN1, FR_IN2, FR_PWM); // M3: Front-Right
    setMotorPWM(wheel_RL, RL_IN1, RL_IN2, RL_PWM); // M4: Rear-Left
    setMotorPWM(wheel_RR, RR_IN1, RR_IN2, RR_PWM); // M2: Rear-Right
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
 * - จำกัดค่าการเคลื่อนที่ของเซอร์โวไม่ให้ออกนอกช่วงที่กำหนด (`MIN_POSITION_SV` - `MAX_POSITION_SV`)
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
            current_position_1 = max(current_position_1 - STEP_SIZE_X, MIN_POSITION_SV);
            servo1.writeMicroseconds(current_position_1);
            break;

        case 0x2: // ขวา
            current_position_1 = min(current_position_1 + STEP_SIZE_X, MAX_POSITION_SV);
            servo1.writeMicroseconds(current_position_1);
            break;

        case 0x3: // ขึ้น
            current_position_2 = max(current_position_2 - STEP_SIZE_Y, MIN_POSITION_SV);
            servo2.writeMicroseconds(current_position_2);
            break;

        case 0x4: // ลง
            current_position_2 = min(current_position_2 + STEP_SIZE_Y, MAX_POSITION_SV);
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
        int pan_deg  = map(current_position_1, MIN_POSITION_SV, MAX_POSITION_SV, 0, 180);
        int tilt_deg = map(current_position_2, MIN_POSITION_SV, MAX_POSITION_SV, 0, 180);
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
        moveRobot(direction, pwm);

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

void publishFusedIMUData() {
    //อ่าน Gyro & Accelerometer จาก MPU6050
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    const float accelScale = 16384.0; // MPU6050 in ±2g mode
    const float gyroScale = 131.0;    // MPU6050 in ±250°/s mode
    const float PI_OVER_180 = 0.01745329251; // PI / 180.0

    //ใส่ข้อมูลความเร่งและความเร็วเชิงมุมลงใน message
    imu_msg.linear_acceleration.x = (ax / accelScale) * 9.81;
    imu_msg.linear_acceleration.y = (ay / accelScale) * 9.81;
    imu_msg.linear_acceleration.z = (az / accelScale) * 9.81;

    imu_msg.angular_velocity.x = (gx / gyroScale) * PI_OVER_180;
    imu_msg.angular_velocity.y = (gy / gyroScale) * PI_OVER_180;
    imu_msg.angular_velocity.z = (gz / gyroScale) * PI_OVER_180;

    //อ่าน Yaw (Heading) จาก Compass (QMC5883L)
    Wire.beginTransmission(QMC5883L_ADDRESS);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) { return; } // Exit if compass not found

    Wire.requestFrom((uint8_t)QMC5883L_ADDRESS, (uint8_t)6);
    if (Wire.available() < 6) { return; } // Exit on read timeout

    int16_t x = Wire.read() | Wire.read() << 8;
    int16_t y = Wire.read() | Wire.read() << 8;
    Wire.read(); Wire.read(); // skip Z

    float heading = atan2(static_cast<float>(y) - OFFSET_Y, static_cast<float>(x) - OFFSET_X);
    if (heading < 0) {
        heading += TWO_PI;
    }

    // แปลง Heading (Yaw) เป็น Quaternion
    float qx, qy, qz, qw;
    yawToQuaternion(heading, qx, qy, qz, qw);

    // บรรจุข้อมูล Orientation จาก Compass ลงใน message
    imu_msg.orientation.x = qx;
    imu_msg.orientation.y = qy;
    imu_msg.orientation.z = qz;
    imu_msg.orientation.w = qw;

    //ตั้งค่า Covariance เพื่อบอก EKF ว่าจะเชื่อถือข้อมูลส่วนไหน
    // เราเชื่อถือ Yaw จาก Compass แต่ไม่เชื่อ Roll/Pitch (ตั้งค่าความไม่แน่นอนสูงๆ)
    imu_msg.orientation_covariance[0] = 1e-2;  // Roll variance
    imu_msg.orientation_covariance[4] = 1e-2;  // Pitch variance
    imu_msg.orientation_covariance[8] = 0.05;    // Yaw variance (ความไม่แน่นอนต่ำ = เชื่อถือ)

    //เชื่อถือ Gyroscope และ Accelerometer
    imu_msg.angular_velocity_covariance[0] = 0.02; // gx
    imu_msg.angular_velocity_covariance[4] = 0.02; // gy
    imu_msg.angular_velocity_covariance[8] = 0.02; // gz

    imu_msg.linear_acceleration_covariance[0] = 0.05; // ax
    imu_msg.linear_acceleration_covariance[4] = 0.05; // ay
    imu_msg.linear_acceleration_covariance[8] = 0.05; // az

    // ตั้งค่า Header และ Publish
    imu_msg.header.stamp = nh.now();
    imu_msg.header.frame_id = "imu_link";
    imu_pub.publish(&imu_msg);
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
*/

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


/*
void readCompass() {   
    Wire.beginTransmission(QMC5883L_ADDRESS);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) {
        Serial2.println("[ERROR] Compass not responding!");
        return;
    }

    Wire.requestFrom((uint8_t)QMC5883L_ADDRESS, (uint8_t)6);
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

    // ----- ส่วนที่ต้องเพิ่มเข้ามา -----
    // บอก EKF ว่าเรามีข้อมูลแค่ orientation (yaw) เท่านั้น
    mag_msg.orientation_covariance[0] = 99999; // Roll - ไม่ได้ใช้
    mag_msg.orientation_covariance[4] = 99999; // Pitch - ไม่ได้ใช้
    mag_msg.orientation_covariance[8] = 0.05;  // Yaw - ความไม่แน่นอนต่ำ (ค่าบวก)

    // บอก EKF ให้ "ไม่ต้องสนใจ" ข้อมูลส่วนที่เหลือโดยสิ้นเชิง
    mag_msg.angular_velocity_covariance[0] = -1;
    mag_msg.linear_acceleration_covariance[0] = -1;
    // --------------------------------

    mag_pub.publish(&mag_msg);
}
*/
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
 * - จำกัดค่าของ `pwm_value` ให้อยู่ในช่วงที่เซอร์โวรับได้ (`MIN_POSITION_SV` ถึง `MAX_POSITION`)
 * - ตรวจสอบว่าค่าพินที่รับเข้ามาตรงกับเซอร์โวตัวที่ 1 (`SERVO1_PIN`) หรือ เซอร์โวตัวที่ 2 (`SERVO2_PIN`)
 * - ใช้ไลบรารี `ServoTimer2` ในการสั่งให้เซอร์โวเคลื่อนที่ไปยังตำแหน่งที่กำหนด
 */
void moveServo(int pin, int pwm_value) {
    // จำกัดค่าการเคลื่อนที่ของเซอร์โวให้อยู่ในช่วงที่กำหนด
    pwm_value = constrain(pwm_value, MIN_POSITION_SV, MAX_POSITION_SV); 

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

void calculateAndPublishOdometry() {
    static unsigned long last_time = millis();
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0;
    if (dt <= 0) return;

    noInterrupts();
    long currentFL = counter_FL; 
    long currentFR = counter_FR;
    long currentRL = counter_RL; 
    long currentRR = counter_RR;
    interrupts();

    double deltaFL = currentFL - prevFL; 
    double deltaFR = currentFR - prevFR;
    double deltaRL = currentRL - prevRL; 
    double deltaRR = currentRR - prevRR;

    prevFL = currentFL; prevFR = currentFR;
    prevRL = currentRL; prevRR = currentRR;

    double w_fl = (deltaFL / PPR) * (TWO_PI) / dt; 
    double w_fr = (deltaFR / PPR) * (TWO_PI) / dt;
    double w_rl = (deltaRL / PPR) * (TWO_PI) / dt; 
    double w_rr = (deltaRR / PPR) * (TWO_PI) / dt;

    w_fr = -w_fr;
    w_rr = -w_rr;

    // --- คำนวณความเร็ว (Twist) - ใช้สูตร Forward Kinematics ที่ถูกต้อง ---
    double vx = (WHEEL_RADIUS / 4.0) * (w_fl + w_fr + w_rl + w_rr);
    double vy = (WHEEL_RADIUS / 4.0) * (-w_fl + w_fr - w_rl + w_rr);
    double vth = (WHEEL_RADIUS / (4.0 * (L1 + L2))) * (-w_fl + w_fr - w_rl + w_rr);

    // --- 4. คำนวณและอัปเดตตำแหน่ง (Pose) ---
    // คำนวณระยะทางที่เปลี่ยนไปในแต่ละแกน (ใน frame ของหุ่นยนต์)
    double delta_x = (vx * cos(theta) - vy * sin(theta)) * dt;
    double delta_y = (vx * sin(theta) + vy * cos(theta)) * dt;
    double delta_theta = vth * dt;

    // นำระยะทางที่เปลี่ยนไปมาบวกสะสมในตัวแปร global
    x_pos += delta_x;
    y_pos += delta_y;
    theta += delta_theta;
    
    // ทำให้มุม theta อยู่ในช่วง 0 ถึง 2*PI
    if (theta >= TWO_PI) theta -= TWO_PI;
    if (theta < 0) theta += TWO_PI;


    // --- 5. บรรจุข้อมูลทั้งหมดลงใน Odometry Message ---

    // แปลงมุม theta (Yaw) เป็น Quaternion สำหรับ message
    odom_msg.pose.pose.orientation.w = cos(theta / 2.0);
    odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;

    // ตั้งค่า Header
    odom_msg.header.stamp = nh.now();
    odom_msg.header.frame_id = odom_frame;
    odom_msg.child_frame_id = base_link_frame;

    // บรรจุข้อมูลตำแหน่ง (Pose)
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.position.y = y_pos;
    odom_msg.pose.pose.position.z = 0.0; // สำหรับหุ่นยนต์ 2D

    // บรรจุข้อมูลความเร็ว (Twist)
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.angular.z = vth;

    odom_msg.twist.covariance[0] = 0.1;  // vx
    odom_msg.twist.covariance[7] = 0.1;  // vy
    odom_msg.twist.covariance[35] = 0.05; // vth

    odom_msg.pose.covariance[0] = 0.1;   // x
    odom_msg.pose.covariance[7] = 0.1;   // y
    odom_msg.pose.covariance[35] = 0.05;  // theta

    // --- 6. Publish Odometry Message ---
    odom_pub.publish(&odom_msg);


    // --- 7. สร้างและส่ง TF Transform (odom -> base_link) ---
    odom_trans.header.stamp = nh.now();
    odom_trans.header.frame_id = odom_frame;
    odom_trans.child_frame_id = base_link_frame;

    odom_trans.transform.translation.x = x_pos;
    odom_trans.transform.translation.y = y_pos;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_msg.pose.pose.orientation; // ใช้ Quaternion เดียวกัน

    broadcaster.sendTransform(odom_trans);

    // อัปเดตเวลาสำหรับการคำนวณรอบถัดไป
    last_time = current_time;
}

void isr_FL() { // สำหรับ Encoder Front-Left
    if (micros() - last_interrupt_time_FL < ENCODER_DEBOUNCE_DELAY_US) return;
    byte a = (PINE >> PINE4) & 1; // อ่านค่าขา A (Pin 2)
    byte b = (PING >> PING5) & 1; // อ่านค่าขา B (Pin 4)
    if (a != prevA_FL) { // ตรวจสอบว่าขา A มีการเปลี่ยนแปลงจริง
        // ใช้ XOR (eXclusive OR) logic ในการเช็คทิศทาง
        // a ^ b จะเป็นจริง (1) ถ้า a และ b มีค่าต่างกัน
        // และจะเป็นเท็จ (0) ถ้า a และ b มีค่าเหมือนกัน
        if (a ^ b) counter_FL++; // ถ้าต่างกัน หมุนทิศหนึ่ง
        else       counter_FL--; // ถ้าเหมือนกัน หมุนอีกทิศหนึ่ง
    }
    prevA_FL = a; // อัปเดตสถานะล่าสุดของขา A เพื่อใช้ในการเปรียบเทียบครั้งต่อไป
    last_interrupt_time_FL = micros();
}

void isr_FR() { // สำหรับ Encoder Front-Right
    if (micros() - last_interrupt_time_FR < ENCODER_DEBOUNCE_DELAY_US) return;
    byte a = (PIND >> PIND3) & 1; // อ่านค่าขา A (Pin 18)
    byte b = (PINA >> PINA0) & 1; // อ่านค่าขา B (Pin 22)
    if (a != prevA_FR) {
        if (a ^ b) counter_FR++;
        else       counter_FR--;
    }
    prevA_FR = a;
    last_interrupt_time_FR = micros();
}

void isr_RL() { // สำหรับ Encoder Rear-Lef
    if (micros() - last_interrupt_time_RL < ENCODER_DEBOUNCE_DELAY_US) return;
    byte a = (PIND >> PIND2) & 1; // อ่านค่าขา A (Pin 19)
    byte b = (PINA >> PINA1) & 1; // อ่านค่าขา B (Pin 23)
    if (a != prevA_RL) {
        if (a ^ b) counter_RL++;
        else       counter_RL--;
    }
    prevA_RL = a;
    last_interrupt_time_RL = micros();

}

void isr_RR() { // สำหรับ Encoder Rear-Right
    if (micros() - last_interrupt_time_RR < ENCODER_DEBOUNCE_DELAY_US) return;
    byte a = (PINE >> PINE5) & 1; // อ่านค่าขา A (Pin 3)
    byte b = (PINE >> PINE3) & 1; // อ่านค่าขา B (Pin 5)
    if (a != prevA_RR) {
        if (a ^ b) counter_RR++;
        else       counter_RR--;
    }
    prevA_RR = a;
    last_interrupt_time_RR = micros();
}
//Encoder Reset Function
void checkAndResetEncoder(volatile long &encoderCount) {
    if (encoderCount <= 0 || encoderCount >= MAX_POSITION_ENC) {
        noInterrupts();
        encoderCount = ENCODER_MIDPOINT;
        snprintf(debug_buffer, sizeof(debug_buffer),
                 "Encoders: RESET");
        DebugPublish(debug_buffer);
        interrupts();
    }
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