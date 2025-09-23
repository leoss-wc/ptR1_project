#include <PinChangeInterrupt.h>  // ใช้สำหรับสร้าง Interrupt บนขาดิจิทัลที่ไม่ใช่ขา Interrupt เพื่อใช้กับ encoder 4 ตัว
#include <ros.h>
#include <std_msgs/UInt32.h>
#include <stdlib.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

// Encoder Pin Definitions
// FL == M1
#define ENCODER1_PIN_A 11  // PORTB3   (D10)
#define ENCODER1_PIN_B 10  // PORTB4   (D11)

// FR == M2
#define ENCODER2_PIN_A 5   // PORTD5   (D5)
#define ENCODER2_PIN_B 6   // PORTD6   (D6)

// RL == M3
#define ENCODER3_PIN_A 3   // PORTD3   (D3)
#define ENCODER3_PIN_B 4   // PORTD4   (D4)

// RR == M4
#define ENCODER4_PIN_A 7   // PORTD7   (D7)
#define ENCODER4_PIN_B 12  // PORTB2   (D12)




#define LOOP_INTERVAL 20  // 20 ms = 50 Hz

extern unsigned int __heap_start;
extern void *__brkval;

/*
   M1(FL)-----M2(FR)
     |          |
     |          |
     |          | 
     |          |     
   M3(RL)-----M4(RR)
*/


// Threshold and Limits
#define MAX_POSITION 20000000
#define ENCODER_MIDPOINT 10000000
#define BAUD_RATE 57600


// Encoder Variables
  volatile long counterM1 = ENCODER_MIDPOINT, 
                counterM2 = ENCODER_MIDPOINT, 
                counterM3 = ENCODER_MIDPOINT, 
                counterM4 = ENCODER_MIDPOINT;

  long prevM1 = 0, prevM2 = 0, prevM3 = 0, prevM4 = 0;
//   const float PPR_M1  = 660;
//   const float PPR_M2  = 660;
//   const float PPR_M3  = 660;
//   const float PPR_M4  = 660;
  const float PPR  = 660;

  volatile byte prevA1 = 0;
  volatile byte prevA2 = 0;
  volatile byte prevA3 = 0;
  volatile byte prevA4 = 0;

ros::NodeHandle nh;

std_msgs::Int32  encoder_msg1, encoder_msg2, encoder_msg3, encoder_msg4 ,freeMemory_msgs;
ros::Publisher encoder1_pub("encoder1", &encoder_msg1);
ros::Publisher encoder2_pub("encoder2", &encoder_msg2);
ros::Publisher encoder3_pub("encoder3", &encoder_msg3);
ros::Publisher encoder4_pub("encoder4", &encoder_msg4);
ros::Publisher freeMemory_pub("freeMemory_nano", &freeMemory_msgs);

std_msgs::Bool is_still_msg;
ros::Publisher is_still_pub("is_still", &is_still_msg);

//interrupt vaiable
volatile byte a1, b1, a2, b2, a3, b3, a4, b4;

unsigned long last_time = 0;

void setup() {
    Serial.begin(BAUD_RATE);
    delay(100); 
    nh.initNode();
    nh.advertise(encoder1_pub);
    nh.advertise(encoder2_pub);
    nh.advertise(encoder3_pub);
    nh.advertise(encoder4_pub);
    nh.advertise(freeMemory_pub);
    nh.advertise(is_still_pub);

    pinMode(ENCODER1_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER1_PIN_B, INPUT_PULLUP);
    pinMode(ENCODER2_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER2_PIN_B, INPUT_PULLUP);
    pinMode(ENCODER3_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER3_PIN_B, INPUT_PULLUP);
    pinMode(ENCODER4_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER4_PIN_B, INPUT_PULLUP);

    attachPCINT(digitalPinToPCINT(ENCODER1_PIN_A), counterM1_ISR, CHANGE);
    attachPCINT(digitalPinToPCINT(ENCODER1_PIN_B), counterM1_ISR, CHANGE);

    attachPCINT(digitalPinToPCINT(ENCODER2_PIN_A), counterM2_ISR, CHANGE);
    attachPCINT(digitalPinToPCINT(ENCODER2_PIN_B), counterM2_ISR, CHANGE);

    attachPCINT(digitalPinToPCINT(ENCODER3_PIN_A), counterM3_ISR, CHANGE);
    attachPCINT(digitalPinToPCINT(ENCODER3_PIN_B), counterM3_ISR, CHANGE);
    
    attachPCINT(digitalPinToPCINT(ENCODER4_PIN_A), counterM4_ISR, CHANGE);
    attachPCINT(digitalPinToPCINT(ENCODER4_PIN_B), counterM4_ISR, CHANGE);



}

void loop() {
    unsigned long current_time = millis();

    static unsigned long last_debug_time = 0;

    if (current_time - last_debug_time > 1000) {
        freeMemory_msgs.data = freeMemory();
        freeMemory_pub.publish(&freeMemory_msgs);
        last_debug_time = current_time;
    }

    if (current_time - last_time >= LOOP_INTERVAL) {  // ตรวจสอบช่วงเวลา 20 ms = 50 Hz
        // คำนวณความต่างของ encoder
        long deltaM1 = abs(counterM1 - prevM1);
        long deltaM2 = abs(counterM2 - prevM2);
        long deltaM3 = abs(counterM3 - prevM3);
        long deltaM4 = abs(counterM4 - prevM4);

        // ประเมินว่า "นิ่ง"
        bool isStill = deltaM1 < 2 && deltaM2 < 2 && deltaM3 < 2 && deltaM4 < 2;

        // publish is_still
        is_still_msg.data = isStill;
        is_still_pub.publish(&is_still_msg);


        publishEncoderValues();          
        prevM1 = counterM1;
        prevM2 = counterM2;
        prevM3 = counterM3;
        prevM4 = counterM4;

        last_time = current_time;  // อัปเดตเวลา

    }

    checkAndResetEncoder(counterM1);
    checkAndResetEncoder(counterM2);
    checkAndResetEncoder(counterM3);
    checkAndResetEncoder(counterM4);
    // อัปเดต ROS Node และรอรับคำสั่งใหม่
    static unsigned long lastSpinTime = 0;
    if (current_time - lastSpinTime >= 5) {  // 200Hz
      nh.spinOnce();
      lastSpinTime = current_time;
    }
}




/**
 * @brief ตรวจสอบค่าของ Encoder และรีเซ็ตเป็น 0 หากถึง MAX_POSITION
 * @param encoderCount (volatile long&) ตัวแปรที่ใช้เก็บค่าจำนวนพัลส์ของ Encoder
 * 
 * ฟังก์ชันนี้ช่วยป้องกันค่า Encoder ไม่ให้เกินขีดจำกัดที่กำหนด (`MAX_POSITION`)
 * โดยจะรีเซ็ตค่า `encoderCount` กลับเป็น 0 เมื่อถึงขีดจำกัด
 */
void checkAndResetEncoder(volatile long &encoderCount) {
    // ตรวจสอบว่าค่า Encoder เกินขีดจำกัดที่กำหนดหรือไม่
    if (encoderCount <= 0 || encoderCount >= MAX_POSITION) {
        // ปิดการขัดจังหวะชั่วคราว เพื่อป้องกันการเปลี่ยนแปลงค่า encoderCount โดย Interrupt
        noInterrupts();
        encoderCount = ENCODER_MIDPOINT; // รีเซ็ตค่า Encoder กลับเป็น 0
        interrupts(); // เปิดใช้งาน Interrupt อีกครั้ง
        // แสดงข้อความแจ้งเตือนผ่าน Serial Monitor (สำหรับ Debug)
    }
}

// M1 = Front Left = D11 (PB4), D12 (PB2)
    void counterM1_ISR() {
    a1 = (PINB >> PB3) & 1;
    b1 = (PINB >> PB4) & 1;


    if (a1 != prevA1) {
        if (a1 ^ b1) counterM1--;
        else         counterM1++;
    }
    prevA1 = a1;
    }

// M2 = Front Right = D5 (PD5), D6 (PD6)
    void counterM2_ISR() {
    a2 = (PIND >> PD5) & 1;
    b2 = (PIND >> PD6) & 1;

    if (a2 != prevA2) {
        if (a2 ^ b2) counterM2++;
        else         counterM2--;
    }
    prevA2 = a2;
    }

// M3 = Rear Left = D3 (PD3), D4 (PD4)
    void counterM3_ISR() {
    a3 = (PIND >> PD3) & 1;
    b3 = (PIND >> PD4) & 1;

    if (a3 != prevA3) {
        if (a3 ^ b3) counterM3++;
        else         counterM3--;
    }
    prevA3 = a3;
    }

// M4 = Rear Right = D7 (PD7), D10 (PB3)
    void counterM4_ISR() {
    a4 = (PIND >> PD7) & 1;
    b4 = (PINB >> PB2) & 1;

    if (a4 != prevA4) {
        if (a4 ^ b4) counterM4++;
        else         counterM4--;
    }
    prevA4 = a4;
    }

void publishEncoderValues() {
    encoder_msg1.data = counterM1;
    encoder_msg2.data = counterM2;
    encoder_msg3.data = counterM3;
    encoder_msg4.data = counterM4;

    encoder1_pub.publish(&encoder_msg1);
    encoder2_pub.publish(&encoder_msg2);
    encoder3_pub.publish(&encoder_msg3);
    encoder4_pub.publish(&encoder_msg4);
}

int freeMemory() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}




