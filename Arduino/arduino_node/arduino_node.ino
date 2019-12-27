#include <ros.h>
#include <Servo.h>
#include <EnableInterrupt.h>
#include <std_msgs/Float32.h>

// GLOBAL VARIABLES //
const int numofpoles = 4;
const float pi = 3.14159265359;
const float r = 0.0325;
volatile unsigned long dt;
volatile unsigned long t0;
float velocity = 0;
float encoder_count = 0;
volatile unsigned long pwm_t0;
int received_pwm_signal = 0;
int pwm_esc = 1499;
int pwm_steer = 1499;
int first_signal = 1;
int loopcount = 1;

// Brushless Motor Encoder //
class Encoder {
  public:

    void encoder_setup();
    void check_flags();

    void encoder_read() {
      phasestate1 = digitalRead(phasepin1);
      phasestate2 = digitalRead(phasepin2);
      phasestate3 = digitalRead(phasepin3);
    }

    void Encoder::phase1interfunc() {
      encoder_read();
      if (phasestate1 + phasestate2 + phasestate3 == 2) {
        if ( phasestate2 == HIGH) {
          dir = -1;
          dt = millis();
        } else {
          dir = 1;
          dt = millis();
        }
      } else {
        if ( phasestate2 == HIGH) {
          dir = 1;
          dt = millis();
        } else {
          dir = -1;
          dt = millis();
        }
      }
      encoder_time_old = encoder_time;
      encoder_time = micros();
      updateFlagsShared |= ENCODER_FLAG;
      encoder_count += dir;
    }

    void Encoder::phase2interfunc() {
      encoder_read();
      if (phasestate1 + phasestate2 + phasestate3 == 2) {
        if ( phasestate1 == HIGH) {
          dir = 1;
          dt = millis();
        } else {
          dir = -1;
          dt = millis();
        }
      } else {
        if ( phasestate1 == HIGH) {
          dir = -1;
          dt = millis();
        } else {
          dir = 1;
          dt = millis();
        }
      }
      encoder_time_old = encoder_time;
      encoder_time = micros();
      updateFlagsShared |= ENCODER_FLAG;
      encoder_count += dir;
    }

    void Encoder::phase3interfunc() {
      encoder_read();
      if (phasestate1 + phasestate2 + phasestate3 == 2) {
        if ( phasestate1 == HIGH) {
          dir = -1;
          dt = millis();
        } else {
          dir = 1;
          dt = millis();
        }
      } else {
        if ( phasestate1 == HIGH) {
          dir = 1;
          dt = millis();
        } else {
          dir = -1;
          dt = millis();
        }
      }
      encoder_time_old = encoder_time;
      encoder_time = micros();
      updateFlagsShared |= ENCODER_FLAG;
      encoder_count += dir;
    }

    void calc_velocity() {
      if (encoder_count_old != encoder_count) {
        velocity = dir * 0.25 * pi * r / (encoder_dt / 1000000.0);
      }
      else {
        velocity = 0;
      }
      encoder_count_old = encoder_count;
    }

  private:
    int dir = 0;

    int encoder_count_old = 0;

    int phasestate1 = 0;
    int phasestate2 = 0;
    int phasestate3 = 0;

    int prevphasestate1 = 0;
    int prevphasestate2 = 0;
    int prevphasestate3 = 0;

    const int phasepin1 = 2;
    const int phasepin2 = 3;
    const int phasepin3 = 4;

    const int ENCODER_FLAG = 1;

    unsigned long encoder_dt = 0;
    volatile unsigned long encoder_time = 0;
    volatile unsigned long encoder_time_old = 0;
    volatile uint8_t updateFlagsShared;
    uint8_t updateFlags;
};

Encoder motorencoder;

void phase1interfuncCB() {
  motorencoder.phase1interfunc();
}

void phase2interfuncCB() {
  motorencoder.phase2interfunc();
}

void phase3interfuncCB() {
  motorencoder.phase3interfunc();
}

//class Reciever {
//  public:
//    void enable_reciever() {
//      pinMode(motorin, INPUT_PULLUP);
//      pinMode(servoin, INPUT_PULLUP);
//      enableInterrupt(motorin, test1, CHANGE);
//      enableInterrupt(servoin, test2, CHANGE);
//    }
//
//    void test1 () {
//
//    }
//
//    void test2 () {
//
//    }
//
//  private:
//    const int motorin = 5;
//    const int servoin = 6;
//};

ros::NodeHandle nh;

std_msgs::Float32 enc_cnt;
std_msgs::Float32 vel;

ros::Publisher pub_encoder_count("encoder", &enc_cnt);
ros::Publisher pub_velocity("velocity", &vel);

//ros::Publisher pub_rc_inputs("rc_inputs", &rc_inputs);

void setup() {
  nh.getHardware()->setBaud(9600);
  
  motorencoder.encoder_setup();

  nh.initNode();

  nh.advertise(pub_encoder_count);
  nh.advertise(pub_velocity);

  pwm_t0 = millis();
}

void loop() {

  motorencoder.check_flags();
  motorencoder.calc_velocity();

  vel.data = velocity;
  pub_velocity.publish(&vel);

  enc_cnt.data = encoder_count;
  pub_encoder_count.publish(&enc_cnt);

  nh.spinOnce();

  t0 = millis();
}

void Encoder::encoder_setup() {
  pinMode(phasepin1, INPUT_PULLUP);
  pinMode(phasepin2, INPUT_PULLUP);
  pinMode(phasepin3, INPUT_PULLUP);
  enableInterrupt(phasepin1, phase1interfuncCB, CHANGE);
  enableInterrupt(phasepin2, phase2interfuncCB, CHANGE);
  enableInterrupt(phasepin3, phase3interfuncCB, CHANGE);
}

void Encoder::check_flags() {
  if (updateFlagsShared) {
    noInterrupts();

    updateFlags = updateFlagsShared;

    if (updateFlags & ENCODER_FLAG) {
      encoder_dt = encoder_time - encoder_time_old;
    }

    updateFlagsShared = 0;
    interrupts();
  }
}
