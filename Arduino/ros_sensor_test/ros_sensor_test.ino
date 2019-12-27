#include <ros.h>
#include <std_msgs/Float32.h>
#include <EnableInterrupt.h>

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
      phasestate1 = digitalRead(2);
      phasestate2 = digitalRead(3);
      phasestate3 = digitalRead(4);
    }

    void Encoder::phase1interfunc() {
      encoder_read();
      if (phasestate1 + phasestate2 + phasestate3 == 2) {
        if ( phasestate2 == HIGH) { dir = -1; } else { dir = 1; } } else {
        if ( phasestate2 == HIGH) { dir = 1; } else { dir = -1;} }
      encoder_time_old = encoder_time;
      encoder_time = micros();
      updateFlagsShared |= ENCODER_FLAG;
      encoder_count += dir;
    }

    void Encoder::phase2interfunc() {
      encoder_read();
      if (phasestate1 + phasestate2 + phasestate3 == 2) {
        if ( phasestate1 == HIGH) { dir = 1; } else { dir = -1; } } else {
        if ( phasestate1 == HIGH) { dir = -1; } else { dir = 1; } }
      encoder_time_old = encoder_time;
      encoder_time = micros();
      updateFlagsShared |= ENCODER_FLAG;
      encoder_count += dir;
    }

    void Encoder::phase3interfunc() {
      encoder_read();
      if (phasestate1 + phasestate2 + phasestate3 == 2) {
        if ( phasestate1 == HIGH) { dir = -1; } else { dir = 1; } } else {
        if ( phasestate1 == HIGH) { dir = 1; } else { dir = -1; } }
      encoder_time_old = encoder_time;
      encoder_time = micros();
      updateFlagsShared |= ENCODER_FLAG;
      encoder_count += dir;
    }

    void calc_velocity() {
      if (encoder_count_old != encoder_count) {
        velocity = dir * 0.25 * pi * r / (encoder_time - encoder_time_old / 1000000.0);
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

    const int ENCODER_FLAG = 1;

    //    unsigned long encoder_dt = 0;
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

ros::NodeHandle nh;

std_msgs::Float32 enc_cnt;
std_msgs::Float32 vel;

ros::Publisher pub_encoder_count("encoder", &enc_cnt);
ros::Publisher pub_velocity("velocity", &vel);

void setup() {
  
  motorencoder.encoder_setup();
  
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.advertise(pub_encoder_count);
  nh.advertise(pub_velocity);

}

void loop() {

  motorencoder.check_flags();
  motorencoder.calc_velocity();

  vel.data = velocity;
  pub_velocity.publish(&vel);

  enc_cnt.data = encoder_count;
  pub_encoder_count.publish(&enc_cnt);

  nh.spinOnce();

}

void Encoder::encoder_setup() {
  pinMode(2, INPUT_PULLUP); // attach pin for phase 1 to pin 2
  pinMode(3, INPUT_PULLUP); // attach pin for phase 2 to pin 3
  pinMode(4, INPUT_PULLUP); // attach pin for phase 3 to pin 4
  enableInterrupt(2, phase1interfuncCB, CHANGE);
  enableInterrupt(3, phase2interfuncCB, CHANGE);
  enableInterrupt(4, phase3interfuncCB, CHANGE);
}

void Encoder::check_flags() {
  if (updateFlagsShared) {
    noInterrupts();

    updateFlags = updateFlagsShared;

    updateFlagsShared = 0;
    interrupts();
  }
}
