#include <EnableInterrupt.h>

const float pi = 3.14159265359;
const float r = 0.0325;
float velocity = 0;
float encoder_count = 0;
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

void setup() {

  motorencoder.encoder_setup();
  delay(1500);
  Serial.begin(57600);

}

void loop() {

  motorencoder.check_flags();
  motorencoder.calc_velocity();
  
  Serial.print("vel: ");
  Serial.print(velocity,9);
  Serial.print(", enc_cnt: ");
  Serial.print(encoder_count);
  Serial.println();
   
  
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
