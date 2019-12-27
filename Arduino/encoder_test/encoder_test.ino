#include <ros.h>
#include <Servo.h>
#include <EnableInterrupt.h>
#include <std_msgs/Float32.h>

// GLOBAL VARIABLES //
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
      dt = millis();
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
      dt = millis();
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
      dt = millis();
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


// Steering Servo //
class Steer {
  public:

    void steer_cb(float pwm_steer);

    void init_steer() {
      steering.attach(10); // attach Servo to Pin 10
    }

    void neutral_steer() {
      steering.writeMicroseconds( 1500.0 ); //center pwm value 1500
    }

    void steer_pwm_out(float pwm_steer) {
      if ( pwm_steer > 2000 ) {
        pwm_steer = 2000; // Servo max pwm value
      } else if ( pwm_steer < 1000) {
        pwm_steer = 1000; // Servo min pwm value
      }
      steering.writeMicroseconds( pwm_steer );
    }

  private:
    Servo steering;
};


// Brushless ESC Control //
class ESC {
  public:
  
    void esc_cb(float pwm_esc);
    
    void init_ESC() {
      ESCmotor.attach(9); // attach ESC to Pin 9
    }
    
    void neutral_ESC() {
      ESCmotor.writeMicroseconds( 1500.0 ); //center pwm value 1500
    }
    // ESC INITIALIZATION
    void startup_ESC(int runcount) {
      if ( runcount == 1 ) {
        ESCmotor.writeMicroseconds( 2000 ); //sweeping pwm from 2000 to 1500
        delayMicroseconds(50);
      }
      ESCmotor.writeMicroseconds( 1500.0 );
      delay(5);
    }

    void esc_pwm_out(float pwm_esc) {
      if ( pwm_esc > 1800 ) {
        pwm_esc = 1800; // ESC max pwm value
      } else if ( pwm_esc < 1100) {
        pwm_esc = 1100; // ESC min pwm value
      }
      ESCmotor.writeMicroseconds( pwm_esc );
    }

  private:
    Servo ESCmotor;
};

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

Steer steeringservo;
ESC brushlessmotor;

void steer_cb( const std_msgs::Float32& msg) {
  pwm_steer = msg.data;
  steeringservo.steer_pwm_out(pwm_steer);
  received_pwm_signal = 1;
}

void esc_cb( const std_msgs::Float32& msg) {
  pwm_esc = msg.data;
  brushlessmotor.esc_pwm_out(pwm_esc);
  received_pwm_signal = 1;
}

ros::NodeHandle nh;

std_msgs::Float32 enc_cnt;
std_msgs::Float32 vel;

ros::Publisher pub_encoder_count("encoder", &enc_cnt);
ros::Publisher pub_velocity("velocity", &vel);

//ros::Publisher pub_rc_inputs("rc_inputs", &rc_inputs);

//ros::Subscriber<std_msgs::Float32> sub_esc("esc_pwm", &esc_cb );
//ros::Subscriber<std_msgs::Float32> sub_steer("steer_pwm", &steer_cb );

void setup() {
  nh.getHardware()->setBaud(57600);

  steeringservo.init_steer();
  brushlessmotor.init_ESC();
  motorencoder.encoder_setup();

  nh.initNode();

  nh.advertise(pub_encoder_count);
  nh.advertise(pub_velocity);
  //  nh.subscribe(sub_esc);
  //  nh.subscribe(sub_steer);

  steeringservo.neutral_steer();
  brushlessmotor.neutral_ESC();
  delay(1500);

  pwm_t0 = millis();
}

void loop() {
  dt = millis() - t0;

  if ( (millis() - pwm_t0) >= 250) {
    if (!received_pwm_signal) {
      steeringservo.neutral_steer();
      brushlessmotor.neutral_ESC();
      first_signal = 1;
      loopcount = 1;
    } else {
      if (loopcount < 30) {
        brushlessmotor.startup_ESC(loopcount);
      }
      loopcount += 1;
      received_pwm_signal = 0;
    }
    pwm_t0 = millis();
  }

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

//    if (updateFlags & ENCODER_FLAG) {
//      encoder_dt = encoder_time - encoder_time_old;
//    }

    updateFlagsShared = 0;
    interrupts();
  }
}
