#include <ros.h>
#include <Servo.h>
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
        if ( phasestate2 == HIGH) {
          dir = -1;
        } else {
          dir = 1;
        }
      } else {
        if ( phasestate2 == HIGH) {
          dir = 1;
        } else {
          dir = -1;
        }
      }
      encoder_time_old = encoder_time;
      encoder_time = micros();
      updateFlagsShared |= ENCODER_FLAG;
      encoder_count -= dir;
    }

    void Encoder::phase2interfunc() {
      encoder_read();
      if (phasestate1 + phasestate2 + phasestate3 == 2) {
        if ( phasestate1 == HIGH) {
          dir = 1;
        } else {
          dir = -1;
        }
      } else {
        if ( phasestate1 == HIGH) {
          dir = -1;
        } else {
          dir = 1;
        }
      }
      encoder_time_old = encoder_time;
      encoder_time = micros();
      updateFlagsShared |= ENCODER_FLAG;
      encoder_count -= dir;
    }

    void Encoder::phase3interfunc() {
      encoder_read();
      if (phasestate1 + phasestate2 + phasestate3 == 2) {
        if ( phasestate1 == HIGH) {
          dir = -1;
        } else {
          dir = 1;
        }
      } else {
        if ( phasestate1 == HIGH) {
          dir = 1;
        } else {
          dir = -1;
        }
      }
      encoder_time_old = encoder_time;
      encoder_time = micros();
      updateFlagsShared |= ENCODER_FLAG;
      encoder_count -= dir;
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
    void steer_pwm_out(float pwm_steer);

    void init_steer() {
      steering.attach(servo_in);
    }

    void neutral_steer() {
      steering.writeMicroseconds( (uint16_t) servo_neutral_pwm );
    }

  private:
    int steer_curr = 0;

    const int steer_min =  1100;
    const int steer_max = 2000;

    const int servo_in = 10;

    float servo_neutral_pwm = 1500.0;

    Servo steering;
};


// Brushless ESC Control //
class ESC {
  public:

    void esc_cb(float pwm_esc);
    void esc_pwm_out(float pwm_esc);

    void init_ESC() {
      ESCmotor.attach(ESC_in);
    }

    void neutral_ESC() {
      ESCmotor.writeMicroseconds( (uint16_t) ESC_neutral_pwm );
    }
    // ESC INITIALIZATION
    void startup_ESC(int runcount) {
      if ( runcount == 1 ) {
        ESCmotor.writeMicroseconds( sweephigh );
        delayMicroseconds(50);
      }
      ESCmotor.writeMicroseconds( ESC_neutral_pwm );
    }

  private:
    int ESC_curr = 0;

    const int ESC_min = 1100;
    const int ESC_max = 1800;

    const int ESC_in = 9;

    const int sweephigh = 2000;

    float ESC_neutral_pwm = 1500.0;

    Servo ESCmotor;
};

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

ros::Subscriber<std_msgs::Float32> sub_esc("esc_pwm", &esc_cb );
ros::Subscriber<std_msgs::Float32> sub_steer("steer_pwm", &steer_cb );

void setup() {

  motorencoder.encoder_setup();

  nh.getHardware()->setBaud(9600);

  steeringservo.init_steer();
  brushlessmotor.init_ESC();

  nh.initNode();
  nh.advertise(pub_encoder_count);
  nh.advertise(pub_velocity);
  nh.subscribe(sub_esc);
  nh.subscribe(sub_steer);

  steeringservo.neutral_steer();
  brushlessmotor.neutral_ESC();

  delay(1500);

  pwm_t0 = millis();
}

void loop() {
  dt = millis() - t0;

  nh.spinOnce();

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
  nh.spinOnce();

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

    updateFlagsShared = 0;
    interrupts();
  }
}

void Steer::steer_pwm_out(float pwm_steer) {
  if ( pwm_steer > steer_max ) {
    pwm_steer = steer_max;
  } else if ( pwm_steer < steer_min) {
    pwm_steer = steer_min;
  }
  steering.writeMicroseconds( pwm_steer );
}

void ESC::esc_pwm_out(float pwm_esc) {
  if ( pwm_esc > ESC_max ) {
    pwm_esc = ESC_max;
  } else if ( pwm_esc < ESC_min) {
    pwm_esc = ESC_min;
  }
  ESCmotor.writeMicroseconds( pwm_esc );
}
