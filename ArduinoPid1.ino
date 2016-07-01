#include <Arduino.h>
#include <Servo.h>
#include <Encoder.h>

Encoder myEnc(2, 3);
Servo myservo;

// pin config
const int pin_pot_p = A0;
const int pin_pot_i = A1;
const int pin_pot_d = A2;
const int pin_servo = 11;

// set the control interval
const int control_interval = 10; // ms
// set how many digits serial prints the floats
const int print_precision = 6; // digits

// set the preset setpoints and time between step
const int step_interval = 1000; // ms
const double setpoint_1 = 0; // rad
const double setpoint_2 = M_PI/6.; // rad

// persistent variables for PID control
double setpoint = 0; // rad
unsigned long last_control_time = 0;
double last_error = 0;
double sum_error = 0;
// persistent variables for stepping
unsigned long last_step_time = 0;
boolean step_control = 0;


void setup() {
  pinMode(pin_pot_p, INPUT);
  pinMode(pin_pot_i, INPUT);
  pinMode(pin_pot_d, INPUT);
  pinMode(pin_servo, OUTPUT);

  Serial.begin(115200);
  myservo.attach(11, 1000, 2000);
  myEnc.write(0);

  // home
  myservo.write(75);
  boolean moving = 1;
  long last_encoder = myEnc.read();
  while (moving) {
    delay(300);
    long current_encoder = myEnc.read();
    moving = abs(current_encoder - last_encoder) > 10 ? 1 : 0;
    last_encoder = current_encoder;
  }
  myEnc.write(-1100);
}

void loop() {
  unsigned long now = millis();
  if (now - last_control_time >= control_interval) {
    // read the pot
    double kp = analogRead(pin_pot_p) * 0.1;
    double ki = analogRead(pin_pot_i) * 0.01;
    double kd = analogRead(pin_pot_d) * 0.01;

    // read the current position from encoder
    long current_position_encoder_tick = myEnc.read();
    // convert it to radians
    double current_position = current_position_encoder_tick * 2 * M_PI / 3600.; // rad

    // calculate error
    double error = current_position - setpoint;
    // calculate derivative of error
    double diff_error = error - last_error;
    // integrate error
    sum_error += error;

    // calculate how many volts should be applied to motor terminals
    double control_input = (kp*error + ki*sum_error + kd*diff_error)*-1;

    // convert motor terminal voltage to servo signal
    int servo_signal = map(control_input, -5., 5., 70, 90);
    // actually drive the motor
    myservo.write(servo_signal);

    last_error = error;
    last_control_time = now;

    Serial.print(current_position, print_precision);
    Serial.print(',');
    Serial.print(setpoint, print_precision);
    Serial.print(',');
    Serial.print(control_input, print_precision);
    Serial.print(';');
    Serial.print(kp, print_precision);
    Serial.print(',');
    Serial.print(ki, print_precision);
    Serial.print(',');
    Serial.print(kd, print_precision);
    Serial.print('\n');
  }

  if (now - last_step_time >= step_interval) {
    step_control ^= 1;
    setpoint = step_control ? setpoint_1 : setpoint_2;
    last_step_time = now;
  }
}
