#include <PID_v1.h>

#define L298N_enA 9
#define L298N_in1 12
#define L298N_in2 13
#define L298N_in3 7
#define L298N_in4 8
#define L298N_enB 11

#define right_encoder_phaseA 3
#define right_encoder_phaseB 5
#define left_encoder_phaseA 2
#define left_encoder_phaseB 4

volatile unsigned int right_encoder_counter = 0;
volatile unsigned int left_encoder_counter = 0;
String right_encoder_sign = "p";
String left_encoder_sign = "p";
double right_wheel_meas_vel = 0.0;  // rad/s
double left_wheel_meas_vel = 0.0;
bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;
double right_wheel_cmd_vel = 0.0;
double left_wheel_cmd_vel = 0.0;

unsigned long last_millis = 0;
const unsigned long interval = 100;

// Encoder data with exp
const double COUNTS_PER_WHEEL_REV = 374.1;

// For safety
unsigned long last_cmd_millis = 0;
const unsigned long cmd_timeout = 500;  // 500 ms
bool received_cmd = false;

double right_wheel_cmd = 0;
double left_wheel_cmd = 0;

// trial and error!!
double Kp_r = 11.5;
double Ki_r = 7.5;
double Kd_r = 0.1;
double Kp_l = 12.8;
double Ki_l = 8.3;
double Kd_l = 0.1;

PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_cmd_vel, Kp_l, Ki_l, Kd_l, DIRECT);
// PID(Input, Output(PWD), Setpoint, Kp, Ki, Kd, direction)

void setup()
{
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);
  pinMode(right_encoder_phaseA, INPUT);
  pinMode(right_encoder_phaseB, INPUT);
  pinMode(left_encoder_phaseA, INPUT);
  pinMode(left_encoder_phaseB, INPUT);

  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA),rightEncoderCallback,RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA),leftEncoderCallback,RISING);

  digitalWrite(L298N_in1, LOW);
  digitalWrite(L298N_in2, HIGH);
  digitalWrite(L298N_in3, LOW);
  digitalWrite(L298N_in4, HIGH);
  

  rightMotor.SetMode(AUTOMATIC); //PID 自己根據 Input / Setpoint 計算 Output
  leftMotor.SetMode(AUTOMATIC);

  Serial.begin(115200);
}

void loop()
{
  if(Serial.available())
  {
    char chr = Serial.read();
    if(chr == 'r')
    {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0;
      is_cmd_complete = false;
    }
    else if(chr == 'l')
    {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
    }
    else if (chr == 'p')
    {
      if(is_right_wheel_cmd && !is_right_wheel_forward)
      {
          digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
          digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
          is_right_wheel_forward = true;
      }
      else if(is_left_wheel_cmd && !is_left_wheel_forward)
      {
          digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
          digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
          is_left_wheel_forward = true;
      }
    }
    else if (chr == 'n')
    {
      if(is_right_wheel_cmd && is_right_wheel_forward)
      {
          digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
          digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
          is_right_wheel_forward = false;
      }
      else if(is_left_wheel_cmd && is_left_wheel_forward)
      {
          digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
          digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
          is_left_wheel_forward = false;
      }
    }
    else if (chr == ',')
    {
      if(is_right_wheel_cmd)
      {
        right_wheel_cmd_vel = atof(value);
      }
      else if(is_left_wheel_cmd)
      {
        left_wheel_cmd_vel = atof(value);
        is_cmd_complete = true;

        // A complete right + left command has been received
        last_cmd_millis = millis();
        received_cmd = true;
      }
      value_idx = 0;
      value[0] = '0';
      value[1] = '0';
      value[2] = '.';
      value[3] = '0';
      value[4] = '0';
      value[5] = '\0';
    }
    else
    {
      if(value_idx < 5)
      {
        value[value_idx] = chr;
        value_idx++;
      }
    }
  }
  
  // Communication watchdog
  if(received_cmd && millis() - last_cmd_millis > cmd_timeout)
  {
    right_wheel_cmd_vel = 0.0;
    left_wheel_cmd_vel = 0.0;

    right_wheel_cmd = 0.0;
    left_wheel_cmd = 0.0;

    analogWrite(L298N_enA, 0);
    analogWrite(L298N_enB, 0);

    received_cmd = false;
  }
  
  unsigned long current_millis = millis();

  if(current_millis - last_millis >= interval)
  {
    unsigned int right_count;
    unsigned int left_count;

    noInterrupts();

    right_count = right_encoder_counter;
    left_count = left_encoder_counter;

    right_encoder_counter = 0;
    left_encoder_counter = 0;

    interrupts();

    double dt = (current_millis - last_millis) / 1000.0;

    right_wheel_meas_vel =
        right_count * (2.0 * PI / COUNTS_PER_WHEEL_REV) / dt;

    left_wheel_meas_vel =
        left_count * (2.0 * PI / COUNTS_PER_WHEEL_REV) / dt;

    rightMotor.Compute();
    leftMotor.Compute();

    if(right_wheel_cmd_vel == 0.0)
    {
      right_wheel_cmd = 0.0;
    }

    if(left_wheel_cmd_vel == 0.0)
    {
      left_wheel_cmd = 0.0;
    }

    String encoder_read =
        "r" + right_encoder_sign + String(right_wheel_meas_vel) +
        ",l" + left_encoder_sign + String(left_wheel_meas_vel) + ",";

    Serial.println(encoder_read);

    last_millis = current_millis;

    analogWrite(L298N_enA, right_wheel_cmd);
    analogWrite(L298N_enB, left_wheel_cmd);
  }
}

void rightEncoderCallback()
{
  right_encoder_counter++;

  if (digitalRead(right_encoder_phaseB) == HIGH)
  {
    right_encoder_sign = "n";
  }
  else
  {
    right_encoder_sign = "p";
  }
}

void leftEncoderCallback() 
{
  left_encoder_counter++;

  if (digitalRead(left_encoder_phaseB) == HIGH)
  {
    left_encoder_sign = "p";  //opposite rotational direction 
  }
  else
  {
    left_encoder_sign = "n";
  }
}