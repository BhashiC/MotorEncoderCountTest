#include <Arduino.h>

volatile long current_position_motor_left = 0;
volatile long current_position_motor_right = 0;

#define pwm_motor_left 6 // pwm motor left connect to AN1 at MDS10
#define dir_motor_left 4 // // direction motor left connect to DIG1 at MDS10
// Actually cnannel B is 19, but to ensure possitive encoder count when robot moving forward we changed
#define encoderA_motor_left 19 // encoder pinA motor left
// Actually cnannel A is 18, but to ensure possitive encoder count when robot moving forward we changed
#define encoderB_motor_left 18 // encoder pinB motor left

#define pwm_motor_right 5       // pwm motor right connect to AN2 at MDS10
#define dir_motor_right 7       // direction motor right connect to DIG2 at MDS10
#define encoderA_motor_right 20 // encoder pinA motor right
#define encoderB_motor_right 21 // encoder pinB motor right

/// @brief Left motor forward direction
void motroLeftForward()
{
  digitalWrite(dir_motor_left, HIGH);
}

/// @brief Left motor backward direction
void motorLeftBackward()
{
  digitalWrite(dir_motor_left, LOW);
}

/// @brief Right motor forward direction
void motorRightForward()
{
  digitalWrite(dir_motor_right, HIGH);
}

/// @brief Right motor backward direction
void motorRightBackward()
{
  digitalWrite(dir_motor_right, LOW);
}

/// @brief Left motor Stop
void motorLeftStop()
{
  analogWrite(pwm_motor_left, 0);
}

/// @brief Right motor Stop
void motroRightStop()
{
  analogWrite(pwm_motor_right, 0);
}

/// @brief Left motor Set to PWM
/// @param pwm pwm value must be 0-255
void motroLeftSetPwm(int pwm)
{
  if (pwm < 0 || pwm > 255)
  {
    Serial.println("Wrong Input: PWM must be 0-255");
    return;
  }
  analogWrite(pwm_motor_left, pwm);
}

/// @brief Right motor Set pwm
/// @param pwm pwm value must be 0-255
void motroRightSetPwm(int pwm)
{
  if (pwm < 0 || pwm > 255)
  {
    Serial.println("Wrong Input: PWM must be 0-255");
    return;
  }
  analogWrite(pwm_motor_right, pwm);
}

/// @brief Left motor encoder channel A
void countEnAMotorLeft()
{
  if (digitalRead(encoderA_motor_left) != digitalRead(encoderB_motor_left))
  {
    current_position_motor_left++;
  }
  else
  {
    current_position_motor_left--;
  }
}

/// @brief Left motor encoder channel B
void countEnBMotorLeft()
{
  if (digitalRead(encoderA_motor_left) == digitalRead(encoderB_motor_left))
  {
    current_position_motor_left++;
  }
  else
  {
    current_position_motor_left--;
  }
}

/// @brief Right motor encoder channel A
void countEnAMotorRight()
{
  if (digitalRead(encoderA_motor_right) != digitalRead(encoderB_motor_right))
  {
    current_position_motor_right++;
  }
  else
  {
    current_position_motor_right--;
  }
}

/// @brief Right motor encoder channel B
void countEnBMotorRight()
{
  if (digitalRead(encoderA_motor_right) == digitalRead(encoderB_motor_right))
  {
    current_position_motor_right++;
  }
  else
  {
    current_position_motor_right--;
  }
}

/// @brief Test motor functions
void testMotors()
{
  if (Serial.available() > 0)
  {
    char ch = Serial.read();
    switch (ch)
    {
    case 'F':
      motroLeftForward();
      motorRightForward();
      Serial.println("Motors Forward!");
      break;

    case 'X':
      current_position_motor_left = 0;
      current_position_motor_right = 0;
      break;

    case 'B':
      motorLeftBackward();
      motorRightBackward();
      Serial.println("Motors Backward!");
      break;

    case 'S':
      int pwm = Serial.parseInt();
      if (pwm == 0)
      {
        motorLeftStop();
        motroRightStop();
        Serial.println("Motors Stop!");
        break;
      }

      if (pwm < 0 || pwm > 255)
      {
        Serial.println("Wrong Input: PWM must be 0-255");
        break;
      }
      motroLeftSetPwm(pwm);
      motroRightSetPwm(pwm);
      Serial.print("Motors PWM = ");
      Serial.println(pwm);
      break;
    }
  }
}

/// @brief print left and right motors' encoder counts
void printEncoderCounts()
{
  Serial.print("current_position_motor_left = ");
  Serial.print(current_position_motor_left);
  Serial.print("  current_position_motor_right = ");
  Serial.println(current_position_motor_right);
}

/// @brief Test blink led
void testBlink()
{
  digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
  delay(1000);                     // wait for a second
  digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
  delay(1000);
}

void setup()
{
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(pwm_motor_right, OUTPUT);
  pinMode(pwm_motor_left, OUTPUT);
  pinMode(dir_motor_right, OUTPUT);
  pinMode(dir_motor_left, OUTPUT);

  pinMode(encoderA_motor_left, INPUT_PULLUP);
  pinMode(encoderB_motor_left, INPUT_PULLUP);
  pinMode(encoderA_motor_right, INPUT_PULLUP);
  pinMode(encoderB_motor_right, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderA_motor_left), countEnAMotorLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB_motor_left), countEnBMotorLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderA_motor_right), countEnAMotorRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB_motor_right), countEnBMotorRight, CHANGE);

  Serial.println("Setup Done!");
}

void loop()
{
  testMotors();
  printEncoderCounts();
}
