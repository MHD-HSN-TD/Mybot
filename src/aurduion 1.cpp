#include <AccelStepper.h>

const int left_step_pin = 3;
const int left_dir_pin = 4;
const int left_encoder_pinA = 5;
const int left_encoder_pinB = 6;

const int right_step_pin = 7;
const int right_dir_pin = 8;
const int right_encoder_pinA = 9;
const int right_encoder_pinB = 10;

//const int battery_pin = A0;
//const double battery_factor = 7.2 / 1024;
const double gear_factor = -1.0 / 3.0;

AccelStepper left_motor(AccelStepper::DRIVER, left_step_pin, left_dir_pin);
AccelStepper right_motor(AccelStepper::DRIVER, right_step_pin, right_dir_pin);

void setup() {
  Wire.begin();

  left_motor.setMaxSpeed(2000.0);
  right_motor.setMaxSpeed(2000.0);

  left_motor.setAcceleration(1000.0);
  right_motor.setAcceleration(1000.0);

  pinMode(battery_pin, INPUT);

  Serial.begin(115200);
}

void loop() {
  if (Serial.available())
  {
    parseCommand();
    sendReply();
  }

  left_motor.run();
  right_motor.run();

  delay(10);
}

void parseCommand()
{
  while (Serial.read() != '$');
  left_motor.setSpeed(Serial.parseInt());
  right_motor.setSpeed(-1 * Serial.parseInt());
  Serial.read(); // Take the newline out of the receive buffer
}

void sendReply()
{
  Serial.print("$");
  Serial.print(left_motor.currentPosition());
  Serial.print(",");
  Serial.print(-1 * right_motor.currentPosition());
  Serial.print(",");
  Serial.print(left_motor.speed());
  Serial.print(",");
  Serial.print(-1 * right_motor.speed());
  Serial.print(",");
 // Serial.print(battery_factor * analogRead(battery_pin));
  Serial.println();
}