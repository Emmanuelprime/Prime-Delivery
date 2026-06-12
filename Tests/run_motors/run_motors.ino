#include <Arduino.h>

#define RIGHT_PWM 19
#define RIGHT_DIR 18

#define LEFT_PWM 26
#define LEFT_DIR 27

#define PWM_FREQ 1000
#define PWM_RES 8

#define RIGHT_CH 0
#define LEFT_CH  1

void setup()
{
    Serial.begin(115200);

    pinMode(RIGHT_DIR, OUTPUT);
    pinMode(LEFT_DIR, OUTPUT);

    ledcSetup(RIGHT_CH, PWM_FREQ, PWM_RES);
    ledcSetup(LEFT_CH, PWM_FREQ, PWM_RES);

    ledcAttachPin(RIGHT_PWM, RIGHT_CH);
    ledcAttachPin(LEFT_PWM, LEFT_CH);

    stopMotors();
}

void loop()
{
    forward(30);
    delay(3000);

    stopMotors();
    delay(1000);

    backward(30);
    delay(3000);

    stopMotors();
    delay(1000);
}

void forward(uint8_t speed)
{
    digitalWrite(RIGHT_DIR, LOW);
    digitalWrite(LEFT_DIR, HIGH);

    ledcWrite(RIGHT_CH, speed);
    ledcWrite(LEFT_CH, speed);
}

void backward(uint8_t speed)
{
    digitalWrite(RIGHT_DIR, HIGH);
    digitalWrite(LEFT_DIR, LOW);

    ledcWrite(RIGHT_CH, speed);
    ledcWrite(LEFT_CH, speed);
}

void stopMotors()
{
    ledcWrite(RIGHT_CH, 0);
    ledcWrite(LEFT_CH, 0);
}