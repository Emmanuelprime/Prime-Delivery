#include <Arduino.h>

#define RIGHT_PWM 19
#define RIGHT_DIR 18
#define RIGHT_SIG 23

#define LEFT_PWM 26
#define LEFT_DIR 27
#define LEFT_SIG 32

#define PWM_FREQ 1000
#define PWM_RES 8

#define RIGHT_CH 0
#define LEFT_CH  1

volatile long right_pulse_count = 0;
volatile long left_pulse_count  = 0;

unsigned long last_time = 0;

float right_rpm = 0;
float left_rpm = 0;

const float PULSES_PER_REV = 90.0;

void IRAM_ATTR rightISR()
{
    static uint32_t last = 0;
    uint32_t now = micros();

    if (now - last > 200)
    {
        right_pulse_count++;
        last = now;
    }
}

void IRAM_ATTR leftISR()
{
    static uint32_t last = 0;
    uint32_t now = micros();

    if (now - last > 200)
    {
        left_pulse_count++;
        last = now;
    }
}

void setup()
{
    Serial.begin(115200);

    pinMode(RIGHT_DIR, OUTPUT);
    pinMode(LEFT_DIR, OUTPUT);

    pinMode(RIGHT_SIG, INPUT_PULLUP);
    pinMode(LEFT_SIG, INPUT_PULLUP);

    ledcSetup(RIGHT_CH, PWM_FREQ, PWM_RES);
    ledcSetup(LEFT_CH, PWM_FREQ, PWM_RES);

    ledcAttachPin(RIGHT_PWM, RIGHT_CH);
    ledcAttachPin(LEFT_PWM, LEFT_CH);

    attachInterrupt(digitalPinToInterrupt(RIGHT_SIG), rightISR, RISING);
    attachInterrupt(digitalPinToInterrupt(LEFT_SIG), leftISR, RISING);

    last_time = millis();

    delay(2000);

    stopMotors();
}

void loop()
{
    forward(30);

    if (millis() - last_time >= 1000)
    {
        noInterrupts();

        long right_count = right_pulse_count;
        long left_count  = left_pulse_count;

        right_pulse_count = 0;
        left_pulse_count = 0;

        interrupts();

        right_rpm = (right_count * 60.0) / PULSES_PER_REV;
        left_rpm  = (left_count * 60.0) / PULSES_PER_REV;

        Serial.print("Right RPM: ");
        Serial.print(right_rpm);

        Serial.print(" | Left RPM: ");
        Serial.println(left_rpm);

        last_time = millis();
    }

    delay(10);
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