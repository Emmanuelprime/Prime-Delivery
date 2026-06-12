#include <Arduino.h>
#include <math.h>

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

// ================= ENCODERS =================
volatile long right_pulse_count = 0;
volatile long left_pulse_count  = 0;

// ================= ODOMETRY =================
float x = 0;
float y = 0;
float theta = 0;

float goal_x = 1.0;
float goal_y = 0.0;

float goal_threshold = 0.10;

// ================= ROBOT PARAMETERS =================
const float WHEEL_RADIUS = 0.16;
const float WHEEL_BASE   = 0.52;
const float PULSES_PER_REV = 90.0;

// ================= MOTION PARAMETERS =================
const float MAX_SPEED_PWM = 80;
const float MAX_DECEL = 1.2;   // tune this (m/s² equivalent scaling)
const float KP_ANGLE = 50;

// ================= TIMING =================
unsigned long last_time = 0;

// ================= ISR =================
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

// ================= SETUP =================
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
}

// ================= MOTOR CONTROL =================
void forward(int left, int right)
{
    digitalWrite(RIGHT_DIR, LOW);
    digitalWrite(LEFT_DIR, HIGH);

    ledcWrite(RIGHT_CH, right);
    ledcWrite(LEFT_CH, left);
}

void stopMotors()
{
    ledcWrite(RIGHT_CH, 0);
    ledcWrite(LEFT_CH, 0);
}

// ================= LOOP =================
void loop()
{
    // ================= GOAL =================
    float dx = goal_x - x;
    float dy = goal_y - y;

    float distance = sqrt(dx * dx + dy * dy);

    // STOP CONDITION
    if (distance < goal_threshold)
    {
        stopMotors();
        Serial.println("ARRIVED");
        delay(100);
        return;
    }

    // ================= VELOCITY PLANNING =================
    float v = sqrt(2 * MAX_DECEL * distance);

    // clamp speed
    if (v > MAX_SPEED_PWM) v = MAX_SPEED_PWM;
    if (v < 12) v = 12;

    int base_speed = (int)v;

    // ================= HEADING CONTROL =================
    float target_angle = atan2(dy, dx);
    float angle_error = target_angle - theta;

    while (angle_error > PI) angle_error -= 2 * PI;
    while (angle_error < -PI) angle_error += 2 * PI;

    int turn = angle_error * KP_ANGLE;

    int left_speed  = base_speed - turn;
    int right_speed = base_speed + turn;

    left_speed  = constrain(left_speed, 0, MAX_SPEED_PWM);
    right_speed = constrain(right_speed, 0, MAX_SPEED_PWM);

    forward(left_speed, right_speed);

    // ================= ODOMETRY UPDATE =================
    if (millis() - last_time >= 100)
    {
        noInterrupts();

        long right_count = right_pulse_count;
        long left_count  = left_pulse_count;

        right_pulse_count = 0;
        left_pulse_count = 0;

        interrupts();

        float right_dist = (2 * PI * WHEEL_RADIUS) * (right_count / PULSES_PER_REV);
        float left_dist  = (2 * PI * WHEEL_RADIUS) * (left_count / PULSES_PER_REV);

        float d_center = (right_dist + left_dist) / 2.0;
        float d_theta  = (right_dist - left_dist) / WHEEL_BASE;

        theta += d_theta;
        x += d_center * cos(theta);
        y += d_center * sin(theta);

        Serial.print("X:");
        Serial.print(x, 3);

        Serial.print(" Y:");
        Serial.print(y, 3);

        Serial.print(" θ:");
        Serial.print(theta, 3);

        Serial.print(" Dist:");
        Serial.println(distance);

        last_time = millis();
    }

    delay(10);
}