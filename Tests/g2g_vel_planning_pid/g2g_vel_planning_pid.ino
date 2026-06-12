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

volatile long right_pulse_count = 0;
volatile long left_pulse_count  = 0;

float x = 0;
float y = 0;
float theta = 0;

float goal_x = 1.0;
float goal_y = 0.0;
float goal_threshold = 0.10;

const float WHEEL_RADIUS = 0.16;
const float WHEEL_BASE   = 0.52;
const float PULSES_PER_REV = 90.0;

const float MAX_PWM = 80;
const float MAX_DECEL = 1.2;

float Kp_v = 0.8;
float Ki_v = 0.02;
float Kd_v = 0.0;

float Kp_h = 40;
float Ki_h = 0.0;
float Kd_h = 2.0;

float err_r = 0, err_l = 0;
float integral_r = 0, integral_l = 0;
float prev_err_r = 0, prev_err_l = 0;

float heading_integral = 0;
float prev_heading_err = 0;

unsigned long last_time = 0;

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

void loop()
{

    float dx = goal_x - x;
    float dy = goal_y - y;

    float distance = sqrt(dx * dx + dy * dy);

    if (distance < goal_threshold)
    {
        stopMotors();
        Serial.println("ARRIVED");
        delay(200);
        return;
    }

    float target_angle = atan2(dy, dx);

    float heading_error = target_angle - theta;

    while (heading_error > PI) heading_error -= 2 * PI;
    while (heading_error < -PI) heading_error += 2 * PI;

    heading_integral += heading_error;
    float heading_derivative = heading_error - prev_heading_err;

    float heading_output =
        Kp_h * heading_error +
        Ki_h * heading_integral +
        Kd_h * heading_derivative;

    prev_heading_err = heading_error;

    float v = sqrt(2 * MAX_DECEL * distance);

    if (v > MAX_PWM) v = MAX_PWM;
    if (v < 12) v = 12;

    float base_speed = v;

    float target_left  = base_speed - heading_output;
    float target_right = base_speed + heading_output;

    if (millis() - last_time >= 100)
    {
        noInterrupts();

        long rc = right_pulse_count;
        long lc = left_pulse_count;

        right_pulse_count = 0;
        left_pulse_count = 0;

        interrupts();

        float right_speed = (rc * 60.0) / PULSES_PER_REV;
        float left_speed  = (lc * 60.0) / PULSES_PER_REV;

        err_l = target_left - left_speed;
        integral_l += err_l;
        float d_l = err_l - prev_err_l;

        float out_l = Kp_v * err_l + Ki_v * integral_l + Kd_v * d_l;
        prev_err_l = err_l;

        err_r = target_right - right_speed;
        integral_r += err_r;
        float d_r = err_r - prev_err_r;

        float out_r = Kp_v * err_r + Ki_v * integral_r + Kd_v * d_r;
        prev_err_r = err_r;

        int pwm_l = constrain(out_l, 0, MAX_PWM);
        int pwm_r = constrain(out_r, 0, MAX_PWM);

        forward(pwm_l, pwm_r);

        float right_dist = (2 * PI * WHEEL_RADIUS) * (rc / PULSES_PER_REV);
        float left_dist  = (2 * PI * WHEEL_RADIUS) * (lc / PULSES_PER_REV);

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