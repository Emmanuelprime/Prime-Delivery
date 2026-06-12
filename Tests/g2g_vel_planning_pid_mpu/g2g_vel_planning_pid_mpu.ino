#include <Arduino.h>
#include <math.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

// Motor pins
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

// MPU6050 object
MPU6050 mpu6050(Wire);

volatile long right_pulse_count = 0;
volatile long left_pulse_count  = 0;

float x = 0;
float y = 0;
float theta = 0;

// Fused heading (radians)
float fused_heading = 0;
unsigned long last_fusion_time = 0;

// Complementary filter coefficient (0-1)
const float ALPHA = 0.98;

float goal_x = 2.0;
float goal_y = 2.0;
float goal_threshold = 0.10;

const float WHEEL_RADIUS = 0.17;
const float WHEEL_BASE   = 0.521;
const float PULSES_PER_REV = 90.0;

const float MAX_PWM = 60;
const float MIN_PWM = 8;        // Reduced from 12
const float MAX_ACCEL = 1.2;    // Reduced from 1.5 for smoother acceleration
const float MAX_DECEL = 0.8;    // Reduced from 1.2 for smoother deceleration

// Current velocity for acceleration limiting
float current_velocity = 0;
unsigned long last_velocity_update = 0;

// TUNED GAINS - Reduced to prevent oscillation
float Kp_v = 0.9;      // Reduced from 0.8
float Ki_v = 0.01;     // Reduced from 0.02
float Kd_v = 0.0;

float Kp_h = 18;       // Greatly reduced from 40
float Ki_h = 0.0;
float Kd_h = 0.8;      // Reduced from 2.0

// Integral windup limits
const float HEADING_INTEGRAL_MAX = 30.0;  // Reduced from 50
const float SPEED_INTEGRAL_MAX = 150.0;   // Reduced from 200

// Heading deadband (radians)
const float HEADING_DEADBAND = 0.05;  // ~3 degrees

// Feed-forward heading factor
const float HEADING_FF_FACTOR = 0.3;

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

// Trapezoidal velocity profile planner
float planVelocity(float distance_to_target) {
    unsigned long current_time = millis();
    float dt_seconds = (current_time - last_velocity_update) / 1000.0;
    
    // Prevent dt from being too large on first run
    if (dt_seconds > 0.1) dt_seconds = 0.02;
    
    last_velocity_update = current_time;
    
    // Calculate target velocity based on distance (deceleration phase)
    float decel_target = sqrt(2 * MAX_DECEL * distance_to_target);
    
    // Limit max speed
    if (decel_target > MAX_PWM) decel_target = MAX_PWM;
    
    // Calculate the maximum allowed velocity change based on acceleration limits
    float max_velocity_change = MAX_ACCEL * dt_seconds;
    
    // Apply acceleration limiting (ramp up)
    if (decel_target > current_velocity) {
        // Accelerating - limit the rate
        current_velocity += max_velocity_change;
        if (current_velocity > decel_target) {
            current_velocity = decel_target;
        }
    } 
    // Apply deceleration limiting (ramp down)
    else if (decel_target < current_velocity) {
        // Decelerating - limit the rate
        current_velocity -= max_velocity_change;
        if (current_velocity < decel_target) {
            current_velocity = decel_target;
        }
    }
    
    // Apply absolute limits
    if (current_velocity > MAX_PWM) current_velocity = MAX_PWM;
    if (current_velocity < MIN_PWM && distance_to_target > goal_threshold) current_velocity = MIN_PWM;
    
    // Slow down when very close to goal (fixed type mismatch)
    if (distance_to_target < 0.3) {
        if (current_velocity > 30.0) {
            current_velocity = 30.0;
        }
    }
    
    return current_velocity;
}

void setup()
{
    Serial.begin(115200);

    // Initialize I2C for MPU6050
    Wire.begin();
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);

    // Initialize fused heading with MPU6050's absolute angle
    fused_heading = mpu6050.getAngleZ() * PI / 180.0;
    last_fusion_time = micros();
    last_velocity_update = millis();

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
    
    Serial.println("Robot starting...");
}

void loop()
{
    // Update MPU6050
    mpu6050.update();
    
    // Get gyro angular velocity (degrees/sec -> radians/sec)
    float gyro_z = mpu6050.getGyroZ() * PI / 180.0;
    
    // Calculate time delta for gyro integration
    unsigned long current_time = micros();
    float dt = (current_time - last_fusion_time) / 1000000.0;
    
    // Limit dt to prevent spikes
    if (dt > 0.02) dt = 0.02;
    
    last_fusion_time = current_time;
    
    // Integrate gyro to get angle change
    float gyro_angle = fused_heading + gyro_z * dt;
    
    float dx = goal_x - x;
    float dy = goal_y - y;

    float distance = sqrt(dx * dx + dy * dy);

    if (distance < goal_threshold)
    {
        stopMotors();
        // Reset integrals and velocity planner
        integral_r = 0;
        integral_l = 0;
        heading_integral = 0;
        current_velocity = 0;
        prev_err_r = 0;
        prev_err_l = 0;
        prev_heading_err = 0;
        Serial.println("ARRIVED at goal!");
        delay(200);
        return;
    }

    float target_angle = atan2(dy, dx);

    // Get encoder-based heading
    float encoder_heading = theta;
    
    // COMPLEMENTARY FILTER
    fused_heading = ALPHA * gyro_angle + (1.0 - ALPHA) * encoder_heading;
    
    // Normalize fused heading to [-PI, PI]
    while (fused_heading > PI) fused_heading -= 2 * PI;
    while (fused_heading < -PI) fused_heading += 2 * PI;
    
    float current_heading = fused_heading;
    float heading_error_raw = target_angle - current_heading;

    // Normalize heading error to [-PI, PI]
    while (heading_error_raw > PI) heading_error_raw -= 2 * PI;
    while (heading_error_raw < -PI) heading_error_raw += 2 * PI;

    // APPLY HEADING DEADBAND to prevent oscillation
    float heading_error = heading_error_raw;
    if (fabs(heading_error) < HEADING_DEADBAND) {
        heading_error = 0;
        heading_integral = 0; // Reset integral when within deadband
    }

    // HEADING PID WITH WINDUP PROTECTION
    heading_integral += heading_error;
    
    if (heading_integral > HEADING_INTEGRAL_MAX) {
        heading_integral = HEADING_INTEGRAL_MAX;
    } else if (heading_integral < -HEADING_INTEGRAL_MAX) {
        heading_integral = -HEADING_INTEGRAL_MAX;
    }
    
    float heading_derivative = heading_error - prev_heading_err;

    // ADD FEED-FORWARD TERM for better tracking
    float heading_ff = 0;
    if (distance > 0.5 && fabs(heading_error) > HEADING_DEADBAND) {
        // Increase feed-forward when farther away, reduce when close
        heading_ff = heading_error * HEADING_FF_FACTOR * (distance / 2.0);
        if (heading_ff > 5) heading_ff = 5;
        if (heading_ff < -5) heading_ff = -5;
    }

    float heading_output = Kp_h * heading_error + 
                           Ki_h * heading_integral + 
                           Kd_h * heading_derivative + 
                           heading_ff;

    prev_heading_err = heading_error;

    // TRAPEZOIDAL VELOCITY PLANNING
    float base_speed = planVelocity(distance);
    
    // Reduce base speed when heading error is large to prevent overshoot
    if (fabs(heading_error) > 0.3) {
        base_speed = base_speed * 0.5;
    }

    float target_left  = base_speed - heading_output;
    float target_right = base_speed + heading_output;

    if (millis() - last_time >= 50)  // Changed from 100ms to 50ms for faster response (20Hz)
    {
        noInterrupts();

        long rc = right_pulse_count;
        long lc = left_pulse_count;

        right_pulse_count = 0;
        left_pulse_count = 0;

        interrupts();

        // Calculate actual speeds (revolutions per second)
        float right_speed = (rc * 60.0) / PULSES_PER_REV;
        float left_speed  = (lc * 60.0) / PULSES_PER_REV;

        // SPEED PID WITH INTEGRAL WINDUP PROTECTION
        err_l = target_left - left_speed;
        integral_l += err_l;
        
        if (integral_l > SPEED_INTEGRAL_MAX) {
            integral_l = SPEED_INTEGRAL_MAX;
        } else if (integral_l < -SPEED_INTEGRAL_MAX) {
            integral_l = -SPEED_INTEGRAL_MAX;
        }
        
        float d_l = err_l - prev_err_l;

        float out_l = Kp_v * err_l + Ki_v * integral_l + Kd_v * d_l;
        prev_err_l = err_l;

        err_r = target_right - right_speed;
        integral_r += err_r;
        
        if (integral_r > SPEED_INTEGRAL_MAX) {
            integral_r = SPEED_INTEGRAL_MAX;
        } else if (integral_r < -SPEED_INTEGRAL_MAX) {
            integral_r = -SPEED_INTEGRAL_MAX;
        }
        
        float d_r = err_r - prev_err_r;

        float out_r = Kp_v * err_r + Ki_v * integral_r + Kd_v * d_r;
        prev_err_r = err_r;

        // SCALE MOTORS TOGETHER to maintain turning ratio
        float max_needed = max(fabs(out_l), fabs(out_r));
        if (max_needed > MAX_PWM) {
            float scale = MAX_PWM / max_needed;
            out_l *= scale;
            out_r *= scale;
        }
        
        // Ensure minimum PWM for movement when not at goal
        if (distance > goal_threshold) {
            if (out_l < MIN_PWM && out_l > 0) out_l = MIN_PWM;
            if (out_r < MIN_PWM && out_r > 0) out_r = MIN_PWM;
        }
        
        int pwm_l = constrain(out_l, 0, MAX_PWM);
        int pwm_r = constrain(out_r, 0, MAX_PWM);

        forward(pwm_l, pwm_r);

        // Update odometry
        float right_dist = (2 * PI * WHEEL_RADIUS) * (rc / PULSES_PER_REV);
        float left_dist  = (2 * PI * WHEEL_RADIUS) * (lc / PULSES_PER_REV);

        float d_center = (right_dist + left_dist) / 2.0;
        float d_theta  = (right_dist - left_dist) / WHEEL_BASE;

        theta += d_theta;
        x += d_center * cos(fused_heading);
        y += d_center * sin(fused_heading);
        
        // Normalize theta
        while (theta > PI) theta -= 2 * PI;
        while (theta < -PI) theta += 2 * PI;

        // Enhanced debugging output
        Serial.print("X:");
        Serial.print(x, 3);
        Serial.print(" Y:");
        Serial.print(y, 3);
        Serial.print(" D:");
        Serial.print(distance, 3);
        Serial.print(" Vp:");
        Serial.print(current_velocity, 1);
        Serial.print(" θ_f:");
        Serial.print(fused_heading * 180.0 / PI, 1);
        Serial.print(" θ_err:");
        Serial.print(heading_error * 180.0 / PI, 1);
        Serial.print(" PWM_L:");
        Serial.print(pwm_l);
        Serial.print(" PWM_R:");
        Serial.println(pwm_r);

        last_time = millis();
    }

    delay(5);  // Small delay to prevent overwhelming the CPU
}