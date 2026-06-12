#include <Arduino.h>
#include <math.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

// ============================================
// HARDWARE CONFIGURATION
// ============================================
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

// ============================================
// ROBOT PARAMETERS (CALIBRATE THESE!)
// ============================================
const float WHEEL_RADIUS = 0.17;      // meters
const float WHEEL_BASE   = 0.521;     // meters
const float PULSES_PER_REV = 90.0;

// ============================================
// MOTOR LIMITS
// ============================================
const float MAX_VEL = 0.5;            // m/s
const float MAX_OMEGA = 1.5;          // rad/s
const float MAX_ACCEL = 0.5;          // m/s²
const float MAX_ALPHA = 2.0;          // rad/s² (angular accel)
const int MIN_PWM = 8;
const int MAX_PWM = 60;

// ============================================
// PID GAINS (tune these!)
// ============================================
float Kp = 0.5;
float Ki = 0.1;
float Kd = 0.0;
const float INTEGRAL_MAX = 30.0;

// ============================================
// GLOBAL STATE
// ============================================
MPU6050 mpu6050(Wire);

// Encoder counts (ISR updated)
volatile long right_pulses = 0;
volatile long left_pulses = 0;

// Odometry state
float x = 0.0, y = 0.0, theta = 0.0;
float left_vel = 0.0;   // Current measured left wheel velocity (m/s)
float right_vel = 0.0;  // Current measured right wheel velocity (m/s)

// Velocity commands (from Python)
float v_cmd = 0.0;       // m/s
float omega_cmd = 0.0;   // rad/s

// Current velocities (with ramping)
float v_current = 0.0;
float omega_current = 0.0;

// PID state per wheel
float left_integral = 0.0, right_integral = 0.0;
float left_prev_error = 0.0, right_prev_error = 0.0;

// Timing
unsigned long last_control_time = 0;
unsigned long last_telemetry_time = 0;
const int CONTROL_DT_MS = 20;    // 50Hz control loop
const int TELEMETRY_DT_MS = 20;  // 50Hz telemetry

// Command timeout - stop if no command received
unsigned long last_command_time = 0;
const unsigned long COMMAND_TIMEOUT_MS = 500;  // 500ms timeout

// ============================================
// INTERRUPT SERVICE ROUTINES
// ============================================
void IRAM_ATTR rightISR() {
    static uint32_t last = 0;
    uint32_t now = micros();
    if (now - last > 200) {
        right_pulses++;
        last = now;
    }
}

void IRAM_ATTR leftISR() {
    static uint32_t last = 0;
    uint32_t now = micros();
    if (now - last > 200) {
        left_pulses++;
        last = now;
    }
}

// ============================================
// MOTOR CONTROL
// ============================================
void setMotorPWM(const char* motor, int pwm) {
    if (strcmp(motor, "left") == 0) {
        if (pwm >= 0) {
            digitalWrite(LEFT_DIR, HIGH);  // Forward
            ledcWrite(LEFT_CH, constrain(pwm, MIN_PWM, MAX_PWM));
        } else {
            digitalWrite(LEFT_DIR, LOW);   // Reverse
            ledcWrite(LEFT_CH, constrain(-pwm, MIN_PWM, MAX_PWM));
        }
    } else if (strcmp(motor, "right") == 0) {
        if (pwm >= 0) {
            digitalWrite(RIGHT_DIR, LOW);  // Forward (mirrored mounting)
            ledcWrite(RIGHT_CH, constrain(pwm, MIN_PWM, MAX_PWM));
        } else {
            digitalWrite(RIGHT_DIR, HIGH); // Reverse (mirrored mounting)
            ledcWrite(RIGHT_CH, constrain(-pwm, MIN_PWM, MAX_PWM));
        }
    }
}

void stopMotors() {
    ledcWrite(LEFT_CH, 0);
    ledcWrite(RIGHT_CH, 0);
    v_current = 0;
    omega_current = 0;
    left_integral = 0;
    right_integral = 0;
}

// ============================================
// WHEEL SPEED PID CONTROLLER
// ============================================
int wheelPID(const char* motor, float target_vel, float measured_vel, float dt) {
    float error = target_vel - measured_vel;
    float* integral = (motor[0] == 'l') ? &left_integral : &right_integral;
    float* prev_error = (motor[0] == 'l') ? &left_prev_error : &right_prev_error;
    
    *integral += error * dt;
    *integral = constrain(*integral, -INTEGRAL_MAX, INTEGRAL_MAX);
    
    float derivative = 0.0;
    if (dt > 0.001) {
        derivative = (error - *prev_error) / dt;
    }
    *prev_error = error;
    
    float output = Kp * error + Ki * (*integral) + Kd * derivative;
    
    // Convert to PWM (approximate - should calibrate)
    int pwm = (int)(output * 200.0);  // 200 PWM per m/s (rough)
    return constrain(pwm, -MAX_PWM, MAX_PWM);
}

// ============================================
// COMMAND PARSER
// ============================================
void parseCommand(String cmd) {
    cmd.trim();
    
    if (cmd.startsWith("v ")) {
        // Format: "v 0.3 0.0" (linear_vel angular_vel)
        int idx1 = cmd.indexOf(' ', 2);
        if (idx1 > 0) {
            v_cmd = cmd.substring(2, idx1).toFloat();
            omega_cmd = cmd.substring(idx1 + 1).toFloat();
            
            // Safety limits
            v_cmd = constrain(v_cmd, -MAX_VEL, MAX_VEL);
            omega_cmd = constrain(omega_cmd, -MAX_OMEGA, MAX_OMEGA);
            
            last_command_time = millis();
        }
    }
    else if (cmd.startsWith("s")) {
        // Emergency stop
        v_cmd = 0;
        omega_cmd = 0;
        stopMotors();
        Serial.println("EMERGENCY_STOP");
    }
    else if (cmd.startsWith("kp ")) {
        // Tune PID gains remotely
        Kp = cmd.substring(3).toFloat();
        Serial.print("Kp set to: "); Serial.println(Kp);
    }
    else if (cmd.startsWith("ki ")) {
        Ki = cmd.substring(3).toFloat();
        Serial.print("Ki set to: "); Serial.println(Ki);
    }
    else if (cmd.startsWith("kd ")) {
        Kd = cmd.substring(3).toFloat();
        Serial.print("Kd set to: "); Serial.println(Kd);
    }
    else if (cmd.startsWith("reset")) {
        // Reset odometry
        x = 0; y = 0; theta = 0;
        Serial.println("Odometry reset");
    }
}

// ============================================
// VELOCITY RAMPING
// ============================================
void rampVelocity(float dt) {
    // Linear velocity ramping
    float dv = MAX_ACCEL * dt;
    if (v_cmd > v_current) {
        v_current = min(v_current + dv, v_cmd);
    } else if (v_cmd < v_current) {
        v_current = max(v_current - dv, v_cmd);
    }
    
    // Angular velocity ramping
    float domega = MAX_ALPHA * dt;
    if (omega_cmd > omega_current) {
        omega_current = min(omega_current + domega, omega_cmd);
    } else if (omega_cmd < omega_current) {
        omega_current = max(omega_current - domega, omega_cmd);
    }
}

// ============================================
// TELEMETRY SENDER
// ============================================
void sendTelemetry() {
    mpu6050.update();
    float gyro_z = mpu6050.getGyroZ() * PI / 180.0;
    
    // Format: "o x y theta v omega gyro_z left_vel right_vel"
    Serial.print("o ");
    Serial.print(x, 4); Serial.print(" ");
    Serial.print(y, 4); Serial.print(" ");
    Serial.print(theta, 4); Serial.print(" ");
    Serial.print(v_current, 4); Serial.print(" ");
    Serial.print(omega_current, 4); Serial.print(" ");
    Serial.print(gyro_z, 4); Serial.print(" ");
    Serial.print(left_vel, 4); Serial.print(" ");
    Serial.println(right_vel, 4);
}

// ============================================
// SETUP
// ============================================
void setup() {
    Serial.begin(115200);  // Match Python baud rate
    
    Wire.begin();
    Wire.setClock(400000);
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);
    
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
    
    last_control_time = millis();
    last_command_time = millis();
    
    delay(1000);
    Serial.println("ESP32_READY");
    Serial.flush();  // Force send immediately
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
    unsigned long now = millis();
    
    // ==========================================
    // 1. CHECK FOR COMMAND TIMEOUT (SAFETY)
    // ==========================================
    if (now - last_command_time > COMMAND_TIMEOUT_MS) {
        v_cmd = 0;
        omega_cmd = 0;
    }
    
    // ==========================================
    // 2. READ COMMANDS FROM PYTHON
    // ==========================================
    while (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        parseCommand(cmd);
    }
    
    // ==========================================
    // 3. RUN CONTROL LOOP (50Hz)
    // ==========================================
    if (now - last_control_time >= CONTROL_DT_MS) {
        float dt = (now - last_control_time) / 1000.0;
        last_control_time = now;
        
        // Prevent huge dt on first run or after lag
        if (dt > 0.1) dt = 0.02;
        
        // Read encoders
        noInterrupts();
        long rc = right_pulses;
        long lc = left_pulses;
        right_pulses = 0;
        left_pulses = 0;
        interrupts();
        
        // Calculate wheel velocities (update globals)
        if (dt > 0) {
            right_vel = (2 * PI * WHEEL_RADIUS * rc) / (PULSES_PER_REV * dt);
            left_vel  = (2 * PI * WHEEL_RADIUS * lc) / (PULSES_PER_REV * dt);
        }
        
        // Ramp commanded velocities
        rampVelocity(dt);
        
        // Differential drive kinematics
        float target_left_vel  = v_current - omega_current * WHEEL_BASE / 2.0;
        float target_right_vel = v_current + omega_current * WHEEL_BASE / 2.0;
        
        // Run wheel PID controllers
        int left_pwm  = wheelPID("left",  target_left_vel,  left_vel,  dt);
        int right_pwm = wheelPID("right", target_right_vel, right_vel, dt);
        
        // Send PWM to motors
        setMotorPWM("left", left_pwm);
        setMotorPWM("right", right_pwm);
        
        // Update odometry
        float d_center = (right_vel + left_vel) * dt / 2.0;
        float d_theta  = (right_vel - left_vel) * dt / WHEEL_BASE;
        
        theta += d_theta;
        x += d_center * cos(theta);
        y += d_center * sin(theta);
        
        // Normalize theta
        while (theta > PI) theta -= 2 * PI;
        while (theta < -PI) theta += 2 * PI;
    }
    
    // ==========================================
    // 4. SEND TELEMETRY (50Hz)
    // ==========================================
    if (now - last_telemetry_time >= TELEMETRY_DT_MS) {
        last_telemetry_time = now;
        sendTelemetry();
    }
}