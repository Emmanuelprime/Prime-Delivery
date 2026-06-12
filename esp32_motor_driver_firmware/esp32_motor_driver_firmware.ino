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
const int MIN_PWM = 0;                // Changed from 8 to 0 for testing
const int MAX_PWM = 60;              // Changed from 60 to 255 for full range

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
const int TELEMETRY_DT_MS = 50;  // 20Hz telemetry (less serial spam)

// Command timeout - DISABLED for testing
unsigned long last_command_time = 0;
const unsigned long COMMAND_TIMEOUT_MS = 30000;  // 30 seconds timeout

// Debug mode
bool debug_enabled = true;
unsigned long last_debug_time = 0;

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
    // Ensure PWM is within bounds
    pwm = constrain(pwm, -MAX_PWM, MAX_PWM);
    
    if (strcmp(motor, "left") == 0) {
        if (pwm >= 0) {
            digitalWrite(LEFT_DIR, HIGH);  // Forward
            ledcWrite(LEFT_CH, abs(pwm));
        } else {
            digitalWrite(LEFT_DIR, LOW);   // Reverse
            ledcWrite(LEFT_CH, abs(pwm));
        }
    } else if (strcmp(motor, "right") == 0) {
        if (pwm >= 0) {
            digitalWrite(RIGHT_DIR, LOW);  // Forward (mirrored mounting)
            ledcWrite(RIGHT_CH, abs(pwm));
        } else {
            digitalWrite(RIGHT_DIR, HIGH); // Reverse (mirrored mounting)
            ledcWrite(RIGHT_CH, abs(pwm));
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
    
    // Convert to PWM
    // For a typical motor: 0.5 m/s ≈ 100 PWM
    int pwm = (int)(output * 200.0);
    return constrain(pwm, -MAX_PWM, MAX_PWM);
}

// ============================================
// COMMAND PARSER
// ============================================
void parseCommand(String cmd) {
    cmd.trim();
    
    if (cmd.length() == 0) return;
    
    // Debug: print received command
    if (debug_enabled) {
        Serial.print("CMD:'"); Serial.print(cmd); Serial.println("'");
    }
    
    if (cmd.startsWith("v ")) {
        // Format: "v 0.3 0.0" (linear_vel angular_vel)
        int idx1 = cmd.indexOf(' ', 2);
        if (idx1 > 0) {
            float new_v = cmd.substring(2, idx1).toFloat();
            float new_omega = cmd.substring(idx1 + 1).toFloat();
            
            // Safety limits
            v_cmd = constrain(new_v, -MAX_VEL, MAX_VEL);
            omega_cmd = constrain(new_omega, -MAX_OMEGA, MAX_OMEGA);
            
            last_command_time = millis();
            
            if (debug_enabled) {
                Serial.print("SET: v="); Serial.print(v_cmd);
                Serial.print(" omega="); Serial.println(omega_cmd);
            }
        }
    }
    else if (cmd.startsWith("s") || cmd.startsWith("stop")) {
        // Emergency stop
        v_cmd = 0;
        omega_cmd = 0;
        stopMotors();
        Serial.println("STOP");
    }
    else if (cmd.startsWith("kp ")) {
        Kp = cmd.substring(3).toFloat();
        Serial.print("Kp="); Serial.println(Kp);
    }
    else if (cmd.startsWith("ki ")) {
        Ki = cmd.substring(3).toFloat();
        Serial.print("Ki="); Serial.println(Ki);
    }
    else if (cmd.startsWith("kd ")) {
        Kd = cmd.substring(3).toFloat();
        Serial.print("Kd="); Serial.println(Kd);
    }
    else if (cmd.startsWith("reset")) {
        x = 0; y = 0; theta = 0;
        left_integral = 0;
        right_integral = 0;
        Serial.println("RESET");
    }
    else if (cmd.startsWith("debug")) {
        debug_enabled = !debug_enabled;
        Serial.print("Debug: "); Serial.println(debug_enabled ? "ON" : "OFF");
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
// DEBUG OUTPUT
// ============================================
void printDebug(float dt, float target_left, float target_right, 
                int left_pwm, int right_pwm,
                long lc, long rc) {
    if (!debug_enabled) return;
    
    unsigned long now = millis();
    if (now - last_debug_time < 500) return;  // Every 500ms
    last_debug_time = now;
    
    Serial.print("DBG: dt="); Serial.print(dt, 3);
    Serial.print(" enc(L="); Serial.print(lc); Serial.print(" R="); Serial.print(rc); Serial.print(")");
    Serial.print(" vel(L="); Serial.print(left_vel, 3); Serial.print(" R="); Serial.print(right_vel, 3); Serial.print(")");
    Serial.print(" tgt(L="); Serial.print(target_left, 3); Serial.print(" R="); Serial.print(target_right, 3); Serial.print(")");
    Serial.print(" PWM(L="); Serial.print(left_pwm); Serial.print(" R="); Serial.print(right_pwm); Serial.print(")");
    Serial.print(" cmd(v="); Serial.print(v_cmd, 3); Serial.print(" w="); Serial.print(omega_cmd, 3); Serial.print(")");
    Serial.print(" cur(v="); Serial.print(v_current, 3); Serial.print(" w="); Serial.print(omega_current, 3); Serial.println(")");
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
    
    // Initialize motor pins to known state
    digitalWrite(RIGHT_DIR, LOW);
    digitalWrite(LEFT_DIR, LOW);
    
    ledcSetup(RIGHT_CH, PWM_FREQ, PWM_RES);
    ledcSetup(LEFT_CH, PWM_FREQ, PWM_RES);
    ledcAttachPin(RIGHT_PWM, RIGHT_CH);
    ledcAttachPin(LEFT_PWM, LEFT_CH);
    
    // Ensure motors are stopped
    ledcWrite(LEFT_CH, 0);
    ledcWrite(RIGHT_CH, 0);
    
    attachInterrupt(digitalPinToInterrupt(RIGHT_SIG), rightISR, RISING);
    attachInterrupt(digitalPinToInterrupt(LEFT_SIG), leftISR, RISING);
    
    last_control_time = millis();
    last_command_time = millis();
    
    delay(1000);
    Serial.println("ESP32_READY");
    Serial.flush();
    
    Serial.println("FIRMWARE V2 - Motor Test Ready");
    Serial.println("Commands: v [speed] [omega], s=stop, debug=toggle debug");
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
        if (v_cmd != 0 || omega_cmd != 0) {
            v_cmd = 0;
            omega_cmd = 0;
            if (debug_enabled) {
                Serial.println("TIMEOUT - No command received, stopping");
            }
        }
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
        if (dt <= 0) dt = 0.02;
        
        // Read encoders atomically
        noInterrupts();
        long rc = right_pulses;
        long lc = left_pulses;
        right_pulses = 0;
        left_pulses = 0;
        interrupts();
        
        // Calculate wheel velocities (m/s)
        if (dt > 0) {
            right_vel = (2.0 * PI * WHEEL_RADIUS * (float)rc) / (PULSES_PER_REV * dt);
            left_vel  = (2.0 * PI * WHEEL_RADIUS * (float)lc) / (PULSES_PER_REV * dt);
        }
        
        // Ramp commanded velocities
        rampVelocity(dt);
        
        // Differential drive kinematics
        float target_left_vel  = v_current - omega_current * WHEEL_BASE / 2.0;
        float target_right_vel = v_current + omega_current * WHEEL_BASE / 2.0;
        
        // Run wheel PID controllers
        int left_pwm  = wheelPID("left",  target_left_vel,  left_vel,  dt);
        int right_pwm = wheelPID("right", target_right_vel, right_vel, dt);
        
        // Debug output
        printDebug(dt, target_left_vel, target_right_vel, left_pwm, right_pwm, lc, rc);
        
        // Send PWM to motors (only if we have a command)
        if (now - last_command_time < COMMAND_TIMEOUT_MS) {
            setMotorPWM("left", left_pwm);
            setMotorPWM("right", right_pwm);
        } else {
            setMotorPWM("left", 0);
            setMotorPWM("right", 0);
        }
        
        // Update odometry
        float d_center = (right_vel + left_vel) * dt / 2.0;
        float d_theta  = (right_vel - left_vel) * dt / WHEEL_BASE;
        
        theta += d_theta;
        x += d_center * cos(theta);
        y += d_center * sin(theta);
        
        // Normalize theta to [-PI, PI]
        while (theta > PI) theta -= 2.0 * PI;
        while (theta < -PI) theta += 2.0 * PI;
    }
    
    // ==========================================
    // 4. SEND TELEMETRY (20Hz)
    // ==========================================
    if (now - last_telemetry_time >= TELEMETRY_DT_MS) {
        last_telemetry_time = now;
        sendTelemetry();
    }
}