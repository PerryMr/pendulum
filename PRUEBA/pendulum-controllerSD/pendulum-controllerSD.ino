/**
* @file pendulum-controller-single-swing.ino
* @brief Controlador de péndulo invertido - Swing-up con movimiento único
* @author Versión modificada para swing-up con movimiento único preciso
* @date 2025-08-22
*
* @details
* MODIFICACIONES PRINCIPALES:
* 1. Swing-up con movimiento único: 180° forward + 45° backward
* 2. Movimiento con máxima fuerza y rapidez
* 3. Monitoreo constante del encoder del péndulo
* 4. Parámetros ajustables para reducir el recorrido si es necesario
* 5. NUEVO: Comando para control directo sin swing-up
*/
#include "RotaryEncoder.h"
#include "L6474.h"
#include "control-comms.hpp"
#include <math.h>

/******************************************************************************
* Constants and globals
*/
// Pin definitions
const int LED_PIN = LED_BUILTIN;
const int ENC_A_PIN = D4; // Green wire - Encoder (péndulo sensor)
const int ENC_B_PIN = D5; // White wire - Encoder (péndulo sensor)

// Stepper motor pins
const int STP_FLAG_IRQ_PIN = D2;
const int STP_STBY_RST_PIN = D8;
const int STP_DIR_PIN = D7;
const int STP_PWM_PIN = D9;
const int8_t STP_SPI_CS_PIN = D10;
const int8_t STP_SPI_MOSI_PIN = D11;
const int8_t STP_SPI_MISO_PIN = D12;
const int8_t STP_SPI_SCK_PIN = D13;

// Communication constants
static const unsigned int BAUD_RATE = 500000;
static const ControlComms::DebugLevel CTRL_DEBUG = ControlComms::DEBUG_ERROR;
static constexpr size_t NUM_ACTIONS = 6;
static constexpr size_t NUM_OBS = 6;

// Command definitions - ACTUALIZADOS
static const unsigned int CMD_SET_HOME = 0;
static const unsigned int CMD_MOVE_TO = 1;
static const unsigned int CMD_MOVE_BY = 2;
static const unsigned int CMD_SET_STEP_MODE = 3;
static const unsigned int CMD_SELECT_CONTROLLER = 4;
static const unsigned int CMD_SET_PID_GAINS = 5;
static const unsigned int CMD_SET_LQR_GAINS = 6;
static const unsigned int CMD_START_CONTROL = 7;          // Inicia con swing-up
static const unsigned int CMD_STOP_CONTROL = 8;
static const unsigned int CMD_SET_SWING_PARAMS = 9;
static const unsigned int CMD_START_DIRECT_CONTROL = 10;  // NUEVO: Control directo sin swing-up
static const unsigned int CMD_SET_SWING_MODE = 11;        // NUEVO: Habilitar/Deshabilitar swing-up

// Status codes
static const unsigned int STATUS_OK = 0;
static const unsigned int STATUS_STP_MOVING = 1;
static const unsigned int STATUS_SWING_UP = 2;
static const unsigned int STATUS_CONTROL_ACTIVE = 3;
static const unsigned int STATUS_UPRIGHT_ACHIEVED = 4;
static const unsigned int STATUS_SWING_FORWARD = 5;
static const unsigned int STATUS_SWING_BACKWARD = 6;
static const unsigned int STATUS_DIRECT_CONTROL = 7;     // NUEVO: Control directo activo

// Physical constants
const float SAMPLE_TIME = 0.004; // 4ms sampling time
const float l = 0.258; // Length parameter
const float r = 0.141; // Rotor parameter
const float g = 9.806; // Gravity
const float sigma = g/(l*pow(2*PI*1.19, 2));

// Encoder and stepper constants
const int ENC_STEPS_PER_ROTATION = 1200;
const int STP_STEPS_PER_ROTATION = 200;

// Control parameters
const float UPRIGHT_THRESHOLD = 15.0; // degrees from 180
const float SETPOINT = 180.0; // Upright position in degrees

// PARÁMETROS DE SWING-UP MODIFICADO - MOVIMIENTO ÚNICO
float SWING_FORWARD_ANGLE = 180.0;  // Ángulo de avance inicial (ajustable)
float SWING_BACKWARD_ANGLE = 45.0;  // Ángulo de retroceso (ajustable)
float SWING_FORWARD_SPEED = 3500.0; // Velocidad máxima para avance (steps/sec)
float SWING_BACKWARD_SPEED = 1800.0; // Velocidad para retroceso (steps/sec)
const float SWING_WAIT_BEFORE_BACKWARD = 5.0; // Tiempo de espera antes del retroceso (segundos)
const float SWING_MAX_DURATION = 6.0; // Tiempo máximo para swing-up
const float SWING_SUCCESS_THRESHOLD = 150.0; // Ángulo mínimo para considerar éxito

// Variables globales adicionales para configuración dinámica
bool swing_up_mode = false;
bool swing_up_enabled = true;  // NUEVO: Controla si swing-up está habilitado globalmente
int swing_up_div_per_step = 4; // Divisor de pasos para swing-up (para más torque)

// Controller selection
typedef enum {
  CONTROLLER_PID = 0,
  CONTROLLER_LQR = 1
} ControllerType;

// System states - ACTUALIZADOS
typedef enum {
  STATE_IDLE = 0,
  STATE_SWING_UP = 1,
  STATE_CONTROL = 2,
  STATE_STOPPED = 3,
  STATE_DIRECT_CONTROL = 4  // NUEVO: Control directo sin swing-up
} SystemState;

// Swing-up sub-states modificados para movimiento único
typedef enum {
  SWING_FORWARD_PHASE = 0,    // Movimiento hacia adelante 180°
  SWING_WAIT_PHASE = 1,       // Espera breve
  SWING_BACKWARD_PHASE = 2,   // Movimiento hacia atrás 45°
  SWING_MONITOR_PHASE = 3,    // Monitoreo para verificar éxito
  SWING_COMPLETED = 4         // Swing-up completado
} SwingState;

// Global variables
RotaryEncoder *encoder = nullptr;
SPIClass dev_spi(STP_SPI_MOSI_PIN, STP_SPI_MISO_PIN, STP_SPI_SCK_PIN);
L6474 *stepper;
ControlComms ctrl;
unsigned int div_per_step = 16;

// Control variables
ControllerType current_controller = CONTROLLER_PID;
SystemState system_state = STATE_IDLE;

// PID gains
float K_p = 0.1, K_i = 0.0008, K_d = 0.012;
float pid_error_prev = 0.0, pid_integral = 0.0;

// LQR gains
float K_lqr[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

// State variables
float phi = 0.0; // Rotor angle (degrees)
float phi_prev = 0.0;
float dphi = 0.0; // Rotor angular velocity
float theta = 0.0; // Pendulum angle (degrees)
float theta_prev = 0.0;
float dtheta = 0.0; // Pendulum angular velocity
float integral_error = 0.0;

// Swing-up variables modificadas para movimiento único
float phi_reference = 0.0;
float phi_start_position = 0.0; // Posición inicial del motor
float theta_start_angle = 0.0;  // Ángulo inicial del péndulo
unsigned long swing_start_time = 0;
unsigned long swing_phase_start_time = 0;
bool upright_achieved = false;
SwingState swing_state = SWING_FORWARD_PHASE;
float theta_max_achieved = 0.0;

// Timing variables
unsigned long current_time = 0;
unsigned long prev_time = 0;
unsigned long control_loop_timer = 0;
const unsigned long CONTROL_INTERVAL = 4; // 4ms = 250Hz

// Debug variables
unsigned long last_debug_print = 0;
const unsigned long DEBUG_INTERVAL = 200; // Print debug cada 200ms

// Stepper configuration para swing-up con máxima potencia
L6474_init_t stepper_config_swing_up = {
  20000,    // acc - Aceleración máxima
  20000,    // dec - Deceleración máxima
  3500,     // max_speed - Velocidad alta
  500,      // min_speed - Velocidad mínima
  800,      // tval - Corriente máxima para torque alto
  L6474_OCD_TH_4500mA, // Umbral de sobrecorriente alto
  L6474_CONFIG_OC_SD_ENABLE,
  L6474_CONFIG_EN_TQREG_TVAL_USED,
  L6474_STEP_SEL_1_4,  // Micropasos reducidos para mayor torque
  L6474_SYNC_SEL_1_2,
  L6474_FAST_STEP_4us,  // Tiempo de step rápido
  L6474_TOFF_FAST_2us,  // Tiempo off rápido
  3,
  15,
  L6474_CONFIG_TOFF_024us,
  L6474_CONFIG_SR_320V_us,
  L6474_CONFIG_INT_16MHZ,
  L6474_ALARM_EN_OVERCURRENT | L6474_ALARM_EN_THERMAL_SHUTDOWN |
  L6474_ALARM_EN_THERMAL_WARNING | L6474_ALARM_EN_UNDERVOLTAGE |
  L6474_ALARM_EN_SW_TURN_ON | L6474_ALARM_EN_WRONG_NPERF_CMD
};

L6474_init_t stepper_config_control = {
  8000,     // acc
  8000,     // dec
  1500,     // max_speed
  200,      // min_speed
  400,      // tval - Corriente moderada para control
  L6474_OCD_TH_1500mA,
  L6474_CONFIG_OC_SD_ENABLE,
  L6474_CONFIG_EN_TQREG_TVAL_USED,
  L6474_STEP_SEL_1_16, // Micropasos altos para precisión
  L6474_SYNC_SEL_1_2,
  L6474_FAST_STEP_12us,
  L6474_TOFF_FAST_8us,
  3,
  21,
  L6474_CONFIG_TOFF_044us,
  L6474_CONFIG_SR_320V_us,
  L6474_CONFIG_INT_16MHZ,
  L6474_ALARM_EN_OVERCURRENT | L6474_ALARM_EN_THERMAL_SHUTDOWN |
  L6474_ALARM_EN_THERMAL_WARNING | L6474_ALARM_EN_UNDERVOLTAGE |
  L6474_ALARM_EN_SW_TURN_ON | L6474_ALARM_EN_WRONG_NPERF_CMD
};

/******************************************************************************
* Interrupt service routines
*/
void stepperISR(void) {
  stepper->isr_flag = TRUE;
  unsigned int status = stepper->get_status();
  if ((status & L6474_STATUS_NOTPERF_CMD) == L6474_STATUS_NOTPERF_CMD) {
    // Serial.println("WARNING: FLAG interrupt triggered.");
  }
  stepper->isr_flag = FALSE;
}

void encoderISR() {
  encoder->tick();
}

/******************************************************************************
* Utility functions
*/
float get_encoder_angle() {
  int pos = encoder->getPosition();
  pos = pos % ENC_STEPS_PER_ROTATION;
  pos = pos >= 0 ? pos : pos + ENC_STEPS_PER_ROTATION;
  return (float)pos * (360.0 / ENC_STEPS_PER_ROTATION);
}

float get_stepper_angle() {
  int pos = stepper->get_position();
  pos = pos % (STP_STEPS_PER_ROTATION * div_per_step);
  pos = pos >= 0 ? pos : pos + (STP_STEPS_PER_ROTATION * div_per_step);
  return (float)pos * (360.0 / (STP_STEPS_PER_ROTATION * div_per_step));
}

void set_stepper_home() {
  stepper->set_home();
  phi_reference = 0.0;
  upright_achieved = false;
  Serial.println("Stepper home position set");
}

void move_stepper_to(float deg) {
  int steps = (int)(deg * STP_STEPS_PER_ROTATION * div_per_step / 360.0);
  stepper->go_to(steps);
}

void move_stepper_by(float deg) {
  StepperMotor::direction_t stp_dir = StepperMotor::FWD;
  int steps = (int)(deg * STP_STEPS_PER_ROTATION * div_per_step / 360.0);
  if (steps < 0) {
    steps = -1 * steps;
    stp_dir = StepperMotor::BWD;
  }
  stepper->move(stp_dir, steps);
}

void move_stepper_fast_precise(float deg, float speed) {
  // Configurar velocidad específica para el movimiento
  stepper->set_max_speed((int)speed);
  stepper->set_acceleration(20000); // Aceleración máxima
  stepper->set_deceleration(15000); // Deceleración controlada
  move_stepper_by(deg);
  
  Serial.print("Movimiento preciso: "); Serial.print(deg, 1);
  Serial.print("° a "); Serial.print(speed, 0); Serial.println(" steps/sec");
}

void set_step_mode(int mode) {
  switch (mode) {
    case 0: stepper->set_step_mode(StepperMotor::STEP_MODE_FULL); div_per_step = 1; break;
    case 1: stepper->set_step_mode(StepperMotor::STEP_MODE_HALF); div_per_step = 2; break;
    case 2: stepper->set_step_mode(StepperMotor::STEP_MODE_1_4); div_per_step = 4; break;
    case 3: stepper->set_step_mode(StepperMotor::STEP_MODE_1_8); div_per_step = 8; break;
    case 4: stepper->set_step_mode(StepperMotor::STEP_MODE_1_16); div_per_step = 16; break;
    default: break;
  }
}

/******************************************************************************
* State estimation
*/
void update_states() {
  current_time = millis();
  float dt = (current_time - prev_time) / 1000.0;
  if (dt <= 0.0) dt = SAMPLE_TIME;

  // Update angles
  phi_prev = phi;
  theta_prev = theta;
  phi = get_stepper_angle();
  theta = get_encoder_angle();

  // Calculate derivatives
  dphi = (phi - phi_prev) / dt;
  dtheta = (theta - theta_prev) / dt;

  // Limit velocities to avoid noise spikes
  if (abs(dphi) > 1000.0) dphi = 0.0;
  if (abs(dtheta) > 1000.0) dtheta = 0.0;

  // Update integral error for control
  float error = SETPOINT - theta;
  integral_error += error * dt;

  // Limit integral windup
  if (integral_error > 100.0) integral_error = 100.0;
  if (integral_error < -100.0) integral_error = -100.0;

  // Track maximum angle achieved during swing-up
  if (system_state == STATE_SWING_UP) {
    float angle_from_bottom = theta;
    if (angle_from_bottom > 180.0) angle_from_bottom = 360.0 - angle_from_bottom;
    if (angle_from_bottom > theta_max_achieved) {
      theta_max_achieved = angle_from_bottom;
    }
  }

  prev_time = current_time;
}

/******************************************************************************
* Control algorithms
*/
float compute_pid_control() {
  float error = SETPOINT - theta;
  pid_integral += error * SAMPLE_TIME;

  // Limit integral windup
  if (pid_integral > 100.0) pid_integral = 100.0;
  if (pid_integral < -100.0) pid_integral = -100.0;

  float derivative = (error - pid_error_prev) / SAMPLE_TIME;
  float output = K_p * error + K_i * pid_integral + K_d * derivative;
  pid_error_prev = error;

  // Limit output
  if (output > 45.0) output = 45.0;
  if (output < -45.0) output = -45.0;

  return output;
}

float compute_lqr_control() {
  // State vector: [phi_error, dphi, theta_error, dtheta, integral_error]
  float phi_error_rad = (phi - phi_reference) * PI / 180.0;
  float dphi_rad = dphi * PI / 180.0;
  float theta_error_rad = (theta - SETPOINT) * PI / 180.0;
  float dtheta_rad = dtheta * PI / 180.0;

  // LQR control law: u = -K * x
  float control = -(K_lqr[0] * phi_error_rad +
                   K_lqr[1] * dphi_rad +
                   K_lqr[2] * theta_error_rad +
                   K_lqr[3] * dtheta_rad +
                   K_lqr[4] * integral_error);

  // Convert back to degrees and limit
  control = control * 180.0 / PI;
  if (control > 45.0) control = 45.0;
  if (control < -45.0) control = -45.0;

  return control;
}

bool is_pendulum_upright() {
  float distance_from_180 = abs(theta - 180.0);
  if (distance_from_180 > 180.0) {
    distance_from_180 = 360.0 - distance_from_180;
  }
  return distance_from_180 <= UPRIGHT_THRESHOLD;
}

void configure_stepper_for_swing_up() {
  Serial.println("=== CONFIGURANDO MOTOR PARA SWING-UP ÚNICO (MÁXIMA POTENCIA) ===");
  
  // Configurar parámetros para máxima potencia
  stepper->set_parameter(L6474_TVAL, 800); // Corriente máxima
  stepper->set_parameter(L6474_OCD_TH, L6474_OCD_TH_4500mA); // Umbral alto
  stepper->set_parameter(L6474_TON_MIN, 3);
  stepper->set_parameter(L6474_TOFF_MIN, 15);
  
  // Configurar micropasos para mayor torque
  stepper->set_step_mode(StepperMotor::STEP_MODE_1_4);
  div_per_step = swing_up_div_per_step;
  
  // Configurar aceleraciones máximas
  stepper->set_acceleration(20000);
  stepper->set_deceleration(15000);
  
  swing_up_mode = true;
  
  Serial.print("- Corriente: 800 (máxima)"); Serial.println();
  Serial.print("- Micropasos: 1/4 (para torque)"); Serial.println();
  Serial.print("- Aceleración: 20000 (máxima)"); Serial.println();
  Serial.println("Motor listo para swing-up único");
}

void configure_stepper_for_control() {
  Serial.println("=== CONFIGURANDO MOTOR PARA CONTROL (PRECISIÓN) ===");
  
  stepper->set_parameter(L6474_TVAL, 400); // Corriente moderada
  stepper->set_parameter(L6474_OCD_TH, L6474_OCD_TH_1500mA);
  stepper->set_parameter(L6474_TON_MIN, 3);
  stepper->set_parameter(L6474_TOFF_MIN, 21);
  
  // Micropasos altos para precisión
  stepper->set_step_mode(StepperMotor::STEP_MODE_1_16);
  div_per_step = 16;
  
  stepper->set_max_speed(200);
  stepper->set_acceleration(500);
  stepper->set_deceleration(500);
  
  swing_up_mode = false;
  
  Serial.println("Motor configurado para control de precisión");
}

/******************************************************************************
* NUEVO: Funciones para control directo
*/
void start_direct_control() {
  Serial.println("=== INICIANDO CONTROL DIRECTO (SIN SWING-UP) ===");
  Serial.println("Asumiendo que el péndulo está cerca de la posición invertida");
  
  // Configurar motor para control de precisión inmediatamente
  configure_stepper_for_control();
  
  // Establecer posición de referencia actual
  phi_reference = phi;
  
  // Reset controlador
  pid_integral = 0.0;
  pid_error_prev = 0.0;
  integral_error = 0.0;
  upright_achieved = false;
  
  // Cambiar directamente al estado de control
  system_state = STATE_DIRECT_CONTROL;
  
  Serial.print("Posición inicial del péndulo: "); Serial.print(theta, 1); Serial.println("°");
  Serial.print("Posición inicial del motor: "); Serial.print(phi, 1); Serial.println("°");
  Serial.println("Control activo - sin fase de swing-up");
  
  // Verificar si ya está en posición invertida
  if (is_pendulum_upright()) {
    Serial.println("¡Péndulo ya está en posición invertida!");
    upright_achieved = true;
  } else {
    float distance_from_180 = abs(theta - 180.0);
    if (distance_from_180 > 180.0) {
      distance_from_180 = 360.0 - distance_from_180;
    }
    Serial.print("Distancia a posición invertida: "); Serial.print(distance_from_180, 1); Serial.println("°");
    if (distance_from_180 > 90.0) {
      Serial.println("ADVERTENCIA: Péndulo muy lejos de posición invertida");
      Serial.println("El control directo puede no ser efectivo");
    }
  }
}

void run_direct_control() {
  // Mismo algoritmo de control que STATE_CONTROL, pero con status diferente
  float control_output = 0.0;
  
  if (current_controller == CONTROLLER_PID) {
    control_output = compute_pid_control();
  } else {
    control_output = compute_lqr_control();
  }

  // Aplicar control como corrección de posición
  float target_phi = phi_reference + control_output;
  float phi_error = target_phi - phi;
  
  // Mover stepper solo si el error es significativo
  if (abs(phi_error) > 1.0 && stepper->get_device_state() == INACTIVE) {
    move_stepper_by(phi_error * 0.5); // Movimiento suave
  }
  
  // Verificar si se logró la posición invertida
  if (is_pendulum_upright() && !upright_achieved) {
    upright_achieved = true;
    Serial.println("*** ¡POSICIÓN INVERTIDA LOGRADA CON CONTROL DIRECTO! ***");
    Serial.print("Ángulo del péndulo: "); Serial.print(theta, 1); Serial.println("°");
  }
}

/******************************************************************************
* SWING-UP ALGORITHM MODIFICADO - MOVIMIENTO ÚNICO
*/
void reset_swing_up_variables() {
  swing_start_time = millis();
  swing_phase_start_time = millis();
  upright_achieved = false;
  swing_state = SWING_FORWARD_PHASE;
  theta_max_achieved = 0.0;
  
  // Guardar posiciones iniciales
  phi_start_position = phi;
  theta_start_angle = theta;

  // CONFIGURAR MOTOR PARA MÁXIMA POTENCIA
  configure_stepper_for_swing_up();

  Serial.println("=== INICIANDO SWING-UP ÚNICO MODIFICADO ===");
  Serial.print("Ángulo inicial péndulo: "); Serial.print(theta, 1); Serial.println("°");
  Serial.print("Posición inicial stepper: "); Serial.print(phi, 1); Serial.println("°");
  Serial.println("SECUENCIA PLANIFICADA:");
  Serial.print("1. Avance rápido: "); Serial.print(SWING_FORWARD_ANGLE, 1); 
  Serial.print("° a "); Serial.print(SWING_FORWARD_SPEED, 0); Serial.println(" steps/sec");
  Serial.print("2. Espera: "); Serial.print(SWING_WAIT_BEFORE_BACKWARD * 1000, 0); Serial.println(" ms");
  Serial.print("3. Retroceso: "); Serial.print(SWING_BACKWARD_ANGLE, 1); 
  Serial.print("° a "); Serial.print(SWING_BACKWARD_SPEED, 0); Serial.println(" steps/sec");
  Serial.println("4. Monitoreo para verificar éxito");
}

void run_swing_up_modified() {
  float elapsed_time = (current_time - swing_start_time) / 1000.0;
  float phase_elapsed = (current_time - swing_phase_start_time) / 1000.0;

  // Debug cada fase
  if (current_time - last_debug_print > DEBUG_INTERVAL) {
    Serial.print("SWING [T:"); Serial.print(elapsed_time, 1);
    Serial.print("s] θ:"); Serial.print(theta, 1);
    Serial.print("° φ:"); Serial.print(phi, 1);
    Serial.print("° θ_max:"); Serial.print(theta_max_achieved, 1);
    Serial.print("° Fase:");
    switch(swing_state) {
      case SWING_FORWARD_PHASE: Serial.print("AVANCE"); break;
      case SWING_WAIT_PHASE: Serial.print("ESPERA"); break;
      case SWING_BACKWARD_PHASE: Serial.print("RETROCESO"); break;
      case SWING_MONITOR_PHASE: Serial.print("MONITOR"); break;
      case SWING_COMPLETED: Serial.print("COMPLETADO"); break;
    }
    Serial.print(" Motor:"); 
    Serial.print(stepper->get_device_state() == INACTIVE ? "PARADO" : "MOVIENDO");
    Serial.println();
    last_debug_print = current_time;
  }

  // VERIFICAR POSICIÓN INVERTIDA EN CUALQUIER MOMENTO
  if (is_pendulum_upright()) {
    if (!upright_achieved) {
      stepper->hard_stop();
      phi_reference = phi;
      upright_achieved = true;
      Serial.println("*** ¡POSICIÓN INVERTIDA ALCANZADA! ***");
      Serial.print("Tiempo: "); Serial.print(elapsed_time, 2); Serial.println("s");
      Serial.print("Ángulo final: "); Serial.print(theta, 1); Serial.println("°");
      Serial.print("Fase actual: "); Serial.println((int)swing_state);
    }

    // TRANSICIÓN AL CONTROL
    system_state = STATE_CONTROL;
    configure_stepper_for_control();

    // Reset controlador
    pid_integral = 0.0;
    pid_error_prev = 0.0;
    integral_error = 0.0;

    Serial.println("=== TRANSICIÓN A CONTROL DE ESTABILIZACIÓN ===");
    return;
  }

  // Verificar timeout
  if (elapsed_time > SWING_MAX_DURATION) {
    Serial.println("*** TIMEOUT EN SWING-UP - ANÁLISIS ***");
    Serial.print("Tiempo total: "); Serial.print(elapsed_time, 1); Serial.println("s");
    Serial.print("Ángulo máximo alcanzado: "); Serial.print(theta_max_achieved, 1); Serial.println("°");
    Serial.print("Ángulo final: "); Serial.print(theta, 1); Serial.println("°");
    Serial.print("Fase final: "); Serial.println((int)swing_state);
    
    if (theta_max_achieved > SWING_SUCCESS_THRESHOLD) {
      Serial.println("RESULTADO: Swing-up exitoso pero sin estabilizar");
      Serial.println("SUGERENCIA: Iniciar control manual");
    } else {
      Serial.println("RESULTADO: Swing-up fallido");
      Serial.println("SUGERENCIAS:");
      Serial.print("- Reducir SWING_FORWARD_ANGLE (actualmente "); 
      Serial.print(SWING_FORWARD_ANGLE); Serial.println("°)");
      Serial.print("- Aumentar SWING_FORWARD_SPEED (actualmente "); 
      Serial.print(SWING_FORWARD_SPEED); Serial.println(" steps/sec)");
      Serial.println("- Verificar conexiones mecánicas y eléctricas");
    }

    stepper->hard_stop();
    phi_reference = phi;
    system_state = STATE_CONTROL;
    configure_stepper_for_control();
    return;
  }

  // MÁQUINA DE ESTADOS DEL SWING-UP MODIFICADO
  switch(swing_state) {
    case SWING_FORWARD_PHASE:
      // Iniciar movimiento hacia adelante
      if (stepper->get_device_state() == INACTIVE) {
        Serial.println("*** INICIANDO FASE DE AVANCE ***");
        move_stepper_fast_precise(SWING_FORWARD_ANGLE, SWING_FORWARD_SPEED);
        swing_phase_start_time = current_time;
      }
      
      // Verificar si el movimiento ha terminado
      if (stepper->get_device_state() == INACTIVE && phase_elapsed > 0.5) {
        float motor_movement = phi - phi_start_position;
        Serial.print("AVANCE COMPLETADO - Movimiento real: "); 
        Serial.print(motor_movement, 1); Serial.println("°");
        swing_state = SWING_WAIT_PHASE;
        swing_phase_start_time = current_time;
      }
      break;

    case SWING_WAIT_PHASE:
      // Esperar tiempo definido antes del retroceso
      if (phase_elapsed >= SWING_WAIT_BEFORE_BACKWARD) {
        Serial.println("*** INICIANDO FASE DE RETROCESO ***");
        swing_state = SWING_BACKWARD_PHASE;
        swing_phase_start_time = current_time;
      }
      break;

    case SWING_BACKWARD_PHASE:
      // Iniciar movimiento hacia atrás
      if (stepper->get_device_state() == INACTIVE) {
        move_stepper_fast_precise(-SWING_BACKWARD_ANGLE, SWING_BACKWARD_SPEED);
      }
      
      // Verificar si el movimiento ha terminado
      if (stepper->get_device_state() == INACTIVE && phase_elapsed > 0.5) {
        Serial.println("RETROCESO COMPLETADO");
        swing_state = SWING_MONITOR_PHASE;
        swing_phase_start_time = current_time;
      }
      break;

    case SWING_MONITOR_PHASE:
      // Monitorear por un tiempo adicional para ver si se alcanza la posición
      if (phase_elapsed > 3.0) { // Monitorear por 3 segundos adicionales
        Serial.println("*** FASE DE MONITOREO COMPLETADA ***");
        Serial.print("Ángulo final del péndulo: "); Serial.print(theta, 1); Serial.println("°");
        Serial.print("Ángulo máximo alcanzado: "); Serial.print(theta_max_achieved, 1); Serial.println("°");
        
        if (theta_max_achieved > SWING_SUCCESS_THRESHOLD) {
          Serial.println("SWING-UP EXITOSO pero péndulo no estable en posición invertida");
        } else {
          Serial.println("SWING-UP NO EXITOSO - Ángulo insuficiente");
        }
        
        swing_state = SWING_COMPLETED;
      }
      break;

    case SWING_COMPLETED:
      // Swing-up completado, mantener monitoreo pero no hacer más movimientos
      // El sistema permanecerá aquí hasta timeout o logro de posición invertida
      if (phase_elapsed > 2.0) {
        // Si ha pasado suficiente tiempo sin lograr la posición, intentar control directo
        if (theta_max_achieved > 120.0) { // Si se acercó bastante
          Serial.println("Intentando transición a control directo...");
          system_state = STATE_CONTROL;
          configure_stepper_for_control();
          phi_reference = phi;
          pid_integral = 0.0;
          pid_error_prev = 0.0;
          integral_error = 0.0;
        }
      }
      break;
  }
}

/******************************************************************************
* Main control state machine - ACTUALIZADA
*/
void run_control_loop() {
  // Only run control loop at specified intervals
  if (millis() - control_loop_timer < CONTROL_INTERVAL) {
    return;
  }
  control_loop_timer = millis();

  update_states();

  float control_output = 0.0;

  switch (system_state) {
    case STATE_SWING_UP:
      run_swing_up_modified();
      // Para monitoreo, reportar el estado actual
      switch(swing_state) {
        case SWING_FORWARD_PHASE: control_output = SWING_FORWARD_ANGLE; break;
        case SWING_BACKWARD_PHASE: control_output = -SWING_BACKWARD_ANGLE; break;
        default: control_output = 0.0; break;
      }
      break;

    case STATE_CONTROL:
      {
        // CONTROL ACTIVO CON STEPPER (post swing-up)
        if (current_controller == CONTROLLER_PID) {
          control_output = compute_pid_control();
        } else {
          control_output = compute_lqr_control();
        }

        // Aplicar control como corrección de posición
        float target_phi = phi_reference + control_output;
        float phi_error = target_phi - phi;
        
        // Mover stepper solo si el error es significativo
        if (abs(phi_error) > 1.0 && stepper->get_device_state() == INACTIVE) {
          move_stepper_by(phi_error * 0.5); // Movimiento suave
        }
      }
      break;

    case STATE_DIRECT_CONTROL:
      {
        // NUEVO: CONTROL DIRECTO (sin swing-up previo)
        run_direct_control();
        // El control_output ya se calcula dentro de run_direct_control()
        if (current_controller == CONTROLLER_PID) {
          control_output = compute_pid_control();
        } else {
          control_output = compute_lqr_control();
        }
      }
      break;

    default:
      control_output = 0.0;
      break;
  }
}

/******************************************************************************
* Function to set swing-up parameters
*/
void set_swing_parameters(float forward_angle, float backward_angle, float forward_speed, float backward_speed) {
  SWING_FORWARD_ANGLE = forward_angle;
  SWING_BACKWARD_ANGLE = backward_angle;
  SWING_FORWARD_SPEED = forward_speed;
  SWING_BACKWARD_SPEED = backward_speed;
  
  Serial.println("*** PARÁMETROS DE SWING-UP ACTUALIZADOS ***");
  Serial.print("Ángulo de avance: "); Serial.print(SWING_FORWARD_ANGLE, 1); Serial.println("°");
  Serial.print("Ángulo de retroceso: "); Serial.print(SWING_BACKWARD_ANGLE, 1); Serial.println("°");
  Serial.print("Velocidad de avance: "); Serial.print(SWING_FORWARD_SPEED, 0); Serial.println(" steps/sec");
  Serial.print("Velocidad de retroceso: "); Serial.print(SWING_BACKWARD_SPEED, 0); Serial.println(" steps/sec");
}

/******************************************************************************
* NUEVAS FUNCIONES para manejo del swing-up
*/
void set_swing_mode(bool enabled) {
  swing_up_enabled = enabled;
  Serial.print("*** MODO SWING-UP: ");
  Serial.println(enabled ? "HABILITADO" : "DESHABILITADO");
  
  if (enabled) {
    Serial.println("Al iniciar control se ejecutará: Swing-up → Control");
  } else {
    Serial.println("Al iniciar control se ejecutará: Control directo");
    Serial.println("NOTA: Asegúrese de que el péndulo esté cerca de la posición invertida");
  }
}

/******************************************************************************
* Setup and main loop
*/
void setup() {
  // Configure pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(D4, INPUT_PULLUP);
  pinMode(D5, INPUT_PULLUP);

  // Initialize communication
  Serial.begin(BAUD_RATE);
  Serial.println("=== SISTEMA DE CONTROL PÉNDULO - SWING-UP ÚNICO MODIFICADO ===");
  Serial.println("VERSIÓN CON CONTROL DIRECTO OPCIONAL");
  ctrl.init(Serial, CTRL_DEBUG);

  // Configure encoder
  encoder = new RotaryEncoder(ENC_A_PIN, ENC_B_PIN, RotaryEncoder::LatchMode::TWO03);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encoderISR, CHANGE);
  Serial.println("Encoder del péndulo configurado (sensor únicamente)");

  // Initialize stepper
  Serial.println("Inicializando motor stepper...");
  stepper = new L6474(STP_FLAG_IRQ_PIN, STP_STBY_RST_PIN, STP_DIR_PIN, STP_PWM_PIN, STP_SPI_CS_PIN, &dev_spi);
  
  if (stepper->init(&stepper_config_control) != COMPONENT_OK) {
    Serial.println("ERROR: No se pudo inicializar el driver del stepper");
    while(1);
  }

  stepper->attach_flag_irq(&stepperISR);
  stepper->enable_flag_irq();
  stepper->set_home();

  // Configuración inicial para control
  configure_stepper_for_control();
  Serial.println("Motor stepper inicializado correctamente");

  // Initialize timing
  prev_time = millis();
  control_loop_timer = millis();
  last_debug_print = millis();

  Serial.println("=== SISTEMA COMPLETAMENTE INICIALIZADO ===");
  Serial.println("MÉTODOS DE CONTROL DISPONIBLES:");
  Serial.println("1. SWING-UP + CONTROL (Comando 7)");
  Serial.println("   - Ejecuta swing-up automático");
  Serial.println("   - Transición automática al control");
  Serial.println("2. CONTROL DIRECTO (Comando 10)");
  Serial.println("   - Control inmediato sin swing-up");
  Serial.println("   - Para péndulos ya cerca de posición invertida");
  Serial.println("=== ESPERANDO COMANDOS ===");
  
  // Mostrar parámetros actuales
  Serial.println("PARÁMETROS ACTUALES:");
  Serial.print("- Swing-up habilitado: "); Serial.println(swing_up_enabled ? "SÍ" : "NO");
  Serial.print("- Ángulo de avance: "); Serial.print(SWING_FORWARD_ANGLE, 1); Serial.println("°");
  Serial.print("- Ángulo de retroceso: "); Serial.print(SWING_BACKWARD_ANGLE, 1); Serial.println("°");
  Serial.print("- Velocidad de avance: "); Serial.print(SWING_FORWARD_SPEED, 0); Serial.println(" steps/sec");
  Serial.print("- Velocidad de retroceso: "); Serial.print(SWING_BACKWARD_SPEED, 0); Serial.println(" steps/sec");
  Serial.println("");
  Serial.println("COMANDOS NUEVOS:");
  Serial.println("- CMD_START_DIRECT_CONTROL (10): Control directo");
  Serial.println("- CMD_SET_SWING_MODE (11): Habilitar/Deshabilitar swing-up");
}

void loop() {
  int command;
  float action[NUM_ACTIONS];
  float observation[NUM_OBS];
  ControlComms::StatusCode rx_code;

  // Always run control loop if active
  if (system_state == STATE_SWING_UP || system_state == STATE_CONTROL || system_state == STATE_DIRECT_CONTROL) {
    run_control_loop();
  }

  // Handle communication
  rx_code = ctrl.receive_action<NUM_ACTIONS>(&command, action);

  if (rx_code == ControlComms::OK) {
    switch (command) {
      case CMD_SET_HOME:
        set_stepper_home();
        Serial.println("Home establecido");
        break;

      case CMD_MOVE_TO:
        if (system_state == STATE_IDLE) {
          move_stepper_to(action[0]);
          Serial.print("Moviendo stepper a: "); Serial.print(action[0], 1); Serial.println("°");
        }
        break;

      case CMD_MOVE_BY:
        if (system_state == STATE_IDLE) {
          move_stepper_by(action[0]);
          Serial.print("Moviendo stepper por: "); Serial.print(action[0], 1); Serial.println("°");
        }
        break;

      case CMD_SET_STEP_MODE:
        set_step_mode((int)action[0]);
        set_stepper_home();
        Serial.print("Modo de paso establecido: "); Serial.println((int)action[0]);
        break;

      case CMD_SELECT_CONTROLLER:
        current_controller = (ControllerType)((int)action[0]);
        Serial.print("Controlador seleccionado: ");
        Serial.println(current_controller == CONTROLLER_PID ? "PID" : "LQR");
        break;

      case CMD_SET_PID_GAINS:
        K_p = action[0];
        K_i = action[1];
        K_d = action[2];
        Serial.print("Ganancias PID - Kp: "); Serial.print(K_p, 6);
        Serial.print(", Ki: "); Serial.print(K_i, 6);
        Serial.print(", Kd: "); Serial.println(K_d, 6);
        break;

      case CMD_SET_LQR_GAINS:
        for (int i = 0; i < 5; i++) {
          K_lqr[i] = action[i];
        }
        Serial.print("Ganancias LQR: [");
        for (int i = 0; i < 5; i++) {
          Serial.print(K_lqr[i], 6);
          if (i < 4) Serial.print(", ");
        }
        Serial.println("]");
        break;

      case CMD_SET_SWING_PARAMS:
        // Comando para ajustar parámetros de swing-up
        // action[0] = forward_angle, action[1] = backward_angle
        // action[2] = forward_speed, action[3] = backward_speed
        if (action[0] > 0 && action[1] > 0 && action[2] > 0 && action[3] > 0) {
          set_swing_parameters(action[0], action[1], action[2], action[3]);
        } else {
          Serial.println("Error: Todos los parámetros deben ser positivos");
        }
        break;

      case CMD_START_CONTROL:
        if (system_state == STATE_IDLE) {
          // Decidir si usar swing-up o control directo basado en swing_up_enabled
          if (swing_up_enabled) {
            system_state = STATE_SWING_UP;
            reset_swing_up_variables();
            Serial.println("*** CONTROL INICIADO - MODO SWING-UP ***");
          } else {
            start_direct_control();
            Serial.println("*** CONTROL INICIADO - MODO DIRECTO ***");
          }

          // Reset estados del controlador
          pid_integral = 0.0;
          pid_error_prev = 0.0;
          integral_error = 0.0;
        } else {
          Serial.println("Error: Sistema no está en estado IDLE");
        }
        break;

      case CMD_START_DIRECT_CONTROL:
        // NUEVO: Comando explícito para control directo
        if (system_state == STATE_IDLE) {
          start_direct_control();
          Serial.println("*** COMANDO EXPLÍCITO: CONTROL DIRECTO ***");
        } else {
          Serial.println("Error: Sistema no está en estado IDLE");
        }
        break;

      case CMD_SET_SWING_MODE:
        // NUEVO: Comando para habilitar/deshabilitar swing-up
        // action[0] = 1 para habilitar, 0 para deshabilitar
        set_swing_mode((int)action[0] != 0);
        break;

      case CMD_STOP_CONTROL:
        system_state = STATE_IDLE;
        stepper->hard_stop();
        Serial.println("*** CONTROL DETENIDO ***");
        break;

      default:
        Serial.print("Comando desconocido: "); Serial.println(command);
        break;
    }

    // Always update states and send observation
    update_states();

    // Prepare observation
    observation[0] = theta; // Pendulum angle
    observation[1] = phi; // Rotor angle
    observation[2] = dtheta; // Pendulum velocity
    observation[3] = dphi; // Rotor velocity
    observation[4] = (float)swing_state; // Current swing state
    observation[5] = theta_max_achieved; // Maximum angle achieved

    // Determine status - ACTUALIZADO
    int status = STATUS_OK;
    if (system_state == STATE_SWING_UP) {
      switch(swing_state) {
        case SWING_FORWARD_PHASE: status = STATUS_SWING_FORWARD; break;
        case SWING_BACKWARD_PHASE: status = STATUS_SWING_BACKWARD; break;
        default: status = STATUS_SWING_UP; break;
      }
    } else if (system_state == STATE_CONTROL) {
      status = is_pendulum_upright() ? STATUS_UPRIGHT_ACHIEVED : STATUS_CONTROL_ACTIVE;
    } else if (system_state == STATE_DIRECT_CONTROL) {
      status = is_pendulum_upright() ? STATUS_UPRIGHT_ACHIEVED : STATUS_DIRECT_CONTROL;
    } else if (stepper->get_device_state() != INACTIVE) {
      status = STATUS_STP_MOVING;
    }

    // Send observation
    ctrl.send_observation(status, millis(), false, observation, NUM_OBS);
  
  } else if (rx_code == ControlComms::ERROR) {
    // Solo reportar errores críticos ocasionalmente
    static unsigned long last_error_report = 0;
    if (millis() - last_error_report > 10000) { // Cada 10 segundos
      Serial.println("Sistema en espera de comandos...");
      last_error_report = millis();
    }
  }

  // LED de estado
  static unsigned long last_led_update = 0;
  if (millis() - last_led_update > 1000) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    last_led_update = millis();
  }
}