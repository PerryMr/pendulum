/**
* @file pendulum-controller-stepper-swingup-improved.ino
* @brief Controlador de péndulo invertido - Swing-up MEJORADO con oscilaciones graduales
* @author Versión mejorada con swing-up controlado por oscilaciones
* @date 2025-08-22
*
* @details
* MEJORAS PRINCIPALES IMPLEMENTADAS:
* 1. Swing-up por oscilaciones graduales (5-6 oscilaciones programadas)
* 2. Incremento progresivo y controlado de la energía del péndulo
* 3. Velocidad del motor más suave y consistente
* 4. Mejor sincronización entre impulsos
* 5. Control de timing mejorado para evitar movimientos bruscos
* 6. Análisis en tiempo real del progreso del swing-up
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

// Command definitions
static const unsigned int CMD_SET_HOME = 0;
static const unsigned int CMD_MOVE_TO = 1;
static const unsigned int CMD_MOVE_BY = 2;
static const unsigned int CMD_SET_STEP_MODE = 3;
static const unsigned int CMD_SELECT_CONTROLLER = 4;
static const unsigned int CMD_SET_PID_GAINS = 5;
static const unsigned int CMD_SET_LQR_GAINS = 6;
static const unsigned int CMD_START_CONTROL = 7;
static const unsigned int CMD_STOP_CONTROL = 8;

// Status codes
static const unsigned int STATUS_OK = 0;
static const unsigned int STATUS_STP_MOVING = 1;
static const unsigned int STATUS_SWING_UP = 2;
static const unsigned int STATUS_CONTROL_ACTIVE = 3;
static const unsigned int STATUS_UPRIGHT_ACHIEVED = 4;

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

// *** PARÁMETROS DE SWING-UP MEJORADOS PARA OSCILACIONES GRADUALES ***
const int SWING_PLANNED_OSCILLATIONS = 6; // Número de oscilaciones planificadas
const float SWING_OSCILLATION_TIME = 2.5; // Duración de cada oscilación (segundos)
const float SWING_INITIAL_ANGLE = 25.0; // Ángulo inicial del primer impulso
const float SWING_ANGLE_INCREMENT = 8.0; // Incremento por oscilación
const float SWING_MAX_ANGLE = 65.0; // Ángulo máximo del impulso
const float SWING_BASE_SPEED = 800.0; // Velocidad base del motor (steps/sec)
const float SWING_SPEED_INCREMENT = 200.0; // Incremento de velocidad por oscilación
const float SWING_MAX_SPEED = 2000.0; // Velocidad máxima del motor

// Parámetros de timing mejorado
const float SWING_IMPULSE_DURATION = 0.6; // Duración de cada impulso (segundos)
const float SWING_WAIT_AFTER_IMPULSE = 0.4; // Espera después del impulso
const float SWING_SETTLE_TIME = 1.5; // Tiempo de asentamiento entre oscilaciones
const float SWING_MAX_TOTAL_TIME = 20.0; // Tiempo máximo total (segundos)

// Parámetros de análisis de progreso
const float SWING_ENERGY_TARGET = 45.0; // Energía objetivo
const float SWING_MIN_PROGRESS_ANGLE = 5.0; // Progreso mínimo por oscilación
const float SWING_EFFICIENCY_THRESHOLD = 0.7; // Umbral de eficiencia

// Variables globales adicionales para configuración dinámica
bool swing_up_mode = false;
int swing_up_div_per_step = 8; // Micropasos para swing-up

// Controller selection
typedef enum {
  CONTROLLER_PID = 0,
  CONTROLLER_LQR = 1
} ControllerType;

// System states
typedef enum {
  STATE_IDLE = 0,
  STATE_SWING_UP = 1,
  STATE_CONTROL = 2,
  STATE_STOPPED = 3
} SystemState;

// Estados mejorados para swing-up por oscilaciones
typedef enum {
  SWING_INIT = 0,           // Inicialización
  SWING_IMPULSE_OUT = 1,    // Impulso hacia afuera
  SWING_WAIT_OUT = 2,       // Espera después del impulso hacia afuera  
  SWING_IMPULSE_BACK = 3,   // Impulso de regreso
  SWING_WAIT_BACK = 4,      // Espera después del impulso de regreso
  SWING_SETTLE = 5,         // Tiempo de asentamiento entre oscilaciones
  SWING_EVALUATE = 6        // Evaluación del progreso
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

// Variables mejoradas para swing-up por oscilaciones
float phi_reference = 0.0; // Referencia del stepper cuando se alcanza 180°
unsigned long swing_start_time = 0;
unsigned long swing_phase_start_time = 0;
unsigned long swing_oscillation_start_time = 0;
int swing_current_oscillation = 0; // Oscilación actual (0 a SWING_PLANNED_OSCILLATIONS-1)
bool upright_achieved = false;
SwingState swing_state = SWING_INIT;
bool swing_direction_out = true; // Dirección del impulso actual

// Variables para control progresivo
float current_swing_angle = SWING_INITIAL_ANGLE;
float current_swing_speed = SWING_BASE_SPEED;
float theta_max_this_oscillation = 0.0;
float theta_max_overall = 0.0;
float theta_initial = 0.0; // Ángulo inicial al comenzar el swing-up

// Energy estimation variables
float pendulum_energy = 0.0;
float previous_energy = 0.0;
float energy_progress = 0.0;

// Variables de análisis de progreso
float oscillation_efficiency = 0.0;
bool oscillation_successful = false;
int successful_oscillations = 0;

// Timing variables
unsigned long current_time = 0;
unsigned long prev_time = 0;
unsigned long control_loop_timer = 0;
const unsigned long CONTROL_INTERVAL = 4; // 4ms = 250Hz

// Debug variables
unsigned long last_debug_print = 0;
const unsigned long DEBUG_INTERVAL = 500; // Print debug cada 500ms

// Configuraciones del motor optimizadas
L6474_init_t stepper_config_swing_up = {
  12000,    // acc: Aceleración moderada para swing-up suave
  12000,    // dec: Deceleración moderada
  1500,     // max_speed: Velocidad inicial moderada
  800,      // min_speed: Velocidad mínima
  650,      // tval: Corriente alta pero controlada
  L6474_OCD_TH_3375mA, // Umbral de sobrecorriente alto pero seguro
  L6474_CONFIG_OC_SD_ENABLE,
  L6474_CONFIG_EN_TQREG_TVAL_USED,
  L6474_STEP_SEL_1_8, // Micropasos para balance entre torque y precisión
  L6474_SYNC_SEL_1_2,
  L6474_FAST_STEP_10us, // Timing más suave
  L6474_TOFF_FAST_6us,
  4,
  22,
  L6474_CONFIG_TOFF_036us,
  L6474_CONFIG_SR_320V_us,
  L6474_CONFIG_INT_16MHZ,
  L6474_ALARM_EN_OVERCURRENT | L6474_ALARM_EN_THERMAL_SHUTDOWN |
  L6474_ALARM_EN_THERMAL_WARNING | L6474_ALARM_EN_UNDERVOLTAGE |
  L6474_ALARM_EN_SW_TURN_ON | L6474_ALARM_EN_WRONG_NPERF_CMD
};

L6474_init_t stepper_config_control = {
  10000,    // acc: Aceleración normal
  10000,    // dec: Deceleración normal
  3000,     // max_speed: Velocidad moderada para control fino
  1000,     // min_speed: Velocidad mínima
  400,      // tval: Corriente moderada para control
  L6474_OCD_TH_1500mA, // Umbral de sobrecorriente normal
  L6474_CONFIG_OC_SD_ENABLE,
  L6474_CONFIG_EN_TQREG_TVAL_USED,
  L6474_STEP_SEL_1_16, // Micropasos para precisión en control
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

// Nueva función para movimientos suaves del swing-up
void move_stepper_smooth_swing(float deg, float speed) {
  // Configurar velocidad de manera suave
  stepper->set_max_speed(speed);
  stepper->set_acceleration(8000);  // Aceleración alta pero controlada
  stepper->set_deceleration(10000); // Deceleración ligeramente mayor para paradas suaves
  move_stepper_by(deg);
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

// Función auxiliar para min con casting explícito
float safe_min(float a, float b) {
  return (a < b) ? a : b;
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

  // Calculate derivatives with smoothing
  dphi = (phi - phi_prev) / dt;
  dtheta = (theta - theta_prev) / dt;

  // Limit velocities to avoid noise spikes
  if (abs(dphi) > 1500.0) dphi = 0.0;
  if (abs(dtheta) > 1500.0) dtheta = 0.0;

  // Calculate pendulum energy for swing-up control
  float theta_rad = theta * PI / 180.0;
  float dtheta_rad = dtheta * PI / 180.0;
  previous_energy = pendulum_energy;
  pendulum_energy = 0.5 * dtheta_rad * dtheta_rad + sigma * (1.0 - cos(theta_rad));
  
  // Calcular progreso de energía
  energy_progress = pendulum_energy - previous_energy;

  // Update integral error for control
  float error = SETPOINT - theta;
  integral_error += error * dt;

  // Limit integral windup
  if (integral_error > 100.0) integral_error = 100.0;
  if (integral_error < -100.0) integral_error = -100.0;

  // Track maximum angle achieved
  if (system_state == STATE_SWING_UP) {
    float angle_from_bottom = theta;
    if (angle_from_bottom > 180.0) angle_from_bottom = 360.0 - angle_from_bottom;
    
    // Máximo de esta oscilación
    if (angle_from_bottom > theta_max_this_oscillation) {
      theta_max_this_oscillation = angle_from_bottom;
    }
    
    // Máximo general
    if (angle_from_bottom > theta_max_overall) {
      theta_max_overall = angle_from_bottom;
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
  Serial.println("=== CONFIGURANDO MOTOR PARA SWING-UP SUAVE ===");
  
  // Aplicar configuración optimizada para swing-up suave
  stepper->set_parameter(L6474_TVAL, 650);
  stepper->set_parameter(L6474_OCD_TH, L6474_OCD_TH_3375mA);
  stepper->set_parameter(L6474_TON_MIN, 4);
  stepper->set_parameter(L6474_TOFF_MIN, 22);
  
  // Configurar micropasos para balance torque/suavidad
  stepper->set_step_mode(StepperMotor::STEP_MODE_1_8);
  div_per_step = swing_up_div_per_step;
  
  // Configurar velocidades iniciales suaves
  stepper->set_max_speed(current_swing_speed);
  stepper->set_acceleration(8000);
  stepper->set_deceleration(10000);
  
  swing_up_mode = true;
  
  Serial.println("Motor configurado para swing-up suave y controlado");
}

void configure_stepper_for_control() {
  Serial.println("=== CONFIGURANDO MOTOR PARA CONTROL DE PRECISIÓN ===");
  
  // Aplicar configuración de control directamente con valores
  stepper->set_parameter(L6474_TVAL, 400);
  stepper->set_parameter(L6474_OCD_TH, L6474_OCD_TH_1500mA);
  stepper->set_parameter(L6474_TON_MIN, 3);
  stepper->set_parameter(L6474_TOFF_MIN, 21);
  
  // Configurar micropasos para precisión
  stepper->set_step_mode(StepperMotor::STEP_MODE_1_16);
  div_per_step = 16;
  
  // Configurar velocidades moderadas
  stepper->set_max_speed(200);
  stepper->set_acceleration(500);
  stepper->set_deceleration(500);
  
  swing_up_mode = false;
  
  Serial.println("Motor configurado para control de precisión");
}

/******************************************************************************
* SWING-UP ALGORITHM MEJORADO CON OSCILACIONES GRADUALES
*/
void reset_swing_up_variables() {
  swing_current_oscillation = 0;
  swing_start_time = millis();
  swing_phase_start_time = millis();
  swing_oscillation_start_time = millis();
  upright_achieved = false;
  swing_state = SWING_INIT;
  swing_direction_out = true;
  
  // Reset parámetros progresivos
  current_swing_angle = SWING_INITIAL_ANGLE;
  current_swing_speed = SWING_BASE_SPEED;
  theta_max_this_oscillation = 0.0;
  theta_max_overall = 0.0;
  theta_initial = theta;
  
  // Reset análisis
  oscillation_efficiency = 0.0;
  oscillation_successful = false;
  successful_oscillations = 0;
  
  // Configurar motor para swing-up suave
  configure_stepper_for_swing_up();

  Serial.println("=== SWING-UP MEJORADO CON OSCILACIONES GRADUALES ===");
  Serial.print("Ángulo inicial péndulo: "); Serial.print(theta, 1); Serial.println("°");
  Serial.print("Posición inicial stepper: "); Serial.print(phi, 1); Serial.println("°");
  Serial.println("CONFIGURACIÓN OPTIMIZADA:");
  Serial.print("- Oscilaciones planificadas: "); Serial.println(SWING_PLANNED_OSCILLATIONS);
  Serial.print("- Ángulo inicial de impulso: ±"); Serial.print(current_swing_angle, 1); Serial.println("°");
  Serial.print("- Velocidad inicial: "); Serial.print(current_swing_speed, 1); Serial.println(" steps/sec");
  Serial.print("- Incremento por oscilación: "); Serial.print(SWING_ANGLE_INCREMENT, 1); Serial.println("°");
  Serial.print("- Duración por oscilación: "); Serial.print(SWING_OSCILLATION_TIME, 1); Serial.println(" s");
}

void run_swing_up_improved() {
  float elapsed_total = (current_time - swing_start_time) / 1000.0;
  float elapsed_phase = (current_time - swing_phase_start_time) / 1000.0;
  float elapsed_oscillation = (current_time - swing_oscillation_start_time) / 1000.0;

  // Debug periódico más informativo
  if (current_time - last_debug_print > DEBUG_INTERVAL) {
    Serial.print("SWING [T:"); Serial.print(elapsed_total, 1);
    Serial.print("s] Osc:"); Serial.print(swing_current_oscillation + 1);
    Serial.print("/"); Serial.print(SWING_PLANNED_OSCILLATIONS);
    Serial.print(" θ:"); Serial.print(theta, 1);
    Serial.print("° θ_max:"); Serial.print(theta_max_overall, 1);
    Serial.print("° E:"); Serial.print(pendulum_energy, 2);
    Serial.print(" Estado:");
    
    switch(swing_state) {
      case SWING_INIT: Serial.print("INIT"); break;
      case SWING_IMPULSE_OUT: Serial.print("IMPULSO→"); break;
      case SWING_WAIT_OUT: Serial.print("ESPERA→"); break;
      case SWING_IMPULSE_BACK: Serial.print("IMPULSO←"); break;
      case SWING_WAIT_BACK: Serial.print("ESPERA←"); break;
      case SWING_SETTLE: Serial.print("ASENTAR"); break;
      case SWING_EVALUATE: Serial.print("EVALUAR"); break;
    }
    
    Serial.print(" Ángulo:"); Serial.print(current_swing_angle, 1);
    Serial.print("° Vel:"); Serial.print(current_swing_speed, 0);
    Serial.print(" Motor:"); Serial.print(stepper->get_device_state() == INACTIVE ? "PARADO" : "MOVIENDO");
    Serial.println();
    last_debug_print = current_time;
  }

  // VERIFICAR POSICIÓN INVERTIDA
  if (is_pendulum_upright()) {
    if (!upright_achieved) {
      stepper->hard_stop();
      phi_reference = phi;
      upright_achieved = true;
      Serial.println("*** ¡POSICIÓN INVERTIDA ALCANZADA CON ÉXITO! ***");
      Serial.print("Tiempo total: "); Serial.print(elapsed_total, 1); Serial.println("s");
      Serial.print("Oscilaciones completadas: "); Serial.println(swing_current_oscillation);
      Serial.print("Oscilaciones exitosas: "); Serial.println(successful_oscillations);
      Serial.print("Ángulo máximo alcanzado: "); Serial.print(theta_max_overall, 1); Serial.println("°");
      Serial.print("Energía final: "); Serial.println(pendulum_energy, 2);
      
      // TRANSICIÓN AL CONTROL
      system_state = STATE_CONTROL;
      configure_stepper_for_control();

      // Reset estados del controlador
      pid_integral = 0.0;
      pid_error_prev = 0.0;
      integral_error = 0.0;

      Serial.println("=== TRANSICIÓN A CONTROL DE ESTABILIZACIÓN ===");
    }
    return;
  }

  // Verificar timeout
  if (elapsed_total > SWING_MAX_TOTAL_TIME || swing_current_oscillation >= SWING_PLANNED_OSCILLATIONS) {
    Serial.println("*** ANÁLISIS FINAL DEL SWING-UP ***");
    Serial.print("Tiempo total: "); Serial.print(elapsed_total, 1); Serial.println("s");
    Serial.print("Oscilaciones completadas: "); Serial.print(swing_current_oscillation); 
    Serial.print("/"); Serial.println(SWING_PLANNED_OSCILLATIONS);
    Serial.print("Oscilaciones exitosas: "); Serial.println(successful_oscillations);
    Serial.print("Ángulo máximo alcanzado: "); Serial.print(theta_max_overall, 1); Serial.println("°");
    Serial.print("Energía final: "); Serial.println(pendulum_energy, 2);
    Serial.print("Eficiencia promedio: "); Serial.print(oscillation_efficiency, 2); Serial.println("%");

    if (theta_max_overall < 90.0) {
      Serial.println("DIAGNÓSTICO: Progreso insuficiente - Verificar conexiones mecánicas");
      Serial.println("SUGERENCIA: Aumentar corriente del motor o reducir fricción");
    } else if (theta_max_overall < 150.0) {
      Serial.println("DIAGNÓSTICO: Progreso moderado - Considerar más oscilaciones");
      Serial.println("SUGERENCIA: Aumentar ángulo de impulso o velocidad");
    } else {
      Serial.println("DIAGNÓSTICO: Excelente progreso - Muy cerca del objetivo");
      Serial.println("SUGERENCIA: Ajustar timing o probar control directo");
    }

    stepper->hard_stop();
    phi_reference = phi;
    system_state = STATE_CONTROL;
    configure_stepper_for_control();
    return;
  }

  // MÁQUINA DE ESTADOS MEJORADA PARA OSCILACIONES GRADUALES
  switch(swing_state) {
    case SWING_INIT:
      Serial.print("=== INICIANDO OSCILACIÓN "); Serial.print(swing_current_oscillation + 1);
      Serial.print(" DE "); Serial.print(SWING_PLANNED_OSCILLATIONS); Serial.println(" ===");
      Serial.print("Ángulo de impulso: ±"); Serial.print(current_swing_angle, 1); Serial.println("°");
      Serial.print("Velocidad del motor: "); Serial.print(current_swing_speed, 0); Serial.println(" steps/sec");
      
      theta_max_this_oscillation = 0.0; // Reset para esta oscilación
      swing_oscillation_start_time = current_time;
      swing_state = SWING_IMPULSE_OUT;
      swing_phase_start_time = current_time;
      break;

    case SWING_IMPULSE_OUT:
      if (stepper->get_device_state() == INACTIVE) {
        // Impulso hacia afuera (dirección positiva)
        float impulse_angle = swing_direction_out ? current_swing_angle : -current_swing_angle;
        move_stepper_smooth_swing(impulse_angle, current_swing_speed);
        
        Serial.print("IMPULSO HACIA AFUERA: "); Serial.print(impulse_angle, 1);
        Serial.print("° a "); Serial.print(current_swing_speed, 0); Serial.println(" steps/sec");
        
        swing_state = SWING_WAIT_OUT;
        swing_phase_start_time = current_time;
      }
      break;

    case SWING_WAIT_OUT:
      if (elapsed_phase >= SWING_IMPULSE_DURATION) {
        stepper->hard_stop(); // Asegurar parada
        swing_state = SWING_IMPULSE_BACK;
        swing_phase_start_time = current_time;
      }
      break;

    case SWING_IMPULSE_BACK:
      if (stepper->get_device_state() == INACTIVE && elapsed_phase >= SWING_WAIT_AFTER_IMPULSE) {
        // Impulso de regreso (dirección opuesta)
        float impulse_angle = swing_direction_out ? -current_swing_angle : current_swing_angle;
        move_stepper_smooth_swing(impulse_angle, current_swing_speed);
        
        Serial.print("IMPULSO DE REGRESO: "); Serial.print(impulse_angle, 1);
        Serial.print("° a "); Serial.print(current_swing_speed, 0); Serial.println(" steps/sec");
        
        swing_state = SWING_WAIT_BACK;
        swing_phase_start_time = current_time;
      }
      break;

    case SWING_WAIT_BACK:
      if (elapsed_phase >= SWING_IMPULSE_DURATION) {
        stepper->hard_stop(); // Asegurar parada
        swing_state = SWING_SETTLE;
        swing_phase_start_time = current_time;
      }
      break;

    case SWING_SETTLE:
      // Tiempo de asentamiento entre oscilaciones
      if (elapsed_phase >= SWING_SETTLE_TIME) {
        swing_state = SWING_EVALUATE;
      }
      break;

    case SWING_EVALUATE:
      // Evaluar el progreso de esta oscilación
      float angle_progress = theta_max_this_oscillation - (theta_initial > 90 ? 360 - theta_initial : theta_initial);
      
      if (angle_progress >= SWING_MIN_PROGRESS_ANGLE) {
        oscillation_successful = true;
        successful_oscillations++;
      } else {
        oscillation_successful = false;
      }
      
      // Calcular eficiencia de esta oscilación
      float expected_progress = SWING_MIN_PROGRESS_ANGLE * (swing_current_oscillation + 1);
      oscillation_efficiency = (theta_max_overall / expected_progress) * 100.0;
      
      Serial.print("EVALUACIÓN OSCILACIÓN "); Serial.print(swing_current_oscillation + 1); Serial.println(":");
      Serial.print("- Ángulo máximo alcanzado: "); Serial.print(theta_max_this_oscillation, 1); Serial.println("°");
      Serial.print("- Progreso en esta oscilación: "); Serial.print(angle_progress, 1); Serial.println("°");
      Serial.print("- Exitosa: "); Serial.println(oscillation_successful ? "SÍ" : "NO");
      Serial.print("- Eficiencia acumulada: "); Serial.print(oscillation_efficiency, 1); Serial.println("%");
      Serial.print("- Energía alcanzada: "); Serial.print(pendulum_energy, 2); 
      Serial.print("/"); Serial.println(SWING_ENERGY_TARGET, 2);
      
      // AJUSTE ADAPTATIVO DE PARÁMETROS
      if (oscillation_successful) {
        // Si fue exitosa, incremento moderado
        if (swing_current_oscillation < SWING_PLANNED_OSCILLATIONS - 2) {
          current_swing_angle = safe_min(current_swing_angle + SWING_ANGLE_INCREMENT, SWING_MAX_ANGLE);
          current_swing_speed = safe_min(current_swing_speed + SWING_SPEED_INCREMENT, SWING_MAX_SPEED);
        }
        Serial.println("→ Oscilación exitosa: incremento moderado de parámetros");
      } else {
        // Si no fue exitosa, incremento más agresivo
        current_swing_angle = safe_min(current_swing_angle + SWING_ANGLE_INCREMENT * 1.5, SWING_MAX_ANGLE);
        current_swing_speed = safe_min(current_swing_speed + SWING_SPEED_INCREMENT * 1.3, SWING_MAX_SPEED);
        Serial.println("→ Oscilación no exitosa: incremento agresivo de parámetros");
      }
      
      // Actualizar configuración del motor con nuevos parámetros
      stepper->set_max_speed(current_swing_speed);
      
      Serial.print("→ Nuevos parámetros para siguiente oscilación: Ángulo=±");
      Serial.print(current_swing_angle, 1); Serial.print("°, Velocidad=");
      Serial.print(current_swing_speed, 0); Serial.println(" steps/sec");
      
      // Continuar con siguiente oscilación
      swing_current_oscillation++;
      
      if (swing_current_oscillation < SWING_PLANNED_OSCILLATIONS) {
        // Alternar dirección para la siguiente oscilación
        swing_direction_out = !swing_direction_out;
        swing_state = SWING_INIT;
      } else {
        // Se completaron todas las oscilaciones planificadas
        Serial.println("=== TODAS LAS OSCILACIONES COMPLETADAS ===");
        Serial.print("Resultado final: "); Serial.print(theta_max_overall, 1); Serial.println("° máximo");
        
        if (theta_max_overall >= 170.0) {
          Serial.println("¡Muy cerca del objetivo! Intentando transición al control...");
          system_state = STATE_CONTROL;
          configure_stepper_for_control();
          phi_reference = phi;
        } else {
          Serial.println("No se alcanzó el objetivo. Deteniendo swing-up.");
          system_state = STATE_IDLE;
        }
      }
      break;
  }
}

/******************************************************************************
* Main control state machine
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
      run_swing_up_improved(); // Usar algoritmo mejorado
      control_output = current_swing_angle * (swing_direction_out ? 1.0 : -1.0); // Para monitoreo
      break;

    case STATE_CONTROL:
      {
        // CONTROL ACTIVO CON STEPPER
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

    default:
      control_output = 0.0;
      break;
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
  Serial.println("=== SISTEMA DE CONTROL PÉNDULO - SWING-UP MEJORADO ===");
  ctrl.init(Serial, CTRL_DEBUG);

  // Configure encoder
  encoder = new RotaryEncoder(ENC_A_PIN, ENC_B_PIN, RotaryEncoder::LatchMode::TWO03);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encoderISR, CHANGE);
  Serial.println("Encoder del péndulo configurado (sensor únicamente)");

  // Initialize stepper
  Serial.println("Inicializando motor stepper (swing-up suave y control)...");
  stepper = new L6474(STP_FLAG_IRQ_PIN, STP_STBY_RST_PIN, STP_DIR_PIN, STP_PWM_PIN, STP_SPI_CS_PIN, &dev_spi);
  
  if (stepper->init(&stepper_config_control) != COMPONENT_OK) {
    Serial.println("ERROR: No se pudo inicializar el driver del stepper");
    while(1);
  }

  stepper->attach_flag_irq(&stepperISR);
  stepper->enable_flag_irq();
  stepper->set_home();

  // Configuración inicial moderada
  configure_stepper_for_control();
  Serial.println("Motor stepper inicializado correctamente");

  // Initialize timing
  prev_time = millis();
  control_loop_timer = millis();
  last_debug_print = millis();

  Serial.println("=== SISTEMA COMPLETAMENTE INICIALIZADO ===");
  Serial.println("Controlador de Péndulo Invertido - Swing-up Mejorado con Oscilaciones Graduales");
  Serial.println("CARACTERÍSTICAS DEL NUEVO ALGORITMO:");
  Serial.println("• Swing-up por oscilaciones planificadas (6 oscilaciones)");
  Serial.println("• Incremento progresivo de ángulo y velocidad");
  Serial.println("• Análisis en tiempo real del progreso");
  Serial.println("• Ajuste adaptativo de parámetros");
  Serial.println("• Movimientos suaves y controlados");
  Serial.println("• Transición automática al control");
  Serial.println("Encoder: Sensor de ángulo del péndulo");
  Serial.println("Stepper: Swing-up suave (oscilaciones graduales) y Control (movimientos finos)");
  Serial.println("=== ESPERANDO COMANDOS ===");
}

void loop() {
  int command;
  float action[NUM_ACTIONS];
  float observation[NUM_OBS];
  ControlComms::StatusCode rx_code;

  // Always run control loop if active
  if (system_state == STATE_SWING_UP || system_state == STATE_CONTROL) {
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

      case CMD_START_CONTROL:
        if (system_state == STATE_IDLE) {
          system_state = STATE_SWING_UP;
          reset_swing_up_variables(); // Inicializar swing-up mejorado

          // Reset estados del controlador
          pid_integral = 0.0;
          pid_error_prev = 0.0;
          integral_error = 0.0;

          Serial.println("*** CONTROL INICIADO CON SWING-UP MEJORADO ***");
          Serial.println("FASE 1: Swing-up por oscilaciones graduales y controladas");
          Serial.println("FASE 2: Control automático de precisión al alcanzar 180°");
          Serial.println("MEJORAS IMPLEMENTADAS:");
          Serial.println("- Oscilaciones planificadas con incremento progresivo");
          Serial.println("- Movimientos suaves sin impulsos bruscos");
          Serial.println("- Análisis en tiempo real del progreso");
          Serial.println("- Ajuste adaptativo de parámetros");
          Serial.println("- Mejor sincronización de timing");
          Serial.println("- Transición suave al control de estabilización");
        } else {
          Serial.println("Error: Sistema no está en estado IDLE");
        }
        break;

      case CMD_STOP_CONTROL:
        system_state = STATE_IDLE;
        stepper->hard_stop(); // Detener stepper inmediatamente
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
    observation[4] = (system_state == STATE_SWING_UP) ? current_swing_angle : 0.0; // Current swing angle
    observation[5] = (system_state == STATE_SWING_UP) ? pendulum_energy : integral_error; // Energy or control error

    // Determine status
    int status = STATUS_OK;
    if (system_state == STATE_SWING_UP) {
      status = STATUS_SWING_UP;
    } else if (system_state == STATE_CONTROL) {
      status = is_pendulum_upright() ? STATUS_UPRIGHT_ACHIEVED : STATUS_CONTROL_ACTIVE;
    } else if (stepper->get_device_state() != INACTIVE) {
      status = STATUS_STP_MOVING;
    }

    // Send observation
    ctrl.send_observation(status, millis(), false, observation, NUM_OBS);
  
  } else if (rx_code == ControlComms::ERROR) {
    // Solo reportar errores críticos, no timeouts normales
    if (millis() % 10000 == 0) { // Cada 10 segundos
      Serial.println("Estado del sistema: Esperando comandos...");
    }
  }

  // LED de estado para debugging visual
  static unsigned long last_led_update = 0;
  if (millis() - last_led_update > 1000) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle LED cada segundo
    last_led_update = millis();
  }
}