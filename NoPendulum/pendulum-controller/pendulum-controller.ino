/**
* @file pendulum-controller-direct.ino
* @brief Controlador de péndulo invertido - Solo control directo (sin swing-up)
* @author Versión modificada para control directo únicamente
* @date 2025-08-26
*
* @details
* MODIFICACIONES REALIZADAS:
* 1. Eliminada toda la funcionalidad de swing-up
* 2. Mantenido solo control PID/LQR directo
* 3. Simplificado el código eliminando estados de swing-up
* 4. Conservadas todas las funciones básicas de control
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
static const unsigned int STATUS_CONTROL_ACTIVE = 2;
static const unsigned int STATUS_UPRIGHT_ACHIEVED = 3;

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

// Controller selection
typedef enum {
  CONTROLLER_PID = 0,
  CONTROLLER_LQR = 1
} ControllerType;

// System states
typedef enum {
  STATE_IDLE = 0,
  STATE_CONTROL = 1,
  STATE_STOPPED = 2
} SystemState;

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

// Reference for control
float phi_reference = 0.0;

// Timing variables
unsigned long current_time = 0;
unsigned long prev_time = 0;
unsigned long control_loop_timer = 0;
const unsigned long CONTROL_INTERVAL = 4; // 4ms = 250Hz

// Debug variables
unsigned long last_debug_print = 0;
const unsigned long DEBUG_INTERVAL = 500; // Print debug cada 500ms

// Stepper configuration for control (precision)
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

  if (system_state == STATE_CONTROL) {
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

    // Debug periodic
    if (current_time - last_debug_print > DEBUG_INTERVAL) {
      Serial.print("CONTROL θ:");
      Serial.print(theta, 1);
      Serial.print("° φ:");
      Serial.print(phi, 1);
      Serial.print("° u:");
      Serial.print(control_output, 2);
      Serial.print(" upright:");
      Serial.println(is_pendulum_upright() ? "YES" : "NO");
      last_debug_print = current_time;
    }
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
  Serial.println("=== CONTROL PÉNDULO DIRECTO (SIN SWING-UP) ===");
  ctrl.init(Serial, CTRL_DEBUG);

  // Configure encoder
  encoder = new RotaryEncoder(ENC_A_PIN, ENC_B_PIN, RotaryEncoder::LatchMode::TWO03);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encoderISR, CHANGE);
  Serial.println("Encoder del péndulo configurado");

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

  Serial.println("Motor stepper configurado para control de precisión");

  // Initialize timing
  prev_time = millis();
  control_loop_timer = millis();
  last_debug_print = millis();

  Serial.println("=== SISTEMA INICIALIZADO ===");
  Serial.println("MODO: Control directo únicamente");
  Serial.println("Para iniciar control, posicione el péndulo cerca de 180° y use el comando START_CONTROL");
  Serial.println("=== ESPERANDO COMANDOS ===");
}

void loop() {
  int command;
  float action[NUM_ACTIONS];
  float observation[NUM_OBS];
  ControlComms::StatusCode rx_code;

  // Always run control loop if active
  if (system_state == STATE_CONTROL) {
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
          system_state = STATE_CONTROL;
          phi_reference = phi; // Usar posición actual como referencia

          // Reset estados del controlador
          pid_integral = 0.0;
          pid_error_prev = 0.0;
          integral_error = 0.0;

          Serial.println("*** CONTROL DIRECTO INICIADO ***");
          Serial.print("Controlador: ");
          Serial.println(current_controller == CONTROLLER_PID ? "PID" : "LQR");
          Serial.print("Posición inicial θ: "); Serial.print(theta, 1); Serial.println("°");
          Serial.print("Referencia φ: "); Serial.print(phi_reference, 1); Serial.println("°");
          Serial.println("POSICIONE EL PÉNDULO CERCA DE 180° PARA MEJOR CONTROL");
        } else {
          Serial.println("Error: Sistema no está en estado IDLE");
        }
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
    observation[4] = current_controller; // Current controller type
    observation[5] = integral_error; // Integral error

    // Determine status
    int status = STATUS_OK;
    if (system_state == STATE_CONTROL) {
      status = is_pendulum_upright() ? STATUS_UPRIGHT_ACHIEVED : STATUS_CONTROL_ACTIVE;
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