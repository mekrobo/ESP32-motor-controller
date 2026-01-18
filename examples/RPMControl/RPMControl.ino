#include <Arduino.h>
#include <ESP32Encoder.h>
#include "config.h"
#include "motor.h"
#include "PID.h"

ESP32Encoder motor1_encoder, motor2_encoder; // Encoder objects for motor 1 and motor 2
// Motor Controller objects
MotorController motor1(Channel_1); // Motor Controller object for channel 1
MotorController motor2(Channel_2); // Motor Controller object for channel 2

TaskHandle_t SensorReadHandle = NULL; // Sensor Reading Task Handle

float TICKS2RPM_1 = 60000.0 / (COUNTS_PER_REV_1 * SENSOR_PERIOD_MS); // RPM calculation factor for motor 1
float TICKS2RPM_2 = 60000.0 / (COUNTS_PER_REV_2 * SENSOR_PERIOD_MS); // RPM calculation factor for motor 2
float VALUE2AMP = (3.3 * 1.7483) / 4095.0; // Assuming a 12-bit ADC and 3.3V reference

rpm des_rpm = {0.0,0.0}, mes_rpm = {0.0, 0.0}; // Desired and Measured RPM structures
current mes_current;             // Measured current structure
int64_t last_pulse_count_m1 = 0; // store previous pulse count for motor 1
int64_t last_pulse_count_m2 = 0; // store previous pulse count for motor 2
int64_t pulse_count_m1 = 0;      // current pulse count for motor 1
int64_t pulse_count_m2 = 0;      // current pulse count for motor 2
char data_sensor[64];            // serial print buffer

int ctrl_pwm1 = 0;  // control input PWM for motor 1
int ctrl_pwm2 = 0;  // control input PWM for motor 2

// PID Parameters for Motor 1
pid_param pid_param_m1 = {
    .kff = Kff_1,
    .fr = Kfr_1,
    .kp = KP_1,
    .ki = KI_1,
    .kd = KD_1,
    .pwm_max = PWM_MAX,
    .integral_max = 100,
    .integral_error = 0,
    .prev_error = 0,
};
// PID Parameters for Motor 2
pid_param pid_param_m2 = {
    .kff = Kff_2,
    .fr = Kfr_2,
    .kp = KP_2,
    .ki = KI_2,
    .kd = KD_2,
    .pwm_max = PWM_MAX,
    .integral_max = 100,
    .integral_error = 0,
    .prev_error = 0,
};

/**
 * @brief Sensor reading in a periodic interval of SENSOR_PERIOD_MS (miliseconds)
 * 
 * @param parameter Empty
 */
void sensor_read(void *parameter) {
  const TickType_t xDelay = pdMS_TO_TICKS(SENSOR_PERIOD_MS); // read at every SENSOR_PERIOD_MS
  TickType_t xLastWakeTime = xTaskGetTickCount();
  int i = 0;
  for (;;) 
  {
    // Read sensors here
    pulse_count_m1 = motor1_encoder.getCount();
    pulse_count_m2 = motor2_encoder.getCount();
    mes_rpm.motor1 = (pulse_count_m1 - last_pulse_count_m1) * TICKS2RPM_1; // RPM calculation channel 1
    mes_rpm.motor2 = (pulse_count_m2 - last_pulse_count_m2) * TICKS2RPM_2; // RPM calculation channel 2
    last_pulse_count_m1 = pulse_count_m1;
    last_pulse_count_m2 = pulse_count_m2;
    mes_current.motor1 = analogRead(MOTOR1_C) * VALUE2AMP; // Assuming a 12-bit ADC and 3.3V reference
    mes_current.motor2 = analogRead(MOTOR2_C) * VALUE2AMP; // Assuming a 12-bit ADC and 3.3V reference
    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}
/** PID loop callback for RPM control with CONTROL_PERIOD_MS interval
 * 
 * @param args desired rpm structure pointer
 */
static void pid_loop_cb(void *args)
{
    rpm *req_rpm = (rpm *)args;
    ctrl_pwm1 = pid_compute(&pid_param_m1, req_rpm->motor1, mes_rpm.motor1);
    ctrl_pwm2 = pid_compute(&pid_param_m2, req_rpm->motor2, mes_rpm.motor2);
    motor1.rotate(ctrl_pwm1);
    motor2.rotate(ctrl_pwm2);
}

void setup() {
    Serial.begin(115200); // For Serial printing
    delay(100);
    // Initialize encoders
    ESP32Encoder::useInternalWeakPullResistors = UP;
    motor1_encoder.attachFullQuad(MOTOR1_ENC_A, MOTOR1_ENC_B);
    motor2_encoder.attachFullQuad(MOTOR2_ENC_A, MOTOR2_ENC_B);

    // Sensor read task creation in core 0
    delay(50);
    xTaskCreatePinnedToCore(sensor_read, "SensorRead", 4096, NULL, 1, &SensorReadHandle, 0); // Core 0

    // PID control timer setup at interval of CONTROL_PERIOD_MS
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = pid_loop_cb,
        .arg = &des_rpm,
        .name = "pid_loop"
    };
    esp_timer_handle_t pid_loop_timer = NULL;
    esp_timer_create(&periodic_timer_args, &pid_loop_timer);
    esp_timer_start_periodic(pid_loop_timer, CONTROL_PERIOD_MS * 1000); //! interrupt function callback for pid control
}

void loop() {

  for(int i = 0; i <= 80; i++){   
    // changing the RPM with PWM
    des_rpm.motor1 = (float)(i); // Desired RPM for motor 1
    des_rpm.motor2 = (float)(i); // Desired RPM for motor 2

    //Sensor data print
    data_sensor[0] = '\0';
    snprintf(data_sensor, sizeof(data_sensor), "Motor1: PWM:%d, RPM:%0.2f, RPM: %0.2f, Current: %0.3f\n",
      ctrl_pwm1, des_rpm.motor1, mes_rpm.motor1, mes_current.motor1);
    Serial.print(data_sensor);
    data_sensor[0] = '\0';
    snprintf(data_sensor, sizeof(data_sensor), "Motor2: PWM:%d, RPM:%0.2f, RPM: %0.2f, Current: %0.3f\n",
      ctrl_pwm2, des_rpm.motor2, mes_rpm.motor2, mes_current.motor2);
    Serial.print(data_sensor);

    delay(100);
  }

  for(int i = 80; i >= -80; i--){
    // changing the RPM with PWM

    des_rpm.motor1 = (float)(i); // Desired RPM for motor 1
    des_rpm.motor2 = (float)(i); // Desired RPM for motor 2
    
    // Sensor data print
    data_sensor[0] = '\0';
    snprintf(data_sensor, sizeof(data_sensor), "Motor1: PWM:%d, RPM:%0.2f, RPM: %0.2f, Current: %0.3f\n",
      ctrl_pwm1, des_rpm.motor1, mes_rpm.motor1, mes_current.motor1);
    Serial.print(data_sensor);
    data_sensor[0] = '\0';
    snprintf(data_sensor, sizeof(data_sensor), "Motor2: PWM:%d, RPM:%0.2f, RPM: %0.2f, Current: %0.3f\n",
      ctrl_pwm2, des_rpm.motor2, mes_rpm.motor2, mes_current.motor2);
    Serial.print(data_sensor);

    delay(100);
  }

  for(int i = -80; i <= 0; i++){
    // changing the RPM with PWM
    des_rpm.motor1 = (float)(i); // Desired RPM for motor 1
    des_rpm.motor2 = (float)(i); // Desired RPM for motor 2

    // Sensor data print
    data_sensor[0] = '\0';
    snprintf(data_sensor, sizeof(data_sensor), "Motor1: PWM:%d, RPM:%0.2f, RPM: %0.2f, Current: %0.3f\n",
      ctrl_pwm1, des_rpm.motor1, mes_rpm.motor1, mes_current.motor1);
    Serial.print(data_sensor);
    data_sensor[0] = '\0';
    snprintf(data_sensor, sizeof(data_sensor), "Motor2: PWM:%d, RPM:%0.2f, RPM: %0.2f, Current: %0.3f\n",
      ctrl_pwm2, des_rpm.motor2, mes_rpm.motor2, mes_current.motor2);
    Serial.print(data_sensor);

    delay(100);
  }
}
