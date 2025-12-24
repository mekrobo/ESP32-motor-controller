#include <Arduino.h>
#include <ESP32Encoder.h>
#include "config.h"
#include "motor.h"

ESP32Encoder motor1_encoder, motor2_encoder; // Encoder object for motor 1 and motor 2
MotorController motor1(Channel_1); // Motor Controller object for channel 1
MotorController motor2(Channel_2); // Motor Controller object for channel 2

TaskHandle_t SensorReadHandle = NULL; // Sensor Reading Task Handle

float TICKS2RPM_1 = 60000.0 / (COUNTS_PER_REV_1 * SENSOR_PERIOD_MS); // RPM calculation factor for motor 1
float TICKS2RPM_2 = 60000.0 / (COUNTS_PER_REV_2 * SENSOR_PERIOD_MS); // RPM calculation factor for motor 2
float VALUE2AMP = (3.3 * 1.7483) / 4095.0; // Assuming a 12-bit ADC and 3.3V reference

rpm mes_rpm;                     // Measured RPM structure
current mes_current;             // Measured current structure
int64_t last_pulse_count_m1 = 0; // store previous pulse count for motor 1
int64_t last_pulse_count_m2 = 0; // store previous pulse count for motor 2
int64_t pulse_count_m1 = 0;      // current pulse count for motor 1
int64_t pulse_count_m2 = 0;      // current pulse count for motor 2
char data_sensor[56];            // serial print buffer

// Sensor reading in a periodic interval of SENSOR_PERIOD_MS (miliseconds)
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

void setup() {
    Serial.begin(115200); // For Serial printing
    delay(100);
    // Initialize encoders
    ESP32Encoder::useInternalWeakPullResistors = UP;
    motor1_encoder.attachFullQuad(MOTOR1_ENC_A, MOTOR1_ENC_B);
    motor2_encoder.attachFullQuad(MOTOR2_ENC_A, MOTOR2_ENC_B);

    // Additional setup code can be added here
    delay(50);
    xTaskCreatePinnedToCore(sensor_read, "SensorRead", 4096, NULL, 1, &SensorReadHandle, 0); // Core 0

}

void loop() {
// changing PWM
for(int dutyCycle = 0; dutyCycle <= 80; dutyCycle++){   
    
    motor1.rotate(dutyCycle);
    motor2.rotate(-dutyCycle);
    data_sensor[0] = '\0';
    snprintf(data_sensor, sizeof(data_sensor), "Motor1:: PWM:%d, RPM: %0.2f, Current: %0.3f\n",
      dutyCycle, mes_rpm.motor1, mes_current.motor1);
    Serial.print(data_sensor);
    data_sensor[0] = '\0';
    snprintf(data_sensor, sizeof(data_sensor), "Motor2:: PWM:%d, RPM: %0.2f, Current: %0.3f\n",
      -dutyCycle, mes_rpm.motor2, mes_current.motor2);
    Serial.print(data_sensor);

    delay(100);
  }

for(int dutyCycle = 80; dutyCycle >= -80; dutyCycle--){

    motor1.rotate(dutyCycle);
    motor2.rotate(-dutyCycle);

    data_sensor[0] = '\0';
    snprintf(data_sensor, sizeof(data_sensor), "Motor1:: PWM:%d, RPM: %0.2f, Current: %0.3f\n",
      dutyCycle, mes_rpm.motor1, mes_current.motor1);
    Serial.print(data_sensor);
    data_sensor[0] = '\0';
    snprintf(data_sensor, sizeof(data_sensor), "Motor2:: PWM:%d, RPM: %0.2f, Current: %0.3f\n",
      -dutyCycle, mes_rpm.motor2, mes_current.motor2);
    Serial.print(data_sensor);

    delay(100);
  }

for(int dutyCycle = -80; dutyCycle <= 0; dutyCycle++){

    motor1.rotate(dutyCycle);
    motor2.rotate(-dutyCycle);

    data_sensor[0] = '\0';
    snprintf(data_sensor, sizeof(data_sensor), "Motor1:: PWM:%d, RPM: %0.2f, Current: %0.3f\n",
      dutyCycle, mes_rpm.motor1, mes_current.motor1);
    Serial.print(data_sensor);
    data_sensor[0] = '\0';
    snprintf(data_sensor, sizeof(data_sensor), "Motor2:: PWM:%d, RPM: %0.2f, Current: %0.3f\n",
      -dutyCycle, mes_rpm.motor2, mes_current.motor2);
    Serial.print(data_sensor);

    delay(100);
  }

}
