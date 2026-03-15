#pragma once
#include "esphome.h"
#include <cmath>

inline void run_stove_control_loop() {
  static float pellet_bucket = 0.0f;

  // Blocco anti-watchdog se la stufa è in lock critico
  if (id(led_error).state) {
    return;
  }

  // 1. ANTI-CLOGGING MODE
  if (id(anticlogging_mode).state) {
    static uint32_t last_pulse_start = 0;
    uint32_t now = millis();
    
    if (now - last_pulse_start >= 12000) {
      if (!id(coclea_pulse).is_running()) {
        id(coclea_pulse).execute(6000);
      }
      last_pulse_start = now;
    }
    return;
  }

  // 2. MASTER POWER CHECK
  if (!id(stove_pwr_status).state) {
    pellet_bucket = 0.0f; 
    return;
  }

  if (id(led_error).state) {
    return;
  }

  uint32_t now = millis();
  float T_smoke = id(temp_smoke).state;
  float T_water = id(temp_water).state;

  // 3. SANITY CHECK (Sensor Health)
  if (std::isnan(T_smoke) || T_smoke < 0) {
    id(status_message) = "ERROR_SMOKE_SENSOR_FAIL";
    ESP_LOGE("SAFETY", "CRITICAL: Sonda Fumi Fail!");
    id(critical_exit).execute();
    return;
  }

  // 4. STARTUP LOGIC
  if (!id(launch_status)) {
    
    if (!id(smoke_fan).state) id(smoke_fan).turn_on();
    if (!id(water_pump).state) id(water_pump).turn_on();

    if (T_smoke < 45.0f) {
      if (!id(spark_plug).state) id(spark_plug).turn_on();
    } else {
      if (id(spark_plug).state) id(spark_plug).turn_off();
    }

    // Case A. Fire detected
    if (T_smoke >= id(smoke_start_threshold)) {
      id(launch_status) = true;
      id(led_ok).turn_on();
      if (id(spark_plug).state) id(spark_plug).turn_off();
      
      id(lastTime) = now;
      id(prev_error) = id(T_target) - T_water; 
      id(integral) = 0.0; 
      pellet_bucket = 0.0f; 
      
      id(status_message) = "RUN_OK";
      ESP_LOGW("STATE_MGR", ">>> FIRE DETECTED. Entering RUN Mode. <<<");
      return;
    }

    // Case B. Timeout
    if ((now - id(starting_time)) >= id(launch_time_cycle)) {
      id(spark_plug).turn_off();
      id(smoke_fan).turn_off();

      id(status_message) = "ERROR_STARTUP_TIMEOUT";
      ESP_LOGE("STATE_MGR", "BOOT FAILED: Timeout.");
      id(critical_exit).execute();
      return;
    }

    // Case C UI Feedback: Waiting for high fire, small fire detected
    if (!id(boot_sequence).is_running()) {
      if (T_smoke < 45.0f) {
        id(status_message) = "START_WAITING_FIRE";
      } else {
        ESP_LOGW("STATE_MGR", ">>> SMALL FIRE DETECTED. Continuing BOOT SEQUENCE. <<<");
        if (!id(coclea_pulse).is_running()) {
          id(coclea_pulse).execute((int)(8 * 1000));
        }
      }
    }
    return;
  }

  // --- 5. RUN LOGIC & SAFETY ---

  if (T_smoke < id(smoke_min_run)) {
    id(status_message) = "ERROR_FLAME_LOSS";
    ESP_LOGE("SAFETY", "FLAME LOSS detected.");
    id(critical_exit).execute();
    return;
  }

  if (T_smoke > id(smoke_max_safety)) {
    id(status_message) = "ERROR_SMOKE_OVERHEAT";
    ESP_LOGE("SAFETY", "OVERHEAT SMOKE.");
    id(critical_exit).execute();
    return;
  }

  if (!std::isnan(T_water) && T_water > 85.0) {
    id(status_message) = "ERROR_WATER_OVERHEAT";
    ESP_LOGE("SAFETY", "OVERHEAT WATER.");
    id(critical_exit).execute();
    return;
  }

  id(status_message) = "RUN_OK";

  // --- 6. PID LOOP & ACTUATOR CONTROL ---
  
  uint32_t dt_ms = now - id(lastTime);
  
  if (dt_ms < id(cycleTime)) return;

  float dt = dt_ms / 1000.0f; 
  id(lastTime) = now;

  float error = id(T_target) - T_water;
  if (std::isnan(error)) return;

  // A. PID
  id(integral) += error * dt;
  id(integral) = fmaxf(fminf(id(integral), 50.0f), -50.0f);

  float derivative = (error - id(prev_error)) / dt;
  id(prev_error) = error;

  float pid_power = (id(K_p) * error) + (id(K_i) * id(integral)) + (id(K_d) * derivative);
  
  pid_power = fmaxf(fminf(pid_power, 0.65f), 0.0f);
  
  // B. "Smart Bucket" Logic
  const float MIN_KEEP_ALIVE = 0.125f; 
  pid_power = fmaxf(pid_power, MIN_KEEP_ALIVE);

  if (pid_power > 0) {
      pellet_bucket += pid_power * dt; 
  }

  pellet_bucket = fminf(pellet_bucket, 4.5f);
  
  // C. Actuation
  const float MIN_MOTOR_ON = 2.0f; 
  const float MIN_REST_TIME = 2.5f; 

  float t_on = 0.0f;

  if (pellet_bucket >= MIN_MOTOR_ON) {
      float max_run_this_cycle = dt - MIN_REST_TIME;
      t_on = fminf(pellet_bucket, max_run_this_cycle);
      pellet_bucket -= t_on;
  } else if (T_smoke < 130.0f) {
      t_on = dt * 0.35f; 
  }

  ESP_LOGD("PID_CTRL", "Err:%.1f | P:%.2f I:%.2f D:%.2f | Out:%.2f | Buck:%.2fs | ACT:%.2fs", 
          error, (id(K_p)*error), (id(K_i)*id(integral)), (id(K_d)*derivative), 
          pid_power, (pellet_bucket + t_on), t_on);

  if (t_on > 0.0f && !id(coclea_pulse).is_running()) {
    id(coclea_pulse).execute((int)(t_on * 1000));
  }
}