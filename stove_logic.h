#pragma once
#include "esphome.h"
#include <cmath>

inline void run_stove_control_loop() {
  
  if (!id(stove_pwr_status).state) return;


  uint32_t now = millis();
  float T_smoke = id(temp_smoke).state;
  float T_water = id(temp_water).state;

  // 1. SANITY CHECK (Sensor Health) ---
  if (std::isnan(T_smoke) || T_smoke < 0) {
    id(status_message) = "ERRORE: Sonda Fumi Guasta";
    ESP_LOGE("SAFETY", "CRITICAL: Sonda Fumi Fail!");
    id(critical_exit).execute();
    return;
  }

  // 2. STARTUP LOGIC
  if (!id(launch_status)) {
    // Case A. Fire detected
    if (T_smoke >= id(smoke_start_threshold)) {
      id(launch_status) = true;
      id(led_ok).turn_on();
      
      // Anti-Windup at startup
      id(lastTime) = now;
      id(prev_error) = id(T_target) - T_water; 
      id(integral) = 0.0; 
      
      id(status_message) = "RUN: OK";
      ESP_LOGW("STATE_MGR", ">>> FIRE DETECTED. Entering RUN Mode. <<<");
      return;
    }

    // Case B. Timeout
    if ((now - id(starting_time)) >= id(launch_time_cycle)) {
      id(status_message) = "ERRORE: Mancata Accensione";
      ESP_LOGE("STATE_MGR", "BOOT FAILED: Timeout.");
      id(critical_exit).execute();
      return;
    }

    // Case C. Waiting (UI Update)
    if (!id(boot_sequence).is_running()) {
      id(status_message) = "AVVIO: Attesa Fiamma";
    }
    return;
  }

  // --- 3. RUN LOGIC & SAFETY ---

  // Safety: Loss of Flame
  if (T_smoke < id(smoke_min_run)) {
    id(status_message) = "ERRORE: Fiamma Persa";
    ESP_LOGE("SAFETY", "FLAME LOSS detected.");
    id(critical_exit).execute();
    return;
  }

  // Safety: Smoke Overheat
  if (T_smoke > id(smoke_max_safety)) {
    id(status_message) = "ERRORE: Surriscaldamento Fumi";
    ESP_LOGE("SAFETY", "OVERHEAT SMOKE.");
    id(critical_exit).execute();
    return;
  }

  // Safety: Water Overheat
  if (!std::isnan(T_water) && T_water > 85.0) {
    id(status_message) = "ERRORE: Temp Acqua Alta";
    ESP_LOGE("SAFETY", "OVERHEAT WATER.");
    id(critical_exit).execute();
    return;
  }

  id(status_message) = "RUN: OK";

  // --- 4. PID LOOP & ACTUATOR CONTROL ---
  
  uint32_t dt_ms = now - id(lastTime);
  
  // Sampling Enforcement: Execute PID only every cycleTime
  if (dt_ms < id(cycleTime)) return;

  float dt = dt_ms / 1000.0f; // Seconds
  id(lastTime) = now;

  float error = id(T_target) - T_water;
  if (std::isnan(error)) return;

  // A. PID

  // Integral
  id(integral) += error * dt;
  id(integral) = fmaxf(fminf(id(integral), 50.0f), -50.0f);

  // Derivative
  float derivative = (error - id(prev_error)) / dt;
  id(prev_error) = error;

  // PID output(normalized 0.0 - 1.0+)
  float pid_power = (id(K_p) * error) + (id(K_i) * id(integral)) + (id(K_d) * derivative);

  // Output Saturation (0.0 - 1.2 for aggressive boost, but min 0)
  pid_power = fmaxf(fminf(pid_power, 1.2f), 0.0f);
  
  
  // B. "Smart Bucket" Logic
  static float pellet_bucket = 0.0f;
  
  if (pid_power > 0) {
      pellet_bucket += pid_power * dt; 
  }

  // Clamp Bucket (Max 10s reserve)
  pellet_bucket = fminf(pellet_bucket, 10.0f);
  
  // C. Actuation
  const float MIN_MOTOR_ON = 0.8f; // Minimum relay activation time
  float t_on = 0.0f;

  if (pellet_bucket >= MIN_MOTOR_ON) {
      float max_run_this_cycle = dt - 0.2f; // Leave some margin in the cycle
      t_on = fminf(pellet_bucket, max_run_this_cycle);
      pellet_bucket -= t_on;
  }

  // Debug Telemetry
  ESP_LOGD("PID_CTRL", "Err:%.1f | P:%.2f I:%.2f D:%.2f | Out:%.2f | Buck:%.2fs | ACT:%.2fs", 
           error, (id(K_p)*error), (id(K_i)*id(integral)), (id(K_d)*derivative), 
           pid_power, (pellet_bucket + t_on), t_on);

  if (t_on > 0.0f) {
    // Call the script defined in YAML passing the milliseconds
    id(coclea_pulse).execute((int)(t_on * 1000));
  }
}