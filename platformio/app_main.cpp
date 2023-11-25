/**
 * @file app_main.cpp
 * @brief MicroMouse Control MOdule Test App
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-01-25
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#include <esp_app_format.h>
#include <esp_ota_ops.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>  //< for vTaskDelay
#include <soc/rtc.h>

#include <iostream>

static int get_cpu_freq_in_mhz() {
  rtc_cpu_freq_config_t conf;
  rtc_clk_cpu_freq_get_config(&conf);
  return conf.freq_mhz;
}
static char* get_app_version() {
  static esp_app_desc_t app_desc;
  const esp_partition_t* running = esp_ota_get_running_partition();
  ESP_ERROR_CHECK(esp_ota_get_partition_description(running, &app_desc));
  return app_desc.version;
}

static void measurement_task(void* arg) {
  /* accel */
  void app_ctrl_accel();
  app_ctrl_accel();
  /* end */
  vTaskDelete(NULL);
}
static void serial_control_task(void* arg) {
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(10));
    char c = getc(stdin);
    switch (c) {
      case 'r':
        esp_restart();
        break;
      default:
        break;
    }
  }
}

/* called by arduino */
void setup() {
  vTaskDelay(pdMS_TO_TICKS(3000));
  std::cout << "Hello, this is ESP32." << std::endl;
  std::cout << "CPU Freq: " << get_cpu_freq_in_mhz() << " MHz" << std::endl;
  std::cout << "Build Timestamp: " << __DATE__ << " " << __TIME__ << std::endl;
  std::cout << "Version: " << get_app_version() << std::endl;
  /* measurement task */
  xTaskCreatePinnedToCore(measurement_task, "meas", 4096, NULL,
                          configMAX_PRIORITIES, NULL, APP_CPU_NUM);
  /* serial control task */
  xTaskCreatePinnedToCore(serial_control_task, "serial", 4096, NULL,
                          configMAX_PRIORITIES, NULL, PRO_CPU_NUM);
}
void loop() { vTaskDelay(portMAX_DELAY); }

/* called by esp-idf */
#if 0
extern "C" __attribute__((weak)) int app_main() {
  setup();
  while (1) {
    loop();
    vPortYield();
  }
}
#endif
