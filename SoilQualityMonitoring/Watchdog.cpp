#include <Watchdog.h>
/**
 * @brief Construct a new Watchdog:: Watchdog object
 * 
 * @param alarmPeriod in milliseconds
 */
Watchdog::Watchdog(int alarmPeriod){
  _alarmPeriod = alarmPeriod;
}

/**
 * @brief begion(). Initiate the service
 * 
 */
void Watchdog::begin(){
  esp_task_wdt_config_t _config;
  _config.idle_core_mask = 0x03;  // monitor IDLETask of both cores
  _config.trigger_panic = true;
  _config.timeout_ms = _alarmPeriod;

  esp_task_wdt_init(&_config);
}

/**
 * @brief hamndle(). To trigger the wathcdog on the loop()
 * 
 */
void Watchdog::handle(){
   esp_task_wdt_reset();
}