#pragma once

#include <cmath>
#include <string>

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/api/custom_api_device.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace hot_tub {

class HotTub : public Component, public api::CustomAPIDevice {
 public:
  // --- ESPHome lifecycle ---
  void setup() override;
  void loop() override;

  // --- Setters from codegen (sensor.py) ---
  void set_temperature_sensor(sensor::Sensor *s) { temperature_ = s; }
  void set_pressure_switch_sensor(sensor::Sensor *s) { pressure_switch_ = s; }
  void set_pump_1_sensor(sensor::Sensor *s) { pump_1_ = s; }
  void set_pump_2_sensor(sensor::Sensor *s) { pump_2_ = s; }
  void set_heating_active_sensor(sensor::Sensor *s) { heating_active_ = s; }
  void set_error_messages_sensor(sensor::Sensor *s) { error_messages_ = s; }

  // --- Services / HA subscriptions (kept from your code) ---
  void On_Pump_1_Button();
  void On_Pump_2_Button();
  void On_Reset_Button();

  void on_commanded_temperature_changed(std::string state);
  void on_commanded_filter_cycle(std::string state);
  void on_commanded_inhibit_heating(std::string state);
  void on_commanded_debug(std::string state);

 private:
  // --- Publish helper ---
  void publish_if_changed_();

  // --- Original logic moved into methods ---
  void Debounce_Temperature();
  void Read_Temperature_ADC();
  void Service_Pump_Buttons();
  void Service_Pressure_Switch();
  void Main_State_Machine();
  bool Check_Pressure_Switch();
  void Run_Alive_Function();
  void Service_Pump_Timers();
  void Request_Pump_1(int PumpReq);
  void Start_Pump_1_Timer();
  void Start_Pump_2_Timer();
  void Clear_Pump_1_Timer();
  void Clear_Pump_2_Timer();
  void Set_Pump_1_OFF();
  void Set_Pump_1_ON();
  void Set_Pump_1_HIGH();
  void Set_Pump_2_OFF();
  void Set_Pump_2_ON();
  void Turn_Heater_On();
  void Turn_Heater_Off();
  void Set_HT_State(int State);
  int Get_HT_State();
  void Set_HT_Error(int Err);
  int Get_HT_Error();
  void Error_Shutdown();
  void CalculateTemp(int AverageReading);

  // --- Sensors (optional) ---
  sensor::Sensor *temperature_{nullptr};
  sensor::Sensor *pressure_switch_{nullptr};
  sensor::Sensor *pump_1_{nullptr};
  sensor::Sensor *pump_2_{nullptr};
  sensor::Sensor *heating_active_{nullptr};
  sensor::Sensor *error_messages_{nullptr};

  // last-published (so we only publish on change)
  float last_temp_{NAN};
  int last_pressure_{-1};
  int last_pump1_{-1};
  int last_pump2_{-1};
  int last_heat_{-1};
  int last_err_{-1};

  // -----------------------------
  // Original “globals” made members
  // -----------------------------
  float CommandedTemp{0};
  float MeasuredTemp{0.0f};
  int Pump1State{0};
  int Pump2State{0};
  bool HeatingActive{false};
  int ErrorMessages{0};
  bool PressureSwitchState{false};
  bool FilterCycleActive{false};
  bool InhibitHeating{false};
  bool Hot_Tub_Debug{false};

  // internal state
  int HT_State{0};
  int HT_State_Inv{0};
  int Pump1Timer{0};
  int Pump2Timer{0};
  float RawMeasuredTemp{0};

  // loop timing
  unsigned long PreviousTime{0};
  int DelayCounter{0};

  // constants (from your header)
  static constexpr const char *const TAG = "hot_tub";

  static const int HA_SERVICE_UPDATE_INTERVAL = 1000;  // ms

  static const int WATER_TEMP_INIT_VALUE = 101;

  static const int PUMP_OFF = 0;
  static const int PUMP_ON = 1;
  static const int PUMP_HIGH = 2;

  static const int PUMP_1_RUN_TIME = 20;  // minutes
  static const int PUMP_2_RUN_TIME = 30;  // minutes

  static const int HEAT_ON = 1;
  static const int HEAT_OFF = 0;

  static const int TRUE_ = 1;
  static const int FALSE_ = 0;

  static const int STARTUP_DELAY = 30;

  // You said: CLOSED is 1.
  static const int PRESSURE_OPEN = 0;
  static const int PRESSURE_CLOSED = 1;
  static const int PRESSURE_SWITCH_CYCLES = 20;

  static const int PUMP_BUTTON_NOT_PRESSED = 1;
  static const int PUMP_BUTTON_PRESSED = 0;
  static const int PUMP_BUTTON_DEBOUNCE_CYCLES = 10;

  static const int HT_STATE_INIT = 0;
  static const int HT_STATE_RUN = 1;
  static const int HT_STATE_PRE_HEAT = 2;
  static const int HT_STATE_HEAT = 3;
  static const int HT_STATE_POST_HEAT = 4;
  static const int HT_STATE_ERROR = 5;

  static constexpr float TEMP_UPPER_HYSTERESIS = 0.5f;
  static constexpr float TEMP_LOWER_HYSTERESIS = 0.0f;

  static const int PRE_HEAT_CIRC_TIMEOUT = 10;  // seconds
  static const int POST_HEAT_CIRC_TIME = 60;    // seconds
  static const int ONE_MINUTE = 60;             // seconds
  static const int ONE_SECOND = 1000;           // milliseconds

  // pins (same as your header)
  static const int PUMP_1_LOW = 13;
  static const int PUMP_1_HIGH = 33;
  static const int PUMP_2_HIGH = 25;
  static const int HEATER_1 = 26;
  static const int HEATER_2 = 27;
  static const int ONBOARD_LED = 2;
  static const int MONITOR_TOGGLE_PIN = 18;

  static const int TEMPERATURE_SENSOR = 35;
  static const int HIGH_LIMIT_SWITCH = 34;  // currently unused
  static const int PRESSURE_SWITCH = 4;
  static const int PUMP_1_BUTTON = 14;
  static const int PUMP_2_BUTTON = 12;

  // Error States
  static const int ERR_OK = 0x00;
  static const int ERR_PRESSURE_SWITCH_PRE_HEAT = 0x01;
  static const int ERR_HIGH_LIMIT = 0x02;
  static const int ERR_MAIN_STATE_DEFAULT = 0x04;
  static const int ERR_OVER_SET_TEMP = 0x08;
  static const int ERR_STATE_VAR = 0x10;
  static const int ERR_PRESSURE_SWITCH_HEAT = 0x20;

  // temp lookup
  static const int TEMP_LOOKUP_SIZE = 121;
  static const int TEMP_LOOKUP_VALUES = 2;
  static const int TEMP_LOOKUP_ADVALUES = 0;
  static const int TEMP_LOOKUP_TEMPERATURE = 1;

  static const int ADC_SAMPLES = 200;
  static const int TEMP_DEBOUNCE_CYCLES = 10;

  float TemperatureLookup[TEMP_LOOKUP_SIZE][TEMP_LOOKUP_VALUES] = {
      {0, 61.0},   {0, 61.5},   {0, 62.0},   {3, 62.5},   {5, 63.0},   {29, 63.5},  {54, 64.0},
      {78, 64.0},  {103, 64.0}, {127, 64.5}, {151, 65.0}, {176, 65.5}, {200, 66.0}, {231, 66.5},
      {261, 67.0}, {292, 67.5}, {322, 68.0}, {353, 68.5}, {383, 69.0}, {414, 69.5}, {444, 70.0},
      {475, 70.5}, {505, 71.0}, {536, 71.5}, {566, 72.0}, {597, 72.5}, {627, 73.0}, {658, 73.0},
      {688, 73.0}, {719, 73.5}, {749, 74.0}, {780, 74.5}, {810, 75.0}, {841, 75.5}, {871, 76.0},
      {902, 76.5}, {933, 77.0}, {971, 77.5}, {1010, 78.0}, {1048, 78.5}, {1086, 79.0}, {1124, 79.5},
      {1163, 80.0}, {1201, 80.5}, {1239, 81.0}, {1277, 81.5}, {1316, 82.0}, {1354, 82.0}, {1393, 82.0},
      {1421, 82.5}, {1449, 83.0}, {1478, 83.5}, {1506, 84.0}, {1534, 84.5}, {1562, 85.0}, {1590, 85.5},
      {1619, 86.0}, {1647, 86.5}, {1675, 87.0}, {1703, 87.5}, {1731, 88.0}, {1760, 88.5}, {1788, 89.0},
      {1816, 89.5}, {1844, 90.0}, {1872, 90.5}, {1901, 91.0}, {1929, 91.0}, {1957, 91.0}, {1985, 91.5},
      {2013, 92.0}, {2042, 92.5}, {2070, 93.0}, {2098, 93.5}, {2126, 94.0}, {2153, 94.5}, {2181, 95.0},
      {2209, 95.5}, {2237, 96.0}, {2264, 96.5}, {2292, 97.0}, {2320, 97.0}, {2348, 97.0}, {2376, 97.5},
      {2404, 98.0}, {2431, 98.5}, {2457, 99.0}, {2484, 99.5}, {2510, 100.0}, {2539, 100.5}, {2568, 101.0},
      {2596, 101.5}, {2625, 102.0}, {2652, 102.5}, {2680, 103.0}, {2707, 103.5}, {2734, 104.0}, {2764, 104.5},
      {2794, 105.0}, {2823, 105.5}, {2853, 106.0}, {2888, 106.5}, {2924, 107.0}, {2959, 107.5}, {2994, 108.0},
      {3019, 108.5}, {3045, 109.0}, {3070, 109.0}, {3095, 109.0}, {3129, 109.5}, {3163, 110.0}, {3196, 110.5},
      {3230, 111.0}, {3264, 111.5}, {3298, 112.0}, {3331, 112.5}, {3365, 113.0}, {3399, 113.5}, {3433, 114.0},
      {3466, 114.5}, {3500, 115.0},
  };
};

}  // namespace hot_tub
}  // namespace esphome