#include "hot_tub.h"

#include <Arduino.h>

namespace esphome {
namespace hot_tub {

void HotTub::setup() {
  int RawADValue = 0;

  // initialize variables
  CommandedTemp = WATER_TEMP_INIT_VALUE;
  MeasuredTemp = 0.0;
  Pump1State = PUMP_OFF;
  Pump2State = PUMP_OFF;
  HeatingActive = HEAT_OFF;
  Set_HT_State(HT_STATE_INIT);
  ErrorMessages = 0;
  PressureSwitchState = PRESSURE_OPEN;
  FilterCycleActive = false;
  InhibitHeating = false;

  // setup pins
  pinMode(PUMP_1_LOW, OUTPUT);
  pinMode(PUMP_1_HIGH, OUTPUT);
  pinMode(PUMP_2_HIGH, OUTPUT);
  pinMode(HEATER_1, OUTPUT);
  pinMode(HEATER_2, OUTPUT);
  pinMode(ONBOARD_LED, OUTPUT);
  pinMode(PUMP_1_BUTTON, INPUT_PULLUP);
  pinMode(PUMP_2_BUTTON, INPUT_PULLUP);
  pinMode(PRESSURE_SWITCH, INPUT_PULLUP);
  pinMode(TEMPERATURE_SENSOR, ANALOG);
  pinMode(HIGH_LIMIT_SWITCH, ANALOG);
  pinMode(MONITOR_TOGGLE_PIN, OUTPUT);

  // turn off outputs
  Set_Pump_1_OFF();
  Set_Pump_2_OFF();
  Turn_Heater_Off();

  // register pump buttons as API services
  register_service(&HotTub::On_Pump_1_Button, "Pump_1_Button");
  register_service(&HotTub::On_Pump_2_Button, "Pump_2_Button");
  register_service(&HotTub::On_Reset_Button, "Reset_Button");

  // subscribe to HA input_number states
  subscribe_homeassistant_state(&HotTub::on_commanded_temperature_changed, "input_number.hot_tub_temperature");
  subscribe_homeassistant_state(&HotTub::on_commanded_filter_cycle, "input_number.filter_cycle_active");
  subscribe_homeassistant_state(&HotTub::on_commanded_inhibit_heating, "input_number.inhibit_heating");
  subscribe_homeassistant_state(&HotTub::on_commanded_debug, "input_number.hot_tub_debug");

  // initialize temperature with 1 a/d read
  RawADValue = analogRead(TEMPERATURE_SENSOR);
  CalculateTemp(RawADValue);
  MeasuredTemp = RawMeasuredTemp;

  // publish initial values once
  publish_if_changed_();

  ESP_LOGI(TAG, "HotTub setup complete");
}

void HotTub::loop() {
  // fast loop functions
  Service_Pump_Buttons();
  Service_Pressure_Switch();
  Read_Temperature_ADC();

  // 1 second task loop
  unsigned long CurrentTime = millis();
  unsigned long DeltaTime = CurrentTime - PreviousTime;

  if (DeltaTime >= (unsigned long) ONE_SECOND) {
    if (DelayCounter >= STARTUP_DELAY) {
      Main_State_Machine();
    } else {
      DelayCounter++;
    }

    Service_Pump_Timers();
    Run_Alive_Function();
    Debounce_Temperature();

    // push to HA sensors only when something changed
    publish_if_changed_();

    PreviousTime = CurrentTime;
  }
}

void HotTub::publish_if_changed_() {
  // temperature
  if (temperature_ != nullptr) {
    if (std::isnan(last_temp_) || last_temp_ != MeasuredTemp) {
      temperature_->publish_state(MeasuredTemp);
      last_temp_ = MeasuredTemp;
    }
  }

  // pressure switch: CLOSED = 1, OPEN = 0
  if (pressure_switch_ != nullptr) {
    int p = (PressureSwitchState == PRESSURE_CLOSED) ? 1 : 0;
    if (last_pressure_ != p) {
      pressure_switch_->publish_state(p);
      last_pressure_ = p;
    }
  }

  if (pump_1_ != nullptr && last_pump1_ != Pump1State) {
    pump_1_->publish_state(Pump1State);
    last_pump1_ = Pump1State;
  }

  if (pump_2_ != nullptr && last_pump2_ != Pump2State) {
    pump_2_->publish_state(Pump2State);
    last_pump2_ = Pump2State;
  }

  if (heating_active_ != nullptr) {
    int h = HeatingActive ? 1 : 0;
    if (last_heat_ != h) {
      heating_active_->publish_state(h);
      last_heat_ = h;
    }
  }

  if (error_messages_ != nullptr && last_err_ != ErrorMessages) {
    error_messages_->publish_state(ErrorMessages);
    last_err_ = ErrorMessages;
  }
}

// -------------------- HA callbacks --------------------

void HotTub::on_commanded_temperature_changed(std::string state) {
  CommandedTemp = std::stoi(state);
}

void HotTub::on_commanded_filter_cycle(std::string state) {
  FilterCycleActive = (bool) std::stoi(state);

  // check pump state and turn on if necessary
  if ((FilterCycleActive == true) && (Pump1State == PUMP_OFF)) {
    Request_Pump_1(PUMP_ON);
  }

  // check pump state and turn off if possible
  if (FilterCycleActive == false) {
    // check if heating
    if (HeatingActive == false) {
      Request_Pump_1(PUMP_OFF);
    }
  }
}

void HotTub::on_commanded_inhibit_heating(std::string state) {
  InhibitHeating = (bool) std::stoi(state);
  ESP_LOGD(TAG, "Inhibit Heating %d", (int) InhibitHeating);
}

void HotTub::on_commanded_debug(std::string state) {
  Hot_Tub_Debug = (bool) std::stoi(state);
}

// -------------------- Core logic (ported from your header) --------------------

void HotTub::Debounce_Temperature() {
  static int DebounceCounter = 0;
  static float PreviousTemp = 0.0;
  float CurrentTemp = 0.0;

  CurrentTemp = RawMeasuredTemp;

  if (CurrentTemp == PreviousTemp) {
    if (DebounceCounter < TEMP_DEBOUNCE_CYCLES) {
      DebounceCounter++;
    }
  } else {
    DebounceCounter = 0;
  }

  if (DebounceCounter == TEMP_DEBOUNCE_CYCLES) {
    MeasuredTemp = CurrentTemp;
    //ESP_LOGD(TAG, "MeasuredTemp %f", MeasuredTemp);
  }

  PreviousTemp = CurrentTemp;
}

void HotTub::Read_Temperature_ADC() {
  static int counter = 0;
  static double average = 0;
  int temp;

  if (counter < ADC_SAMPLES) {
    counter++;
    average += analogRead(TEMPERATURE_SENSOR);
  } else {
    temp = (int) (average / ADC_SAMPLES);
    average = 0;
    counter = 0;
    CalculateTemp(temp);
  }
}

void HotTub::Service_Pump_Buttons() {
  static int Pump1PreviousButton = PUMP_BUTTON_NOT_PRESSED;
  static int Pump2PreviousButton = PUMP_BUTTON_NOT_PRESSED;

  static int Pump1DebounceCounter = 0;
  static int Pump2DebounceCounter = 0;

  // pump 1
  if (digitalRead(PUMP_1_BUTTON) == PUMP_BUTTON_PRESSED) {
    if ((Pump1DebounceCounter >= PUMP_BUTTON_DEBOUNCE_CYCLES) &&
        (Pump1PreviousButton == PUMP_BUTTON_NOT_PRESSED)) {
      On_Pump_1_Button();
      Pump1DebounceCounter = PUMP_BUTTON_DEBOUNCE_CYCLES;
      Pump1PreviousButton = PUMP_BUTTON_PRESSED;
    } else {
      Pump1DebounceCounter++;
    }
  } else {
    Pump1PreviousButton = PUMP_BUTTON_NOT_PRESSED;
    Pump1DebounceCounter = 0;
  }

  // pump 2
  if (digitalRead(PUMP_2_BUTTON) == PUMP_BUTTON_PRESSED) {
    if ((Pump2DebounceCounter >= PUMP_BUTTON_DEBOUNCE_CYCLES) &&
        (Pump2PreviousButton == PUMP_BUTTON_NOT_PRESSED)) {
      On_Pump_2_Button();
      Pump2DebounceCounter = PUMP_BUTTON_DEBOUNCE_CYCLES;
      Pump2PreviousButton = PUMP_BUTTON_PRESSED;
    } else {
      Pump2DebounceCounter++;
    }
  } else {
    Pump2PreviousButton = PUMP_BUTTON_NOT_PRESSED;
    Pump2DebounceCounter = 0;
  }
}

void HotTub::Service_Pressure_Switch() {
  static int PressureSwitchCounter = 0;

  // since CLOSED is 1 (your requirement), we treat the input reading accordingly:
  // If your hardware reads LOW when closed due to pullup, then digitalRead == 0 means closed.
  // Your original code used: if(digitalRead(...) == PRESSURE_CLOSED) with PRESSURE_CLOSED=0.
  // Now we keep semantics "PressureSwitchState is 1 when closed" but still interpret pin as LOW=closed.
  // So: pin LOW => state CLOSED (1). pin HIGH => OPEN (0).

  bool pin_closed = (digitalRead(PRESSURE_SWITCH) == LOW);

  if (pin_closed) {
    if (PressureSwitchCounter >= PRESSURE_SWITCH_CYCLES) {
      PressureSwitchCounter = PRESSURE_SWITCH_CYCLES;
      PressureSwitchState = PRESSURE_CLOSED;  // 1
    } else {
      PressureSwitchCounter++;
    }
  } else {
    if (PressureSwitchCounter > 0) {
      PressureSwitchCounter--;
    }
    if (PressureSwitchCounter == 0) {
      PressureSwitchState = PRESSURE_OPEN;  // 0
    }
  }
}

void HotTub::Main_State_Machine() {
  static int PostHeatCounter = 0;
  static int PreHeatCounter = 0;

  switch (Get_HT_State()) {
    case HT_STATE_INIT: {
      Set_Pump_1_OFF();
      Set_Pump_2_OFF();
      Turn_Heater_Off();
      Set_HT_State(HT_STATE_RUN);
      break;
    }

    case HT_STATE_RUN: {
      if ((MeasuredTemp < (CommandedTemp - TEMP_LOWER_HYSTERESIS)) && (InhibitHeating == false)) {
        Set_HT_State(HT_STATE_PRE_HEAT);
      }
      break;
    }

    case HT_STATE_PRE_HEAT: {
      if (Pump1State == PUMP_OFF) {
        Set_Pump_1_ON();
      }

      HeatingActive = HEAT_ON;

      if (Check_Pressure_Switch() == true) {
        Set_HT_State(HT_STATE_HEAT);
        PreHeatCounter = 0;
      } else {
        PreHeatCounter++;
      }

      if (PreHeatCounter >= PRE_HEAT_CIRC_TIMEOUT) {
        Set_HT_Error(ERR_PRESSURE_SWITCH_PRE_HEAT);
        HeatingActive = HEAT_OFF;
      }
      break;
    }

    case HT_STATE_HEAT: {
      Turn_Heater_On();

      if ((MeasuredTemp > (CommandedTemp + TEMP_UPPER_HYSTERESIS)) || (InhibitHeating == true)) {
        Set_HT_State(HT_STATE_POST_HEAT);
      }

      if (Check_Pressure_Switch() != true) {
        Set_HT_Error(ERR_PRESSURE_SWITCH_HEAT);
        HeatingActive = HEAT_OFF;
      }

      break;
    }

    case HT_STATE_POST_HEAT: {
      Turn_Heater_Off();

      PostHeatCounter++;

      if (PostHeatCounter <= POST_HEAT_CIRC_TIME) {
        if ((MeasuredTemp < (CommandedTemp - TEMP_LOWER_HYSTERESIS)) && (InhibitHeating == false)) {
          Set_HT_State(HT_STATE_PRE_HEAT);
          PostHeatCounter = 0;
        } else {
          // wait
        }
      } else {
        HeatingActive = HEAT_OFF;
        PostHeatCounter = 0;
        Set_HT_State(HT_STATE_RUN);

        if (Pump1Timer == 0) {
          if (FilterCycleActive == false) {
            Request_Pump_1(PUMP_OFF);
          }
        }
      }
      break;
    }

    case HT_STATE_ERROR: {
      Error_Shutdown();
      break;
    }

    default: {
      Set_HT_State(HT_STATE_ERROR);
      Error_Shutdown();
      Set_HT_Error(ERR_MAIN_STATE_DEFAULT);
      break;
    }
  }
}

bool HotTub::Check_Pressure_Switch() {
  return (PressureSwitchState == PRESSURE_CLOSED);
}

void HotTub::Run_Alive_Function() {
  static bool Indicator = false;

  if (Indicator == false) {
    digitalWrite(ONBOARD_LED, LOW);
    Indicator = true;
    digitalWrite(MONITOR_TOGGLE_PIN, HIGH);
  } else {
    digitalWrite(ONBOARD_LED, HIGH);
    Indicator = false;
    digitalWrite(MONITOR_TOGGLE_PIN, LOW);
  }
}

void HotTub::Service_Pump_Timers() {
  static int TimerCounter = 0;
  static int PreviousPump1Timer = 0;
  static int PreviousPump2Timer = 0;

  if (TimerCounter == ONE_MINUTE) {
    if (Pump1Timer >= 1) {
      Pump1Timer--;
    }

    if ((Pump1Timer == 0) && (PreviousPump1Timer == 1)) {
      if ((HeatingActive == true) && (Pump1State == PUMP_HIGH)) {
        Request_Pump_1(PUMP_OFF);
      } else if (HeatingActive == false) {
        Request_Pump_1(PUMP_OFF);
      } else {
        // do nothing
      }
      Pump1Timer = 0;
    }
    PreviousPump1Timer = Pump1Timer;

    if (Pump2Timer >= 1) {
      Pump2Timer--;
    }

    if ((Pump2Timer == 0) && (PreviousPump2Timer == 1)) {
      Set_Pump_2_OFF();
      Pump2Timer = 0;
    }
    PreviousPump2Timer = Pump2Timer;

    TimerCounter = 0;
  }

  TimerCounter++;
}

void HotTub::On_Pump_1_Button() {
  if (Pump1State == PUMP_OFF) {
    Request_Pump_1(PUMP_ON);
    Start_Pump_1_Timer();
  } else if (Pump1State == PUMP_ON) {
    Request_Pump_1(PUMP_HIGH);
    Start_Pump_1_Timer();
  } else if (Pump1State == PUMP_HIGH) {
    Request_Pump_1(PUMP_OFF);
    Clear_Pump_1_Timer();
  }
}

void HotTub::On_Pump_2_Button() {
  Pump2State++;

  if (Pump2State > PUMP_ON) {
    Pump2State = PUMP_OFF;
  }

  if (Pump2State == PUMP_ON) {
    Set_Pump_2_ON();
    Start_Pump_2_Timer();
  } else {
    Set_Pump_2_OFF();
    Clear_Pump_2_Timer();
  }
}

void HotTub::On_Reset_Button() {
  Set_HT_State(HT_STATE_INIT);
  ErrorMessages = 0;
}

void HotTub::Request_Pump_1(int PumpReq) {
  switch (PumpReq) {
    case PUMP_OFF: {
      if ((HeatingActive == HEAT_ON) || (FilterCycleActive == true)) {
        if (Pump1State == PUMP_ON) {
          Pump1State = PUMP_HIGH;
        } else {
          Pump1State = PUMP_ON;
        }
      } else {
        Pump1State = PUMP_OFF;
      }
      break;
    }
    case PUMP_ON: {
      Pump1State = PUMP_ON;
      break;
    }
    case PUMP_HIGH: {
      Pump1State = PUMP_HIGH;
      break;
    }
    default: {
      Pump1State = PUMP_OFF;
      break;
    }
  }

  switch (Pump1State) {
    case PUMP_OFF:
      Set_Pump_1_OFF();
      break;
    case PUMP_ON:
      Set_Pump_1_ON();
      break;
    case PUMP_HIGH:
      Set_Pump_1_HIGH();
      break;
    default:
      Set_Pump_1_OFF();
      break;
  }
}

void HotTub::Start_Pump_1_Timer() { Pump1Timer = PUMP_1_RUN_TIME; }
void HotTub::Start_Pump_2_Timer() { Pump2Timer = PUMP_2_RUN_TIME; }
void HotTub::Clear_Pump_1_Timer() { Pump1Timer = 0; }
void HotTub::Clear_Pump_2_Timer() { Pump2Timer = 0; }

void HotTub::Set_Pump_1_OFF() {
  digitalWrite(PUMP_1_LOW, LOW);
  digitalWrite(PUMP_1_HIGH, LOW);
  Pump1State = PUMP_OFF;
}

void HotTub::Set_Pump_1_ON() {
  if (Get_HT_Error() == ERR_OK) {
    digitalWrite(PUMP_1_LOW, HIGH);
    digitalWrite(PUMP_1_HIGH, LOW);
    Pump1State = PUMP_ON;
  }
}

void HotTub::Set_Pump_1_HIGH() {
  if (Get_HT_Error() == ERR_OK) {
    digitalWrite(PUMP_1_LOW, LOW);
    digitalWrite(PUMP_1_HIGH, HIGH);
    Pump1State = PUMP_HIGH;
  }
}

void HotTub::Set_Pump_2_OFF() {
  digitalWrite(PUMP_2_HIGH, LOW);
  Pump2State = PUMP_OFF;
}

void HotTub::Set_Pump_2_ON() {
  if (Get_HT_Error() == ERR_OK) {
    digitalWrite(PUMP_2_HIGH, HIGH);
    Pump2State = PUMP_ON;
  }
}

void HotTub::Turn_Heater_On() {
  if ((Get_HT_State() == HT_STATE_HEAT) && (Get_HT_Error() == ERR_OK)) {
    digitalWrite(HEATER_1, HIGH);
    digitalWrite(HEATER_2, HIGH);
  }
}

void HotTub::Turn_Heater_Off() {
  digitalWrite(HEATER_1, LOW);
  digitalWrite(HEATER_2, LOW);
}

void HotTub::Set_HT_State(int State) {
  HT_State = State;
  HT_State_Inv = ~State;
}

int HotTub::Get_HT_State() {
  int retVal = HT_STATE_ERROR;

  if (HT_State == (~HT_State_Inv)) {
    if (HT_State <= HT_STATE_ERROR) {
      retVal = HT_State;
    } else {
      Error_Shutdown();
      Set_HT_Error(ERR_STATE_VAR);
    }
  } else {
    Error_Shutdown();
    Set_HT_Error(ERR_STATE_VAR);
  }

  return retVal;
}

void HotTub::Set_HT_Error(int Err) {
  ErrorMessages = (ErrorMessages | Err);
  Set_HT_State(HT_STATE_ERROR);
}

int HotTub::Get_HT_Error() { return ErrorMessages; }

void HotTub::Error_Shutdown() {
  Set_Pump_1_OFF();
  Set_Pump_2_OFF();
  Turn_Heater_Off();
  Set_HT_State(HT_STATE_ERROR);
}

void HotTub::CalculateTemp(int AverageReading) {
  int midpoint = 0;

  if ((AverageReading > TemperatureLookup[TEMP_LOOKUP_SIZE - 1][TEMP_LOOKUP_ADVALUES]) ||
      (AverageReading < TemperatureLookup[0][TEMP_LOOKUP_ADVALUES])) {
    AverageReading = (int) TemperatureLookup[TEMP_LOOKUP_SIZE - 1][TEMP_LOOKUP_ADVALUES];
    ESP_LOGD(TAG, "Temp out of range");
  }

  for (int i = (TEMP_LOOKUP_SIZE - 1); i >= 1; i--) {
    if (AverageReading < TemperatureLookup[i][TEMP_LOOKUP_ADVALUES]) {
      midpoint = (int) (TemperatureLookup[i][TEMP_LOOKUP_ADVALUES] -
                        ((TemperatureLookup[i][TEMP_LOOKUP_ADVALUES] - TemperatureLookup[i - 1][TEMP_LOOKUP_ADVALUES]) / 2));

      if (AverageReading > midpoint) {
        RawMeasuredTemp = TemperatureLookup[i][TEMP_LOOKUP_TEMPERATURE];
      } else {
        RawMeasuredTemp = TemperatureLookup[i - 1][TEMP_LOOKUP_TEMPERATURE];
      }
    }
  }
}

}  // namespace hot_tub
}  // namespace esphome