#define USE_ESP32

const int button_pins[2] = { 17, 16 };  //Button input pins
const int pwm_pin = 5;                //Output PWM pin
const int pwm_channel = 0;            //Only for ESP32

const int t_s = 3;                                //Base speed for turning light on or of / 255*3ms = 765ms for turning on or off
const int dims[6] = { 8, 16, 32, 64, 128, 255 };  //selectable dimming values
const int super_low_dims[3] = { 1, 2, 4 };        //selectable simming values when in_super_low
const int sleep_timer_1 = 32000;                  //32000*50ms = ~26min
const int sleep_timer_2 = 4;                      //4*26min = 104min -> 104min sleep timer

bool on = false;
bool in_super_low = false;
int value = 255;
int super_low_value = 1;
int count = 5;
int super_low_count = 0;
int timer = 0;
int turn = 1;
int on_time = 0;
int sleep_count = 0;
int length_dims = 0;
int length_super_low_dims = 0;

void writePWM(int dutycycle){
#ifdef USE_ESP32
  ledcWrite(pwm_channel, dutycycle);
#else
  writePWM(0);
#endif
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initialize...");
  pinMode(button_pins[0], INPUT_PULLUP);
  pinMode(button_pins[1], INPUT_PULLUP);
#ifdef USE_ESP32
  ledcSetup(pwm_channel, 1000, 8);
  ledcAttachPin(pwm_pin, pwm_channel);
  ledcWrite(pwm_channel,0);
#else
  pinMode(pwm_pin, OUTPUT);
  writePWM(0);
#endif
  length_dims = sizeof(dims) / sizeof(dims[0]);
  length_super_low_dims = sizeof(super_low_dims) / sizeof(super_low_dims[0]);
  Serial.println("Initialization done");
}

void loop() {
  if (digitalRead(button_pins[0]) == 0 || digitalRead(button_pins[1]) == 0) {
    if (!on) {
      //Check for long press when leds off. If so, turn leds on on lowest value
      while (digitalRead(button_pins[0]) == 0 || digitalRead(button_pins[1]) == 0) {
        timer++;
        delay(10);
        if (timer >= 50) {
          Serial.println("Long press turn on");
          turn = 0;
          super_low_value = super_low_dims[super_low_count];
          turn_on(super_low_value, 0);
          in_super_low = true;
          on = true;
        }
      }
      //After button is released, check if it was a long press (turn = 0). If not turn leds on as normal.
      timer = 0;
      if (turn == 1) {
        Serial.println("Normal turn on");
        int t = (t_s * 255) / value;
        turn_on(value, t);
        in_super_low = false;
        on = true;
      }
      turn = 1;
    } else {
      delay(100);
      //Set dimming value by presing both buttons at the same time and then click one of them to increase/descrease the value
      if ((digitalRead(button_pins[0]) == 0 && digitalRead(button_pins[1]) == 0)) {
        if (in_super_low) {
          writePWM(0);
          delay(500);
          writePWM(super_low_value);
          delay(500);
          writePWM(0);
          delay(500);
          writePWM(super_low_value);
        } else {
          writePWM(max(value - (value / 2), 5));
          delay(500);
          writePWM(value);
          delay(500);
          writePWM(max(value - (value / 2), 5));
          delay(500);
          writePWM(value);
        }
        while (1) {
          if (digitalRead(button_pins[0]) == 0 && digitalRead(button_pins[1]) == 0) break;  //Stop setting mode when both buttons are pressed at the same time again
          else if (digitalRead(button_pins[0]) == 0) {
            if (in_super_low) {
              super_low_count--;
              if (super_low_count <= 0) super_low_count = 0;
              super_low_value = super_low_dims[super_low_count];
              writePWM(super_low_value);
              delay(500);
            } else {
              count--;
              if (count <= 0) count = 0;
              value = dims[count];
              writePWM(value);
              delay(500);
            }
          } else if (digitalRead(button_pins[1]) == 0) {
            if (in_super_low) {
              super_low_count++;
              if (super_low_count >= length_super_low_dims) super_low_count = length_super_low_dims;
              super_low_value = dims[super_low_count];
              writePWM(super_low_value);
              delay(500);
            } else {
              count++;
              if (count >= length_dims) count = length_dims;
              value = dims[count];
              writePWM(value);
              delay(500);
            }
          }

          delay(50);
        }
        if (in_super_low) {
          delay(50);
          writePWM(0);
          delay(500);
          writePWM(super_low_value);
        } else {
          delay(50);
          writePWM(max(value - (value / 2), 5));
          delay(500);
          writePWM(value);
        }
      }
      //Set dimming values by holding on button pressed until the leds blink and the dimming value is increased step by step
      else {
        while ((digitalRead(button_pins[0]) == 0 || digitalRead(button_pins[1]) == 0)) {
          timer++;
          delay(10);
          if (timer == 150) {
            Serial.print("Dimming");
            if (in_super_low) {
              writePWM(0);
              delay(500);
              writePWM(super_low_value);
              delay(500);
              writePWM(0);
              delay(500);
              writePWM(super_low_value);
            } else {
              writePWM(max(value - (value / 2), 5));
              delay(500);
              writePWM(value);
              delay(500);
              writePWM(max(value - (value / 2), 5));
              delay(500);
              writePWM(value);
            }
            turn = 0;
          }
          if (timer >= 150) {
            Serial.print(".");
            if (in_super_low) {
              super_low_count++;
              if (super_low_count >= length_super_low_dims) super_low_count = 0;
              super_low_value = super_low_dims[super_low_count];
              writePWM(super_low_value);
              delay(1000);
            } else {
              count++;
              if (count >= length_dims) count = 0;
              value = dims[count];
              writePWM(value);
              delay(1000);
            }
          }
        }
        Serial.println();
        //After button is released, check if it was a long press (turn = 0). If not, turn leds off
        timer = 0;
        if (turn == 1) {
          Serial.println("Turn off");
          if (in_super_low) {
            turn_off(super_low_value, 0);
            in_super_low = false;
            on = false;
          } else {
            int t = (t_s * 255) / value;
            turn_off(value, t);
            in_super_low = false;
            on = false;
          }
        }
        turn = 1;
      }
    }
    delay(50);
  }

  if (on) {
    on_time++;
    if (on_time == sleep_timer_1) {
      on_time = 0;
      sleep_count++;
    }
    if (sleep_count == sleep_timer_2) {
      if (in_super_low) {
        turn_off(super_low_value, 0);
        in_super_low = false;
        on = false;
      } else {
        int t = (t_s * 255) / value;
        turn_off(value, t);
        in_super_low = false;
        on = false;
      }
      sleep_count = 0;
      on_time = 0;
    }
  } else {
    sleep_count = 0;
    on_time = 0;
  }
  delay(50);
}

void turn_on(int value, int t) {
  if (t == 0) {
    writePWM(value);
  } else {
    for (int i = 0; i <= value; i++) {
      writePWM(i);
      delay(t);
    }
  }
  return;
}

void turn_off(int value, int t) {
  if (t == 0) {
    writePWM(0);
  } else {
    for (int i = value; i >= 0; i--) {
      writePWM(i);
      delay(t);
    }
  }
  return;
}
