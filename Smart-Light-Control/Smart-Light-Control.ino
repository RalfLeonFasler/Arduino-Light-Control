#include <WiFi.h>
#include <PubSubClient.h>

// Define your Wi-Fi and MQTT broker credentials
const char* ssid = "Your WIFI SSID";
const char* password = "Your WIFI Password";
const char* mqtt_server = "Your MQTT Broker's IP";
const int mqtt_port = 1883;
const char* mqtt_topic_incoming = "bed/switch/incoming";
const char* mqtt_topic_outgoing = "bed/switch/outgoing";
const char* mqtt_topic_turn_time = "bed/settings/turnTime";
const char* mqtt_topic_alarm_turn_time = "bed/settings/alarmTurnTime";
const char* mqtt_topic_alarm = "bed/switch/alarm";
const char* mqtt_topic_sleep = "bed/settings/sleepTimer";

const unsigned long reconnect_timer = 5 * 600000000;  //reconnect every x [µs], 5*60000000=5min
unsigned long last_reconnect_attempt = 0;

// Create an instance of the PubSubClient
WiFiClient espClient;
PubSubClient client(espClient);

const int button_pins[2] = { 17, 16 };  //Button input pins
const int pwm_pin = 5;                  //Output PWM pin
const int pwm_channel = 0;              //Only for ESP32

const int dims[6] = { 8, 16, 32, 64, 128, 255 };  //selectable dimming values
const int super_low_dims[3] = { 1, 2, 4 };        //selectable simming values when in_super_low
int length_dims = 0;
int length_super_low_dims = 0;

const unsigned long ts = 10000;  //cycle time µs
unsigned int turn_time = 1500;   //time to turn lights on and off in ms
int step = 0;
unsigned long alarm_turn_time = 1800000;  //time to turn lights on in case its an alarm in ms

const unsigned int short_press_threshold = 2;
const unsigned int long_press_threshold = 50;  //In multiples of ts
const unsigned int xtra_long_press_threshold = 150;

int value = 255;
int super_low_value = 1;
int count = 5;
int super_low_count = 0;

bool on = false;
bool in_super_low = false;
bool in_alarm = false;
unsigned int pressed_count = 0;
float current_value = 0;
unsigned int target_value = 0;
unsigned int previous_value = 0;

unsigned long start = 0;
unsigned long sleep_timer = 540000;  //turn light of after this in ms
unsigned long sleep_timer_count = 0;

int findClosestValueIndex(int reference, const int* array, int arrayLength) {
  int closestIndex = 0;
  int minDifference = abs(reference - array[0]);  // Initialize with the difference to the first element

  for (int i = 1; i < arrayLength; i++) {
    int difference = abs(reference - array[i]);

    if (difference < minDifference) {
      minDifference = difference;
      closestIndex = i;
    }
  }

  return closestIndex;
}

// Function to handle MQTT messages
void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println("Received: " + message + " from " + topic);
  if (strcmp(topic, mqtt_topic_incoming) == 0) {
    int temp_value = 0;
    temp_value = message.toInt();
    if (temp_value > 255) temp_value = 255;
    if (temp_value < 0) temp_value = 0;
    previous_value = current_value;
    if (temp_value == 0) {
      target_value = 0;
      on = false;
    } else {
      value = temp_value;
      target_value = value;
      on = true;
      count = findClosestValueIndex(value, dims, length_dims);
    }
    in_alarm = false;
    in_super_low = false;
    sendInt(target_value);
  } else if (strcmp(topic, mqtt_topic_alarm) == 0) {
    int temp_value = 0;
    temp_value = message.toInt();
    if (temp_value > 255) temp_value = 255;
    if (temp_value < 0) temp_value = 0;
    previous_value = current_value;
    if (temp_value == 0) {
      target_value = 0;
      on = false;
    } else {
      value = temp_value;
      target_value = value;
      on = true;
      in_alarm = true;
    }
    in_super_low = false;
    sendInt(target_value);
  } else if (strcmp(topic, mqtt_topic_turn_time) == 0) {
    turn_time = message.toInt() * 1000;
    if (turn_time > 10000) turn_time = 10000;
    if (turn_time < 1) turn_time = 1;
  } else if (strcmp(topic, mqtt_topic_alarm_turn_time) == 0) {
    alarm_turn_time = message.toInt() * 1000 * 60;
    if (alarm_turn_time > 3600000) alarm_turn_time = 3600000;
    if (alarm_turn_time < 60000) alarm_turn_time = 60000;
  } else if (strcmp(topic, mqtt_topic_sleep)==0) {
    sleep_timer = message.toInt() * 1000 * 60;
    if (sleep_timer > 14400000) sleep_timer = 14400000;
    if (sleep_timer < 60000) sleep_timer = 60000;
  } else {
    Serial.println("No matching topic");
  }
}

// Function to reconnect to Wi-Fi and MQTT
void reconnect(bool wait) {
  if (WiFi.status() != WL_CONNECTED) {  //not connected to wifi
    //TODO
  } else {
    if (!client.connected() && ((micros() - last_reconnect_attempt > reconnect_timer) || !wait)) {
      last_reconnect_attempt = micros();
      Serial.println("Attempting to connect to MQTT broker...");
      if (client.connect("ESP32Client")) {
        Serial.println("Connected to MQTT broker");
        client.subscribe(mqtt_topic_incoming);
        client.subscribe(mqtt_topic_turn_time);
        client.subscribe(mqtt_topic_alarm_turn_time);
        client.subscribe(mqtt_topic_alarm);
        client.subscribe(mqtt_topic_sleep);
      } else {
        Serial.print("Failed, rc=");
        Serial.println(client.state());
      }
      Serial.println((micros() - last_reconnect_attempt) / 1000);
    }
  }
}

void sendInt(int value) {
  client.publish(mqtt_topic_outgoing, String(value).c_str());
}

void writePWM(int dutycycle) {
  ledcWrite(pwm_channel, dutycycle);
  Serial.println("Write:" + String(dutycycle));
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initialize...");
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  int wifi_count = 0;
  while (WiFi.status() != WL_CONNECTED) {
    wifi_count++;
    if (wifi_count > 10) break;
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to Wi-Fi");
  pinMode(button_pins[0], INPUT_PULLUP);
  pinMode(button_pins[1], INPUT_PULLUP);
  ledcSetup(pwm_channel, 1000, 8);
  ledcAttachPin(pwm_pin, pwm_channel);
  // Set the MQTT server and callback
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  reconnect(false);
  writePWM(0);
  length_dims = sizeof(dims) / sizeof(dims[0]);
  length_super_low_dims = sizeof(super_low_dims) / sizeof(super_low_dims[0]);
  Serial.println("Initialization done");
}

void loop() {
  start = micros();
  if (!client.connected()) {
    reconnect(true);
  }
  client.loop();
  if (digitalRead(button_pins[0]) == 0 || digitalRead(button_pins[1]) == 0) {
    pressed_count++;
    if (pressed_count > long_press_threshold && !on) {  //long press
      previous_value = current_value;
      target_value = super_low_value;
      in_super_low = true;
      in_alarm = false;
    } else if (pressed_count > xtra_long_press_threshold && on) {
      unsigned int hold = pressed_count - xtra_long_press_threshold;
      if (hold < 2000 * 1000 / ts) {          //Toggle leds for visual indication of mode
        current_value = (float)target_value;  //disable setting of leds in loop by setting current_value to target_value
        if (hold == 1) {
          if (in_super_low) writePWM(0);
          else writePWM(target_value / 2);
        } else if (hold == 500 * 1000 / ts) {
          writePWM(target_value);
        } else if (hold == 1000 * 1000 / ts) {
          if (in_super_low) writePWM(0);
          else writePWM(target_value / 2);
        } else if (hold == 1500 * 1000 / ts) {
          writePWM(target_value);
        }
      } else {
        if (hold % (1000 * 1000 / ts) == 0) {
          if (in_super_low) {
            super_low_count++;
            if (super_low_count >= length_super_low_dims) super_low_count = 0;
            super_low_value = super_low_dims[super_low_count];
            target_value = super_low_value;
            current_value = (float)target_value;
            previous_value = current_value;
            writePWM(target_value);
            sendInt(target_value);
          } else {
            count++;
            if (count >= length_dims) count = 0;
            value = dims[count];
            target_value = value;
            current_value = (float)target_value;
            previous_value = current_value;
            writePWM(target_value);
            sendInt(target_value);
          }
        }
      }
    }
  } else {
    if (pressed_count > short_press_threshold) {
      if (pressed_count < long_press_threshold) {  //Short Press released
        if (!on) {                                 //Not on in the moment
          previous_value = current_value;
          target_value = value;
          on = true;
          in_super_low = false;
          in_alarm = false;
          sendInt(target_value);
        } else {  //On in the moment
          previous_value = current_value;
          target_value = 0;
          on = false;
          in_super_low = false;
          in_alarm = false;
          sendInt(target_value);
        }
      } else if (pressed_count < xtra_long_press_threshold) {  //long Press released
        if (!on) {
          //target_value = super_low_dims[super_low_count]; //target value already set when long press is reached to directly turn the lights on
          on = true;
          sendInt(target_value);
        } else {
          //Do nothing on long press released when its on
        }
      } else {  //xtra long press released
        if (!on) {
          on = true;
          sendInt(target_value);
        } else {
          //Do nothing on xtra long press released when its on
        }
      }
      pressed_count = 0;
    }
  }
  if (round(current_value) != target_value) {
    if ((target_value < dims[0] && target_value != 0)||(previous_value < dims[0] && previous_value != 0)) current_value = (float)target_value;  //in_super_low
    else if (in_alarm) current_value += ((float)target_value - (float)previous_value) * ((float)ts / 1000.0) / (float)alarm_turn_time;
    else current_value += ((float)target_value - (float)previous_value) * ((float)ts / 1000.0) / (float)turn_time;
    if (target_value < previous_value && current_value < (float)target_value) current_value = (float)target_value;
    if (target_value > previous_value && current_value > (float)target_value) current_value = (float)target_value;
    if (current_value > 255.0) current_value = 255.0;
    if (current_value < 0) current_value = 0.0;
    Serial.println("T:" + String(target_value) + ",C:" + String(current_value) + ",P:" + String(previous_value));
    writePWM(round(current_value));
  } else {
    current_value = (float)target_value;
    previous_value = current_value;
  }
  if (on) {
    sleep_timer_count++;
    if (sleep_timer_count > sleep_timer * 1000 / ts) {
      previous_value = current_value;
      target_value = 0;
      sendInt(0);
      on = false;
      in_super_low = false;
      in_alarm = false;
      sleep_timer_count = 0;
    }
  } else {
    sleep_timer_count = 0;
  }
  while (micros() - start < ts)
    ;  //Wait for the rest of the cycle
}
