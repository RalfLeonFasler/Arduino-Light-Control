# Arduino-Light-Control

If you want to use an ESP32, check that #define USE_ESP32 is not commented out. If you use an arduino, comment this line out.

With this, you can control a LED light strip (or any other PWM driven stuff) via 2 Buttons. Just connect the buttons to pin 17 and 18 (or changing button pins at the top of the script) and your led strip (with a suitable driver) to pin 5 (you can change it as well).

You can turn the light on or off by just pressing one off the buttons. To switch between dim settings (also changeable in the code), just press and hold one of the buttons till the leds blink. Hold the button. Now the leds will go through the preconfigured dimming settings one after another. Let go off the button when the desired brightness is presented. It will increase the brightness step by step. When it has reached the top end, it will start at the bottom again. Another way is to hold both of the buttons at the same time and let go ones it blinks. After this you can increase or decrease the brightness by pressing one of the buttons. Here it will again go through the preconfigures dimming settins. A schematic of the electronics will follow soon. I use this for over a year now and it never failed or behaved weird.

# Smart-Light-Control

This only works with an ESP32. The general behaviour is the same. But here, you can connect to your Wifi and a MQTT broker of you choice. The ESP32 will subscribe to the given topics and will react to them. You can turn the lights on, with a given brigthness, or off. You can adjust the time that it needs for turning the lights on and off (turn_time) and the sleep timer (sleep_timer). It is also possible to turn the lights on as an alarm clock. For this you can also set the time it takes to turn on (alarm_turn_time).

There are plenty of Apps that you can use for connecting to the mqtt broker with your phone. There you can often set the topic and payload and like this you can control the lights. 