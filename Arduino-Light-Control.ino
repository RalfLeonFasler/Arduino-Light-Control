const int button_pins[2] = {9,8}; //Button input pins
const int pwm_pin = 3; //Output PWM pin

const int t_s=3; //Base speed for turning light on or of / 255*3ms = 765ms for turning on or off
const int dims[7] = {1,10,50,100,150,200,255}; //selectable dimming values
const int sleep_timer_1 = 32000; //32000*50ms = ~26min
const int sleep_timer_2 = 4; //4*26min = 104min -> 104min sleep timer

int on=0;
int value=255;
int count = 5;
int timer = 0;
int turn =1;
int on_time = 0;
int sleep_count = 0;
int length_dims = 0;

void setup() {
  pinMode(pwm_pin, OUTPUT);
  pinMode(button_pins[0],INPUT_PULLUP);
  pinMode(button_pins[1],INPUT_PULLUP);
  length_dims = sizeof(dims)/sizeof(dims[0]);
  
  analogWrite(pwm_pin,0);
}

void loop() {
  if(digitalRead(button_pins[0])==0||digitalRead(button_pins[1])==0){
    if(on==0){
      int t = (t_s*255)/value;
      turn_on(value,t);
      on=1;
    }
    else{
      delay(100);
      if(digitalRead(button_pins[0])==0&&digitalRead(button_pins[1])==0){
          analogWrite(pwm_pin,max(value-50,5));
          delay(500);
          analogWrite(pwm_pin,value);
          delay(500);
          analogWrite(pwm_pin,max(value-50,5));
          delay(500);
          analogWrite(pwm_pin,value);
          while(1){
            if(digitalRead(button_pins[0])==0&&digitalRead(button_pins[1])==0) break;
            else if(digitalRead(button_pins[0])==0){
              count--;
              if (count<=0) count =0;
              value = dims[count];
              analogWrite(pwm_pin,value);
              delay(500);
            }
            else if(digitalRead(button_pins[1])==0){
              count++;
              if (count>=length_dims) count = length_dims;
              value = dims[count];
              analogWrite(pwm_pin,value);
              delay(500);
            }
            
            delay(50);
          }
          delay(50);
          analogWrite(pwm_pin,max(value-50,5));
          delay(500);
          analogWrite(pwm_pin,value);
      }
      else{
        while(digitalRead(button_pins[0])==0||digitalRead(button_pins[1])==0){
          timer++;
          delay(10);
          if(timer == 150){
          analogWrite(pwm_pin,max(value-50,5));
          delay(500);
          analogWrite(pwm_pin,value);
          delay(500);
          analogWrite(pwm_pin,max(value-50,5));
          delay(500);
          analogWrite(pwm_pin,value);
          turn = 0;
          }
          if(timer>=150){                                           
          value = dims[count];
          count++;
          if(count >= length_dims) count =0;
          analogWrite(pwm_pin,value);
          delay(1000);
          }
        }
        timer = 0;
       if(turn ==1){
        int t = (t_s*255)/value;
        turn_off(value,t);
        on=0;
       }
       turn =1;
      }
    }
    delay(50);
  }
  
  if(on==1){
    on_time++;
    if(on_time == sleep_timer_1){
      on_time = 0;
      sleep_count++;
    }
    if(sleep_count == sleep_timer_2){
      int t = (t_s*255)/value;
      turn_off(value,t*5);
      on=0;
      sleep_count = 0;
      on_time = 0;
    }
  }
  else{
    sleep_count = 0;
    on_time=0;
  }
  delay(50);
}

void turn_on(int value, int t){
  for(int i =0;i<=value;i++){
    analogWrite(pwm_pin,i);
    delay(t);
  }
  return;
}

void turn_off(int value, int t){
  for(int i =value;i>=0;i--){
    analogWrite(pwm_pin,i);
    delay(t);
  }
  return;
}