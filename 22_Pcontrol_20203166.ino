#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9     
#define PIN_SERVO 10   
#define PIN_IR A0    

// Framework setting
#define _DIST_TARGET 255  
#define _DIST_MIN 100      
#define _DIST_MAX 410 

// Distance sensor
#define _DIST_ALPHA 0.5 
#define LENGTH 30
#define k_LENGTH 8

// Servo range
#define _DUTY_MIN 1000   
#define _DUTY_NEU 1570    
#define _DUTY_MAX 2000   

// Servo speed control
#define _SERVO_ANGLE 20  
#define _SERVO_SPEED 50   

// Event periods
#define _INTERVAL_DIST 20  
#define _INTERVAL_SERVO 20 
#define _INTERVAL_SERIAL 100 

// PID parameters
#define _KP 1.2   
#define _KI 0.0
#define _KD 0.0


//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;  
// Distance sensor
float dist_target; // location to send the ball 
float dist_raw, dist_ema, dist_celi;    

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial; 

// Servo speed control
int duty_chg_per_interval;  
int duty_target, duty_curr;   

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm; 
int correction_dist, iter, a, b;
float dist_list[LENGTH], sum, alpha;

void setup() {
// initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED, OUTPUT);  
  myservo.attach(PIN_SERVO); 
// initialize global variables
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0; 
  event_dist = event_servo = event_serial = false;  
  dist_target = _DIST_TARGET; 
 
  correction_dist = 0;
  iter = 0; sum = 0;
  alpha = _DIST_ALPHA;

// move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU); 
// initialize serial port
  Serial.begin(57600);                          
// convert angle speed into duty change per interval.
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / _SERVO_ANGLE * _INTERVAL_SERVO / 1000;   
  
}
  

void loop() {
/////////////////////
// Event generator // 
/////////////////////
unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }


////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
    event_dist = false;
  // get a distance reading from the distance sensor
    dist_raw = ir_distance_filtered();  

  // PID control logic
    error_curr = dist_target - dist_ema; 
    pterm = _KP*error_curr; 
    control = pterm; // + iterm + dterm 

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + (control) * (_SERVO_ANGLE * 10.3) / (dist_target - _DIST_MIN);

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target < _DUTY_MIN) {
      duty_target = _DUTY_MIN;  
    } else if (duty_target > _DUTY_MAX) {
      duty_target = _DUTY_MAX; 
    } 
  }
  
  if(event_servo) {
    event_servo = false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }  
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    } 
    
    // update servo position
    myservo.writeMicroseconds(duty_curr);
  }
  
  if(event_serial) {
    event_serial = false;
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410,");
  }
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;     
}

float ir_distance_filtered() {
  a = 76;
  b = 331;
  sum = 0;
  iter = 0;
  while (iter < LENGTH)
  {
    dist_list[iter] = 100 + 300.0 / (b - a) * (ir_distance() - a);
    sum += dist_list[iter];
    iter++;
  }

  for (int i = 0; i < LENGTH-1; i++){
    for (int j = i+1; j < LENGTH; j++){
      if (dist_list[i] > dist_list[j]) {
        float tmp = dist_list[i];
        dist_list[i] = dist_list[j];
        dist_list[j] = tmp;
      }
    }
  }
  
  for (int i = 0; i < k_LENGTH; i++) {
    sum -= dist_list[i];
  }
  for (int i = 1; i <= k_LENGTH; i++) {
    sum -= dist_list[LENGTH-i];
  }

  float dist_cali = sum/(LENGTH-2*k_LENGTH);
  dist_ema = alpha*dist_cali + (1-alpha)*dist_ema;
  return dist_ema;
}
