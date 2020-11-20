#include <Servo.h>

//ema필터, 노이즈필터, 서보속도 제어, 샘플링주기 개별설정, true or false 
//적외선 디스탠스, 서보모터
//적외선센서에는 dist_prev, dist_raw, dist_min, dist_max
//서보모터 duty_max, duty_min, duty_target, duty_curr
//ema필터에는 alpha, dist_ema, dist_cali
//노이즈필터 ex_reading
//서보속도 제어 servo_speed,servo_angle

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//프레임워크 어퍼암길이를 수정하여 11주 1강 기록이 달라졌고, 적외선센서 거리측정도 가조립때 기록과 많이 달라져 현재 기록에 맞춰 코드짰습니다.//
//프레임워크 : 300, 195, 123 - r:30기준, 적외선센서 a: 68, b: 240 기준                                                       //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Arduino pin assignment
#define PIN_LED 9 
#define PIN_SERVO 10
#define PIN_IR A0

// Configurable parameters
#define _DIST_MIN 68 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 240 // maximum distance to be measured (unit: mm)
#define _DIST_TARGET 255 // target distance (unit: mm)
#define _DIST_ALPHA 0.5 

#define _DUTY_MAX 1878 // servo at framework counterclockwise position (_DUTY_NEU + r')
#define _DUTY_MIN 1262 // servo at framework clockwise position (_DUTY_NEU - r')
#define _DUTY_NEU 1570 // servo at framework neutral position (90 degree at framework)
//min, max, neu 기준은 프레임워크 수평 +- r'

#define _SERVO_SPEED 16 // servo speed limit (unit: degree/second)
#define _SERVO_ANGLE 60

#define _INTERVAL_DIST 25 // IR interval (unit: ms)
#define _INTERVAL_SERVO 20 // servo interval (unit: ms)
#define _INTERVAL_SERIAL 100 // serial interval (unit: ms)

// global variables
Servo myservo;

float dist_target, dist_raw, dist_ema, dist_celi; // unit: mm
float alpha;

int duty_chg_per_interval; // maximum duty difference per interval
int duty_target, duty_curr; 

unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; // unit: ms
bool event_dist, event_servo, event_serial;

int a, b;


void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);

  // initialize serial port
  last_sampling_time_dist=last_sampling_time_servo=last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;

  // initialize IR related variables
  dist_raw = 0.0; // raw distance output from IR (unit: mm)
  dist_target = _DIST_TARGET; // unit: mm
  alpha = _DIST_ALPHA;
  a = _DIST_MIN;
  b = _DIST_MAX;

  // convert angle speed into duty change per interval
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / _SERVO_ANGLE * _INTERVAL_SERVO / 1000;  

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
// wait until next sampling time
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO){
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL){
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }


  if(event_dist){
    event_dist = false;
    // get a distance reading from the IR
    dist_raw = ir_distance();
    dist_celi = 100 + 300.0 / (b - a) * (dist_raw - a);
    dist_ema = (alpha * dist_celi) + (1-alpha)*dist_ema;
    if(dist_ema < dist_target){duty_target = _DUTY_MAX;}
    else{duty_target = _DUTY_MIN;}
  }
  
  if(event_servo){
    event_servo = false;
    // adfust servo position : duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr){
      duty_curr = duty_curr + duty_chg_per_interval;
      if(duty_curr > duty_target){duty_curr = duty_target;}
    }
    else{
      duty_curr = duty_curr - duty_chg_per_interval;
      if(duty_curr<duty_target){duty_curr = duty_target;}
    }

    //update servo position
    myservo.writeMicroseconds(duty_curr);
  }

  if(event_serial){
    event_serial = false;
    // output the read value to the serial port
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",dist_celi:");
    Serial.print(dist_celi);
    Serial.print(",dist_ema:");
    Serial.print(dist_ema);
    Serial.print(",duty_target:");
    Serial.print(duty_target);
    Serial.print(",duty_curr:");
    Serial.print(duty_curr);
    Serial.print(",duty_chg_per_interval");
    Serial.println(duty_chg_per_interval);
  }
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

float ir_distance_filtered(void){
  float reading = ir_distance();
  float ex_reading;
  if(reading >= _DIST_MIN & reading <= _DIST_MAX) ex_reading = reading;
  if(reading<=_DIST_MIN || reading > _DIST_MAX) reading = 0.0;
  if(reading == 0.0) reading = ex_reading;
  return reading;
}
