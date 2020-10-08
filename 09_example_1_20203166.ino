// Arduino pin assignment
#define PIN_LED 9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300 // maximum distance to be measured (unit: mm)
#define N 10 //N은 상수로 지정

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw, median; // unit: mm
int m, n; //m:현재 배열의 인덱스, n:총 인덱스수
float rawArray[N]; //median값 기록할 배열(C언어 배열함수는 숫자칸에 변수 못들어가서 상수 N을 넣음)
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO,INPUT);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX; 
  median = 0.0; // median:중위수
  n = N;
  m = -1; // loop에 들어갈 때 바로 +1해야하니 -1로 시작
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;

// initialize serial port
  Serial.begin(57600);

// initialize last sampling time
  last_sampling_time = 0;
}

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

// get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);

// rawArray 인덱스변수 업데이트 (가장 최근의 인덱스가 마지막 인덱스일 때 가장 나중의 인덱스부터 다시 활용)
  m += 1;
  if(m == n) m = 0;

// raw값이 0이 아닐 때 -> 바로 median값과 rawArray배열 업데이트
  if(dist_raw != 0){
    rawArray[m] = dist_raw;
    median = dist_raw;
  }
// raw값이 0일 때 
  else{
    float new_rA[N]; //중위수 판별할 새로운 배열 생성
    float x = 0; // 배열 sorting할 때 사용할 변수
    float a = 0; // rawArray배열 복사할 때 사용할 변수
    for(int i=0; i<n; i++){
      a = rawArray[i];
      new_rA[i] = a;
    } // rawArray배열 new_rA배열에 복사
    for(int j=0; j<n; j++){ // 배열 오름차순으로 sorting하기
      for(int k=0; k<(n-j); k++){ // 가장 오른쪽인덱스부터 큰수로 채워질테니 n-j범위까지 for문 사용
        if(new_rA[k] > new_rA[k+1]){ // 가장 왼쪽인덱스부터 오른쪽 인덱스값과 비교하여 높은 수는 오른쪽에 배치
          x = new_rA[k];
          new_rA[k] = new_rA[k+1];
          new_rA[k+1] = x;
        }
      }
    }
    median = new_rA[n/2]; // n이 짝수일 시 중위수 숫자는 2개이나 1개밖에 선택 못하니 n/2번째 인덱스값 선택
    rawArray[m] = median; // rawArray배열 업데이트
  }

// output the read value to the serial port
  Serial.print("Min:0,");
  Serial.print("raw:");
  Serial.print(dist_raw);
  Serial.print(",");
  Serial.print("median:");
  Serial.print(map(median,0,400,100,500));
//  Serial.print(map(dist_ema,0,400,100,500));
  Serial.print(",");
  Serial.println("Max:500");

// turn on the LED if the distance is between dist_min and dist_max
  if(dist_raw < dist_min || dist_raw > dist_max) {
    analogWrite(PIN_LED, 255);
  }
  else {
    analogWrite(PIN_LED, 0);
  }

// update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  reading = pulseIn(ECHO, HIGH, timeout) * scale; // unit: mm
  if(reading < dist_min || reading > dist_max) reading = 0.0; // return 0 when out of range.
  return reading;
}
