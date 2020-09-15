#define PIN_LED 13
unsigned int count, toggle;

void setup() {
  pinMode(PIN7, OUTPUT);
  Serial.begin(4800); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.println("Hello World!");
  count = 0;
  toggle = 0;
  digitalWrite(PIN7, toggle);
}


void loop() {
  if(count == 0){
    Serial.println(count);
    digitalWrite(PIN7, toggle);
    delay(1000);
  }
  
  if(0< count < 10){
    Serial.println(++count);
    toggle = toggle_state(toggle);
    digitalWrite(PIN7, toggle);
    delay(100);
  }
  
  if (count >= 10){
    Serial.println(++count);
    toggle = toggle_state(toggle);
    digitalWrite(PIN7,toggle);
    while(1){}
  }
}

int toggle_state(int toggle) {
  if (toggle == 0)
    return toggle = 1;
  if (toggle == 1)
    return toggle = 0;
}
