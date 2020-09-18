#define PIN_LED 17
unsigned int count, toggle;

void setup() {
  pinMode(PIN7, OUTPUT);
  Serial.begin(9600); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.println("Hello World!");
  count = toggle = 0;
  digitalWrite(PIN7, toggle); //turn on LED
}

void loop() {
  if(count == 0){
    Serial.println(count);
    digitalWrite(PIN7, toggle);
    count += 1;
    delay(1000);
  }

  for (int i = 0; i < 10;  i++){
    if(count == 1){ 
      Serial.println(count); }
    toggle = toggle_state(toggle); 
    digitalWrite(PIN7, toggle); 
    count += 1;
    delay(100);
  }

  if(count >= 11){
    Serial.println(2);
    toggle = toggle_state(toggle); 
    digitalWrite(PIN7, toggle); 
    while(1){}
  }
}

int toggle_state(int toggle) {
  if (toggle == 0)
    return toggle = 1;
  if (toggle == 1)
    return toggle = 0;
}
