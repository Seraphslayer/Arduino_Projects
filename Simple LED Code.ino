// Define LED pins
int led1 = 8;
int led2 = 9;
int led3 = 10;

void setup() {
  // Set LED pins as outputs
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  
  // Ensure all LEDs start off
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);
}

void loop() {
  // Turn on LED 1 for 2 seconds
  digitalWrite(led1, HIGH);
  delay(2000);
  digitalWrite(led1, LOW);
  
  // Turn on LED 2 for 2 seconds
  digitalWrite(led2, HIGH);
  delay(2000);
  digitalWrite(led2, LOW);
  
  // Turn on LED 3 for 2 seconds
  digitalWrite(led3, HIGH);
  delay(2000);
  digitalWrite(led3, LOW);
  
  // Optional: Add a pause before repeating
  // delay(1000);
}
