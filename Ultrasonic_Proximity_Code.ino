// Pin definitions
const int trigPin = 9;      // Ultrasonic trigger pin
const int echoPin = 10;     // Ultrasonic echo pin
const int redLED = 7;       // Red LED pin
const int greenLED = 6;     // Green LED pin
const int buzzer = 8;       // Buzzer pin

// Distance thresholds 
const int maxDistance = 400; // Maximum detection distance (ultrasonic sensor max range)
const int minDistance = 2;   // Minimum distance for fastest beep

// Variables
long duration;
int distance;
unsigned long previousMillis = 0;
int beepInterval = 1000;    // Initial beep interval in milliseconds

void setup() {
  // Initialize the pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(buzzer, OUTPUT);
  
  // Initialize the serial communication for debugging
  Serial.begin(9600);
  
  // Startsd with green LED on (no object detected)
  digitalWrite(greenLED, HIGH);
  digitalWrite(redLED, LOW);
}

void loop() {
  // Measuring the distance
  distance = getDistance();
  
  // Printing the distance 
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  // Checks if object is detected 
  if (distance > 0 && distance <= maxDistance) {
    // Object detected - turn on red LED, turn off green LED
    digitalWrite(redLED, HIGH);
    digitalWrite(greenLED, LOW);
    
    // Calculate beep interval based on distance
    // Closer objects = faster beeping
    beepInterval = map(distance, minDistance, maxDistance, 50, 2000);
    beepInterval = constrain(beepInterval, 50, 2000);
    
    // Handle buzzer beeping with non-blocking delay
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= beepInterval) {
      previousMillis = currentMillis;
      tone(buzzer, 1000, 100); // Beep at 1000Hz for 100ms
    }
  } else {
    // No object detected - turn on green LED, turn off red LED
    digitalWrite(greenLED, HIGH);
    digitalWrite(redLED, LOW);
    noTone(buzzer); // Stop buzzer
  }
  
  delay(50); // Small delay for stability
}

int getDistance() {
  // Clear the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Send 10 microsecond pulse to trigger
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echo pin
  duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30ms
  
  // Calculate distance nya sa cm
  // Speed of sound is 343 m/s or 0.0343 cm/µs
  // Distance = (duration / 2) / 29.1
  int dist = duration * 0.034 / 2;
  
  return dist;
}
