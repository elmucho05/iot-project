#include "BluetoothSerial.h"

BluetoothSerial ESP_BT;


enum State { CLOSED_OFF, OPEN_RED, OPEN_GREEN, CLOSED_TAKEN }; // states of FSM

// Define the Compartment structure
struct Compartment {
  int redLedPin;
  int greenLedPin;
  int openButtonPin;
  int confirmButtonPin;
  State currentState;
  byte lastStateOpenButton;
  byte lastStateConfirmButton;

  void initialize(int redPin, int greenPin, int openPin, int confirmPin) {
    redLedPin = redPin;
    greenLedPin = greenPin;
    openButtonPin = openPin;
    confirmButtonPin = confirmPin;
    currentState = CLOSED_OFF;
    lastStateOpenButton = HIGH;
    lastStateConfirmButton = HIGH;

    pinMode(redLedPin, OUTPUT);
    pinMode(greenLedPin, OUTPUT);
    pinMode(openButtonPin, INPUT_PULLUP);
    pinMode(confirmButtonPin, INPUT_PULLUP);

    digitalWrite(redLedPin, LOW);
    digitalWrite(greenLedPin, LOW);
  }

  void updateState(byte currentStateOpenButton, byte currentStateConfirmButton) {
    switch (currentState) {
      case CLOSED_OFF:
        // when i press open button
        if (currentStateOpenButton == LOW && lastStateOpenButton == HIGH) {
          currentState = OPEN_RED;
          digitalWrite(redLedPin, HIGH);
          digitalWrite(greenLedPin, LOW);
          Serial.println("State: OPEN_RED (Box Opened, Red LED On)");
        }
        break;

      case OPEN_RED:
        //after i open the box, if i take a pill
        if (currentStateConfirmButton == LOW && lastStateConfirmButton == HIGH) {
          currentState = OPEN_GREEN;
          digitalWrite(redLedPin, LOW);
          digitalWrite(greenLedPin, HIGH);
          Serial.println("State: OPEN_GREEN (Pill Confirmed, Green LED On)");
        }
        // if i close the box, go the the state before
        else if (currentStateOpenButton == LOW && lastStateOpenButton == HIGH) {
          currentState = CLOSED_OFF;
          digitalWrite(redLedPin, LOW);
          digitalWrite(greenLedPin, LOW);
          Serial.println("State: CLOSED_OFF (Box Closed, Lights Off)");
        }
        break;

      case OPEN_GREEN:
        // while on green led, if i press again the green button, nothing happens, stays on the same state
        if (currentStateConfirmButton == LOW && lastStateConfirmButton == HIGH) {
          Serial.println("State: OPEN_GREEN (Pill Already Confirmed, Green LED On)");
        }
        // if i have taken the pill and then i close the box, go to state 4 where pill has been taken and leds are off
        else if (currentStateOpenButton == LOW && lastStateOpenButton == HIGH) {
          currentState = CLOSED_TAKEN;
          digitalWrite(redLedPin, LOW);
          digitalWrite(greenLedPin, LOW);
          Serial.println("State: CLOSED_TAKEN (Box Closed, Pill Taken)");
        }
        break;

      case CLOSED_TAKEN:
        // pill taken, open box so go to state 3
        if (currentStateOpenButton == LOW && lastStateOpenButton == HIGH) {
          currentState = OPEN_GREEN;
          digitalWrite(redLedPin, LOW);
          digitalWrite(greenLedPin, HIGH);
          Serial.println("State: OPEN_GREEN (Box Opened, Green LED On)");
        }
        break;
    }

    // Update last button states
    lastStateOpenButton = currentStateOpenButton;
    lastStateConfirmButton = currentStateConfirmButton;
  }
};

Compartment compartments[1]; //will have 3 compartments
unsigned long timestamp = 0; // For periodic Bluetooth data transmission


void setup() {
  Serial.begin(115200);
  ESP_BT.begin("SmartMedsESP32");
  Serial.println("Bluetooth device is ready to pair");

  // Initialize compartment and it's  pins
  compartments[0].initialize(16, 17, 35, 34); // red green open confirm
  Serial.println("System initialized. All compartments are closed, and LEDs are off.");
}

void loop() {
  for (int i = 0; i < 1; i++) { // Loop through compartments
    byte currentStateOpenButton = digitalRead(compartments[i].openButtonPin);
    byte currentStateConfirmButton = digitalRead(compartments[i].confirmButtonPin);
    
    // just call the fsm update
    compartments[i].updateState(currentStateOpenButton, currentStateConfirmButton);
  }

    if (millis() - timestamp > 5000) {
    for (int i = 0; i < 1; i++) { // Send state of each compartment
      ESP_BT.write(0xFF);                   // Start byte
      ESP_BT.write(i + 1);                  // Compartment ID
      ESP_BT.write(compartments[i].currentState); // Current FSM state
      ESP_BT.write(0xFE);                   // End byte
    }
    Serial.println("Bluetooth data sent.");
    timestamp = millis();
    
  }

  // Handle incoming Bluetooth data
  if (ESP_BT.available()) {
    String received = ESP_BT.readString(); // Read incoming data
    Serial.println("Received: " + received);

    // Example: Parse received data for specific commands
    if (received == "RESET") {
      for (int i = 0; i < 1; i++) {
        compartments[i].currentState = CLOSED_OFF; // Reset to initial state
        digitalWrite(compartments[i].redLedPin, LOW);
        digitalWrite(compartments[i].greenLedPin, LOW);
        Serial.println("Compartment reset to CLOSED_OFF state.");
      }
    }

    // Respond back with acknowledgment
    ESP_BT.println("Acknowledged: " + received);
  }

  delay(20);
}
