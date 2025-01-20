#include "BluetoothSerial.h"

BluetoothSerial ESP_BT; // Create a Bluetooth Serial object

// Define the Compartment structure
struct Compartment {
  int redLedPin;
  int greenLedPin;
  int openButtonPin;
  int confirmButtonPin;
  bool isBoxOpen=false;
  bool isPillConfirmed = false;
  byte lastStateOpenButton;
  byte lastStateConfirmButton;

  // Initialize the compartment with the relevant pins
  void initialize(int redPin, int greenPin, int openPin, int confirmPin) {
    redLedPin = redPin;
    greenLedPin = greenPin;
    openButtonPin = openPin;
    confirmButtonPin = confirmPin;
    isBoxOpen = false;
    isPillConfirmed = false;
    lastStateOpenButton = LOW;
    lastStateConfirmButton = LOW;

    pinMode(redLedPin, OUTPUT);
    pinMode(greenLedPin, OUTPUT);
    pinMode(openButtonPin, INPUT_PULLUP);
    pinMode(confirmButtonPin, INPUT_PULLUP);

    // Set initial LED state (off because box is closed)
    digitalWrite(redLedPin, LOW);
    digitalWrite(greenLedPin, LOW);
  }

  // Open the compartment
  void openCompartment() {
    isBoxOpen = true;
    if (isPillConfirmed) {
      digitalWrite(redLedPin, LOW);  // Red LED off
      digitalWrite(greenLedPin, HIGH); // Green LED on
    } else {
      digitalWrite(redLedPin, HIGH); // Red LED on
      digitalWrite(greenLedPin, LOW);  // Green LED off
    }
  }

  // Close the compartment
  void closeCompartment() {
    isBoxOpen = false;
    digitalWrite(redLedPin, LOW);   // Turn off red LED
    digitalWrite(greenLedPin, LOW); // Turn off green LED
  }

  // Confirm pill taken
  void confirmPillTaken() {
    if (isBoxOpen) {
      isPillConfirmed = true;
      digitalWrite(redLedPin, LOW);  // Turn off red LED
      digitalWrite(greenLedPin, HIGH); // Turn on green LED
    }
  }
    void pillNotTaken() {
    if (isBoxOpen) {
      isPillConfirmed = false;
      digitalWrite(redLedPin, HIGH);  // Turn off red LED
      digitalWrite(greenLedPin, LOW); // Turn on green LED
    }
  }
};

// Create an array of 3 compartments
Compartment compartments[3];

void setup() {
  Serial.begin(115200);
  ESP_BT.begin("SmartMedsESP32");
  Serial.println("Bluetooth device is ready to pair");

  // Initialize the compartments with their respective pins
                          //red  green open confirm
  compartments[0].initialize(16, 17, 35, 34); // Pins for compartment 1
  //compartments[1].initialize(18, 19, 36, 37); // Pins for compartment 2
  //compartments[2].initialize(20, 21, 38, 39); // Pins for compartment 3

  Serial.println("System initialized. All compartments are closed, and LEDs are off.");
}

void loop() {
  for (int i = 0; i < 3; i++) {
    // Read button states
    byte currentStateOpenButton = digitalRead(compartments[i].openButtonPin);
    byte currentStateConfirmButton = digitalRead(compartments[i].confirmButtonPin);

    // Check for open/close button press (state transition)
    if (currentStateOpenButton == LOW && compartments[i].lastStateOpenButton == HIGH) {
      delay(50); // Debounce
      compartments[i].isBoxOpen = !compartments[i].isBoxOpen;

      if(compartments[i].isBoxOpen){
        Serial.println("Box Opened");
        if(compartments[i].isPillConfirmed){
          compartments[i].confirmPillTaken();
        }else{
          compartments[i].pillNotTaken();
        }
      }else{
        Serial.println("Box closed");
        compartments[i].closeCompartment();
      }
    }
    
    // Check for confirm button press (state transition)
    if (compartments[i].isBoxOpen && currentStateConfirmButton == LOW && compartments[i].lastStateConfirmButton == HIGH) {
      delay(50); // Debounce
      if (digitalRead(compartments[i].confirmButtonPin) == LOW) { // Confirm button press
        compartments[i].confirmPillTaken();
        Serial.print("Pill confirmed in Compartment ");
        Serial.println(i + 1);
      }
    }

    // Update the last button states
    compartments[i].lastStateOpenButton = currentStateOpenButton;
    compartments[i].lastStateConfirmButton = currentStateConfirmButton;
  }

  delay(20); // General debounce for the loop
}
