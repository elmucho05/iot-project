#include "BluetoothSerial.h"

BluetoothSerial ESP_BT; // Create a Bluetooth Serial object

byte lastStateOpenButton;
byte lastStateConfirmButton;
bool isBoxOpen = false;
bool isPillConfirmed = false;

void setup() {
  pinMode(35, INPUT_PULLUP); // Open/Close Box Button
  pinMode(34, INPUT_PULLUP); // Confirm Button
  pinMode(16, OUTPUT);       // Red LED
  pinMode(17, OUTPUT);       // Green LED
  // Initialize variables and LEDs
  lastStateOpenButton = digitalRead(35);
  lastStateConfirmButton = digitalRead(34);
  isBoxOpen = false;
  isPillConfirmed = false;
  digitalWrite(16, LOW);
  digitalWrite(17, LOW);

  Serial.begin(115200);
  Serial.println("Setup complete");
  Serial.print("Initial state of Open Button: ");
  Serial.println(lastStateOpenButton == LOW ? "Pressed" : "Released");
  Serial.print("Initial state of Confirm Button: ");
  Serial.println(lastStateConfirmButton == LOW ? "Pressed" : "Released");

  ESP_BT.begin("SmartMedsESP32"); // Name of the Bluetooth device
  Serial.println("Bluetooth device is ready to pair");

}

void loop() {
  byte currentStateOpenButton = digitalRead(35);
  byte currentStateConfirmButton = digitalRead(34);

  // Handle box open/close logic
  if (currentStateOpenButton == LOW && lastStateOpenButton == HIGH) {
    isBoxOpen = !isBoxOpen;

    if (isBoxOpen) {
      Serial.println("Box open");
      if (isPillConfirmed) {
        digitalWrite(16, LOW);  // Red LED off
        digitalWrite(17, HIGH); // Green LED on
      } else {
        digitalWrite(16, HIGH); // Red LED on
        digitalWrite(17, LOW);  // Green LED off
      }
    } else {
      Serial.println("Box closed");
      digitalWrite(16, LOW);   // Red LED off
      digitalWrite(17, LOW);   // Green LED off
    }
  }


  ESP_BT.println("hi from pc");

  /*
   if (ESP_BT.available()) {
    String received = ESP_BT.readString(); // Read incoming data
    Serial.println("Received: " + received);

    // Respond back with acknowledgment
    ESP_BT.println("Acknowledged: " + received);
  }*/
  


  delay(20); // Small delay to debounce the buttons
}
