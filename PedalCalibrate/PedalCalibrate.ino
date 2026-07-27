// Analog Read Example for nRF52840 Feather Express
// Reads analog voltage from pin A1 and prints it to Serial
#include <Adafruit_TinyUSB.h>
void setup() {
  // Start the Serial monitor
  Serial.begin(115200);
  while (!Serial) {
    // Wait for Serial to be ready
  }

  // Initialize analog pin A1
  pinMode(A5, INPUT);

  Serial.println("Analog read example started. Reading from pin A1...");
}

void loop() {
  // Read analog value (0–1023 for 10-bit ADC)
  int analogValue = analogRead(A5);

  // Print the value to the serial monitor
  Serial.print("Analog value from A0: ");
  Serial.println(analogValue);

  // Delay to avoid spamming the serial monitor
  delay(500);
}