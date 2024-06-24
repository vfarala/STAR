#include <Wire.h>              // Library for I2C communication
#include <Adafruit_GFX.h>      // Graphics library for OLED
#include <Adafruit_SSD1306.h>  // OLED display library
#include <math.h>              // Math library for calculations
#include <BluetoothSerial.h>   // Bluetooth serial library for ESP32
#include <esp_bt_device.h>     // ESP32 Bluetooth device library

BluetoothSerial SerialBT;  // Instantiate BluetoothSerial object

#define SCREEN_WIDTH 128  // OLED display width in pixels
#define SCREEN_HEIGHT 64  // OLED display height in pixels

#define OLED_RESET -1  // OLED reset pin (-1 if not used)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define UpperThreshold 560   // Upper threshold for pulse detection
#define LowerThreshold 530   // Lower threshold for pulse detection
#define PULSE_SENSOR_PIN 13  // Pin for pulse sensor

#define ECG_SAMPLE_SIZE 128  // Number of samples for ECG waveform
#define ECG_AMPLITUDE 20     // Amplitude for ECG waveform

#define motor1 12  // Vibration motor #1 pin
#define motor2 14  // Vibration motor #2 pin

unsigned long lastBeatTime = 0;       // Time of last heartbeat detected
unsigned long intervalStartTime = 0;  // Start time of the interval
int bpm = 0;                          // Beats per minute
int ecgWaveform[ECG_SAMPLE_SIZE];     // Array to store ECG waveform data
int ecgIndex = 0;                     // Index for ECG waveform array
int pulseCount = 0;                   // Pulse count in current interval
int interval = 10000;                 // Interval time in milliseconds (10 sec)
float decayRate = 0.9;                // Decay rate for BPM if no pulse detected

float sdnn = 0.0;         // Standard deviation of NN intervals
float rmssd = 0.0;        // Root mean square of successive differences
String stressLevel = "";  // Stress level

unsigned long nnIntervals[ECG_SAMPLE_SIZE];  // Array to store NN intervals
int nnIndex = 0;                             // Index for NN intervals array

unsigned long lastTransmissionTime = 0;           // Last Bluetooth transmission time
const unsigned long transmissionInterval = 5000;  // Interval for Bluetooth transmission

void setup() {
  Serial.begin(115200);  // Start serial communication

  // Initialize Bluetooth with the device name "ESP32_BPM"
  SerialBT.begin("ESP32_BPM");

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Loop forever if OLED initialization fails
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  pinMode(motor1, OUTPUT);  // Set motor1 pin as output
  pinMode(motor2, OUTPUT);  // Set motor2 pin as output
}

void loop() {
  unsigned long currentMillis = millis();  // Current time in milliseconds

  int pulseValue = analogRead(PULSE_SENSOR_PIN);  // Read pulse sensor value
  Serial.println(pulseValue);                     // Print pulse value for debugging

  // Update ECG waveform array with new sample
  ecgWaveform[ecgIndex] = map(pulseValue, 0, 1023, 0, ECG_AMPLITUDE);
  ecgIndex = (ecgIndex + 1) % ECG_SAMPLE_SIZE;  // Loop back to start of array

  // Detect pulse
  if (pulseValue > UpperThreshold && (currentMillis - lastBeatTime) > 600) {
    unsigned long nnInterval = currentMillis - lastBeatTime;
    lastBeatTime = currentMillis;

    nnIntervals[nnIndex] = nnInterval;          // Store NN interval
    nnIndex = (nnIndex + 1) % ECG_SAMPLE_SIZE;  // Loop back to start of array
    pulseCount++;
  }

  // Check if the interval has passed
  if (currentMillis - intervalStartTime >= interval) {
    intervalStartTime = currentMillis;  // Reset interval start time
    if (pulseCount > 0) {
      bpm = (pulseCount * 60) / (interval / 1000);  // Calculate BPM
    } else {
      bpm *= decayRate;      // Decay BPM if no pulse detected
      if (bpm < 1) bpm = 0;  // Ensure BPM doesn't go below 0
    }

    computeHRV();                                  // Compute HRV metrics
    Serial.println("SDNN: " + String(sdnn));    // Print SDNN with 3 decimal places
    Serial.println("RMSSD: " + String(rmssd));  // Print RMSSD with 3 decimal places

    // Determine stress level based on HRV metrics
    if (bpm > 0) {
      if (bpm > 90 && sdnn < 50 && rmssd < 30) {
        stressLevel = "Very High";
      } else if (bpm > 80 && sdnn >= 50 && sdnn <= 100 && rmssd >= 30 && rmssd <= 70) {
        stressLevel = "High";
      } else if (bpm > 70 && sdnn > 100 && rmssd > 70) {
        stressLevel = "Moderate";
      } else {
        stressLevel = "Low";
      }
    } else {
      stressLevel = "None";
    }

    // Check for flatline or minimal spikes in ECG waveform
    bool minimalVariation = true;
    for (int i = 1; i < ECG_SAMPLE_SIZE; i++) {
      if (abs(ecgWaveform[i] - ecgWaveform[i - 1]) > 1) {  // Allow minimal variation
        minimalVariation = false;
        break;
      }
    }
    if (minimalVariation) {
      bpm = 0;
      stressLevel = "None";
    }

    pulseCount = 0;  // Reset pulse count for the next interval
  }

  // Control vibration motors based on stress level
  controlVibeMotors(stressLevel);

  // Display ECG waveform and BPM on OLED
  updateDisplay();

  // Check if it's time to send data over Bluetooth
  if (currentMillis - lastTransmissionTime >= transmissionInterval) {
    lastTransmissionTime = currentMillis;  // Update last transmission time
    sendBluetoothData();                   // Send data over Bluetooth
  }
}

void computeHRV() {
  if (nnIndex < 2) {  // If not enough NN intervals
    sdnn = 0;
    rmssd = 0;
    return;
  }

  float sumNN = 0.0;
  float sumSquareDiff = 0.0;

  // Calculate mean of NN intervals
  for (int i = 0; i < nnIndex; i++) {
    sumNN += nnIntervals[i];
  }

  float meanNN = sumNN / nnIndex;

  // Calculate SDNN
  for (int i = 0; i < nnIndex; i++) {
    sumSquareDiff += pow(nnIntervals[i] - meanNN, 2);
  }

  sdnn = sqrt(sumSquareDiff / (nnIndex - 1));  // Standard deviation (corrected for population)

  // Calculate RMSSD
  float sumSquareDiffSuccessive = 0.0;
  for (int i = 1; i < nnIndex; i++) {
    sumSquareDiffSuccessive += pow(nnIntervals[i] - nnIntervals[i - 1], 2);
  }

  // Truncate RMSSD to three digits
  rmssd = int(sqrt(sumSquareDiffSuccessive / (nnIndex - 1))) % 1000;
}


void controlVibeMotors(String stressLevel) {
  if (stressLevel == "Very High") {
    digitalWrite(motor1, HIGH);  // Turn on both motors for very high stress
    digitalWrite(motor2, HIGH);
  } else if (stressLevel == "High") {
    digitalWrite(motor1, HIGH);  // Turn on only motor1 for high stress
    digitalWrite(motor2, LOW);
  } else {
    digitalWrite(motor1, LOW);  // Turn off both motors for moderate or low stress
    digitalWrite(motor2, LOW);
  }
}

void updateDisplay() {
  display.clearDisplay();  // Clear the OLED display

  // Display ECG waveform
  for (int i = 1; i < ECG_SAMPLE_SIZE; i++) {
    display.drawLine(i - 1, 40 - ecgWaveform[i - 1], i, 40 - ecgWaveform[i], SSD1306_WHITE);
  }

  display.drawLine(0, 40, 127, 40, SSD1306_BLACK);  // Draw divider line

  display.setTextSize(1);
  display.setCursor(0, 45);
  display.print("BPM: ");  // Display BPM
  display.println(bpm);

  display.setCursor(0, 55);
  display.print("Stress: ");  // Display stress level
  display.println(stressLevel);

  display.display();  // Update the display
}

void sendBluetoothData() {
  String data = String(bpm) + "," + stressLevel;  // Format data as CSV
  SerialBT.println(data);                         // Send data over Bluetooth
}