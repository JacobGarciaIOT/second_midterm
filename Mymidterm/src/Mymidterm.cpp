#include "Particle.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_GFX.h"
#include "Grove_Air_quality_Sensor.h"
#include "Adafruit_BME280.h"
#include "neopixel.h"

// SYSTEM SETTINGS
SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

// PINS & CONSTANTS
const int MOTOR_PIN = D10;       // Motor (water pump) pin
const int MOISTURE_PIN = A1;     // Moisture sensor pin
const int AIR_QUALITY_PIN = A2;  // Air quality sensor pin
const int PIXEL_PIN = D2;        // NeoPixel ring pin
const int PIXELCOUNT = 11;
const int OLED_RESET = -1;
const int MOISTURE_THRESHOLD = 1759; // Adjust threshold as necessary

unsigned int lastMoiCheck;

// SENSORS & COMPONENTS
Adafruit_BME280 bme;
Adafruit_SSD1306 display(OLED_RESET);
Adafruit_NeoPixel pixel(PIXELCOUNT, SPI1, WS2812B);
AirQualitySensor airSensor(AIR_QUALITY_PIN);

// Function to smoothly update NeoPixel ring color based on moisture level
void updatePixelRingColor(int moistureLevel);

void setup() {
  
  Serial.begin(9600);
    waitFor(Serial.isConnected, 10000);


  Serial.println("Starting setup...");
  Wire.begin();
  pinMode(MOTOR_PIN, OUTPUT);       // Water pump motor pin setup
  pinMode(MOISTURE_PIN, INPUT);      // Moisture sensor setup
  digitalWrite(MOTOR_PIN, LOW);      // Ensure pump is off initially
  
  Particle.syncTime();
  Time.zone(-7);                     // Set time zone

  // Display & Pixel Initialization
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Initialize display
  display.display();
  delay(5000);
  
 display.clearDisplay();
 display.setTextSize(1);
 display.setTextColor(WHITE);
 display.setCursor(0, 0);

  pixel.begin();
  pixel.setBrightness(255);           // Set brightness
  pixel.show();
  pixel.clear();

  // Initialize BME280 sensor
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);  // Halt if sensor init fails
  } else {
    Serial.println("BME280 sensor initialized.");
  }
}

void loop() {
  unsigned int currentTime = millis();

  // Moisture check every 60 seconds for pumping logic
  if ((currentTime - lastMoiCheck) > 60000) {
    lastMoiCheck = millis();
    
    int moistureLevel = analogRead(MOISTURE_PIN); // Read moisture level
    Serial.printf("Moisture Level: %i\n", moistureLevel);

    float temperature = bme.readTemperature();    // Read temperature
    Serial.printf("Temperature: %.1f C\n", temperature);

    // Pump logic based on moisture level
    if (moistureLevel > MOISTURE_THRESHOLD) {
      digitalWrite(MOTOR_PIN, HIGH);  // Turn on pump
      delay(500);                     // Keep it on briefly
      digitalWrite(MOTOR_PIN, LOW);   // Turn off pump
      Serial.println("Pump activated due to low moisture.");
    }

    // Update OLED display with current moisture level and temperature
    display.clearDisplay();
    display.setCursor(0, 0);
    display.printf("Moisture: %i\n", moistureLevel);
    display.setCursor(0, 10);
    display.printf("Temp: %.1f C\n", temperature);
    display.display();
    Serial.println("OLED display updated with moisture and temperature.");
  }

  // Constantly update NeoPixel ring color based on moisture level
  int moistureLevel = analogRead(MOISTURE_PIN);
  updatePixelRingColor(moistureLevel);

  delay(50);  // Small delay for smooth transitions
}

// Smoothly map moisture level to a color (purple to red) and update the NeoPixel ring
void updatePixelRingColor(int moistureLevel) {
  // Map the moisture level to color values from purple (wet) to red (dry)
  int red = map(moistureLevel, 0, 4095, 128, 255);    // Red increases as soil gets dry
  int green = map(moistureLevel, 0, 4095, 0, 64);     // Small amount of green to create orange shades in the middle
  int blue = map(moistureLevel, 0, 4095, 255, 0);     // Blue decreases as soil gets dry

  // Apply the color to all pixels in the ring
  for (int i = 0; i < PIXELCOUNT; i++) {
    pixel.setPixelColor(i, pixel.Color(red, green, blue));
  }
  pixel.show();
}
