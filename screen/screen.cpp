#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// Define the pins for the TFT display
#define TFT_CS    10  // Chip Select pin
#define TFT_RST   9   // Reset pin  
#define TFT_DC    8   // Data/Command pin
// MOSI = pin 11, MISO = pin 12, SCK = pin 13 (hardware SPI pins)

// Create display object
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  Serial.begin(9600);
  Serial.println("TFT Test!");
  
  // Initialize the display
  tft.begin();
  
  // Clear the screen with black
  tft.fillScreen(ILI9341_BLACK);
  
  // Set text color and size
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  
  // Display "Hello World!"
  tft.setCursor(50, 100);
  tft.println("Hello World!");
  
  // Draw some basic shapes
  drawBasicShapes();
  
  Serial.println("Display initialized!");
}

void loop() {
  // Simple color cycling animation
  static unsigned long lastUpdate = 0;
  static int colorIndex = 0;
  
  if (millis() - lastUpdate > 2000) {
    // Array of colors to cycle through
    uint16_t colors[] = {
      ILI9341_RED, 
      ILI9341_GREEN, 
      ILI9341_BLUE, 
      ILI9341_YELLOW, 
      ILI9341_MAGENTA, 
      ILI9341_CYAN
    };
    
    // Clear and redraw with new color
    tft.fillScreen(ILI9341_BLACK);
    
    tft.setTextColor(colors[colorIndex]);
    tft.setTextSize(3);
    tft.setCursor(30, 50);
    tft.println("Arduino");
    tft.setCursor(60, 90);
    tft.println("TFT");
    tft.setCursor(30, 130);
    tft.println("Display");
    
    // Draw a rectangle with the current color
    tft.drawRect(20, 180, 200, 80, colors[colorIndex]);
    
    colorIndex = (colorIndex + 1) % 6;
    lastUpdate = millis();
  }
}

void drawBasicShapes() {
  delay(1000);
  
  // Draw some lines
  tft.drawLine(0, 0, 240, 320, ILI9341_RED);
  tft.drawLine(240, 0, 0, 320, ILI9341_GREEN);
  
  delay(1000);
  
  // Draw circles
  tft.drawCircle(120, 160, 50, ILI9341_BLUE);
  tft.drawCircle(120, 160, 30, ILI9341_YELLOW);
  
  delay(1000);
  
  // Draw rectangles
  tft.drawRect(50, 50, 140, 100, ILI9341_CYAN);
  tft.fillRect(60, 60, 120, 80, ILI9341_MAGENTA);
  
  delay(2000);
}