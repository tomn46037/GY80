
// Include the LCD ibrary
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// Configure the LCD library
LiquidCrystal_I2C lcd(0x21,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// Configure the WiFi
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

char ssid[] = "Gisempleq";       //  your network SSID (name)
char pass[] = "What1100Ever1";   // your network password
const int PACKET_SIZE = 64;      // This is a WAG

unsigned int localPort = 55152;   // local port to listen for UDP packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;
byte packetBuffer[PACKET_SIZE];

// tomn - defining the on disk data structure

struct AHRS_Data {
  
  unsigned long current_millis;
  float euler_Yaw;
  float euler_Pitch;
  float euler_Roll;
  
  float temperature;
  float pressure;
  float altitude;
  
} fileData;

int packetCount = 0;
unsigned long lastMillis = 0, currentMillis = 0;


void setup() {
  
  // Initialize the LCD display
  lcd.init();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("AHRS LCD Display");
  lcd.setCursor(2,1);
  lcd.print("Starting WiFi");
  
  // Initialize the WiFi on the ESP-8266
  WiFi.begin(ssid, pass);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  
  lcd.setCursor(2,1);
  lcd.print("                ");
  
  lcd.setCursor(2,1);
  lcd.println(WiFi.localIP());

  delay(5000);
  
  // Setup a listener
  udp.begin(localPort);

  initDisplay();

}

void loop() {

  // Check to see if we have a packet
  int cb = udp.parsePacket();
  if (!cb) {
    // Put in code here to blank out if it's been more then 200ms since our last packet
    Serial.println("no packet yet");
  }
  else {
    Serial.print("packet received, length=");
    Serial.println(cb);
    packetCount++;
    // We've received a packet, read the data from it
    udp.read(packetBuffer, PACKET_SIZE);

    // Save off the lastMillis number so we can figure out HZ
    lastMillis = fileData.current_millis;
    
    // Copy the packet data to the fileData structure so updateDisplay can access it    
    memcpy(&fileData, &packetBuffer, PACKET_SIZE-4);
    
  }
  
  if ( millis() > currentMillis + 500 ) {
    updateDisplay();
    currentMillis = millis();
  }

}

void initDisplayOld() {
  lcd.setCursor(0,0);
  lcd.print("ROL PIT YAW  HZ ");

  lcd.setCursor(2,1);
  lcd.print("                ");

}

void initDisplay() {
  lcd.setCursor(0,0);
  lcd.print("                ");

  lcd.setCursor(2,1);
  lcd.print("                ");

}

void updateDisplay() {

  initDisplay();

  // Only display data if it's less than 200ms since the last packet.  
  if ( fileData.current_millis-lastMillis < 200 ) {

    lcd.setCursor(0,0);
    lcd.print(fileData.euler_Roll);
    lcd.setCursor(8,0);
    lcd.print(fileData.euler_Pitch);
    lcd.setCursor(0,1);
    lcd.print(fileData.euler_Yaw);
    lcd.setCursor(8,1);
    if ( fileData.current_millis-lastMillis ) {
      lcd.print(1000/(fileData.current_millis-lastMillis));
    } else {
      lcd.print("XXXXXXXX");
    }

  } else {

    lcd.setCursor(0,0);
    lcd.print("XXXXXXXX");
    lcd.setCursor(8,0);
    lcd.print("XXXXXXXX");
    lcd.setCursor(0,1);
    lcd.print("XXXXXXXX");
    lcd.setCursor(8,1);
    lcd.print("XXXXXXXX");

  }

}
