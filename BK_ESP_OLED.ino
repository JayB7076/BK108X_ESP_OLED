#include <Adafruit_SSD1306.h>
#include <splash.h>
#include <BK108X.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include "Rotary.h"

const int SDA_PIN = 5;
const int SCL_PIN = 15; 

// Enconder PINs
#define ENCODER_PIN_A 34
#define ENCODER_PIN_B 35

// Buttons controllers
#define VOLUME_UP 27      // Volume Up
#define VOLUME_DOWN 26    // Volume Down
#define BAND_MODE_SWITCH_UP 25     // Next Band
#define BAND_MODE_SWITCH_DOWN 33  // Previous Band
#define SEEK_FUNCTION 36  //Encoder push button

#define DEFAULT_VOLUME_LEVEL 25

#define POLLING_TIME 1000
#define RDS_MSG_TYPE_TIME 25000
#define POLLING_RDS 250

#define STORE_TIME 10000  // Time of inactivity to make the current receiver status writable (10s / 10000 milliseconds).
#define PUSH_MIN_DELAY 300

#define EEPROM_SIZE 512
#define MIN_ELAPSED_TIME 150

bool DisplayStyle = false;

uint8_t rssi;
uint8_t snr;

/*
 * Structure used to refers a band / mode of the receiver
*/

typedef struct
{
  uint8_t  mode; // Bande mode.
  char    *name;  
  uint32_t minimum_frequency; // Minimum frequency to the band (KHz)
  uint32_t maximum_frequency; // Maximum frequency to the band (KHz)
  uint32_t default_frequency; // default frequency (KHz)
  uint16_t step;               // step used (KHz)
  uint8_t band_space;           // AM: (0=1KHz; 1 = 5KHz; 2=9KHz; 3 = 10KHz) | FM: Not used here.
} tabBand;

tabBand band[] = {
  {BK_MODE_FM, (char *) "FM", 6400, 10800, 9830, 10, 2}, // 100kHz - Band space
  {BK_MODE_AM, (char *) "MW1", 520, 1710, 810, 10, 3},    // 10 kHz
  {BK_MODE_AM, (char *) "MW2", 522, 1710, 810,  1, 2},    // MW/AM - Europe - 9kHz
  {BK_MODE_AM, (char *) "60m", 4700, 5600, 4885, 5, 1},   // Always 5 kHz
  {BK_MODE_AM, (char *) "49m", 5700, 6400, 6100, 5, 1},
  {BK_MODE_AM, (char *) "41m", 6800, 8200, 7205, 5, 1},
  {BK_MODE_AM, (char *) "31m", 9200, 10500, 9600, 5, 1},
  {BK_MODE_AM, (char *) "25m", 11400, 12200, 11940, 5, 1},
  {BK_MODE_AM, (char *) "22m", 13400, 14300, 13600, 5, 1},
  {BK_MODE_AM, (char *) "19m", 15000, 16100, 15300, 5, 1},
  {BK_MODE_AM, (char *) "16m", 17400, 17900, 17600, 5, 1},
  {BK_MODE_AM, (char *) "13m", 21400, 21900, 21525, 5, 1}
};


const int lastBand = (sizeof band / sizeof(tabBand)) - 1;
int8_t bandIdx = 0; // FM

const uint8_t app_id = 88;  // Useful to check the EEPROM content before processing useful data
const int eeprom_address = 0;
long storeTime = millis();

uint8_t seekDirection = 1;  // 0 = Down; 1 = Up. This value is set by the last encoder direction.

long pollin_elapsed = millis();

// Encoder control variables
volatile int encoderCount = 0;
uint16_t currentFrequency;
uint16_t previousFrequency;

// Encoder control
Rotary encoder = Rotary(ENCODER_PIN_A, ENCODER_PIN_B);

// BK108X
BK108X rx;

// OLED
Adafruit_SSD1306 display(128, 64, &Wire, -1);

void setup() {
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);

  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  // Push button pin
  pinMode(VOLUME_UP, INPUT_PULLUP);
  pinMode(VOLUME_DOWN, INPUT_PULLUP);
  pinMode(SEEK_FUNCTION, INPUT_PULLUP);
  pinMode(BAND_MODE_SWITCH_UP, INPUT_PULLUP);
  pinMode(BAND_MODE_SWITCH_DOWN, INPUT_PULLUP);

  EEPROM.begin(EEPROM_SIZE);

    // OLED 
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Not working
  display.clearDisplay();
  display.setTextColor(WHITE);

  // If you want to reset the eeprom, keep the ENCODER PUSH BUTTON  pressed during statup
  if (digitalRead(SEEK_FUNCTION) == LOW) {
    EEPROM.write(eeprom_address, 0);
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.print("RESET");
    delay(1500);
    display.clearDisplay();
    display.setTextSize(1);
    showSplash();
  }

  // Encoder interrupt
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), rotaryEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), rotaryEncoder, CHANGE);

  rx.setup(21, 22);
  // rx.setup(I2C_SDIO_PIN, I2C_SCLK_PIN, OSCILLATOR_TYPE_REFCLK, 32768); // It is not working - Trying to get more information about the device and setup
  delay(200); 

  // Checking the EEPROM content
  if (EEPROM.read(eeprom_address) == app_id) {
    readAllReceiverInformation();
  } else {
    // Default values
    rx.setMono(false);  // Force stereo
   //  rx.setRBDS(true);  //  set RDS and RBDS. See setRDS.
     rx.setRDS(true);
   // rx.RdssetRdsFifo(true);
    useBand();
    rx.setVolume(DEFAULT_VOLUME_LEVEL);
  }

  showSplash();
  delay(1000);
  display.clearDisplay();
  
  rx.setRDS(true);
  rx.setAfc(true);
 
}


void saveAllReceiverInformation() {
  EEPROM.begin(EEPROM_SIZE);

  // The write function/method writes data only if the current data is not equal to the stored data.
  EEPROM.write(eeprom_address, app_id);
  EEPROM.write(eeprom_address + 1, rx.getVolume());           // stores the current Volume
  EEPROM.write(eeprom_address + 2, currentFrequency >> 8);    // stores the current Frequency HIGH byte for the band
  EEPROM.write(eeprom_address + 3, currentFrequency & 0xFF);  // stores the current Frequency LOW byte for the band
  EEPROM.write(eeprom_address + 4, DisplayStyle);

  EEPROM.write(eeprom_address + 6, bandIdx);

  EEPROM.end();
}

void readAllReceiverInformation() {
  rx.setVolume(EEPROM.read(eeprom_address + 1));
  currentFrequency = EEPROM.read(eeprom_address + 2) << 8;
  currentFrequency |= EEPROM.read(eeprom_address + 3);
  previousFrequency = currentFrequency;
  DisplayStyle = EEPROM.read(eeprom_address + 4);


  bandIdx = EEPROM.read(eeprom_address + 6);
  band[bandIdx].default_frequency = currentFrequency;
  useBand();
}


/*
   To store any change into the EEPROM, it is needed at least STORE_TIME  milliseconds of inactivity.
*/
void resetEepromDelay() {
  delay(PUSH_MIN_DELAY);
  storeTime = millis();
  previousFrequency = 0;
}


/*
    Reads encoder via interrupt
    Use Rotary.h and  Rotary.cpp implementation to process encoder via interrupt
*/
void rotaryEncoder() {  // rotary encoder events
  uint8_t encoderStatus = encoder.process();
  if (encoderStatus)
    encoderCount = (encoderStatus == DIR_CW) ? 1 : -1;
}

void showSplash() {
  display.setCursor(0, 0);
  display.print("PU2CLR-BK1088");
  display.setCursor(0, 10);
  display.print("Arduino Library");
  display.setCursor(0, 20);
  display.print("OLED TEST");
  display.display();
  delay(1000);
}

/*
   Shows the static content on  display
*/
void showTemplate() {
}


/*
//   Shows frequency information on Display

void showFrequency() {
  currentFrequency = rx.getFrequency();
  display.setCursor(4, 1);
  if ( band[bandIdx].mode == BK_MODE_FM)
     display.print(rx.formatCurrentFrequency());
  else {
      if ( currentFrequency < 1000) 
        display.print(rx.formatCurrentFrequency(' ',0));
      else   
        display.print(rx.formatCurrentFrequency('.',2));
  }
  display.display();
}
*/

/* *******************************
//  Shows RSSI status
 
void showRSSI() {
  char rssi[12];
  rx.convertToChar( rx.getRssi(), rssi, 3, 0, '.');
  strcat(rssi, "dB");
  display.setCursor(13, 1);
  display.print(rssi);
}
*/

void bandUp()
{
  // save the current frequency for the band
  band[bandIdx].default_frequency = currentFrequency;

  if (bandIdx < lastBand)
  {
    bandIdx++;
  }
  else
  {
    bandIdx = 0;
  }
  useBand();
}

void bandDown()
{
  // save the current frequency for the band
  band[bandIdx].default_frequency = currentFrequency;

  if (bandIdx > 0)
  {
    bandIdx--;
  }
  else
  {
    bandIdx = lastBand;
  }
  useBand();
}


void useBand() {

  if (band[bandIdx].mode ==  BK_MODE_FM)
  {
    rx.setFM(band[bandIdx].minimum_frequency, band[bandIdx].maximum_frequency, band[bandIdx].default_frequency, band[bandIdx].step);
    rx.setRDS(true);
    rx.setFmDeemphasis(DE_EMPHASIS_75);
    rx.setSoftMute(true);
    rx.setMono(false);  // Force stereo
  }
  else
  {
    rx.setAM(band[bandIdx].minimum_frequency, band[bandIdx].maximum_frequency, band[bandIdx].default_frequency, band[bandIdx].step, band[bandIdx].band_space);
  }
  delay(100);
  currentFrequency = band[bandIdx].default_frequency;
  rx.setFrequency(currentFrequency);
  delay(MIN_ELAPSED_TIME); // waits a little more for releasing the button.

}

/*********************************************************
   RDS
 *********************************************************/

String newRdsText = rx.getRdsStationName();

char *ProgramInfo;
char *StationName;
long polling_rds = millis();

int progInfoIndex = 0;  // controls the part of the rdsMsg text will be shown on display 16x2 Display

long delayProgramInfo = millis();
long delayLocalTime = millis();
long waitTime = 700L;

/*
void clearRdsText(char *txt, int size) {
  for (int i = 0; i < size; i++) 
    if ( txt[i] < 32 ) txt[i] = ' ';
}
*/

/**
  showProgramInfo - Shows the Program Information
*/
/*
void showProgramInfo() {
  char txtAux[16];

  if (programInfo == NULL || strlen(programInfo) < 2 || (millis() - delayProgramInfo) < waitTime) return;
  delayProgramInfo = millis();
  programInfo[61] = '\0';  // Truncate the message to fit on display line
  strncpy(txtAux, &programInfo[progInfoIndex], 16);
  txtAux[15] = '\0';
  clearRdsText(txtAux,17); // replace unwanted ASCII symbol to space. 
  progInfoIndex += 2;
  if (progInfoIndex > 60) progInfoIndex = 0;
  display.setCursor(0, 0);
  display.print(txtAux);
  Serial.println(txtAux);
  waitTime = 700;
}
*/

/*
void showTime() {
  if (localTime == NULL || strlen(localTime) < 4 || (millis() - delayLocalTime) < 50000) return;
  Serial.println(localTime);
  delayProgramInfo = millis(); // Stop showing Program Information for 10s
  delayLocalTime = millis(); 
  waitTime = 10000L;
} 
*/


void clearRds() {
//  programInfo = NULL;
  rx.clearRdsBuffer();
}

/*
void checkRDS() {

  char *ProgramInfo;
  char *StationName;
  
  // You must call getRdsReady before calling any RDS query function.
  if (rx.getRdsReady()) {
      pProgramInfo = rx.getRdsProgramInformation();
      pStationName = rx.getRdsStationName();
        Serial.println(pStationName);    
        Serial.println(pProgramInfo);
      
  }
}
*/

void rds() {
  if (rx.getRdsReady()) {
    processRdsInfo();
  }
}

void processRdsInfo() {

//  char *ProgramInfo;
//  char *StationName;

      ProgramInfo = rx.getRdsProgramInformation();
      StationName = rx.getRdsStationName();
      Serial.println(StationName);    
      Serial.println(ProgramInfo);
      Serial.println(rx.getRdsTime());
      Serial.println(rx.getRdsLocalTime());
  
}





/*********************************************************

 *********************************************************/



/**
   Process seek command.
   The seek direction is based on the last encoder direction rotation.
*/
void doSeek() {

  rx.setSeekThreshold(70, 15);
  rx.seek(BK_SEEK_WRAP, seekDirection, oled);  // showFrequency will be called by the seek function during the process.
  delay(200);
}

void oled() {
  
  if (DisplayStyle == false) {
    
    display.clearDisplay();
    
    currentFrequency = rx.getFrequency();
    display.setCursor(0, 0);
    display.print("FREQ:");
   if ( band[bandIdx].mode == BK_MODE_FM)
      display.print(rx.formatCurrentFrequency());
   else {
   if ( currentFrequency < 1000) 
      display.print(rx.formatCurrentFrequency(' ',0));
   else   
      display.print(rx.formatCurrentFrequency('.',2));
  }
//   display.print(rx.getFrequency());

    display.drawLine(0, 9, 127, 9, WHITE);
    
    display.setCursor(0, 12);
    display.print("VOL: ");
    display.print(rx.getVolume());
    
    display.setCursor(64, 12);
    display.print("BAND: ");
    display.print(band[bandIdx].name);

    display.setCursor(0, 21);
    display.print("dB: ");
    display.print(rssi);

    display.setCursor(64 ,21);
    display.print("SNR: ");
    display.print(snr);

 display.drawLine(0, 30, 127, 30, WHITE);
 
    display.setCursor(0 ,33);
    display.print(ProgramInfo);
    display.setCursor(0, 43);
    display.print(StationName);

if (rx.isStereo() == true && band[bandIdx].mode == BK_MODE_FM) {
  display.setCursor(92, 0);
  display.print("STEREO");
} else {
  display.setCursor(92, 0);
  display.print("MONO");
}
    
  display.display();

} else if (DisplayStyle == true) {
  
    display.clearDisplay();

if (band[bandIdx].mode == BK_MODE_FM) {
    display.setCursor(0, 0);
    display.print("FM");
} else if (band[bandIdx].mode == BK_MODE_AM) {
    display.setCursor(0, 0);
    display.print("AM");
}
    
    currentFrequency = rx.getFrequency();
    display.setCursor(20, 0);
    display.setTextSize(2);
   if (band[bandIdx].mode == BK_MODE_FM)
      display.print(rx.formatCurrentFrequency());
   else {
       if (currentFrequency < 1000) 
     { display.setCursor(8, 0);
      display.print(rx.formatCurrentFrequency(' ',0));
     } else {  
      display.print(rx.formatCurrentFrequency('.',2));
  }
   }
//   display.print(rx.getFrequency());

    display.setTextSize(1);
    display.drawLine(0, 30, 127, 30, WHITE);

    if (rx.isStereo() == true && band[bandIdx].mode == BK_MODE_FM) {
  display.setCursor(92, 0);
  display.print("STEREO");
} else {
  display.setCursor(94, 0);
  display.print("MONO");
}
       
    display.setCursor(0, 20);
    display.print("dB:");
    display.print(rssi);

    display.setCursor(43 ,20);
    display.print("SNR:");
    display.print(snr);

    display.setCursor(92, 20);
    display.print("VOL:");
    display.print(rx.getVolume());
    
 display.drawLine(0, 32, 127, 32, WHITE);
 
    display.setCursor(0 ,35);
    display.print(ProgramInfo);
    display.setCursor(0, 45);
    display.print(StationName);

     display.display();
 }
 
}

void loop() {
  
rssi = rx.getRssi();
snr = rx.getSnr();

if (digitalRead(SEEK_FUNCTION) == LOW && digitalRead(VOLUME_UP) == LOW) {
   DisplayStyle = true;
   oled();
} else if (digitalRead(SEEK_FUNCTION) == LOW && digitalRead(VOLUME_DOWN) == LOW) {
   DisplayStyle = false;
   oled();
}

  // Check if the encoder has moved.
  if (encoderCount != 0) {
    if (encoderCount == 1) {
      rx.setFrequencyUp();
      seekDirection = BK_SEEK_UP;
    } else {
      rx.setFrequencyDown();
      seekDirection = BK_SEEK_DOWN;
    }
    oled();
    encoderCount = 0;
    storeTime = millis();
  }

  if (digitalRead(VOLUME_UP) == LOW) {
    rx.setVolumeUp();
    oled();
    resetEepromDelay();
  } else if (digitalRead(VOLUME_DOWN) == LOW) {
    rx.setVolumeDown();
    oled();
    resetEepromDelay();
  } else if (digitalRead(SEEK_FUNCTION) == LOW) {
    doSeek();
    oled();
  } else if (digitalRead(BAND_MODE_SWITCH_UP) == LOW) {
    bandUp();
    oled();
  } else if (digitalRead(BAND_MODE_SWITCH_DOWN) == LOW) {
    bandDown();
    oled();
  }
  if ((millis() - pollin_elapsed) > POLLING_TIME) {
    oled();
    pollin_elapsed = millis();
  }

  if ((millis() - polling_rds) > POLLING_RDS && band[bandIdx].mode == BK_MODE_FM)  {
    rds();
    polling_rds = millis();
  }



  // Show the current frequency only if it has changed
  if ((currentFrequency = rx.getFrequency()) != previousFrequency) {
    clearRds();
    if ((millis() - storeTime) > STORE_TIME) {
      saveAllReceiverInformation();
      storeTime = millis();
      previousFrequency = currentFrequency;
    }
  }
  delay(5);
}
