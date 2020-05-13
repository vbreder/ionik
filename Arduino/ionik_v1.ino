/*
 * IONIK IONOSPHERIC MONITOR v1.0
 * 2016-11-14 - github.com/victorbreder
 */

#include <string.h>
#include <SPI.h>
#include <SD.h>

/* LEDS */
// module that controls the behavior of the LEDs
// uses pins 9 and 8

// led states
const int LED_OFF = 0;
const int LED_ON = 1;
const int LED_BLINK = 2;

const int OK_LED_PIN = 5; // green led
const int ERROR_LED_PIN = 6; // red led

int okValue = 0; // green led state
int errorValue = 0; // red led state

// sets up LED pins for digital output
void setupLEDs() {
  pinMode(OK_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
}

// sets the green led state
int setOK(int value) {
  okValue = value;
  if (okValue == LED_ON) {
    digitalWrite(OK_LED_PIN, HIGH);
  } else if (okValue == LED_OFF) {
    digitalWrite(OK_LED_PIN, LOW);
  }
}

// sets the red led state
void setError(int value) {
  errorValue = value;
  if (errorValue == LED_ON) {
    digitalWrite(ERROR_LED_PIN, HIGH);
  } else if (errorValue == LED_OFF) {
    digitalWrite(ERROR_LED_PIN, LOW);
  }
}

// startup led animation to indicate normal behavior
void startLEDs() {
  setOK(LED_ON);
  setError(LED_ON);
  delay(1200);
  setOK(LED_OFF);
  setError(LED_OFF);
  delay(100);
}

// shutdown led animation
void stopLEDs() {
  setOK(LED_OFF);
  setError(LED_OFF);
  delay(1000);
}

// updates blinking leds
void updateLEDs() {
  bool on = (millis() / 200) % 2 == 0;
  if (okValue == LED_BLINK) {
    digitalWrite(OK_LED_PIN, (on ? HIGH : LOW));
  }
  if (errorValue == LED_BLINK) {
    digitalWrite(ERROR_LED_PIN, (on ? HIGH : LOW));
  }
}

/* BUTTON */
// module that controls the behavior of the button
// uses pin 7
const int BUTTON_PIN = 7;

// toggles the recording if the button is pressed
int lastButtonState = LOW;
void updateButton() {
  int buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == HIGH && lastButtonState == LOW) {
    if (running) {
      stop();
    } else {
      start();
    }
  }
  lastButtonState = buttonState;
}

/* SD CARD */
// module that constrols the behavior of the SD card
// the SD card must be formatted as FAT32
// uses pins 11, 12, 13 and 4 (all related to the SPI bus)

File logFile; // output file
bool sdCardStarted = false;

void startSDCard() {
  bool error = false;

  // initializes the SD card
  if (!sdCardStarted) {
    if (SD.begin(4)) {
      sdCardStarted = true;
    } else {
      error = true;
    }
  }

  // selects an available filename
  char filename[15];
  strcpy(filename, "GPSLOG00.txt");
  for (int i = 0; i < 100; i++) {
    filename[6] = '0' + i / 10;
    filename[7] = '0' + i % 10;
    if (!SD.exists(filename)) {
      break;
    }
  }

  // creates a new output file
  logFile = SD.open(filename, FILE_WRITE);
  if (logFile) {
    error = false;
  } else {
    error = true;
  }

  if (error) {
    stop(); // stop recording
    setError(LED_BLINK); // show error to user
  }
}

// safely terminates the output file
void stopSDCard() {
  logFile.close();
}

/* GPS */
// module that controls the communication with the GPS sensor
// uses pins 0 and 1 (makes use of the UART chip for hardware buffering of
// received serial data)

void startGPS() {
  // starts serial connection with the GPS sensor in the default baudrate
  // of 9600 baud and changes the baud rate to a faster 57600 baud
  Serial.begin(9600);
  delay(100);
  Serial.println("$PMTK251,57600*2C"); // PMTK_SET_NMEA_BAUDRATE_57600
  Serial.flush();
  Serial.end();
  delay(100);

  // restarts the serial connection with the new baudrate 57600 baud
  Serial.begin(57600);
  delay(100);

  // hack that forces the sensor to use the 10 Hz sampling rate
  Serial.println("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");
    // PMTK_SET_NMEA_OUTPUT_RMCONLY
  Serial.println("$PMTK220,100*2F"); // PMTK_SET_NMEA_UPDATE_10HZ

  // selects the NMEA information the GPS sensor should output over serial
  // reference: https://www.adafruit.com/datasheets/PMTK_A11.pdf
  Serial.println("$PMTK314,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,0*28");
  Serial.flush();
  delay(1000);

  // starts to read the UART buffer
  useInterrupt(true);
}

// stos the reading of the UART buffer
void stopGPS() {
  useInterrupt(false);
}

// if interrupt is enabled, runs every millisecond and reads the UART buffer
SIGNAL(TIMER0_COMPA_vect) {
  while (read());
}

// enables or disables the interrupt every millisecond
void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
  }
}

/* ADAFRUIT */
// stripped down reimplementation of the Adafruit GPS library
// reference: https://github.com/adafruit/Adafruit_GPS
const int MAXLINELENGTH = 120;

// double buffering
volatile char line1[MAXLINELENGTH];
volatile char line2[MAXLINELENGTH];

volatile uint8_t lineidx=0;

volatile char *currentline;
volatile char *lastline;
volatile boolean recvdflag;
volatile boolean inStandbyMode;

// returns if NMEA line was completely read
boolean newNMEAreceived() {
  return recvdflag;
}

// points to the last full NMEA line received
char* lastNMEA() {
  recvdflag = false;
  return (char *) lastline;
}

// reads data from the UART buffer and writes to memory
char read(void) {
  char c = 0;
  if (!Serial.available()) return c;
  c = Serial.read();

  if (c == '\n') {
    currentline[lineidx] = 0;
    if (currentline == line1) {
      currentline = line2;
      lastline = line1;
    } else {
      currentline = line1;
      lastline = line2;
    }
    lineidx = 0;
    recvdflag = true;
  }
  currentline[lineidx++] = c;
  if (lineidx >= MAXLINELENGTH)
    lineidx = MAXLINELENGTH-1;
  return c;
}

/* MAIN */
// main module that controls the overall behavior of the circuit

bool running = false;

// sets up pins
void setup() {
  pinMode(BUTTON_PIN, INPUT);
  setupLEDs();
  start();
}

// starts (or restarts) the program
void start() {
  running = true;
  startLEDs();
  startSDCard();
  startGPS();
}

// ends the program
void stop() {
  running = false;
  stopGPS();
  stopSDCard();
  stopLEDs();
}

// gets NMEA frames from the GPS sensor and writes to the SD card
int frameCount = 0;
char lastFrame[120];
void loop() {

  if (running) {
    if (newNMEAreceived()) {
      // copies data to local scope to prevent data racing with interrupt
      memcpy(&lastFrame, lastNMEA(), 120);

      // detects if the GPS sensor has a FIX
      if (lastFrame && lastFrame[4] == 'G' && lastFrame[5] == 'S' && lastFrame[6] == 'A') { // GPGSA
        if (lastFrame[10] == '2' || lastFrame[10] == '3') { // FIX
          setOK(LED_ON); // signals the FIX to the user
        } else {
          setOK(LED_BLINK); // signals that the GPS sensor is searching
        }
      }

      // writes the NMEA frame to the SD card
      uint8_t stringsize = strlen((char *) &lastFrame);
      logFile.write((uint8_t *) &lastFrame, stringsize);
      frameCount++;
      if (frameCount == 100) { // flushs the buffer every 100 frames to prevent
        logFile.flush();       // data loss on unexpected power loss
        frameCount = 0;
      }
    }
  }

  updateButton();
  updateLEDs();
}
