#include <Arduino.h>

// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEMIDI.h"

#include "BluefruitConfig.h"
#include "notes.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

//BLE SETTINGS
#define BLE_FACTORY_RESET_ENABLE  0 //If 1, the BLE device will factory reset and will revert any custom device name you've programmed into it. Avoid this unless something's really gone wrong in the BLE chip.
#define MINIMUM_FIRMWARE_VERSION  "0.7.0"

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BLEMIDI midi(ble);
LSM9DS1 imu;

bool isConnected = false;

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void connected(void)
{
  isConnected = true;
  Serial.println(F("BLE connected."));
  delay(1000);
}

void disconnected(void)
{
  Serial.println("BLE disconnected");
  isConnected = false;
}

void BleMidiRX(uint16_t timestamp, uint8_t status, uint8_t byte1, uint8_t byte2)
{
  Serial.print("[MIDI ");
  Serial.print(timestamp);
  Serial.print(" ] ");

  Serial.print(status, HEX); Serial.print(" ");
  Serial.print(byte1 , HEX); Serial.print(" ");
  Serial.print(byte2 , HEX); Serial.print(" ");

  Serial.println();
}

//9-DOF board configuration - shouldn't need to change this
#define LSM9DS1_M	  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -10.31 // Declination (degrees) in Fairfax, VA

#define HISTORY_DEPTH 5
float heading_history[HISTORY_DEPTH];
float pitch_history[HISTORY_DEPTH];
float roll_history[HISTORY_DEPTH];
int history_index = 0;
unsigned long total_measurements = 0;

//The latest values
float roll = 0.0;
float pitch = 0.0;
float heading = 0.0;

//A running average of the last HISTORY_DEPTH values
float heading_avg = 0.0;
float pitch_avg = 0.0;
float roll_avg = 0.0;

float heading_min = -180.0; //__FLT_MAX__;
float heading_max = 180.0; //__FLT_MIN__;
float pitch_min = -90.0; //__FLT_MAX__;
float pitch_max = 90.0; //__FLT_MIN__;
float roll_min = -180.0; //__FLT_MAX__;
float roll_max = 180.0; //__FLT_MIN__;

byte curr_notes[3] = {0,0,0};
byte prev_notes[3] = {0,0,0};

void setup_ble_midi()
{
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("Bluefruit found.") );

  if ( BLE_FACTORY_RESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  //ble.sendCommandCheckOK(F("AT+uartflow=off"));
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Set BLE callbacks */
  ble.setConnectCallback(connected);
  ble.setDisconnectCallback(disconnected);

  // Set MIDI RX callback
  midi.setRxCallback(BleMidiRX);

  Serial.println(F("Enabling MIDI."));
  if ( ! midi.begin(true) )
  {
    error(F("Could not enable MIDI"));
  }

  ble.verbose(false);
  Serial.println(F("Waiting for BLE connection..."));
}

void setup_imu()
{
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin())
  {
    error(F("Failed to communicate with LSM9DS1."));
  }
}

void setup() 
{
  delay(500);
  Serial.begin(115200);
  delay(500);

  setup_imu();
  setup_ble_midi();
}

void noteOn(byte channel, byte pitch, byte velocity) {
  midi.send(0x90 | channel, pitch, velocity);
}

void noteOff(byte channel, byte pitch, byte velocity) {
  midi.send(0x80 | channel, pitch, velocity);
}

//val should be a 14-bit value
void pitchBend(uint16_t val) {
  byte lsb = val & 0x007F;
  byte msb = (val & 0x3F10) >> 7;
  midi.send(0xE0, lsb, msb);
}

//Maintain the BLE connection. Call this each loop.
void ble_tick()
{
  ble.update(500);
}

void read_9dof()
{
  if ( imu.gyroAvailable() )
  {
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    imu.readMag();
  }
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void calculate_attitude()
{
  roll = atan2(-imu.ax, imu.az);
  pitch = atan2(imu.ay, sqrt(imu.ax * imu.ax + imu.az * imu.az));
  
  if (imu.my == 0)
    heading = (imu.mx < 0) ? PI : 0;
  else
    heading = atan2(imu.my, imu.mx);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
}

void update_attitude_history()
{
  heading_history[history_index] = heading;
  pitch_history[history_index] = pitch;
  roll_history[history_index] = roll;
  history_index = (history_index + 1) % HISTORY_DEPTH;

  if (heading > heading_max) heading_max = heading;
  if (heading < heading_min) heading_min = heading;
  if (roll > roll_max) roll_max = roll;
  if (roll < roll_min) roll_min = roll;
  if (pitch > pitch_max) pitch_max = pitch;
  if (pitch < pitch_min) pitch_min = pitch;

  heading_avg = 0.0;
  pitch_avg = 0.0;
  roll_avg = 0.0;

  //A smarter way to do this would be to keep a running average that gets adjusted with each measurement rather 
  //than computing it from scratch each time. If the performance lags this may be a culprit.
  for (int i = 0; i < HISTORY_DEPTH; i++)
  {
    heading_avg += heading_history[i];
    pitch_avg += pitch_history[i];
    roll_avg += roll_history[i];
  }

  heading_avg /= HISTORY_DEPTH;
  pitch_avg /= HISTORY_DEPTH;
  roll_avg /= HISTORY_DEPTH;

  total_measurements++;
}

//Logic for if there is movement: if the most recent measurements are above the running average of the past HISTORY_DEPTH measurements.
//That may be a naive way to determine it, but let's see how it goes.
bool is_movement()
{
  return ((heading > heading_avg) || (pitch > pitch_avg) || (roll > roll_avg));
}

//Helper function to map a current value within a range to an index into an array
int get_index(float curr, float min, float max, int num_indices)
{
  return (int)(((num_indices - 1) * ((curr - min) / (max - min))) + 0.5);
}

void calculate_notes()
{
  curr_notes[0] = note_pitches_roll[get_index(roll_avg, roll_min, roll_max, NUM_NOTES)];
  curr_notes[1] = note_pitches_pitch[get_index(pitch_avg, pitch_min, pitch_max, NUM_NOTES)];
  curr_notes[2] = note_pitches_heading[get_index(heading_avg, heading_min, heading_max, NUM_NOTES)];
}

void print_measurements()
{
  Serial.print("RPH:\t");
  Serial.print(roll_avg);
  Serial.print("\t");
  Serial.print(pitch_avg);
  Serial.print("\t");
  Serial.print(heading_avg);

  Serial.print("\t\t(Min|Max)\t");

  Serial.print("(");
  Serial.print(roll_min);
  Serial.print("|");
  Serial.print(roll_max);
  Serial.print(")");
  Serial.print("\t");
  Serial.print("(");
  Serial.print(pitch_min);
  Serial.print("|");
  Serial.print(pitch_max);
  Serial.print(")");
  Serial.print("\t");
  Serial.print("(");
  Serial.print(heading_min);
  Serial.print("|");
  Serial.print(heading_max);
  Serial.print(")");
  Serial.print("\t");
  
  Serial.println("");
}

void print_notes()
{
  Serial.print("Notes:\t");
  Serial.print(curr_notes[0]);
  Serial.print("\t");
  Serial.print(curr_notes[1]);
  Serial.print("\t");
  Serial.println(curr_notes[2]);
}

unsigned long last_read_time = 0;
unsigned long last_send_time = 0;
#define READ_SLEEP_MS (100) // This limits how frequently you read from the sensor (milliseconds)
#define SEND_SLEEP_MS (100) // This limits how frequently you send output (milliseconds)
#define ALWAYS_SEND (0)     // Set to 1 to always send a MIDI note regardless of if there's movement. Set to 0 to require movement to send a note.

//These three values are used to mute one of the sensor dimensions' contribution to the notes being played.
//Set any of them to 0 to mute that particular dimension from being played. Set to 1 to include it in the notes being played.
#define PLAY_ROLL_NOTES     (1)
#define PLAY_PITCH_NOTES    (1)
#define PLAY_HEADING_NOTES  (1)

void loop()
{
  ble_tick();
  
  unsigned long curr_time = millis();
  if ( (curr_time - last_read_time) > READ_SLEEP_MS) 
  {
    read_9dof();
    calculate_attitude();
    update_attitude_history();
    //print_measurements(); //Uncomment this to see the values being read off the sensor in the serial monitor

    last_read_time = curr_time;
  }

  if ((total_measurements > HISTORY_DEPTH) && ((curr_time - last_send_time) > SEND_SLEEP_MS))
  {
    calculate_notes();
    //print_notes(); //Uncomment this to see the note values which have been calculated based off sensor readings in the serial monitor

    bool play_roll    = (PLAY_ROLL_NOTES    && (ALWAYS_SEND || (curr_notes[0] != prev_notes[0])));
    bool play_pitch   = (PLAY_PITCH_NOTES   && (ALWAYS_SEND || (curr_notes[1] != prev_notes[1])));
    bool play_heading = (PLAY_HEADING_NOTES && (ALWAYS_SEND || (curr_notes[2] != prev_notes[2])));

    if (play_roll)    noteOn(0, curr_notes[0], 100);
    if (play_pitch)   noteOn(0, curr_notes[1], 100);
    if (play_heading) noteOn(0, curr_notes[2], 100);

    delay(50);

    if (play_roll)    noteOff(0, curr_notes[0], 100);
    if (play_pitch)   noteOff(0, curr_notes[1], 100);
    if (play_heading) noteOff(0, curr_notes[2], 100);
    
    last_send_time = curr_time;

    prev_notes[0] = curr_notes[0];
    prev_notes[1] = curr_notes[1];
    prev_notes[2] = curr_notes[2];

  }
}