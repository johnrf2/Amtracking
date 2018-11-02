// John Farwick Code attempt 3
#include <Keypad.h>          // Keypad
#include <Wire.h>            // Faster contact library
#include <LiquidCrystal.h>   // LCD
#include <SdFat.h>           // SD Card Reader
#include <SPI.h>             // Accelerometer
#include <Adafruit_GPS.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
////////////////////// PRELIMINARY CODE ///////////////////

///////////////////// Global Variable Scope //////////////////////
float avg_xavg_0;
float avg_yavg_0;
float avg_zavg_0;

float xavg_g;
float xavg_0;
float yavg_g;
float yavg_0;
float zavg_g;
float zavg_0;

float xrawsum;
float yrawsum;
float zrawsum;

float xScale_new;
float yScale_new;
float zScale_new;

float xG_force;
float yG_force;
float zG_force;

Adafruit_GPS GPS(&Serial2);
#define GPSECHO  false
boolean usingInterrupt = false; // this keeps track of whether we're using the interrupt, off by default
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

///////////////////// **GLOBAL VARIABLE SCOPE END** //////////////////////

// LSM9DS1 Defines
#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5




////////////////////// ADC START //////////////////////
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))

// same idea, but to set one bit:
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
////////////////////// ** ADC END** //////////////////////




////////////////////// ACCELEROMETER PARAMETERS START //////////////////////
// define which Arduino analog input pins look at the three outputs from
// the accelerometer breakout board.
const int axInput = A0;
const int ayInput = A1;
const int azInput = A2;

// specify initial values for the zero acceleration and 1 g scales, assuming
// I am using the 2.56 V ADC internal reference.
// This is the Raw base that Gollin measured for each direction, it is based on the refernce (2.56)
/// under the internal voltage reference that it measures in free fall (~1.1-1.5) times the bit total of 1023
float xRawZero = 663.3;
float yRawZero = 657.0;
float zRawZero = 674.7;
// then after calibration (is adjusted by the loop) you measure what each one feels oriented for gravity
float xRawOneG = 687.6;
float yRawOneG = 682.7;
float zRawOneG = 699.6;

// scale factors: ADC counts per g (g = 9.81 m/sec^2).
float xScale = 24.32;
float yScale = 25.66;
float zScale = 24.88;
// now we know what the difference between the raw and One G is, so when we measure our data, we take the
/// difference, multiply it by what our data was, and that gives us the given amount of G's at whatever time
// how many times to read the accelerometer when calculating an average for
// calibration purposes...
const int number_to_average = 50;
////////////////////// **ACCELEROMETER PARAMTERS END** //////////////////////

////////////////////// KEYPAD START //////////////////////
// our keypad has four rows and three columns. since this will never change
// while the program is runniong, declare these as constants so that they
// will live in flash (program code) memory instead of the much smaller
// SRAM.
const byte ROWS = 4;
const byte COLS = 3;

// keypad layout
char keys[ROWS][COLS] = {
    {'1','2','3'},
    {'4','5','6'},
    {'7','8','9'},
    {'*','0','#'}
};

// looking down on the keyboard from above (the side with the keys), the pins are
// numbered 1 - 8, going from left to right, though 8 is not used.

// Since I am using an Arduino Mega 2560 with a number of breakout boards,
// I have the following Arduino pin assignments in order to allow the column pins to
// generate interrupts in some future version of this program.
byte Arduino_colPins[COLS] = {2, 3, 18};
byte Arduino_rowPins[ROWS] = {31, 33, 35, 37};

// now instantiate a Keypad object, call it kpd. Also map its pins.
Keypad kpd = Keypad( makeKeymap(keys), Arduino_rowPins, Arduino_colPins, ROWS, COLS );
////////////////////// **KEYPAD END** //////////////////////

////////////////////// LCD START //////////////////////
// initialize the LCD library by associating LCD interface pins
// with the arduino pin numbers to which they are connected
const int rs = 12, en = 11, data4 = 36, data5 = 34, data6 = 32, data7 = 30;

// now instantiate (create an instance of) a LiquidCrystal object.
LiquidCrystal lcd(rs, en, data4, data5, data6, data7);

// a global variable, accessible to all functions:
int lines_written;
////////////////////// **LCD END**//////////////////////

////////////////////// SD CARD START//////////////////////
// "#define" is an instruction to the compiler. In this case, it will
// replace all occurrences of "SD_CS_PIN" with "SS", whose value is
// already known to the compiler.
#define SD_CS_PIN SS

// instantiate a file system object
SdFat SD;

// create a file object to which I'll write data
File myFile;

// file name
char filename[ ] = "DataLog.txt";
////////////////////// **SD CARD END** //////////////////////

/**
 * Waits for a key press, then returns it as a char 
 */
char getKey() {
    while(1) {
        if (kpd.getKeys() && kpd.key[0].kstate == PRESSED) {
            return kpd.key[0].kchar;
        }
    }
}

/**
 * Setup the GPS module
 */
void setupGps() {
    Serial.println("Setting up the GPS!");
    // 9600 NMEA is the default baud rate for MTK - some use 4800
    GPS.begin(9600);

    // You can adjust which sentences to have the module emit, below
    // uncomment this line to turn on only the "minimum recommended" data for high update rates!
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);

    // Set the update rate
    // Note you must send both commands below to change both the output rate (how often the position
    // is written to the serial line), and the position fix rate.
    // 1 Hz update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);

    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA);

    // the nice thing about this code is you can have a timer0 interrupt go off
    // every 1 millisecond, and read data from the GPS for you. that makes the
    // loop code a heck of a lot easier!
    useInterrupt(true);
}

/**
 * Setup the Serial line
 */
void setupSerial() {
    Serial.begin(115200);
    Serial.println("Setting up the serial!");
}

/**
 * Setup the LCD
 */
void setupLcd() {
    Serial.println("Setting up the LCD!");
    lcd.begin(16, 2);
}

/**
 * Setup the SD card
 *
 * Just reopen it for write whenever something needs to input data
 * will feed out into the text file which now does not have the
 * garbage in it need to make sure each statement has a comma after it
 * for creating a list
 */
void setupSd() {
    Serial.println("Setting up SD!");
    if (!SD.begin(SD_CS_PIN)) {
        // Testing if the SD card is present
        Serial.println("SD initialization failed!");
    }
    // if the SD file exists already, delete it.
    if(SD.exists(filename)) {
        SD.remove(filename);

        // wait a bit just to make sure we're finished with the file remove
        delay(100);
    }
    // open the new file then check that it opened properly.
    myFile = SD.open(filename, FILE_WRITE);

    if (myFile) {
        Serial.print("Writing to "); Serial.print(filename); Serial.println("...");

    } else {

        // if the file didn't open, print an error:
        Serial.print("error opening "); Serial.println(filename);

        // delay a bit to give the serial port time to display the message...
        delay(100);
        exit(0);
    }
    myFile.close();
}

/**
 * Takes an averaging of the ADXL accelerometer 
 */
class AvgADXL {
public:
    float x, y, z;

    void take_avg(int max_reads) {
        float x_sum = 0.;
        float y_sum = 0.;
        float z_sum = 0.;

        for (int reading=0; reading<max_reads; reading++) {
            x_sum += analogRead(axInput);
            y_sum += analogRead(ayInput);
            z_sum += analogRead(azInput);
        }
        x = x_sum / max_reads;
        y = y_sum / max_reads;
        z = z_sum / max_reads;
    }
};

void setupAccel() {
    // ADXL326 3 axis accelerometer and ADC setup:
    // Calculate scales, in ADC counts per g (= 9.81 m/sec^2)
    xScale = xRawOneG - xRawZero;
    yScale = yRawOneG - yRawZero;
    zScale = zRawOneG - zRawZero;

    // Enable the ADC by setting the ADEN bit in the ADCSRA register.
    sbi(ADCSRA, ADEN);

    // let's use a prescale of 16: ADPS2, ADPS1, and ADPS0 are set to 1, 0, 0.
    // faster than this and the ADC becomes inaccurate. Slower than this doesn't
    // yield an appreciable increase in accuracy.
    sbi(ADCSRA, ADPS2);
    cbi(ADCSRA, ADPS1);
    cbi(ADCSRA, ADPS0);

    // Sert the ADC reference voltage to 2.56 volts.
    analogReference(INTERNAL2V56);

    // now read the ADC a few times so we can force it to uptake its parameters.
    for (int i = 0; i < 10; i++) {analogRead(A0);}

    // wait 15 ms just to be sure ADC is all done.
    delay(15);
}

/**
 * Setup LSM9DS1
 */
void setupLsm() {
    // Try to initialise and warn if we couldn't detect the chip
    if (!lsm.begin()) {
        Serial.println("Check the wiring to the LSM9DS1");
        // while (1);
    }
    Serial.println("Found LSM9DS1 9DOF");
  
    // 1.) Set the accelerometer range
    lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
    //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
    //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
    //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
    // 2.) Set the magnetometer sensitivity
    lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
    //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
    //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
    //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

    // 3.) Setup the gyroscope
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
    //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
    //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

char promptKey(char *msg) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(msg);
    lcd.setCursor(0, 1);
    lcd.print("(press any key)");
    char ret_key = getKey();
    lcd.clear();
    return ret_key;
}

void readLsm() {
    lsm.read();  /* ask it to read in the data */ 

    /* Get a new sensor event */ 
    sensors_event_t a, m, g, temp;

    lsm.getEvent(&a, &m, &g, &temp); 

    Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
    Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
    Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");

    Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
    Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
    Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" gauss");

    Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" dps");
    Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" dps");
    Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" dps");

    Serial.println();
    delay(200);
}

void setup() {
    setupSerial();
    setupSd();
    setupGps();
    setupLcd();
    setupAccel();
    setupLsm();
    
    // initialize the grand averages
    avg_xavg_0 = avg_yavg_0 = avg_zavg_0 = 0;

    // Create the averager
    AvgADXL accelAvg;

    //////////////// ax calibration //////////////////
    promptKey("Calibrating X");

    accelAvg.take_avg(number_to_average);
    xavg_g = accelAvg.x;
    yavg_0 = accelAvg.y;
    zavg_0 = accelAvg.z;

    avg_yavg_0 += accelAvg.y;
    avg_zavg_0 += accelAvg.z;

    Serial.println("Done calibrating ax");
    Serial.print("xavg_g = "); Serial.println(xavg_g);
    Serial.print("yavg_0 = "); Serial.println(yavg_0);
    Serial.print("zavg_0 = "); Serial.println(zavg_0);

    //////////////// ay calibration //////////////////
    promptKey("Calibrating Y");

    accelAvg.take_avg(number_to_average);
    xavg_0 = accelAvg.x;
    yavg_g = accelAvg.y;
    zavg_0 = accelAvg.z;

    avg_xavg_0 += accelAvg.x;
    avg_zavg_0 += accelAvg.z;

    Serial.println("Done calibrating ay");
    Serial.print("xavg_0 = "); Serial.println(xavg_0);
    Serial.print("yavg_g = "); Serial.println(yavg_g);
    Serial.print("zavg_0 = "); Serial.println(zavg_0);

    //////////////// az calibration //////////////////
    promptKey("Calibrating Z");

    accelAvg.take_avg(number_to_average);
    xavg_0 = accelAvg.x;
    yavg_0 = accelAvg.y;
    zavg_g = accelAvg.z;

    avg_xavg_0 += accelAvg.x;
    avg_yavg_0 += accelAvg.y;

    Serial.println("Done calibrating az");
    Serial.print("xavg_0 = "); Serial.println(xavg_0);
    Serial.print("yavg_0 = "); Serial.println(yavg_0);
    Serial.print("zavg_g = "); Serial.println(zavg_g);

    // print the previous values

    Serial.println("\nOld accelerometer calibration parameters");

    Serial.print("ADC counts for zero acceleration: x = "); Serial.print(xRawZero);
    Serial.print("  y = "); Serial.print(yRawZero);
    Serial.print("  z = "); Serial.println(zRawZero);

    Serial.print("Counts per g of acceleration: x = "); Serial.print(xScale);
    Serial.print("  y = "); Serial.print(yScale);
    Serial.print("  z = "); Serial.println(zScale);

    // now recalculate stuff. we can average the two measurements taken for each
    // axis that were perpendicular to the earth's gravitational field.

    xRawZero = avg_xavg_0 / 2.;
    yRawZero = avg_yavg_0 / 2.;
    zRawZero = avg_zavg_0 / 2.;

    xRawOneG = xavg_g;
    yRawOneG = yavg_g;
    zRawOneG = zavg_g;

    // scale factors: ADC counts per g (= 9.81 m/sec^2).
    xScale = xRawOneG - xRawZero;
    yScale = yRawOneG - yRawZero;
    zScale = zRawOneG - zRawZero;

////////////////////// **ACCELEROMETER END** //////////////////////

    promptKey("Ready!");
}




////////////////////// GPS START //////////////////////

SIGNAL(TIMER0_COMPA_vect)
{
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
        if (c) UDR0 = c;
    // writing direct to UDR0 is much much faster than Serial.print
    // but only one character can be written at a time.
}

void useInterrupt(boolean v)
{
    if (v)
    {
        // Timer0 is already used for millis() - we'll just interrupt somewhere
        // in the middle and call the "Compare A" function above
        OCR0A = 0xAF;
        TIMSK0 |= _BV(OCIE0A);
        usingInterrupt = true;
    }
    else
    {
        // do not call the interrupt function COMPA anymore
        TIMSK0 &= ~_BV(OCIE0A);
        usingInterrupt = false;
    }
}
////////////////////// **GPS END** //////////////////////

void loop() {
////////////////////// GPS START //////////////////////
    {
        if (GPS.newNMEAreceived())
        {
            GPS.parse(GPS.lastNMEA());
        }
    }
    Serial.print("Latitude = "); Serial.print(GPS.latitude);
    Serial.print("\nLongitude = "); Serial.println(GPS.longitude);

    myFile = SD.open("DataLog.txt", FILE_WRITE);
    myFile.print(GPS.latitude, 4);
    myFile.print(",");
    myFile.print(GPS.longitude, 4);
    myFile.print(",");
    myFile.close();
////////////////////// **GPS END** //////////////////////




////////////////////// ACCELEROMETER START //////////////////////
// only need the analog read data for each axis no need to do any calculation
    float x = analogRead(axInput);
    float y = analogRead(ayInput);
    float z = analogRead(azInput);
//Take the difference in the readings and divide it by the scale to get it's value in Gs
    float xG_force = (x - xRawZero)/(xScale);
    float yG_force = (y - yRawZero)/(yScale); //
    float zG_force = (z - zRawZero)/(zScale);
    // multiply by 9.81 to get the force in m/s^2
    float xMs = xG_force * 9.81;
    float yMs = yG_force * 9.81;
    float zMs = zG_force * 9.81;
    myFile = SD.open("DataLog.txt", FILE_WRITE);
    myFile.print(xMs);myFile.print('/');
    myFile.print(yMs);myFile.print('/');
    myFile.print(zMs);myFile.print(',');
    myFile.println();
    myFile.close();

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(GPS.latitude, 4);
    lcd.print(",");
    lcd.print(GPS.longitude, 4);

    lcd.setCursor(0,1);
    lcd.print((int)xMs);lcd.print('/');
    lcd.print((int)yMs);lcd.print('/');
    lcd.print((int)zMs);
    readLsm();
    
////////////////////// **ACCELEROMETER END** //////////////////////
}
