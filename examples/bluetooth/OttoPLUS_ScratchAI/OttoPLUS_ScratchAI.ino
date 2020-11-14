/**
 * Otto DIY Scratch AI firmware with standard baudrate of 9600 for BLE Bluetooth modules.
 * This code have all modes and functions therefore memory is almost full but ignore the alert it works perfectly.
 * Designed to work with the basic Otto or PLUS  or other biped robots. some of these functions will need a good power source such as a LIPO battery.
 * Otto DIY invests time and resources providing open source code and hardware,  please support by purchasing kits from (https:// www.ottodiy.com)
 *
 * If you wish to use this software under Open Source Licensing, you must contribute all your source code to the community and all text above must be
 * included in any redistribution in accordance with the GPL Version 2 when your application is distributed. See http:// www.gnu.org/copyleft/gpl.html
 *
 * ADDED Progmem for MOUTHS and GESTURES: Paul Van De Veen October 2018
 * ADDED PIN definitions for ease of use: Jason Snow November 2018
 * ADDED Battery meassurementin in mode 3 Jason Snow August 2019
 * ADDED changed to original Software serial library Camilo Parra May 2020
 * DELETED interrupts and modes to use BTserial Camilo Parra May 2020
 * ADDED BLE communication for Scratch AI Jorge Gonzalez August 2020 https:// ottoschool.com/scratch/
 * ADDED LEDs Neo pixel control for fun by Jorge Gonzalez August 2020
 *
 */
#include <EEPROM.h>
#include <SerialCommand.h> // Library to manage serial commands
#include <Otto9.h>
#include "Adafruit_LEDBackpack.h"

Otto9 Otto;  // This is Otto!

/**
 *                   ╭─────────╮
 *                   │  O   O  │
 *                   ├─────────┤
 *                   │         │
 *   RIGHT LEG  3    ├──┬───┬──┤    LEFT LEG  2
 *                   ├──┤   ├──┤
 *   RIGHT FOOT 5  ━━┷━━┙   ┕━━┷━━  LEFT FOOT 4
 */

/**
 * Servo PINs
 */
#define PIN_LEG_LEFT   2 // servo[0]  left leg
#define PIN_LEG_RIGHT  3 // servo[1]  right leg
#define PIN_FOOT_LEFT  4 // servo[2]  left foot
#define PIN_FOOT_RIGHT 5 // servo[3]  right foot

/**
 * Servo Assembly PIN
 * to help assemble Otto's feet and legs - wire link between pin 7 and GND
 */
#define PIN_ASSEMBLY   7   // ASSEMBLY pin (7) LOW = assembly    HIGH  = normal operation

/**
 * Ultrasonic PINs
 */
#define PIN_US_TRIGGER 8  // TRIGGER pin (8)
#define PIN_US_ECHO    9  // ECHO pin (9)

/**
 * BLE Module PINs
 */
#define PIN_BLE_STATE  10
#define PIN_BLE_TX     11
#define PIN_BLE_RX     12

/**
 * Buzzer PIN
 */
#define PIN_BUZZER     13 // BUZZER pin (13)

/**
 * Touch Sensor or Push Button
 */
#define PIN_BUTTON     A0 // TOUCH SENSOR Pin (A0) PULL DOWN RESISTOR MAYBE REQUIRED to stop false interrupts (interrupt PIN)

/**
 * LED Mouth Matrix PINs
 */
#define PIN_MOUTH_CLK  A1 // CLK pin (A1)
#define PIN_MOUTH_CS   A2 // CS  pin (A2)
#define PIN_MOUTH_DIN  A3 // DIN pin (A3)
#define LED_DIRECTION   1 // LED MATRIX CONNECTOR position (orientation) 1 = top 2 = bottom 3 = left 4 = right  DEFAULT = 1

/**
 * Sound Sensor PIN
 */
#define PIN_NOISE      A6  // SOUND SENSOR   ANALOG pin (A6)

/**
 * Global Variables
 */

SoftwareSerial BTserial = SoftwareSerial(PIN_BLE_TX, PIN_BLE_RX); //  TX  RX of the Bluetooth
SerialCommand SCmd(BTserial);  // The SerialCommand object

const char programID[] = "OttoPLUS_Scratch_V9"; // Each program will have a ID
const char factory_name = '$'; // Factory name
const char first_name = '#'; // First name

Adafruit_8x16matrix eyesMatrix = Adafruit_8x16matrix();

/**
 * Movement parameters
 */
int duration = 1000;     // Initial duration of movement
int moveID = 0;          // Number of movement
int moveSize = 15;       // Asociated with the height of some movements

/**
 * Button
 */
volatile bool buttonPushed = false;  // Variable to remember when a button has been pushed
unsigned long previousMillis = 0;
int randomDance = 0;
int randomSteps = 0;
bool obstacleDetected = false;
double batteryCHECK = 0;

/**
 * LEDs Matrix 8x8 MAX7219ENG
 */
unsigned long int mouthMatrix;

/**
 * LED Matrix images 8x16
 */
static const uint8_t PROGMEM
logo_bmp[] = {  B01111110,B10000001,B10111001,B10101001,B10111001,B10010001,B10111001,B10010001,B10010001,B10111001,B10010001,B10111001,B10101001,B10111001,B10000001,B01111110},
happy_bmp[] = {  B00000000,B00111100,B00000010,B00000010,B00000010,B00000010,B00111100,B00000000,B00000000,B00111100,B00000010,B00000010,B00000010,B00000010,B00111100,B00000000},
eyes_bmp[] = {  B00000000,B00111100,B01000010,B01001010,B01000010,B01000010,B00111100,B00000000,B00000000,B00111100,B01000010,B01001010,B01000010,B01000010,B00111100,B00000000},
sad_bmp[] =  {  B00000000,B00010000,B00010000,B00010000,B00010000,B00010000,B00010000,B00000000,B00000000,B00010000,B00010000,B00010000,B00010000,B00010000,B00010000,B00000000},
xx_bmp[] =  {  B00000000,B00100010,B00010100,B00001000,B00010100,B00100010,B00000000,B00000000,B00000000,B00000000,B00100010,B00010100,B00001000,B00010100,B00100010,B00000000},
XX_bmp[] = {  B01000001,B00100010,B00010100,B00001000,B00010100,B00100010,B01000001,B00000000,B00000000,B01000001,B00100010,B00010100,B00001000,B00010100,B00100010,B01000001},
angry_bmp[] = {  B00000000,B00011110,B00111100,B01111000,B01110000,B00100000,B00000000,B00000000,B00000000,B00000000,B00100000,B01110000,B01111000,B00111100,B00011110,B00000000},
angry2_bmp[] = {  B00000000,B00000010,B00000100,B00001000,B00010000,B00100000,B00000000,B00000000,B00000000,B00000000,B00100000,B00010000,B00001000,B00000100,B00000010,B00000000},
sleep_bmp[] = {  B00000000,B00100010,B00110010,B00101010,B00100110,B00100010,B00000000,B00000000,B00000000,B00000000,B00100010,B00110010,B00101010,B00100110,B00100010,B00000000},
freetful_bmp[] = {  B00000000,B00100000,B00010000,B00001000,B00000100,B00000010,B00000000,B00000000,B00000000,B00000000,B00000010,B00000100,B00001000,B00010000,B00100000,B00000000},
love_bmp[] = {  B00000000,B00001100,B00011110,B00111100,B00111100,B00011110,B00001100,B00000000,B00000000,B00001100,B00011110,B00111100,B00111100,B00011110,B00001100,B00000000},
confused_bmp[] = {  B00000000,B01111100,B10000010,B10111010,B10101010,B10001010,B01111000,B00000000,B00000000,B01111100,B10000010,B10111010,B10101010,B10001010,B01111000,B00000000},
wave_bmp[] = {  B00000000,B00100000,B00010000,B00001000,B00010000,B00100000,B00010000,B00000000,B00000000,B00100000,B00010000,B00001000,B00010000,B00100000,B00010000,B00000000},
magic_bmp[] = {  B00000000,B00000000,B01111110,B11111111,B01111110,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B01111110,B11111111,B01111110,B00000000,B00000000},
fail_bmp[] = {  B00000000,B00110000,B01111000,B01111000,B01111100,B00111100,B00001000,B00000000,B00000000,B00001000,B00111100,B01111100,B01111000,B01111000,B00110000,B00000000},
full_bmp[] =  {   B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111 };

/**
 * BLE Connection status
 */
int msToDefineConnection = 1500; // some time longer than the pulse
long highStartTimestamp;
int lastConnState = LOW;
bool confirmedConnected = false;

/**
 * Touch Sensor
 */
int lastTouchState = LOW;
int currentTouchState;

/**
 * Setup
 */
void setup() {
    // Serial communication initialization
    Serial.begin(9600);

    // Bluetooth communication initialization
    BTserial.begin(9600);
    pinMode(PIN_BLE_STATE, INPUT);
  
    // Otto initialization
    Otto.init(PIN_LEG_LEFT, PIN_LEG_RIGHT, PIN_FOOT_LEFT, PIN_FOOT_RIGHT, true, PIN_NOISE, PIN_BUZZER, PIN_US_TRIGGER, PIN_US_ECHO); // Set the servo pins and ultrasonic pins

    // Mouth LED Matrix initialization
    Otto.initMATRIX( PIN_MOUTH_DIN, PIN_MOUTH_CS, PIN_MOUTH_CLK, LED_DIRECTION);   // set up Matrix display pins = DIN pin,CS pin, CLK pin, MATRIX orientation 
    Otto.matrixIntensity(1);// set up Matrix display intensity

    // Eyes LED Matrix initialization
    eyesMatrix.begin(0x70);  // pass in the address
    eyesMatrix.setBrightness(0);
    
    // Assembly Mode initialization
    randomSeed(analogRead(PIN_NOISE));   // Set a random seed
    pinMode(PIN_ASSEMBLY, INPUT_PULLUP); // - Easy assembly pin - LOW is assembly Mode

    // Touch Sensor initialization
    pinMode(PIN_BUTTON, INPUT); // - ensure pull-down resistors are used

    // Setup callbacks for SerialCommand RECEIVE commands
    SCmd.addCommand("S", receiveStop);      //  sendAck & sendFinalAck
    SCmd.addCommand("L", receiveLED);       //  sendAck & sendFinalAck
    SCmd.addCommand("T", receiveBuzzer);    //  sendAck & sendFinalAck
    SCmd.addCommand("M", receiveMovement);  //  sendAck & sendFinalAck
    SCmd.addCommand("H", receiveGesture);   //  sendAck & sendFinalAck
    SCmd.addCommand("K", receiveSing);      //  sendAck & sendFinalAck
    SCmd.addCommand("J", receiveMode);      //  sendAck & sendFinalAck
    SCmd.addCommand("C", receiveTrims);     //  sendAck & sendFinalAck
    SCmd.addCommand("G", receiveServo);     //  sendAck & sendFinalAck
    SCmd.addCommand("R", receiveName);      //  sendAck & sendFinalAck

    // Setup callbacks for SerialCommand REQUEST commands
    SCmd.addCommand("E", requestName);
    SCmd.addCommand("D", requestDistance);
    SCmd.addCommand("N", requestNoise);
    SCmd.addCommand("I", requestProgramId);

    SCmd.addDefaultHandler(receiveStop);

    // Otto wake up!
    Otto.sing(S_connection);
    Otto.home();
    // Animation Uuuuuh - A little moment of initial surprise
    for (int i = 0; i < 2; i++) {
        for (int i = 0; i < 8; i++) {
            Otto.putAnimationMouth(littleUuh, i);
            delay(150);
        }
    }
    // Smile for a happy Otto :)
    Otto.putMouth(smile);
    Otto.sing(S_happy);
    delay(200);

    // If Otto's name is '#' means that Otto hasn't been baptized
    // In this case, Otto does a longer greeting
    // 5 = EEPROM address that contains first name character
    if (EEPROM.read(5) == first_name) {
        Otto.jump(1, 700);
        delay(200);
        Otto.shakeLeg(1, duration, 1);
        Otto.putMouth(smallSurprise);
        Otto.swing(2, 800, 20);
        Otto.home();
    }

    Otto.putMouth(happyOpen);
    previousMillis = millis();
    // if Pin 7 is LOW then place OTTO's servos in home mode to enable easy assembly,
    // when you have finished assembling Otto, remove the link between pin 7 and GND
    while (digitalRead(PIN_ASSEMBLY) == LOW) {
        Otto.home();
        Otto.sing(S_happy_short);   // sing every 5 seconds so we know OTTO is still working
        delay(5000);
    }

    //eyesMatrix.clear();
    //eyesMatrix.drawBitmap(0, 0, + eyes_bmp , 8, 16, LED_ON);
    //eyesMatrix.writeDisplay();
}

/**
 * Main Loop
 */
void loop() {
    checkIfTouched();
    checkIfConnected();
    SCmd.readSerial();    // If Otto is moving yet
    if (Otto.getRestState() == false) {
        move(moveID);
    }
}

/**
 * Functions
 */

/**
 * Function to read distance sensor & to actualize obstacleDetected variable
 */
void obstacleDetector() {
    int distance = Otto.getDistance();
    if (distance < 15) obstacleDetected = true;
    else obstacleDetected = false;
}

/**
 * Function to receive Stop command.
 */
void receiveStop() {
    sendAck();
    Otto.home();
    sendFinalAck();
}

/**
 * Function to receive LED commands
 */
void receiveLED() {
    // sendAck & stop if necessary
    sendAck();
    Otto.home();
    // Examples of receiveLED Bluetooth commands
    // L 000000001000010100100011000000000
    //unsigned long int mouthMatrix;
    char *arg;
    char *endstr;
    arg = SCmd.next();
    Serial.println(arg);
    if (arg != NULL) {
        mouthMatrix = strtoul(arg, &endstr, 2); // Converts a char string to unsigned long integer
        Otto.putMouth(mouthMatrix, false);
    } else {
        Otto.putMouth(xMouth);
        delay(2000);
        Otto.clearMouth();
    }
    sendFinalAck();
}

/**
 *Function to receive buzzer commands
 */
void receiveBuzzer() {
    // sendAck & stop if necessary
    sendAck();
    Otto.home();

    bool error = false;
    int frec;
    int duration;
    char *arg;

    arg = SCmd.next();
    if (arg != NULL) frec = atoi(arg);  // Converts a char string to an integer
    else error = true;

    arg = SCmd.next();
    if (arg != NULL) duration = atoi(arg);  // Converts a char string to an integer
    else error = true;
    if (error == true) {
        Otto.putMouth(xMouth);
        delay(2000);
        Otto.clearMouth();
    } else Otto._tone(frec, duration, 1);
    sendFinalAck();
}

/**
 * Function to receive TRims commands
 */
void receiveTrims() {
    // sendAck & stop if necessary
    sendAck();
    Otto.home();
    int trimLegLeft, trimLegRIGHT, trimFootLeft, trimFootRight;

    // Definition of Servo Bluetooth command
    // C trimLegLeft trimLegRIGHT trimFootLeft trimFootRight
    // Examples of receiveTrims Bluetooth commands
    // C 20 0 -8 3
    bool error = false;
    char *arg;
    arg = SCmd.next();
    if (arg != NULL) trimLegLeft = atoi(arg);  // Converts a char string to an integer
    else error = true;

    arg = SCmd.next();
    if (arg != NULL) trimLegRIGHT = atoi(arg);  // Converts a char string to an integer
    else error = true;

    arg = SCmd.next();
    if (arg != NULL) trimFootLeft = atoi(arg);  // Converts a char string to an integer
    else error = true;

    arg = SCmd.next();
    if (arg != NULL) trimFootRight = atoi(arg);  // Converts a char string to an integer
    else error = true;
    if (error == true) {
        Otto.putMouth(xMouth);
        delay(2000);
        Otto.clearMouth();

    } else { // Save it on EEPROM
        Otto.setTrims(trimLegLeft, trimLegRIGHT, trimFootLeft, trimFootRight);
        Otto.saveTrimsOnEEPROM(); // Uncomment this only for one upload when you finaly set the trims.
    }
    sendFinalAck();
}

/**
 * Function to receive Servo commands
 */
void receiveServo() {
    sendAck();
    moveID = 30;

    // Definition of Servo Bluetooth command
    // G  servoLegLeft servoLegRight servoFootLeft servoFootRight
    // Example of receiveServo Bluetooth commands
    // G 90 85 96 78
    bool error = false;
    char *arg;
    int servoLegLeft, servoLegRight, servoFootLeft, servoFootRight;

    arg = SCmd.next();
    if (arg != NULL) servoLegLeft = atoi(arg);  // Converts a char string to an integer
    else error = true;

    arg = SCmd.next();
    if (arg != NULL) servoLegRight = atoi(arg);  // Converts a char string to an integer
    else error = true;

    arg = SCmd.next();
    if (arg != NULL) servoFootLeft = atoi(arg);  // Converts a char string to an integer
    else error = true;

    arg = SCmd.next();
    if (arg != NULL) {
        servoFootRight = atoi(arg);  // Converts a char string to an integer
    } else error = true;
    if (error == true) {
        Otto.putMouth(xMouth);
        delay(2000);
        Otto.clearMouth();
    } else { // Update Servo:
        int servoPos[4] = {servoLegLeft, servoLegRight, servoFootLeft, servoFootRight};
        Otto._moveServos(200, servoPos);   // Move 200ms
    }
    sendFinalAck();
}

/**
 * Function to receive movement commands
 */
void receiveMovement() {
    sendAck();
    if (Otto.getRestState() == true) Otto.setRestState(false);

    // Definition of Movement Bluetooth commands
    // M  moveID  duration   moveSize
    char *arg;
    arg = SCmd.next();
    if (arg != NULL) moveID = atoi(arg);
    else {
        Otto.putMouth(xMouth);
        delay(2000);
        Otto.clearMouth();
        moveID = 0; // stop
    }
    arg = SCmd.next();
    if (arg != NULL) duration = atoi(arg);
    else duration = 1000;
    arg = SCmd.next();
    if (arg != NULL) moveSize = atoi(arg);
    else moveSize = 15;
}

/**
 * Function to execute the right movement according the movement command received.
 * @param moveID
 */
void move(int moveID) {
    bool manualMode = false;
    switch (moveID) {
        case 0:
            Otto.home();
            break;
        case 1: // M 1 1000
            Otto.walk(1, duration, 1);
            break;
        case 2: // M 2 1000
            Otto.walk(1, duration, -1);
            break;
        case 3: // M 3 1000
            Otto.turn(1, duration, 1);
            break;
        case 4: // M 4 1000
            Otto.turn(1, duration, -1);
            break;
        case 5: // M 5 1000 30
            Otto.updown(1, duration, moveSize);
            break;
        case 6: // M 6 1000 30
            Otto.moonwalker(1, duration, moveSize, 1);
            break;
        case 7: // M 7 1000 30
            Otto.moonwalker(1, duration, moveSize, -1);
            break;
        case 8: // M 8 1000 30
            Otto.swing(1, duration, moveSize);
            break;
        case 9: // M 9 1000 30
            Otto.crusaito(1, duration, moveSize, 1);
            break;
        case 10: // M 10 1000 30
            Otto.crusaito(1, duration, moveSize, -1);
            break;
        case 11: // M 11 1000
            Otto.jump(1, duration);
            break;
        case 12: // M 12 1000 30
            Otto.flapping(1, duration, moveSize, 1);
            break;
        case 13: // M 13 1000 30
            Otto.flapping(1, duration, moveSize, -1);
            break;
        case 14: // M 14 1000 20
            Otto.tiptoeSwing(1, duration, moveSize);
            break;
        case 15: // M 15 500
            Otto.bend(1, duration, 1);
            break;
        case 16: // M 16 500
            Otto.bend(1, duration, -1);
            break;
        case 17: // M 17 500
            Otto.shakeLeg(1, duration, 1);
            break;
        case 18: // M 18 500
            Otto.shakeLeg(1, duration, -1);
            break;
        case 19: // M 19 500 20
            Otto.jitter(1, duration, moveSize);
            break;
        case 20: // M 20 500 15
            Otto.ascendingTurn(1, duration, moveSize);
            break;
        default:
            manualMode = true;
            break;
    }
    if (!manualMode) sendFinalAck();
    BTserial.println("M:END");
}

/**
 * Function to receive gesture commands
 */
void receiveGesture() {
    // sendAck & stop if necessary
    sendAck();
    Otto.home();

    // Definition of Gesture Bluetooth commands
    // H gestureID
    int gesture = 0;
    char *arg;
    arg = SCmd.next();
    if (arg != NULL) gesture = atoi(arg);
    else delay(2000); // Otto.putMouth(xMouth); // Otto.clearMouth();
    switch (gesture) {
        case 1: // H 1
            Otto.playGesture(OttoHappy);
            break;
        case 2: // H 2
            Otto.playGesture(OttoSuperHappy);
            break;
        case 3: // H 3
            Otto.playGesture(OttoSad);
            break;
        case 4: // H 4
            Otto.playGesture(OttoSleeping);
            break;
        case 5: // H 5
            Otto.playGesture(OttoFart);
            break;
        case 6: // H 6
            Otto.playGesture(OttoConfused);
            break;
        case 7: // H 7
            Otto.playGesture(OttoLove);
            break;
        case 8: // H 8
            Otto.playGesture(OttoAngry);
            break;
        case 9: // H 9
            Otto.playGesture(OttoFretful);
            break;
        case 10: // H 10
            Otto.playGesture(OttoMagic);
            break;
        case 11: // H 11
            Otto.playGesture(OttoWave);
            break;
        case 12: // H 12
            Otto.playGesture(OttoVictory);
            break;
        case 13: // H 13
            Otto.playGesture(OttoFail);
            break;
        default:
            break;
    }
    sendFinalAck();
    BTserial.println("H:END");
}

/**
 * Function to receive sing commands
 */
void receiveSing() {
    // sendAck & stop if necessary
    sendAck();
    Otto.home();
    // Definition of Sing Bluetooth commands
    // K  SingID
    int sing = 0;
    char *arg;
    arg = SCmd.next();
    if (arg != NULL) sing = atoi(arg);
    else delay(2000); // Otto.putMouth(xMouth); // Otto.clearMouth();
    switch (sing) {
        case 1: // K 1
            Otto.sing(S_connection);
            break;
        case 2: // K 2
            Otto.sing(S_disconnection);
            break;
        case 3: // K 3
            Otto.sing(S_surprise);
            break;
        case 4: // K 4
            Otto.sing(S_OhOoh);
            break;
        case 5: // K 5
            Otto.sing(S_OhOoh2);
            break;
        case 6: // K 6
            Otto.sing(S_cuddly);
            break;
        case 7: // K 7
            Otto.sing(S_sleeping);
            break;
        case 8: // K 8
            Otto.sing(S_happy);
            break;
        case 9: // K 9
            Otto.sing(S_superHappy);
            break;
        case 10: // K 10
            Otto.sing(S_happy_short);
            break;
        case 11: // K 11
            Otto.sing(S_sad);
            break;
        case 12: // K 12
            Otto.sing(S_confused);
            break;
        case 13: // K 13
            Otto.sing(S_fart1);
            break;
        case 14: // K 14
            Otto.sing(S_fart2);
            break;
        case 15: // K 15
            Otto.sing(S_fart3);
            break;
        case 16: // K 16
            Otto.sing(S_mode1);
            break;
        case 17: // K 17
            Otto.sing(S_mode2);
            break;
        case 18: // K 18
            Otto.sing(S_mode3);
            break;
        case 19: // K 19
            Otto.sing(S_buttonPushed);
            break;
        default:
            break;
    }
    sendFinalAck();
}

/**
 * Function to receive mode selection.
 */
void receiveMode() {
    sendAck();
    Otto.home();
    // Definition of Mode Bluetooth commands
    // J ModeID
    int modeId = 0;
    char *arg;
    arg = SCmd.next();
    if (arg != NULL) modeId = atoi(arg);
    else delay(2000);
    switch (modeId) {
        case 0: // J 0
            Otto.putMouth(0);
            Otto.sing(S_cuddly);
            Otto.home();
            break;
        case 1: // J 1
            Otto.putMouth(one);
            randomDance = random(5, 21); // 5,20
            if ((randomDance > 14) && (randomDance < 19)) {
                randomSteps = 1;
                duration = 1600;
            } else {
                randomSteps = random(3, 6); // 3,5
                duration = 1000;
            }
            Otto.putMouth(random(10, 21));
            for (int i = 0; i < randomSteps; i++) move(randomDance);
            break;
        case 2: // J 2
            Otto.putMouth(two);
            break;
        case 3: // J 3
            Otto.putMouth(three);
            break;
        case 4: // J 4
            Otto.putMouth(four);
            break;
        default:
            break;
    }
    sendFinalAck();
}

/**
 * Function to receive Name command
 */
void receiveName() {
    // sendAck & stop if necessary
    sendAck();
    Otto.home();
    char newOttoName[11] = "";  // Variable to store data read from Serial.
    int eeAddress = 5;          // Location we want the data to be in EEPROM.
    char *arg;
    arg = SCmd.next();
    if (arg != NULL) {
        // Complete newOttoName char string
        int k = 0;
        while ((*arg) && (k < 11)) {
            newOttoName[k] = *arg++;
            k++;
        }
        EEPROM.put(eeAddress, newOttoName);
        BTserial.write("AT+NAMEOTTODIA");
        // BTserial.println(newOttoName);
    } else {
        // Otto.putMouth(xMouth);
        delay(2000);
        // Otto.clearMouth();
    }
    sendFinalAck();
}

/**
 * Function to send Otto's name
 */
void requestName() {
    Otto.home(); // stop if necessary
    char actualOttoName[11] = ""; // Variable to store data read from EEPROM.
    int eeAddress = 5;            // EEPROM address to start reading from
    // Get the float data from the EEPROM at position 'eeAddress'
    EEPROM.get(eeAddress, actualOttoName);
    Serial.print(F("&&"));
    Serial.print(F("E "));
    Serial.print(actualOttoName);
    BTserial.print("E:");
    BTserial.println(actualOttoName);
    Serial.println(F("%%"));
    Serial.flush();
}

/**
 * Function to send ultrasonic sensor measure (distance in "cm")
 */
void requestDistance() {
    Otto.home();  // stop if necessary
    int distance = Otto.getDistance();
    Serial.print(F("&&"));
    Serial.print(F("D "));
    Serial.print(distance);
    BTserial.print("D:");
    BTserial.println(distance);
    Serial.println(F("%%"));
    Serial.flush();
}

/**
 * Function to send noise sensor measure
 */
void requestNoise() {
    Otto.home();  // stop if necessary
    int microphone = Otto.getNoise(); // analogRead(PIN_NOISE);
    Serial.print(F("&&"));
    Serial.print(F("N "));
    Serial.print(microphone);
    BTserial.print("N:");
    BTserial.println(microphone);
    Serial.println(F("%%"));
    Serial.flush();
}

/**
 * Function to send battery voltage percent
 */
void requestBattery() {
    Otto.home();  // stop if necessary
    // The first read of the battery is often a wrong reading, so we will discard this value.
    double batteryLevel = Otto.getBatteryLevel();
    Serial.print(F("&&"));
    Serial.print(F("B "));
    Serial.print(batteryLevel);
    BTserial.print("B:");
    BTserial.println(batteryLevel);
    Serial.println(F("%%"));
    Serial.flush();
}

/**
 * Function to send program ID
 */
void requestProgramId() {
    Otto.home();   // stop if necessary
    Serial.print(F("&&"));
    Serial.print(F("I "));
    Serial.print(programID);
    BTserial.print("I:");
    BTserial.println(programID);
    Serial.println(F("%%"));
    Serial.flush();
}

/**
 * Function to send Ack comand (A)
 */
void sendAck() {
    delay(30);
    Serial.print(F("&&"));
    Serial.print(F("A"));
    Serial.println(F("%%"));
    Serial.flush();
}

/**
 * Function to send final Ack comand (F)
 */
void sendFinalAck() {
    delay(30);
    Serial.print(F("&&"));
    Serial.print(F("F"));
    Serial.println(F("%%"));
    Serial.flush();
}

/**
 * Function executed when  button is pushed / Touch sensor VIA INTERRUPT routine
 */
void ButtonPushed() {
    if (!buttonPushed) {
        buttonPushed = true;
        Otto.putMouth(smallSurprise);
    }
}

/**
 * Function to Read Touch Sensor
 */
void checkIfTouched() {
    currentTouchState = digitalRead(PIN_BUTTON);
    if(lastTouchState == LOW && currentTouchState == HIGH) {
        Otto.sing(S_buttonPushed);
        Serial.println("The sensor is touched");
        BTserial.println(F("T:END"));
    }
    lastTouchState = currentTouchState;
}

/**
 * Function to check BLE connection status
 * @return TRUE if connected
 */
bool checkIfConnected() {
    int state = digitalRead(PIN_BLE_STATE);
    long now = millis();
    if (state == HIGH) {
        if (confirmedConnected == false) {
            if (lastConnState == LOW) {
                // start the timer for HIGH
                highStartTimestamp = now;
                lastConnState = HIGH;
            } else {
                if (now - highStartTimestamp >= msToDefineConnection) {
                    Serial.println("C");
                    confirmedConnected = true;
                    // do stuff like control an LED
                    Otto.sing(S_connection);
                    // Empty the buffer
                    while (BTserial.available() > 0) {
                        char t = BTserial.read();
                        Serial.write(t);
                    }
                }
            }
        }
    } else { // state is LOW
        if (lastConnState == HIGH && confirmedConnected == true) {
            Otto.sing(S_disconnection);
            Serial.println("D");
            confirmedConnected = false;
        }
        lastConnState = LOW;
    }
    return confirmedConnected;
}
