/*
  MAX30102 Breakout: Take IR reading to sense presence
  By: Vova Dumanskyi
  Date: April 11nd, 2018
  https://github.com/sparkfun/MAX30105_Breakout

  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA) for ESP8266 D2
  -SCL = A5 (or SCL) for ESP8266 D1
  -INT = 5 (digital) for ESP8266 D3

  The MAX30105 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
  but it will also run at 3.3V.

*/

#define MAX30102 //Choise model
#define STORAGE_SIZE_4 100//Each long is 9 bytes so limit this to fit on your micro
//Every 9 bytes is 4 samples!

#include <MAX30105.h> 

#define Project_DEBUG true // SWITCHER DEBUGGING ON/OFF

#if Project_DEBUG == true
#define DEBUG_Serial Serial
//#define debug Serial
#define DEBUG_print(...)    DEBUG_Serial.print(__VA_ARGS__)
#define DEBUG_println(...)  DEBUG_Serial.println(__VA_ARGS__)
#define DEBUG_begin(...)    DEBUG_Serial.begin(__VA_ARGS__)
#define DEBUG_printf(...)   DEBUG_Serial.printf(__VA_ARGS__)
#define DEBUG_flush(...)    DEBUG_Serial.flush(__VA_ARGS__)
#define DEBUG_var(token)  { DEBUG_print( #token " = "); DEBUG_println(token); }
#else
#define DEBUG_print(...)
#define DEBUG_println(...)
#define DEBUG_begin(...)
#define DEBUG_printf(...)
#define DEBUG_flush(...)
#define DEBUG_var(token)
#endif // END Project_DEBUG == true

//Permissions for interruptions
#define A_FULL true
#define PPG_RDY false
#define ALC_OVF true
#define PROX_INT false  
#define PWR_RDY true
#define DIE_TEMP_RDY false

#ifdef ESP8266
MAX30105 particleSensor;
#define IRQ D3
#else 
MAX30105 particleSensor;
#define IRQ 5
#endif // END ESP8266

long samplesTaken = 0; //Counter for calculating the Hz or read rate
long unblockedValue; //Average IR at power up
long startTime; //Used to calculate measurement rate
long inerruptTime = micros();

bool visible = true;
bool visible_2 = false;

float temp;

void readReg();

void setup()
{
  DEBUG_begin(115200); // 153600 // 230400
  DEBUG_println("Initializing...");
  DEBUG_print("\n");

  /* setup the IRQ pin*/
  pinMode(IRQ, INPUT); // INPUT_PULLUP

  /*Activate the interrupt*/
  attachInterrupt(IRQ, readReg, FALLING);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    DEBUG_println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }

  //Setup to sense up to 18 inches, max LED brightness
  byte ledBrightness = (byte)(5 *5.1); //Options: 0=Off to 255=50mA
  byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 400; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 2048; //Options: 2048, 4096, 8192, 16384

  /** SpO2 Mode (Allowed Settings),  led mode > 1
   * * * * * * * * * * * * * * * * * * * * * * * *
   *                     *    PULSE WIDTH (μs)   *
   * SAMPLES PER SECOND  * * * * * * * * * * * * *
   *                     *  69 * 118 * 215 * 411 *
   * * * * * * * * * * * * * * * * * * * * * * * *
   *         50          *  OK *  OK *  OK *  OK *
   *        100          *  OK *  OK *  OK *  OK *
   *        200          *  OK *  OK *  OK *  OK *
   *        400          *  OK *  OK *  OK *  OK *
   *        800          *  OK *  OK *  OK *     *
   *       1000          *  OK *  OK *     *     *
   *       1600          *  OK *     *     *     *
   *       3200          *     *     *     *     *
   * * * * * * * * * * * * * * * * * * * * * * * *
   *  Resolution (bits)  *  15 *  16 *  17 *  18 *
   * * * * * * * * * * * * * * * * * * * * * * * *
 
    HR Mode (Allowed Settings),  led mode = 1
   * * * * * * * * * * * * * * * * * * * * * * * *
   *                     *    PULSE WIDTH (μs)   *
   * SAMPLES PER SECOND  * * * * * * * * * * * * *
   *                     *  69 * 118 * 215 * 411 *
   * * * * * * * * * * * * * * * * * * * * * * * *
   *         50          *  OK *  OK *  OK *  OK *
   *        100          *  OK *  OK *  OK *  OK *
   *        200          *  OK *  OK *  OK *  OK *
   *        400          *  OK *  OK *  OK *  OK *
   *        800          *  OK *  OK *  OK *  OK *
   *       1000          *  OK *  OK *  OK *  OK *
   *       1600          *  OK *  OK *  OK *     *
   *       3200          *  OK *     *     *     *
   * * * * * * * * * * * * * * * * * * * * * * * *
   *  Resolution (bits)  *  15 *  16 *  17 *  18 *
   * * * * * * * * * * * * * * * * * * * * * * * */

  /** Slot Timing in Multi-LED Modes (Page 26)
   * 
   *      69us                  931us
   *    │<---->│<---------------------------------->│
   *     ______   RED LED 660nm                      _____
   *  __│      │________________________ _ _ _ _____│     │_________________________
   * 
   *           │    358us      69us                  931us
   *           │<----------->│<---->│<---------------------------------->│
   *   INFRARED LED 880nm     ______                                      _____
   *  _______________________│      │________________________ _ _ _ _____│     │____
   *  
   *  Channel Slot Timing for the SpO2 Mode with a 1kHz Sample Rate
   */

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  if (A_FULL)       particleSensor.enableAFULL(); //Enable the almost full interrupt (default is 32 samples)
  if (PPG_RDY)      particleSensor.enableDATARDY();
  if (ALC_OVF)      particleSensor.enableALCOVF();
  if (PROX_INT)     particleSensor.enablePROXINT();
  if (DIE_TEMP_RDY) particleSensor.enableDIETEMPRDY();
  //particleSensor.disableFIFORollover();

  particleSensor.setFIFOAlmostFull(1); //Set almost full int to fire at 29 samples;
  /* var; time between interruptions
   *  0  --ms (there are no interruptions)
   *  1  78ms
   *  2  75ms
   *  3  72ms
   *  4  70ms
   *  5  68ms
   *  6  66ms
   *  7  63ms
   *  8  60ms
   *  9  58ms
   *  10 55ms
   *  11 53ms
   *  12 50ms
   *  13 48ms
   *  14 46ms
   *  15 43ms
   */
  particleSensor.setPROXINTTHRESH(0x01);

  startTime = micros();
}

void loop()
{
  for (uint8_t i = particleSensor.available(); i != 0; --i, samplesTaken++) {
    DEBUG_print(i);
    DEBUG_print("\t");
    if (visible) {
      DEBUG_print("[IR] = ");
      DEBUG_print(particleSensor.getFIFOIR());
      DEBUG_print("\t[Red] = ");
      DEBUG_print(particleSensor.getFIFORed());
      DEBUG_print("\n");
    }
    particleSensor.nextSample();
  }
  
  if (DIE_TEMP_RDY && visible) { // test
    DEBUG_print("[Temp] = ");
    DEBUG_println(temp);
  } else {
    DEBUG_print("[Temp] = ");
    DEBUG_print(particleSensor.readTemperature());
  }
  
  DEBUG_print("\t[Hz] - ");
  DEBUG_print((float)samplesTaken / ((micros() - startTime) / 1000000.0), 2);
  DEBUG_print("\n");

  delay(500); // Simulation work time (with interrutions)

  // Supports interruptions
  if ( micros() - inerruptTime > 80*1000 ) { particleSensor.check(); particleSensor.readTemperature(); } // 80ms > 78ms
  while (digitalRead(IRQ) == LOW) readReg(); 

#if Project_DEBUG == true
  // Control panel in the Serial
  while(Serial.available() > 0) {
    switch(Serial.read()) {
      case 'r':
      case 'R':
        readReg();
        break;
      case 'c':
        particleSensor.check();  
        break;
      case 'C':
        particleSensor.check();  
        particleSensor.check();  
        break;
      case 't':
      case 'T':
        particleSensor.readTemperature();
        break;
      case 'h':
      case 'H':
        startTime = micros();
        samplesTaken = 0;
        break;
      case 's':
      case 'S':
        if (Serial.available() >= 2) {
          switch(Serial.read()) {
            case 'p':
            case 'P':
              particleSensor.setPROXINTTHRESH(Serial.parseInt()); //Write to the Serial "sp31" or "SP1" or...
              break;
            case 'f':
            case 'F':
              particleSensor.setFIFOAlmostFull(Serial.parseInt()); //Write to the Serial "sf15" or "SF1" or...
              break;
          }
        }
        break;
      case 'v':
        visible = !visible;
        break;
      case 'V':
        visible_2 = !visible_2;
        break;
      case 'W':
        delay(Serial.parseInt());
        break;
      case 'w':
        delay(1000);
        break;
      case 'i': 
      case 'I':
        DEBUG_print("Count Overflow = ");       
        DEBUG_print(particleSensor.getCountOverflow());
        DEBUG_println();
        break;
    }
  }
#endif //Project_DEBUG == true
}

void readReg() {
  if ( micros() - inerruptTime < 100 ) { 
    return; 
  }
  inerruptTime = micros();
  uint8_t reg1 = particleSensor.getINT1();
  uint8_t reg2 = particleSensor.getINT2();
  DEBUG_print("reg1 = ");
  DEBUG_print(reg1, BIN);
  DEBUG_print("\treg2 = ");
  DEBUG_print(reg2, BIN);
  DEBUG_print("\n");

  DEBUG_print("Count Overflow = ");       
  DEBUG_print(particleSensor.getCountOverflow());
  DEBUG_println();
  if (reg1 & 0x01 << 7 && A_FULL)  particleSensor.check(); // FIFO_A_FULL[3:0] register був заповнений
  if (reg1 & 0x01 << 6 && PPG_RDY) particleSensor.check(); // Нові дані пульсації готові
  if (reg1 & 0x01 << 5 && ALC_OVF)  DEBUG_print("WARNING!!! Over Flow"); // Переповнення ануляції наколишнього освітлення
  if (reg1 & 0x01 << 4 && PROX_INT); // Коли досягнута порогова близькість, для колекіонування даних SpO2
  if (reg1 & 0x01 << 0 && PWR_RDY) ; // start
  if (reg2 & 0x01 << 1 && DIE_TEMP_RDY) temp = particleSensor.readTemperature(); // Дані температури готові
}

