/*File: /Users/jamesrobar/Library/Mobile Documents/com~apple~CloudDocs/Arduino Sketches/Arduino/Projects/Xmas Lights/LED_String_RTC*/
char FileName[80] = { "CloudDocs/Arduino Sketches/Arduino*/Projects/Xmas Lights/LED_String_RTC" };  //$09
char LastModified[80] = { "15Apr24 at 9:43 A.M." };                                                //$09
/*

  Last run: 14Apr24
  Status:  
  Description:
       ø  Runs on UNO. pwm pins 3, 5, 6, 9, 10, 11 drive up to six LED strings.

       ø  LED strings driven from pwm pins (limit <= 40 ma o/p max /pin) 4.6 volts at 255 level.

        ø Jumper D4 pin to +5V pin (RTC overide ON)  or to GND (CONTINUOUS; RTC overide OFF). Jumper D2 to +5v (OFF)

  WIP:
  set RTC On and OFF times            $10
  allow 1 printout on RTC out of window; to do @ line 462
  lengthen time each display; tbd; do this after testing; to do
  shorten pause btn displays tbd; do this after testing; done
  declare 'int pwmMax' & define 127; done
  install 'sleep()'; done
  shorten ON times; done
  updte start times; done


  CONNECTORS:

    Display Control:  ON RTC - jumper to +5V, pin 4
                      ON continuous - jumper to GND pin 4
                      OFF - jumper to +5V, pin 2
 
   REAL TIME CLOCK (RTC)
        To set RTC current time, uncomment line  "setDS3231time()".
        Enter the correct time, upload, comment out, then upload again.
        RTC pinout:
            +       : 3.3V-5V
            D (SDA) : A4
            C (SCL) : A5
            NC      : Connect to GND
            -       : GND


       SET (find @ $xx):
                    set RTC overide jumper              $08
                    set RTC On and OFF times            $10    setup ON/OFF 
                    set RTC 'setDS3231time'             $12    in 'Setup()'; load current time into R
                    ouput file info                     $04
                    FileName & LastModified             $09
*/

#include "Wire.h"
#include <LowPower.h>
#define DS3231_I2C_ADDRESS 0x68

//$09
//SET RTC ON/OFF TIMES $10
//{hour,minute,minutes on}
//previous  double turnONtimes[5][4] = { { 28, 00, 360 }, { 5, 0, 210 }, { 19, 00, 180 }, { 28, 0, 255 } };
double turnONtimes[5][4] = { { 28, 00, 360 }, { 28, 0, 0 }, { 20, 0, 120 }, { 28, 0, 255 } };  //  {{T_hrON, T_minON, Delta_T_minON}, ...}}
bool RTC_IsOn_Is = false;                                                                       // changes to 'true' when within RTC ON times
int pwmMax = 100;
int pwmVal = 0;  // TheShow[i][6]   current pwm set value
int NmbrOfStrings = 6;
// < < < < < < < < < < < < < < can select < < < < < < < < < < < < < < FOR
int NmbrOfRoutines = 7;
int RoutineSelect[10] = { 0, 1, 2, 3, 4, 5, 6 };  // e.g. random(0,2) will probably select x: #entries of x/total number of entries  w
int RoutineNum = 0;                               // routine number
// The Light String Profile for Each String
// TheShow[a][b]: {{?, ?, ?, time btw pwm incr, ?, ?, pwm val, ?, pin #, ?} . . . }
unsigned int TheShow[6][10] = {
  { 2, 0, 0, 250, 0, 255, 0, 0, 3, 5000 }, { 2, 0, 0, 1000, 0, 255, 0, 0, 5, 5000 }, { 2, 0, 0, 0, 0, 255, 0, 0, 6, 5000 }, { 2, 0, 0, 0, 0, 255, 0, 0, 9, 5000 }, { 2, 0, 0, 0, 0, 255, 0, 0, 10, 5000 }, { 2, 0, 0, 500, 0, 255, 0, 0, 11, 5000 }
};
unsigned long RoutineDuration;
unsigned long TimeRoutineStarted;
unsigned long TimeRoutineWillEnd;
//Sparkle[x][y] {{ON_time0, ON_duration0 }, ... {ON_time5, ON_duration5}}
unsigned long Sparkle[7][2] = { { 0, 100 }, { 0, 100 }, { 0, 100 }, { 0, 100 }, { 0, 100 }, { 0, 100 } };
void setup() {
  Wire.begin();
  Serial.begin(115200);
  pinMode(2, INPUT);    // ON/OFF select
  pinMode(4, INPUT);    // RTC/ON mode select
  pinMode(3, OUTPUT);   // string 0
  pinMode(5, OUTPUT);   // string 1
  pinMode(6, OUTPUT);   // string 2
  pinMode(9, OUTPUT);   // string 3
  pinMode(10, OUTPUT);  // string 4
  pinMode(11, OUTPUT);  // string 5
  Serial.print("\n\nFileName      ");
  Serial.println(FileName);
  Serial.print("LastModified  ");
  Serial.println(LastModified);
  Serial.print("NmbrOfStrings\t\t");
  Serial.println(NmbrOfStrings);
  Serial.print("NmbrOfRoutines\t\t");
  Serial.println(NmbrOfRoutines);
  randomSeed(analogRead(A1) * analogRead(A2));
  //**********set the RTC current time here**********// $12 // uncomment line "setDS3231time( . . ." , upload, then comment out
  // DS3231 seconds, minutes, hours, day, date, month, year
  //setDS3231time(0, 41, 9, 3, 3, 4, 24);  //uncomment then upload with correct time, then comment out and upload again
  Serial.print("Time\t\t\t");
  printTime();
  Serial.println();
  Serial.print("Hour Turn ON\t\t");
  for (int i = 0; i < 4; i++) {
    Serial.print((float)turnONtimes[i][0] + (float)turnONtimes[i][1] / 60.);
    Serial.print("  ");
  }
  Serial.println();
  TurnShowOffOn(255);
  delay(5000);
  TurnShowOffOn(pwmMax);
  delay(5000);
  TurnShowOffOn(0);
}

void loop() {
  /* *****CHECK FOR CONTINUOUS OFF***** */
  //Serial.println(digitalRead(2));  //ON/OFF select
  if (HIGH == digitalRead(2)) {
    TurnShowOffOn(0);
    Serial.println("Display OFF");
    delay(5);  //allow sleep printout to finish
               /*TEST
    // Enter power down state for 8 s with ADC, BOD & TIMER2 OFF
    Serial.print("Sleep 8s  ");
    delay(5);
    LowPower.powerExtStandby(SLEEP_8S, ADC_OFF, BOD_OFF, TIMER2_OFF);  // 8 second sleep
    Serial.println("\nWake");
    */
    return;    // display OFF
  } else {
    Serial.print("\nDisplay ON ");
  }
  /* *****CHECK FOR CONTINUOUS ON or RTC CONTROL***** */
  if (LOW == digitalRead(4)) {  //to TEST RTC, tie pin4 to gnd; then set to HIGH
    RTC_IsOn_Is = true;         //??NEEDED?? set to fake a time window ON for continuous ON
    TurnShowOffOn(1);
    Serial.println("Continuous");
  } else {  // use Real Time Clock (RTC)
    Serial.println("RTC");
    Check_RTC_ON_OFF();  //check for time window
    if (false == RTC_IsOn_Is) {
      //Serial.println("145 false");  //XXX
      TurnShowOffOn(0);
      /*
      // Enter power down state for 8 s with ADC and BOD module disabled
      Serial.print("Sleep  ");
      LowPower.powerExtStandby(SLEEP_8S, ADC_OFF, BOD_OFF, TIMER2_OFF);  // 2 second sleep
      Serial.println("\nWake");
      */
      return;
    } else {
      //Serial.println("155 true");  //XXX
      TurnShowOffOn(1);
    }
  }
  RandomizeRoutines();
  for (int i = 0; i < NmbrOfRoutines; i++) {
    RoutineNum = RoutineSelect[i];
    //RoutineNum = 2;  //<<******//Uncomment TO TEST //RoutineNum = 2 is ALL ON;Also set RoutineDuration = 2000;
    RunRoutine();
  }
  Serial.print("\nEnd Sequence  \t");
  printTime();
  TurnShowOffOn(0);
}

//^^^^^^^^^^^^ run routines ^^^^^^^^^                 // select the next routine and run it
void RunRoutine() {
  TurnShowOffOn(0);  //turn all strings OFF to begin
  //unsigned long PauseBeforeRoutine = 1000 * random(3, 6);  //delay between routines
  unsigned long PauseBeforeRoutine = 2000;
  Serial.print("\n");
  Serial.print(PauseBeforeRoutine / 1000);
  Serial.print(" sec pause  ");
  printTime();
  delay(PauseBeforeRoutine);
  Serial.print("\nroutine ");
  Serial.print(RoutineNum);

  /*
      case 0: // SPARKLE
      case 1: // SLIDE
      case 2: // ALL ON
      case 3: // WACKAMOLE
      case 4: // SNAKE
      case 5: // SUNRISE SUNSET
      case 6: // WAVE
  */
  if (6 == RoutineNum) {
    //case 6:     // WAVES  repeated increase/decrease brightness of all strings, with decreasing period
    Serial.println(" WAVES");
    float DurationFirstWave = 2500. * random(2, 5);
    int NmbrOfWaves = 15;
    int WaveNmbr = 0;
    int StepsPerWave = 20;
    float WaveFreq;                                         // wave frequency in radian / sec
    float PeriodModifier = 0.7;                             // factor to decrease period of wave
    float WavePeriod = DurationFirstWave / PeriodModifier;  // at each iteration w.b. modified by factor
    int WaveTimeStep = WavePeriod / StepsPerWave;           // deltatime between changes in pwmVal
    TurnShowOffOn(0);
    do {                                         // update 'WavePeriod' for next wave
      WavePeriod = PeriodModifier * WavePeriod;  // milliseconds
      WaveTimeStep = WavePeriod / StepsPerWave;
      WaveFreq = 1.571 / WavePeriod;                            // pi radians in a half wave (2 pi); e.g. pi radians in 10000 ms initially
      WaveTimeStep = WavePeriod / StepsPerWave;                 //divide half period into 10ths; e.g. 100 ms
      for (int j = 0; j <= WavePeriod; j = j + WaveTimeStep) {  // wave time is less than half period                                                     // iterate through steps in pwm update time
        pwmVal = pwmMax * sin(WaveFreq * 2 * j);                //TEST
        TurnShowOffOn(pwmVal);
        delay(WaveTimeStep);
      }
      WaveNmbr++;
      //     ;  //TEST
    } while (WaveNmbr <= NmbrOfWaves);
  }

  //case 0: Sparkle                                                   //SPARKLE
  else if (0 == RoutineNum) {
    int SparkleEndCoast = 5000;                               //$1
    RoutineDuration = 500 * random(8, 17) + SparkleEndCoast;  // extend length of routine (ms)  $1                                //<<******//Uncomment TO TEST
    TimeRoutineWillEnd = millis() + RoutineDuration;
    Serial.print(" SPARKLE for (ms))\t");
    Serial.println(RoutineDuration);
    while (millis() <= TimeRoutineWillEnd) {
      ;  //TEST
      for (int j = 0; j <= NmbrOfStrings - 1; j++) {
        //Sparkle[][] = {{turnsON_time0, ON_duration0 }, ... }
        if ((digitalRead(LOW == TheShow[j][8])) && (millis() > Sparkle[j][0])) {  //check if string 'j' is OFF
          if (millis() > TimeRoutineWillEnd - SparkleEndCoast) {                  // enter if in 'SparkleEndCoast' time
            Sparkle[j][1] = 50;                                                   //duration string 'j' w.b. ON, DURING last 'SparkleEndCoast'; same as interval between
          } else {                                                                // enter if before 'SparkleEndCoast' time
            Sparkle[j][1] = 20 * random(1, 12);                                   // duration string 'j' w.b. ON when in before last 'SparkleEndCoast' time
          }
          digitalWrite(TheShow[j][8], pwmMax);       //turn string 'j' ON
          Sparkle[j][0] = millis() + Sparkle[j][1];  //when string 'j' will turn OFF again
        } else if (digitalRead(HIGH == TheShow[j][8]) && (millis() > Sparkle[j][0])) {
          digitalWrite(TheShow[j][8], 0);            //turn string 'j' OFF
          Sparkle[j][0] = millis() + Sparkle[j][1];  //when string 'j' will turn ON again
        }
      }
    }
  }

  //switch (RoutineNum) {// switch not working??????
  else if (1 == RoutineNum) {  // SLIDE - turn strings sequentially On then OFF, then reverse
    //case 1:
    //NOTE: SLIDE engages all strings; other routines cannot run on them until SLIDE completes
    RoutineDuration = (2 * NmbrOfStrings) * 200 * random(2, 4);
    TimeRoutineWillEnd = millis() + RoutineDuration;
    TheShow[0][3] = 200 * random(1, 4);
    Serial.print(" SLIDE for (ms))\t");
    Serial.println(RoutineDuration);
    while (millis() <= TimeRoutineWillEnd) {
      ;  //TEST
      for (int j = 0; j <= NmbrOfStrings - 1; j++) {
        analogWrite(TheShow[j][8], pwmMax);  // set pwm value ON
        delay(TheShow[0][3]);                // time between increments to pwm
        analogWrite(TheShow[j][8], 0);
      }
      for (int j = NmbrOfStrings - 1; j >= 0; j--) {
        analogWrite(TheShow[j][8], pwmMax);
        delay(TheShow[0][3]);
        analogWrite(TheShow[j][8], 0);
      }
      delay(500);
    }
  }

  else if (2 == RoutineNum) {
    //case 12:                                              // ALL ON - SET ALL STRINGS TO HIGH FOR 'RoutineDuration' ms routine DURATION
    RoutineDuration = 500 * random(10, 40);  //set routine DURATION
    Serial.print(" ALL ON for (ms))\t");
    Serial.println(RoutineDuration);
    TimeRoutineWillEnd = millis() + RoutineDuration;
    TurnShowOffOn(pwmMax);                    //start with all strings ON
    while (millis() <= TimeRoutineWillEnd) {  // wait here for DURATION
      ;                                       //TEST
    }
    TurnShowOffOn(0);  //set all strings LOW after DURATION
  }

  else if (3 == RoutineNum) {  // WACKAMOLE - randomly turn a string ON then OFF
    //case 13:
    RoutineDuration = 1000 * random(5, 9);  // randomly select Show On time
    TimeRoutineWillEnd = millis() + RoutineDuration;
    for (int j = 0; j <= NmbrOfStrings - 1; j++) {
      TheShow[j][3] = 250 * random(1, 4);  // randomly select individual string ON time
    }
    Serial.print(" WACKAMOLE for (ms))\t");
    Serial.println(RoutineDuration);
    while (millis() <= TimeRoutineWillEnd) {  // is same value for all strings
      ;                                       //TEST
      int sString = random(0, NmbrOfStrings);
      analogWrite(TheShow[sString][8], pwmMax);
      delay(TheShow[sString][3]);
      analogWrite(TheShow[sString][8], 0);  // turn OFF before selecting next string
    }
    TurnShowOffOn(0);  //set all strings LOW after DURATION
  }

  else if (4 == RoutineNum) {
    //case 14:             // SNAKE - turn strings ON sequentially, leave ON, then reverse
    int numRepeats = random(4, 7);       //
    TheShow[0][3] = 250 * random(1, 4);  // time between turning sequential strings ON
    //RoutineDuration = random(1, 4) * NmbrOfStrings * TheShow[0][3];   // duration of routine
    RoutineDuration = numRepeats * NmbrOfStrings * TheShow[0][3];
    Serial.print(" routine SNAKE for (ms))\t");
    Serial.println(RoutineDuration);
    TimeRoutineWillEnd = millis() + RoutineDuration;
    while (millis() <= TimeRoutineWillEnd) {
      ;                                               //TEST
      for (int j = 0; j <= NmbrOfStrings - 1; j++) {  // snake up
        analogWrite(TheShow[j][8], pwmMax);
        delay(TheShow[0][3]);
      }
      for (int j = NmbrOfStrings - 1; j >= 0; j--) {  // snake down
        analogWrite(TheShow[j][8], 0);
        delay(TheShow[0][3]);
      }
      delay(1000);  //pause OFF
    }
  }

  else if (5 == RoutineNum) {  //SUNRISE SUNSET increase/decrease brightness of strings sequentially with delay
    RoutineDuration = 60000;
    Serial.print(" SUNRISE SUNSET for (ms))\t");
    Serial.println(RoutineDuration);
    //unsigned long RoutineStartTime = millis();
    //unsigned long RoutineEndTime = RoutineStartTime + RoutineDuration;
    TurnShowOffOn(0);  //turn off all strings
    int RampDirn[NmbrOfStrings + 1] = { 1, 1, 1, 1, 1, 1 };
    // Ramp[xx][yy]
    //{ (0)pwm step,                (1)pwm max,                 (2)number steps in ramp,
    //  (3)delta step time,         (4)step number;             (5)duration in ramp one direction,
    //  (6)ramp start delay,        (7)time after one dir'n,    (8)time of step
    //  (9)pwm value after step     (10)ramp dirn}
    unsigned long Ramp[NmbrOfStrings][12] = { { 10, 255, 0, 0, 0, 4000, 0, 0, 0, 0, 1 }, { 10, 255, 0, 0, 0, 4000, 4000, 0, 0, 0, 1 }, { 10, 255, 0, 0, 0, 4000, 8000, 0, 0, 0, 1 }, { 10, 255, 0, 0, 0, 4000, 12000, 0, 0, 0, 1 }, { 10, 255, 0, 0, 0, 4000, 16000, 0, 0, 0, 1 }, { 10, 255, 0, 0, 0, 4000, 20000, 0, 0, 0, 1 } };

    for (int j = 0; j <= NmbrOfStrings - 1; j++) {  // loop all strings to set brightnessand timing
      Ramp[j][2] = Ramp[j][1] / Ramp[j][0];         //number of steps
      Ramp[j][3] = Ramp[j][5] / Ramp[j][2];         //delta step time
    }
    TimeRoutineStarted = millis();
    while (millis() <= TimeRoutineStarted + RoutineDuration) {
      ;              //TEST
      int jmin = 0;  // CHANGE TO TEST
      int jmax = 5;  // CHANGE TO TEST
      for (int j = jmin; j <= jmax; j++) {
        unsigned long TimeNow = millis();
        if (TimeNow >= TimeRoutineStarted + Ramp[j][6]) {        //check if past start delay//allow for string 'j' start delay
          if (millis() >= Ramp[j][8] + Ramp[j][3]) {             //calc time in ramp; check if ready to change brightness
            Ramp[j][9] = Ramp[j][9] + RampDirn[j] * Ramp[j][0];  //pwm value after step//change pwm (brightness) by pwm (brighness) step
            if (Ramp[j][9] > Ramp[j][1]) {                       //reverse ramp dirn to downward when pwm exceed max
              Ramp[j][9] = Ramp[j][1];
              RampDirn[j] = -1;  // downward ramp
            }
            if (Ramp[j][9] < 10) {  //reverse ramp dirn to upward when pwm goes neg
              Ramp[j][9] = 0;
              RampDirn[j] = 1;  //upward ramp
            }
            analogWrite(TheShow[j][8], Ramp[j][9]);
            Ramp[j][8] = TimeNow;  // time of step
          }
        }
      }
    }
  }
}

//^^^^^^^^^^^^ randomize run order of routines ^^^^^^^^^
void RandomizeRoutines() {
  for (int i = 0; i < NmbrOfRoutines; i++) {  // store randomized routine numbers in array 'RoutineSelect'
    int pos = random(NmbrOfRoutines);
    int t = RoutineSelect[i];
    RoutineSelect[i] = RoutineSelect[pos];
    RoutineSelect[pos] = t;
  }
  Serial.println("\n\n\nNext Sequence\nOrder of Routines:");
  for (int i = 0; i < NmbrOfRoutines; i++) {
    Serial.print(" ");
    Serial.print(RoutineSelect[i]);
  }
  Serial.println();
}

//^^^^^^^^^^^^ set all strings ON or OFF ^^^^^^^^^
void TurnShowOffOn(int pwmMax) {
  for (int j = 0; j <= NmbrOfStrings - 1; j++) {  //set all strings 'pwmMax'
    analogWrite(TheShow[j][8], pwmMax);
  }
}

//^^^^^^^^^^^^ writing to RTC ^^^^^^^^^
void setDS3231time(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year) {
  // sets time and date data to DS3231  $5
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0);                     // set next input to start at the seconds register
  Wire.write(decToBcd(second));      // set seconds
  Wire.write(decToBcd(minute));      // set minutes
  Wire.write(decToBcd(hour));        // set hours
  Wire.write(decToBcd(dayOfWeek));   // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(dayOfMonth));  // set date (1 to 31)
  Wire.write(decToBcd(month));       // set month
  Wire.write(decToBcd(year));        // set year (0 to 99)
  Wire.endTransmission();
}

//^^^^^^^^^^^^ reading from RTC ^^^^^^^^^^^^
void readDS3231time(byte* second,
                    byte* minute,
                    byte* hour,
                    byte* dayOfWeek,
                    byte* dayOfMonth,
                    byte* month,
                    byte* year) {  // addresses are passed to 'readDS3231time(address for 'second', for 'minute', ... $5
  // addresses are rec'd by 'readDS3231time(... )'
  // '*second' gets the value at the address sent to 'readDS3231time(... )' and pointed to by 'second' etc.
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0);  // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  *second = bcdToDec(Wire.read() & 0x7f);  // 0x7f--> 1111111 //get bcd from RTC then convert to decimal
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);  // 0x3f-->  111111
  *dayOfWeek = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month = bcdToDec(Wire.read());
  *year = bcdToDec(Wire.read());
}

//^^^^^^^^^^^^ check for RTC window^^^^^^^^^^^^
void Check_RTC_ON_OFF() {
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
  Serial.println("\nCheck RTC");  //XXX
  printTime();
  Serial.println((float)turnONtimes[0][2] / 60.);
  for (int i = 0; i < 4; i++) {  //examine turnONtimes array elements
    if (((float)hour + minute / 60.) >= ((float)turnONtimes[i][0] + (float)turnONtimes[i][1] / 60.)) {
      Serial.print(i);
      Serial.print("  inside RTC window\t");  //XXX<----------------
      Serial.print(turnONtimes[i][0]);        //hour
      Serial.print("\t");
      Serial.print(turnONtimes[i][1]);  //minute
      Serial.print("\t");
      Serial.print(turnONtimes[i][2]);  //time duration on
      Serial.print("\t");
      Serial.print(((float)turnONtimes[i][0] + (float)turnONtimes[i][1] / 60. + (float)turnONtimes[i][2] / 60.), 1);  //time off
      Serial.print("\t");
      Serial.println((float)hour + minute / 60., 1);  //time now
      if (((float)hour + minute / 60.) < ((float)turnONtimes[i][0] + (float)turnONtimes[i][1] / 60. + (float)turnONtimes[i][2] / 60.)) {
        Serial.print(i);
        Serial.println("  inside RTC window");  //XXX<----------------
        RTC_IsOn_Is = true;
        break;  // found a valid ON time interval
      } else {
        /*
        if outside RTC window and already printed, don't print again
        */
        Serial.print(i);
        Serial.println("  past RTC window");  //XXX<----------------
        RTC_IsOn_Is = false;
      }
    } else {
      RTC_IsOn_Is = false;
      Serial.print(i);
      Serial.println("  before RTC window");  //XXX<----------------
    }
  }
  Serial.print("RTC is ");
  Serial.println(RTC_IsOn_Is);
}

//^^^^^^^^^^^^ Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val) {
  return ((val / 16 * 10) + (val % 16));
}

//^^^^^^^^^^^^ Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val) {
  return ((val / 10 * 16) + (val % 10));
}

//^^^^^^^^^^^^ output to serial monitor ^^^^^^^^^^^^
void printTime() {
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  // retrieve data from DS3231
  // send the address '&second' for 'second' to fcn 'readDS3231time(&second... )
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
  // send it to the serial monitor
  Serial.print(hour, DEC);
  // convert the byte variable to a decimal number when displayed
  Serial.print(":");
  if (minute < 10) {
    Serial.print("0");
  }
  Serial.print(minute, DEC);
  Serial.print(":");
  if (second < 10) {
    Serial.print("0");
  }
  Serial.print(second, DEC);
  /*
  Serial.print(" ");
  Serial.print(dayOfMonth, DEC);
  Serial.print("/");
  Serial.print(month, DEC);
  Serial.print("/");
  Serial.print(year, DEC);
  Serial.print(" Day of week: ");
  Serial.print(dayOfWeek);
*/
}

// https://www.guru99.com/c-function-pointers.html
// https://www.guru99.com/c-pointers.html
// https://byjus.com/maths/octal-number-system/