//-------Used Structs----------
 
typedef struct {
   int lightValuesMeasured [40];
   int indexToInsertNext;
   int arraySize;
   double average;
 }LightValues;
 //used for calculation of average lightvalue
LightValues lv = {
  {0,0,0,0,0,0,0,0,0,0,
   0,0,0,0,0,0,0,0,0,0,
   0,0,0,0,0,0,0,0,0,0, 
   0,0,0,0,0,0,0,0,0,0},
  0,
  40,
  0.0
};
//-------global variables-------
boolean toilettOccupied = false;
//the previous timer valuer from millis().
unsigned long prevTimeSinceProgStartForTwoHourDetection = 0;
unsigned long prevTimeSinceProgStartForTwentySecondDetection = 20000;


/*
 * light value. this value was measured with smartphone light sensor. This should be equal to around 2000lx.
 * the higher the analog output of the sensor is, the darker it is.
 */
unsigned const int MINIMUM_LIGHT_NEEDED = 22;

/* 
 *  the cutOf of the readed lightvalues. should be always be a divisor of MINIMUM_LIGHT_NEEDED.
 *  if sensor reads a light value of: 1,2,3,4,5,6,7,9,10,11, the value measured will be read as 11 if the CUT_OFF is 11.
 *  the values 12,13,14,15,16, .. 22 will be read as 22 and 22,23,24,25, ..., 33 as 33 and so on.
 */
unsigned const int CUT_OFF = 11;

// 2 hours = 7200000ms.
unsigned const long TIMER_SEND_USAGECOUNT = 7200000;
unsigned const long TIMER_TILL_NEXT_USAGE_DETECTION = 20000;

//counter of detected toilett guests
unsigned int usageCounter = 0;

//===output pins===
unsigned const short LED_OUTPUT_PIN = 13;
//Analog pin A3
unsigned const short RELAIS_OUTPUT_PIN = 3;
//===input pins ===
//Digital Pin D7
const short INPUT_PIN_REED_SWITCH = 7;
//Analog Pin A1
const short INPUT_PIN_LIGHT_SENSOR = 1;

// ============>>>put your setup code here, to run once<<<============
void setup() {
  //setup console output
  Serial.begin(9600);
  pinMode(LED_OUTPUT_PIN, OUTPUT);
  pinMode(INPUT_PIN_REED_SWITCH, INPUT);
  pinMode(INPUT_PIN_LIGHT_SENSOR, INPUT_PULLUP);
  pinMode(RELAIS_OUTPUT_PIN, OUTPUT);
//fill the struct with lightvalues for initialisation.
  for(int i = 0; i < lv.arraySize; i++){
    lightValuesStructHandler();
  }
}//setup()

// ============>>>put your main code here, to run repeatedly<<<============
void loop() {
  lightValuesStructHandler();
  if (enoughLightForSolarPowering()) {
    switchLEDOn();
  } else {
    switchLEDOff();
  }
  if (checkIfTwoHoursHavePassedAndResetTimer() == true) {
    transmitDataAndResetUsageCounter();
  }
  checkIfToilettIsOccupiedAndHandleThatEvent();
  checkIfToilettWasFreedAgain();
  delay(500);
}//loop()



//-------/helper functions\-------
void checkIfToilettIsOccupiedAndHandleThatEvent() {
  if (digitalRead(INPUT_PIN_REED_SWITCH) == LOW 
        && toilettOccupied == false 
        && (millis() - prevTimeSinceProgStartForTwentySecondDetection >= TIMER_TILL_NEXT_USAGE_DETECTION) 
      ) {
      prevTimeSinceProgStartForTwentySecondDetection = millis();
      usageCounter++;
      Serial.print("New usage! Counter is now:");
      Serial.println(usageCounter);
      toilettOccupied = true;
  }
}//checkIfToilettIsOccupiedAndHandleThatEvent()

void checkIfToilettWasFreedAgain() {
  if (digitalRead(INPUT_PIN_REED_SWITCH) == HIGH && toilettOccupied == true ) {
    toilettOccupied = false;
  }

}//checkIfToilettWasFreedAgain()
void letOnboardLEDBlink(int delayInMS) {
  letOnboardLEDBlink(delayInMS, 1);
}//letOnboardLEDBlink()

boolean checkIfTwoHoursHavePassedAndResetTimer() {
  if ((unsigned long) (millis() - prevTimeSinceProgStartForTwoHourDetection ) >= TIMER_SEND_USAGECOUNT ) {
    Serial.println("2 Hours are over! ");
    prevTimeSinceProgStartForTwoHourDetection = millis();
    return true;
  }
  return false;
}//checkIfTwoHoursHavePassedAndResetTimer

void transmitDataAndResetUsageCounter() {
  Serial.println("Transmitting Data and resetting the counter of persons used the toilett.");
  //sendToiletteUsageCountMessage(usageCounter);
  usageCounter = 0;
}//transmitDataAndResetUsageCounter()

void switchLEDOn() {
  digitalWrite(LED_OUTPUT_PIN, HIGH);
}//switchLEDOn()

void switchLEDOff() {
  digitalWrite(LED_OUTPUT_PIN, LOW);
}//switchLEDOff

void letOnboardLEDBlink( int delayInMS,  int blinkamount) {
  if (delayInMS < 100) {
    delayInMS = 100;
  }
  if (blinkamount <= 0) {
    blinkamount = 1;
  }
  for (int i = 0; i < blinkamount; i++ ) {
    digitalWrite(LED_OUTPUT_PIN, HIGH);
    delay(delayInMS);
    digitalWrite(LED_OUTPUT_PIN, LOW);
    delay(delayInMS);
  }
}//letOnboardLEDBlink()

bool enoughLightForSolarPowering() {
  if (lv.average <= MINIMUM_LIGHT_NEEDED) {
    return true;
  } else {
    return false;
  }

}//enoughLightForSolarPowering()
/*
* reeds a new light value and updates Data in lv from type struct LightValues.
* this is the only function, which modifies the entries of the struct.
*/
void lightValuesStructHandler(){
   unsigned long average = 0;
   unsigned int lightval = analogRead(INPUT_PIN_LIGHT_SENSOR);
     //Serial.print("lightval: ");
     //Serial.println(lightval);
   lightval = calcCutOff(lightval, CUT_OFF);
     //Serial.print("cutOffedLightVal: ");
     //Serial.println(lightval);
   if(lv.indexToInsertNext >= lv.arraySize){
      lv.indexToInsertNext = 0;
   }
   lv.lightValuesMeasured[lv.indexToInsertNext] = lightval;
   lv.indexToInsertNext++;
   for(int i = 0; i < lv.arraySize; i++){
    average = average + lv.lightValuesMeasured[i];
   }
   lv.average = (double)average / lv.arraySize;
   //Serial.println(lv.average);
   //Serial.println("= = = = = = = = = = = = = = =");
}//lightValuesStructHandler()

//Cut off value to be set. If the Cut Off is eg.: 25, values like 1,2,3,4,5,6, ..., 22,23,24,25 are set to be 25. 26 will then therefore be 50.
//used for light measurement. All measured values from the light sensor lesser then the minimum light needed, will be set to one value.
int calcCutOff(int lightvalueFromSensor, int cutOff) {
    int thresholdedValue = 0;
    if( (lightvalueFromSensor % cutOff) > 0){
      thresholdedValue = lightvalueFromSensor + (cutOff - (lightvalueFromSensor % cutOff));
    }else{
      thresholdedValue = lightvalueFromSensor;
    }
    return thresholdedValue;
}//calcCutOff() 



/**
 * Idee: lass beim setup() 5 lichtwerte aufnehmen. einen Wert pro sekunde.
 * Dannach: Berechne durchschnitt aus den array elementen. Dieser wird gethresholded. 
 * Merke dir, wo du zuletzt eingefügt hast.
 * füge eins weiter ein und merke dir diese position, 
 * array ende: starte bei 0.
 * berechne nach dem einfügen eines neuen werts den neuen durchschnitt.
 * gebe diesen thresholded zurück
 * 
 * 
 **/

void switchSolarOn(){
    
}
