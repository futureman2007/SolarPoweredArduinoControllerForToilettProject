//-------Used Structs----------
//used for calculation of average lightvalue
typedef struct {
  unsigned static const int ARR_SIZE = 40;
  unsigned long lightValuesMeasured [ARR_SIZE];
  unsigned int indexToInsertNext;
  double average;
 }LightValues;
 //used for calculation of average lightvalue
LightValues lv = {
  {0,0,0,0,0,0,0,0,0,0,
   0,0,0,0,0,0,0,0,0,0,
   0,0,0,0,0,0,0,0,0,0, 
   0,0,0,0,0,0,0,0,0,0},
  0,
  0.0
};


//-------global variables-------
boolean toilettOccupied = false;
//previous timer value from millis().
unsigned long prevTimeSinceProgStartForTwoHourDetection = 0;
unsigned long prevTimeSinceProgStartForTwentySecondDetection = 20000;

/*
 * light value. this value was measured with smartphone light sensor. This should be equal to around 2000lx.
 * the higher the analog output of the sensor is, the darker it is.
 * NOTE: A relais is used, to disable or enable the powersupply from the solar panel.
 * If the railis switches on, the measured lightvalue becomes automaticly darker eg:
 * lightamount read with relais off: 20. Lightamount read with relais on: 26.
 * The higher this number, the darker it is.
 */
unsigned static const int MINIMUM_LIGHT_NEEDED = 21;
/*
 * As mentioned befor, switching on the railis causes a darker light value. 
 * If the relais is on, lightValuesStructHandler() will correct the value by LIGHT_VAL_OFFSET (6).
 * 6 was measured during tests. when the relais switched on, the lightValue was higher by 6.
 * if other sensors will be connected to the arduino in future, 6 could be not enough.
 * the more power is drawn, the more the lightValue will be offsetted.
 */
unsigned static const short LIGHT_VAL_OFFSET = 7; 
boolean lightvalCorrectionNeeded = false;
/* 
 *  the cutOf of the readed lightvalues. should be always be a divisor of MINIMUM_LIGHT_NEEDED.
 *  if sensor reads a light value of: 1,2,3,4,5,6,7,9,10,11, the value measured will be read as 11 if the CUT_OFF is 11.
 *  the values 12,13,14,15,16, .. 22 will be read as 22 and 22,23,24,25, ..., 33 as 33 and so on.
 */
unsigned static const int CUT_OFF = 7;

// 2 hours = 7200000ms. After that time, theThingsUno transmitts, how many people used the toilet.
unsigned static const long TIMER_SEND_USAGECOUNT = 7200000;
//20sec. = 20000ms. The time which has to elapse beforea a new usage of the toilett can be registered.
unsigned static const long TIMER_TILL_NEXT_USAGE_DETECTION = 20000;

//counter of detected toilett guests
unsigned int usageCounter = 0;

//===output pins===
//onboard led
unsigned static const short LED_OUTPUT_PIN = 13;
//Digital pin D3
unsigned static const short RELAIS_OUTPUT_PIN = 3;
//===input pins ===
//Digital Pin D7
unsigned static const short INPUT_PIN_REED_SWITCH = 7;
//Analog Pin A1
unsigned static const short INPUT_PIN_LIGHT_SENSOR = 1;

// ============>>>put your setup code here, to run once<<<============
void setup() {
  //setup console output
  Serial.begin(9600);
  pinMode(LED_OUTPUT_PIN, OUTPUT);
  pinMode(RELAIS_OUTPUT_PIN, OUTPUT);
  pinMode(INPUT_PIN_REED_SWITCH, INPUT);
  pinMode(INPUT_PIN_LIGHT_SENSOR, INPUT_PULLUP);
//fill the struct with lightvalues = 500 for initialisation.
  lightValuesStructInitializer();

}//setup()

// ============>>>put your main code here, to run repeatedly<<<============
void loop() {
  lightValuesStructHandler();
  if (enoughLightForSolarPowering()) {
    switchSolarOn();
  } else {
    switchSolarOff();
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
      ) {
      prevTimeSinceProgStartForTwentySecondDetection = millis();
      usageCounter++;
      Serial.println("--------====> TOILETTE USAGE <====--------");
      Serial.println("New usage! Counter is now:");
      Serial.println("--------====> TOILETTE USAGE END<====--------");
      Serial.println(usageCounter);
      toilettOccupied = true;
      //remove letOnboardLEDBlink
      letOnboardLEDBlink(200,3);

  }
}//checkIfToilettIsOccupiedAndHandleThatEvent()

void checkIfToilettWasFreedAgain() {
  if (digitalRead(INPUT_PIN_REED_SWITCH) == HIGH 
      && toilettOccupied == true 
      && (millis() - prevTimeSinceProgStartForTwentySecondDetection >= TIMER_TILL_NEXT_USAGE_DETECTION)
     ){
    toilettOccupied = false;
  }

}//checkIfToilettWasFreedAgain()
void letOnboardLEDBlink(int delayInMS) {
  letOnboardLEDBlink(delayInMS, 1);
}//letOnboardLEDBlink()

boolean checkIfTwoHoursHavePassedAndResetTimer() {
  if ((unsigned long) (millis() - prevTimeSinceProgStartForTwoHourDetection ) >= TIMER_SEND_USAGECOUNT ) {
    Serial.println("--------====> TWO HOURS TIMER FOR DATA TRANSMISSION <====--------");
    Serial.println("2 Hours are over! ");
    Serial.println("--------====> TWO HOURS TIMER FOR DATA TRANSMISSION END <====--------");
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
    Serial.println("==========> THERE IS ENOUGH LIGHT! <==========");
    return true;
  } else {
    Serial.println("==========> THERE IS NOT ENOUGH LIGHT! <==========");
    return false;
  }

}//enoughLightForSolarPowering()
/*
* reeds a new light value and updates Data in lv from type struct LightValues.
* this is the only function, which modifies the entries of the struct.
*/

void lightValuesStructInitializer(){
  
   for(int i = 0; i < lv.ARR_SIZE; i++){
    lv.lightValuesMeasured[i] = 500;
   }
   lv.average = 500,
   lv.indexToInsertNext = 0;
}

void lightValuesStructHandler(){
   unsigned long average = 0;
   unsigned int lightval = analogRead(INPUT_PIN_LIGHT_SENSOR) -1;
    if(lightvalCorrectionNeeded){
      Serial.println("===!!!!!!!===LIGHTVALUE WILL BE CORRECTED.===!!!!!!!===");
      Serial.println(LIGHT_VAL_OFFSET);
      Serial.println("===!!!!!!!===    ===!!!!!!!===");
      lightval = lightval - LIGHT_VAL_OFFSET;
    }
   
     Serial.println("--------====>    LIGHTVALUES   <====--------");
     Serial.print("lightval: ");
     Serial.println(lightval);
   lightval = calcCutOff(lightval, CUT_OFF);
     Serial.print("cutOffedLightVal: ");
     Serial.println(lightval);
   if(lv.indexToInsertNext >= lv.ARR_SIZE){
      lv.indexToInsertNext = 0;
   }
   lv.lightValuesMeasured[lv.indexToInsertNext] = lightval;
   lv.indexToInsertNext++;
   for(int i = 0; i < lv.ARR_SIZE; i++){
    average = average + lv.lightValuesMeasured[i];
   }
   lv.average = (double)average / lv.ARR_SIZE;
   Serial.print("Calculated Average: ");
   Serial.println(lv.average);
   Serial.println("--------====>  LIGHTVALUES_END  <====--------");
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

void switchSolarOn(){
  lightvalCorrectionNeeded = true;
  digitalWrite(RELAIS_OUTPUT_PIN, LOW);
}

void switchSolarOff(){
  lightvalCorrectionNeeded = false;
  digitalWrite(RELAIS_OUTPUT_PIN, HIGH);
}
