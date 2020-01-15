//-------global variables-------
boolean toilettOccupied = false;

//the previous timer valuer from millis().
unsigned long prevTimeSinceProgStart = 0;


//light value. this value was measured with smartphone light sensor. This should be equal to around 2000lx.
//the higher the analog output of the sensor is, the darker it is.
unsigned const int MINIMUM_LIGHT_NEEDED = 20;

//**this threshold should only use the values 1,10 and 100.
//if for example a threshold of 10 is used and the light value is set to 490, values like 490,491,492, ... ,499 are accepted as 490.**/
unsigned const short LIGHT_VALUE_THRESHOLD = 10;


// 2 hours in millis.
unsigned const long TWO_HOUR_TIMER_IN_MS = 7200000;

unsigned const long DELAY_TILL_NEXT_USAGE_DETECTION = 20000;

//counter of detected toilett guests
unsigned int usageCounter = 0;

//===output pins===
unsigned const int LED_OUTPUT_PIN = 13;

//===input pins ===
//Digital Pin D7
const short INPUT_PIN_REED_SWITCH = 7;
//Analog Pin A1
const short INPUT_PIN_LIGHT_SENSOR = 1;

// ============>>>put your setup code here, to run once<<<============
void setup() {
  //setup console output
  Serial.begin(9600);
  //setup LED Pin port as output.
  pinMode(LED_OUTPUT_PIN, OUTPUT);
  //setup the Digital pin port as input.
  pinMode(INPUT_PIN_REED_SWITCH, INPUT);
  pinMode(INPUT_PIN_LIGHT_SENSOR, INPUT_PULLUP);

}//setup()

// ============>>>put your main code here, to run repeatedly<<<============
void loop() {
  if (enoughLightForSolarPowering(LIGHT_VALUE_THRESHOLD)) {
    switchLEDOn();
  } else {
    switchLEDOff();
  }

  if (checkIfTwoHoursHavePassed() == true) {
    transmitDataAndResettCounter();
  }
  checkIfTwoHoursHavePassed();
  checkIfToilettIsOccupiedAndHandleThatEvent();
  checkIfToilettWasFreedAgain();
  delay(400);
}//loop()



//-------/helper functions\-------
void checkIfToilettIsOccupiedAndHandleThatEvent() {
  if (digitalRead(INPUT_PIN_REED_SWITCH) == LOW && toilettOccupied == false ) {
    handleToilettOccupied();
    toilettOccupied = true;
  }
}//checkIfToilettIsOccupiedAndHandleThatEvent()

void checkIfToilettWasFreedAgain() {
  if (digitalRead(INPUT_PIN_REED_SWITCH) == HIGH && toilettOccupied == true ) {
    toilettOccupied = false;
  }

}//checkIfToilettWasFreedAgain()

void handleToilettOccupied() {
  usageCounter++;
  Serial.print("New usage! Counter is now:");
  Serial.println(usageCounter);
  //letOnboardLEDBlink(250,3);
  delay(DELAY_TILL_NEXT_USAGE_DETECTION);
}//handleToilettOccupied()
void letOnboardLEDBlink(int delayInMS) {
  letOnboardLEDBlink(delayInMS, 1);
}//letOnboardLEDBlink()

boolean checkIfTwoHoursHavePassed() {
  if ((unsigned long) (millis() - prevTimeSinceProgStart ) >= TWO_HOUR_TIMER_IN_MS ) {
    Serial.println("2 Hours are over! ");
    return true;
  }
  return false;
}//checkIfTwoHoursHavePassed

void transmitDataAndResettCounter() {
  Serial.println("Transmitting Data and resetting the counter of persons used the toilett.");
  //sendToiletteUsageCountMessage(usageCounter);
  usageCounter = 0;
  prevTimeSinceProgStart = millis();
}//transmitDataAndResettCounter()

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

bool enoughLightForSolarPowering(int threshold) {
  int lightval = analogRead(INPUT_PIN_LIGHT_SENSOR);
  int threholdedValue = calcThreholdedValue(lightval, threshold);;
  if (threshold <= 9 ) {
    threshold = 1;
  }

  if (threshold >= 10  && threshold <= 99) {
    threshold = 10;
  }

  if (threshold >= 100) {
    threshold = 100;
  }

  //test

  Serial.print("Light value is currently: ");
  Serial.println(lightval);

  Serial.print("Light value thresholded is currently: ");
  Serial.println(threholdedValue);
  //\test
  if (threholdedValue <= MINIMUM_LIGHT_NEEDED) {
    return true;
  } else {
    return false;
  }

}//enoughLightForSolarPowering()
//asuming that threshold is 1, 10, 100 and so on. 
//Value of light will be always rounded up. Eg: 20 => 20; 21=>30; 22=>30 ... 29=>30, 30=>30, 31 =>40 and so on.
int calcThreholdedValue(int lightvalueFromSensor, int threshold) {
    int thresholdedValue = 0;
    if( (lightvalueFromSensor % threshold) > 0){
      thresholdedValue = lightvalueFromSensor + (threshold - (lightvalueFromSensor % threshold));
    }else{
      thresholdedValue = lightvalueFromSensor;
    }
    return thresholdedValue;
}//calcThreholdedValue() 
