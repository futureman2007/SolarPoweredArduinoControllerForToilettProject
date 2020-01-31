#include <TheThingsNetwork.h>
/**@file sketch_dec23a.ino */
//////////////////////////////////////
//----LoRa communikation config------
//////////////////////////////////////

#define freqPlan TTN_FP_EU868
#define loraSerial Serial1
#define debugSerial Serial
TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);
//////////////////////////////////////
//-------verwendete Structs----------
//////////////////////////////////////

/**
   Die LightValues Struct wird verwendet, um von den gelesenen Lichtwerten des Lichtsensors einen Durschnittswert zu bilden.
   Die Struct wird von der Funktion readLightValuesAndCalculateCutOffedAverage() verwaltet. Es wird empfohlen nur lesend auf alle
   Datentypen der Strukt zuzugreifen.
   Weitere Informationen, wie die Strukt verwaltet wird, können in readLightValuesAndCalculateCutOffedAverage() nachgelesen werden.
*/
typedef struct {
  //! Größe des Arrays.
  unsigned static const int ARR_SIZE = 40;
  //! Lcihtwerte die vom Lichtsensor (Photoresistor) gemessen wurden.
  unsigned long lightValuesMeasured [ARR_SIZE];
  //! Der nächste Index, bei dem in lightValuesMeasured ein neuer Wert eingefügt werden kann.
  unsigned int indexToInsertNext;
  //! Durschnittlich ermittelte Lichtwert.
  double average;
} LightValues;

/**
    Initialisierung des LightValues lv Objektes. Auf lv wird lediglich lesend zugegriffen,
    mit Ausnahme von readLightValuesAndCalculateCutOffedAverage().
    readLightValuesAndCalculateCutOffedAverage() verwaltet lv. Mehr Informationen dazu können in readLightValuesAndCalculateCutOffedAverage()
    nachgelesen werden.
*/

LightValues lv = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0
  },
  0,
  0.0
};
  /**
   Doe ToiletController ist für das zählen der Benutzungen zuständig.
   Zudem hält die Strukt fest, ob die Toilette Besetzt ist oder nicht.
   Die Variable ToiletController.toilettOccupied gibt an, ob die toilette besetzt ist oder nicht.
   Dabei steht "true" für besetzt und "false" für nicht besetzt.
   Bemerkt der Türsensor, dass die Tür abgeschlossen wurde, wird dieses Event durch
   checkIfToilettIsOccupiedAndHandleThatEvent() bearbeitet.
   Dabei wird der Wert der Variable auf "true" gesetzt.
   Es wird davon ausgegangen, dass ein Toilettengast mindestens 20 Sekunden benötigt
   Erst nach Ablauf der 20sek. kann ein neuer Gast erfasst werden.
   Für weitere Inforamtionen, wie genau die Benutzung der Toilette erfasst wird, siehe Dokumentation für checkIfToilettWasFreedAgain() und
   checkIfToilettIsOccupiedAndHandleThatEvent().
*/
typedef struct {
  
  //! Besetztstatus der Toilette
  boolean toilettOccupied = false;
  //! Zähler für die Anzahl der registrierten Toilettengäste
  unsigned int usageCounter = 0;

} ToiletController;

ToiletController tc;


//////////////////////////////////////
//-------verwendete Konstanten--------
//////////////////////////////////////

//! Die AppEui der TheThingsNetwork App, zudem die gesendeten Daten der Toilette gehören.
const char *appEui = "70B3D57ED0027C59";
//! Verwendeter Schlüssel für die Übtertragung. Hängt mit TheThingsNetowrk App zusamen.
const char *appKey = "5813DF9589E3ED9136413C6436123BBC";

  /**
   Dieser Wert gibt an, ab welchem durschnittlichen Lichtwert des Lichtsensors
   (Photoresistor) die Solaranlage genug Strom für die Versorgung des Arduinos durh
   den PassTbrough Mechanismus der PowerBank liefert.
   Mehr Informationen dazu können in enoughLightForSolarPowering() nachgelesen werden.
*/
unsigned static const int MINIMUM_LIGHT_NEEDED = 18;

/**
   Um die Stromzufuhr der Solaranlage zu aktivieren, wird ein Relais verwendet.
   Durch die Aktivierung des Relais steigt der Stromanspruch des Arduinos spürbar an.
   Ein daraus resultierender Seiteneffekt ist, dass Lichtwerte des Lichtsensors (Photoresistor)
   Dunkler erfasst werden. Diese Variable soll entsprechende korrekturen beim einschalten des Relais vornehmen.
   Diese Korrektur wird dann in readLightValuesAndCalculateCutOffedAverage() vorgenommen.
*/
unsigned static const short LIGHT_VAL_OFFSET = 7;

/*
    Der CutOff Wert für gemessene Lichtwerte. Dieser sollte immer ein Teiler von MINIMUM_LIGHT_NEEDED sein.
    Wenn bspw. MINIMUM_LIGHT_NEEDED = 18 gesetzt wird, sollte CUT_OFF auf 2,3 oder 9 gesetzt werden.
    Mehr informationen zur CUT_OFF Funktion kann unter calcCutOff() nachgelesen werden.
*/
unsigned static const int CUT_OFF = 9;

/**
  Zwei Stunden = 7200000ms. Nach dieser Zeit, übermittelt das TheThingsUno die Anzahl
  der Toilettenbenutzungen mittels des eingebauten LoRa Typ A Senders.
  Das Senden wird in transmitDataAndResetUsageCounter() durchgeführt.
*/
unsigned static const long TIMER_SEND_USAGECOUNT = 7200000;
/**
  20sec. = 20000ms.
  Erst nach dieser Zeit wird eine neue Toilettenbenutzung registriert.
  Mehr zu dieser Funktion kann in checkIfToilettWasFreedAgain() nachgelesen werden.
*/
unsigned static const long TIMER_TILL_NEXT_USAGE_DETECTION = 20000;



//////////////////////////////////////
//------------output pins------------
//////////////////////////////////////

//!Wert der die Onboard LED repräsentiert.
unsigned static const short LED_OUTPUT_PIN = 13;

//!Wert der den digitalen Pin D3 repräsentiert.
unsigned static const short RELAIS_OUTPUT_PIN = 3;

//////////////////////////////////////
//------------input pins------------
//////////////////////////////////////

//!Wert des digitalen Pin D7.
unsigned static const short INPUT_PIN_REED_SWITCH = 7;

//!Wert des analogen Pins A1.
unsigned static const short INPUT_PIN_LIGHT_SENSOR = 1;

//////////////////////////////////////
//---------globale Variablen---------
//////////////////////////////////////

/**
   Der Arduino zählt die millisekunden, seitdem das Programm gestartet wurde.
   Diese Variable wird verwendet, um festzustellen, ob zwei Stunden seit Programmstart vergangen sind.
   Ist dies der Fall, wird der Wert der Variable auf den aktuellen Zählerstand von millis() gesetzt.
   Weitere Informationen zur Reaktion darauf, kann in der Dokumentation von checkIfTwoHoursHavePassedAndResetTimer()
   und transmitDataAndResetUsageCounter()nachgelesen werden.
*/
unsigned long prevTimeSinceProgStartForSendingData = 0;

/**
   Ähnlich zu prevTimeSinceProgStartForSendingData.
   Hier wird allerdings die Zeit von 20sek. festgehalten.
   Mehr dazu in der Dokumentation von checkIfToilettWasFreedAgain().
*/
unsigned long prevTimeSinceProgStartForTwentySecondDetection = 20000;

//! Gibt an, ob Korrekturen beim erfassen der Lichtwerte vorgenommen werden müssen. Bspw. nach dem einschalten des Relais in switchSolarOn().
boolean lightvalCorrectionNeeded = false;



//!Setup Code. Wird nur einmal ausgeführt um entsprechende Pins zu Konfigurieren sowie die LightValues lv struct zu initialisieren.
void setup() {
  //Konfigurieren des Seriellen Outputs. Damit können Konsolenausgaben in der Entwicklungsumgebung vom Arduino empfangen werden.
  Serial.begin(9600);
  pinMode(LED_OUTPUT_PIN, OUTPUT);
  pinMode(RELAIS_OUTPUT_PIN, OUTPUT);
  pinMode(INPUT_PIN_REED_SWITCH, INPUT);
  pinMode(INPUT_PIN_LIGHT_SENSOR, INPUT_PULLUP);
  switchSolarOff();
  lightValuesStructInitializer();
//Konfiguration, um mit einem LoRa Gateway in der nähe kommunizieren zu können. Over the air activation wird verwendet.
  loraSerial.begin(57600);
  debugSerial.begin(9600);
  
  // Wait a maximum of 10s for Serial Monitor
  while (!debugSerial && millis() < 10000)
    ;
    
  debugSerial.println("-- STATUS");
  ttn.showStatus();

  debugSerial.println("-- JOIN");
  ttn.join(appEui, appKey);
}//setup()
/**
    Maincode. Führt diesen immer alle 500ms aus.
    Es wird mittels readLightValuesAndCalculateCutOffedAverage() ein neuer Lichtwert gelesen und entsprechende Verwaltungsarbeit für die Strukt ausgeführt.
    Dannach wird mit enoughLightForSolarPowering() geprüft, ob genügend Licht für den Solarbetrieb zur Verfügung steht.
    Ensprechend des zurückgegebenen Ergebnisses von enoughLightForSolarPowering() wird entweder switchSolarOn() oder switchSolarOff() aufgerufen.
    Liefert checkIfTwoHoursHavePassedAndResetTimer() den Wert "true" zurück, wird transmitDataAndResetUsageCounter() aufgerufen.
    Als nächstes wird mit checkIfToilettIsOccupiedAndHandleThatEvent() geprüft ob, die Toilette besetzt wurde. Anschließend wird in checkIfToilettWasFreedAgain() geprüft,
    ob diese wieder frei ist.
    Zum Schluss wird mit delay() 500ms. lang nichts getan.
*/
void loop() {
  readLightValuesAndCalculateCutOffedAverage();
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


//////////////////////////////////////
//----------Hilfsfunktionen----------
//////////////////////////////////////

/**
   Diese Funktion prüft, ob der Türsensor ein Zuschließen der Toilettentür registriert.
   Ist dies der Fall und ToiletController.toilettOccupied ist auf "false" gesetzt, wird eine Toilettenbenutzung registriert.
   Dabei wird prevTimeSinceProgStartForTwentySecondDetection auf die aktuelle Zeit seit beginn des Arduino Programms gesetzt
   und ToiletController.usageCounter um eins inkrementiert sowie ToiletController.toilettOccupied auf den Wert "true" gesetzt.
*/
void checkIfToilettIsOccupiedAndHandleThatEvent() {
  if (digitalRead(INPUT_PIN_REED_SWITCH) == LOW
      && tc.toilettOccupied == false
     ) {
    prevTimeSinceProgStartForTwentySecondDetection = millis();
    tc.usageCounter++;
    Serial.println("--------====> TOILETE USAGE <====--------");
    Serial.println("New usage! Counter is now:");
    Serial.print(tc.usageCounter);
    Serial.println("--------====> TOILETE USAGE END<====--------");
    tc.toilettOccupied = true;
    letOnboardLEDBlink(250,5);
  }
}//checkIfToilettIsOccupiedAndHandleThatEvent()

/**
   Diese Funktion Prüft, ob die Toilette als "nicht besetzt" angesehen werden kann.
   Eine Toilette gillt erst wieder als niht besetzt, wenn:
   - Der Türsensor registriert, dass die Tür aufgeschlossen wurde,
   - der Wert von ToiletController.toilettOccupied auf "true" gesetzt ist,
   - 20 Sekunden vergangen sind, seit festgestellt wurde, dass die Toilette besetzt ist.
   Sind all diese Kriterien erfüllt wird ToiletController.toilettOccupied auf "false" gesetzt.
   Dies soll verhindern, dass bspw. Kinder am Türschloss spielen, diesen ständig auf und zu schließen
   und damit falsche Benutzungswerte erzeugen.
   Die Variable ToiletController.toilettOccupied wird verwendet, damit nicht ständig nach ablauf der 20 Sekunden eine neue Benutzung
   festgestellt wird, wenn jemand die Toilette länger als 20 Sekunden in Anspruch nimmt.
*/
void checkIfToilettWasFreedAgain() {
  if (digitalRead(INPUT_PIN_REED_SWITCH) == HIGH
      && tc.toilettOccupied == true
      && (millis() - prevTimeSinceProgStartForTwentySecondDetection >= TIMER_TILL_NEXT_USAGE_DETECTION)
     ) {
    tc.toilettOccupied = false;
  }

}//checkIfToilettWasFreedAgain()

//! TESTFUNKTION, um die LED des Arduino für entwicklungszwecke Blinken zu lassen.
void letOnboardLEDBlink(int delayInMS) {
  letOnboardLEDBlink(delayInMS, 1);
}//letOnboardLEDBlink()

/**
    Sind zwei Stunden seit Programmstart laut des internen Zählers des Arduinos vergangen,
    wird die Variable prevTimeSinceProgStartForSendingData auf den aktuellen Zeitwert
    des Arduinos gesetzt und der Wert "true" von der Methode zurückgeliefert.
    Andernfalls wird lediglich der Wert "false" zurückgegeben.
*/
boolean checkIfTwoHoursHavePassedAndResetTimer() {
  if ((unsigned long) (millis() - prevTimeSinceProgStartForSendingData ) >= TIMER_SEND_USAGECOUNT ) {
    Serial.println("--------====> TWO HOURS TIMER FOR DATA TRANSMISSION <====--------");
    Serial.println("2 Hours are over! ");
    Serial.println("--------====> TWO HOURS TIMER FOR DATA TRANSMISSION END <====--------");
    prevTimeSinceProgStartForSendingData = millis();
    return true;
  }
  return false;
}//checkIfTwoHoursHavePassedAndResetTimer
/**
   Übermittelt mittels des eingebauten LoRa Senders Typ A die Anzahl der Toilettenbenutzungen 
   und setzt ToiletController.usageCounter auf den Wert "0".
   Die Daten für die Anzahl der Toiletenbenutzungen werden in "little Endien" Format gesendet.  
   Zudem wird in Ascii zeichencodierung die Nachricht "guests:" vorab eingefügt.
   Somit besteht die Nachricht aus: "Guests:" und LOW-BYTE usage und HIGH-BYTE usage.
*/
void transmitDataAndResetUsageCounter() {
  Serial.println("===> Transmitting Data and resetting the counter of persons used the toilett.");
  String message = "Guests:";
  byte payload [9];
  message.getBytes(payload, 9);
  payload[7] = lowByte(tc.usageCounter);
  payload[8] = highByte(tc.usageCounter);

  for(int i = 0; i < 9; i++){
    Serial.print(payload[i]);
    Serial.print(" ");  
  }
  Serial.println();
  ttn.sendBytes(payload, sizeof(payload));
  
  tc.usageCounter = 0;
}//transmitDataAndResetUsageCounter()

//!TESTFUNKTION zum einschalten der eingebauten LED.
void switchLEDOn() {
  digitalWrite(LED_OUTPUT_PIN, HIGH);
}//switchLEDOn()

//!TESTFUNKTION zum ausschalten der eingebauten LED.
void switchLEDOff() {
  digitalWrite(LED_OUTPUT_PIN, LOW);
}//switchLEDOff


//!TESTFUNKTION Lässt die eingebaute LED blinken.
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

/**
   Stellt fest, ob der im durchschnitt ermittelte Lichtwert ausreichend für die Stormversorgung durch die Solaranlage ist.
   Ist dies der Fall, wird der Wert "true" zurückgegeben, andernfalls "false".
*/
bool enoughLightForSolarPowering() {
  if (lv.average <= MINIMUM_LIGHT_NEEDED) {
    //Serial.println("==========> THERE IS ENOUGH LIGHT! <==========");
    return true;
  } else {
    //Serial.println("==========> THERE IS NOT ENOUGH LIGHT! <==========");
    return false;
  }
}//enoughLightForSolarPowering()

/**
  Wird durch setup() einmalig aufgerufen. Initialisiert das LightValues lv Objekt bzw. Struct.
*/
void lightValuesStructInitializer() {

  for (int i = 0; i < lv.ARR_SIZE; i++) {
    lv.lightValuesMeasured[i] = 500;
  }
  lv.average = 500,
     lv.indexToInsertNext = 0;
}

/**
   Liest Werte vom Lichtsensor (Photoresistor) ein.
   Verwaltet Struct LightValues.
   LightValues.lightValuesMeasured ist ein zyklisches Array. Es wird der älteste Wert im Array überschrieben.
   Dafür wird LightValues.indexToInsertNext wieder auf "0" gesetzt, sobald das Ende des Arrays erreicht wurde.
   Die eingelesenen Lichtwerte werden durch calcCutOff() abgeschnitten. Mehr dazu siehe unter calcCutOff().
   Der abgeschnittene Wert wird in das LightValues.lightValuesMeasured Array gespeichert. Anschließend wird
   LightValues.indexToInsertNext um eins erhöht.Dannach werden alle Werte aus LightValues.lightValuesMeasured
   aufaddiert und durch die Anzahl der Elemente (LightValues.ARR_SIZE) geteilt. Der erechnete Wert wird dann
   LightValues.average zugewiesen.
   Ist die globale Variable lightvalCorrectionNeeded auf den Wert "true" gesetzt, wird der gelesene Lichtwert
   um LIGHT_VAL_OFFSET (Globale Konstante) verringert.
   Wird beispielsweise das Relais aktiviert, ensteht eine "Verdunkelung" der gemessenen Lichtwerte. Grund dafür
   ist in diesem Fall der hohe Stromverbrauch des Relais.
   Je höher der zurückgegebene Wert des Lichtsensors (Photoresistor) ausfällt, desto Dunkler ist die Umgebung.
   Alle Variablen dürfen Lesend zugegriffen werden. Schreibender Zugriff sollte nur von dieser Funktion vorgenommen werden.
*/
void readLightValuesAndCalculateCutOffedAverage() {
  unsigned long average = 0;
  unsigned int lightval = analogRead(INPUT_PIN_LIGHT_SENSOR);
  if (lightvalCorrectionNeeded) {
    //Serial.println("===!!!!!!!===LIGHTVALUE WILL BE CORRECTED.===!!!!!!!===");
    //Serial.println(LIGHT_VAL_OFFSET);
    //Serial.println("===!!!!!!!===    ===!!!!!!!===");
    lightval = lightval - LIGHT_VAL_OFFSET;
  }
  //Serial.println("--------====>    LIGHTVALUES   <====--------");
  //Serial.print("lightval: ");
  //Serial.println(lightval);
  lightval = calcCutOff(lightval, CUT_OFF);
  //Serial.print("cutOffedLightVal: ");
  //Serial.println(lightval);
  if (lv.indexToInsertNext >= lv.ARR_SIZE) {
    lv.indexToInsertNext = 0;
  }
  lv.lightValuesMeasured[lv.indexToInsertNext] = lightval;
  lv.indexToInsertNext++;
  for (int i = 0; i < lv.ARR_SIZE; i++) {
    average = average + lv.lightValuesMeasured[i];
  }
  lv.average = (double)average / lv.ARR_SIZE;
  //Serial.print("Calculated Average: ");
  //Serial.println(lv.average);
  //Serial.println("--------====>  LIGHTVALUES_END  <====--------");
}//readLightValuesAndCalculateCutOffedAverage()

/**
   Werte des Lichtsensors werden auf den Wert von cutOff aufgerundet.
   Ist lightvalueFromSensor größer als der cutOff Wert, wird auf das nächste vielfache von cutOff aufgerundet.
   Lichtwerte bspw. 6,7,8,9,10,11,12 mit einem cutOff von 9, ergeben folgende Werteaufrundungen:
   6 -> 9; 7 -> 9; 8 -> 9; 9 -> 9; 10 -> 18; 11 -> 18; 12 -> 18, usw.
   Der aufgerundete Wert wird dann zurückgegeben.
*/
int calcCutOff(int lightvalueFromSensor, int cutOff) {
  int thresholdedValue = 0;
  if ( (lightvalueFromSensor % cutOff) > 0) {
    thresholdedValue = lightvalueFromSensor + (cutOff - (lightvalueFromSensor % cutOff));
  } else {
    thresholdedValue = lightvalueFromSensor;
  }
  return thresholdedValue;
}//calcCutOff()

/**
   Aktiviert die Stormzufuhr der Solaranlage, indem das Relais die Kabelverbindung von
   Solaranlage und PassThrough Powerbank schließt.
   Da das Relais einen merklich hohen Energieanspruch hat (Energiebedarf des Arduino vervierfacht sich),
   muss lightvalCorrectionNeeded auf "true" gesetzt werden.
*/
void switchSolarOn() {
  lightvalCorrectionNeeded = true;
  digitalWrite(RELAIS_OUTPUT_PIN, LOW);
}
/**
   Deaktiviert die Stormzufuhr der Solaranlage, indem das Relais die Kabelverbindung
   von Solaranlage und PassThrough Powerbank öffnet. Da das Relais einen merklich
   hohen Energieanspruch hat (Energiebedarf des Arduino vervierfacht sich) und dieses nun ausgeschaltet wird,
   muss lightvalCorrectionNeeded auf "false" gesetzt werden.
*/
void switchSolarOff() {
  lightvalCorrectionNeeded = false;
  digitalWrite(RELAIS_OUTPUT_PIN, HIGH);
}
