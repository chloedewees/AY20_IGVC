int statePin = 7;
int lightPin = 5;
int ledPin = 13;
bool lightState = true;
bool doBlink = false;
unsigned long tm = millis();

void setup() {
  pinMode(statePin, INPUT);
  pinMode(lightPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  if (millis()-tm>500) {
    tm = millis();
    digitalWrite(lightPin, !lightState);  // lights on with LOW.
    digitalWrite(ledPin, lightState);
    if (digitalRead(statePin))  {
      doBlink = true;
    }
    else {
      doBlink = false;
    }
    if (doBlink) {
      lightState = !lightState;
    }
    else {
      lightState = true;
    }
  }
}
