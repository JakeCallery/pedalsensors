int const POT_PIN = A2;
int potVal;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  potVal = analogRead(POT_PIN);
  Serial.print("potval: ");
  Serial.println(potVal);

}
