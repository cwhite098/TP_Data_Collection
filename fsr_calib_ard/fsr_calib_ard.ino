
int fsr = A2;


void setup() {
  // put your setup code here, to run once:
  pinMode(fsr, INPUT); // init the fsr pin as an input
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int data = analogRead(fsr);
  Serial.println(data);
  delay(100);
}
