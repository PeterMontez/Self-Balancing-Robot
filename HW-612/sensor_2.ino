float gX, gx2,aX, ax2;

void setup() {
  Serial.begin(9600);

  Serial.println("Porta serial ligada!");

  setupSensor();
}

int count;
void loop() {
  Serial.println(String(getGyro()));
  Serial.println(String(getAccel()));
}
