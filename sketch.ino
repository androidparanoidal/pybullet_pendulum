int sensorPin = A5;    // Порт к которому подключен потенциометр
int sensorValue = 0;  // Переменная для хранения показаний


void setup() {
  Serial.begin(9600);
}

void loop() {
  sensorValue = analogRead(sensorPin);
  //Serial.print("sensor = " );
  Serial.print(sensorValue);
  Serial.print(",");
  delay(25); // 25мс = 0.025сек
}