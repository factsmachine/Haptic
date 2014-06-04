const int buffer_size = 10;
char in_buff[buffer_size];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  Serial.readBytes(in_buff, buffer_size);
  Serial.println(atoi(in_buff));
}
