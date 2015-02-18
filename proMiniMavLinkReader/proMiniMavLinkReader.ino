void setup()
{
  Serial1.begin(57600); 
  Serial.begin(57600);
}

void loop()
{
  uint8_t data = 0;
  while(Serial1.available())
  {
    data = Serial1.read();
    Serial.println(data);
  } 
}

