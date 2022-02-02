int inBytes;
int t_conf = 5;
String data = "5,25"; //"5, 7, 22, 4, 30"; // 0-20C
int volt[5] = {25, 26, 40, 15, 25}; // 20-33.6V
int flag;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if  (Serial.available() > 0)
  {
    flag = Serial.read();

    if (flag == 1)
    {
      Serial.write(20); //Temp
    }
    if (flag == 2)
    {
      Serial.write(30); //Volt
    }
  }
}
