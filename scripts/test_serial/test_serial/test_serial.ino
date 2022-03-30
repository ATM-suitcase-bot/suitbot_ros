float counter = 0.0;
float x1 = 0.0, x2 = 0.0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (!Serial.available());
  String strIn = Serial.readString();
  
  int n = strIn.length();
  // declaring character array
  char char_array[n + 1];
  // copying the contents of the
  // string to char array
  strcpy(char_array, strIn.c_str());
  
  char * pch;
  pch = strtok(char_array," ");
  if (pch != NULL)
  {
    String x1_str(pch);
    x1 = x1_str.toFloat();
    pch = strtok (NULL, " ");
    if (pch != NULL)
    {
      String x2_str(pch);
      x2 = x2_str.toFloat();
    }
  }
  String dataToSend = String(x1, 8) + "\t" + String(x2,8);
  Serial.println(dataToSend);
}
