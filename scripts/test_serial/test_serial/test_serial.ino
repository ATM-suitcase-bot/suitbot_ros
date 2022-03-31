float counter = 0.0;
float x1 = 0.0, x2 = 0.0;
void setup() {
  // put your setup code here, to run once:
  Serial.setTimeout(50);
  Serial.begin(115200);
}



void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()){
  long start = micros();
  String strIn = Serial.readString();
  
  long ends = micros();
  
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
  long delta = ends - start;
  String dataToSend = "data\t2\t2\t" + String(delta) + "\t" + String(x2,4);
  Serial.println(dataToSend);
  }
}
