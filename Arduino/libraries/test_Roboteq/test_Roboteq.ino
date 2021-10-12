void setup() {
  // put your setup code here, to run once:
  Serial1.begin(115200);
}

void loop() {
  

    Serial1.print("!PR");  // Motor GO command
    Serial1.print(" ");   //   Space
    Serial1.print("02");   // Channel Number
    Serial1.print(" ");   //   Space
    Serial1.println("+100");   // Motor Power Value

    delay(1000);
  
  
}
