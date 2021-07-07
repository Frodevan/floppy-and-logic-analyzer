#define DIR_PIN 2
#define STEP_PIN 3
#define MOTOR_PIN 4
#define DS_PIN 5
#define HEAD_PIN 6
#define TRK0_PIN 13

void setup() {
  pinMode(TRK0_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, HIGH);
  pinMode(HEAD_PIN, OUTPUT);
  digitalWrite(HEAD_PIN, HIGH);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, HIGH);
  pinMode(DS_PIN, OUTPUT);
  digitalWrite(DS_PIN, HIGH);
}

void _step(){
  digitalWrite(STEP_PIN, LOW);
  delay(5);
  digitalWrite(STEP_PIN, HIGH);
  delay(5);
}

void step(String dir){
  if(dir == "+"){
    digitalWrite(DIR_PIN, LOW);  
    _step();
  }
  else if(dir == "-"){
    digitalWrite(DIR_PIN, HIGH);
    _step();
  }
}

void loop() {
  String command = Serial.readString();
  command.trim();
  if(command.startsWith("step")) {
    String arg = command.substring(4);
    arg.trim();
    if(arg == "0"){
      while(digitalRead(TRK0_PIN)){
        step("-");
      }
    }
    else step(arg);
    delay(50);
  }
  else if(command.startsWith("head")) {
    command.substring(4).trim();
    String arg = command.substring(4);
    arg.trim();
    if (arg == "0") digitalWrite(HEAD_PIN, HIGH);
    else if (arg == "1") digitalWrite(HEAD_PIN, LOW);
  }
  else if(command == "start") {digitalWrite(MOTOR_PIN, LOW); digitalWrite(DS_PIN, LOW);}
  else if(command == "stop") {digitalWrite(MOTOR_PIN, HIGH); digitalWrite(DS_PIN, HIGH);}
  if(command.length() > 0)
  {
    Serial.print(command);
    Serial.print("\r\n");
    Serial.print("OK");
    Serial.print("\r\n");
  }
}
