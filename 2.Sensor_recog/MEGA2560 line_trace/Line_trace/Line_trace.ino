// 핀 설정
const int sensorPins[5] = {2,3,4,5,6}; // 디지털 입력 (IR array)

int sensorVal[5];

void setup(){
  for(int i=0;i<5;i++) pinMode(sensorPins[i], INPUT);
  pinMode(motorL_pwm, OUTPUT);
  pinMode(motorR_pwm, OUTPUT);
  pinMode(motorL_dir, OUTPUT);
  pinMode(motorR_dir, OUTPUT);
  digitalWrite(motorL_dir, HIGH);
  digitalWrite(motorR_dir, HIGH);
  Serial.begin(9600);
}

void loop(){
  

  
}