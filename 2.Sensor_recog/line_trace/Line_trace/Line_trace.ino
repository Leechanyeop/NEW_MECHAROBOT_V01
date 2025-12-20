// 핀 설정
const int sensorPins[5] = {2,3,4,5,6}; // 디지털 입력 (IR array)




// 모터 핀설정
const int motorL_pwm = 9;
const int motorR_pwm = 10;
const int motorL_dir = 7;
const int motorR_dir = 8;

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
  int sum=0, weightedSum=0;
  for(int i=0;i<5;i++){
    sensorVal[i] = digitalRead(sensorPins[i]); // 0 또는 1
    int pos = i*100; // 위치 가중치: 0,100,200,300,400
    weightedSum += sensorVal[i]*pos;
    sum += sensorVal[i];
  }
  int position = (sum==0) ? 200 : weightedSum / sum; // 0..400, 중앙=200

  int error = position - 200; // 음수: 왼쪽, 양수: 오른쪽
  int baseSpeed = 150;
  int Kp = 1; // 비례 상수 (튜닝 필요)
  int diff = Kp * error;

  int leftSpeed = constrain(baseSpeed - diff, 0, 255);
  int rightSpeed = constrain(baseSpeed + diff, 0, 255);

  analogWrite(motorL_pwm, leftSpeed);
  analogWrite(motorR_pwm, rightSpeed);

  Serial.print("pos:"); Serial.print(position);
  Serial.print(" L:"); Serial.print(leftSpeed);
  Serial.print(" R:"); Serial.println(rightSpeed);
  delay(10);
}