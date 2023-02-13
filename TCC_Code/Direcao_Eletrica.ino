#include <Servo.h>

Servo saida;

class PID {
  public:

    double error;
    double sample;
    double lastSample;
    double kP, kI, kD;
    double P, I, D;
    double pid;

    double setPoint;
    long lastProcess;

    PID(double _kP, double _kI, double _kD) {
      kP = _kP;
      kI = _kI;
      kD = _kD;
    }

    void addNewSample(double _sample) {
      sample = _sample;
    }

    void setSetPoint(double _setPoint) {
      setPoint = _setPoint;
    }

    double process() {
      // Implementação P ID
      error = setPoint - sample;
      float deltaTime = (millis() - lastProcess) / 1000.0;
      lastProcess = millis();

      //P
      P = error * kP;

      //I
      I = I + (error * kI) * deltaTime;

      //D
      D = (lastSample - sample) * kD / deltaTime;
      lastSample = sample;

      // Soma tudo
      pid = P + I + D;

      if (pid < -100) {
        pid = -100;
      }

      if (pid > 100) {
        pid = 100;
      }
      return pid;
    }
};

#define pSETPOINT       A4
#define pFEEDBACK       A1

//PID Pid(0.8, 0.0, 0.05);
PID Pid(3.0, 0.0, 0.0);

long setPoint = 127;
long setPointSerial = 127;
//long setPoint = 0;
long posicao = 0;
int outputValue = 0;

int controlePwm = 50;

void setup() {
  Serial.begin(115200);
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);
  saida.attach(9);
  delay(5000);
}

int valorSerial = 127;

void loop() {
  //setPoint = 0;
  posicao = 0;


  //    for(int i = 0; i < 100; i++){
  //      setPoint = setPoint + analogRead(pSETPOINT);
  //    }
  //    setPoint = map( (setPoint/100) , 0, 1023, 140, -140);


  valorSerial = Serial.read();
  if (valorSerial != -1 && valorSerial < 255 && valorSerial > 0) {
    setPointSerial = valorSerial;
  }
  Serial.println(analogRead(pSETPOINT));
  setPoint = map(setPointSerial, 0, 255, -140, 140);

  Pid.setSetPoint(setPoint);

  for (int i = 0; i < 100; i++) {
    posicao = posicao + analogRead(pFEEDBACK);
  }
  posicao = map( (posicao / 100) , 438, 578, -140, 140);
  Pid.addNewSample(posicao);

  // Converte para controle
  controlePwm = (Pid.process());
  controlePwm = map( controlePwm , -100, 100, 180, 0);

  //    Serial.print("Set Point = ");
  //    Serial.print(setPoint);
  //    Serial.print("\t Posicao = ");
  //    Serial.print(posicao);
  //    Serial.print("\t Saida = ");
  //    Serial.println(controlePwm);

  // Saída do controle
  saida.write(controlePwm);
  delay(2);
}
