// =====================================================
// 3 MOTORES + 3 POTENCIOMETROS
// ESP32 + L293D
//
// PWM:
//   -255 ... 0 ... +255
//
// Feedback:
//   FB,pos1,pos2,pos3
//
// O ESP32 NÃO faz PID.
// O PID será executado no computador.
// =====================================================


// =====================================================
// MOTOR 1
// =====================================================

const int M1_IN1 = 25;
const int M1_IN2 = 26;
const int M1_EN  = 27;
const int M1_POT = 34;


// =====================================================
// MOTOR 2
// =====================================================

const int M2_IN1 = 18;
const int M2_IN2 = 19;
const int M2_EN  = 23;
const int M2_POT = 35;


// =====================================================
// MOTOR 3
// =====================================================

const int M3_IN1 = 16;
const int M3_IN2 = 17;
const int M3_EN  = 22;
const int M3_POT = 32;


// =====================================================
// CONFIGURAÇÃO
// =====================================================

// PWM máximo
const int PWM_MAX = 255;

// PWM mínimo aproximado para começar a girar
const int PWM_MIN = 60;


// =====================================================
// COMANDOS RECEBIDOS
// =====================================================

int cmd1 = 0;
int cmd2 = 0;
int cmd3 = 0;


// =====================================================
// FUNÇÃO PARA CONTROLAR MOTOR
// =====================================================

void setMotor(
  int in1,
  int in2,
  int en,
  int command
)
{
  command = constrain(command, -PWM_MAX, PWM_MAX);


  // ---------------------------------------------
  // MOTOR PARADO
  // ---------------------------------------------

  if (command == 0)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);

    ledcWrite(en, 0);

    return;
  }


  // ---------------------------------------------
  // PWM
  // ---------------------------------------------

  int pwm = abs(command);


  // ---------------------------------------------
  // COMPENSAÇÃO DO ATRITO
  //
  // Se o comando estiver entre 1 e 59,
  // usamos PWM_MIN.
  //
  // Porém preservamos o zero.
  // ---------------------------------------------

  if (pwm < PWM_MIN)
  {
    pwm = PWM_MIN;
  }


  pwm = constrain(pwm, PWM_MIN, PWM_MAX);


  // ---------------------------------------------
  // DIREÇÃO
  // ---------------------------------------------

  if (command > 0)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }


  // ---------------------------------------------
  // APLICA PWM
  // ---------------------------------------------

  ledcWrite(en, pwm);
}


// =====================================================
// PARAR TODOS
// =====================================================

void stopAllMotors()
{
  setMotor(
    M1_IN1,
    M1_IN2,
    M1_EN,
    0
  );

  setMotor(
    M2_IN1,
    M2_IN2,
    M2_EN,
    0
  );

  setMotor(
    M3_IN1,
    M3_IN2,
    M3_EN,
    0
  );
}


// =====================================================
// RECEBER COMANDO SERIAL
//
// Formato:
//
// CMD,100,0,-50
// =====================================================

void readCommand()
{
  if (!Serial.available())
    return;


  String data = Serial.readStringUntil('\n');

  data.trim();


  if (!data.startsWith("CMD"))
    return;


  int c1;
  int c2;
  int c3;


  int result = sscanf(
    data.c_str(),
    "CMD,%d,%d,%d",
    &c1,
    &c2,
    &c3
  );


  if (result == 3)
  {
    cmd1 = constrain(
      c1,
      -PWM_MAX,
      PWM_MAX
    );

    cmd2 = constrain(
      c2,
      -PWM_MAX,
      PWM_MAX
    );

    cmd3 = constrain(
      c3,
      -PWM_MAX,
      PWM_MAX
    );
  }
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);


  // ---------------------------------------------
  // GPIO
  // ---------------------------------------------

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);

  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);


  // ---------------------------------------------
  // PWM
  //
  // 1000 Hz
  // 8 bits
  // 0 ... 255
  // ---------------------------------------------

  ledcAttach(
    M1_EN,
    1000,
    8
  );

  ledcAttach(
    M2_EN,
    1000,
    8
  );

  ledcAttach(
    M3_EN,
    1000,
    8
  );


  // ---------------------------------------------
  // ADC
  // ---------------------------------------------

  analogReadResolution(12);


  // ---------------------------------------------
  // GARANTIR MOTORES PARADOS
  // ---------------------------------------------

  stopAllMotors();


  Serial.println("READY");
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
  // ---------------------------------------------
  // RECEBER COMANDOS
  // ---------------------------------------------

  readCommand();


  // ---------------------------------------------
  // APLICAR COMANDOS
  // ---------------------------------------------

  setMotor(
    M1_IN1,
    M1_IN2,
    M1_EN,
    cmd1
  );

  setMotor(
    M2_IN1,
    M2_IN2,
    M2_EN,
    cmd2
  );

  setMotor(
    M3_IN1,
    M3_IN2,
    M3_EN,
    cmd3
  );


  // ---------------------------------------------
  // FEEDBACK
  //
  // 100 Hz
  // ---------------------------------------------

  static unsigned long lastFeedback = 0;


  if (millis() - lastFeedback >= 10)
  {
    lastFeedback = millis();


    int pos1 = analogRead(M1_POT);
    int pos2 = analogRead(M2_POT);
    int pos3 = analogRead(M3_POT);


    Serial.print("FB,");

    Serial.print(pos1);
    Serial.print(",");

    Serial.print(pos2);
    Serial.print(",");

    Serial.println(pos3);
  }
}