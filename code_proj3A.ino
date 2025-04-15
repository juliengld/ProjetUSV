// --- Codeur et moteur ---
#define CHA 2
#define CHB 3
#define PWM 9
#define DIRA 8
#define E2 10 //Asservissement en couple
#define M2 7 //Direction du troncanneur
#define switchGauchePin = 0;
#define switchDroitePin = 1;


bool lastStateGauche = HIGH;
bool lastStateDroite = HIGH;

bool diraHIGH = false;
int PwmMax = 255;
int Umax = 12;

volatile int codor_count = 0;
static const int N = 500;
static const float R = 0.02585;
float L_init = 0;
float L = L_init;
float L_fin = 2;
float pi = 3.1415;

float Kp = 0;
float Ki = 0;
float Kd = 0;

float desired_speed = 0.0;
float current_speed = 0.0;
float last_error = 0.0;
float integral = 0.0;
float error = 0.0;
float derivative = 0.0;
float delta_time = 0;

int pwm_output = 0;
float output = 0.0;

bool startMove = false;
bool autoMode = false;
bool orderToStop = false;

int cptTickDepasses = 0;
int E2value = 212;
int cmd_PWM = 0;

unsigned long last_time = 0;
unsigned long current_time;

void setup() {
  // Moteur et codeur
  pinMode(CHA, INPUT);
  pinMode(CHB, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(M2, OUTPUT);

  // Switchs de fin de course
  pinMode(switchGauchePin, INPUT_PULLUP);
  pinMode(switchDroitePin, INPUT_PULLUP);

  Serial.begin(115200);
  digitalWrite(DIRA, LOW);

  attachInterrupt(digitalPinToInterrupt(CHA), read_encodeur, RISING);
}

void loop() {
  checkLevers();  // <--- Nouveau bloc

  static String receivedData = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      receivedData.trim();
      processMessage(receivedData);
      receivedData = "";
    } else {
      receivedData += c;
    }
  }

  AsservissementVitesse();
  analogWrite(PWM, pwm_output);

  if (autoMode) {
    if (startMove) {
      decision();
    } else {
      if (cmd_PWM != 0) {
        cmd_PWM = 0;
        analogWrite(PWM, cmd_PWM);
        afficheTickDepasses();
        L = 0;
        delay(1000);
      }
    }
  }

  delay(500);
}

void decision() {
  if (cmd_PWM == 0) {
    rotaDira();
    cmd_PWM = 150;
    analogWrite(PWM, cmd_PWM);
  }

  if (abs(L) > L_fin) {
    startMove = false;
    orderToStop = true;
  }
}

void afficheTickDepasses() {
  Serial.print("cptTickDepasses = ");
  Serial.println(cptTickDepasses);
  cptTickDepasses = 0;
  afficheL();
  orderToStop = false;
}

void afficheL() {
  Serial.print("L = ");
  Serial.println(L);
}

void controlWithKeyboard() {
  if (Serial.available() > 0) {
    char inChar = Serial.read();
    if (inChar == 'a') {
      fullAuto();
    }
    if (inChar == 's') {
      startMove = false;
      autoMode = true;
      orderToStop = false;
    }
    if (inChar == 'p') {
      autoMode = false;
      cmd_PWM = min(cmd_PWM + 10, 255);
      analogWrite(PWM, cmd_PWM);
    }
    if (inChar == 'm') {
      autoMode = false;
      cmd_PWM = max(0, cmd_PWM - 10);
      analogWrite(PWM, cmd_PWM);
    }
    if (inChar == 'r') {
      autoMode = false;
      rotaDira();
    }
    if (inChar == 'L') {
      afficheL();
    }
  }
  delay(100);
}

void fullAuto() {
  startMove = true;
  autoMode = true;
}

void rotaDira() {
  analogWrite(PWM, 0);
  delay(1000);
  if (diraHIGH) {
    diraHIGH = false;
    digitalWrite(DIRA, LOW);
  } else {
    diraHIGH = true;
    digitalWrite(DIRA, HIGH);
  }
}

void read_encodeur() {
  bool etatA = digitalRead(CHA);
  bool etatB = digitalRead(CHB);

  current_time = millis();
  delta_time = (current_time - last_time) / 1000.0;
  last_time = current_time;

  if (etatA && !etatB) {
    codor_count++;
    L = L - 2 * pi * R / N;
  }
  if (etatA && etatB) {
    codor_count--;
    L = L + 2 * pi * R / N;
  }

  current_speed = (2 * pi * R / N) / delta_time;

  if (orderToStop) {
    cptTickDepasses++;
  }
}

void AsservissementVitesse() {
  read_encodeur();

  error = desired_speed - current_speed;
  integral += error * delta_time;
  derivative = (error - last_error) / delta_time;
  output = Kp * error + Ki * integral + Kd * derivative;

  pwm_output = PwmMax * output / Umax;
  pwm_output = constrain(pwm_output, 0, 255);
  last_error = error;
}

void processMessage(String message) {
  message.trim();
  int separatorIndex = message.indexOf(":");

  if (separatorIndex != -1) {
    String command = message.substring(0, separatorIndex);
    String valueStr = message.substring(separatorIndex + 1);
    valueStr.trim();

    if (command == "SPEED") {
      desired_speed = valueStr.toFloat();  // 🔧 Utiliser toFloat pour les vitesses comme 0.3
      Serial.print("Vitesse reçue : ");
      Serial.println(desired_speed);
    }
  }
}

void controlDirection() {
  if (desired_speed <= 0) {
    analogWrite(E2, E2value);
    digitalWrite(M2, LOW);
    digitalWrite(DIRA, LOW);
  }
  if (desired_speed >= 0) {
    analogWrite(E2, 0);
    digitalWrite(M2, HIGH);
    digitalWrite(DIRA, HIGH);
  }
}

// --- vérifie les leviers
void checkLevers() {
  // GESTION LEVIER GAUCHE
  bool currentGauche = digitalRead(switchGauchePin);
  if (currentGauche != lastStateGauche) {
    delay(20);
    currentGauche = digitalRead(switchGauchePin);
    if (currentGauche != lastStateGauche) {
      lastStateGauche = currentGauche;
      if (currentGauche == HIGH) {
        Serial.println("SWITCHG:appuie");
      } else {
        Serial.println("SWITCHG:relache");
      }
    }
  }

  // GESTION LEVIER DROITE
  bool currentDroite = digitalRead(switchDroitePin);
  if (currentDroite != lastStateDroite) {
    delay(20);
    currentDroite = digitalRead(switchDroitePin);
    if (currentDroite != lastStateDroite) {
      lastStateDroite = currentDroite;
      if (currentDroite == HIGH) {
        Serial.println("SWITCHD:appuie");
      } else {
        Serial.println("SWITCHD:relache");
      }
    }
  }
}
