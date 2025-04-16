// --- Codeur et moteur ---
#define CHA 2
#define CHB 3
#define PWM 9
#define DIRA 8
#define E2 10 //Asservissement en couple
#define M2 7 //Direction du troncanneur
#define switchGauchePin 0
#define switchDroitePin 1


bool lastStateGauche = HIGH;
bool lastStateDroite = HIGH;

bool diraHIGH = false;
int PwmMax = 255;
int Umax = 12;

volatile int codor_count = 0;
volatile int speed_tick_count = 0;
static const int N = 500;
static const float R = 0.02585;
float L_init = 0;
float L = L_init;
float L_fin = 2;
float pi = 3.1415;

float Kp = 27;
float Kd = 0;

float desired_speed = -0.00;
float current_speed = 0.0;
float last_error = 0.0;
float integral = 0.0;
float error = 0.0;
float derivative = 0.0;
float delta_time = 0;

int pwm_output = 0;
float output = 0.0;
float last_output = 0.0;
bool first_run = true;

bool startMove = false;
bool autoMode = false;
bool orderToStop = false;

int cptTickDepasses = 0;
int E2value = 212;
int cmd_PWM = 0;

unsigned long last_time = 0;
unsigned long current_time;
unsigned long last_speed_time = 0;

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
  delay(2000);
}

void loop() {
  checkLevers();

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
  controlDirection();
  AsservissementVitesse();
  delay(100);
}

void afficheL() {
  Serial.print("L = ");
  Serial.println(L);
}


void read_encodeur() {
  bool etatA = digitalRead(CHA);
  bool etatB = digitalRead(CHB);

  current_time = millis();
  delta_time = (current_time - last_time) / 1000.0;
  last_time = current_time;

  speed_tick_count++;

  if (etatA && !etatB) {
    codor_count++;
    L = L - 2 * pi * R / N;
  }
  if (etatA && etatB) {
    codor_count--;
    L = L + 2 * pi * R / N;
  }

  current_speed = (2 * pi * R / N) / delta_time;
}

void updateSpeed(){
  static unsigned long last_speed = 0;
  unsigned long now = millis();
  delta_time = (now-last_speed_time)/1000.0;
  last_speed_time =now;

  noInterrupts();
  int ticks = speed_tick_count;
  speed_tick_count =0;
  interrupts();

  float delta_L = (2*pi*R*ticks)/N;
  current_speed = delta_L/delta_time;

   if (digitalRead(DIRA) == LOW){
    current_speed = abs(current_speed);
  }
  else {
    current_speed = -abs(current_speed);
  }

  Serial.print("current_speed: ");
  Serial.println(current_speed);

  
}

void AsservissementVitesse() {
  updateSpeed();
  error = desired_speed - current_speed;
  derivative = (error - last_error) / delta_time;
  output = Kp*error;
  
  if ((output > 0 && last_output < 0) || (output < 0 && last_output > 0) || first_run){
    Serial.print("Changement de signe: ");
    Serial.println(output);
    if(output > 0){
      digitalWrite(DIRA, LOW);
    }
    else{
      digitalWrite(DIRA, HIGH);
    }
    first_run = false;
  } 

  pwm_output = PwmMax * abs(output) / Umax;
  pwm_output = constrain(pwm_output, 0, 255);

  analogWrite(PWM, pwm_output);
  
  last_output = output;
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
  if (current_speed < 0) {
    analogWrite(E2, E2value);
    digitalWrite(M2, LOW);
  }
  if (current_speed >= 0) {
    analogWrite(E2, 0);
    digitalWrite(M2, HIGH);
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
