#include <Arduino.h>
#include "main.h"

#include "Servo.h"
#include "utils.h"

// TODO : Sélection du camps avec un bouton et non un define
// Côté bleu = 1 et Côté jaune = 2
#define PAMI_SIDE 1
#define PAMI_NUM 2
#define START_DELAY 5000

// Stepper
volatile int32_t speed_steps_per_sec = 0;
volatile int32_t steps_target = 0;
volatile int32_t steps_done = 0;
volatile bool movementInProgress = false;
Direction lastDirection = STOP;
volatile int32_t lastSpeed = 0;

// Scénario
#if PAMI_NUM == 1
Step scenario[] = {
  {STEP_FORWARD, 105, 2500},
  {STEP_ROTATE, PAMI_SIDE == 1 ? -90 : 90, 1000},
  {STEP_FORWARD, 50, 2000}
};
#elif PAMI_NUM == 2
Step scenario[] = {
  {STEP_BACKWARD, 35, 2500},
  {STEP_BACKWARD, 80 - 10 * PAMI_NUM, 3000}
};
#else
Step scenario[] = {
  {STEP_FORWARD, 35, 2500},
  {STEP_ROTATE, PAMI_SIDE == 1 ? -45 + 10 * (PAMI_NUM - 2) : 45 - 10 * (PAMI_NUM - 2), 500},
  {STEP_FORWARD, 80 - 10 * PAMI_NUM, 3000}
};
#endif

const int scenarioLength = sizeof(scenario) / sizeof(Step);
int currentScenarioStep = 0;
bool scenarioInProgress = false;

// Ultrason (non bloquant)
volatile unsigned long distanceCM = 1000;
const unsigned int obstacleThresholdCM = 7;
unsigned long lastUltrasonicTrigger = 0;
bool waitingEcho = false;
unsigned long echoStart = 0;

// Servo
const int SERVO_EAT_UP = 135;
const int SERVO_EAT_DOWN = 45;
Servo servo_eat;

// Tirette et EMG
bool tirettePose = false;
uint32_t startTime = 0;

// Tâche ultrason sur Core 0
[[noreturn]] void ultrasonicTask(void *pvParameters) {
  for (;;) {
    // Trigger
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(20);
    digitalWrite(TRIG_PIN, LOW);

    // Lecture bloquante
    unsigned long duration = pulseIn(ECHO_PIN, HIGH, 20000); // timeout 20ms
    unsigned long newDistance = duration * 0.034 / 2; // cm

    distanceCM = newDistance;

    vTaskDelay(50 / portTICK_PERIOD_MS); // 50ms entre mesures
  }
}

// ========================= Setup =========================
void setup() {
  Serial.begin(115200);

  // Pins moteurs
  // Moteur 1 (Droite)
  pinMode(M1_STEP_PIN, OUTPUT);
  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M1_ENABLE_PIN, OUTPUT);
  // Moteur 2 (Gauche)
  pinMode(M2_STEP_PIN, OUTPUT);
  pinMode(M2_DIR_PIN, OUTPUT);
  pinMode(M2_ENABLE_PIN, OUTPUT);

  // Capteurs
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  // Tirette de démarrage
  pinMode(TIRETTE_PIN, INPUT_PULLUP);
  // Bouton d'arrêt d'urgence (HIGH si appuyé)
  pinMode(EMG_PIN, INPUT);

  // Servo
  servo_eat = Servo(SERVO_PIN, 1);
  servo_eat.writeAngle(SERVO_EAT_UP);


  enableMotorDrivers();
  speed_steps_per_sec = 0;

  // Créer la tâche sur Core 0
  xTaskCreatePinnedToCore(
    ultrasonicTask,
    "UltrasonicTask",
    2000,
    NULL,
    1,
    NULL,
    0); // Core 0

  Serial.println("Setup complete");
}

// ========================= Steppers =========================
void updateSteppers() {
  static uint32_t last_step_time1 = 0;
  uint32_t now = micros();

  if (speed_steps_per_sec != 0 && steps_done < steps_target) {
    uint32_t interval = 1000000UL / abs(speed_steps_per_sec);
    if (now - last_step_time1 >= interval) {
      digitalWrite(M1_STEP_PIN, HIGH);
      digitalWrite(M2_STEP_PIN, HIGH);
      delayMicroseconds(2);
      digitalWrite(M1_STEP_PIN, LOW);
      digitalWrite(M2_STEP_PIN, LOW);
      last_step_time1 = now;
      steps_done++;
    }
  }

  if (movementInProgress && steps_done >= steps_target) {
    speed_steps_per_sec = 0;
    movementInProgress = false;
    Serial.println("Etape terminée");
  }
}

// Arrêt des moteurs
void stopMotors() {
  speed_steps_per_sec = 0;
}

bool isMoving() {
  return speed_steps_per_sec != 0;
}

// ========================= Détection obstacles =========================
void detectObstacles() {
  if (distanceCM > 0 && distanceCM < obstacleThresholdCM) {
    if (isMoving()) {
      lastSpeed = speed_steps_per_sec;
      speed_steps_per_sec = 0;
      Serial.println("Obstacle détecté : Arrêt");
      Serial.println(distanceCM);
    }
  } else if (!isMoving() && movementInProgress) {
    Serial.println("Obstacle évité : reprise mouvement");
    Serial.println(distanceCM);
    speed_steps_per_sec = lastSpeed;
  }
}

// ========================= Scenario =========================
void moveAsyncSteps(int32_t steps, int32_t speed, bool forwardDir) {
  steps_target = steps;
  steps_done = 0;
  digitalWrite(M1_DIR_PIN, forwardDir ? HIGH : LOW);
  digitalWrite(M2_DIR_PIN, forwardDir ? HIGH : LOW);
  speed_steps_per_sec = speed;
  movementInProgress = true;
  lastDirection = forwardDir ? FORWARD : BACKWARD;
}

void rotateAsync(float angleDeg, int32_t speed, bool toRight) {
  steps_target = getRotationSteps(angleDeg + 10.0);
  steps_done = 0;
  digitalWrite(M1_DIR_PIN, toRight ? HIGH : LOW);
  digitalWrite(M2_DIR_PIN, toRight ? LOW : HIGH);
  speed_steps_per_sec = speed;
  movementInProgress = true;
  lastDirection = toRight ? RIGHT : LEFT;
}

int processScenario() {
  if (!scenarioInProgress) return 0;
  if (movementInProgress) return 1;

  if (currentScenarioStep >= scenarioLength) {
    Serial.println("Scénario terminé !");
    scenarioInProgress = false;
    return 0;
  }

  Step &step = scenario[currentScenarioStep];
  switch (step.type) {
    case STEP_FORWARD: moveAsyncSteps(getStepsForDistance(step.value), step.speed, true);
      break;
    case STEP_BACKWARD: moveAsyncSteps(getStepsForDistance(step.value), step.speed, false);
      break;
    case STEP_ROTATE:
      if (step.value >= 0) rotateAsync(step.value, step.speed, true);
      else rotateAsync(-step.value, step.speed, false);
      break;
  }
  currentScenarioStep++;
  return 1;
}

// ========================= Loop principal =========================
void loop() {
  // Tirette
  if (digitalRead(TIRETTE_PIN) == LOW && !tirettePose) {
    tirettePose = true;
    Serial.println("Trigger initialized");
  } else if (digitalRead(TIRETTE_PIN) == HIGH && tirettePose) {
    Serial.println("Trigger removed");
    Serial.print("Switch : ");
    Serial.println(readSwitchOnce(SWITCH_PIN) ? "OFF" : "ON");
    delay(START_DELAY);
    Serial.println("Starting the script");
    startTime = millis();
    scenarioInProgress = true;

    while (true) {
      updateSteppers();
      detectObstacles();
      processScenario();

      // Fin de scénario ou timeout
      if (!scenarioInProgress || millis() - startTime >= 100000) {
        stopMotors();
        Serial.println("Script finished, motor stopped");

        // Servo en boucle jusqu'à reset
        Serial.println("Starting the post game action");
        while (true) {
          if (!digitalRead(EMG_PIN)) {
            servo_eat.writeAngle(SERVO_EAT_UP);
            delay(500);
            servo_eat.writeAngle(SERVO_EAT_DOWN);
            delay(500);
          } else {
            Serial.println("Restarting the ESP 32 ...");
            ESP.restart();
          }
        }
      }
    }
  }
}
