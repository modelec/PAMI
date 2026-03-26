#include <Arduino.h>
#include "main.h"

#include "Servo.h"
#include "Stepper.h"
#include "Steppers.h"
#include "UltraSonicSensor.h"
#include "utils.h"

#define PAMI_NUM 2
#define START_DELAY 5000

// Côté bleu = 0 et Côté jaune = 1
int side;

// Stepper
volatile int32_t speed_steps_per_sec = 0;
volatile int32_t steps_target = 0;
volatile int32_t steps_done = 0;
volatile bool movementInProgress = false;
Direction lastDirection = STOP;
volatile int32_t lastSpeed = 0;

// TODO : Générer le tableau de scénario après avoir chosit le coté
// Scénario
#if PAMI_NUM == 1
Step scenario[] = {
  {STEP_FORWARD, 105, 2500},
  {STEP_ROTATE, PAMI_SIDE == 1 ? -90 : 90, 1000},
  {STEP_FORWARD, 50, 2000}
};
#elif PAMI_NUM == 2
Step scenario[] = {
  {STEP_FORWARD, 80 - 10 * PAMI_NUM, 3000},
  {STEP_BACKWARD, 35, 2500}
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

// Ultrason
volatile unsigned long distanceCM = 1000;
const unsigned int obstacleThresholdCM = 7;
UltraSonicSensor ultraSonic = UltraSonicSensor(TRIG_PIN, ECHO_PIN);

// Servo
const int SERVO_EAT_UP = 135;
const int SERVO_EAT_DOWN = 45;
Servo servo_eat = Servo(SERVO_PIN, 1);

auto stepperRight = Stepper(M1_ENABLE_PIN, M1_DIR_PIN, M1_STEP_PIN, true);
auto stepperLeft = Stepper(M2_ENABLE_PIN, M2_DIR_PIN, M2_STEP_PIN);
auto steppers = Steppers({stepperLeft, stepperRight});

// Tirette et EMG
bool tirettePose = false;
uint32_t startTime = 0;

// Tâche ultrason sur Core 0
[[noreturn]] void ultrasonicTask(void *pvParameters) {
  for (;;) {
    // Lecture bloquante
    distanceCM = ultraSonic.readCm();

    vTaskDelay(200 / portTICK_PERIOD_MS); // 50ms entre mesures
  }
}

// ========================= Setup =========================
void setup() {
  Serial.begin(115200);

  // Pins moteurs
  steppers.init();

  // Capteurs
  ultraSonic.init();
  // Tirette de démarrage
  pinMode(TIRETTE_PIN, INPUT_PULLUP);
  // Bouton d'arrêt d'urgence (HIGH si appuyé)
  pinMode(EMG_PIN, INPUT);

  // Servo
  servo_eat.init();
  servo_eat.writeAngle(SERVO_EAT_UP);


  steppers.enable();
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
      steppers.stepAll();
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
  steppers.writeDir(forwardDir);
  speed_steps_per_sec = speed;
  movementInProgress = true;
  lastDirection = forwardDir ? FORWARD : BACKWARD;
}

void rotateAsync(float angleDeg, int32_t speed, bool toRight) {
  steps_target = getRotationSteps(angleDeg + 10.0);
  steps_done = 0;
  stepperLeft.writeDir(toRight);
  stepperRight.writeDir(!toRight);
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
    side = readSwitchOnce(SWITCH_PIN);
    Serial.print("Side : ");
    Serial.println(side ? "BLUE" : "YELLOW");
  } else if (digitalRead(TIRETTE_PIN) == HIGH && tirettePose) {
    Serial.println("Trigger removed");
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
