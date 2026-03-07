#include <Arduino.h>
// #define SIMULATOR // Commenter cette ligne pour utiliser le matériel réel
//     Définitions des constantes dans main.h
#include "main.h"
#include "utils.h"

// TODO : Sélection du camps avec un bouton et non un define
// Côté bleu = 1 et Côté jaune = 2
#define PAMI_SIDE 1
// TODO : Brève description de quel pami correspond a quel numéro
// Numéro du pami pour les différents robots car chemins différents
#define PAMI_NUM 2
// FIXME : Mettre au min à START_DELAY 85000 (85s)
#define START_DELAY 5000

//FIXME : Le reset des value devrait pas etre fait dans le setup ? (Si on considère que le code peut s'auto restart)
uint32_t startTime = 0;

volatile int32_t speed_steps_per_sec = 0; // target speed (signed)
uint32_t last_step_time = 0;

// Pour le redémarrage des moteurs après capteur obstacle
Direction lastDirection = STOP;
volatile int32_t lastSpeed = 0;

volatile int32_t steps_target = 0;
volatile int32_t steps_done = 0;
bool movementInProgress = false;

// Les différents scénarios possibles
#if PAMI_NUM == 1
// Chaque étape du scénario écureuil ninja
// TODO
Step scenario[] = {
  {STEP_FORWARD, 105, 2500},
  {STEP_ROTATE, PAMI_SIDE == 1 ? -90 : 90, 1000}, // Tourner à gauche si côté bleu, droite si jaune
  {STEP_FORWARD, 50, 2000}
};
#elif PAMI_NUM == 2
// TODO : Voir comment gerer les différence de distance a parcourir selon PAMI_NUM, plein de if ?
Step scenario[] = {
  {STEP_BACKWARD, 35, 2500},
  // {STEP_ROTATE, PAMI_SIDE == 1 ? -45 + 10 * (PAMI_NUM - 2) : 45 - 10 * (PAMI_NUM - 2), 500},
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

// Arrêt des moteurs
void stopMotors() {
  speed_steps_per_sec = 0;
}

// Fonction d'initialisation
void setup() {
  Serial.begin(115200);
  // Bouton d'arrêt d'urgence (HIGH si appuyé)
  pinMode(EMG_PIN, INPUT);
  // Moteur 1 (Droite)
  pinMode(M1_STEP_PIN, OUTPUT);
  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M1_ENABLE_PIN, OUTPUT);
  // Moteur 2 (Gauche)
  pinMode(M2_STEP_PIN, OUTPUT);
  pinMode(M2_DIR_PIN, OUTPUT);
  pinMode(M2_ENABLE_PIN, OUTPUT);
  // Tirette
  pinMode(TIRETTE_PIN, INPUT_PULLUP);
  // Capteur à ultrasons
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Servo moteur
  pinMode(SERVO_PIN, OUTPUT);

  ledcSetup(0, 50, 16);
  ledcAttachPin(SERVO_PIN, 0);

  setServoAngle(130);

  enableMotorDrivers();
  stopMotors();

  scenarioInProgress = false;
  currentScenarioStep = 0;

  Serial.println("Setup complete");
  Serial.println("Waiting for the trigger initialization");
}

void moveAsyncSteps(int32_t steps, int32_t speed, bool forwardDir) {
  // Le nombre de pas à faire
  steps_target = steps;
  steps_done = 0;

  // La direction du moteur
  digitalWrite(M1_DIR_PIN, forwardDir ? HIGH : LOW);
  digitalWrite(M2_DIR_PIN, forwardDir ? HIGH : LOW);

  speed_steps_per_sec = speed;
  movementInProgress = true;
  lastDirection = forwardDir ? FORWARD : BACKWARD;
}

void rotateAsync(float angleDeg, int32_t speed, bool toRight) {
  // Le nombre de pas à faire
  steps_target = getRotationSteps(angleDeg + 10.0); // 10.0 correspond à une correction trouvée à la main :)
  steps_done = 0;

  // La direction du moteur
  digitalWrite(M1_DIR_PIN, toRight ? HIGH : LOW);
  digitalWrite(M2_DIR_PIN, toRight ? LOW : HIGH);

  speed_steps_per_sec = speed;
  movementInProgress = true;
  lastDirection = toRight ? RIGHT : LEFT;
}

// Non-blocking stepper update function
void updateSteppers() {
  uint32_t now = micros();

  // Moteur 1
  if (speed_steps_per_sec != 0 && (steps_done < steps_target)) {
    uint32_t interval1 = 1000000UL / abs(speed_steps_per_sec);
    static uint32_t last_step_time1 = 0;
    if (now - last_step_time1 >= interval1) {
      digitalWrite(M1_STEP_PIN, HIGH);
      digitalWrite(M2_STEP_PIN, HIGH);
      delayMicroseconds(2); // Short pulse width for step signal
      digitalWrite(M1_STEP_PIN, LOW);
      digitalWrite(M2_STEP_PIN, LOW);
      last_step_time1 = now;
      steps_done++;
    }
  }

  // Comptage des pas (on considère le moteur le plus lent pour terminer l'étape)
  if (movementInProgress && steps_done >= steps_target) {
    stopMotors();
    movementInProgress = false;
    Serial.println("Etape terminé");
  }
}

// Vérifie si les moteurs sont en mouvement
bool isMoving() {
  return speed_steps_per_sec != 0;
}

// Capteur à ultrasons
const int obstacleCheckInterval = 100; // ms between distance checks
unsigned long lastObstacleCheckTime = 0;
const int obstacleThresholdCM = 7; // stop if closer than 7 cm

long readDistanceCM(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000); // timeout 20 ms
  long distance = duration * 0.034 / 2;
  return distance;
}

bool recherchePlace = false;
int rechercheStep = 0;
// Nombre de pas restants à faire après la reprise de place
int stepsRemainingPlace = 0;

// Sous-scénario pour la recherche de place
const int NB_RECHERCHE_STEPS = 3;
Step rechercheScenario[NB_RECHERCHE_STEPS] = {
  {STEP_FORWARD, 20},
  {STEP_ROTATE, PAMI_SIDE == 1 ? -90 : 90}
};

void detectObstacles() {
  unsigned long now = millis();
  if (now - lastObstacleCheckTime >= obstacleCheckInterval) {
    lastObstacleCheckTime = now;
    // TODO : Threashold pour rotation ?
    if (lastDirection == RIGHT || lastDirection == LEFT) {
      return;
    }
    long distance = readDistanceCM(TRIG_PIN, ECHO_PIN);
    if (distance > 0 && distance < obstacleThresholdCM) {
      if (isMoving()) {
        lastSpeed = speed_steps_per_sec;
        stopMotors();
        Serial.println("Obstacle détecté : Arrêt");
      }
    } else {
      if (!isMoving() && movementInProgress) {
        Serial.println("Plus d'obstacle : Reprise du mouvement");
        switch (lastDirection) {
          case FORWARD:
            moveAsyncSteps(steps_target - steps_done, lastSpeed, true);
            break;
          case BACKWARD:
            moveAsyncSteps(steps_target - steps_done, lastSpeed, false);
            break;
          default:
            break;
        }
      }
    }
  }
}

// On gère le scénario du robot, retourne 0 si le scénario est terminé, 1 si en cours
int processScenario() {
  if (!scenarioInProgress)
    return 0;
  if (movementInProgress)
    return 1;

  if (currentScenarioStep >= scenarioLength) {
    Serial.println("Scénario terminé !");
    scenarioInProgress = false;
    return 0;
  }

  Step &step = scenario[currentScenarioStep];
  switch (step.type) {
    case STEP_FORWARD:
      moveAsyncSteps(getStepsForDistance(step.value), step.speed, true);
      break;
    case STEP_BACKWARD:
      moveAsyncSteps(getStepsForDistance(step.value), step.speed, false);
      break;
    case STEP_ROTATE:
      if (step.value >= 0)
        rotateAsync(step.value, step.speed, true);
      else
        rotateAsync(-step.value, step.speed, false);
      break;
  }
  currentScenarioStep++;

  return 1;
}

bool tirettePose = false;

void loop() {
  if (digitalRead(TIRETTE_PIN) == LOW && !tirettePose) {
    // Tirette posée
    tirettePose = true;
    Serial.println("Trigger initialized");
  } else if (digitalRead(TIRETTE_PIN) == HIGH && tirettePose) {
    // Tirette activée
    Serial.println("Trigger removed");
    // TODO : Mettre le bon delay selon le num
    uint32_t startDelay = START_DELAY + (PAMI_NUM - 1) * /*2000*/0;
    Serial.print("Waiting : ");
    Serial.println(startDelay);
    delay(startDelay);
    Serial.println("Starting the script");
    startTime = millis();

    scenarioInProgress = true;
    while (true) {
      updateSteppers(); // Mise à jour des moteurs asynchrone

      // detectObstacles(); // Vérification des obstacles

      // Gestion du scénario puis arrêt si terminé ou si temps dépassé
      if (processScenario() == 0 || millis() - startTime >= 100000 - startDelay) {
        Serial.println("Scénario terminé, arrêt des moteurs.");
        // TODO : Faire bouger le servo a la fin
        stopMotors();

        while (true) {
          if (!digitalRead(EMG_PIN)) {
            Serial.println("Servo run");
            setServoAngle(45);
            delay(500);
            setServoAngle(135);
            delay(500);
          } else {
            Serial.println("Restart");
            ESP.restart();
            break;
          }
        }
      }
    }
  }
}
