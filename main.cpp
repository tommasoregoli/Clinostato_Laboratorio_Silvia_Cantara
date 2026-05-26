#include <Arduino.h>
#include <TMCStepper.h>
#include <AccelStepper.h> // NUOVA LIBRERIA

#define R_SENSE 0.11f

// Pin Motore X
#define X_EN_PIN   12
#define X_STEP_PIN 11
#define X_DIR_PIN  10

// Pin Motore Y
#define Y_EN_PIN   7
#define Y_STEP_PIN 6
#define Y_DIR_PIN  5

TMC2209Stepper driverX(&Serial2, R_SENSE, 0);
TMC2209Stepper driverY(&Serial2, R_SENSE, 2);

// --- INIZIALIZZAZIONE ACCELSTEPPER ---
// Parametro 1 = DRIVER (significa che usiamo 1 pin per STEP e 1 per DIR)
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

// --- PARAMETRI DI COMPORTAMENTO DELLA RPM ---
// A 16 microstep, 3200 step = 1 giro. 1000 step/sec corrispondono a circa 18.7 RPM
// I motori muovono delle pulegge da 16 denti, quella dell'asse x è connessa direttamente ad una puleggia da 50 denti, quindi la massima velocità raggiungibile in RPM è
// 60/[(3200/290)*(50/16)] = 1,74 RPM. L'asse y invece è collegato ad una puleggia da 50 denti ma poi è mosso da una puleggia di 30 denti,
// quindi la massima velocità raggiungibile è 60/[(3200/290)*(50/16)*(30/50)] = 2,90 RPM

const float MAX_SPEED = 290.0;     // Velocità massima in step/secondo
const float MAX_ACCEL = 150.0;     // Accelerazione in step/sec^2 (più è basso, più è "gentile")
const long MIN_INTERVAL = 60000;    // Tempo minimo tra un cambio di target e l'altro (millisecondi)
const long MAX_INTERVAL = 180000;   // Tempo massimo tra un cambio di target e l'altro (millisecondi)

// --- VARIABILI PER IL CONTROLLO CASUALE ASINCRONO ---
float targetSpeedX = 0;
unsigned long lastChangeX = 0;
long nextIntervalX = 0;

float targetSpeedY = 0;
unsigned long lastChangeY = 0;
long nextIntervalY = 0;

// Timer per l'aggiornamento dell'accelerazione
unsigned long lastAccelUpdate = 0;

void setup() {
  Serial.begin(115200);

  // Inizializza il generatore di numeri casuali leggendo un pin analogico vuoto
  randomSeed(analogRead(A0));

  // Impostazione Pin Enable
  pinMode(X_EN_PIN, OUTPUT);
  pinMode(Y_EN_PIN, OUTPUT);
  digitalWrite(X_EN_PIN, HIGH); // Motori spenti
  digitalWrite(Y_EN_PIN, HIGH);

  // Inizializza UART hardware
  Serial2.setTX(8);
  Serial2.setRX(9);
  Serial2.begin(115200);

  driverX.begin();
  driverY.begin();

  // --- CONFIGURAZIONE TMC VIA UART ---
  Serial.println("Configuro Motori via UART...");
  driverX.toff(5);
  driverX.rms_current(800);
  driverX.microsteps(16);
  driverX.en_spreadCycle(false);
  driverX.pwm_autoscale(true);

  driverY.toff(5);
  driverY.rms_current(800);
  driverY.microsteps(16);
  driverY.en_spreadCycle(false);
  driverY.pwm_autoscale(true);

  // --- CONFIGURAZIONE ACCELSTEPPER ---
  // AccelStepper ha bisogno di conoscere la velocità e accelerazione massima
  stepperX.setMaxSpeed(MAX_SPEED);
  stepperY.setMaxSpeed(MAX_SPEED);
  // Non usiamo la funzione setAcceleration di AccelStepper perché useremo runSpeed()
  // e calcoleremo noi la rampa di velocità per avere un moto continuo senza fermate a posizioni fisse.

  // Abilita fisicamente i motori
  digitalWrite(X_EN_PIN, LOW);
  digitalWrite(Y_EN_PIN, LOW);

  Serial.println("Setup completato. Avvio simulazione microgravità!");
}

void loop() {
  unsigned long currentMillis = millis();

  // --- 1. GENERATORE DI TARGET CASUALI MOTORE X ---
  if (currentMillis - lastChangeX > nextIntervalX) {
    lastChangeX = currentMillis;
    nextIntervalX = random(MIN_INTERVAL, MAX_INTERVAL); // Prossimo cambio tra 1/3 minuti
    // Genera una velocità a caso tra -MAX_SPEED e +MAX_SPEED
    // Il segno negativo farà invertire automaticamente la direzione!
    targetSpeedX = random(-MAX_SPEED, MAX_SPEED);
  }

  // --- 2. GENERATORE DI TARGET CASUALI MOTORE Y (Completamente Asincrono) ---
  if (currentMillis - lastChangeY > nextIntervalY) {
    lastChangeY = currentMillis;
    nextIntervalY = random(MIN_INTERVAL, MAX_INTERVAL);
    targetSpeedY = random(-MAX_SPEED, MAX_SPEED);
  }

  // --- 3. GESTIONE DELL'ACCELERAZIONE "GENTILE" ---
  // Aggiorniamo la velocità corrente verso la velocità target 100 volte al secondo (ogni 10ms)
  if (currentMillis - lastAccelUpdate >= 10) {
    lastAccelUpdate = currentMillis;

    // Calcola l'incremento di velocità per questo step di tempo (0.01 secondi)
    float speedStep = MAX_ACCEL * 0.01;

    // Rampa per X
    float currentSpeedX = stepperX.speed();
    if (currentSpeedX < targetSpeedX) {
      currentSpeedX += speedStep;
      if (currentSpeedX > targetSpeedX) currentSpeedX = targetSpeedX;
    } else if (currentSpeedX > targetSpeedX) {
      currentSpeedX -= speedStep;
      if (currentSpeedX < targetSpeedX) currentSpeedX = targetSpeedX;
    }
    stepperX.setSpeed(currentSpeedX);

    // Rampa per Y
    float currentSpeedY = stepperY.speed();
    if (currentSpeedY < targetSpeedY) {
      currentSpeedY += speedStep;
      if (currentSpeedY > targetSpeedY) currentSpeedY = targetSpeedY;
    } else if (currentSpeedY > targetSpeedY) {
      currentSpeedY -= speedStep;
      if (currentSpeedY < targetSpeedY) currentSpeedY = targetSpeedY;
    }
    stepperY.setSpeed(currentSpeedY);
  }

  // --- 4. ESECUZIONE DEL PASSO ---
  // runSpeed() genera l'impulso SOLO se è passato il tempo necessario per la velocità corrente impostata.
  // Questa funzione deve girare il più velocemente possibile nel loop.
  stepperX.runSpeed();
  stepperY.runSpeed();
}
