#include <Servo.h>

#define N_SERVOS 18       // Número total de servos
#define FREQB    0.0      // Desfase 1 en grados
#define FREQY    20.0     // Desfase 2 en grados
#define FREQG    40.0     // Desfase 3 en grados
#define D        false   // Motor invertido o no
#define I        true   // Motor invertido o no

const int    STEPS         = 720;   // Estados discretos
const float  FREQUENCY     = 0.2f;  // Hz
const float  OMEGA         = 2.0f * PI * FREQUENCY;

// Parámetros de ruido sobre la fase (inicialmente desactivado)
float noisePhaseAmpDeg = 0.0f;     // Amplitud de ruido en fase (grados)
float noisePhaseAmpRad = 0.0f;     // Amplitud de ruido convertida a radianes
bool  noisePhaseEnabled = false;   // Si true, añadimos ruido a la fase

// Parámetros de ruido sobre la amplitud (inicialmente desactivado)
float noiseAmpAmpDeg = 0.0f;       // Amplitud de ruido en amplitud (grados)
bool  noiseAmpEnabled = false;     // Si true, añadimos ruido a la amplitud

struct ServoConfig {
  uint8_t pin;           // Pin de control (PWM)
  float   phaseDeg;      // Desfase en grados
  float   amplitude;     // Amplitud de oscilación (grados)
  bool    inverted;      // Si true, invierte el seno
  float   center;        // Ángulo central (grados)
};

// ----------------------------------------------
// 1) Definición del array de configuraciones
ServoConfig configs[N_SERVOS] = {
  // pin, phaseDeg, amplitude, inverted, center
  {  12,   FREQY,     20.0f,    D,    90.0f },   // Servo 15
  {  11,   FREQB,     20.0f,    I,    90.0f },   // Servo 16
  {  10,   FREQG,     20.0f,    D,    90.0f },   // Servo 17
  {   8,   FREQB,     20.0f,    I,    90.0f },   // Servo 14
  {   7,   FREQY,     20.0f,    I,    90.0f },   // Servo 18
  {   6,   FREQB,     20.0f,    I,    90.0f },   // Servo 13
  {  53,   FREQB,     20.0f,    I,    90.0f },   // Servo 6
  {  51,   FREQG,     20.0f,    I,    90.0f },   // Servo 9
  {   3,   FREQB,     20.0f,    I,    90.0f },   // Servo 11
  {  45,   FREQB,     20.0f,    D,    90.0f },   // Servo 1
  {  43,   FREQG,     20.0f,    D,    90.0f },   // Servo 12
  {  41,   FREQY,     20.0f,    I,    90.0f },   // Servo 10
  {  39,   FREQY,     20.0f,    I,    90.0f },   // Servo 8
  {  35,   FREQB,     20.0f,    I,    90.0f },   // Servo 7
  {  33,   FREQG,     20.0f,    I,    90.0f },   // Servo 3
  {  31,   FREQY,     20.0f,    D,    90.0f },   // Servo 5
  {  25,   FREQB,     20.0f,    D,    90.0f },   // Servo 4
  {  27,   FREQY,     20.0f,    I,    90.0f }    // Servo 2
};

// 2) Array paralelo de objetos Servo
Servo servos[N_SERVOS];

bool isRotating = false;

void setup() {
  Serial.begin(9600);
  // Semilla para la aleatoriedad (ruido)
  randomSeed(analogRead(A0));

  for (int i = 0; i < N_SERVOS; i++) {
    servos[i].attach(configs[i].pin);
  }
  printHelp();
}

void loop() {
  if (isRotating) {
    // MODO ROTACIÓN CONTINUA
    float t = millis() / 1000.0f;

    for (int i = 0; i < N_SERVOS; i++) {
      // 1) Convertimos desfase en grados a radianes
      float phaseRad = configs[i].phaseDeg * (PI / 180.0f);

      // 2) Le añadimos ruido aleatorio a la fase (si está activado)
      float noisyPhase = OMEGA * t + phaseRad;
      if (noisePhaseEnabled) {
        // Valor aleatorio entre -1 y +1
        float rPhase = ((float)random(-1000, 1001)) / 1000.0f;
        noisyPhase += rPhase * noisePhaseAmpRad;
      }

      // 3) Calculamos seno con la fase (ruidosa o sin ruido)
      float base = sin(noisyPhase);

      // 4) Si está invertido, multiplicamos por -1
      if (configs[i].inverted) base = -base;

      // 5) Calculamos la amplitud efectiva (ruido sobre amplitud)
      float effectiveAmp = configs[i].amplitude;
      if (noiseAmpEnabled) {
        // Valor aleatorio entre -1 y +1
        float rAmp = ((float)random(-1000, 1001)) / 1000.0f;
        effectiveAmp += rAmp * noiseAmpAmpDeg;
      }

      // 6) Ángulo final y envío al servo
      float angle = configs[i].center + effectiveAmp * base;
      angle = constrain(angle, 0.0f, 180.0f);
      servos[i].write(angle);
    }

    // Revisar comandos de Serial en modo rotación:
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd.equalsIgnoreCase("S")) {
        isRotating = false;
        Serial.println("Rotación detenida.");
        printHelp();
      }
      else if (cmd.startsWith("Z")) {
        // Comando para ajustar ruido de fase: "Z <amplitud_en_grados>"
        parseNoisePhaseCommand(cmd);
      }
      else if (cmd.startsWith("Y")) {
        // Comando para ajustar ruido de amplitud: "Y <amplitud_en_grados>"
        parseNoiseAmpCommand(cmd);
      }
    }

    delay(20); // ~50 Hz de refresco
    return;
  }

  // MODO INTERACTIVO - ÍNDICE DISCRETO O COMANDOS
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.equalsIgnoreCase("E")) {
    reportExtremesAndNeutral();
  }
  else if (cmd.equalsIgnoreCase("R")) {
    isRotating = true;
    Serial.println("Iniciando rotación continua. Envía 'S' para parar.");
    Serial.println("Para ajustar ruido de fase: 'Z <amplitud_en_grados>'.");
    Serial.println("Para ajustar ruido de amplitud: 'Y <amplitud_en_grados>'.");
  }
  else if (cmd.startsWith("Z")) {
    // Comando para ajustar ruido de fase mientras NO está rotando
    parseNoisePhaseCommand(cmd);
  }
  else if (cmd.startsWith("Y")) {
    // Comando para ajustar ruido de amplitud mientras NO está rotando
    parseNoiseAmpCommand(cmd);
  }
  else {
    // Comando numérico: índice discreto [0..STEPS-1]
    int idx = cmd.toInt();
    idx = constrain(idx, 0, STEPS - 1);
    float phase = 2.0f * PI * idx / STEPS; // fase base en radianes

    for (int i = 0; i < N_SERVOS; i++) {
      // 1) Convertimos desfase en grados a radianes
      float phaseRad = configs[i].phaseDeg * (PI / 180.0f);

      // 2) Le añadimos ruido de fase (si está activado)
      float noisyPhase = phase + phaseRad;
      if (noisePhaseEnabled) {
        float rPhase = ((float)random(-1000, 1001)) / 1000.0f;
        noisyPhase += rPhase * noisePhaseAmpRad;
      }

      // 3) Calculamos seno con fase (ruidosa o sin ruido)
      float base = sin(noisyPhase);
      if (configs[i].inverted) base = -base;

      // 4) Calculamos la amplitud efectiva (ruido sobre amplitud)
      float effectiveAmp = configs[i].amplitude;
      if (noiseAmpEnabled) {
        float rAmp = ((float)random(-1000, 1001)) / 1000.0f;
        effectiveAmp += rAmp * noiseAmpAmpDeg;
      }

      // 5) Ángulo final
      float angle = configs[i].center + effectiveAmp * base;
      angle = constrain(angle, 0.0f, 180.0f);
      servos[i].write(angle);

      // Para no saturar la Serial, imprimimos sólo los 3 primeros servos
      if (i < 3) {
        Serial.print("Servo ");
        Serial.print(i);
        Serial.print(" -> ");
        Serial.print(angle, 1);
        if (i < 2) Serial.print(" | ");
        else Serial.println();
      }
    }
  }
}

// ------------------------------------------------------------------
// Función que parsea el comando de ruido de fase “Z <amplitud_en_grados>”
void parseNoisePhaseCommand(const String &cmd) {
  // Sintaxis esperada: "Z 5.0" o "Z 0" para desactivar
  float nuevaAmp = 0.0f;
  bool ok = false;

  int spacePos = cmd.indexOf(' ');
  if (spacePos > 0) {
    String valStr = cmd.substring(spacePos + 1);
    valStr.trim();
    nuevaAmp = valStr.toFloat();
    ok = true;
  }

  if (!ok) {
    Serial.println("Error al parsear comando Z. Sintaxis: Z <amplitud_en_grados>");
    return;
  }

  noisePhaseAmpDeg = fabs(nuevaAmp);
  noisePhaseAmpRad = noisePhaseAmpDeg * (PI / 180.0f);
  noisePhaseEnabled = (noisePhaseAmpDeg > 0.0f);

  if (noisePhaseEnabled) {
    Serial.print("Ruido de fase activado con amplitud = ");
    Serial.print(noisePhaseAmpDeg, 1);
    Serial.println(" grados.");
  } else {
    Serial.println("Ruido de fase desactivado.");
  }
}

// ------------------------------------------------------------------
// Función que parsea el comando de ruido de amplitud “Y <amplitud_en_grados>”
void parseNoiseAmpCommand(const String &cmd) {
  // Sintaxis esperada: "Y 5.0" o "Y 0" para desactivar
  float nuevaAmp = 0.0f;
  bool ok = false;

  int spacePos = cmd.indexOf(' ');
  if (spacePos > 0) {
    String valStr = cmd.substring(spacePos + 1);
    valStr.trim();
    nuevaAmp = valStr.toFloat();
    ok = true;
  }

  if (!ok) {
    Serial.println("Error al parsear comando Y. Sintaxis: Y <amplitud_en_grados>");
    return;
  }

  noiseAmpAmpDeg = fabs(nuevaAmp);
  noiseAmpEnabled = (noiseAmpAmpDeg > 0.0f);

  if (noiseAmpEnabled) {
    Serial.print("Ruido de amplitud activado con amplitud = ");
    Serial.print(noiseAmpAmpDeg, 1);
    Serial.println(" grados.");
  } else {
    Serial.println("Ruido de amplitud desactivado.");
  }
}

// ------------------------------------------------------------------
// Recorre todos los pasos para cada servo y calcula mínimos, máximos y neutro
void reportExtremesAndNeutral() {
  float minVal[N_SERVOS], maxVal[N_SERVOS];
  int   minIdx[N_SERVOS], maxIdx[N_SERVOS];
  for (int i = 0; i < N_SERVOS; i++) {
    minVal[i] =  1e6;
    maxVal[i] = -1e6;
    minIdx[i] = maxIdx[i] = 0;
  }

  float bestDev = 1e6;
  int   neutralIdx = 0;

  for (int step = 0; step < STEPS; step++) {
    float phase = 2.0f * PI * step / STEPS;
    float sumDev = 0.0f;

    for (int i = 0; i < N_SERVOS; i++) {
      float phaseRad = configs[i].phaseDeg * (PI / 180.0f);
      float basePhase = phase + phaseRad;

      // En modo “extremos” NO añadimos ruido: queremos los rangos teóricos
      float base = sin(basePhase);
      if (configs[i].inverted) base = -base;
      float ang = configs[i].center + configs[i].amplitude * base;
      ang = constrain(ang, 0.0f, 180.0f);

      if (ang < minVal[i]) { minVal[i] = ang; minIdx[i] = step; }
      if (ang > maxVal[i]) { maxVal[i] = ang; maxIdx[i] = step; }
      sumDev += fabs(ang - configs[i].center);
    }

    if (sumDev < bestDev) {
      bestDev   = sumDev;
      neutralIdx = step;
    }
  }

  Serial.println();
  Serial.println("=== Extremos y Pose Neutra (sin ruido) ===");
  for (int i = 0; i < N_SERVOS; i++) {
    Serial.print("Servo ");
    Serial.print(i);
    Serial.print(": min=");
    Serial.print(minVal[i], 1);
    Serial.print(" (idx=");
    Serial.print(minIdx[i]);
    Serial.print(")  max=");
    Serial.print(maxVal[i], 1);
    Serial.print(" (idx=");
    Serial.print(maxIdx[i]);
    Serial.println(")");
  }
  Serial.print("Índice neutro global = ");
  Serial.println(neutralIdx);

  // Mostrar ángulos neutros de los primeros 3:
  float phaseN = 2.0f * PI * neutralIdx / STEPS;
  Serial.print("Ángulos neutros (servo 0..2): ");
  for (int i = 0; i < min(3, N_SERVOS); i++) {
    float phaseRad = configs[i].phaseDeg * (PI / 180.0f);
    float base = sin(phaseN + phaseRad);
    if (configs[i].inverted) base = -base;
    float ang = configs[i].center + configs[i].amplitude * base;
    ang = constrain(ang, 0.0f, 180.0f);
    Serial.print(ang, 1);
    if (i < 2) Serial.print(", ");
    else Serial.println();
  }
  Serial.println("===============================================");
  Serial.println();
}

void printHelp() {
  Serial.println();
  Serial.print("Envía 0–");
  Serial.print(STEPS - 1);
  Serial.println("  → para mover al estado discreto.");
  Serial.println("Envía 'E'   → para informar extremos y pose neutra.");
  Serial.println("Envía 'R'   → para iniciar rotación continua.");
  Serial.println("  (En rotación, envía 'S' para parar.)");
  Serial.println("Envía 'Z X' → para ajustar ruido de fase (X = amplitud en grados).");
  Serial.println("  Ejemplo: 'Z 5.0' activa ruido de fase ±5º. 'Z 0' lo desactiva.");
  Serial.println("Envía 'Y X' → para ajustar ruido de amplitud (X = amplitud en grados).");
  Serial.println("  Ejemplo: 'Y 5.0' activa ruido de amplitud ±5º. 'Y 0' lo desactiva.");
  Serial.println();
}
