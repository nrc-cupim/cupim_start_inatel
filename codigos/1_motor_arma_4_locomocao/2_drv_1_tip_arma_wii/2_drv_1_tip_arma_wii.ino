#include <Bluepad32.h>
#include "parametros.h"

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
bool roboLigado, armaLigada;

void desligaRobo() {
  digitalWrite(SENTIDO_MOTOR_ESQUERDO1, LOW);
  analogWrite(VELOCIDADE_MOTOR_ESQUERDO1, 0);

  digitalWrite(SENTIDO_MOTOR_ESQUERDO2, LOW);
  analogWrite(VELOCIDADE_MOTOR_ESQUERDO2, 0);

  digitalWrite(SENTIDO_MOTOR_DIREITO1, LOW);
  analogWrite(VELOCIDADE_MOTOR_DIREITO1, 0);

  digitalWrite(SENTIDO_MOTOR_DIREITO2, LOW);
  analogWrite(VELOCIDADE_MOTOR_DIREITO2, 0);

  digitalWrite(PINO_ARMA, LOW);

  roboLigado = false, armaLigada = false;
}

void onConnectedController(ControllerPtr ctl) {

  bool foundEmptySlot = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.println("AVISO: controle conectado.");
      myControllers[i] = ctl;
      foundEmptySlot = true;

      /* Caso deseje realizar alguma tarefa assim que a conexão 
         com o contole for estabelecidada, coloque o código aqui. */

      break;
    }
  }

  if (!foundEmptySlot) {
    Serial.println("AVISO: Nao foi possivel conectar o controle.");
    Serial.println("AVISO: Reinicie a ESP32 e tente novamente.");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("AVISO: controle desconectado");
      myControllers[i] = nullptr;
      desligaRobo();

      /* Caso deseje realizar alguma tarefa assim que o 
         controle for desconectado, coloque o código aqui */

      break;
    }
  }
}

void processControllers() {
  for (auto myController : myControllers) {

    if (myController && myController->isConnected()
        && myController->hasData() && myController->isGamepad()) {

      /* A partir daqui inicia-se a lógica de funcionamento do robô.
         Qualquer alteração / nova implementação deve ser feita aqui. */

      uint16_t estadoMiscButtons = myController->miscButtons();

      // Se SELECT for presionado, desliga robô.
      if (estadoMiscButtons == 0x02) {
        roboLigado = false;
        Serial.println("Robo desligado");
      }

      // Se START for presionado, liga robô.
      else if (estadoMiscButtons == 0x04) {
        roboLigado = true;
        Serial.println("Robo iniciado");
      }

      if (roboLigado) {

        uint16_t botoesPressionados = myController->buttons();

        // B liga e desliga a arma
        if (botoesPressionados == 0x0001)
          armaLigada = !armaLigada;
        digitalWrite(PINO_ARMA, armaLigada);

        // Lê valor em Y do analógico direito (R-right).
        int32_t valorAnalogicoDireito = -myController->axisRY();

        // Lê valor em X do analógico esquerdo (L-left).
        int32_t valorAnalogicoEsquerdo = myController->axisRX();

        // Exibe valores no monitor serial.

        Serial.print("Y analogico R: ");
        Serial.println(valorAnalogicoDireito);

        Serial.print("X analogico L: ");
        Serial.println(valorAnalogicoEsquerdo);
        int pwmMotorDireito, pwmMotorEsquerdo;

        if (valorAnalogicoDireito > (centerAnalogR_Y + toleranciaAnalogico)) {

          digitalWrite(SENTIDO_MOTOR_DIREITO1, HIGH);
          digitalWrite(SENTIDO_MOTOR_ESQUERDO1, HIGH);
          digitalWrite(SENTIDO_MOTOR_DIREITO2, HIGH);
          digitalWrite(SENTIDO_MOTOR_ESQUERDO2, HIGH);

          if (valorAnalogicoEsquerdo > (centerAnalogR_X + toleranciaAnalogico)) {

            pwmMotorDireito = map(valorAnalogicoDireito - valorAnalogicoEsquerdo,
                                  centerAnalogR_Y - maxAnalogR_X,
                                  maxAnalogR_Y - centerAnalogR_X,
                                  maxPWM,
                                  minPWM);

            pwmMotorEsquerdo = map(valorAnalogicoDireito + valorAnalogicoEsquerdo,
                                   centerAnalogR_Y + centerAnalogR_X,
                                   maxAnalogR_Y + maxAnalogR_X,
                                   maxPWM,
                                   minPWM);
          }

          else if (valorAnalogicoEsquerdo < (centerAnalogR_X - toleranciaAnalogico)) {

            pwmMotorDireito = map(valorAnalogicoDireito - valorAnalogicoEsquerdo,
                                  centerAnalogR_Y - centerAnalogR_X,
                                  maxAnalogR_Y - minAnalogR_X,
                                  maxPWM,
                                  minPWM);

            pwmMotorEsquerdo = map(valorAnalogicoDireito + valorAnalogicoEsquerdo,
                                   centerAnalogR_Y + minAnalogR_X,
                                   maxAnalogR_Y + centerAnalogR_X,
                                   maxPWM,
                                   minPWM);
          }

          else {
            pwmMotorDireito = map(valorAnalogicoDireito, centerAnalogR_Y, maxAnalogR_Y, maxPWM, minPWM);
            pwmMotorEsquerdo = map(valorAnalogicoDireito, centerAnalogR_Y, maxAnalogR_Y, maxPWM, minPWM);
          }
        }

        else if (valorAnalogicoDireito < (centerAnalogR_Y - toleranciaAnalogico)) {

          digitalWrite(SENTIDO_MOTOR_DIREITO1, LOW);
          digitalWrite(SENTIDO_MOTOR_ESQUERDO1, LOW);
          digitalWrite(SENTIDO_MOTOR_DIREITO2, LOW);
          digitalWrite(SENTIDO_MOTOR_ESQUERDO2, LOW);

          if (valorAnalogicoEsquerdo > (centerAnalogR_X + toleranciaAnalogico)) {
            pwmMotorDireito = map(valorAnalogicoEsquerdo + valorAnalogicoDireito,
                                  centerAnalogR_X + minAnalogR_X,
                                  maxAnalogR_X + centerAnalogR_Y,
                                  minPWM,
                                  maxPWM);

            pwmMotorEsquerdo = map(valorAnalogicoEsquerdo - valorAnalogicoDireito,
                                   centerAnalogR_X - centerAnalogR_Y,
                                   maxAnalogR_X - minAnalogR_Y,
                                   minPWM,
                                   maxPWM);
          }

          else if (valorAnalogicoEsquerdo < (centerAnalogR_X - toleranciaAnalogico)) {
            pwmMotorDireito = map(valorAnalogicoEsquerdo + valorAnalogicoDireito,
                                  centerAnalogR_X + centerAnalogR_Y,
                                  minAnalogR_X + minAnalogR_Y,
                                  minPWM,
                                  maxPWM);

            pwmMotorEsquerdo = map(valorAnalogicoEsquerdo - valorAnalogicoDireito,
                                   minAnalogR_X - centerAnalogR_Y,
                                   centerAnalogR_X - minAnalogR_Y,
                                   minPWM,
                                   maxPWM);
          }

          else {
            pwmMotorDireito = map(valorAnalogicoDireito, centerAnalogR_Y, minAnalogR_Y, minPWM, maxPWM);
            pwmMotorEsquerdo = map(valorAnalogicoDireito, centerAnalogR_Y, minAnalogR_Y, minPWM, maxPWM);
          }
        }

        else {

          if (valorAnalogicoEsquerdo > (centerAnalogR_X + toleranciaAnalogico)) {
            digitalWrite(SENTIDO_MOTOR_DIREITO1, LOW);
            digitalWrite(SENTIDO_MOTOR_ESQUERDO1, HIGH);
            digitalWrite(SENTIDO_MOTOR_DIREITO2, LOW);
            digitalWrite(SENTIDO_MOTOR_ESQUERDO2, HIGH);
            pwmMotorDireito = map(valorAnalogicoEsquerdo, centerAnalogR_X, maxAnalogR_X, minPWM, maxPWM);
            pwmMotorEsquerdo = map(valorAnalogicoEsquerdo, centerAnalogR_X, maxAnalogR_X, maxPWM, minPWM);
          }

          else if (valorAnalogicoEsquerdo < (centerAnalogR_X - toleranciaAnalogico)) {
            digitalWrite(SENTIDO_MOTOR_DIREITO1, HIGH);
            digitalWrite(SENTIDO_MOTOR_ESQUERDO1, LOW);
            digitalWrite(SENTIDO_MOTOR_DIREITO2, HIGH);
            digitalWrite(SENTIDO_MOTOR_ESQUERDO2, LOW);
            pwmMotorDireito = map(valorAnalogicoEsquerdo, centerAnalogR_X, minAnalogR_X, maxPWM, minPWM);
            pwmMotorEsquerdo = map(valorAnalogicoEsquerdo, centerAnalogR_X, minAnalogR_X, minPWM, maxPWM);
          }

          else {
            digitalWrite(SENTIDO_MOTOR_ESQUERDO1, LOW);
            digitalWrite(SENTIDO_MOTOR_DIREITO1, LOW);
            digitalWrite(SENTIDO_MOTOR_ESQUERDO2, LOW);
            digitalWrite(SENTIDO_MOTOR_DIREITO2, LOW);
            pwmMotorDireito = 0;
            pwmMotorEsquerdo = 0;
          }
        }

        Serial.print("PWM Direito: ");
        Serial.println(pwmMotorDireito);
        Serial.print("PWM Esquerdo: ");
        Serial.println(pwmMotorEsquerdo);

        if (pwmMotorDireito > maxPWM)
          pwmMotorDireito = maxPWM;
        else if (pwmMotorDireito < minPWM)
          pwmMotorDireito = minPWM;

        if (pwmMotorEsquerdo > maxPWM)
          pwmMotorEsquerdo = maxPWM;
        else if (pwmMotorEsquerdo < minPWM)
          pwmMotorEsquerdo = minPWM;

        analogWrite(VELOCIDADE_MOTOR_DIREITO1, pwmMotorDireito);
        analogWrite(VELOCIDADE_MOTOR_ESQUERDO1, pwmMotorEsquerdo);
        analogWrite(VELOCIDADE_MOTOR_DIREITO2, pwmMotorDireito);
        analogWrite(VELOCIDADE_MOTOR_ESQUERDO2, pwmMotorEsquerdo);
      }

      else
        desligaRobo();
    }
  }
}

void setup() {

  Serial.begin(115200);

  // Inicia comunicação Bluetooth que permite a conexão com controle.
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);

  // Desparea os controles que haviam sido conectados anteriormente.
  BP32.forgetBluetoothKeys();

  // Configura pinos da ESP32 para controle dos motores de arma.
  pinMode(SENTIDO_MOTOR_DIREITO2, OUTPUT);
  pinMode(VELOCIDADE_MOTOR_DIREITO2, OUTPUT);

  pinMode(SENTIDO_MOTOR_ESQUERDO2, OUTPUT);
  pinMode(SENTIDO_MOTOR_ESQUERDO2, OUTPUT);

  // Configura pinos da ESP32 para controle dos motores de locomoção.
  pinMode(SENTIDO_MOTOR_ESQUERDO1, OUTPUT);
  pinMode(VELOCIDADE_MOTOR_ESQUERDO1, OUTPUT);

  pinMode(SENTIDO_MOTOR_DIREITO1, OUTPUT);
  pinMode(VELOCIDADE_MOTOR_DIREITO1, OUTPUT);

  pinMode(PINO_ARMA, OUTPUT);

  // Desliga movimentação e arma do robô.
  desligaRobo();
}

void loop() {
  // Checa se houve atualização nos dados do controle
  bool dataUpdated = BP32.update();

  // Se sim, chama a função processControllers() para processar os dados
  if (dataUpdated)
    processControllers();
}