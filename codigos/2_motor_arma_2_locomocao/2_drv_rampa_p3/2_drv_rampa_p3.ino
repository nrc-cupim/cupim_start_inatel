#include <Bluepad32.h>
#include "parametros.h"

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
bool roboLigado;

void desligaRobo() {
  digitalWrite(SENTIDO_ARMA1, LOW);
  analogWrite(VELOCIDADE_ARMA1, 0);

  digitalWrite(SENTIDO_ARMA2, LOW);
  analogWrite(VELOCIDADE_ARMA2, 0);

  digitalWrite(SENTIDO_MOTOR_ESQUERDO, LOW);
  analogWrite(VELOCIDADE_MOTOR_ESQUERDO, 0);

  digitalWrite(SENTIDO_MOTOR_DIREITO, LOW);
  analogWrite(VELOCIDADE_MOTOR_DIREITO, 0);

  roboLigado = false;
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

        // Lógica de funcionamento da rampa
        int32_t pressaoR2 = myController->throttle();
        int32_t pressaoL2 = myController->brake();
        int pwmArmas;

        if (pressaoL2 > toleranciaGatilhos) {
          digitalWrite(SENTIDO_ARMA1, HIGH);
          digitalWrite(SENTIDO_ARMA2, HIGH);
          pwmArmas = map(pressaoL2, minL2, maxL2, maxPWM, minPWM);
        } else if (pressaoR2 > toleranciaGatilhos) {
          digitalWrite(SENTIDO_ARMA1, LOW);
          digitalWrite(SENTIDO_ARMA2, LOW);
          pwmArmas = map(pressaoR2, minR2, maxR2, minPWM, maxPWM);
        } else {
          digitalWrite(SENTIDO_ARMA1, LOW);
          digitalWrite(SENTIDO_ARMA2, LOW);
          pwmArmas = 0;
        }

        analogWrite(VELOCIDADE_ARMA1, pwmArmas);
        analogWrite(VELOCIDADE_ARMA2, pwmArmas);

        // Lê valor em Y do analógico direito (R-right) e exibe valor na serial.
        int32_t valorAnalogicoDireito = -myController->axisRY();
        Serial.print("Y analogico R: ");
        Serial.println(valorAnalogicoDireito);

        // Lê valor em X do analógico esquerdo (L-left) e exibe valor na serial.
        int32_t valorAnalogicoEsquerdo = myController->axisX();
        Serial.print("X analogico L: ");
        Serial.println(valorAnalogicoEsquerdo);

        int pwmMotorDireito, pwmMotorEsquerdo;

        if (valorAnalogicoDireito > (centerAnalogR_Y + toleranciaAnalogico)) {

          digitalWrite(SENTIDO_MOTOR_DIREITO, HIGH);
          digitalWrite(SENTIDO_MOTOR_ESQUERDO, HIGH);

          if (valorAnalogicoEsquerdo > (centerAnalogL_X + toleranciaAnalogico)) {

            pwmMotorDireito = map(valorAnalogicoDireito - valorAnalogicoEsquerdo,
                                  centerAnalogR_Y - maxAnalogL_X,
                                  maxAnalogR_Y - centerAnalogL_X,
                                  maxPWM,
                                  minPWM);

            pwmMotorEsquerdo = map(valorAnalogicoDireito + valorAnalogicoEsquerdo,
                                   centerAnalogR_Y + centerAnalogL_X,
                                   maxAnalogR_Y + maxAnalogL_X,
                                   maxPWM,
                                   minPWM);
          }

          else if (valorAnalogicoEsquerdo < (centerAnalogL_X - toleranciaAnalogico)) {

            pwmMotorDireito = map(valorAnalogicoDireito - valorAnalogicoEsquerdo,
                                  centerAnalogR_Y - centerAnalogL_X,
                                  maxAnalogR_Y - minAnalogL_X,
                                  maxPWM,
                                  minPWM);

            pwmMotorEsquerdo = map(valorAnalogicoDireito + valorAnalogicoEsquerdo,
                                   centerAnalogR_Y + minAnalogL_X,
                                   maxAnalogR_Y + centerAnalogL_X,
                                   maxPWM,
                                   minPWM);
          }

          else {
            pwmMotorDireito = map(valorAnalogicoDireito, centerAnalogR_Y, maxAnalogR_Y, maxPWM, minPWM);
            pwmMotorEsquerdo = map(valorAnalogicoDireito, centerAnalogR_Y, maxAnalogR_Y, maxPWM, minPWM);
          }
        }

        else if (valorAnalogicoDireito < (centerAnalogR_Y - toleranciaAnalogico)) {

          digitalWrite(SENTIDO_MOTOR_DIREITO, LOW);
          digitalWrite(SENTIDO_MOTOR_ESQUERDO, LOW);

          if (valorAnalogicoEsquerdo > (centerAnalogL_X + toleranciaAnalogico)) {
            pwmMotorDireito = map(valorAnalogicoEsquerdo + valorAnalogicoDireito,
                                  centerAnalogL_X + minAnalogL_X,
                                  maxAnalogL_X + centerAnalogR_Y,
                                  minPWM,
                                  maxPWM);

            pwmMotorEsquerdo = map(valorAnalogicoEsquerdo - valorAnalogicoDireito,
                                   centerAnalogL_X - centerAnalogR_Y,
                                   maxAnalogL_X - minAnalogR_Y,
                                   minPWM,
                                   maxPWM);
          }

          else if (valorAnalogicoEsquerdo < (centerAnalogL_X - toleranciaAnalogico)) {
            pwmMotorDireito = map(valorAnalogicoEsquerdo + valorAnalogicoDireito,
                                  centerAnalogL_X + centerAnalogR_Y,
                                  minAnalogL_X + minAnalogR_Y,
                                  minPWM,
                                  maxPWM);

            pwmMotorEsquerdo = map(valorAnalogicoEsquerdo - valorAnalogicoDireito,
                                   minAnalogL_X - centerAnalogR_Y,
                                   centerAnalogL_X - minAnalogR_Y,
                                   minPWM,
                                   maxPWM);
          }

          else {
            pwmMotorDireito = map(valorAnalogicoDireito, centerAnalogR_Y, minAnalogR_Y, minPWM, maxPWM);
            pwmMotorEsquerdo = map(valorAnalogicoDireito, centerAnalogR_Y, minAnalogR_Y, minPWM, maxPWM);
          }
        }

        else {

          if (valorAnalogicoEsquerdo > (centerAnalogL_X + toleranciaAnalogico)) {
            digitalWrite(SENTIDO_MOTOR_DIREITO, LOW);
            digitalWrite(SENTIDO_MOTOR_ESQUERDO, HIGH);
            pwmMotorDireito = map(valorAnalogicoEsquerdo, centerAnalogL_X, maxAnalogL_X, minPWM, maxPWM);
            pwmMotorEsquerdo = map(valorAnalogicoEsquerdo, centerAnalogL_X, maxAnalogL_X, maxPWM, minPWM);
          }

          else if (valorAnalogicoEsquerdo < (centerAnalogL_X - toleranciaAnalogico)) {
            digitalWrite(SENTIDO_MOTOR_DIREITO, HIGH);
            digitalWrite(SENTIDO_MOTOR_ESQUERDO, LOW);
            pwmMotorDireito = map(valorAnalogicoEsquerdo, centerAnalogL_X, minAnalogL_X, maxPWM, minPWM);
            pwmMotorEsquerdo = map(valorAnalogicoEsquerdo, centerAnalogL_X, minAnalogL_X, minPWM, maxPWM);
          }

          else {
            digitalWrite(SENTIDO_MOTOR_ESQUERDO, LOW);
            digitalWrite(SENTIDO_MOTOR_DIREITO, LOW);
            pwmMotorDireito = 0;
            pwmMotorEsquerdo = 0;
          }
        }

        Serial.print("PWM Direito: ");
        Serial.println(pwmMotorDireito);
        Serial.print("PWM Esquerdo: ");
        Serial.println(pwmMotorEsquerdo);

        analogWrite(VELOCIDADE_MOTOR_DIREITO, pwmMotorDireito);
        analogWrite(VELOCIDADE_MOTOR_ESQUERDO, pwmMotorEsquerdo);
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
  pinMode(SENTIDO_ARMA1, OUTPUT);
  pinMode(VELOCIDADE_ARMA1, OUTPUT);

  pinMode(SENTIDO_ARMA2, OUTPUT);
  pinMode(VELOCIDADE_ARMA2, OUTPUT);

  // Configura pinos da ESP32 para controle dos motores de locomoção.
  pinMode(SENTIDO_MOTOR_ESQUERDO, OUTPUT);
  pinMode(VELOCIDADE_MOTOR_ESQUERDO, OUTPUT);

  pinMode(SENTIDO_MOTOR_DIREITO, OUTPUT);
  pinMode(VELOCIDADE_MOTOR_DIREITO, OUTPUT);

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