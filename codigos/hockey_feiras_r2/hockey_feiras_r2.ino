#include <Bluepad32.h>
#include "parametros.h"

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
bool roboLigado;

/* Foi necessário utilizar essas variáveis para permitir a 
   inversão do sentido de giro de cada motor de locomoção */

int sentidoMotorEsquerdo, velocidadeMotorEsquerdo;
int sentidoMotorDireito, velocidadeMotorDireito;

bool direitoVesquerdoH, direitoHesquerdoV;

void desligaRobo() {
  analogWrite(sentidoMotorDireito, 0);
  analogWrite(velocidadeMotorDireito, 0);

  analogWrite(sentidoMotorEsquerdo, 0);
  analogWrite(velocidadeMotorEsquerdo, 0);

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
        Serial.println("Robo desligado.");
      }

      // Se START for presionado, liga robô.
      else if (estadoMiscButtons == 0x04) {
        roboLigado = true;
        Serial.println("Robo ligado.");
      }

      if (roboLigado) {

        /* ----------------- Lógica de inversão dos analógicos de movimentação ----------------- */

        uint16_t r1_l1 = myController->buttons() & 0x00F0;  // Lógica E bit a bit pra isolar primeiro segundo

        // Se L1 for pressionado, locomoção Direito V - Esquerdo H
        if (r1_l1 == 0x0010) {
          direitoVesquerdoH = true, direitoHesquerdoV = false;
        }

        // Se R1 for pressionado, locomoção Direito H - Esquerdo V
        else if (r1_l1 == 0x0020) {
          direitoVesquerdoH = false, direitoHesquerdoV = true;
        }

        /* ----------------- Lógica de funcionamento da movimentação ----------------- */

        int32_t valorAnalogicoV, valorAnalogicoH;

        if (direitoVesquerdoH) {
          // Lê valor em Y do analógico direito (R-right).
          valorAnalogicoV = -myController->axisRY();

          // Lê valor em X do analógico esquerdo (L-left).
          valorAnalogicoH = myController->axisX();
        }

        if (direitoHesquerdoV) {
          // Lê valor em X do analógico direito (R-right).
          valorAnalogicoH = myController->axisRX();

          // Lê valor em Y do analógico esquerdo (L-left).
          valorAnalogicoV = -myController->axisY();
        }

        // Exibe valores no monitor serial.
        Serial.print("Y analogico: ");
        Serial.println(valorAnalogicoV);

        Serial.print("X analogico: ");
        Serial.println(valorAnalogicoH);

        int pwmMotorDireito1, pwmMotorDireito2, pwmMotorEsquerdo1, pwmMotorEsquerdo2;

        if (valorAnalogicoV > (CENTER_ANALOG_Y + TOLERANCIA_ANALOGICO)) {

          pwmMotorDireito1 = MAX_PWM;
          pwmMotorEsquerdo1 = MAX_PWM;

          if (valorAnalogicoH > (CENTER_ANALOG_X + TOLERANCIA_ANALOGICO)) {

            pwmMotorDireito2 = map(valorAnalogicoV - valorAnalogicoH,
                                   CENTER_ANALOG_Y - MAX_ANALOG_X,
                                   MAX_ANALOG_Y - CENTER_ANALOG_X,
                                   MAX_PWM,
                                   MIN_PWM);

            pwmMotorEsquerdo2 = map(valorAnalogicoV + valorAnalogicoH,
                                    CENTER_ANALOG_Y + CENTER_ANALOG_X,
                                    MAX_ANALOG_Y + MAX_ANALOG_X,
                                    MAX_PWM,
                                    MIN_PWM);
          }

          else if (valorAnalogicoH < (CENTER_ANALOG_X - TOLERANCIA_ANALOGICO)) {

            pwmMotorDireito2 = map(valorAnalogicoV - valorAnalogicoH,
                                   CENTER_ANALOG_Y - CENTER_ANALOG_X,
                                   MAX_ANALOG_Y - MIN_ANALOG_X,
                                   MAX_PWM,
                                   MIN_PWM);

            pwmMotorEsquerdo2 = map(valorAnalogicoV + valorAnalogicoH,
                                    CENTER_ANALOG_Y + MIN_ANALOG_X,
                                    MAX_ANALOG_Y + CENTER_ANALOG_X,
                                    MAX_PWM,
                                    MIN_PWM);
          }

          else {
            pwmMotorDireito2 = map(valorAnalogicoV, CENTER_ANALOG_Y, MAX_ANALOG_Y, MAX_PWM, MIN_PWM);
            pwmMotorEsquerdo2 = map(valorAnalogicoV, CENTER_ANALOG_Y, MAX_ANALOG_Y, MAX_PWM, MIN_PWM);
          }
        }

        else if (valorAnalogicoV < (CENTER_ANALOG_Y - TOLERANCIA_ANALOGICO)) {

          pwmMotorDireito1 = MIN_PWM;
          pwmMotorEsquerdo1 = MIN_PWM;

          if (valorAnalogicoH > (CENTER_ANALOG_X + TOLERANCIA_ANALOGICO)) {
            pwmMotorDireito2 = map(valorAnalogicoH + valorAnalogicoV,
                                   CENTER_ANALOG_X + MIN_ANALOG_X,
                                   MAX_ANALOG_X + CENTER_ANALOG_Y,
                                   MIN_PWM,
                                   MAX_PWM);

            pwmMotorEsquerdo2 = map(valorAnalogicoH - valorAnalogicoV,
                                    CENTER_ANALOG_X - CENTER_ANALOG_Y,
                                    MAX_ANALOG_X - MIN_ANALOG_Y,
                                    MIN_PWM,
                                    MAX_PWM);
          }

          else if (valorAnalogicoH < (CENTER_ANALOG_X - TOLERANCIA_ANALOGICO)) {
            pwmMotorDireito2 = map(valorAnalogicoH + valorAnalogicoV,
                                   CENTER_ANALOG_X + CENTER_ANALOG_Y,
                                   MIN_ANALOG_X + MIN_ANALOG_Y,
                                   MIN_PWM,
                                   MAX_PWM);

            pwmMotorEsquerdo2 = map(valorAnalogicoH - valorAnalogicoV,
                                    MIN_ANALOG_X - CENTER_ANALOG_Y,
                                    CENTER_ANALOG_X - MIN_ANALOG_Y,
                                    MIN_PWM,
                                    MAX_PWM);
          }

          else {
            pwmMotorDireito2 = map(valorAnalogicoV, CENTER_ANALOG_Y, MIN_ANALOG_Y, MIN_PWM, MAX_PWM);
            pwmMotorEsquerdo2 = map(valorAnalogicoV, CENTER_ANALOG_Y, MIN_ANALOG_Y, MIN_PWM, MAX_PWM);
          }
        }

        else {

          if (valorAnalogicoH > (CENTER_ANALOG_X + TOLERANCIA_ANALOGICO)) {
            pwmMotorDireito1 = MIN_PWM;
            pwmMotorDireito2 = map(valorAnalogicoH, CENTER_ANALOG_X, MAX_ANALOG_X, MIN_PWM, MAX_PWM);
            pwmMotorEsquerdo1 = map(valorAnalogicoH, CENTER_ANALOG_X, MAX_ANALOG_X, MIN_PWM, MAX_PWM);
            pwmMotorEsquerdo2 = MIN_PWM;
          }

          else if (valorAnalogicoH < (CENTER_ANALOG_X - TOLERANCIA_ANALOGICO)) {
            pwmMotorDireito1 = map(valorAnalogicoH, CENTER_ANALOG_X, MIN_ANALOG_X, MIN_PWM, MAX_PWM);
            pwmMotorDireito2 = MIN_PWM;
            pwmMotorEsquerdo1 = MIN_PWM;
            pwmMotorEsquerdo2 = map(valorAnalogicoH, CENTER_ANALOG_X, MIN_ANALOG_X, MIN_PWM, MAX_PWM);
          }

          else {
            pwmMotorDireito1 = MIN_PWM;
            pwmMotorEsquerdo1 = MIN_PWM;
            pwmMotorDireito2 = MIN_PWM;
            pwmMotorEsquerdo2 = MIN_PWM;
          }
        }

        /* ----------------- Lógica de inversão de giro da movimentação ----------------- */

        uint8_t leituraSetinhas = myController->dpad();

        // Se seta cima ou seta baixo forem pressionadas, inverte sentido motor direito
        switch (leituraSetinhas) {
          case 0x01:
            sentidoMotorEsquerdo = PINO_1_MOTOR_ESQUERDO, velocidadeMotorEsquerdo = PINO_2_MOTOR_ESQUERDO;
            sentidoMotorDireito = PINO_1_MOTOR_DIREITO, velocidadeMotorDireito = PINO_2_MOTOR_DIREITO;
            break;
          case 0x02:
            sentidoMotorEsquerdo = PINO_1_MOTOR_ESQUERDO, velocidadeMotorEsquerdo = PINO_2_MOTOR_ESQUERDO;
            sentidoMotorDireito = PINO_2_MOTOR_DIREITO, velocidadeMotorDireito = PINO_1_MOTOR_DIREITO;
            break;
          case 0x04:
            sentidoMotorEsquerdo = PINO_2_MOTOR_ESQUERDO, velocidadeMotorEsquerdo = PINO_1_MOTOR_ESQUERDO;
            sentidoMotorDireito = PINO_1_MOTOR_DIREITO, velocidadeMotorDireito = PINO_2_MOTOR_DIREITO;
            break;
          case 0x08:
            sentidoMotorEsquerdo = PINO_2_MOTOR_ESQUERDO, velocidadeMotorEsquerdo = PINO_1_MOTOR_ESQUERDO;
            sentidoMotorDireito = PINO_2_MOTOR_DIREITO, velocidadeMotorDireito = PINO_1_MOTOR_DIREITO;
            break;
        }

        analogWrite(sentidoMotorDireito, pwmMotorDireito1);
        analogWrite(velocidadeMotorDireito, pwmMotorDireito2);

        analogWrite(sentidoMotorEsquerdo, pwmMotorEsquerdo1);
        analogWrite(velocidadeMotorEsquerdo, pwmMotorEsquerdo2);

        Serial.print("PWM direito: ");
        Serial.print(pwmMotorDireito1);
        Serial.println(pwmMotorDireito2);

        Serial.print("PWM esquerdo:");
        Serial.print(pwmMotorEsquerdo1);
        Serial.println(pwmMotorEsquerdo2);
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
  //BP32.forgetBluetoothKeys();

  // Configura pinos da ESP32 para controle dos motores de locomoção.
  pinMode(PINO_1_MOTOR_ESQUERDO, OUTPUT);
  pinMode(PINO_2_MOTOR_ESQUERDO, OUTPUT);

  pinMode(PINO_1_MOTOR_DIREITO, OUTPUT);
  pinMode(PINO_2_MOTOR_DIREITO, OUTPUT);

  sentidoMotorEsquerdo = PINO_1_MOTOR_ESQUERDO, velocidadeMotorEsquerdo = PINO_2_MOTOR_ESQUERDO;
  sentidoMotorDireito = PINO_1_MOTOR_DIREITO, velocidadeMotorDireito = PINO_2_MOTOR_DIREITO;

  direitoVesquerdoH = true, direitoHesquerdoV = false;

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