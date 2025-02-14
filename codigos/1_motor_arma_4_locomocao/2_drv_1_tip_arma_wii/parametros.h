
#ifndef parametros_h
#define parametros_h

#include <Arduino.h>

#define SENTIDO_MOTOR_ESQUERDO1 12
#define VELOCIDADE_MOTOR_ESQUERDO1 13

#define SENTIDO_MOTOR_ESQUERDO2 14
#define VELOCIDADE_MOTOR_ESQUERDO2 27

#define SENTIDO_MOTOR_DIREITO1 26
#define VELOCIDADE_MOTOR_DIREITO1 25

#define SENTIDO_MOTOR_DIREITO2 33
#define VELOCIDADE_MOTOR_DIREITO2 32

#define PINO_ARMA 23

// Comportamento natural do controle em ambos os analógicos
// Cima - Baixo +
// Direita + Esquerda -

const int32_t minAnalogR_Y = -392, centerAnalogR_Y = -12, maxAnalogR_Y = 384;  // Valores reais * -1
const int32_t minAnalogR_X = -384, centerAnalogR_X = -12, maxAnalogR_X = 404;

const int32_t toleranciaAnalogico = 10;  // zona morta do controle

// Pino de controle em HIGH: 255 (menor velocidade) a 0 (maior velocidade)
// Pino de controle em LOW: 0 (menor velocidade) a 255 (maior velocidade)

const int minPWM = 0, maxPWM = 255;

#endif