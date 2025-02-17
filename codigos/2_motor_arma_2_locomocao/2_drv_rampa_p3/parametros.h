
#ifndef parametros_h
#define parametros_h

#include <Arduino.h>

#define SENTIDO_ARMA2 13
#define VELOCIDADE_ARMA2 12

#define SENTIDO_MOTOR_ESQUERDO 14
#define VELOCIDADE_MOTOR_ESQUERDO 27

#define SENTIDO_MOTOR_DIREITO 26
#define VELOCIDADE_MOTOR_DIREITO 25

#define SENTIDO_ARMA1 33
#define VELOCIDADE_ARMA1 32

/* ............................................ Analógicos ............................................ */

// Comportamento natural do controle em ambos os analógicos
// Cima - Baixo +
// Direita + Esquerda -

// Limites dos valores retornados ao movimentar os analógicos (R - cima/baixo) (L - esquerda/direita)

const int32_t minAnalogR_Y = -508, centerAnalogR_Y = 0, maxAnalogR_Y = 512;  // Valores reais * -1
const int32_t minAnalogL_X = -512, centerAnalogL_X = 0, maxAnalogL_X = 508;

const int32_t toleranciaAnalogico = 10;  // zona morta do controle

/* ............................................. L2 e R2 .............................................. */

// Limites dos valores retornados ao prescionar L2 e R2

const int32_t minL2 = 0, maxL2 = 1023;
const int32_t minR2 = 0, maxR2 = 1023;
const int32_t toleranciaGatilhos = 10;

/* ............................................ Saída PWM ............................................. */

// Lógica de funcionamento da ponte H DRV8833:
// Pino de controle em HIGH: 255 (menor velocidade) a 0 (maior velocidade)
// Pino de controle em LOW: 0 (menor velocidade) a 255 (maior velocidade)

const int minPWM = 0, maxPWM = 255;

#endif