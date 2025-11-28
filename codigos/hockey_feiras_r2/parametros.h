
#ifndef parametros_h
#define parametros_h

#include <Arduino.h>

#define PINO_LED_INTERNO 2

#define PINO_1_MOTOR_ESQUERDO 33
#define PINO_2_MOTOR_ESQUERDO 32

#define PINO_1_MOTOR_DIREITO 26
#define PINO_2_MOTOR_DIREITO 25

// Comportamento natural do controle em ambos os analógicos
// Cima - Baixo +
// Direita + Esquerda -

// Ambos os analógicos (E e D) tem o mesmo comportamento, retornam os mesmos valores para os mesmos sentidos / mesmas direções
// Ambas as direções retornam a mesma faixa de valores
// Os valores retornados para cada um dos sentidos em uma mesma direção (esquerda/direita ou cima/baixo) não são simétricos

const int32_t MIN_ANALOG_Y = -508, CENTER_ANALOG_Y = 0, MAX_ANALOG_Y = 512;  // Valores reais * -1
const int32_t MIN_ANALOG_X = -512, CENTER_ANALOG_X = 0, MAX_ANALOG_X = 508;

const int32_t TOLERANCIA_ANALOGICO = 10;  // zona morta do controle

// Pino de controle em HIGH: 255 (menor velocidade) a 0 (maior velocidade)
// Pino de controle em LOW: 0 (menor velocidade) a 255 (maior velocidade)

const int MIN_PWM = 0, MAX_PWM = 255;  // valores limite para o PWM da ESP32

#endif