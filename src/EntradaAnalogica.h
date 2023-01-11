#ifndef ENTRADA_ANALOGICAH
#define ENTRADA_ANALOGICAH

#include <Arduino.h>

class EntradaAnalogica {
  static const uint8_t padraoTamFiltro = 32;
  uint8_t pino;
  uint8_t tamFiltro;
  unsigned *medicoes;
  unsigned long somaFiltro;
  uint8_t iteradorFiltro;

 public:
  EntradaAnalogica(const uint8_t pino);
  EntradaAnalogica(const uint8_t pino, const uint8_t _tamFiltro);
  ~EntradaAnalogica();
  void setPino(const uint8_t pino);
  uint8_t getPino() const;
  unsigned medeComFiltro();
};

#endif