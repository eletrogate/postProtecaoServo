#include <EntradaAnalogica.h>

EntradaAnalogica::EntradaAnalogica(const uint8_t pino) {
  this->setPino(pino);
  this->tamFiltro = padraoTamFiltro;
  this->medicoes = new unsigned[padraoTamFiltro];
}

EntradaAnalogica::EntradaAnalogica(const uint8_t pino, const uint8_t _tamFiltro) {
  this->setPino(pino);
  this->tamFiltro = _tamFiltro;
  this->medicoes = new unsigned[_tamFiltro];
}

EntradaAnalogica::~EntradaAnalogica() {
  delete[] this->medicoes;
}

void EntradaAnalogica::setPino(const uint8_t pino) {
  this->pino = pino;
}

uint8_t EntradaAnalogica::getPino() const {
  return this->pino;
}

unsigned EntradaAnalogica::medeComFiltro() {
  for(iteradorFiltro = 0, somaFiltro = 0; iteradorFiltro < this->tamFiltro - 1; iteradorFiltro ++) {
    somaFiltro += medicoes[iteradorFiltro];
    medicoes[iteradorFiltro] = medicoes[iteradorFiltro + 1];
  }
  somaFiltro += (medicoes[this->tamFiltro - 1] = analogRead(this->pino));
  return somaFiltro / this->tamFiltro;
}