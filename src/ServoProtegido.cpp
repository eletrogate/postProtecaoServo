#include <ServoProtegido.h>

ServoProtegido::ServoProtegido(const uint8_t pinoSensor) {
  this->sensor = new EntradaAnalogica(pinoSensor);
}

ServoProtegido::~ServoProtegido() {
  delete this->sensor;
}

void ServoProtegido::setLimiteSaida(const bool extremo, const unsigned int novoLimite) {
  (extremo ? this->limiteSuperiorSaida : this->limiteInferiorSaida) = novoLimite;
}

unsigned ServoProtegido::getLimiteSaida(const bool extremo) const {
  return (extremo ? this->limiteSuperiorSaida : this->limiteInferiorSaida);
}

void ServoProtegido::setSensorMax(const unsigned novoMax) {
  this->sensorMax = novoMax;
}

unsigned ServoProtegido::getSensorMax() const {
  return this->sensorMax;
}

void ServoProtegido::setMargemEntradas(const uint8_t novaMargem) {
  this->margemEntradas = novaMargem;
}

uint8_t ServoProtegido::getMargemEntradas() const {
  return this->margemEntradas;
}

void ServoProtegido::setPasso(const uint8_t novoPasso) {
  this->passo = novoPasso;
}

uint8_t ServoProtegido::getPasso() const {
  return this->passo;
}

void ServoProtegido::setIntervaloControle(const unsigned novoIntervalo) {
  this->intervaloControle = novoIntervalo;
}

unsigned ServoProtegido::getIntervaloControle() const {
  return this->intervaloControle;
}

struct DetalhesExecucao ServoProtegido::controlaServo(const unsigned usManual) {
  usUsado = DetalhesExecucao::automatico;
  this->histLeituraFiltradaSensor[ServoProtegido::tamHistLeiturasFiltradasSensor - 1] = (this->sensor)->medeComFiltro();
  if(this->histLeituraFiltradaSensor[ServoProtegido::tamHistLeiturasFiltradasSensor - 1] > this->sensorMax) {
    if(leiturasMaiorQueMax <= ServoProtegido::maxLeiturasMaiorQueMax) leiturasMaiorQueMax ++;
  } else {
    leiturasMaiorQueMax = 0;
  }
  if(leiturasMaiorQueMax >= ServoProtegido::maxLeiturasMaiorQueMax) {
    miliSegundos = millis();
    if(controleManual or retornaAPosicao) {
      controleManual = false; somaDasDiferencas = 0; qtdLeiturasNaoDecrescentes = 0; qtdLeiturasDecrescentes = 0;
      posicaoBitAtivoMaxLeiturasNaoDecrescentes = posicaoBitAtivoMaxLeiturasNaoDecrescentesInicial;
      if(!retornaAPosicao) usAutomatico = this->histUsManual[ServoProtegido::tamHistUsManual - 2];
    }
    if(miliSegundos - tempoAjuste > this->intervaloControle / 2) {
      tempoAjuste = miliSegundos;
      if(deveCorrigir and usAutomatico > this->limiteInferiorSaida and usAutomatico < this->limiteSuperiorSaida) {
        usAutomatico += direcao * this->passo * posicaoBitAtivoMaxLeiturasNaoDecrescentes;
      } else {
        usAutomatico = (this->limiteInferiorSaida + (2 * this->passo)) * (usAutomatico <= this->limiteInferiorSaida)
                      + (this->limiteSuperiorSaida - (2 * this->passo)) * (usAutomatico >= this->limiteSuperiorSaida);
        direcao = 1 - 2 * (direcao == 1);
        qtdLeiturasDecrescentes = qtdLeiturasNaoDecrescentes = 0;
      }
    }
    if(miliSegundos - tempoLeitura > this->intervaloControle) {
      tempoLeitura = miliSegundos;
      if(somaDasDiferencas >= -(this->margemEntradas)) {
        qtdLeiturasNaoDecrescentes ++;
        qtdLeiturasDecrescentes -= (qtdLeiturasDecrescentes != 0);
      }
      else {
        qtdLeiturasNaoDecrescentes -= (qtdLeiturasNaoDecrescentes != 0);
        qtdLeiturasDecrescentes ++;
      }
      if(qtdLeiturasDecrescentes >= ServoProtegido::maxLeiturasDecrescentes and posicaoBitAtivoMaxLeiturasNaoDecrescentes != posicaoBitAtivoMaxLeiturasNaoDecrescentesInicial) {
        deveCorrigir = false;
        maxLeiturasNaoDecrescentes = 1 << posicaoBitAtivoMaxLeiturasNaoDecrescentesInicial;
        qtdLeiturasDecrescentes = 0;
      }
      if(qtdLeiturasNaoDecrescentes >= maxLeiturasNaoDecrescentes) {
        deveCorrigir = true;        
        qtdLeiturasNaoDecrescentes = 0;
        posicaoBitAtivoMaxLeiturasNaoDecrescentes ++;
        if(posicaoBitAtivoMaxLeiturasNaoDecrescentes == sizeof(unsigned)) posicaoBitAtivoMaxLeiturasNaoDecrescentes = posicaoBitAtivoMaxLeiturasNaoDecrescentesInicial;
        maxLeiturasNaoDecrescentes = (1 << posicaoBitAtivoMaxLeiturasNaoDecrescentes);
        direcao = 1 - 2 * (direcao == 1);
      }
      this->writeMicroseconds(usAutomatico);
    }
    for(iterador = 0; iterador < ServoProtegido::tamHistLeiturasFiltradasSensor - 1; iterador ++) {
      somaDasDiferencas += (this->histDiferencasSensor[iterador] = this->histLeituraFiltradaSensor[iterador + 1] - this->histLeituraFiltradaSensor[iterador]);
      this->histLeituraFiltradaSensor[iterador] = this->histLeituraFiltradaSensor[iterador + 1];
    }
  } else if(this->histUsManual[ServoProtegido::tamHistUsManual - 1] = usManual; controleManual or (usAutomatico > this->histUsManual[ServoProtegido::tamHistUsManual - 2] ?
              this->histUsManual[ServoProtegido::tamHistUsManual - 1] > usAutomatico + this->margemEntradas : this->histUsManual[ServoProtegido::tamHistUsManual - 1] < usAutomatico - this->margemEntradas)) {
    controleManual = true; deveCorrigir = true; retornaAPosicao = false;
    this->histUsManual[ServoProtegido::tamHistUsManual - 2] = this->histUsManual[ServoProtegido::tamHistUsManual - 1];
    usUsado = DetalhesExecucao::manual;
    this->writeMicroseconds(this->histUsManual[ServoProtegido::tamHistUsManual - 2]);
  } else if(this->histLeituraFiltradaSensor[ServoProtegido::tamHistLeiturasFiltradasSensor - 1] < this->sensorMax / 2) {
    retornaAPosicao = true;
    if(millis() - tempoAjuste > this->intervaloControle / 2) {
      tempoAjuste = millis();
      usAutomatico += 1 - 2 * (usAutomatico > this->histUsManual[ServoProtegido::tamHistUsManual - 2]);
      this->writeMicroseconds(usAutomatico);
    }
  }
  return DetalhesExecucao{this->histLeituraFiltradaSensor[ServoProtegido::tamHistLeiturasFiltradasSensor - 1],
                            usUsado, usUsado == DetalhesExecucao::automatico ? usAutomatico : usManual};
}