#ifndef SERVO_PROTEGIDOH
#define SERVO_PROTEGIDOH

#include <Servo.h>
#include <Arduino.h>
#include <EntradaAnalogica.h>

class ServoProtegido : public Servo {
  static const unsigned padraoSensorMax = 350, padraoIntervaloControle = 16, padraoLimiteInferiorSaida = 988, padraoLimiteSuperiorSaida = 2012;
  static const int8_t padraoMargemEntradas = 10, maxLeiturasDecrescentes = 8, posicaoBitAtivoMaxLeiturasNaoDecrescentesInicial = 3,
                        maxLeiturasMaiorQueMax = 16, padraoPasso = 1, tamHistLeiturasFiltradasSensor = 2, tamHistUsManual = 2;

  EntradaAnalogica *sensor;
  uint8_t passo = padraoPasso;
  uint8_t margemEntradas = padraoMargemEntradas;
  unsigned intervaloControle = padraoIntervaloControle;
  unsigned sensorMax = padraoSensorMax;
  unsigned limiteInferiorSaida = padraoLimiteInferiorSaida;
  unsigned limiteSuperiorSaida = padraoLimiteSuperiorSaida;

  unsigned histLeituraFiltradaSensor[tamHistLeiturasFiltradasSensor], histUsManual[tamHistUsManual];
  int histDiferencasSensor[tamHistLeiturasFiltradasSensor - 1];

  unsigned usAutomatico, qtdLeiturasNaoDecrescentes, maxLeiturasNaoDecrescentes, tempoLeitura, tempoAjuste, miliSegundos;
  uint8_t iterador, qtdLeiturasDecrescentes, leiturasMaiorQueMax, posicaoBitAtivoMaxLeiturasNaoDecrescentes, usUsado;
  int somaDasDiferencas;
  int8_t direcao;
  bool controleManual, deveCorrigir, retornaAPosicao;
  
 public:
  enum extremo {inferior, superior};
  ServoProtegido(const uint8_t pinoSensor);
  ~ServoProtegido();
  void setLimiteSaida(const bool extremo, const unsigned novoLimite);
  unsigned getLimiteSaida(const bool extremo) const;
  void setSensorMax(const unsigned novoMax);
  unsigned getSensorMax() const;
  void setMargemEntradas(const uint8_t novaMargem);
  uint8_t getMargemEntradas() const;
  void setPasso(const uint8_t novoPasso);
  uint8_t getPasso() const;
  void setIntervaloControle(const unsigned novoIntervalo);
  unsigned getIntervaloControle() const;
  struct DetalhesExecucao controlaServo(const unsigned usManual);
};

struct DetalhesExecucao {
  enum enumUsUsado {manual, automatico};
  unsigned ultimaLeitura;
  uint8_t usUsado;
  unsigned valorUs;
};

#endif