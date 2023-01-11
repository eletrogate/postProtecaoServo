#include <Servo.h>

//#define DEBUG

Servo servo;

const uint8_t sensor = A0, potenciometro = A1, pinoServo = 5, entradasAnalogicas[] = {sensor, potenciometro}, intervalo = 16;
const uint8_t tamFiltro = 128, tamHist = 2, qtdEntradas = 2, maxLeiturasDecrescentes = 8, posicaoBitAtivoMaxLeiturasNaoDecrescentesInicial = 3, maxLeiturasMaiorQueMax = 32;
const unsigned sensorMax = 350, limiteInferiorSaida = 1000, limiteSuperiorSaida = 2000, servoMin = 988;
const int8_t margemEntradas = 10, passo = 1;

unsigned filtro(uint8_t entrada) {
  static unsigned medicoes[qtdEntradas][tamFiltro];
  unsigned long soma = 0;
  static uint8_t iteradorFiltro, iteradorEntradas;
  for(iteradorEntradas = 0; entradasAnalogicas[iteradorEntradas] != entrada; iteradorEntradas ++);
  for(iteradorFiltro = 0; iteradorFiltro < tamFiltro - 1; iteradorFiltro ++) {
    soma += medicoes[iteradorEntradas][iteradorFiltro];
    medicoes[iteradorEntradas][iteradorFiltro] = medicoes[iteradorEntradas][iteradorFiltro + 1];
  }
  soma += (medicoes[iteradorEntradas][tamFiltro - 1] = analogRead(entrada));
  return soma / tamFiltro;
}

void setup() {
  analogReference(EXTERNAL);
  #ifdef DEBUG
    Serial.begin(115200);
  #endif
  servo.attach(pinoServo);
}

void loop() {
  static unsigned leiturasSensor[tamHist], usPotenciometro[2], usAutomatico,
                    qtdLeiturasNaoDecrescentes, maxLeiturasNaoDecrescentes, tempoLeitura, tempoAjuste, miliSegundos;
  static uint8_t iterador, qtdLeiturasDecrescentes, leiturasMaiorQueMax, posicaoBitAtivoMaxLeiturasNaoDecrescentes;
  static int somaDasDiferencas, diferencasSensor[tamHist - 1];
  static int8_t direcao;
  static bool controleManual, deveCorrigir, retornaAPosicao;

  leiturasSensor[tamHist - 1] = filtro(sensor);
  if(leiturasSensor[tamHist - 1] > sensorMax) {
    if(leiturasMaiorQueMax <= maxLeiturasMaiorQueMax) leiturasMaiorQueMax ++;
  } else {
    leiturasMaiorQueMax = 0;
  }
  if(leiturasMaiorQueMax >= maxLeiturasMaiorQueMax) {
    miliSegundos = millis();
    if(controleManual or retornaAPosicao) {
      controleManual = false; somaDasDiferencas = 0; qtdLeiturasNaoDecrescentes = 0; qtdLeiturasDecrescentes = 0;
      posicaoBitAtivoMaxLeiturasNaoDecrescentes = posicaoBitAtivoMaxLeiturasNaoDecrescentesInicial;
      if(!retornaAPosicao) usAutomatico = usPotenciometro[1];
    }
    if(miliSegundos - tempoAjuste > intervalo / 2) {
      tempoAjuste = miliSegundos;
      if(deveCorrigir and usAutomatico > limiteInferiorSaida and usAutomatico < limiteSuperiorSaida) {
        usAutomatico += direcao * passo * posicaoBitAtivoMaxLeiturasNaoDecrescentes;
      } else {
        usAutomatico = (usAutomatico <= limiteInferiorSaida) ? limiteInferiorSaida + (2 * passo) : limiteSuperiorSaida - (2 * passo);
        direcao = 1 - 2 * (direcao == 1);
        qtdLeiturasDecrescentes = qtdLeiturasNaoDecrescentes = 0;
      }
    }
    if(miliSegundos - tempoLeitura > intervalo) {
      tempoLeitura = miliSegundos;
      if(somaDasDiferencas >= -margemEntradas) {
        qtdLeiturasNaoDecrescentes ++;
        qtdLeiturasDecrescentes -= (qtdLeiturasDecrescentes != 0);
      }
      else {
        qtdLeiturasNaoDecrescentes -= (qtdLeiturasNaoDecrescentes != 0);
        qtdLeiturasDecrescentes ++;
      }
      if(qtdLeiturasDecrescentes >= maxLeiturasDecrescentes and posicaoBitAtivoMaxLeiturasNaoDecrescentes != posicaoBitAtivoMaxLeiturasNaoDecrescentesInicial) {
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
      servo.writeMicroseconds(usAutomatico);
    }
    for(iterador = 0; iterador < tamHist - 1; iterador ++) {
      somaDasDiferencas += (diferencasSensor[iterador] = leiturasSensor[iterador + 1] - leiturasSensor[iterador]);
      leiturasSensor[iterador] = leiturasSensor[iterador + 1];
    }
  } else if(usPotenciometro[0] = filtro(potenciometro) + servoMin; controleManual or (usAutomatico > usPotenciometro[1] ?
              usPotenciometro[0] > usAutomatico + margemEntradas : usPotenciometro[0] < usAutomatico - margemEntradas)) {
    controleManual = true; deveCorrigir = true; retornaAPosicao = false;
    usPotenciometro[1] = usPotenciometro[0];
    servo.writeMicroseconds(usPotenciometro[1]);
  } else if(leiturasSensor[tamHist - 1] < sensorMax / 2) {
    retornaAPosicao = true;
    if(millis() - tempoAjuste > intervalo / 2) {
      tempoAjuste = millis();
      usAutomatico += 1 - 2 * (usAutomatico > usPotenciometro[1]);
      servo.writeMicroseconds(usAutomatico);
    }
  }
  #ifdef DEBUG
    static unsigned long tempoDEBUG;
    if(millis() - tempoDEBUG > 100) {
      tempoDEBUG = millis();
      Serial.print(",sensor:"); Serial.println(leiturasSensor[tamHist - 1]);
      Serial.print(",us:"); Serial.println(controleManual ? usPotenciometro[1] : usAutomatico);
    }
  #endif
}