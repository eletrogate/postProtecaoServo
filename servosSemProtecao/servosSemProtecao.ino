#include <Servo.h>

Servo servo1, servo2, servo3, servo4;               //declara os objetos referentes aos servos que serão controlados
Servo servos[] = {servo1, servo2, servo3, servo4};  //inicializa um vetor com estes objetos

int filtro(int sensor)  {
  static const uint8_t tamFiltro = 128;                       //define o tamanho do vetor de amostras para cada entrada
  static const uint8_t entradas[] = {A0, A1, A2, A3, A4, A5}; //inicializa o vetor de entradas analogicas
  static unsigned medicoes[sizeof(entradas)][tamFiltro];      //declara a matriz com os vetores de amostra
  unsigned long soma = 0;                                     //inicia soma com 0
  static uint8_t i;                                           //declara o iterador
  for(i = 0; i < tamFiltro - 1; medicoes[sensor][i] = medicoes[sensor][i + 1], i ++)  //de 0 a tamFiltro - 2, deslocando cada valor para o endereço anterior a cada iteração, iterar de 1 em 1
    soma += medicoes[sensor][i];                                                      //a cada iteração, acumular o valor do respectivo endereço
  soma += (medicoes[sensor][tamFiltro - 1] = analogRead(entradas[sensor]));           //substituir o ultimo valor pela leitura do sensor e acumular esta
  return soma / tamFiltro;                                                            //retornar o valor acumulado divido pelo tamanho do vetor de amostras
}

void setup()  {
  analogReference(EXTERNAL);            //seleciona a tensão no pino AREF como referência para as leituras
  Serial.begin(115200);                 //inicia a interface UART
  const int saidas[] = {11, 10, 9, 6};  //inicializa o vetor com os pinos referentes às saídas de sinal para os servos
  for(uint8_t i = 0; i < 4; i ++)       //de 0 a 3, incrementando 1 a 1
    (servos[i]).attach(saidas[i]);      //conectar cada saída ao respectivo servo
}

void loop() {
  static uint8_t q;                 //declara o iterador
  static unsigned valorFiltrado[6]; //declara o vetor com as leituras filtradas
  for(q = 0; q < 4; q ++) {                                 //de 0 a 3, incrementando 1 a 1
    valorFiltrado[q] = map(filtro(q), 0, 1023, 500, 2500);  //armazena, em cada posição do vetor, a leitura filtrada e mapeada para a faixa da respectiva saída
    (servos[q]).writeMicroseconds(valorFiltrado[q]);        //define o sinal enviado para o respectivo servo
  }

  static unsigned long tempo;   //declara a variavel auxiliar para o controle das mensagens
  valorFiltrado[4] = filtro(4); //armazena cada leitura
  valorFiltrado[5] = filtro(5); //em uma posição do vetor
  if(millis() - tempo > 100) {  //a cada 100 milisegundos
    Serial.print(",usBase/2:"); Serial.println(valorFiltrado[0]/2);  //registra o sinal enviado para os servos
    Serial.print(",usGarra/2:"); Serial.println(valorFiltrado[3]/2); //divido por 2, para melhor visualização
    Serial.print(",sensorBase:"); Serial.println(valorFiltrado[4]);  //registra o valor lido
    Serial.print(",sensorGarra:"); Serial.println(valorFiltrado[5]); //pelos respectivos sensores
    tempo = millis(); //atualiza a variavel auxiliar
  }
}