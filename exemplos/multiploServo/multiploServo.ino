#include <ServoProtegido.h>

Servo servoBraco, servoAntebraco;             //declara os objetos referentes aos servos sem proteção
ServoProtegido servoGarra(A5), servoBase(A4); //declara os objetos referentes aos servos protegidos

void setup()  {
  analogReference(EXTERNAL);  //seleciona a tensão no pino AREF como referência para as leituras
  Serial.begin(115200);       //inicia a interface UART
  servoBraco.attach(10);      //conecta cada
  servoAntebraco.attach(9);   //servo à
  servoGarra.attach(6);       //respectiva
  servoBase.attach(11);       //saída
}

void loop() {
  static EntradaAnalogica potenciometro0(A0), potenciometro1(A1),   //declara os objetos referentes
                            potenciometro2(A2), potenciometro3(A3); //às entradas analógicas
  static struct DetalhesExecucao detalhes[2];                       //declara as estruturas referentes aos resultados do controle dos servos
  servoBraco.writeMicroseconds(494 + potenciometro0.medeComFiltro());           //envia, para o servo, a leitura do
  servoAntebraco.writeMicroseconds(988 + potenciometro1.medeComFiltro());       //potenciometro somada a um valor de compensação
  detalhes[0] = servoGarra.controlaServo(988 + potenciometro2.medeComFiltro()); //passa o sinal que seria enviado pela função de controle, que avalia
  detalhes[1] = servoBase.controlaServo(988 + potenciometro3.medeComFiltro());  //a possibilidade de o enviar ao servo e registra os resultdados do processo

  static unsigned long tempo; //declara a variavel auxiliar para o controle das mensagens
  if(millis() - tempo > 100) {
    Serial.print(",usGarra/2:"); Serial.println(detalhes[0].valorUs/2);       //escreve o sinal enviado para os servos
    Serial.print(",usBase/2:"); Serial.println(detalhes[1].valorUs/2);        //divido por 2, para melhor visualização
    Serial.print(",sensorGarra:"); Serial.println(detalhes[0].ultimaLeitura); //escreve o valor lido
    Serial.print(",sensorBase:"); Serial.println(detalhes[1].ultimaLeitura);  //pelos respectivos sensores
    tempo = millis(); //atualiza a variavel auxiliar
  }

}