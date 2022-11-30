/*
 *   ROTINA PRINCIPAL PARA MEDICAO DA EXATIDAO DO MEDIDOR
 *   Método: Comparação direta entre o tempo dos pulsos do medidor e o tempo entre os pulsos do padrão
 *   
 *   STATUS: validado ok!
 *   
 *   Instrucao de Uso: 
 *   
 */
#include "principal.h"
#include "Arduino.h"

void exat_pad_vect();
void exat_ust1_vect();
void exat_ust2_vect();
void exat_ust3_vect();
void exat_ust4_vect();


void exatidao();
void tira_ruido_ust1();
void tira_ruido_ust2();
void tira_ruido_ust3();
void tira_ruido_ust4();
void tira_ruido_pad();

int32_t last_value_pad = 0;
int32_t last_value_ust = 0;

bool chavear = false;

void init_exat() {
  // Inicializando o TIMER 1
  cli();                            // Desabilita as interrupções - necessário para ajustar os SFRs
  TCCR5A = 0;                       // Limpa o Registrador de controle A
  TCCR5B = 0;                       // Limpa o Registrador de controle B   
  TIMSK5 |= (1 << TOIE5);           // Habilita a interrupção pelo estouro do TIMER 1, para aumentar o Timer via Software
  TCNT5 = 0;                        // Zera o valor do registrador de contagem do TIMER 1, parte baixa (automatico pelo hardware)
  cont_t5ovf = 0;                   // Zera o valor do registrador de contagem do TIMER 1, parte alta (criado via software)
  //TCCR5B |= (1 << CS12);          // configura prescaler para 256: CS12 = 1
  TCCR5B |= (1 << CS11);            // configura prescaler para 64: CS11 = 1
  TCCR5B |= (1 << CS10);            // configura prescaler para 64: CS10 = 1
  sei();                            // Habilita novamente as interrupções
  
  attachInterrupt(digitalPinToInterrupt(PINPAD), exat_pad_vect, RISING);        // Liga a interrupção do pino de captura de pulso do padrao em borda de subida
  attachInterrupt(digitalPinToInterrupt(PINUST1), exat_ust1_vect, FALLING);     // Liga a interrupção do pino de captura de pulso do medidor sob ensaio em borda de subida
  attachInterrupt(digitalPinToInterrupt(PINUST2), exat_ust2_vect, FALLING);     // Liga a interrupção do pino de captura de pulso do medidor sob ensaio em borda de subida
  attachInterrupt(digitalPinToInterrupt(PINUST3), exat_ust3_vect, FALLING);     // Liga a interrupção do pino de captura de pulso do medidor sob ensaio em borda de subida
  attachInterrupt(digitalPinToInterrupt(PINUST4), exat_ust4_vect, FALLING);     // Liga a interrupção do pino de captura de pulso do medidor sob ensaio em borda de subida
}

//  ************  VETORES DE INTERRUPCAO  ************

ISR(TIMER5_OVF_vect) {              // Vetor da interrupção do Timer 1. Gastar o menor tempo possível. Nome do vetor é padrão do AVR-GCC
  cont_t5ovf++;                     // Para cada estouro de timer, soma 1 no contador de 16 bits, para expandir o contador para 32 bits
}

void exat_pad_vect() {              // vetor da interrupção do pino de captura de pulsos do padrão
  if (!exat_ruido_pad) {
    exat_ruido_pad = true;  // trava a entrada nesta rotina
    exat_verif_pad = true;  // inicializa rotina de filtro
    if (exat_pad_run && libera_exat_pad) {  // se está rodando exatidão no padrão e já liberou para contar pulsos no padrão
      exat_pad_tic ++;  // incrementa contador de pulsos
      if (exat_pad_tic == PUL_INI_PAD) { // verifica se é o pulso inicial
        retorno.uint16[0] = TCNT5;  // pega a parte baixa do contador
        retorno.uint16[1] = cont_t5ovf;  // pega a parte alta do contador
        exat_pad_ini = retorno.uint32;  // salva o numero de pulsos no registrador de pulsos inicial
      } else if (exat_pad_tic == PUL_FIM_PAD) {  // verifica se é o pulso final
        retorno.uint16[0] = TCNT5;  // pega a parte baixa do contador
        retorno.uint16[1] = cont_t5ovf;  // pega a parte alta do contador
        exat_pad_fim = retorno.uint32;  // salva o numero de pulsos no registrador de pulsos final
        exat_pad_run = false;  // ajusta flag para sinalizar fim da medição de pulsos
      }
    }
  }
}

void exat_ust1_vect() {             // vetor da interrupção do pino de captura de pulsos da unidade sob ensaio (UST)
  if (!exat_ruido_ust1) {
    exat_ruido_ust1 = true;  // trava a entrada nesta rotina
    exat_verif_ust1 = true;  // inicializa rotina de filtro
    if (exat_ust1_run) {  // se está rodando exatidão na UST
      exat_ust1_tic ++;  // incrementa contador de pulsos
      if (exat_ust1_tic == PUL_LIBERA_PAD) {  // verifica se é o pulso de liberar o padrão (quando o UST acordou...)
        libera_exat_pad = true;  // Libera a rotina de contagem de pulsos do padrão
      } else if (exat_ust1_tic == PUL_INI_UST) {  // verifica se é o pulso inicial
        retorno.uint16[0] = TCNT5;  // pega a parte baixa do contador de pulsos
        retorno.uint16[1] = cont_t5ovf;  // pega a parte alta do contador de pulsos
        exat_ust1_ini = retorno.uint32;  // salva o numero de pulsos no registrador de pulsos inicial
      } else if (exat_ust1_tic == PUL_FIM_UST) {  // verifica se é o pulso final
        retorno.uint16[0] = TCNT5;  // pega a parte baixa do contador de pulsos
        retorno.uint16[1] = cont_t5ovf;  // pega a parte alta do contador de pulsos
        exat_ust1_fim = retorno.uint32;  // salva o numero de pulsos no registrador de pulsos final
        exat_ust1_run = false;  // ajusta flag para sinalizar fim da medição de pulsos
      }
    }
  }
}

void exat_ust2_vect() {             // vetor da interrupção do pino de captura de pulsos da unidade sob ensaio (UST)
  if (!exat_ruido_ust2) {
    exat_ruido_ust2 = true;  // trava a entrada nesta rotina
    exat_verif_ust2 = true;  // inicializa rotina de filtro
    if (exat_ust2_run) {  // se está rodando exatidão na UST
      exat_ust2_tic ++;  // incrementa contador de pulsos
      if (exat_ust2_tic == PUL_LIBERA_PAD) {  // verifica se é o pulso de liberar o padrão (quando o UST acordou...)
        libera_exat_pad = true;  // Libera a rotina de contagem de pulsos do padrão
      } else if (exat_ust2_tic == PUL_INI_UST) {  // verifica se é o pulso inicial
        retorno.uint16[0] = TCNT5;  // pega a parte baixa do contador de pulsos
        retorno.uint16[1] = cont_t5ovf;  // pega a parte alta do contador de pulsos
        exat_ust2_ini = retorno.uint32;  // salva o numero de pulsos no registrador de pulsos inicial
      } else if (exat_ust2_tic == PUL_FIM_UST) {  // verifica se é o pulso final
        retorno.uint16[0] = TCNT5;  // pega a parte baixa do contador de pulsos
        retorno.uint16[1] = cont_t5ovf;  // pega a parte alta do contador de pulsos
        exat_ust2_fim = retorno.uint32;  // salva o numero de pulsos no registrador de pulsos final
        exat_ust2_run = false;  // ajusta flag para sinalizar fim da medição de pulsos
      }
    }
  }
}

void exat_ust3_vect() {             // vetor da interrupção do pino de captura de pulsos da unidade sob ensaio (UST)
  if (!exat_ruido_ust3) {
    exat_ruido_ust3 = true;  // trava a entrada nesta rotina
    exat_verif_ust3 = true;  // inicializa rotina de filtro
    if (exat_ust3_run) {  // se está rodando exatidão na UST
      exat_ust3_tic ++;  // incrementa contador de pulsos
      if (exat_ust3_tic == PUL_LIBERA_PAD) {  // verifica se é o pulso de liberar o padrão (quando o UST acordou...)
        libera_exat_pad = true;  // Libera a rotina de contagem de pulsos do padrão
      } else if (exat_ust3_tic == PUL_INI_UST) {  // verifica se é o pulso inicial
        retorno.uint16[0] = TCNT5;  // pega a parte baixa do contador de pulsos
        retorno.uint16[1] = cont_t5ovf;  // pega a parte alta do contador de pulsos
        exat_ust3_ini = retorno.uint32;  // salva o numero de pulsos no registrador de pulsos inicial
      } else if (exat_ust3_tic == PUL_FIM_UST) {  // verifica se é o pulso final
        retorno.uint16[0] = TCNT5;  // pega a parte baixa do contador de pulsos
        retorno.uint16[1] = cont_t5ovf;  // pega a parte alta do contador de pulsos
        exat_ust3_fim = retorno.uint32;  // salva o numero de pulsos no registrador de pulsos final
        exat_ust3_run = false;  // ajusta flag para sinalizar fim da medição de pulsos
      }
    }
  }
}

void exat_ust4_vect() {             // vetor da interrupção do pino de captura de pulsos da unidade sob ensaio (UST)
  if (!exat_ruido_ust4) {
    exat_ruido_ust4 = true;  // trava a entrada nesta rotina
    exat_verif_ust4 = true;  // inicializa rotina de filtro
    if (exat_ust4_run) {  // se está rodando exatidão na UST
      exat_ust4_tic ++;  // incrementa contador de pulsos
      if (exat_ust4_tic == PUL_LIBERA_PAD) {  // verifica se é o pulso de liberar o padrão (quando o UST acordou...)
        libera_exat_pad = true;  // Libera a rotina de contagem de pulsos do padrão
      } else if (exat_ust4_tic == PUL_INI_UST) {  // verifica se é o pulso inicial
        retorno.uint16[0] = TCNT5;  // pega a parte baixa do contador de pulsos
        retorno.uint16[1] = cont_t5ovf;  // pega a parte alta do contador de pulsos
        exat_ust4_ini = retorno.uint32;  // salva o numero de pulsos no registrador de pulsos inicial
      } else if (exat_ust4_tic == PUL_FIM_UST) {  // verifica se é o pulso final
        retorno.uint16[0] = TCNT5;  // pega a parte baixa do contador de pulsos
        retorno.uint16[1] = cont_t5ovf;  // pega a parte alta do contador de pulsos
        exat_ust4_fim = retorno.uint32;  // salva o numero de pulsos no registrador de pulsos final
        exat_ust4_run = false;  // ajusta flag para sinalizar fim da medição de pulsos
      }
    }
  }
}

//  ************  ROTINAS DE ENSAIO  ************

void exat_refresh() {
  exatidao();                       // Chama rotina que realiza exatidão, caso tenha sido setado pelo supErvisorio
  tira_ruido_pad();                 // Chama rotina de delay para evitar ruido na entrada de pulso do padrão
  tira_ruido_ust1();                // Chama rotina de delay para evitar ruido na entrada de pulso
  tira_ruido_ust2();
  tira_ruido_ust3();
  tira_ruido_ust4();
}

void exatidao() {
  if (exat_start) {                 // se pediu para iniciar a rotina de exatidão
    exat_start = false;             // para entrar aqui somente uma vez por solicitação...

      // Inicializando o TIMER 1
      cli();                         // Desabilita as interrupções - necessário para ajustar os SFRs
      TCCR5A = 0;                    // Limpa o Registrador de controle A
      TCCR5B = 0;                    // Limpa o Registrador de controle B   
      TIMSK5 |= (1 << TOIE5);        // Habilita a interrupção pelo estouro do TIMER 1, para aumentar o Timer via Software
      TCNT5 = 0;                     // Zera o valor do registrador de contagem do TIMER 1, parte baixa (automatico pelo hardware)
      cont_t5ovf = 0;                // Zera o valor do registrador de contagem do TIMER 1, parte alta (criado via software)
      //TCCR5B |= (1 << CS12);       // configura prescaler para 256: CS12 = 1
      TCCR5B |= (1 << CS11);         // configura prescaler para 64: CS11 = 1
      TCCR5B |= (1 << CS10);         // configura prescaler para 64: CS10 = 1
      sei();                         // Habilita novamente as interrupções

    //ajustar algumas condições iniciais da rotina de exatidão
    exat_pad_tic = 0;
    exat_ust1_tic = 0;
    exat_ust2_tic = 0;
    exat_ust3_tic = 0;
    exat_ust4_tic = 0;
    exat_tmout_init = millis();

    exat_pad_ini = 0;
    exat_pad_fim = 0;
    exat_ust1_ini = 0;
    exat_ust1_fim = 0;
    exat_ust2_ini = 0;
    exat_ust2_fim = 0;
    exat_ust3_ini = 0;
    exat_ust3_fim = 0;
    exat_ust4_ini = 0;
    exat_ust4_fim = 0;

    if(chavear){
      digitalWrite(PINRL1,LOW);
      digitalWrite(PINRL2,LOW);
      delay(1000);
      digitalWrite(PINRL1,LOW);
      digitalWrite(PINRL2,HIGH);
    }
    exat_run = true;                // informa que começamos a exatidão
    libera_exat_pad = false;        // bloqueia a contagem de pulsos do padrão até o medidor UST ligar e pulsar
    exat_pad_run = true;            // liga a exatidão no padrão
    exat_ust1_run = true;           // liga a exatidão na ust
    exat_ust2_run = true;           // liga a exatidão na ust
    exat_ust3_run = true;           // liga a exatidão na ust
    exat_ust4_run = true;           // liga a exatidão na ust
    tini_t = millis();
  }
  if (exat_run) {  // se estamos rodando a rotina de exatidão
    if (!exat_pad_run && !exat_ust1_run  && !exat_ust2_run  && !exat_ust3_run && !exat_ust4_run) {  // se já terminou de coletar pulso do padrão e da ust
      int32_t dif_pad = exat_pad_fim - exat_pad_ini;      // tempo entre pulsos do padrão
      int32_t dif_ust1 = exat_ust1_fim - exat_ust1_ini;   // tempo entre pulsos da ust
      int32_t dif_ust2 = exat_ust2_fim - exat_ust2_ini;   // tempo entre pulsos da ust
      int32_t dif_ust3 = exat_ust3_fim - exat_ust3_ini;   // tempo entre pulsos da ust
      int32_t dif_ust4 = exat_ust4_fim - exat_ust4_ini;   // tempo entre pulsos da ust

      /*Serial.print("incio PADRÃO: ");
      Serial.print(exat_pad_ini);
      Serial.print("  inicio UST: ");
      Serial.println(exat_ust1_ini);

      Serial.print("fim PADRÃO: ");
      Serial.print(exat_pad_fim);
      Serial.print("  fim UST: ");
      Serial.println(exat_ust1_fim);

      Serial.print("dif PADRÃO: ");
      Serial.print(dif_pad);
      Serial.print("  dif UST: ");
      Serial.println(dif_ust);*/

      erro_ust1 = 0;
      erro_ust2 = 0;
      erro_ust3 = 0;
      erro_ust4 = 0;
      bool status_ust = false;  // variável para guardar o teste de se o medidor foi aprovado ou reprovado

      erro_ust1 = (((float)dif_ust1/(float)dif_pad)-(float)1)*(float)100;  // erro em % da UST referente ao valor lido no Padrão
      erro_ust2 = (((float)dif_ust2/(float)dif_pad)-(float)1)*(float)100;  // erro em % da UST referente ao valor lido no Padrão
      erro_ust3 = (((float)dif_ust3/(float)dif_pad)-(float)1)*(float)100;  // erro em % da UST referente ao valor lido no Padrão
      erro_ust4 = (((float)dif_ust4/(float)dif_pad)-(float)1)*(float)100;  // erro em % da UST referente ao valor lido no Padrão

      erro_ust1 -= ERRO_PAD;  // desconta o erro sistêmico do padrão
      erro_ust2 -= ERRO_PAD;  // desconta o erro sistêmico do padrão
      erro_ust3 -= ERRO_PAD;  // desconta o erro sistêmico do padrão
      erro_ust4 -= ERRO_PAD;  // desconta o erro sistêmico do padrão

      float erro_abs1 = abs(erro_ust1);  // float do erro sem sinal

      tfim_t = millis() - tini_t;

      /*Serial.println(tfim_t);
      Serial.print("ERRO 1: ");
      Serial.print(erro_ust1,3);
      Serial.println("%");*/
      //Serial.println((dif_pad - last_value_pad), DEC);
      //Serial.println((dif_ust1 - last_value_ust), DEC);
      //Serial.println("%\n");
      /*last_value_pad = dif_pad;
      last_value_ust = dif_ust1;
      Serial.print("ERRO 2: ");
      Serial.print(erro_ust2,3);
      Serial.println("%");
      Serial.print("ERRO 3: ");
      Serial.print(erro_ust3,3);
      Serial.println("%");
      Serial.print("ERRO 4: ");
      Serial.print(erro_ust4,3);
      Serial.println("%\n");*/

      status_ust = (erro_abs1 <= ERRO_MAX_EXAT);  // teste para definir se o medidor foi aprovado ou reprovado
      char errofim[17];  // cria o bufer para formatar o erro
      char tmpconv[7];  // buffer de conversao
      memset (errofim, '\0', 17);  // limpa o buffer de formatação
      memset (tmpconv, '\0', 7);  // limpa o buffer de conversão
      //dtostrf(erro_ust1, 7, 3, tmpconv);
      sprintf(errofim,"Erro: %s%%", tmpconv);
      if (true){//led_rev_status) {  // Se o LED da Reversa estiver acesso
        Serial.print("");
      } else {  // o LED da reversa está desligado... deu ruim!
        //Serial.println("MEDIDOR REPROVADO");
        status_ust = false;
      }
      if (status_ust) {  // medidor APROVADO
        //atualizaLed(0,2);  //atualiza o status do LED
        Serial.print("");
      } else {  // Medidor REPROVADO
        //atualizaLed(0,1);  //atualiza o status do LED
        Serial.print("");
      }
      exat_run = false;
      //exat_start = true;
    } else if ((millis() - exat_tmout_init) > TOUTEXAT) {  // verifica se deu timeout da exatidão
      Serial.println(exat_ust1_run);
      Serial.println(exat_ust2_run);
      Serial.println(exat_ust3_run);
      Serial.println(exat_ust4_run);
      
      //cancela tudo... e tenta descobrir o motivo...
      exat_run = false;
      fLed = 500;
      String temptxt1 = "";  // variavel temporaria para formatar o debug do motivo do timeout...
      String temptxt2 = "";  // variavel temporaria para formatar o debug do motivo do timeout...

      if (!exat_pad_run && !exat_ust1_run) { // Sem Erro
        temptxt1 = "Erro? UST/PAD OK";
        temptxt2 = "O que faco aqui?";
        //atualizaLed(0,4);  // LED piscando vermelho
      } else if (exat_pad_run && !exat_ust1_run) { // Erro no padrão: sem pulso
        temptxt1 = "Padrao sem Pulso";
        temptxt2 = "Chamar Engenhari";
        //atualizaLed(0,4);  // LED Vermelho piscando
      } else if (!exat_pad_run && exat_ust1_run) { // Erro no medidor: sem pulso
        if (true){//led_rev_status) {  // opa! não pulsou mas acendeu a reversa...
          temptxt2 = " LED Pulso Ruim ";
          temptxt1 = "Medidor Liga mas";
          //atualizaLed(0,1);  // LED Vermelho
        } else {  // humm... sem reversa, o medidor deve estar morto...
          temptxt2 = " Medidor MORTO  ";
          temptxt1 = "Teste REPROVADO!";
          //atualizaLed(0,1);  // LED Vermelho
        }
      } else { // Não veio nada... erro de corrente... conexão
        temptxt1 = "Encaixe Ruim do ";
        temptxt2 = "Medidor na JIG  ";
        //atualizaLed(0,4);  // LED piscando vermelho
      }
      exat_pad_run = false;
      exat_ust1_run = false;
      exat_ust2_run = false;
      exat_ust3_run = false;
      exat_ust4_run = false;
      //lcd.escrever(temptxt1,temptxt2);
      Serial.println(temptxt1);
      Serial.println(temptxt2);
      erro_ust1 = -100.0;
      erro_ust2 = -100.0;
      erro_ust3 = -100.0;
      erro_ust4 = -100.0;
      //exat_start = true;
    }
  }
}

void tira_ruido_pad(){
   if (exat_verif_pad) {  // pediu para iniciar rotina de filtro
    exat_verif_pad = false; // já veio aqui não volta sem pedir
    tmr_ruido_pad = millis();  // ajusta temporizador
    exat_ruido_pad = true;  // indica que estamos fechados para novos pulsos até estourar o timer...
  } else if (exat_ruido_pad) {  // se já estamos aguardando finalizar o tempo do pulso atual...
    if ((millis() - tmr_ruido_pad) > TOUTPULSO) { //  verifica se o tempo já terminou
      exat_ruido_pad = false;  // desliga o bloqueio de novos pulsos
    }
  }
}

void tira_ruido_ust1() {  // rotina de delay para filtrar ruido na entrada de pulso dos medidores
  if (exat_verif_ust1) {  // pediu para iniciar rotina de filtro
    exat_verif_ust1 = false; // já veio aqui não volta sem pedir
    tmr_ruido_ust1 = millis();  // ajusta temporizador
    exat_ruido_ust1 = true;  // indica que estamos fechados para novos pulsos até estourar o timer...
  } else if (exat_ruido_ust1) {  // se já estamos aguardando finalizar o tempo do pulso atual...
    if ((millis() - tmr_ruido_ust1) > TOUTPULSO) { //  verifica se o tempo já terminou
      exat_ruido_ust1 = false;  // desliga o bloqueio de novos pulsos
    }
  }
}

void tira_ruido_ust2() {  // rotina de delay para filtrar ruido na entrada de pulso dos medidores
  if (exat_verif_ust2) {  // pediu para iniciar rotina de filtro
    exat_verif_ust2 = false; // já veio aqui não volta sem pedir
    tmr_ruido_ust2 = millis();  // ajusta temporizador
    exat_ruido_ust2 = true;  // indica que estamos fechados para novos pulsos até estourar o timer...
  } else if (exat_ruido_ust2) {  // se já estamos aguardando finalizar o tempo do pulso atual...
    if ((millis() - tmr_ruido_ust2) > TOUTPULSO) { //  verifica se o tempo já terminou
      exat_ruido_ust2 = false;  // desliga o bloqueio de novos pulsos
    }
  }
}

void tira_ruido_ust3() {  // rotina de delay para filtrar ruido na entrada de pulso dos medidores
  if (exat_verif_ust3) {  // pediu para iniciar rotina de filtro
    exat_verif_ust3 = false; // já veio aqui não volta sem pedir
    tmr_ruido_ust3 = millis();  // ajusta temporizador
    exat_ruido_ust3 = true;  // indica que estamos fechados para novos pulsos até estourar o timer...
  } else if (exat_ruido_ust3) {  // se já estamos aguardando finalizar o tempo do pulso atual...
    if ((millis() - tmr_ruido_ust3) > TOUTPULSO) { //  verifica se o tempo já terminou
      exat_ruido_ust3 = false;  // desliga o bloqueio de novos pulsos
    }
  }
}

void tira_ruido_ust4() {  // rotina de delay para filtrar ruido na entrada de pulso dos medidores
  if (exat_verif_ust4) {  // pediu para iniciar rotina de filtro
    exat_verif_ust4 = false; // já veio aqui não volta sem pedir
    tmr_ruido_ust4 = millis();  // ajusta temporizador
    exat_ruido_ust4 = true;  // indica que estamos fechados para novos pulsos até estourar o timer...
  } else if (exat_ruido_ust4) {  // se já estamos aguardando finalizar o tempo do pulso atual...
    if ((millis() - tmr_ruido_ust4) > TOUTPULSO) { //  verifica se o tempo já terminou
      exat_ruido_ust4 = false;  // desliga o bloqueio de novos pulsos
    }
  }
}


// ************ COMUNNICAÇÃO SERIAL SUPERVISÓRIO ******************
char calc_checksum(char _val, char _cmd, char _rw, char _tam, char *_arg){
    int checksum = 0;
    checksum = _val + _cmd + _rw + _tam;
    for(int i=0; i<_tam; i++){
        checksum += _arg[i];
    }
    
    return checksum;
}

void snd_command(char _cmd, char _rw, char _tam,char * _arg){
    char pct[261];

    pct[0] = 0x7f;
    pct[1] = _cmd;
    pct[2] = _rw;
    pct[3] = _tam;

    for(int i=0; i<_tam; i++){
        pct[i+4] = _arg[i];
    }
    pct[_tam+4] = calc_checksum(0x7f,_cmd,_rw,_tam,_arg);

    for(int i=0; i<_tam+5; i++){
        Serial.print(pct[i]);
    }
}

float bytes2float(unsigned char *bytes){
    union {
        float a;
        unsigned char bytes[4];
    }value;
    
    for(int i=0; i<4; i++){
        value.bytes[i] = bytes[i];
    }

    return value.a;
}

void serialEventLoop(){
    if(comandoSerial == true){
        comandoSerial = false;
        if (val == 0x7f && calc_checksum(val,cmd,rw,tam,arg) == check){

            //AJUSTAR GRANDEZAS
            switch (cmd){
                case 0x51:
                    if(rw == 0x01){
                        char _cmd = cmd;
                        char _rw = rw;
                        char _tam = 0x01;
                        exat_start = true;

                        snd_command(_cmd,_rw,_tam,arg);
                    }else{
                    }
                    break;
                    
                case 0x52:
                    if(rw == 0x01){
                    }else{
                      char _cmd = cmd;
                      char _rw = rw;
                      char _tam = 0x10;
                      char _arg [16];

                      union {
                        float float_variable;
                        unsigned char temp_array[4];
                      }u;

                      for(int j=0; j<4; j++){
                        if(j == 0) u.float_variable = (float)erro_ust1;
                        if(j == 1) u.float_variable = (float)erro_ust2;
                        if(j == 2) u.float_variable = (float)erro_ust3;
                        if(j == 3) u.float_variable = (float)erro_ust4;

                        for(int i=0; i<4; i++){
                            _arg[i+4*j] = u.temp_array[i];
                        }
                      }
                      snd_command(_cmd,_rw,_tam,_arg);
                    }
                    break;
                default:
                    break;
            }
        }

        /*for(int i=0; i<pcBufRxTm; i++){
            Serial.print(pcBufRx[i],HEX);
        }
        Serial.println("");*/

        memset(pcBufRx, 0, pcBufRxTm); // Esvazia o vetor inputString
        pcBufRxTm = 0;
    }
}

// *********** INICIALIZAÇÃO DOS PINOS DE CONTROLE ****************
void init_pin(){
  pinMode(PINRL1,OUTPUT);
  pinMode(PINRL2,OUTPUT);

  digitalWrite(PINRL1,HIGH);
  digitalWrite(PINRL2,LOW);
}

void setup(){
  Serial.begin(115200);
  init_exat();
  init_pin();
  //exat_start = true;
}

void loop(){
  exat_refresh();
  serialEventLoop();
}

void serialEvent(){
    while (Serial.available()) {
        char estebyte = (char)Serial.read();
        pcBufRx[pcBufRxTm] = estebyte;
        if(pcBufRxTm == 0 && pcBufRx[0] == 0x7f) {
            val = estebyte;
        }
        if(pcBufRxTm == 1) cmd = estebyte; 
        if(pcBufRxTm == 2) rw = estebyte;
        if(pcBufRxTm == 3) tam = estebyte;
        if(pcBufRxTm > 3 && pcBufRxTm <= tam + 3){
            arg[pcBufRxTm - 4] = estebyte;
        }
        if(pcBufRxTm > 3 && pcBufRxTm > tam + 3 && comandoSerial == false){
            check = estebyte;
            tmout = 0;
            comandoSerial = true;
        }
        if(pcBufRx[0] == 0x7f){
            pcBufRxTm++;
        }
    }
}