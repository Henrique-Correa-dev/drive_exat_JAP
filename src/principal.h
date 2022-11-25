// ************* INCLUDES *************
#include <avr/wdt.h>                   // Biblioteca AVR do WDT do ATmegega

// ************* DEFINES *************
#define LEDPIN             13          // Numero do pino associado ao LED onboard
#define PINPAD              2          // Pino de entrada do Pulso do LED do padrão
#define PINUST1            18          // Pino de entrada do Pulso do LED do medidor 1 em teste
#define PINUST2            19          // Pino de entrada do Pulso do LED do medidor 2 em teste
#define PINUST3            20          // Pino de entrada do Pulso do LED do medidor 3 em teste
#define PINUST4             3          // Pino de entrada do Pulso do LED do medidor 4 em teste
#define PINRL1             46          // Pino de entrada do Pulso do LED do medidor 3 em teste
#define PINRL2             45          // Pino de entrada do Pulso do LED do medidor 4 em teste



#define PINREV              4          // Pino de entrada do Sinal do LED Reverso do medidor em teste

#define PIN_LED_1R          5          // Pino do LED 1 Vermelho
#define PIN_LED_1G          6          // Pino do LED 1 Verde
#define PIN_LED_2R         12          // Pino do LED 2 Vermelho
#define PIN_LED_2G         12          // Pino do LED 2 Verde
#define PIN_LED_3R         12          // Pino do LED 3 Vermelho
#define PIN_LED_3G         12          // Pino do LED 3 Verde
#define PIN_LED_4R         12          // Pino do LED 4 Vermelho
#define PIN_LED_4G         12          // Pino do LED 4 Verde
#define PIN_LED_COMUM       7          // Pino negativo do LED (comum)
#define PIN_BOTAO           8          // Pino da entrada do botão da jiga
#define PIN_BOT_COMUM       9          // Pino negativo do Botão

#define LCD_LIN             2          // Numero de Linhas do LCD
#define LCD_COL            16          // Numero de Colunas do LCD
#define ENDERECO         0x28          // Endereço do dispositivo LCD no barramento I2C

#define TOUTEXAT        30000          // Timeout da rotina de exatidão, em milissegundos
#define TOUTPULSO          15          // Timeout da rotina que impede novos pulsos, em milissegundos
#define PUL_INI_PAD         1          // pulso inicial do Padrão
#define PUL_FIM_PAD         9          // pulso final do padrão
#define PUL_LIBERA_PAD      1          // pulso da UST que libera a rotina do PAD
#define PUL_INI_UST         2          // pulso inicial da UST
#define PUL_FIM_UST         3          // pulso final da UST
#define ERRO_MAX_EXAT     0.4          // Erro máximo admissível para considerar um medidor aprovado
#define ERRO_PAD        -0.15          // Erro do padrão que deve ser desconsiderado na leitura do erro do medidor

// Erro Padrão da Jiga 1:  0.3715 
// Erro Padrão da Jiga 2:  -0.072
// Erro inicial medido do padrão: -0.0432 (medido apenas uma vez na banca de exat do LEM...)
// Desvio do erro médio de 12 medidores em comparação com a banca da produção: 0.4147

#define ATIVO             LOW          // padrão do botão acionado (medidor presente)
#define DESAT            HIGH          // padrão do botão desativo (medidor ausente)
#define DLBOT             750          // Tempo da rotina de debounce do botão de ação da jiga  
#define DLREV             150          // Tempo da rotina de debounce do sensor do LED da Reversa
#define DLLEDBL           200          // Tempo de pisca-pisca dos LEDs, em milissegundos


int pinoLED[4][2] = {
  {PIN_LED_1R, PIN_LED_1G}, 
  {PIN_LED_2R, PIN_LED_2G}, 
  {PIN_LED_3R, PIN_LED_3G}, 
  {PIN_LED_4R, PIN_LED_4G},
  };  // matriz dos pinos dos LEDs: pinoLED[led][cor]


// ************* GLOBALS *************
int fLed = 500;                        // Variável da frequencia de piscada do led onboard

void debug();                          // prototipo da função com inicializações para debug apenas
void init_exat();                      // prototipo da função de inicialização da exatidão
void init_serial();                    // prototipo da função de inicialização da serial
void serial_refresh();                 // prototipo da função de atualização da serial
void controles_refresh();              // prototipo da função de atualização dos controles da jiga
void mapa_de_pinos();                  // prototipo da função que inicializa os pinos do LCD
void atualizaLed(int, int);            // prototipo da função de troca do status dos LEDs
void controles_refresh();              // Prototipo da função de atualização periodica dos controles da Jiga
volatile uint16_t cont_t5ovf = 0;      // Global do Estouro do contador/timer 1. Para aumentar via software a resolução do contador de 16 bits para 32 bits


volatile bool exat_pad_run = false;    // Global do flag do ensaio do PAD. 

volatile bool exat_ust1_run = false;    // Global do flag do ensaio da UST
volatile bool exat_ust2_run = false;    // Global do flag do ensaio da UST
volatile bool exat_ust3_run = false;    // Global do flag do ensaio da UST
volatile bool exat_ust4_run = false;    // Global do flag do ensaio da UST

volatile uint32_t exat_pad_ini = 0;    // Global do comparador de pulsos inicial do PAD
volatile uint32_t exat_pad_fim = 0;    // Global do comparador de pulsos final do PAD
volatile uint16_t exat_pad_tic = 0;    // Global do contador de pulsos lidos do PAD


volatile uint32_t exat_ust1_ini = 0;    // Global do comparador de pulsos inicial da UST 1
volatile uint32_t exat_ust1_fim = 0;    // Global do comparador de pulsos final da UST 1
volatile uint16_t exat_ust1_tic = 0;    // Global do contador de pulsos lidos da UST 1

volatile uint32_t exat_ust2_ini = 0;    // Global do comparador de pulsos inicial da UST 2
volatile uint32_t exat_ust2_fim = 0;    // Global do comparador de pulsos final da UST 2
volatile uint16_t exat_ust2_tic = 0;    // Global do contador de pulsos lidos da UST 2

volatile uint32_t exat_ust3_ini = 0;    // Global do comparador de pulsos inicial da UST 3
volatile uint32_t exat_ust3_fim = 0;    // Global do comparador de pulsos final da UST 3
volatile uint16_t exat_ust3_tic = 0;    // Global do contador de pulsos lidos da UST 3

volatile uint32_t exat_ust4_ini = 0;    // Global do comparador de pulsos inicial da UST 4
volatile uint32_t exat_ust4_fim = 0;    // Global do comparador de pulsos final da UST 4
volatile uint16_t exat_ust4_tic = 0;    // Global do contador de pulsos lidos da UST 4


volatile bool exat_run = false;        // Global do flag da rotina de exatidão rodando
volatile bool exat_start = false;      // Global do flag de inicio da rotina de exatidão
volatile bool libera_exat_pad = false;  // Global do flag que libera a contagem de pulsos do PAD
volatile bool led_rev_status = false;  // Global do status do LED da Reversa

volatile bool exat_ruido_pad = false;  // Para filtrar ruido na interrupção de entrada do padrao
volatile bool exat_verif_pad = false;  // Para filtrar ruido na interrupção de entrada do padrao - iniciar

volatile bool exat_verif_ust1 = false;  // Para filtrar ruido na interrupção de entrada da UST - iniciar
volatile bool exat_ruido_ust1 = false;  // Para filtrar ruido na interrupção de entrada da UST

volatile bool exat_verif_ust2 = false;  // Para filtrar ruido na interrupção de entrada da UST - iniciar
volatile bool exat_ruido_ust2 = false;  // Para filtrar ruido na interrupção de entrada da UST

volatile bool exat_verif_ust3 = false;  // Para filtrar ruido na interrupção de entrada da UST - iniciar
volatile bool exat_ruido_ust3 = false;  // Para filtrar ruido na interrupção de entrada da UST

volatile bool exat_verif_ust4 = false;  // Para filtrar ruido na interrupção de entrada da UST - iniciar
volatile bool exat_ruido_ust4 = false;  // Para filtrar ruido na interrupção de entrada da UST


volatile uint16_t pulsos_ini = 3;      // Numero do pulso do inicio do intervalo
volatile uint16_t pulsos_fim = 6;      // Numero do pulso do final do intervalo
uint32_t tmr_ruido_pad = 0;            // para rotina que limita ruido no pulso do pad
uint32_t tmr_ruido_ust1 = 0;            // para rotina que limita ruido no pulso do ust
uint32_t tmr_ruido_ust2 = 0;            // para rotina que limita ruido no pulso do ust
uint32_t tmr_ruido_ust3 = 0;            // para rotina que limita ruido no pulso do ust
uint32_t tmr_ruido_ust4 = 0;            // para rotina que limita ruido no pulso do ust

uint32_t exat_tmout_init = 0;          // Global do contador de timeout da rotina de exatidão
uint32_t ledtmr = 0;                   // Global do contador de tempo da rotina de pisca-pisca do LED
uint32_t tmrbot = 0;                   // Global do temporizador da rotina de debounce
uint32_t tmrrev = 0;                   // Global do temporizador da rotina de debounce do sensor da reversa
bool reset_flag = false;               // Flag de reset do hardware por comando de software via WDT

uint32_t tini_t = 0;                   // Global do contador de tempo da rotina de pisca-pisca do LED
uint32_t tfim_t = 0;                   // Global do temporizador da rotina de debounce

// Serial EVENT (Supervisório) ---------------------------------------------
bool comandoSerial = false;
#define        PCRXBUF       50
uint16_t tmout = 0;
uint16_t pcBufRxTm = 0;
char pcBufRx[PCRXBUF];
char val = ' ';
char cmd = ' ';
char rw = ' ';
char tam = ' ';
char arg[PCRXBUF - 5];
char check = ' ';

float erro_ust1 = 0.1;
float erro_ust2 = 0.2;
float erro_ust3 = 0.3;
float erro_ust4 = 0.4;
//--------------------------------------------------------------------------

typedef union                         // Variável Union para transformar dois registradores 16 bits em um registrador 32 bits
{
  uint32_t uint32;                    // Parte 32bits
  uint16_t uint16[2];                 // vetor com as duas partes 16bits. [0] LSB e [1] MSB
} longunion_t;                        // nome do tipo criado

typedef union                         // Variável Union para transformar dois registradores 8 bits em um registrador 16 bits
{
  uint16_t uint16;                    // Parte 16bits
  uint8_t uint8[2];                   // vetor com as duas partes 8bits. [0] LSB e [1] MSB
} uintunion_t;                        // nome do tipo criado

volatile longunion_t retorno;         // Global com o tipo union para transformação do contador 16bits em 32bits com o vetor de interrupção do estouro de timer.