#include <arduino.h>
/*************************
Teste de algoritmo de controle, ISR executando à 100Hz atrelado ao timer2
+ leitura da porta serial e interpretação de comandos
= para comando de PWM associado com led no pino 5 ou 6 (980 Hz; timer0)
Fernando Passold, em 18/04/2024
Revisado em 21/05/2024.

"Dicionário" de comandos interpretados pela porta serial:
p 100.5 = define ganho Proporcional em 100.5
a = toggle entre modo manual e modo automático
a0 - a 0 = "desliga" modo automático
a 1 = "liga" modo automático
v = toggle no modo verbouse (debug)
v 0 = desliga modo verbouse
v 1 = liga modo verbouse
r 50 = define Referencia em 50
u 50 = define sinal de controle (se automático desligado) em 50
u = faz PWM = 0 ("desliga")
s = “status”: publica valores atuais de sp, Kp e Kd

Detalhes da montagem:
- led para "testar" algoritmo de controle conectado no pino 5 ou 6
- led que pisca indicando execução do algoritmo de controle, no pino 10
- todos os leds em config ativo alto
*********************************/

// declaração variáveis globais:
bool ledState = 0;            // para fazer variar nível lógico led monitor
const byte LED_MONITOR = 13;  // toglles a cada 0,5 segundos monitorando algo controle (ISR)
const byte pwm = 9;           // Led à ser "controlado", no pino 5 (ou 6)
const byte analogPin = A0;    // pino (A/D) que recebe sinal analógico

int ledCounter = 0;           // conta até 50 para então picar Led monitor
bool verbouse = true;         // gerar texto confirmando comando recebido pela porta serial
bool mute = false;            // indica ativar ou não o buzzer (feedback auditivo)

const int MAX_U = 255;        // caso do PWM do Arduino, [0, 255]

// variáveis associadas com algoritmo de controle
bool control = false;         // comuta entre modo automático (>=1) e manual (<=0)
float sp = 0;                 // set-point, referência, r[k]
float pv = 0;                 // process-variable, saída do processo, y[k]
int mv = 0;                   // manipulated-variable, saída do controlador, u[k]
float u;                      // u[k]
float Kp = 1.0;               // ganho proporcional

// variáveis associadas com dados (string) capturada via porta serial
String inputString;
char option;
float value;
int numericIndex = 0;
unsigned int lastTime, period;

void Init_Control() {
  // inicializa variáveis associadas com lei de controle
  u = 0;
  mv = 0;
}

void BeepErro(void) {
  // 1 x beep algo longo, comando não reconhecido
  // digitalWrite(BUZZER, HIGH);
  // delay(200);
  // digitalWrite(BUZZER, LOW);
}

void BeepOk(void) {
  // 2 x beeps médios, intervalo curto, aviso de comando reconhecido
  // digitalWrite(BUZZER, HIGH);
  // delay(100);
  // digitalWrite(BUZZER, LOW);
  // delay(50);
  // digitalWrite(BUZZER, HIGH);
  // delay(100);
  // digitalWrite(BUZZER, LOW);
}

void publica_estado(char const msg[], bool estado) {
  Serial.print(msg);
  Serial.print(": ");
  if (estado)
    Serial.println("ON (true)");
  else
    Serial.println("OFF (false)");
}

void publica_parametro(char const msg[], float valor) {
  Serial.print(msg);
  Serial.print(" = ");
  Serial.println(valor, 4);
}

void publica_status(void) {
  float aux;  // copia de algumas variáveis atualizadas muito rapidamente, à cada 100 Hz...
  Serial.println("# Status do sistema ###################################");
  Serial.print("PV: y[k] = ");
  aux = pv;
  Serial.print(aux, 2);
  Serial.print(", MV: u[k] = ");
  aux = (float)mv;
  Serial.print(aux, 2);
  Serial.print(", SP: r[k] = ");
  aux = sp;
  Serial.println(aux, 2);
  Serial.print(" Modo = ");
  if (control)
    Serial.println("AUTO");
  else
    Serial.println("Manual");
  publica_parametro("   Kp", Kp);
  publica_estado("Verbouse", verbouse);
}

void setup() {
  // inicializa pinos dos leds e começa com eles desligados
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(LED_MONITOR, OUTPUT);
  digitalWrite(LED_MONITOR, LOW);
  pinMode(pwm, OUTPUT);
  analogWrite(pwm, 0);  // garante que este led inicia apagado
  // inicializa saída de controle, mv
  Init_Control();
  control = false;  // controle automático desligado
  verbouse = true;  // ativado debug
  mute = false;
  // initialize timer2
  noInterrupts();  // disable all interrupts
  TCCR2A = 0;      // set entire TCCR2A register to 0
  TCCR2B = 0;      // same for TCCR2B
  TCNT2 = 0;       // initialize counter value to 0
  // set compare match register for 10 Hz increments
  OCR2A = 1561;             // set counter up to 1561
  TCCR2A |= (1 << WGM21);  // turn on CTC mode
  // setting prescaler with 1024
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
  TIMSK2 |= (1 << OCIE2A);  // enable timer2 compare interrupt
  interrupts();             // enable all interrupts
  Serial.begin(115200);     //  setup serial
  while (!Serial) {
    ;  // Aguarda até que a porta serial esteja pronta - normalmente 10 ms
  }
  publica_status();
  Serial.println("Modo \"verbouse\" ativado.");
  Serial.println("Aguardando comandos...");
  BeepOk();
  Serial.println(" ");
}


void limita_valor(int *valor) {
  // Bloco "Saturador", trabalha com variáveis int
  // altera diretamente conteúdo da variável valor;
  // passagem de parâmetros, sem cópia, por referência
  // uso: limita_valour(&duty);
  if (*valor > MAX_U) *valor = MAX_U;
  if (*valor < 0) *valor = 0;
}

// tratamento de interrupção - algoritmo de controle (timer2)
ISR(TIMER2_COMPA_vect) {  //timer2 interrupt @ 100 Hz
  // atualiza seção do led monitor (o que pisca)
  ledCounter++;
  if (ledCounter >= 50) {
    ledState = !ledState;
    digitalWrite(LED_MONITOR, ledState);
    digitalWrite(LED_BUILTIN, ledState);  // faz piscar led da placa do Arduino
    ledCounter = 0;
  }
  // inicia seção da lei de controle
  pv = analogRead(analogPin);  // y[k] = info do sensor (A/D de 12-bits: [0..4095])
  if (control) {
    // processa lei de controle
    float e = sp - pv;  // e[k] = r[k] - y[k]
    // ação proporcional - sempre ativada!
    u = Kp * e;
    // saída do sinal para processo: valor atualizado
  }
  mv = (int)u;  // mv é a valor efetivamente jogado para "fora" da placa, via DAC ou PWM
  limita_valor(&mv);
  analogWrite(pwm, mv);  // valores entre 0 ~ 255 (0 ~ 100%) de dutty-cycle
}

void reinit_auto(void) {
  // atualiza variáveis quando sai do modo manual para automático
  u = 0;
  control = true;
  verbouse = false;  // automaticamente desativa modo verbouse; passa a publicar dados do sistema
  Serial.println(" ");
  Serial.println("Ativando Modo AUTOmático:");
  Serial.println("sp: r[k],\tpv: y[k],\tmv: u[k]");
  Serial.println(" ");
}

void Update_bool_variable(bool *bool_var) {
  if (numericIndex > 0) {
    // significa que usuário informou algum "valor"
    if (value <= 0)
      *bool_var = false;  // valores nulos e negativos --> false
    else
      *bool_var = true;  // valores positivos --> true
  } else {
    *bool_var = !(*bool_var);  // alterna modo da variável booleana
  }
}

void process_instruction(char option, float value) {
  bool previous_state = false;
  switch (option) {
    case 'v':  // toggle modo verbouse, ou "modo quieto"
      Update_bool_variable(&verbouse);
      publica_estado("verbouse", verbouse);
      break;
    case 'm':  // toggle no modo de feedback auditivo
      Update_bool_variable(&mute);
      publica_estado("mute", mute);
      break;
    case 'a':  // comuta controle automático para manual
      previous_state = control;
      Update_bool_variable(&control);
      if (control != previous_state) reinit_auto();
      publica_estado("control", control);
      break;
    case 's':  // publica status do sistema
      publica_status();
      break;
    case 'p':  // ajusta ganho proporcional Kp
      Kp = value;
      publica_parametro("Kp", Kp);
      break;
    case 'r':  // ajusta set-point
      sp = value;
      publica_parametro("sp", sp);
      break;
    case 'u':  // ajusta variável manipulada (em modo manual)
      if (control) {
        control = false;
        publica_estado("control", control);
      }
      u = value;
      publica_parametro("u", u);
      break;
    default:  // comando não reconhecido
      Serial.print(option);
      Serial.println(" : Comando não reconhecido.");
      BeepErro();
      break;
  }
  if (!mute) BeepOk();
}

void process_token() {
  // necessário: option (char) e value (float)
  String number;
  option = inputString.charAt(0);
  number = inputString.substring(1);
  number.trim();
  if (number.length() > 0) {
    value = number.toFloat();
    numericIndex = number.length();
  } else {
    value = 0;
    numericIndex = 0;
  }
  if (verbouse) {
    Serial.print("Recebido: ");
    Serial.print(option);
    if (numericIndex > 0) {
      Serial.print(" ");
      Serial.println(value);
    } else {
      Serial.println(" ");
    }
  }
  process_instruction(option, value);
  inputString = "";
}

void loop() {
  // checa por dados na porta serial, quando disponíveis, lê a string e a trata
  if (Serial.available()) {
    inputString = Serial.readString();
    inputString.trim();  // retira possíveis espaços
    if (inputString != "") {
      process_token();
    }
  }
  if (!control) {
    if ((millis() - lastTime) > period) {
      // publica estado de variáveis...
      Serial.print("sp: ");
      Serial.print(sp, 2);
      Serial.print(", pv: ");
      Serial.print(pv, 2);
      Serial.print(", u: ");
      Serial.println(u, 2);
      lastTime = millis();
    }
  }
}
