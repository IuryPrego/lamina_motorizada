/*
 * Firmware da lâmina motorizada — vários eixos
 * Arduino Uno + CNC Shield v3 (eixos X, Y, Z e A — A independente em D12/D13)
 *
 * Perfil de movimento próprio com VELOCIDADE MÍNIMA: o motor arranca direto
 * em velMin, acelera até velMax e desacelera só até velMin antes de parar.
 * Assim ele nunca passa pela faixa de baixa velocidade, onde vibra.
 *
 * Protocolo serial (115200 baud, uma linha por comando, terminada em '\n').
 * Toda linha começa com a letra do eixo, e as respostas voltam com a mesma
 * letra na frente:
 *
 *   X G 45    → "X OK 45.00" na hora e "X DONE 45.00" quando parar
 *   Y P       → "Y POS 12.30"
 *
 * Comandos (depois da letra do eixo):
 *
 *   G <ang>   vai para o ângulo absoluto <ang> (graus da lâmina)
 *             → "OK <ang_real>" na hora e "DONE <ang>" quando parar
 *   S         para com desaceleração          → "OK" e depois "DONE <ang>"
 *   P         consulta a posição              → "POS <ang>"
 *   Z         define a posição atual como 0°  → "POS 0.00"
 *   D         desabilita o driver (sem torque) → "OK"
 *   H         habilita o driver (segura)       → "OK"
 *   M <v>     velocidade mínima, passos/s      → "OK"
 *   V <v>     velocidade máxima, passos/s      → "OK"
 *   A <a>     aceleração, passos/s²            → "OK"
 *   T <ms>    desliga o driver <ms> após parar (0 = segura sempre) → "OK"
 *   R         referencia no fim de curso (zero mecânico)
 *             → "OK" na hora e "DONE <ang>" ao terminar (ou "ERR ...")
 *   F         lê o fim de curso                → "FIM 1" (acionado) ou "FIM 0"
 *   I         configuração atual → "INFO <passos/grau> <velMin> <velMax> <acel>
 *             <desligar_ms> <ang_min> <ang_max> <vel_limite>"
 *
 * Comando geral (sem eixo):
 *
 *   ?         lista os eixos                   → "READY X Y"
 *
 * G, Z, D, M, V, A e R só são aceitos com AQUELE eixo parado; os outros
 * continuam andando normalmente, inclusive durante uma referência.
 * Durante a referência, S cancela.
 * Erros respondem "<eixo> ERR <motivo>" ("? ERR ..." quando não dá para saber
 * o eixo). Na inicialização envia "READY" seguido dos eixos, ex. "READY X Y".
 */

#include <Arduino.h>
#include <stdlib.h>
#include <math.h>

// ── Eixos ───────────────────────────────────────────────────────────────────
// Uma linha por eixo em EIXOS_CFG. Para ligar Z e A é só descomentar a linha
// deles: o resto do firmware e a biblioteca Python se ajustam sozinhos.
//
// Pinos do CNC Shield v3:   STEP  DIR  fim de curso
//   X                         2    5    9   (X- / X+)
//   Y                         3    6   10   (Y- / Y+)
//   Z                         4    7   11   (Z- / Z+)
//   A                        12   13   A3   (jumpers do "A" em D12/D13, e SÓ
//                                            neles; o fim de curso vai no
//                                            conector "CoolEn" = A3. Não use
//                                            SpinEn/SpinDir: são D12/D13)
//
// O ENABLE (pino 8) é um só para todos os drivers do shield: ele só desliga
// de fato quando todos os eixos ligados a ele pedem para desligar.
//
// microstep tem que bater com os jumpers MS1/MS2/MS3 embaixo do driver
// daquele eixo (1 = ON, 0 = OFF):
//
//            A4988            DRV8825
//   1        0 0 0            0 0 0
//   2        1 0 0            1 0 0
//   4        0 1 0            0 1 0
//   8        1 1 0            1 1 0
//   16       1 1 1            0 0 1
//   32       —                1 0 1   (ou 1 1 1)
//
// Ao trocar, as velocidades se ajustam sozinhas (mesma rotação real).
//
// reducao     = voltas do motor por volta da lâmina
// inverter    = true: mesmo sentido do código Firmata antigo
//               (DIR em LOW → ângulo positivo)
// sentido_ref = +1: a referência procura o fim de curso nos ângulos positivos
struct ConfigEixo {
  char    nome;                // letra usada nos comandos ("X G 45")
  uint8_t pinoStep, pinoDir, pinoEnable, pinoFim;
  float   microstep;
  float   reducao;
  bool    inverterDirecao;
  int8_t  sentidoReferencia;
};

const ConfigEixo EIXOS_CFG[] = {
  // nome STEP DIR EN FIM  microstep reducao inverter sentido_ref
  {  'X',   2,  5, 8,  9,    8.0f,    5.0f,   true,     +1 },
  {  'Y',   3,  6, 8, 10,    8.0f,    5.0f,   true,     +1 },
  {  'Z',   4,  7, 8, 11,    8.0f,    5.0f,   true,     +1 },
  {  'A',  12, 13, 8, A3,    8.0f,    5.0f,   true,     +1 },
};
constexpr uint8_t N_EIXOS = sizeof(EIXOS_CFG) / sizeof(EIXOS_CFG[0]);

constexpr float PASSOS_POR_VOLTA_MOTOR = 200.0f;   // passos inteiros, sem microstep

// Ligação do fim de curso (COM no GND, outro terminal no sinal), vale para
// todos os eixos:
//   true  = contato NF (normalmente fechado) — recomendado: fio rompido
//           é lido como "acionado" e a referência para em vez de seguir
//   false = contato NA (normalmente aberto)
constexpr bool FIM_CURSO_NF = true;

// ── Movimento ───────────────────────────────────────────────────────────────
// Os padrões são dados em passos inteiros (sem microstep) e multiplicados
// pelo microstep de cada eixo; acima de VEL_LIMITE o Uno não consegue gerar
// os pulsos. Com vários eixos andando juntos, o que fica limitado é a SOMA
// das velocidades: acima disso os eixos só andam mais devagar que o pedido.
constexpr float VEL_LIMITE     = 5000.0f;  // teto prático num Uno, passos/s
constexpr float limitarVel(float v) { return (v > VEL_LIMITE) ? VEL_LIMITE : v; }

constexpr float VEL_MIN_PADRAO = 150.0f;   // nunca anda mais devagar
constexpr float VEL_MAX_PADRAO = 600.0f;
constexpr float ACEL_PADRAO    = 1000.0f;  // passos/s²

// Desliga o driver este tempo depois de parar (0 = mantém sempre ligado)
constexpr unsigned long DESLIGAR_PARADO_MS = 0;

// ── Referência (homing) ─────────────────────────────────────────────────────
constexpr float ANGULO_FIM_CURSO  = 0.0f;    // ângulo da lâmina quando o sensor aciona
constexpr float BUSCA_MAX_GRAUS   = 3600.0f; // desiste se não achar o sensor (10 voltas)
constexpr float RECUO_GRAUS       = 3.0f;    // recuo antes da aproximação lenta
constexpr float LIBERAR_MAX_GRAUS = 360.0f;  // quanto anda tentando sair do sensor (1 volta)
constexpr float VEL_REF_LENTA     = 50.0f;   // passos inteiros/s na aproximação final

// ── Limites de segurança da lâmina ──────────────────────────────────────────
constexpr float ANGULO_MIN = -3600.0f;   // 10 voltas
constexpr float ANGULO_MAX =  3600.0f;

constexpr unsigned long BAUD = 115200;

// ── Estado ──────────────────────────────────────────────────────────────────
// Fases da referência, na ordem em que acontecem.
enum Fase : uint8_t {
  NENHUMA,         // não está referenciando
  SAIR_INICIAL,    // começou em cima do sensor: sai dele
  RECUO_INICIAL,
  BUSCA,           // procura o sensor na velocidade mínima
  SAIR,            // sai do sensor e recua...
  RECUO,
  APROXIMACAO,     // ...e volta devagar, para o ponto ser repetível
};

struct Eixo {
  const ConfigEixo *cfg = nullptr;
  float passosPorGrau = 1.0f;

  float velMin = 0.0f;                      // passos/s
  float velMax = 0.0f;                      // passos/s
  float acel   = 0.0f;                      // passos/s²

  long          posicao     = 0;            // passos
  long          alvo        = 0;            // passos
  int8_t        sentido     = 1;            // +1 ou -1
  float         vel         = 0.0f;         // passos/s
  unsigned long intervaloUs = 0;
  unsigned long ultimoPasso = 0;
  bool          movendo     = false;
  bool          avisarFim   = false;        // envia DONE quando parar
  bool          querLigado  = false;        // pedido deste eixo para o ENABLE

  unsigned long desligarMs       = DESLIGAR_PARADO_MS;
  unsigned long paradoDesde      = 0;
  bool          desligarPendente = false;

  Fase    fase         = NENHUMA;
  long    passosFase   = 0;                 // quanto ainda pode andar nesta fase
  int8_t  esperaFim    = -1;                // -1 nada, 0 liberado, 1 acionado
  uint8_t confirmacoes = 0;
};

Eixo eixos[N_EIXOS];

char    linha[32];
uint8_t tamLinha   = 0;
bool    linhaLonga = false;

// ── Utilidades ──────────────────────────────────────────────────────────────
static float anguloAtual(const Eixo &e) { return e.posicao / e.passosPorGrau; }

static bool ocupado(const Eixo &e) { return e.movendo || e.fase != NENHUMA; }

static void prefixo(const Eixo &e) {
  Serial.print(e.cfg->nome);
  Serial.print(' ');
}

static void enviarAngulo(const Eixo &e, const __FlashStringHelper *txt, float angulo) {
  prefixo(e);
  Serial.print(txt);
  Serial.println(angulo, 2);
}

static void enviarOk(const Eixo &e) {
  prefixo(e);
  Serial.println(F("OK"));
}

static void erro(const Eixo &e, const __FlashStringHelper *motivo) {
  prefixo(e);
  Serial.print(F("ERR "));
  Serial.println(motivo);
}

static void listarEixos() {
  Serial.print(F("READY"));
  for (const Eixo &e : eixos) {
    Serial.print(' ');
    Serial.print(e.cfg->nome);
  }
  Serial.println();
}

// O ENABLE pode ser dividido entre eixos: o pino fica ligado enquanto algum
// deles quiser. Retorna true se o driver acabou de ser ligado.
static bool habilitar(Eixo &e, bool ligar) {
  e.querLigado = ligar;
  const uint8_t pino = e.cfg->pinoEnable;
  bool algum = false;
  for (const Eixo &o : eixos)
    if (o.cfg->pinoEnable == pino && o.querLigado) algum = true;
  const bool estava = digitalRead(pino) == LOW;   // ativo em LOW
  digitalWrite(pino, algum ? LOW : HIGH);
  return algum && !estava;
}

// Lê um número que ocupa o resto da linha; rejeita "abc" e "90x".
static bool lerNumero(const char *txt, float &valor) {
  char *fim;
  double v = strtod(txt, &fim);
  if (fim == txt) return false;
  while (*fim == ' ') fim++;
  if (*fim != '\0') return false;
  valor = (float)v;
  return true;
}

// ── Perfil de movimento ─────────────────────────────────────────────────────
static float distanciaFreio(const Eixo &e) {
  // passos necessários para ir de vel até velMin com a aceleração atual
  return (e.vel * e.vel - e.velMin * e.velMin) / (2.0f * e.acel);
}

static void definirSentido(Eixo &e, int8_t s) {
  e.sentido = s;
  digitalWrite(e.cfg->pinoDir, ((s > 0) != e.cfg->inverterDirecao) ? HIGH : LOW);
  delayMicroseconds(5);                      // tempo de preparo do DIR
}

static void darPasso(Eixo &e) {
  digitalWrite(e.cfg->pinoStep, HIGH);
  delayMicroseconds(3);
  digitalWrite(e.cfg->pinoStep, LOW);
  e.posicao += e.sentido;
}

// Hora do próximo passo? Mantém a cadência; se atrasou demais, não tenta
// compensar em rajada.
static bool horaDoPasso(Eixo &e) {
  const unsigned long agora = micros();
  if (agora - e.ultimoPasso < e.intervaloUs) return false;
  e.ultimoPasso = (agora - e.ultimoPasso > 2 * e.intervaloUs) ? agora : e.ultimoPasso + e.intervaloUs;
  return true;
}

static void iniciarMovimento(Eixo &e, long novoAlvo) {
  e.alvo = novoAlvo;
  if (e.alvo == e.posicao) return;           // já está lá; DONE sai no loop

  definirSentido(e, (e.alvo > e.posicao) ? 1 : -1);

  e.vel = e.velMin;                          // arranca direto na mínima
  e.intervaloUs = (unsigned long)(1e6f / e.vel);
  e.ultimoPasso = micros() - e.intervaloUs;  // primeiro passo imediato
  e.movendo = true;
}

static void parar(Eixo &e) {
  if (!e.movendo) return;
  const long passosFreio = (long)ceilf(distanciaFreio(e));
  if (passosFreio < labs(e.alvo - e.posicao)) e.alvo = e.posicao + e.sentido * passosFreio;
  if (e.alvo == e.posicao) { e.movendo = false; e.vel = 0.0f; }
}

static void atualizarMotor(Eixo &e) {
  if (!e.movendo || !horaDoPasso(e)) return;

  darPasso(e);

  const long restante = labs(e.alvo - e.posicao);
  if (restante == 0) { e.movendo = false; e.vel = 0.0f; return; }

  // a · dt, com dt = intervalo do passo (multiplicação: sobra tempo para os outros eixos)
  const float dv = e.acel * (e.intervaloUs * 1e-6f);
  if ((float)restante <= distanciaFreio(e)) e.vel -= dv;
  else if (e.vel < e.velMax)                e.vel += dv;

  if (e.vel < e.velMin) e.vel = e.velMin;
  if (e.vel > e.velMax) e.vel = e.velMax;
  e.intervaloUs = (unsigned long)(1e6f / e.vel);
}

// ── Fim de curso e referência ───────────────────────────────────────────────
static bool leituraFim(const Eixo &e) {
  // Com pull-up: contato fechado = LOW, aberto = HIGH.
  // NF abre quando acionado; NA fecha quando acionado.
  const bool aberto = digitalRead(e.cfg->pinoFim) == HIGH;
  return FIM_CURSO_NF ? aberto : !aberto;
}

// Duas leituras iguais seguidas, para ignorar ruído e repique do contato.
static bool fimEsta(const Eixo &e, bool acionado) {
  if (leituraFim(e) != acionado) return false;
  delayMicroseconds(200);
  return leituraFim(e) == acionado;
}

// A referência roda em fases, um passo por vez dentro do loop, para não
// travar os outros eixos nem a serial.
static void iniciarFase(Eixo &e, Fase fase) {
  const long recuo   = lround(RECUO_GRAUS * e.passosPorGrau);
  const long liberar = lround(LIBERAR_MAX_GRAUS * e.passosPorGrau);
  const int8_t ref   = e.cfg->sentidoReferencia;
  float v = e.velMin;

  switch (fase) {
    case SAIR_INICIAL:
    case SAIR:
      definirSentido(e, -ref);
      e.esperaFim  = 0;                      // até liberar
      e.passosFase = liberar;
      break;
    case RECUO_INICIAL:
    case RECUO:
      definirSentido(e, -ref);
      e.esperaFim  = -1;                     // anda o recuo inteiro
      e.passosFase = recuo;
      break;
    case BUSCA:
      definirSentido(e, ref);
      e.esperaFim  = 1;
      e.passosFase = lround(BUSCA_MAX_GRAUS * e.passosPorGrau);
      break;
    case APROXIMACAO: {
      const float lenta = VEL_REF_LENTA * e.cfg->microstep;
      if (lenta < v) v = lenta;
      definirSentido(e, ref);
      e.esperaFim  = 1;
      // precisa cobrir a folga do sensor (que pode chegar a "liberar") + o recuo
      e.passosFase = liberar + recuo * 2;
      break;
    }
    default:
      return;
  }
  e.fase = fase;
  e.confirmacoes = 0;
  e.intervaloUs = (unsigned long)(1e6f / v);
  e.ultimoPasso = micros();
}

static void terminarReferencia(Eixo &e, const __FlashStringHelper *falha) {
  e.fase = NENHUMA;
  if (falha) {
    e.alvo = e.posicao;
    erro(e, falha);
    e.paradoDesde = millis();
    e.desligarPendente = (e.desligarMs > 0);
  } else {
    e.posicao = e.alvo = lround(ANGULO_FIM_CURSO * e.passosPorGrau);
    e.avisarFim = true;                      // DONE sai no loop
  }
}

static void avancarFase(Eixo &e) {
  if (e.fase == APROXIMACAO) terminarReferencia(e, nullptr);
  else                       iniciarFase(e, (Fase)(e.fase + 1));
}

static const __FlashStringHelper *falhaDaFase(Fase fase) {
  switch (fase) {
    case BUSCA:       return F("fim de curso nao encontrado");
    case APROXIMACAO: return F("fim de curso instavel");
    default:          return F("fim de curso nao libera (fio solto?)");
  }
}

static void atualizarReferencia(Eixo &e) {
  if (!horaDoPasso(e)) return;

  if (e.esperaFim >= 0) {
    // Duas leituras iguais em passos seguidos, para ignorar ruído e repique
    // do contato; enquanto confirma, não anda.
    if (leituraFim(e) == (e.esperaFim == 1)) {
      if (++e.confirmacoes >= 2) avancarFase(e);
      return;
    }
    e.confirmacoes = 0;
  }

  if (e.passosFase <= 0) {
    if (e.esperaFim >= 0) terminarReferencia(e, falhaDaFase(e.fase));
    else                  avancarFase(e);
    return;
  }

  darPasso(e);
  e.passosFase--;
}

// ── Comandos ────────────────────────────────────────────────────────────────
static void processarComando(Eixo &e, const char *cmdTxt) {
  const char cmd = toupper(*cmdTxt);
  const char *arg = cmdTxt + 1;
  float valor;

  if (ocupado(e) && cmd != 'S' && cmd != 'P' && cmd != 'H' && cmd != 'T' && cmd != 'F' && cmd != 'I') {
    erro(e, F("em movimento"));
    return;
  }

  switch (cmd) {
    case 'G': {
      if (!lerNumero(arg, valor))                   { erro(e, F("angulo invalido")); return; }
      if (valor < ANGULO_MIN || valor > ANGULO_MAX) { erro(e, F("fora dos limites")); return; }
      // Só espera se o pino acabou de ligar — aí nenhum eixo dele está andando.
      if (habilitar(e, true)) delay(2);          // corrente estabiliza nas bobinas
      e.desligarPendente = false;
      const long novoAlvo = lround(valor * e.passosPorGrau);
      iniciarMovimento(e, novoAlvo);
      e.avisarFim = true;
      enviarAngulo(e, F("OK "), novoAlvo / e.passosPorGrau);
      break;
    }

    case 'S':
      if (e.fase != NENHUMA) terminarReferencia(e, F("referencia cancelada"));
      else                   parar(e);
      e.avisarFim = true;                        // sempre responde DONE
      enviarOk(e);
      break;

    case 'P':
      enviarAngulo(e, F("POS "), anguloAtual(e));
      break;

    case 'Z':
      e.posicao = e.alvo = 0;
      enviarAngulo(e, F("POS "), 0.0f);
      break;

    case 'D':
      habilitar(e, false);
      enviarOk(e);
      break;

    case 'H':
      habilitar(e, true);
      e.desligarPendente = false;                // segura até o próximo movimento
      enviarOk(e);
      break;

    case 'M':
      if (!lerNumero(arg, valor) || valor <= 0 || valor > e.velMax) {
        erro(e, F("minima invalida (0 < M <= V)")); return;
      }
      e.velMin = valor;
      enviarOk(e);
      break;

    case 'V':
      if (!lerNumero(arg, valor) || valor < e.velMin || valor > VEL_LIMITE) {
        erro(e, F("maxima invalida (M <= V <= 5000)")); return;
      }
      e.velMax = valor;
      enviarOk(e);
      break;

    case 'A':
      if (!lerNumero(arg, valor) || valor <= 0) { erro(e, F("aceleracao invalida")); return; }
      e.acel = valor;
      enviarOk(e);
      break;

    case 'R':
      if (habilitar(e, true)) delay(2);
      e.desligarPendente = false;
      enviarOk(e);
      iniciarFase(e, fimEsta(e, true) ? SAIR_INICIAL : BUSCA);
      break;

    case 'F':
      prefixo(e);
      Serial.print(F("FIM "));
      Serial.println(fimEsta(e, true) ? 1 : 0);
      break;

    case 'T':
      if (!lerNumero(arg, valor) || valor < 0 || valor > 600000) {
        erro(e, F("tempo invalido")); return;
      }
      e.desligarMs = (unsigned long)valor;
      enviarOk(e);
      break;

    case 'I':
      prefixo(e);
      Serial.print(F("INFO "));
      Serial.print(e.passosPorGrau, 4); Serial.print(' ');
      Serial.print(e.velMin, 2);        Serial.print(' ');
      Serial.print(e.velMax, 2);        Serial.print(' ');
      Serial.print(e.acel, 2);          Serial.print(' ');
      Serial.print(e.desligarMs);       Serial.print(' ');
      Serial.print(ANGULO_MIN, 2);      Serial.print(' ');
      Serial.print(ANGULO_MAX, 2);      Serial.print(' ');
      Serial.println(VEL_LIMITE, 2);
      break;

    default:
      erro(e, F("comando desconhecido"));
  }
}

// "<eixo> <comando>", ou "?" para listar os eixos.
static void processarLinha(char *txt) {
  while (*txt == ' ') txt++;
  if (*txt == '\0') return;                  // linha vazia
  if (*txt == '?') { listarEixos(); return; }

  const char nome = toupper(*txt++);
  for (Eixo &e : eixos) {
    if (e.cfg->nome != nome) continue;
    while (*txt == ' ') txt++;
    processarComando(e, txt);
    return;
  }
  Serial.println(F("? ERR eixo desconhecido"));
}

// Leitura não bloqueante: nunca segura o motor esperando a serial.
static void lerSerial() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      if (linhaLonga) {
        Serial.println(F("? ERR linha longa"));
      } else {
        linha[tamLinha] = '\0';
        processarLinha(linha);
      }
      tamLinha = 0;
      linhaLonga = false;
    } else if (!linhaLonga) {
      if (tamLinha < sizeof(linha) - 1) linha[tamLinha++] = c;
      else                              linhaLonga = true;
    }
  }
}

// ────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(BAUD);

  for (uint8_t i = 0; i < N_EIXOS; i++) {
    const ConfigEixo &c = EIXOS_CFG[i];
    Eixo &e = eixos[i];
    e.cfg = &c;
    e.passosPorGrau = PASSOS_POR_VOLTA_MOTOR * c.microstep * c.reducao / 360.0f;
    e.velMin = limitarVel(VEL_MIN_PADRAO * c.microstep);
    e.velMax = limitarVel(VEL_MAX_PADRAO * c.microstep);
    e.acel   = ACEL_PADRAO * c.microstep;

    pinMode(c.pinoStep, OUTPUT);
    pinMode(c.pinoDir, OUTPUT);
    pinMode(c.pinoEnable, OUTPUT);
    pinMode(c.pinoFim, INPUT_PULLUP);
    digitalWrite(c.pinoStep, LOW);
    digitalWrite(c.pinoEnable, HIGH);            // liga no primeiro G ou R
  }

  listarEixos();                                 // "READY X Y"
}

void loop() {
  for (Eixo &e : eixos) {
    if (e.fase != NENHUMA) atualizarReferencia(e);
    else                   atualizarMotor(e);
  }

  lerSerial();

  for (Eixo &e : eixos) {
    if (e.avisarFim && !ocupado(e)) {
      e.avisarFim = false;
      enviarAngulo(e, F("DONE "), anguloAtual(e));
      e.paradoDesde = millis();
      e.desligarPendente = (e.desligarMs > 0);
    }

    if (e.desligarPendente && !ocupado(e) && millis() - e.paradoDesde >= e.desligarMs) {
      e.desligarPendente = false;
      habilitar(e, false);
    }
  }
}
