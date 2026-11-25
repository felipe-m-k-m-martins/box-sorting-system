// Separadora de caixas: firmware do ESP32 (placa PAT-DAELN v2.1)
//
// O ESP32 decide tudo: maquina de estados, classificacao, esteira, emergencia,
// servidor web, OLED, LED RGB e registro de eventos. O Arduino Mega com a RAMPS 1.4
// executa os movimentos dos motores e devolve o estado dos sensores por I2C.
//
// Nucleo 0 (TaskControle): maquina de estados, classificacao e I2C com o Mega.
// Nucleo 1 (TaskComunicacao): Wi-Fi, servidor web, OLED, LED RGB e log.
// So o nucleo 0 usa o barramento I2C. O estado compartilhado e protegido por mutex,
// e as mensagens de log vao do nucleo 0 para o nucleo 1 por uma fila.
//
// I2C com o Mega (endereco 0x08):
//   ESP32 -> Mega: pp/sp e pp2/sp2 (bracos 1 e 2), ce/cd (esteira), em/em0 (emergencia)
//   Mega -> ESP32: 1 byte com S1, S2, S3, fim de curso, heartbeat e sensor de alarme

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Rede Wi-Fi: preencher antes de gravar
const char* ssid     = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

#define NOME_GRUPO "Gabi/Felipe/Lucas"

#define MEGA_I2C_ADDR 0x08
WebServer server(80);
Adafruit_SSD1306 oled(128, 64, &Wire, -1);

// LED RGB da placa (R e B invertidos na fiacao da PAT-DAELN)
#define LEDR_PIN 19
#define LEDG_PIN 18
#define LEDB_PIN 5
// Habilita a alimentacao dos LEDs; R, G e B ficam em LOW antes de ligar
#define TLED_PIN 26

// Bits do byte de sensores enviado pelo Mega
#define BIT_S1        0x01
#define BIT_S2        0x02
#define BIT_S3        0x04
#define BIT_FIMCURSO  0x08
#define BIT_HEARTBEAT 0x10
#define BIT_ALARME    0x20

// Buzzer e LM35: alarme a partir de 45 C, normaliza em 40 C
#define BUZZ_PIN 23
#define TEMP_PIN 36
#define LIMITE_TEMP_ALTA   45.0
#define LIMITE_TEMP_NORMAL 40.0

// Botoes da placa: S4 aciona a emergencia (interrupcao), S6 faz o reset
#define BTN_S4_EMERGENCIA_PIN 32
#define BTN_S6_RESET_PIN      25
#define BTN_DEBOUNCE_MS       50

// Sirene: o intervalo entre liga e desliga do buzzer varia entre MIN e MAX ao longo do ciclo
#define SIRENE_CICLO_MS       1400
#define SIRENE_TOGGLE_MIN_MS  40
#define SIRENE_TOGGLE_MAX_MS  180

// Tempo fixo de cada movimento dos bracos (o Mega nao avisa quando termina)
#define DURACAO_B1_MS        11000
#define DURACAO_B2_PEGAR_MS  11000
#define DURACAO_B2_SOLTAR_MS 11000

// Estados do ciclo automatico e do teste da esteira
enum EstadoSistema {
  EST_OFF, EST_IDLE, EST_B1, EST_ESTEIRA, EST_FIM_CURSO,
  EST_B2_PEGAR, EST_B2_SOLTAR, EST_CICLO_DONE, EST_EMERGENCIA
};

enum EstadoTeste {
  TESTE_OFF, TESTE_LIGADO, TESTE_FINALIZANDO, TESTE_CONCLUIDO
};

// Estado compartilhado entre os nucleos, acessado sempre com mutexEstado
struct EstadoCompartilhado {
  bool sistemaLigado   = false;
  bool cicloAtivo      = false;
  bool emergenciaAtiva = false;
  bool tipoCicloLoop   = false;
  int  maxPallets      = 10;

  EstadoSistema estado = EST_OFF;

  long contadorTotal = 0, contadorP = 0, contadorM = 0, contadorG = 0;
  String categoriaAtual = "-";
  String manip1Status = "Parado";
  String manip2Status = "Parado";
  bool esteiraLigadaFlag = false;

  String rgbAtual = "azul";
  unsigned long heartbeat = 0;

  bool latchS1 = false, latchS2 = false, latchS3 = false;
  bool fimCursoPulso = false;

  float temperaturaAtual = 24.0;
  bool  temperaturaAlerta = false;

  bool testeAtivo = false;
  EstadoTeste estadoTeste = TESTE_OFF;

  bool latchTesteS1 = false, latchTesteS2 = false, latchTesteS3 = false;

  bool rawS1 = false, rawS2 = false, rawS3 = false, rawFimCurso = false;
  bool rawAlarme = false;
  bool megaComunicando = false;
  bool megaHeartbeatMudou = true;

  bool resetFlagsPendente = false;

  bool pedidoResetEmergencia = false;
};

EstadoCompartilhado estadoGlobal;
SemaphoreHandle_t mutexEstado;

// Log de eventos: fila entre os nucleos e buffer exibido na pagina web
QueueHandle_t filaLog;
struct LogMsg { char texto[64]; char nivel[8]; };

#define LOG_BUFFER_MAX 30
struct LogEntry { String hora; String texto; String nivel; };
LogEntry logBuffer[LOG_BUFFER_MAX];
int logCount = 0;

// Bordas detectadas nos sensores (usadas so pelo nucleo 0)
bool flagFimCurso = false;
bool flagTorreDetectada = false;

// Interrupcao do botao S4
volatile bool flagEmergenciaISR = false;
volatile unsigned long tUltimaEmergenciaISR = 0;

void IRAM_ATTR isrEmergenciaS4() {
  unsigned long agora = millis();
  if (agora - tUltimaEmergenciaISR > BTN_DEBOUNCE_MS) {
    tUltimaEmergenciaISR = agora;
    flagEmergenciaISR = true;
  }
}

// Sensor de alarme lido pelo Mega
bool flagAlarmeD35 = false;

// Timer de 1 s usado como heartbeat
hw_timer_t* timerPeriodico = NULL;
void IRAM_ATTR isrTimerPeriodico() {
  estadoGlobal.heartbeat++;
}

void addLog(const char* msg, const char* nivel) {
  LogMsg m;
  strncpy(m.texto, msg, sizeof(m.texto) - 1); m.texto[sizeof(m.texto)-1] = '\0';
  strncpy(m.nivel, nivel, sizeof(m.nivel) - 1); m.nivel[sizeof(m.nivel)-1] = '\0';
  xQueueSend(filaLog, &m, 0);
}

String horaAtualStr() {
  unsigned long s = millis() / 1000;
  char buf[10];
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", (s/3600)%24, (s/60)%60, s%60);
  return String(buf);
}

// R e B sao ligados direto; G usa PWM para compor o amarelo
#define RGB_PWM_FREQ 5000
#define RGB_PWM_RESOLUCAO 8
#define RGB_MAX 255
#define RGB_AMARELO_VERDE 210

void setRGB(const char* cor) {
  int g = 0;
  bool r = false, b = false;

  if      (strcmp(cor,"verde")   ==0) { g = RGB_MAX; }
  else if (strcmp(cor,"azul")    ==0) { b = true; }
  else if (strcmp(cor,"amarelo") ==0) { r = true; g = RGB_AMARELO_VERDE; }
  else if (strcmp(cor,"vermelho")==0) { r = true; }

  digitalWrite(LEDR_PIN, r ? HIGH : LOW);
  digitalWrite(LEDB_PIN, b ? HIGH : LOW);
  ledcWrite(LEDG_PIN, g);
}

// I2C com o Mega (somente no nucleo 0)
void enviaComandoMega(const char* cmd) {
  Wire.beginTransmission(MEGA_I2C_ADDR);
  Wire.write((const uint8_t*)cmd, strlen(cmd));
  Wire.endTransmission();
}

bool lerByteSensoresMega(byte &resultado) {
  Wire.requestFrom(MEGA_I2C_ADDR, 1);
  if (Wire.available()) {
    resultado = Wire.read();
    return true;
  }
  return false;
}

// Leitura com debounce: 5 amostras seguidas; falha de I2C e repetida ate 3 vezes
bool lerSensorEstavelMega(byte bitMask) {
  const int AMOSTRAS = 5;
  const int MAX_RETRY_POR_AMOSTRA = 3;
  for (int i = 0; i < AMOSTRAS; i++) {
    byte b;
    bool leituraOk = false;
    for (int r = 0; r <= MAX_RETRY_POR_AMOSTRA; r++) {
      if (lerByteSensoresMega(b)) { leituraOk = true; break; }
      delayMicroseconds(200);
    }
    if (!leituraOk) {
      addLog("Falha I2C persistente no debounce de sensor", "error");
      return false;
    }
    if (!(b & bitMask)) return false;
    delayMicroseconds(500);
  }
  return true;
}

// Le o byte de sensores, atualiza a copia protegida por mutex e detecta bordas.
// Um evento so conta depois de 3 leituras seguidas (2 para o alarme).
void pollSensoresMega() {
  static bool prevS1 = false, prevS2 = false, prevS3 = false, prevFimCurso = false;
  static bool prevHeartbeat = false;
  static int falhasConsecutivas = 0;

  static int fcEstavelCount = 0, torreEstavelCount = 0, alarmeEstavelCount = 0;
  static bool fcEstavelAntes = false, torreEstavelAntes = false, alarmeEstavelAntes = false;
  const int LIMITE_LEITURAS_ESTAVEIS = 3;
  const int LIMITE_ALARME_ESTAVEL = 2;

  byte b;
  bool ok = lerByteSensoresMega(b);

  bool s1, s2, s3, fc, hb, alarme;
  if (ok) {
    s1 = b & BIT_S1;
    s2 = b & BIT_S2;
    s3 = b & BIT_S3;
    fc = b & BIT_FIMCURSO;
    hb = b & BIT_HEARTBEAT;
    alarme = b & BIT_ALARME;
    falhasConsecutivas = 0;
  } else {
    s1 = prevS1; s2 = prevS2; s3 = prevS3; fc = prevFimCurso; hb = prevHeartbeat;
    alarme = false;
    falhasConsecutivas++;
  }

  bool torreAgora = s1 || s2 || s3;
  if (torreAgora) { torreEstavelCount++; if (torreEstavelCount > 100) torreEstavelCount = 100; }
  else             { torreEstavelCount = 0; }
  bool torreEstavel = (torreEstavelCount >= LIMITE_LEITURAS_ESTAVEIS);
  if (torreEstavel && !torreEstavelAntes) flagTorreDetectada = true;
  torreEstavelAntes = torreEstavel;

  if (fc) { fcEstavelCount++; if (fcEstavelCount > 100) fcEstavelCount = 100; }
  else    { fcEstavelCount = 0; }
  bool fcEstavel = (fcEstavelCount >= LIMITE_LEITURAS_ESTAVEIS);
  if (fcEstavel && !fcEstavelAntes) flagFimCurso = true;
  fcEstavelAntes = fcEstavel;

  if (alarme) { alarmeEstavelCount++; if (alarmeEstavelCount > 100) alarmeEstavelCount = 100; }
  else        { alarmeEstavelCount = 0; }
  bool alarmeEstavel = (alarmeEstavelCount >= LIMITE_ALARME_ESTAVEL);
  if (alarmeEstavel && !alarmeEstavelAntes) flagAlarmeD35 = true;
  alarmeEstavelAntes = alarmeEstavel;

  xSemaphoreTake(mutexEstado, portMAX_DELAY);
  estadoGlobal.rawS1 = s1;
  estadoGlobal.rawS2 = s2;
  estadoGlobal.rawS3 = s3;
  estadoGlobal.rawFimCurso = fc;
  estadoGlobal.rawAlarme = alarme;
  estadoGlobal.megaComunicando = (falhasConsecutivas < 5);
  estadoGlobal.megaHeartbeatMudou = (hb != prevHeartbeat);
  xSemaphoreGive(mutexEstado);

  prevS1 = s1; prevS2 = s2; prevS3 = s3; prevFimCurso = fc; prevHeartbeat = hb;
}

const char* estadoTesteToStr(EstadoTeste e) {
  switch (e) {
    case TESTE_OFF:      return "TESTE_OFF";
    case TESTE_LIGADO:   return "TESTE_LIGADO";
    case TESTE_FINALIZANDO: return "TESTE_FINALIZANDO";
    case TESTE_CONCLUIDO: return "TESTE_CONCLUIDO";
    default:                    return "?";
  }
}

// Nucleo 0: emergencia, alarme de temperatura, teste da esteira e ciclo automatico
void TaskControle(void* parameter) {
  static unsigned long tEstadoInicio = 0;
  static bool cdEnviadoEmergencia = false;
  static bool cdEnviadoTemp = false;
  static bool alarmeTempAtivo = false;

  static unsigned long tEstadoTesteInicio = 0;

  static int s1CicloEstavelCount = 0, s2CicloEstavelCount = 0, s3CicloEstavelCount = 0;
  const int LIMITE_CICLO_ESTAVEL = 3;

  for (;;) {
    pollSensoresMega();

    {
      bool resetPedido;
      xSemaphoreTake(mutexEstado, portMAX_DELAY);
      resetPedido = estadoGlobal.resetFlagsPendente;
      estadoGlobal.resetFlagsPendente = false;
      xSemaphoreGive(mutexEstado);
      if (resetPedido) {
        flagFimCurso = false;
        flagTorreDetectada = false;
      }
    }

    // Emergencia pelo botao S4 ou pelo sensor de alarme
    if (flagEmergenciaISR) {
      flagEmergenciaISR = false;
      xSemaphoreTake(mutexEstado, portMAX_DELAY);
      estadoGlobal.emergenciaAtiva = true;
      xSemaphoreGive(mutexEstado);
      addLog("Emergencia acionada pelo botao fisico S4", "error");
    }

    if (flagAlarmeD35) {
      flagAlarmeD35 = false;
      xSemaphoreTake(mutexEstado, portMAX_DELAY);
      estadoGlobal.emergenciaAtiva = true;
      xSemaphoreGive(mutexEstado);
      addLog("Emergencia acionada pelo sensor de alarme (D35)", "error");
    }

    bool sistemaLigado, cicloAtivo, emergenciaAtiva, tipoCicloLoop;
    int  maxPallets;
    EstadoSistema estadoAtual;
    long contadorTotal;
    bool pedidoResetEmergenciaLocal;

    xSemaphoreTake(mutexEstado, portMAX_DELAY);
    sistemaLigado   = estadoGlobal.sistemaLigado;
    cicloAtivo      = estadoGlobal.cicloAtivo;
    emergenciaAtiva = estadoGlobal.emergenciaAtiva;
    tipoCicloLoop   = estadoGlobal.tipoCicloLoop;
    maxPallets      = estadoGlobal.maxPallets;
    estadoAtual     = estadoGlobal.estado;
    contadorTotal   = estadoGlobal.contadorTotal;
    pedidoResetEmergenciaLocal = estadoGlobal.pedidoResetEmergencia;
    xSemaphoreGive(mutexEstado);

    {
      // Reset da emergencia pelo botao S6 (5 leituras seguidas)
      static int s6EstavelCountLoop = 0;
      const int LIMITE_S6_ESTAVEL_LOOP = 5;
      if (digitalRead(BTN_S6_RESET_PIN) == LOW) {
        s6EstavelCountLoop++;
      } else {
        s6EstavelCountLoop = 0;
      }
      if (s6EstavelCountLoop == LIMITE_S6_ESTAVEL_LOOP) {
        if (emergenciaAtiva) {
          xSemaphoreTake(mutexEstado, portMAX_DELAY);
          estadoGlobal.pedidoResetEmergencia = true;
          xSemaphoreGive(mutexEstado);
          pedidoResetEmergenciaLocal = true;
          addLog("Reset de emergencia solicitado pelo botao fisico S6", "info");
        }
      }
    }

    // LM35: 10 mV por grau
    float tempC = analogReadMilliVolts(TEMP_PIN) / 10.0;
    xSemaphoreTake(mutexEstado, portMAX_DELAY);
    estadoGlobal.temperaturaAtual = tempC;
    xSemaphoreGive(mutexEstado);

    if (!alarmeTempAtivo && tempC >= LIMITE_TEMP_ALTA) {
      alarmeTempAtivo = true;
      char buf[48]; snprintf(buf, sizeof(buf), "ALERTA: temperatura alta (%.1f C)", tempC);
      addLog(buf, "error");
    } else if (alarmeTempAtivo && tempC <= LIMITE_TEMP_NORMAL) {
      alarmeTempAtivo = false;
      addLog("Temperatura normalizada", "info");
    }
    xSemaphoreTake(mutexEstado, portMAX_DELAY);
    estadoGlobal.temperaturaAlerta = alarmeTempAtivo;
    xSemaphoreGive(mutexEstado);
    digitalWrite(BUZZ_PIN, alarmeTempAtivo ? HIGH : LOW);

    if (emergenciaAtiva) {
      // Emergencia: para a esteira, aborta os movimentos no Mega e toca a sirene
      if (!cdEnviadoEmergencia) {
        enviaComandoMega("cd");
        enviaComandoMega("em");
        cdEnviadoEmergencia = true;
        addLog("PARADA DE EMERGENCIA - esteira parada, movimentos abortados", "error");
      }
      flagFimCurso = false;
      flagTorreDetectada = false;

      {
        static unsigned long tUltimoToggleSirene = 0;
        static bool buzzerSireneState = false;
        unsigned long agoraSirene = millis();
        unsigned long faseCiclo = agoraSirene % SIRENE_CICLO_MS;
        float posTriangular = (faseCiclo < SIRENE_CICLO_MS/2)
                                 ? ((float)faseCiclo / (SIRENE_CICLO_MS/2))
                                 : ((float)(SIRENE_CICLO_MS - faseCiclo) / (SIRENE_CICLO_MS/2));
        unsigned long intervaloAtual = SIRENE_TOGGLE_MIN_MS +
            (unsigned long)((SIRENE_TOGGLE_MAX_MS - SIRENE_TOGGLE_MIN_MS) * posTriangular);
        if (agoraSirene - tUltimoToggleSirene >= intervaloAtual) {
          tUltimoToggleSirene = agoraSirene;
          buzzerSireneState = !buzzerSireneState;
          digitalWrite(BUZZ_PIN, buzzerSireneState ? HIGH : LOW);
        }
      }

      xSemaphoreTake(mutexEstado, portMAX_DELAY);
      estadoGlobal.estado = EST_EMERGENCIA;
      estadoGlobal.rgbAtual = "vermelho";
      estadoGlobal.esteiraLigadaFlag = false;
      estadoGlobal.cicloAtivo = false;
      estadoGlobal.manip1Status = "Parado";
      estadoGlobal.manip2Status = "Parado";
      estadoGlobal.testeAtivo = false;
      estadoGlobal.estadoTeste = TESTE_OFF;
      xSemaphoreGive(mutexEstado);

      // Reset: volta para parado (ou desligado) e deixa os bracos onde estao
      if (pedidoResetEmergenciaLocal) {
        xSemaphoreTake(mutexEstado, portMAX_DELAY);
        estadoGlobal.emergenciaAtiva = false;
        estadoGlobal.pedidoResetEmergencia = false;
        estadoGlobal.estado = sistemaLigado ? EST_IDLE : EST_OFF;
        xSemaphoreGive(mutexEstado);
        cdEnviadoEmergencia = false;
        digitalWrite(BUZZ_PIN, LOW);
        enviaComandoMega("em0");
        addLog("Emergencia resetada -- sistema em IDLE", "ok");
        vTaskDelay(pdMS_TO_TICKS(50));
        continue;
      }

      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    cdEnviadoEmergencia = false;

    // Temperatura alta: para a esteira e pausa o ciclo ate normalizar
    if (alarmeTempAtivo) {
      if (!cdEnviadoTemp) { enviaComandoMega("cd"); cdEnviadoTemp = true; }
      xSemaphoreTake(mutexEstado, portMAX_DELAY);
      estadoGlobal.rgbAtual = "vermelho";
      estadoGlobal.esteiraLigadaFlag = false;
      xSemaphoreGive(mutexEstado);
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    cdEnviadoTemp = false;

    bool testeAtivoLocal;
    EstadoTeste estadoTesteLocal;
    xSemaphoreTake(mutexEstado, portMAX_DELAY);
    testeAtivoLocal  = estadoGlobal.testeAtivo;
    estadoTesteLocal = estadoGlobal.estadoTeste;
    xSemaphoreGive(mutexEstado);

    static bool testeAtivoAnterior = false;
    static int s1TesteEstavelCount = 0, s2TesteEstavelCount = 0, s3TesteEstavelCount = 0;
    const int LIMITE_TESTE_ESTAVEL = 3;
    if (testeAtivoLocal && !testeAtivoAnterior) {
      s1TesteEstavelCount = 0;
      s2TesteEstavelCount = 0;
      s3TesteEstavelCount = 0;
    }
    testeAtivoAnterior = testeAtivoLocal;

    // Teste da esteira: classifica uma caixa so com os sensores, sem mover os bracos
    if (testeAtivoLocal) {
      switch (estadoTesteLocal) {

        case TESTE_LIGADO:
          {
            bool l1, l2, l3;
            xSemaphoreTake(mutexEstado, portMAX_DELAY);
            l1 = estadoGlobal.rawS1;
            l2 = estadoGlobal.rawS2;
            l3 = estadoGlobal.rawS3;
            xSemaphoreGive(mutexEstado);

            if (l1) { s1TesteEstavelCount++; } else { s1TesteEstavelCount = 0; }
            if (l2) { s2TesteEstavelCount++; } else { s2TesteEstavelCount = 0; }
            if (l3) { s3TesteEstavelCount++; } else { s3TesteEstavelCount = 0; }

            xSemaphoreTake(mutexEstado, portMAX_DELAY);
            if (s1TesteEstavelCount >= LIMITE_TESTE_ESTAVEL) estadoGlobal.latchTesteS1 = true;
            if (s2TesteEstavelCount >= LIMITE_TESTE_ESTAVEL) estadoGlobal.latchTesteS2 = true;
            if (s3TesteEstavelCount >= LIMITE_TESTE_ESTAVEL) estadoGlobal.latchTesteS3 = true;
            xSemaphoreGive(mutexEstado);
          }

          if (flagFimCurso) {
            flagFimCurso = false;
            tEstadoTesteInicio = millis();
            xSemaphoreTake(mutexEstado, portMAX_DELAY);
            estadoGlobal.estadoTeste = TESTE_FINALIZANDO;
            xSemaphoreGive(mutexEstado);
            addLog("Teste: fim de curso detectado, esteira +1s antes de parar", "info");
          }
          break;

        case TESTE_FINALIZANDO:
          if (millis() - tEstadoTesteInicio >= 1000) {
            enviaComandoMega("cd");

            bool l1, l2, l3;
            xSemaphoreTake(mutexEstado, portMAX_DELAY);
            l1 = estadoGlobal.latchTesteS1;
            l2 = estadoGlobal.latchTesteS2;
            l3 = estadoGlobal.latchTesteS3;
            xSemaphoreGive(mutexEstado);

            int count = (l1?1:0)+(l2?1:0)+(l3?1:0);
            String cat;
            if      (count==1) cat="P";
            else if (count==2) cat="M";
            else if (count==3) cat="G";
            else               cat="ERRO";

            xSemaphoreTake(mutexEstado, portMAX_DELAY);
            estadoGlobal.categoriaAtual = cat;
            estadoGlobal.latchS1 = l1;
            estadoGlobal.latchS2 = l2;
            estadoGlobal.latchS3 = l3;
            estadoGlobal.esteiraLigadaFlag = false;
            estadoGlobal.contadorTotal++;
            if      (cat=="P") estadoGlobal.contadorP++;
            else if (cat=="M") estadoGlobal.contadorM++;
            else if (cat=="G") estadoGlobal.contadorG++;
            long total = estadoGlobal.contadorTotal;
            estadoGlobal.estadoTeste = TESTE_CONCLUIDO;
            estadoGlobal.latchTesteS1 = false;
            estadoGlobal.latchTesteS2 = false;
            estadoGlobal.latchTesteS3 = false;
            xSemaphoreGive(mutexEstado);
            s1TesteEstavelCount = 0;
            s2TesteEstavelCount = 0;
            s3TesteEstavelCount = 0;

            digitalWrite(BUZZ_PIN, HIGH); delay(80); digitalWrite(BUZZ_PIN, LOW);
            char buf[64];
            snprintf(buf, sizeof(buf), "Teste esteira concluido - categoria %s (total: %ld)", cat.c_str(), total);
            addLog(buf, "ok");
          }
          break;

        case TESTE_CONCLUIDO:
          xSemaphoreTake(mutexEstado, portMAX_DELAY);
          estadoGlobal.testeAtivo = false;
          estadoGlobal.estadoTeste = TESTE_OFF;
          xSemaphoreGive(mutexEstado);
          break;

        default:
          break;
      }
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    String novoRgb = "azul";
    EstadoSistema novoEstado = estadoAtual;

    // Ciclo automatico: braco 1, esteira, classificacao e braco 2
    switch (estadoAtual) {

      case EST_OFF:
        novoRgb = "azul";
        if (sistemaLigado) novoEstado = EST_IDLE;
        break;

      case EST_IDLE:
        novoRgb = "azul";
        if (!sistemaLigado) { novoEstado = EST_OFF; break; }
        if (cicloAtivo) {
          xSemaphoreTake(mutexEstado, portMAX_DELAY);
          estadoGlobal.latchS1 = estadoGlobal.latchS2 = estadoGlobal.latchS3 = false;
          xSemaphoreGive(mutexEstado);
          s1CicloEstavelCount = s2CicloEstavelCount = s3CicloEstavelCount = 0;
          flagFimCurso = false;
          addLog("Ciclo iniciado: B1 indo buscar caixa", "ok");
          enviaComandoMega("pp");
          xSemaphoreTake(mutexEstado, portMAX_DELAY);
          estadoGlobal.manip1Status = "Movendo (B1)";
          xSemaphoreGive(mutexEstado);
          tEstadoInicio = millis();
          novoEstado = EST_B1;
        }
        break;

      case EST_B1:
        novoRgb = "amarelo";
        if (millis() - tEstadoInicio >= DURACAO_B1_MS) {
          xSemaphoreTake(mutexEstado, portMAX_DELAY);
          estadoGlobal.manip1Status = "Parado";
          estadoGlobal.latchS1 = estadoGlobal.latchS2 = estadoGlobal.latchS3 = false;
          xSemaphoreGive(mutexEstado);
          s1CicloEstavelCount = s2CicloEstavelCount = s3CicloEstavelCount = 0;
          flagFimCurso = false;
          addLog("B1 concluido. Ligando esteira.", "ok");
          enviaComandoMega("ce");
          xSemaphoreTake(mutexEstado, portMAX_DELAY);
          estadoGlobal.esteiraLigadaFlag = true;
          xSemaphoreGive(mutexEstado);
          novoEstado = EST_ESTEIRA;
        }
        break;

      case EST_ESTEIRA: {
        novoRgb = "verde";

        if (!flagFimCurso) {
          bool rs1, rs2, rs3;
          xSemaphoreTake(mutexEstado, portMAX_DELAY);
          rs1 = estadoGlobal.rawS1;
          rs2 = estadoGlobal.rawS2;
          rs3 = estadoGlobal.rawS3;
          xSemaphoreGive(mutexEstado);

          if (rs1) { s1CicloEstavelCount++; } else { s1CicloEstavelCount = 0; }
          if (rs2) { s2CicloEstavelCount++; } else { s2CicloEstavelCount = 0; }
          if (rs3) { s3CicloEstavelCount++; } else { s3CicloEstavelCount = 0; }

          xSemaphoreTake(mutexEstado, portMAX_DELAY);
          if (s1CicloEstavelCount >= LIMITE_CICLO_ESTAVEL) estadoGlobal.latchS1 = true;
          if (s2CicloEstavelCount >= LIMITE_CICLO_ESTAVEL) estadoGlobal.latchS2 = true;
          if (s3CicloEstavelCount >= LIMITE_CICLO_ESTAVEL) estadoGlobal.latchS3 = true;
          xSemaphoreGive(mutexEstado);
        }

        if (flagFimCurso) {
          flagFimCurso = false;
          enviaComandoMega("cd");

          bool l1, l2, l3;
          xSemaphoreTake(mutexEstado, portMAX_DELAY);
          l1 = estadoGlobal.latchS1;
          l2 = estadoGlobal.latchS2;
          l3 = estadoGlobal.latchS3;
          xSemaphoreGive(mutexEstado);

          int count = (l1?1:0)+(l2?1:0)+(l3?1:0);
          String cat;
          if      (count==1) cat="P";
          else if (count==2) cat="M";
          else if (count==3) cat="G";
          else               cat="ERRO";

          digitalWrite(BUZZ_PIN, HIGH); delay(80); digitalWrite(BUZZ_PIN, LOW);

          xSemaphoreTake(mutexEstado, portMAX_DELAY);
          estadoGlobal.categoriaAtual = cat;
          estadoGlobal.esteiraLigadaFlag = false;
          estadoGlobal.fimCursoPulso = true;
          estadoGlobal.latchS1 = false;
          estadoGlobal.latchS2 = false;
          estadoGlobal.latchS3 = false;
          xSemaphoreGive(mutexEstado);
          s1CicloEstavelCount = s2CicloEstavelCount = s3CicloEstavelCount = 0;

          char buf[48];
          snprintf(buf, sizeof(buf), "Caixa no fim de curso - categoria %s", cat.c_str());
          addLog(buf, "ok");

          tEstadoInicio = millis();
          novoEstado = EST_FIM_CURSO;
        }
        break;
      }

      case EST_FIM_CURSO:
        novoRgb = "amarelo";
        if (millis() - tEstadoInicio >= 500) {
          xSemaphoreTake(mutexEstado, portMAX_DELAY);
          estadoGlobal.fimCursoPulso = false;
          xSemaphoreGive(mutexEstado);
          addLog("B2 pegando a caixa na esteira (pp2)", "info");
          enviaComandoMega("pp2");
          xSemaphoreTake(mutexEstado, portMAX_DELAY);
          estadoGlobal.manip2Status = "Movendo (B2 pegar)";
          xSemaphoreGive(mutexEstado);
          tEstadoInicio = millis();
          novoEstado = EST_B2_PEGAR;
        }
        break;

      case EST_B2_PEGAR:
        novoRgb = "amarelo";
        if (millis() - tEstadoInicio >= DURACAO_B2_PEGAR_MS) {
          addLog("B2 guardando a caixa no destino (sp2)", "info");
          enviaComandoMega("sp2");
          xSemaphoreTake(mutexEstado, portMAX_DELAY);
          estadoGlobal.manip2Status = "Movendo (B2 soltar)";
          xSemaphoreGive(mutexEstado);
          tEstadoInicio = millis();
          novoEstado = EST_B2_SOLTAR;
        }
        break;

      case EST_B2_SOLTAR:
        novoRgb = "amarelo";
        if (millis() - tEstadoInicio >= DURACAO_B2_SOLTAR_MS) {
          xSemaphoreTake(mutexEstado, portMAX_DELAY);
          estadoGlobal.manip2Status = "Parado";
          estadoGlobal.contadorTotal++;
          if (estadoGlobal.categoriaAtual=="P") estadoGlobal.contadorP++;
          else if (estadoGlobal.categoriaAtual=="M") estadoGlobal.contadorM++;
          else if (estadoGlobal.categoriaAtual=="G") estadoGlobal.contadorG++;
          long novoTotal = estadoGlobal.contadorTotal;
          xSemaphoreGive(mutexEstado);

          char buf[40];
          snprintf(buf, sizeof(buf), "Ciclo concluido (total: %ld)", novoTotal);
          addLog(buf, "ok");
          tEstadoInicio = millis();
          novoEstado = EST_CICLO_DONE;
        }
        break;

      case EST_CICLO_DONE:
        novoRgb = "verde";
        if (millis() - tEstadoInicio >= 700) {
          bool continuarLoop = tipoCicloLoop && (contadorTotal < maxPallets);
          if (continuarLoop) {
            addLog("Reiniciando ciclo (modo loop)", "info");
            xSemaphoreTake(mutexEstado, portMAX_DELAY);
            estadoGlobal.latchS1 = estadoGlobal.latchS2 = estadoGlobal.latchS3 = false;
            xSemaphoreGive(mutexEstado);
            flagFimCurso = false;
            enviaComandoMega("pp");
            xSemaphoreTake(mutexEstado, portMAX_DELAY);
            estadoGlobal.manip1Status = "Movendo (B1)";
            xSemaphoreGive(mutexEstado);
            tEstadoInicio = millis();
            novoEstado = EST_B1;
          } else {
            xSemaphoreTake(mutexEstado, portMAX_DELAY);
            estadoGlobal.cicloAtivo = false;
            xSemaphoreGive(mutexEstado);
            addLog("Ciclo automatico finalizado", "ok");
            novoEstado = EST_IDLE;
          }
        }
        break;

      default:
        novoEstado = sistemaLigado ? EST_IDLE : EST_OFF;
        break;
    }

    xSemaphoreTake(mutexEstado, portMAX_DELAY);
    estadoGlobal.estado = novoEstado;
    estadoGlobal.rgbAtual = novoRgb;
    xSemaphoreGive(mutexEstado);

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// OLED
const char* estadoToStr(EstadoSistema e) {
  switch(e){
    case EST_OFF: return "OFF"; case EST_IDLE: return "IDLE";
    case EST_B1: return "B1_MOVENDO"; case EST_ESTEIRA: return "ESTEIRA";
    case EST_FIM_CURSO: return "FIM_DE_CURSO"; case EST_B2_PEGAR: return "B2_PEGANDO";
    case EST_B2_SOLTAR: return "B2_SOLTANDO"; case EST_CICLO_DONE: return "CICLO_CONCLUIDO";
    case EST_EMERGENCIA: return "EMERGENCIA"; default: return "?";
  }
}
void oledCtr(int y, const char* s) {
  int x=(128-strlen(s)*6)/2; if(x<0)x=0; oled.setCursor(x,y); oled.print(s);
}
void atualizaOLED(EstadoCompartilhado snap, String ip) {
  oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
  oledCtr(0, NOME_GRUPO); oled.drawLine(0,9,127,9,SSD1306_WHITE);

  if (snap.testeAtivo) {
    oled.setCursor(0,14); oled.print("TESTE ESTEIRA");
    oled.setCursor(0,25); oled.print("Estado: "); oled.println(estadoTesteToStr(snap.estadoTeste));
    oled.setCursor(0,36); oled.print("Categoria: "); oled.println(snap.categoriaAtual);
    oled.setCursor(0,47); oled.print("Total: "); oled.println(snap.contadorTotal);
  } else {
    oled.setCursor(0,14); oled.print("Estado: "); oled.println(estadoToStr(snap.estado));
    oled.setCursor(0,25); oled.print("Categoria: "); oled.println(snap.categoriaAtual);
    oled.setCursor(0,36); oled.print("Total: "); oled.println(snap.contadorTotal);
    oled.setCursor(0,47); oled.print("IP: "); oled.println(ip);
  }

  if(snap.emergenciaAtiva){ oled.setCursor(0,56); oled.print("** EMERGENCIA **"); }
  else if(snap.temperaturaAlerta){ oled.setCursor(0,56); oled.print("** TEMP ALTA "); oled.print((int)snap.temperaturaAtual); oled.print("C **"); }
  oled.display();
  digitalWrite(27, LOW); digitalWrite(14, LOW);
}

// Nucleo 1: Wi-Fi, servidor web, log, LED RGB e OLED
void TaskComunicacao(void* parameter) {
  unsigned long tUltimoOLED = 0, tUltimoWifiCheck = 0;
  unsigned long tUltimoDiagSensores = 0;
  String ipStr = WiFi.localIP().toString();
  for (;;) {
    server.handleClient();
    unsigned long agoraWifi = millis();
    if (agoraWifi - tUltimoWifiCheck >= 5000) {
      tUltimoWifiCheck = agoraWifi;
      if (WiFi.status() != WL_CONNECTED) {
        addLog("WiFi caiu, tentando reconectar...", "error");
        WiFi.disconnect(); WiFi.begin(ssid, password);
      } else { ipStr = WiFi.localIP().toString(); }
    }
    LogMsg m;
    while (xQueueReceive(filaLog, &m, 0) == pdTRUE) {
      if (logCount >= LOG_BUFFER_MAX) {
        for(int i=1;i<LOG_BUFFER_MAX;i++) logBuffer[i-1]=logBuffer[i];
        logCount = LOG_BUFFER_MAX-1;
      }
      logBuffer[logCount].hora=horaAtualStr();
      logBuffer[logCount].texto=String(m.texto);
      logBuffer[logCount].nivel=String(m.nivel);
      logCount++;
    }
    unsigned long agora = millis();
    if (agora - tUltimoOLED >= 400) {
      tUltimoOLED = agora;
      EstadoCompartilhado snap;
      xSemaphoreTake(mutexEstado, portMAX_DELAY); snap=estadoGlobal; xSemaphoreGive(mutexEstado);

      String corFinal;
      if (snap.emergenciaAtiva || snap.temperaturaAlerta || !snap.megaComunicando) {
        corFinal = "vermelho";
      } else {
        corFinal = snap.rgbAtual;
      }
      setRGB(corFinal.c_str());
      atualizaOLED(snap, ipStr);
    }

    if (agora - tUltimoDiagSensores >= 300) {
      tUltimoDiagSensores = agora;
      bool s1, s2, s3, fc, alarme, comOk;
      xSemaphoreTake(mutexEstado, portMAX_DELAY);
      s1 = estadoGlobal.rawS1;
      s2 = estadoGlobal.rawS2;
      s3 = estadoGlobal.rawS3;
      fc = estadoGlobal.rawFimCurso;
      alarme = estadoGlobal.rawAlarme;
      comOk = estadoGlobal.megaComunicando;
      xSemaphoreGive(mutexEstado);

      Serial.print("[SENSORES] S1(Pequena): ");
      Serial.print(s1 ? "DETECTADO" : "livre");
      Serial.print(" | S2(Media): ");
      Serial.print(s2 ? "DETECTADO" : "livre");
      Serial.print(" | S3(Grande): ");
      Serial.print(s3 ? "DETECTADO" : "livre");
      Serial.print(" | FimCurso: ");
      Serial.print(fc ? "DETECTADO" : "livre");
      Serial.print(" | Alarme(D35): ");
      Serial.print(alarme ? "ACIONADO !!" : "livre");
      Serial.println(comOk ? "" : "  !! SEM COMUNICACAO COM O MEGA !!");
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Rotas HTTP
void handleStatus() {
  EstadoCompartilhado snap;
  xSemaphoreTake(mutexEstado, portMAX_DELAY);
  snap = estadoGlobal;
  xSemaphoreGive(mutexEstado);

  String json = "{";
  json += "\"ligado\":" + String(snap.sistemaLigado ? "true":"false") + ",";
  json += "\"emergencia\":" + String(snap.emergenciaAtiva ? "true":"false") + ",";
  json += "\"cicloAtivo\":" + String(snap.cicloAtivo ? "true":"false") + ",";
  json += "\"tipoCicloLoop\":" + String(snap.tipoCicloLoop ? "true":"false") + ",";
  json += "\"maxPallets\":" + String(snap.maxPallets) + ",";
  json += "\"estado\":\"" + String(estadoToStr(snap.estado)) + "\",";
  json += "\"contadorTotal\":" + String(snap.contadorTotal) + ",";
  json += "\"contadorP\":" + String(snap.contadorP) + ",";
  json += "\"contadorM\":" + String(snap.contadorM) + ",";
  json += "\"contadorG\":" + String(snap.contadorG) + ",";
  json += "\"categoria\":\"" + snap.categoriaAtual + "\",";
  json += "\"manip1\":\"" + snap.manip1Status + "\",";
  json += "\"manip2\":\"" + snap.manip2Status + "\",";
  json += "\"esteira\":" + String(snap.esteiraLigadaFlag ? "true":"false") + ",";
  json += "\"latchS1\":" + String(snap.latchS1 ? "true":"false") + ",";
  json += "\"latchS2\":" + String(snap.latchS2 ? "true":"false") + ",";
  json += "\"latchS3\":" + String(snap.latchS3 ? "true":"false") + ",";
  json += "\"fimCursoPulso\":" + String(snap.fimCursoPulso ? "true":"false") + ",";
  json += "\"temperatura\":" + String(snap.temperaturaAtual, 1) + ",";
  json += "\"temperaturaAlerta\":" + String(snap.temperaturaAlerta ? "true":"false") + ",";
  json += "\"rgb\":\"" + snap.rgbAtual + "\",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"testeAtivo\":" + String(snap.testeAtivo ? "true":"false") + ",";
  json += "\"estadoTeste\":\"" + String(estadoTesteToStr(snap.estadoTeste)) + "\",";
  json += "\"megaComunicando\":" + String(snap.megaComunicando ? "true":"false");
  json += "}";

  server.send(200, "application/json", json);
}

void handleEvents() {
  String json = "[";
  for (int i = 0; i < logCount; i++) {
    if (i > 0) json += ",";
    json += "{\"hora\":\"" + logBuffer[i].hora + "\",";
    json += "\"texto\":\"" + logBuffer[i].texto + "\",";
    json += "\"nivel\":\"" + logBuffer[i].nivel + "\"}";
  }
  json += "]";
  server.send(200, "application/json", json);
}

void handleSistemaOn() {
  xSemaphoreTake(mutexEstado, portMAX_DELAY);
  estadoGlobal.sistemaLigado = true;
  xSemaphoreGive(mutexEstado);
  addLog("Sistema ligado", "info");
  server.send(200, "text/plain", "OK");
}

void handleSistemaOff() {
  xSemaphoreTake(mutexEstado, portMAX_DELAY);
  estadoGlobal.sistemaLigado = false;
  estadoGlobal.cicloAtivo = false;
  xSemaphoreGive(mutexEstado);
  enviaComandoMega("cd");
  addLog("Sistema desligado", "info");
  server.send(200, "text/plain", "OK");
}

void handleStart() {
  bool loopMode = false;
  int  maxP = 10;
  if (server.hasArg("loop")) loopMode = (server.arg("loop") == "1");
  if (server.hasArg("max"))  maxP = server.arg("max").toInt();
  if (maxP <= 0) maxP = 10;

  bool ligado, testeAtivo;
  xSemaphoreTake(mutexEstado, portMAX_DELAY);
  ligado = estadoGlobal.sistemaLigado;
  testeAtivo = estadoGlobal.testeAtivo;
  xSemaphoreGive(mutexEstado);

  if (!ligado) { server.send(409, "text/plain", "Sistema desligado"); return; }
  if (testeAtivo) { server.send(409, "text/plain", "Teste da esteira em andamento"); return; }

  xSemaphoreTake(mutexEstado, portMAX_DELAY);
  estadoGlobal.tipoCicloLoop = loopMode;
  estadoGlobal.maxPallets = maxP;
  estadoGlobal.cicloAtivo = true;
  xSemaphoreGive(mutexEstado);

  addLog(loopMode ? "Ciclo automatico (loop) iniciado" : "Ciclo unico iniciado", "info");
  server.send(200, "text/plain", "OK");
}

void handleStop() {
  xSemaphoreTake(mutexEstado, portMAX_DELAY);
  estadoGlobal.cicloAtivo = false;
  xSemaphoreGive(mutexEstado);
  addLog("Ciclo parado pelo usuario", "info");
  server.send(200, "text/plain", "OK");
}

void handleResetProc() {
  xSemaphoreTake(mutexEstado, portMAX_DELAY);
  estadoGlobal.estado = estadoGlobal.sistemaLigado ? EST_IDLE : EST_OFF;
  estadoGlobal.cicloAtivo = false;
  estadoGlobal.manip1Status = "Parado";
  estadoGlobal.manip2Status = "Parado";
  estadoGlobal.esteiraLigadaFlag = false;
  estadoGlobal.latchS1 = estadoGlobal.latchS2 = estadoGlobal.latchS3 = false;
  estadoGlobal.categoriaAtual = "-";
  estadoGlobal.resetFlagsPendente = true;
  xSemaphoreGive(mutexEstado);
  enviaComandoMega("cd");
  addLog("Processo resetado", "info");
  server.send(200, "text/plain", "OK");
}

void handleEmergency() {
  xSemaphoreTake(mutexEstado, portMAX_DELAY);
  estadoGlobal.emergenciaAtiva = true;
  xSemaphoreGive(mutexEstado);
  server.send(200, "text/plain", "OK");
}

void handleResetEmergency() {
  xSemaphoreTake(mutexEstado, portMAX_DELAY);
  estadoGlobal.pedidoResetEmergencia = true;
  xSemaphoreGive(mutexEstado);
  addLog("Reset de emergencia solicitado via web", "info");
  server.send(200, "text/plain", "OK");
}

void handlePP()  { enviaComandoMega("pp");  addLog("Teste manual: pp",  "info"); server.send(200, "text/plain", "OK"); }
void handleSP()  { enviaComandoMega("sp");  addLog("Teste manual: sp",  "info"); server.send(200, "text/plain", "OK"); }
void handlePP2() { enviaComandoMega("pp2"); addLog("Teste manual: pp2", "info"); server.send(200, "text/plain", "OK"); }
void handleSP2() { enviaComandoMega("sp2"); addLog("Teste manual: sp2", "info"); server.send(200, "text/plain", "OK"); }

void handleConveyorOn() {
  enviaComandoMega("ce");
  xSemaphoreTake(mutexEstado, portMAX_DELAY);
  estadoGlobal.esteiraLigadaFlag = true;
  xSemaphoreGive(mutexEstado);
  addLog("Esteira ligada manualmente", "info");
  server.send(200, "text/plain", "OK");
}

void handleConveyorOff() {
  enviaComandoMega("cd");
  xSemaphoreTake(mutexEstado, portMAX_DELAY);
  estadoGlobal.esteiraLigadaFlag = false;
  xSemaphoreGive(mutexEstado);
  addLog("Esteira desligada manualmente", "info");
  server.send(200, "text/plain", "OK");
}

void handleScan() {
  bool l1, l2, l3;
  xSemaphoreTake(mutexEstado, portMAX_DELAY);
  l1 = estadoGlobal.rawS1;
  l2 = estadoGlobal.rawS2;
  l3 = estadoGlobal.rawS3;
  xSemaphoreGive(mutexEstado);

  int count = (l1?1:0)+(l2?1:0)+(l3?1:0);
  String cat;
  if      (count==1) cat="P";
  else if (count==2) cat="M";
  else if (count==3) cat="G";
  else               cat="ERRO";

  xSemaphoreTake(mutexEstado, portMAX_DELAY);
  estadoGlobal.categoriaAtual = cat;
  estadoGlobal.latchS1 = l1; estadoGlobal.latchS2 = l2; estadoGlobal.latchS3 = l3;
  xSemaphoreGive(mutexEstado);

  String json = "{\"categoria\":\"" + cat + "\",";
  json += "\"s1\":" + String(l1?"true":"false") + ",";
  json += "\"s2\":" + String(l2?"true":"false") + ",";
  json += "\"s3\":" + String(l3?"true":"false") + "}";
  server.send(200, "application/json", json);
}

void handleScanBeep() {
  digitalWrite(BUZZ_PIN, HIGH);
  delay(120);
  digitalWrite(BUZZ_PIN, LOW);

  bool l1, l2, l3;
  xSemaphoreTake(mutexEstado, portMAX_DELAY);
  l1 = estadoGlobal.rawS1;
  l2 = estadoGlobal.rawS2;
  l3 = estadoGlobal.rawS3;
  xSemaphoreGive(mutexEstado);
  int count = (l1?1:0)+(l2?1:0)+(l3?1:0);
  String cat;
  if      (count==1) cat="P";
  else if (count==2) cat="M";
  else if (count==3) cat="G";
  else               cat="ERRO";

  xSemaphoreTake(mutexEstado, portMAX_DELAY);
  estadoGlobal.categoriaAtual = cat;
  estadoGlobal.latchS1 = l1; estadoGlobal.latchS2 = l2; estadoGlobal.latchS3 = l3;
  xSemaphoreGive(mutexEstado);

  String json = "{\"categoria\":\"" + cat + "\",";
  json += "\"s1\":" + String(l1?"true":"false") + ",";
  json += "\"s2\":" + String(l2?"true":"false") + ",";
  json += "\"s3\":" + String(l3?"true":"false") + "}";
  server.send(200, "application/json", json);
}

void handleMotor() {
  if (!server.hasArg("id") || !server.hasArg("deg") || !server.hasArg("dir")) {
    server.send(400, "text/plain", "Parametros obrigatorios: id, deg, dir");
    return;
  }
  int id  = server.arg("id").toInt();
  int deg = server.arg("deg").toInt();
  int dir = server.arg("dir").toInt();

  if (id < 1 || id > 6) {
    server.send(400, "text/plain", "id deve ser 1-6");
    return;
  }

  char cmd[24];
  snprintf(cmd, sizeof(cmd), "g%d:%d:%d", id, deg, dir);
  enviaComandoMega(cmd);

  char buf[48];
  snprintf(buf, sizeof(buf), "Motor %d: %d graus, sentido %d", id, deg, dir);
  addLog(buf, "info");
  server.send(200, "text/plain", "OK");
}

void handleMotorCtl() {
  if (!server.hasArg("id") || !server.hasArg("action")) {
    server.send(400, "text/plain", "Parametros obrigatorios: id, action (a|b|s)");
    return;
  }
  int id = server.arg("id").toInt();
  String action = server.arg("action");

  if (id < 1 || id > 7) {
    server.send(400, "text/plain", "id deve ser 1-7");
    return;
  }
  if (action != "a" && action != "b" && action != "s") {
    server.send(400, "text/plain", "action deve ser a, b ou s");
    return;
  }

  char cmd[8];
  snprintf(cmd, sizeof(cmd), "m%d%s", id, action.c_str());
  enviaComandoMega(cmd);

  char buf[48];
  const char* desc = (action=="a") ? "LIGADO direcao A" : (action=="b") ? "LIGADO direcao B" : "PARADO";
  snprintf(buf, sizeof(buf), "Motor %d: %s", id, desc);
  addLog(buf, "info");
  server.send(200, "text/plain", "OK");
}

// Pagina de controle (HTML, CSS e JavaScript), consulta /status, /sensores e /events a cada 1 s
void handleRoot() {
  server.send(200, "text/html", R"HTMLPAGE(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Sistema de Classificação e Separação de Caixas</title>
<style>
  :root{
    --bg-0:#0b0f14;
    --bg-1:#101620;
    --bg-2:#161e2b;
    --bg-3:#1d2735;
    --line:#243042;
    --line-strong:#324154;

    --text-0:#e8edf4;
    --text-1:#9aabc0;
    --text-2:#5d7088;

    --blue:#3aa0ff;
    --blue-dim:#1c4068;
    --green:#36d399;
    --green-dim:#1a4a3a;
    --orange:#ff9f43;
    --orange-dim:#5a3c1a;
    --red:#ff5c66;
    --red-dim:#5a2230;
    --purple:#a78bfa;
    --purple-dim:#3d3266;

    --font-display: 'Segoe UI', system-ui, -apple-system, sans-serif;
    --font-mono: 'Consolas', 'SF Mono', 'Cascadia Code', monospace;

    --radius:10px;
    --radius-sm:6px;
  }

  *{box-sizing:border-box;}
  html,body{margin:0;padding:0;}
  body{
    background:
      radial-gradient(1200px 600px at 85% -10%, rgba(58,160,255,0.06), transparent 60%),
      radial-gradient(900px 500px at -5% 110%, rgba(54,211,153,0.05), transparent 60%),
      var(--bg-0);
    color:var(--text-0);
    font-family:var(--font-display);
    min-height:100vh;
    -webkit-font-smoothing:antialiased;
  }

  .app{
    display:grid;
    grid-template-columns: 320px 1fr;
    grid-template-rows: 1fr;
    grid-template-areas: "sidebar main";
    min-height:100vh;
  }

  .sidebar-clock{
    font-family:var(--font-mono);
    font-size:13px;
    color:var(--text-1);
    padding:9px 14px;
    border:1px solid var(--line);
    border-radius:var(--radius-sm);
    background:var(--bg-2);
    text-align:center;
    margin-bottom:16px;
    letter-spacing:1px;
  }

  .conn-status{
    display:flex;align-items:center;justify-content:center;gap:8px;
    font-family:var(--font-mono);font-size:12px;font-weight:700;
    padding:9px 14px;border-radius:var(--radius-sm);
    margin-bottom:10px;
    border:1px solid var(--line-strong);
    background:var(--bg-2);
    color:var(--text-1);
  }
  .conn-status.conectado{border-color:var(--green);color:var(--green);background:var(--green-dim);}
  .conn-status.desconectado{border-color:var(--red);color:var(--red);background:var(--red-dim);animation:pulse 1s infinite;}
  .conn-status .dot-conn{width:8px;height:8px;border-radius:50%;background:currentColor;flex-shrink:0;}

  aside.sidebar{
    grid-area:sidebar;
    border-right:1px solid var(--line);
    background:var(--bg-1);
    padding:20px 18px 32px;
    overflow-y:auto;
  }

  .panel{
    background:var(--bg-2);
    border:1px solid var(--line);
    border-radius:var(--radius);
    padding:16px;
    margin-bottom:16px;
  }
  .panel-title{
    font-size:11.5px;
    font-weight:700;
    letter-spacing:1.2px;
    text-transform:uppercase;
    color:var(--text-2);
    margin:0 0 12px;
    display:flex;
    align-items:center;
    gap:8px;
  }
  .panel-title .dot{
    width:6px;height:6px;border-radius:50%;
    background:var(--blue);
    box-shadow:0 0 8px var(--blue);
    flex-shrink:0;
  }

  .btn-grid{display:flex;flex-direction:column;gap:8px;}
  button{
    font-family:var(--font-display);
    border:1px solid var(--line-strong);
    background:var(--bg-3);
    color:var(--text-0);
    padding:10px 12px;
    border-radius:var(--radius-sm);
    font-size:13px;
    font-weight:600;
    cursor:pointer;
    transition:transform .08s ease, border-color .15s ease, background .15s ease;
    text-align:left;
    display:flex;
    align-items:center;
    gap:9px;
  }
  button:hover:not(:disabled){border-color:var(--blue);background:#1c2738;}
  button:active:not(:disabled){transform:scale(0.98);}
  button:disabled{opacity:.35;cursor:not-allowed;}
  button .ic{font-size:14px;width:16px;text-align:center;flex-shrink:0;}

  button.primary{background:linear-gradient(135deg,var(--green-dim),var(--bg-3));border-color:var(--green);}
  button.primary:hover:not(:disabled){background:linear-gradient(135deg,#1f5a45,var(--bg-3));}
  button.danger{background:linear-gradient(135deg,var(--red-dim),var(--bg-3));border-color:var(--red);}
  button.danger:hover:not(:disabled){background:linear-gradient(135deg,#6a2733,var(--bg-3));}
  button.info{border-color:var(--blue);}

  button.emergencia{
    background:linear-gradient(135deg,#7a1d28,#4a1118);
    border:2px solid var(--red);
    color:#ffd9dc;
    font-weight:800;
    font-size:14px;
    padding:14px 12px;
    justify-content:center;
    letter-spacing:.5px;
  }
  button.emergencia:hover:not(:disabled){background:linear-gradient(135deg,#8f222e,#5a141c);}
  button.emergencia:disabled{opacity:.5;}

  .divider-label{
    font-size:10.5px;color:var(--text-2);
    text-transform:uppercase;letter-spacing:1px;
    margin:14px 0 6px;
    border-top:1px dashed var(--line);
    padding-top:10px;
  }

  .cycle-toggle{display:flex;gap:6px;margin-top:8px;}
  .cycle-toggle button{flex:1;justify-content:center;font-size:11.5px;padding:8px 6px;}
  .cycle-toggle button.active{border-color:var(--orange);background:var(--orange-dim);color:var(--orange);}

  main.content{
    grid-area:main;
    padding:24px 28px 40px;
    overflow-y:auto;
  }

  .grid-2{display:grid;grid-template-columns: 1.3fr 1fr;gap:18px;align-items:start;}

  @media (max-width: 980px){
    .app{grid-template-columns:1fr;grid-template-areas:"sidebar" "main";}
    aside.sidebar{border-right:none;border-bottom:1px solid var(--line);}
    .grid-2{grid-template-columns:1fr;}
  }

  .state-banner{
    background:linear-gradient(135deg, var(--bg-2), var(--bg-1));
    border:1px solid var(--line);
    border-radius:var(--radius);
    padding:16px 18px;
    margin-bottom:18px;
    display:flex;
    align-items:center;
    justify-content:space-between;
    gap:16px;
    flex-wrap:wrap;
  }
  .state-banner.emergencia{border-color:var(--red);background:linear-gradient(135deg,var(--red-dim),var(--bg-1));}
  .state-banner .left{display:flex;align-items:center;gap:14px;min-width:0;flex:1;}
  .state-banner .icon{
    width:46px;height:46px;border-radius:10px;
    display:flex;align-items:center;justify-content:center;
    font-size:20px;background:var(--bg-3);border:1px solid var(--line-strong);
    flex-shrink:0;
  }
  .state-banner .name{font-size:11px;color:var(--text-2);text-transform:uppercase;letter-spacing:1px;}
  .state-banner .code{font-family:var(--font-mono);font-size:19px;font-weight:700;color:var(--text-0);word-break:break-word;}
  .state-banner .counter{font-family:var(--font-mono);font-size:12.5px;color:var(--text-1);text-align:right;flex-shrink:0;white-space:nowrap;}
  .state-banner .counter .n{color:var(--green);font-weight:700;font-size:16px;}

  @media (max-width: 600px){
    .state-banner .counter{text-align:left;width:100%;}
  }

  .status-row{
    display:grid;
    grid-template-columns:repeat(auto-fit, minmax(130px, 1fr));
    gap:12px;
    margin-bottom:18px;
  }
  .status-card{
    background:var(--bg-1);
    border:1px solid var(--line);
    border-radius:var(--radius);
    padding:13px 14px;
    display:flex;flex-direction:column;gap:8px;
    position:relative;overflow:hidden;min-width:0;
  }
  .status-card .label{font-size:10.5px;color:var(--text-2);text-transform:uppercase;letter-spacing:.8px;}
  .status-card .value{font-family:var(--font-mono);font-size:13.5px;font-weight:700;display:flex;align-items:flex-start;gap:8px;min-width:0;line-height:1.3;}
  .status-card .value .led{margin-top:3px;}

  @media (max-width: 600px){
    .status-row{grid-template-columns:1fr;}
  }

  .led{width:9px;height:9px;border-radius:50%;background:var(--text-2);flex-shrink:0;transition:all .2s ease;}
  .led.on-green{background:var(--green);box-shadow:0 0 9px var(--green);}
  .led.on-blue{background:var(--blue);box-shadow:0 0 9px var(--blue);animation:pulse 1s infinite;}
  .led.on-orange{background:var(--orange);box-shadow:0 0 9px var(--orange);animation:pulse .6s infinite;}
  .led.on-red{background:var(--red);box-shadow:0 0 9px var(--red);animation:pulse .4s infinite;}
  @keyframes pulse{0%,100%{opacity:1;}50%{opacity:.4;}}

  .hw-grid{display:grid;grid-template-columns:1fr 1fr;gap:14px;margin-bottom:18px;}
  @media (max-width: 700px){ .hw-grid{grid-template-columns:1fr;} }

  .hw-card{background:var(--bg-1);border:1px solid var(--line);border-radius:var(--radius);padding:16px;}
  .hw-card .panel-title{margin-bottom:14px;}

  .rgb-display{display:flex;align-items:center;gap:14px;}
  .rgb-orb{
    width:54px;height:54px;border-radius:50%;
    background:var(--text-2);
    box-shadow:0 0 0 transparent;
    transition:all .3s ease;
    flex-shrink:0;
  }
  .rgb-orb.cor-azul{background:radial-gradient(circle at 35% 35%, #7cc4ff, var(--blue));box-shadow:0 0 24px rgba(58,160,255,.6);}
  .rgb-orb.cor-verde{background:radial-gradient(circle at 35% 35%, #8af0c8, var(--green));box-shadow:0 0 24px rgba(54,211,153,.6);}
  .rgb-orb.cor-amarelo{background:radial-gradient(circle at 35% 35%, #ffd98a, var(--orange));box-shadow:0 0 24px rgba(255,159,67,.6);animation:pulse .6s infinite;}
  .rgb-orb.cor-vermelho{background:radial-gradient(circle at 35% 35%, #ff9aa1, var(--red));box-shadow:0 0 24px rgba(255,92,102,.7);animation:pulse .35s infinite;}
  .rgb-info .legenda{font-size:11px;color:var(--text-2);line-height:1.5;}
  .rgb-info .legenda b{color:var(--text-1);}

  .wifi-row{display:flex;align-items:center;gap:10px;font-family:var(--font-mono);font-size:13px;}
  .wifi-row .ip{color:var(--blue);font-weight:700;}

  .fire-row{display:flex;align-items:center;justify-content:space-between;gap:10px;margin-bottom:10px;}
  .buzzer-badge{
    font-size:11px;font-weight:700;padding:5px 10px;border-radius:20px;
    border:1px solid var(--line-strong);color:var(--text-2);
  }
  .buzzer-badge.ativo{border-color:var(--red);color:var(--red);background:var(--red-dim);animation:pulse .3s infinite;}

  .conveyor-panel{
    background:var(--bg-1);
    border:1px solid var(--line);
    border-radius:var(--radius);
    padding:20px;
    margin-bottom:18px;
    overflow-x:auto;
  }
  .conveyor-panel .panel-title{margin-bottom:18px;}

  .factory-stage{
    position:relative;
    height:280px;
    min-width:980px;
    background:linear-gradient(180deg, rgba(255,255,255,0.015), transparent 30%), var(--bg-0);
    border:1px solid var(--line);
    border-radius:8px;
    padding:0 20px;
  }

  .actuator{
    position:absolute;
    top:8px;
    width:90px;height:170px;
    z-index:8;
  }
  .actuator .torre{
    position:absolute;
    left:50%; bottom:10px;
    width:10px; height:120px;
    background:var(--bg-3);
    border:2px solid var(--line-strong);
    border-radius:3px;
    transform:translateX(-50%);
    z-index:1;
  }
  .actuator .torre .fuso{
    position:absolute;inset:2px;
    background:repeating-linear-gradient(0deg, var(--line-strong) 0 4px, transparent 4px 8px);
    opacity:.6;
  }
  .actuator .carro{
    position:absolute;
    left:50%;
    bottom:14px;
    width:30px;height:14px;
    background:var(--bg-2);
    border:2px solid var(--line-strong);
    border-radius:3px;
    transform:translateX(-50%);
    z-index:2;
  }
  .actuator .braco-h{
    position:absolute;
    left:50%; top:50%;
    width:14px; height:8px;
    background:var(--bg-3);
    border:2px solid var(--line-strong);
    border-radius:2px;
    transform:translate(-50%,-50%);
    z-index:3;
  }
  .actuator .garfo{
    position:absolute;
    width:22px;height:6px;
    background:var(--bg-3);
    border:2px solid var(--line-strong);
    border-radius:2px;
    right:-24px;top:50%;
    transform:translateY(-50%);
  }
  .actuator .base-giro{
    position:absolute;
    left:50%; bottom:-2px;
    width:30px;height:10px;
    background:var(--bg-2);
    border:2px solid var(--line-strong);
    border-radius:50%;
    transform:translateX(-50%);
  }

  .actuator.movendo .torre{border-color:var(--orange);}
  .actuator.movendo .base-giro{border-color:var(--orange);}
  .actuator.movendo .carro{border-color:var(--orange);animation:atuadorTrabalhando 1s ease-in-out infinite;}
  .actuator.movendo .braco-h{border-color:var(--orange);}
  @keyframes atuadorTrabalhando{
    0%,100%{bottom:14px;}
    50%{bottom:90px;}
  }

  .actuator-label{
    position:absolute;
    bottom:-20px;left:50%;
    transform:translateX(-50%);
    font-family:var(--font-mono);font-size:8.5px;color:var(--text-2);
    white-space:nowrap;
  }

  .roller{
    position:absolute;width:20px;height:20px;border-radius:50%;
    background:var(--bg-3);border:2px solid var(--line-strong);
    top:50%;transform:translateY(-50%);z-index:3;
  }
  .roller::after{content:'';position:absolute;inset:4px;border-radius:50%;border:1px dashed var(--text-2);}
  .roller.drive.spin::after{animation:spin .5s linear infinite;}
  @keyframes spin{to{transform:rotate(360deg);}}

  .belt-track{
    position:absolute; top:50%; height:24px;
    transform:translateY(-50%);
    background:var(--bg-2);
    border-top:2px solid var(--line-strong);
    border-bottom:2px solid var(--line-strong);
    overflow:hidden;
  }
  .belt-track .surface{
    position:absolute;inset:0;
    background:repeating-linear-gradient(90deg, var(--bg-3) 0 16px, var(--bg-2) 16px 32px);
  }
  .belt-track.running .surface{animation:belttravel .5s linear infinite;}
  @keyframes belttravel{from{background-position:0 0;}to{background-position:-32px 0;}}

  .scanner-unit{
    position:absolute; top:6px; bottom:46px;
    width:26px;
    display:flex;flex-direction:column;justify-content:space-between;
    z-index:6;
  }
  .barreira{
    width:100%;height:7px;
    border-radius:2px;
    background:var(--bg-3);
    border:1.5px solid var(--red);
    transition:all .15s ease;
  }
  .barreira.ativa{border-color:var(--green);background:var(--green-dim);box-shadow:0 0 8px rgba(54,211,153,.5);}
  .scanner-label{
    position:absolute;bottom:-18px;left:50%;transform:translateX(-50%);
    font-size:8.5px;font-family:var(--font-mono);color:var(--text-2);white-space:nowrap;
  }

  .endstop-unit{
    position:absolute; top:50%;
    width:14px;height:14px;border-radius:3px;
    background:var(--bg-3);border:2px solid var(--red);
    transform:translateY(calc(-50% - 22px));
    transition:all .2s ease;
    z-index:6;
  }
  .endstop-unit.ativo{border-color:var(--green);background:var(--green);box-shadow:0 0 10px var(--green);}
  .endstop-label{
    position:absolute;top:-18px;left:50%;transform:translateX(-50%);
    font-size:8.5px;font-family:var(--font-mono);color:var(--text-2);white-space:nowrap;
  }

  .storage-zone{
    position:absolute;
    right:16px; top:10px; bottom:10px;
    width:170px;
    display:flex;flex-direction:column;gap:8px;
  }
  .storage-table{
    flex:1;
    display:flex;flex-direction:column;gap:6px;
    overflow-y:auto;
  }
  .storage-table .table-title{
    font-size:9px;font-family:var(--font-mono);color:var(--text-2);
    text-align:center;text-transform:uppercase;letter-spacing:.5px;
  }
  .storage-row{
    flex:1;
    display:flex;flex-direction:column;
    border:1px solid var(--line);border-radius:4px;
    background:var(--bg-1);
    padding:4px;
    gap:3px;
    min-height:40px;
  }
  .storage-row .row-label{font-size:7.5px;color:var(--text-2);font-family:var(--font-mono);}
  .storage-row .cells{display:flex;gap:3px;flex-wrap:wrap;flex:1;align-content:flex-start;}
  .storage-cell{
    width:14px;height:14px;
    border:1px dashed var(--line-strong);border-radius:3px;
    display:flex;align-items:center;justify-content:center;
    flex:0 0 auto;
  }
  .storage-cell.ocupada{
    border-style:solid;border-color:#7a5226;
    background:linear-gradient(135deg,#c98a4b,#a96f36);
  }
  .storage-cell.novo{
    animation:celulaNova .5s ease;
  }
  @keyframes celulaNova{
    0%{transform:scale(0.2);opacity:0;}
    60%{transform:scale(1.25);opacity:1;}
    100%{transform:scale(1);}
  }
  .storage-row.cat-P{border-left:3px solid var(--blue);}
  .storage-row.cat-M{border-left:3px solid var(--orange);}
  .storage-row.cat-G{border-left:3px solid var(--purple);}

  .guide-rail{position:absolute;height:2px;background:var(--line);opacity:.6;}

  .legend-row{display:flex;gap:18px;margin-top:14px;flex-wrap:wrap;font-size:11px;color:var(--text-2);}
  .legend-row span{display:flex;align-items:center;gap:6px;}
  .legend-row .sw{width:10px;height:10px;border-radius:2px;display:inline-block;}

  .test-result{
    display:flex;align-items:center;gap:14px;margin-top:12px;
    padding:12px;border-radius:8px;background:var(--bg-2);border:1px solid var(--line);
  }
  .test-result .badge-categoria{
    font-family:var(--font-mono);font-size:22px;font-weight:800;
    width:52px;height:52px;border-radius:10px;
    display:flex;align-items:center;justify-content:center;
    background:var(--bg-3);border:2px solid var(--line-strong);color:var(--text-1);
  }
  .test-result .badge-categoria.cat-P{border-color:var(--blue);color:var(--blue);}
  .test-result .badge-categoria.cat-M{border-color:var(--orange);color:var(--orange);}
  .test-result .badge-categoria.cat-G{border-color:var(--purple);color:var(--purple);}
</style>
</head>
<body>

<div class="app">

  <aside class="sidebar">

    <div class="conn-status desconectado" id="connStatus">
      <span class="dot-conn"></span>
      <span id="connStatusText">Conectando ao ESP32...</span>
    </div>
    <div class="sidebar-clock" id="clock">--:--:--</div>

    <div class="panel">
      <p class="panel-title"><span class="dot"></span>Controles principais</p>
      <div class="btn-grid">
        <button class="primary" id="btnLigar"><span class="ic">⏻</span>Ligar sistema</button>
        <button class="danger" id="btnDesligar"><span class="ic">⏻</span>Desligar sistema</button>
      </div>

      <p class="divider-label">Tipo de ciclo automático</p>
      <div class="cycle-toggle">
        <button id="btnCicloUnico">Ciclo único</button>
        <button id="btnCicloLoop">Loop contínuo</button>
      </div>

      <p class="divider-label">Ciclo automático</p>
      <div class="btn-grid">
        <button class="info" id="btnIniciarCiclo"><span class="ic">▶</span>Iniciar ciclo automático</button>
        <button id="btnPararCiclo"><span class="ic">■</span>Parar ciclo</button>
        <button id="btnResetar"><span class="ic">↺</span>Resetar processo</button>
      </div>
    </div>

    <div class="panel">
      <p class="panel-title"><span class="dot" style="background:var(--red);box-shadow:0 0 8px var(--red);"></span>Segurança</p>
      <button class="emergencia" id="btnEmergencia">⛔ PARADA DE EMERGÊNCIA</button>
      <button id="btnResetEmergencia" style="margin-top:8px;justify-content:center;">↺ Resetar após emergência</button>
    </div>

    <div class="panel">
      <p class="panel-title"><span class="dot" style="background:var(--purple);box-shadow:0 0 8px var(--purple);"></span>Classificações</p>
      <div class="btn-grid">
        <button class="info" id="btnTesteEsteira"><span class="ic">🧪</span>Iniciar Teste Esteira</button>
      </div>
      <div class="test-result">
        <div class="badge-categoria" id="testeBadge">-</div>
        <div>
          <div style="font-size:11px;color:var(--text-2);text-transform:uppercase;">Estado do teste</div>
          <div style="font-family:var(--font-mono);font-size:13px;" id="testeEstadoTxt">TESTE_OFF</div>
        </div>
      </div>
    </div>

    <footer style="margin-top:18px;font-size:11px;color:var(--text-2);text-align:center;">Sistema de Classificação de Caixas (ESP32)</footer>
  </aside>

  <main class="content">

    <div class="state-banner" id="stateBanner">
      <div class="left">
        <div class="icon" id="stateIcon">⏸</div>
        <div>
          <div class="name">Estado atual do processo</div>
          <div class="code" id="stateCode">OFF</div>
        </div>
      </div>
      <div class="counter">
        Caixas processadas (total)<br>
        <span class="n" id="counterValue">0</span>
      </div>
    </div>

    <div class="status-row">
      <div class="status-card">
        <span class="label">Sistema</span>
        <span class="value"><span class="led" id="ledSistema"></span><span id="txtSistema">Desligado</span></span>
      </div>
      <div class="status-card">
        <span class="label">Motor / esteira</span>
        <span class="value"><span class="led" id="ledMotor"></span><span id="txtMotor">Parado</span></span>
      </div>
      <div class="status-card">
        <span class="label">Atuador 1</span>
        <span class="value"><span class="led" id="ledAt1"></span><span id="txtAt1">Parado</span></span>
      </div>
      <div class="status-card">
        <span class="label">Atuador 2</span>
        <span class="value"><span class="led" id="ledAt2"></span><span id="txtAt2">Parado</span></span>
      </div>
      <div class="status-card">
        <span class="label">Última classificação</span>
        <span class="value"><span class="led on-blue" id="ledCategoria"></span><span id="txtCategoria">-</span></span>
      </div>
      <div class="status-card">
        <span class="label">Pequenas (P)</span>
        <span class="value"><span class="led on-blue"></span><span id="txtContadorP">0</span></span>
      </div>
      <div class="status-card">
        <span class="label">Médias (M)</span>
        <span class="value"><span class="led on-orange"></span><span id="txtContadorM">0</span></span>
      </div>
      <div class="status-card">
        <span class="label">Grandes (G)</span>
        <span class="value"><span class="led" style="background:var(--purple);box-shadow:0 0 9px var(--purple);"></span><span id="txtContadorG">0</span></span>
      </div>
    </div>

    <div class="hw-grid">
      <div class="hw-card">
        <p class="panel-title"><span class="dot"></span>LED RGB (plataforma PAT)</p>
        <div class="rgb-display">
          <div class="rgb-orb cor-azul" id="rgbOrb"></div>
          <div class="rgb-info">
            <div class="legenda">
              <div><b>Verde</b>: operação normal</div>
              <div><b>Azul</b>: inicializando / aguardando</div>
              <div><b>Amarelo</b>: identificando / manipulando</div>
              <div><b>Vermelho</b>: erro / emergência</div>
            </div>
          </div>
        </div>
      </div>

      <div class="hw-card">
        <p class="panel-title"><span class="dot"></span>Conectividade Wi-Fi</p>
        <div class="wifi-row">
          <span class="led on-red" id="ledWifi"></span>
          <span id="txtWifiStatus">Desconectado</span>
          <span class="ip" id="txtWifiIp">-</span>
        </div>
      </div>

      <div class="hw-card">
        <p class="panel-title"><span class="dot" style="background:var(--red);box-shadow:0 0 8px var(--red);"></span>Alarme: Emergência</p>
        <div class="fire-row">
          <span style="font-size:11px;color:var(--text-2);">Sistema em emergência</span>
          <span class="buzzer-badge" id="emergenciaBadge">Sem dados</span>
        </div>
      </div>

      <div class="hw-card">
        <p class="panel-title"><span class="dot" style="background:var(--orange);box-shadow:0 0 8px var(--orange);"></span>Alarme: Sensor de Chama</p>
        <div class="fire-row">
          <span style="font-size:11px;color:var(--text-2);">Leitura do sensor (bruta)</span>
          <span class="buzzer-badge" id="chamaBadge">Sem dados</span>
        </div>
      </div>
    </div>

    <div class="conveyor-panel">
      <p class="panel-title"><span class="dot"></span>Visualização da planta (vista lateral)</p>

      <div class="factory-stage" id="stage">

        <div class="actuator" id="atuador1" style="left:60px;">
          <div class="torre"><div class="fuso"></div></div>
          <div class="carro" id="carro1">
            <div class="braco-h" id="braco1"><div class="garfo"></div></div>
          </div>
          <div class="base-giro"></div>
          <div class="actuator-label">ATUADOR 1</div>
        </div>

        <div class="roller drive" id="rollerDrive" style="left:170px;"></div>
        <div class="belt-track" id="beltTrack" style="left:170px;right:230px;"><div class="surface"></div></div>
        <div class="roller idle" style="right:230px;"></div>

        <div class="guide-rail" style="left:170px;right:230px;top:calc(50% - 18px);"></div>
        <div class="guide-rail" style="left:170px;right:230px;top:calc(50% + 18px);"></div>

        <div class="scanner-unit" style="left:300px;">
          <div class="barreira" id="barreira3"></div>
          <div class="barreira" id="barreira2"></div>
          <div class="barreira" id="barreira1"></div>
          <div class="scanner-label">SCANNER</div>
        </div>

        <div class="endstop-unit" id="endstopUnit" style="left:430px;">
          <div class="endstop-label">FIM DE CURSO</div>
        </div>

        <div class="actuator" id="atuador2" style="left:520px;">
          <div class="torre"><div class="fuso"></div></div>
          <div class="carro" id="carro2">
            <div class="braco-h" id="braco2"><div class="garfo"></div></div>
          </div>
          <div class="base-giro"></div>
          <div class="actuator-label">ATUADOR 2</div>
        </div>

        <div class="storage-zone">
          <div class="storage-table" id="tabelaOrganizadaEl">
            <div class="table-title">Armazenamento</div>
            <div class="storage-row cat-P">
              <span class="row-label">P</span>
              <div class="cells" id="cellsP"></div>
            </div>
            <div class="storage-row cat-M">
              <span class="row-label">M</span>
              <div class="cells" id="cellsM"></div>
            </div>
            <div class="storage-row cat-G">
              <span class="row-label">G</span>
              <div class="cells" id="cellsG"></div>
            </div>
          </div>
        </div>

      </div>

      <div class="legend-row">
        <span><span class="sw" style="background:var(--orange);"></span>Atuadores (NEMA17 + fuso + cremalheira)</span>
        <span><span class="sw" style="background:#a96f36;"></span>Caixa classificada (armazenamento)</span>
        <span><span class="sw" style="background:var(--red);"></span>Barreira / sensor (livre)</span>
        <span><span class="sw" style="background:var(--green);"></span>Barreira / sensor (acionado)</span>
      </div>
    </div>

    <div class="hw-card" style="margin-bottom:18px;">
      <p class="panel-title"><span class="dot"></span>Registro de eventos</p>
      <div id="logList" style="max-height:240px;overflow-y:auto;display:flex;flex-direction:column-reverse;gap:1px;font-family:var(--font-mono);font-size:12px;border:1px solid var(--line);border-radius:6px;background:var(--bg-0);"></div>
    </div>

  </main>
</div>

<script>

const POLL_INTERVALO_MS = 1000;

const ui = {
  conectado: false,
  ultimoStatus: null,
  ultimoSensores: null,
  tipoCicloSelecionado: 'unico',
  contadorPAnterior: 0,
  contadorMAnterior: 0,
  contadorGAnterior: 0,
  logsRenderizados: 0
};

function horaAtual(){
  const d = new Date();
  return d.toLocaleTimeString('pt-BR', {hour:'2-digit', minute:'2-digit', second:'2-digit'});
}

function chamar(url){
  fetch(url).catch(e => console.log('erro em ' + url, e));
}

async function buscarStatus(){
  try{
    const r = await fetch('/status');
    const s = await r.json();
    ui.conectado = true;
    ui.ultimoStatus = s;
    return s;
  }catch(e){
    ui.conectado = false;
    return null;
  }
}

async function buscarSensores(){
  try{
    const r = await fetch('/sensores');
    const s = await r.json();
    ui.ultimoSensores = s;
    return s;
  }catch(e){
    return null;
  }
}

async function buscarEventos(){
  try{
    const r = await fetch('/events');
    const ev = await r.json();
    renderLog(ev);
  }catch(e){  }
}

document.getElementById('btnLigar').addEventListener('click', () => chamar('/sistema_on'));
document.getElementById('btnDesligar').addEventListener('click', () => chamar('/sistema_off'));
document.getElementById('btnPararCiclo').addEventListener('click', () => chamar('/stop'));
document.getElementById('btnResetar').addEventListener('click', () => chamar('/resetproc'));
document.getElementById('btnEmergencia').addEventListener('click', () => chamar('/emergency'));
document.getElementById('btnResetEmergencia').addEventListener('click', () => chamar('/resetemergency'));
document.getElementById('btnTesteEsteira').addEventListener('click', () => chamar('/testesteira'));

document.getElementById('btnCicloUnico').addEventListener('click', () => {
  ui.tipoCicloSelecionado = 'unico';
  atualizarBotoesTipoCiclo();
});
document.getElementById('btnCicloLoop').addEventListener('click', () => {
  ui.tipoCicloSelecionado = 'loop';
  atualizarBotoesTipoCiclo();
});
document.getElementById('btnIniciarCiclo').addEventListener('click', () => {
  if (ui.tipoCicloSelecionado === 'loop') chamar('/start?loop=1');
  else chamar('/start?loop=0');
});

function atualizarBotoesTipoCiclo(){
  document.getElementById('btnCicloUnico').classList.toggle('active', ui.tipoCicloSelecionado === 'unico');
  document.getElementById('btnCicloLoop').classList.toggle('active', ui.tipoCicloSelecionado === 'loop');
}

function setLed(ledId, txtId, corOn, texto){
  const led = document.getElementById(ledId);
  const txt = document.getElementById(txtId);
  led.className = 'led';
  if (corOn) led.classList.add('on-' + corOn);
  if (txt) txt.textContent = texto;
}

function atualizarConexaoUI(){
  const el = document.getElementById('connStatus');
  const txt = document.getElementById('connStatusText');
  el.classList.toggle('conectado', ui.conectado);
  el.classList.toggle('desconectado', !ui.conectado);
  txt.textContent = ui.conectado ? 'Conectado ao ESP32' : 'Desconectado, tentando novamente...';
}

const iconesPorEstado = {
  OFF:'⏻', IDLE:'⏸', B1_MOVENDO:'🦾', ESTEIRA:'▶', FIM_DE_CURSO:'◎',
  B2_PEGANDO:'🦾', B2_SOLTANDO:'🦾', CICLO_CONCLUIDO:'✔', EMERGENCIA:'⛔'
};

function atualizarInterface(s, sensores){
  if (!s) return;

  document.getElementById('stateCode').textContent = s.estado;
  document.getElementById('counterValue').textContent = s.contadorTotal;
  document.getElementById('stateBanner').classList.toggle('emergencia', s.emergencia);
  document.getElementById('stateIcon').textContent = iconesPorEstado[s.estado] || '?';

  setLed('ledSistema', 'txtSistema', s.ligado ? 'green' : 'red', s.ligado ? 'Ligado' : 'Desligado');
  setLed('ledMotor', 'txtMotor', s.esteira ? 'blue' : null, s.esteira ? 'Rodando' : 'Parado');

  const at1Movendo = s.manip1 && s.manip1 !== 'Parado';
  const at2Movendo = s.manip2 && s.manip2 !== 'Parado';
  setLed('ledAt1', 'txtAt1', at1Movendo ? 'orange' : null, s.manip1 || 'Parado');
  setLed('ledAt2', 'txtAt2', at2Movendo ? 'orange' : null, s.manip2 || 'Parado');
  setLed('ledCategoria', 'txtCategoria', (s.categoria && s.categoria !== '-') ? 'blue' : null, s.categoria || '-');

  document.getElementById('txtContadorP').textContent = s.contadorP;
  document.getElementById('txtContadorM').textContent = s.contadorM;
  document.getElementById('txtContadorG').textContent = s.contadorG;

  const semConexao = !ui.conectado;
  document.getElementById('btnLigar').disabled = semConexao || s.ligado;
  document.getElementById('btnDesligar').disabled = semConexao || !s.ligado;
  document.getElementById('btnIniciarCiclo').disabled = semConexao || !s.ligado || s.emergencia;
  document.getElementById('btnPararCiclo').disabled = semConexao;
  document.getElementById('btnEmergencia').disabled = semConexao || s.emergencia || !s.ligado;
  document.getElementById('btnResetEmergencia').disabled = semConexao || !s.emergencia;
  atualizarBotoesTipoCiclo();

  document.getElementById('rollerDrive').classList.toggle('spin', !!s.esteira);
  document.getElementById('beltTrack').classList.toggle('running', !!s.esteira);

  document.getElementById('atuador1').classList.toggle('movendo', at1Movendo);
  document.getElementById('atuador2').classList.toggle('movendo', at2Movendo);

  if (sensores){
    document.getElementById('barreira1').classList.toggle('ativa', sensores.s1 === 'DETECTADO');
    document.getElementById('barreira2').classList.toggle('ativa', sensores.s2 === 'DETECTADO');
    document.getElementById('barreira3').classList.toggle('ativa', sensores.s3 === 'DETECTADO');
    document.getElementById('endstopUnit').classList.toggle('ativo', sensores.fimCurso === 'DETECTADO');

    const emergBadge = document.getElementById('emergenciaBadge');
    emergBadge.textContent = s.emergencia ? 'EMERGÊNCIA ATIVA' : 'Normal';
    emergBadge.classList.toggle('ativo', s.emergencia);

    const chamaBadge = document.getElementById('chamaBadge');
    chamaBadge.textContent = sensores.alarme === 'ACIONADO' ? 'SENSOR ACIONADO' : 'Normal';
    chamaBadge.classList.toggle('ativo', sensores.alarme === 'ACIONADO');
  }

  atualizarTabelas(s.contadorP, s.contadorM, s.contadorG);

  document.getElementById('rgbOrb').className = 'rgb-orb cor-' + (s.rgb || 'azul');

  setLed('ledWifi', 'txtWifiStatus', ui.conectado ? 'green' : 'red', ui.conectado ? 'Conectado' : 'Desconectado');
  document.getElementById('txtWifiIp').textContent = s.ip || '-';

  document.getElementById('testeEstadoTxt').textContent = s.estadoTeste || 'TESTE_OFF';
  const badge = document.getElementById('testeBadge');
  const cat = s.categoria && s.categoria !== '-' ? s.categoria : '-';
  badge.textContent = cat;
  badge.className = 'badge-categoria' + (cat === 'P' ? ' cat-P' : cat === 'M' ? ' cat-M' : cat === 'G' ? ' cat-G' : '');
}

function atualizarTabelas(p, m, g){
  renderCelulas('cellsP', p, ui.contadorPAnterior);
  renderCelulas('cellsM', m, ui.contadorMAnterior);
  renderCelulas('cellsG', g, ui.contadorGAnterior);
  ui.contadorPAnterior = p;
  ui.contadorMAnterior = m;
  ui.contadorGAnterior = g;
}

function renderCelulas(containerId, total, totalAnterior){
  const el = document.getElementById(containerId);
  const atual = el.children.length;
  if (atual === total) return;

  el.innerHTML = '';
  for (let i = 0; i < total; i++){
    const cell = document.createElement('div');
    cell.className = 'storage-cell ocupada';
    if (i >= totalAnterior) cell.classList.add('novo');
    el.appendChild(cell);
  }
}

function renderLog(eventos){
  const list = document.getElementById('logList');
  list.innerHTML = '';
  eventos.forEach(item => {
    const row = document.createElement('div');
    const cor = item.nivel === 'ok' ? 'var(--green)' : item.nivel === 'warn' ? 'var(--orange)' : item.nivel === 'error' ? 'var(--red)' : 'var(--blue)';
    row.style.display = 'flex';
    row.style.gap = '10px';
    row.style.padding = '7px 12px';
    row.style.borderBottom = '1px solid var(--line)';
    row.innerHTML = '<span style="color:var(--text-2);flex-shrink:0;">[' + item.hora + ']</span><span style="color:' + cor + ';">' + item.texto + '</span>';
    list.appendChild(row);
  });
}

function atualizarRelogio(){
  document.getElementById('clock').textContent = horaAtual();
}
setInterval(atualizarRelogio, 1000);
atualizarRelogio();

async function cicloAtualizacao(){
  const status = await buscarStatus();
  atualizarConexaoUI();
  if (status){
    const sensores = await buscarSensores();
    atualizarInterface(status, sensores);
  }
  await buscarEventos();
}

setInterval(cicloAtualizacao, POLL_INTERVALO_MS);
cicloAtualizacao();

</script>
</body>
</html>

)HTMLPAGE");
}

// Leitura bruta dos sensores para a pagina e para diagnostico
void handleSensores() {
  bool s1, s2, s3, fc, alarme, comOk, hbMudou;
  xSemaphoreTake(mutexEstado, portMAX_DELAY);
  s1 = estadoGlobal.rawS1;
  s2 = estadoGlobal.rawS2;
  s3 = estadoGlobal.rawS3;
  fc = estadoGlobal.rawFimCurso;
  alarme = estadoGlobal.rawAlarme;
  comOk = estadoGlobal.megaComunicando;
  hbMudou = estadoGlobal.megaHeartbeatMudou;
  xSemaphoreGive(mutexEstado);

  String json = "{";
  json += "\"s1\":\"" + String(s1?"DETECTADO":"livre") + "\",";
  json += "\"s2\":\"" + String(s2?"DETECTADO":"livre") + "\",";
  json += "\"s3\":\"" + String(s3?"DETECTADO":"livre") + "\",";
  json += "\"fimCurso\":\"" + String(fc?"DETECTADO":"livre") + "\",";
  json += "\"alarme\":\"" + String(alarme?"ACIONADO":"livre") + "\",";
  json += "\"megaComunicando\":" + String(comOk?"true":"false") + ",";
  json += "\"megaHeartbeatMudou\":" + String(hbMudou?"true":"false");
  json += "}";
  server.send(200, "application/json", json);
}

// Inicia o teste da esteira
void handleTesteEsteira() {
  bool jaAtivo;
  xSemaphoreTake(mutexEstado, portMAX_DELAY);
  jaAtivo = estadoGlobal.testeAtivo;
  xSemaphoreGive(mutexEstado);

  if (jaAtivo) {
    server.send(409, "text/plain", "Teste ja em andamento");
    return;
  }

  xSemaphoreTake(mutexEstado, portMAX_DELAY);
  estadoGlobal.latchS1 = estadoGlobal.latchS2 = estadoGlobal.latchS3 = false;
  estadoGlobal.latchTesteS1 = estadoGlobal.latchTesteS2 = estadoGlobal.latchTesteS3 = false;
  estadoGlobal.categoriaAtual = "-";
  estadoGlobal.testeAtivo = true;
  estadoGlobal.estadoTeste = TESTE_LIGADO;
  estadoGlobal.esteiraLigadaFlag = true;
  estadoGlobal.resetFlagsPendente = true;
  xSemaphoreGive(mutexEstado);

  enviaComandoMega("ce");
  addLog("Teste esteira iniciado -- esteira ligada, monitorando sensores", "info");
  server.send(200, "text/plain", "OK");
}

TaskHandle_t handleTaskControle, handleTaskComunicacao;

// Inicializacao: LEDs, botoes, I2C, OLED, Wi-Fi, timer, rotas e tarefas
void setup() {
  Serial.begin(115200);
  pinMode(LEDR_PIN, OUTPUT);
  pinMode(LEDB_PIN, OUTPUT);
  digitalWrite(LEDR_PIN, LOW);
  digitalWrite(LEDB_PIN, LOW);
  ledcAttach(LEDG_PIN, RGB_PWM_FREQ, RGB_PWM_RESOLUCAO);
  ledcWrite(LEDG_PIN, 0);

  pinMode(TLED_PIN, OUTPUT);
  digitalWrite(TLED_PIN, HIGH);

  setRGB("azul");
  pinMode(BUZZ_PIN, OUTPUT); digitalWrite(BUZZ_PIN, LOW);

  pinMode(BTN_S4_EMERGENCIA_PIN, INPUT_PULLUP);
  pinMode(BTN_S6_RESET_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_S4_EMERGENCIA_PIN), isrEmergenciaS4, FALLING);

  Wire.begin(21, 22);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) Serial.println("OLED nao encontrado!");

  oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
  oledCtr(0,"EQUIPE"); oled.drawLine(0,9,127,9,SSD1306_WHITE);
  oled.setCursor(0,14); oled.println("Gabriela B. Azevedo");
  oled.setCursor(0,25); oled.println("Felipe K. M. Martins");
  oled.setCursor(0,36); oled.println("Lucas H. Horn");
  oled.setCursor(0,50); oled.println("Iniciando...");
  oled.display(); digitalWrite(27,LOW); digitalWrite(14,LOW);
  delay(2000);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(400); Serial.print("."); }
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  oled.clearDisplay(); oledCtr(8,"WiFi conectado");
  oled.setCursor(0,28); oled.print("Rede: "); oled.println(ssid);
  oled.setCursor(0,42); oled.print("IP: "); oled.println(WiFi.localIP());
  oled.display(); digitalWrite(27,LOW); digitalWrite(14,LOW);
  delay(1500);

  timerPeriodico = timerBegin(1000000);
  timerAttachInterrupt(timerPeriodico, &isrTimerPeriodico);
  timerAlarm(timerPeriodico, 1000000, true, 0);

  mutexEstado = xSemaphoreCreateMutex();
  filaLog = xQueueCreate(20, sizeof(LogMsg));

  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/events", HTTP_GET, handleEvents);
  server.on("/sistema_on", HTTP_GET, handleSistemaOn);
  server.on("/sistema_off", HTTP_GET, handleSistemaOff);
  server.on("/start", HTTP_GET, handleStart);
  server.on("/stop", HTTP_GET, handleStop);
  server.on("/resetproc", HTTP_GET, handleResetProc);
  server.on("/emergency", HTTP_GET, handleEmergency);
  server.on("/resetemergency", HTTP_GET, handleResetEmergency);
  server.on("/pp", HTTP_GET, handlePP);
  server.on("/sp", HTTP_GET, handleSP);
  server.on("/pp2", HTTP_GET, handlePP2);
  server.on("/sp2", HTTP_GET, handleSP2);
  server.on("/ce", HTTP_GET, handleConveyorOn);
  server.on("/cd", HTTP_GET, handleConveyorOff);
  server.on("/scan", HTTP_GET, handleScan);
  server.on("/scanbeep", HTTP_GET, handleScanBeep);
  server.on("/motor", HTTP_GET, handleMotor);
  server.on("/motorctl", HTTP_GET, handleMotorCtl);
  server.on("/testesteira", HTTP_GET, handleTesteEsteira);
  server.on("/sensores", HTTP_GET, handleSensores);
  server.begin();

  addLog("Sistema inicializado, aguardando comando", "info");

  xTaskCreatePinnedToCore(TaskControle,    "TaskControle",    10000, NULL, 1, &handleTaskControle,    0);
  xTaskCreatePinnedToCore(TaskComunicacao, "TaskComunicacao", 10000, NULL, 1, &handleTaskComunicacao, 1);
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
