/*
 * Balança Inteligente para Botijão de GLP — V3
 *
 * Hardware: ESP32 + HX711 + 4 células de carga 50 kg (meia ponte)
 * Pinos:    DT -> GPIO19,  SCK -> GPIO18
 * Alimentação: 3,3 V
 *
 * Modelo: RAW = Offset + Peso
 *   - Offset estimado continuamente (compensa deriva mecânica/térmica/eletrônica)
 *   - Tara fixa abandonada
 *
 * Estados: STARTUP -> ESTABILIZACAO -> SEM_PESO -> CAPTURA_DE_PESO -> MONITORAMENTO
 */

#include <HX711.h>
#include <WiFi.h>
#include <WebServer.h>

// ─── Wi-Fi ────────────────────────────────────────────────────────────────────
#define WIFI_SSID     "valhalla"
#define WIFI_PASSWORD "valhallatudominusculo"

// ─── Pinos ───────────────────────────────────────────────────────────────────
#define HX711_DT   19
#define HX711_SCK  18

// ─── Calibração ──────────────────────────────────────────────────────────────
// Fator obtido experimentalmente: ~18504 RAW por kg (512g → ~9474 RAW)
#define FATOR_RAW_POR_KG  18504.0

// ─── Botijão GLP ─────────────────────────────────────────────────────────────
// P13: botijão vazio ~13 kg, carga nominal 13 kg de gás
#define PESO_BOTIJAO_VAZIO_KG  13.0
#define CAPACIDADE_GLP_KG      13.0

// ─── Parâmetros de leitura ───────────────────────────────────────────────────
#define LEITURAS_POR_AMOSTRA   10    // leituras para média no HX711
#define JANELA_ESTABILIDADE    10    // amostras na janela de estabilidade
#define JANELAS_CONFIAVEIS      5    // janelas estáveis consecutivas para confiança
#define DESVIO_MAX_ESTAVEL   100.0   // desvio padrão máximo para considerar estável (RAW)
#define TENDENCIA_MAX         50.0   // diferença máxima entre 1a e 2a metade da janela (RAW)

// ─── Parâmetros de detecção (em RAW) ────────────────────────────────────────
#define LIMIAR_PESO_PRESENTE  3000.0 // diferença RAW mínima para detectar peso (~140g)
#define LIMIAR_PESO_REMOVIDO  1500.0 // abaixo disso considera plataforma livre (~70g)

// ─── Zero tracking (SEM_PESO) ───────────────────────────────────────────────
// Aplica correção proporcional ao erro, com taxa variável:
//   |erro| < FAIXA_FINA  → taxa baixa (ajuste fino)
//   |erro| < FAIXA_GROSSA → taxa alta (recuperação rápida pós-remoção de peso)
//   |erro| >= FAIXA_GROSSA → sem correção (pode ter peso real)
#define ZERO_TRACK_FAIXA_FINA    500.0
#define ZERO_TRACK_FAIXA_GROSSA 2500.0
#define ZERO_TRACK_TAXA_FINA      0.02
#define ZERO_TRACK_TAXA_GROSSA    0.08

// ─── Filtro exponencial (peso filtrado) ──────────────────────────────────────
#define ALFA_FILTRO             0.3  // suavização do peso filtrado (0–1, menor = mais suave)

// ─── Histórico de consumo ─────────────────────────────────────────────────
#define HISTORICO_MAX          1440   // amostras (24h a 1/min)
#define HISTORICO_INTERVALO_MS 60000  // 1 amostra por minuto

// ─── Intervalo de loop ──────────────────────────────────────────────────────
#define INTERVALO_LEITURA_MS  500

// ─── Estados ─────────────────────────────────────────────────────────────────
enum Estado {
  STARTUP,
  ESTABILIZACAO,
  SEM_PESO,
  CAPTURA_DE_PESO,
  MONITORAMENTO
};

// ─── Variáveis globais ───────────────────────────────────────────────────────
HX711 scale;
WebServer server(80);

Estado estadoAtual = STARTUP;

float rawHX           = 0.0;
float offsetEstimado  = 0.0;
float pesoEstimado    = 0.0;  // em RAW
float pesoFiltrado    = 0.0;  // em RAW
float pesoKg          = 0.0;  // convertido para kg
float glpKg           = 0.0;  // massa de gás estimada (kg)
float glpPercentual   = 0.0;  // gás restante (%)

// Janela de estabilidade
float bufferJanela[JANELA_ESTABILIDADE];
int   idxJanela           = 0;
bool  janelaCheia         = false;
int   janelasEstaveis     = 0;

// Timestamp
unsigned long tempoInicio = 0;

// Leitura HX711 em core separado
volatile float rawHX_pronto = 0.0;
volatile bool  novaLeitura  = false;

// Configuração do container (alterável via URL: ?cheio=XX&vazio=YY)
float cfgPesoVazio   = PESO_BOTIJAO_VAZIO_KG;
float cfgPesoCheio   = PESO_BOTIJAO_VAZIO_KG + CAPACIDADE_GLP_KG;
float cfgCapacidade  = CAPACIDADE_GLP_KG;

// Histórico de consumo (ring buffer)
struct AmostraHistorico {
  unsigned long tempoSeg;  // uptime em segundos
  float glpKg;             // gás restante naquele momento
};

AmostraHistorico historico[HISTORICO_MAX];
int   idxHistorico       = 0;
int   numAmostras        = 0;
unsigned long ultimaAmostra = 0;
float consumoKgH         = 0.0;  // taxa de consumo estimada (kg/h)
float autonomiaHoras     = -1.0; // horas restantes estimadas (-1 = sem dados)

// ─── Funções auxiliares ──────────────────────────────────────────────────────

float lerMediaRAW() {
  long soma = 0;
  for (int i = 0; i < LEITURAS_POR_AMOSTRA; i++) {
    soma += scale.read();
  }
  return (float)soma / LEITURAS_POR_AMOSTRA;
}

void adicionarJanela(float valor) {
  bufferJanela[idxJanela] = valor;
  idxJanela = (idxJanela + 1) % JANELA_ESTABILIDADE;
  if (!janelaCheia && idxJanela == 0) {
    janelaCheia = true;
  }
}

float mediaJanela() {
  int n = janelaCheia ? JANELA_ESTABILIDADE : idxJanela;
  if (n == 0) return 0.0;
  float soma = 0.0;
  for (int i = 0; i < n; i++) {
    soma += bufferJanela[i];
  }
  return soma / n;
}

float desvioJanela() {
  int n = janelaCheia ? JANELA_ESTABILIDADE : idxJanela;
  if (n < 2) return 9999.0;
  float media = mediaJanela();
  float somaQuad = 0.0;
  for (int i = 0; i < n; i++) {
    float diff = bufferJanela[i] - media;
    somaQuad += diff * diff;
  }
  return sqrt(somaQuad / (n - 1));
}

// Calcula a tendência: diferença entre média da 2a metade e média da 1a metade
// Valor positivo = subindo, negativo = descendo
float tendenciaJanela() {
  if (!janelaCheia) return 9999.0;
  int metade = JANELA_ESTABILIDADE / 2;
  float soma1 = 0.0, soma2 = 0.0;
  // A janela é circular: idxJanela aponta para o próximo a ser escrito (= o mais antigo)
  for (int i = 0; i < metade; i++) {
    soma1 += bufferJanela[(idxJanela + i) % JANELA_ESTABILIDADE];
  }
  for (int i = metade; i < JANELA_ESTABILIDADE; i++) {
    soma2 += bufferJanela[(idxJanela + i) % JANELA_ESTABILIDADE];
  }
  return (soma2 / (JANELA_ESTABILIDADE - metade)) - (soma1 / metade);
}

void resetarJanela() {
  idxJanela = 0;
  janelaCheia = false;
  janelasEstaveis = 0;
}

void calcularGLP() {
  glpKg = pesoKg - cfgPesoVazio;
  if (glpKg < 0.0) glpKg = 0.0;
  glpPercentual = (glpKg / cfgCapacidade) * 100.0;
  if (glpPercentual > 100.0) glpPercentual = 100.0;
}

void aplicarParametros() {
  if (server.hasArg("vazio")) {
    cfgPesoVazio = server.arg("vazio").toFloat();
  }
  if (server.hasArg("cheio")) {
    cfgPesoCheio = server.arg("cheio").toFloat();
  }
  cfgCapacidade = cfgPesoCheio - cfgPesoVazio;
  if (cfgCapacidade < 0.01) cfgCapacidade = 0.01;
}

String tempoUptime() {
  unsigned long seg = (millis() - tempoInicio) / 1000;
  unsigned long h = seg / 3600;
  unsigned long m = (seg % 3600) / 60;
  unsigned long s = seg % 60;
  char buf[16];
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", h, m, s);
  return String(buf);
}

void registrarAmostra() {
  unsigned long agora = millis();
  if (agora - ultimaAmostra < HISTORICO_INTERVALO_MS) return;
  ultimaAmostra = agora;

  historico[idxHistorico].tempoSeg = (agora - tempoInicio) / 1000;
  historico[idxHistorico].glpKg    = glpKg;
  idxHistorico = (idxHistorico + 1) % HISTORICO_MAX;
  if (numAmostras < HISTORICO_MAX) numAmostras++;

  calcularConsumo();
}

void calcularConsumo() {
  // Calcula taxa de consumo em kg/h usando últimas 10 amostras (ou todas se menos)
  if (numAmostras < 2) { consumoKgH = 0.0; autonomiaHoras = -1.0; return; }

  int n = min(numAmostras, 10);
  // Índice da amostra mais recente
  int iRecente = (idxHistorico - 1 + HISTORICO_MAX) % HISTORICO_MAX;
  // Índice da amostra n posições atrás
  int iAntiga  = (idxHistorico - n + HISTORICO_MAX) % HISTORICO_MAX;

  float deltaKg  = historico[iAntiga].glpKg - historico[iRecente].glpKg;
  float deltaSeg = (float)(historico[iRecente].tempoSeg - historico[iAntiga].tempoSeg);

  if (deltaSeg < 1.0) { consumoKgH = 0.0; autonomiaHoras = -1.0; return; }
  consumoKgH = (deltaKg / deltaSeg) * 3600.0;
  if (consumoKgH < 0.0) consumoKgH = 0.0; // não exibe consumo negativo (ganho = ruído)

  // Estimativa de autonomia
  if (consumoKgH > 0.001) {
    autonomiaHoras = glpKg / consumoKgH;
  } else {
    autonomiaHoras = -1.0; // consumo desprezível — sem estimativa
  }
}

// Retorna amostra pelo índice lógico (0 = mais antiga)
AmostraHistorico amostraLogica(int i) {
  int real = (idxHistorico - numAmostras + i + HISTORICO_MAX) % HISTORICO_MAX;
  return historico[real];
}

String formatarAutonomia() {
  if (autonomiaHoras < 0) return "---";
  if (autonomiaHoras < 1.0) {
    int min = (int)(autonomiaHoras * 60.0);
    return String(min) + " min";
  }
  if (autonomiaHoras < 48.0) {
    int h = (int)autonomiaHoras;
    int min = (int)((autonomiaHoras - h) * 60.0);
    return String(h) + "h " + String(min) + "min";
  }
  int dias = (int)(autonomiaHoras / 24.0);
  int h = (int)(autonomiaHoras - dias * 24.0);
  return String(dias) + "d " + String(h) + "h";
}

void handleHistorico() {
  String json = "[";
  for (int i = 0; i < numAmostras; i++) {
    AmostraHistorico a = amostraLogica(i);
    if (i > 0) json += ",";
    json += "[" + String(a.tempoSeg) + "," + String(a.glpKg, 3) + "]";
  }
  json += "]";
  server.send(200, "application/json", json);
}

void handleRoot() {
  aplicarParametros();
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Balanca GLP</title>";
  html += "<style>";
  html += "body{font-family:sans-serif;background:#1a1a2e;color:#eee;margin:0;padding:20px;text-align:center}";
  html += ".card{background:#16213e;border-radius:12px;padding:20px;margin:10px auto;max-width:400px}";
  html += ".valor{font-size:3em;font-weight:bold;color:#0f0}";
  html += ".label{font-size:0.9em;color:#aaa;margin-top:4px}";
  html += ".barra-fundo{background:#333;border-radius:8px;height:30px;margin:10px auto;max-width:350px;overflow:hidden}";
  html += ".barra{height:100%;border-radius:8px;transition:width 0.5s}";
  html += ".info{font-size:0.85em;color:#888;margin:4px 0}";
  html += "h1{margin:0 0 16px}";
  html += "</style></head><body>";

  html += "<h1>Balanca Inteligente GLP</h1>";

  // Cartao GLP
  html += "<div class='card'>";
  html += "<div class='label'>Gas restante</div>";
  html += "<div class='valor' id='glpKg'>--</div>";
  html += "<div class='barra-fundo'><div class='barra' id='barra'></div></div>";
  html += "<div class='label' id='glpPct'>--</div>";
  html += "</div>";

  // Cartao peso total
  html += "<div class='card'>";
  html += "<div class='label'>Peso total na plataforma</div>";
  html += "<div style='font-size:2em;font-weight:bold' id='pesoKg'>--</div>";
  html += "</div>";

  // Cartao consumo + autonomia
  html += "<div class='card'>";
  html += "<div class='label'>Consumo estimado</div>";
  html += "<div style='font-size:2em;font-weight:bold;color:#fa0' id='consumo'>--</div>";
  html += "<div style='margin-top:12px'></div>";
  html += "<div class='label'>Autonomia estimada</div>";
  html += "<div style='font-size:2em;font-weight:bold' id='autonomia'>--</div>";
  html += "<div class='label' id='amostras'>--</div>";
  html += "</div>";

  // Grafico historico (SVG gerado via JS com dados do /api/historico)
  html += "<div class='card'>";
  html += "<div class='label'>Historico de consumo</div>";
  html += "<div id='grafico' style='margin-top:10px'></div>";
  html += "</div>";

  // Cartao configuracao
  html += "<div class='card'>";
  html += "<div class='label'>Configuracao do container</div>";
  html += "<div class='info'>Peso vazio: <b>" + String(cfgPesoVazio, 3) + " kg</b> | Peso cheio: <b>" + String(cfgPesoCheio, 3) + " kg</b></div>";
  html += "<div class='info'>Capacidade: " + String(cfgCapacidade, 3) + " kg</div>";
  html += "<div class='info' style='color:#666;margin-top:6px'>Alterar: ?cheio=XX&amp;vazio=YY</div>";
  html += "</div>";

  // Cartao estado
  html += "<div class='card'>";
  html += "<div class='info'>Estado: <b id='estado'>--</b></div>";
  html += "<div class='info'>Uptime: <span id='uptime'>--</span></div>";
  html += "<div class='info'>IP: " + WiFi.localIP().toString() + "</div>";
  html += "</div>";

  html += "<script>";
  // Busca historico e desenha grafico SVG
  html += "function desenhar(){";
  html += "fetch('/api/historico').then(r=>r.json()).then(function(d){";
  html += "var g=document.getElementById('grafico');";
  html += "if(!d.length){g.innerHTML='<p style=\"color:#888\">Aguardando dados...</p>';return}";
  html += "var W=360,H=150,pad=40;";
  // Encontra min/max
  html += "var t0=d[0][0],t1=d[d.length-1][0];";
  html += "var vmin=999,vmax=0;";
  html += "for(var i=0;i<d.length;i++){if(d[i][1]<vmin)vmin=d[i][1];if(d[i][1]>vmax)vmax=d[i][1]}";
  html += "if(vmax-vmin<0.5){vmin=Math.max(0,vmin-0.5);vmax=vmax+0.5}";
  // Gera polyline
  html += "var pts='';";
  html += "for(var i=0;i<d.length;i++){";
  html += "var x=pad+(d[i][0]-t0)/(Math.max(t1-t0,1))*(W-pad*2);";
  html += "var y=H-pad-(d[i][1]-vmin)/(vmax-vmin)*(H-pad*2);";
  html += "pts+=x+','+y+' '}";
  // Monta SVG
  html += "var s='<svg width=\"'+W+'\" height=\"'+H+'\" style=\"background:#111;border-radius:8px\">';";
  // Eixo Y labels
  html += "s+='<text x=\"4\" y=\"'+(pad-5)+'\" fill=\"#888\" font-size=\"10\">'+vmax.toFixed(1)+' kg</text>';";
  html += "s+='<text x=\"4\" y=\"'+(H-pad+15)+'\" fill=\"#888\" font-size=\"10\">'+vmin.toFixed(1)+' kg</text>';";
  // Eixo X labels
  html += "var dur=t1-t0;var lbl=dur<3600?(dur/60).toFixed(0)+'min':(dur/3600).toFixed(1)+'h';";
  html += "s+='<text x=\"'+(W-pad)+'\" y=\"'+(H-5)+'\" fill=\"#888\" font-size=\"10\" text-anchor=\"end\">'+lbl+'</text>';";
  html += "s+='<text x=\"'+pad+'\" y=\"'+(H-5)+'\" fill=\"#888\" font-size=\"10\">0</text>';";
  // Linha de dados
  html += "s+='<polyline points=\"'+pts+'\" fill=\"none\" stroke=\"#0f0\" stroke-width=\"2\"/>';";
  html += "s+='</svg>';";
  html += "g.innerHTML=s});";
  html += "}";
  html += "desenhar();";
  html += "setInterval(desenhar,10000);";
  // Atualiza valores via AJAX a cada 2s
  html += "function atualizar(){";
  html += "fetch('/api'+location.search).then(r=>r.json()).then(function(d){";
  html += "document.getElementById('glpKg').textContent=d.glpKg.toFixed(1)+' kg';";
  html += "document.getElementById('glpPct').textContent=d.glpPercentual.toFixed(0)+'%';";
  html += "var b=document.getElementById('barra');";
  html += "b.style.width=d.glpPercentual.toFixed(0)+'%';";
  html += "b.style.background=d.glpPercentual<15?'#f44':d.glpPercentual<40?'#fa0':'#0f0';";
  html += "document.getElementById('pesoKg').textContent=d.pesoKg.toFixed(3)+' kg';";
  html += "document.getElementById('consumo').textContent=d.consumoKgH.toFixed(3)+' kg/h';";
  html += "var a=document.getElementById('autonomia');";
  html += "a.textContent=d.autonomia;";
  html += "a.style.color=d.autonomiaHoras<0?'#0f0':d.autonomiaHoras<24?'#f44':d.autonomiaHoras<72?'#fa0':'#0f0';";
  html += "document.getElementById('amostras').textContent=d.numAmostras+' amostras registradas';";
  html += "document.getElementById('estado').textContent=d.estado;";
  html += "document.getElementById('uptime').textContent=d.uptime;";
  html += "}).catch(function(){});}";
  html += "atualizar();";
  html += "setInterval(atualizar,2000);";
  html += "</script>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleApi() {
  aplicarParametros();
  String json = "{";
  json += "\"estado\":\"" + nomeEstado(estadoAtual) + "\",";
  json += "\"pesoKg\":" + String(pesoKg, 3) + ",";
  json += "\"glpKg\":" + String(glpKg, 3) + ",";
  json += "\"glpPercentual\":" + String(glpPercentual, 1) + ",";
  json += "\"consumoKgH\":" + String(consumoKgH, 3) + ",";
  json += "\"autonomiaHoras\":" + String(autonomiaHoras, 1) + ",";
  json += "\"autonomia\":\"" + formatarAutonomia() + "\",";
  json += "\"numAmostras\":" + String(numAmostras) + ",";
  json += "\"rawHX\":" + String(rawHX, 0) + ",";
  json += "\"offsetEstimado\":" + String(offsetEstimado, 0) + ",";
  json += "\"cfgPesoVazio\":" + String(cfgPesoVazio, 3) + ",";
  json += "\"cfgPesoCheio\":" + String(cfgPesoCheio, 3) + ",";
  json += "\"cfgCapacidade\":" + String(cfgCapacidade, 3) + ",";
  json += "\"uptime\":\"" + tempoUptime() + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

String nomeEstado(Estado e) {
  switch (e) {
    case STARTUP:          return "STARTUP";
    case ESTABILIZACAO:    return "ESTABILIZACAO";
    case SEM_PESO:         return "SEM_PESO";
    case CAPTURA_DE_PESO:  return "CAPTURA_DE_PESO";
    case MONITORAMENTO:    return "MONITORAMENTO";
    default:               return "DESCONHECIDO";
  }
}

void mudarEstado(Estado novo) {
  Serial.println();
  Serial.println("══════════════════════════════════════");
  Serial.print("Estado: ");
  Serial.print(nomeEstado(estadoAtual));
  Serial.print(" -> ");
  Serial.println(nomeEstado(novo));
  Serial.println("══════════════════════════════════════");
  estadoAtual = novo;
  resetarJanela();
}

void imprimirLeitura() {
  unsigned long agora = millis();
  float segundos = (agora - tempoInicio) / 1000.0;

  Serial.print("[");
  Serial.print(segundos, 1);
  Serial.print("s] ");
  Serial.print(nomeEstado(estadoAtual));
  Serial.print(" | RAW=");
  Serial.print(rawHX, 0);
  Serial.print(" | Offset=");
  Serial.print(offsetEstimado, 0);
  Serial.print(" | PesoRAW=");
  Serial.print(pesoEstimado, 0);
  Serial.print(" | Peso=");
  Serial.print(pesoKg, 3);
  Serial.print(" kg");

  if (estadoAtual == MONITORAMENTO || estadoAtual == CAPTURA_DE_PESO) {
    Serial.print(" | GLP=");
    Serial.print(glpKg, 3);
    Serial.print(" kg (");
    Serial.print(glpPercentual, 0);
    Serial.print("%)");
    if (estadoAtual == MONITORAMENTO && autonomiaHoras >= 0) {
      Serial.print(" | Auto=");
      Serial.print(formatarAutonomia());
    }
  }

  Serial.print(" | DP=");
  Serial.print(desvioJanela(), 1);

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(" | IP=");
    Serial.print(WiFi.localIP());
  }

  Serial.println();
}

// ─── Task HX711 (core 0) ──────────────────────────────────────────────────────
void tarefaHX711(void *param) {
  for (;;) {
    float val = lerMediaRAW();
    rawHX_pronto = val;
    novaLeitura = true;
    vTaskDelay(pdMS_TO_TICKS(INTERVALO_LEITURA_MS));
  }
}

// ─── Setup ───────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println();
  Serial.println("========================================");
  Serial.println("  Balanca Inteligente GLP — V3");
  Serial.println("  Fator calibracao: " + String(FATOR_RAW_POR_KG, 0) + " RAW/kg");
  Serial.println("  Botijao vazio: " + String(PESO_BOTIJAO_VAZIO_KG, 1) + " kg");
  Serial.println("  Capacidade GLP: " + String(CAPACIDADE_GLP_KG, 1) + " kg");
  Serial.println("========================================");
  Serial.println();

  scale.begin(HX711_DT, HX711_SCK);

  if (!scale.is_ready()) {
    Serial.println("ERRO: HX711 nao respondeu. Verifique conexoes.");
    while (!scale.is_ready()) {
      delay(500);
      Serial.print(".");
    }
    Serial.println(" OK!");
  }

  Serial.println("HX711 inicializado.");

  // Conectar Wi-Fi
  Serial.print("Conectando Wi-Fi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int tentativas = 0;
  while (WiFi.status() != WL_CONNECTED && tentativas < 40) {
    delay(500);
    Serial.print(".");
    tentativas++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" OK!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(" FALHOU (continua sem Wi-Fi)");
  }

  // Web server
  server.on("/", handleRoot);
  server.on("/api", handleApi);
  server.on("/api/historico", handleHistorico);
  server.begin();

  tempoInicio = millis();
  mudarEstado(ESTABILIZACAO);

  // Leitura HX711 no core 0 (loop/web server ficam no core 1)
  xTaskCreatePinnedToCore(tarefaHX711, "HX711", 4096, NULL, 1, NULL, 0);
}

// ─── Loop principal ──────────────────────────────────────────────────────────

void loop() {
  server.handleClient();

  if (!novaLeitura) return;
  novaLeitura = false;

  rawHX = rawHX_pronto;
  adicionarJanela(rawHX);

  switch (estadoAtual) {

    // ── ESTABILIZAÇÃO ──────────────────────────────────────────────────────
    // Aguarda leituras estáveis E sem tendência para definir offset inicial
    case ESTABILIZACAO: {
      float dp = desvioJanela();
      float tend = tendenciaJanela();

      if (janelaCheia && dp < DESVIO_MAX_ESTAVEL && abs(tend) < TENDENCIA_MAX) {
        janelasEstaveis++;
      } else if (dp > DESVIO_MAX_ESTAVEL * 2) {
        janelasEstaveis = 0;
      }
      // Se DP ok mas tendência alta: não incrementa, mas também não zera

      imprimirLeitura();
      if (janelaCheia) {
        Serial.print("  Tend=");
        Serial.print(tend, 1);
        if (janelasEstaveis > 0) {
          Serial.print("  >> Janela estavel (");
          Serial.print(janelasEstaveis);
          Serial.print("/");
          Serial.print(JANELAS_CONFIAVEIS);
          Serial.print(")");
        }
        Serial.println();
      }

      if (janelasEstaveis >= JANELAS_CONFIAVEIS) {
        offsetEstimado = mediaJanela();
        pesoEstimado = 0.0;
        pesoFiltrado = 0.0;
        pesoKg = 0.0;
        Serial.print("  >> Offset inicial definido: ");
        Serial.println(offsetEstimado, 1);
        mudarEstado(SEM_PESO);
      }
      break;
    }

    // ── SEM PESO ───────────────────────────────────────────────────────────
    // Plataforma vazia — aplica zero tracking para compensar deriva lenta
    case SEM_PESO: {
      pesoEstimado = rawHX - offsetEstimado;
      pesoFiltrado = pesoFiltrado + ALFA_FILTRO * (pesoEstimado - pesoFiltrado);
      pesoKg = pesoFiltrado / FATOR_RAW_POR_KG;

      calcularGLP();

      // Zero tracking com taxa adaptativa
      float absErro = abs(pesoEstimado);
      if (absErro < ZERO_TRACK_FAIXA_FINA) {
        // Ajuste fino: leitura próxima do zero
        offsetEstimado += pesoEstimado * ZERO_TRACK_TAXA_FINA;
      } else if (absErro < ZERO_TRACK_FAIXA_GROSSA) {
        // Ajuste grosso: recuperação pós-remoção de peso (histerese mecânica)
        offsetEstimado += pesoEstimado * ZERO_TRACK_TAXA_GROSSA;
      }
      // Se absErro >= FAIXA_GROSSA: não corrige (pode ser peso real chegando)

      imprimirLeitura();

      // Detecta colocação de peso (usa leitura instantânea, não filtrada)
      if (pesoEstimado > LIMIAR_PESO_PRESENTE) {
        Serial.println("  >> Peso detectado na plataforma!");
        pesoFiltrado = pesoEstimado; // inicializa filtro com leitura atual
        pesoKg = pesoFiltrado / FATOR_RAW_POR_KG;
        mudarEstado(CAPTURA_DE_PESO);
      }
      break;
    }

    // ── CAPTURA DE PESO ────────────────────────────────────────────────────
    // Peso recém-colocado — aguarda estabilização para confirmar
    case CAPTURA_DE_PESO: {
      pesoEstimado = rawHX - offsetEstimado;
      pesoFiltrado = pesoFiltrado + ALFA_FILTRO * (pesoEstimado - pesoFiltrado);
      pesoKg = pesoFiltrado / FATOR_RAW_POR_KG;

      calcularGLP();

      imprimirLeitura();

      // Peso foi removido antes de estabilizar
      if (pesoEstimado < LIMIAR_PESO_REMOVIDO) {
        Serial.println("  >> Peso removido antes de estabilizar.");
        pesoFiltrado = 0.0;
        pesoKg = 0.0;
        glpKg = 0.0;
        glpPercentual = 0.0;
        mudarEstado(SEM_PESO);
        break;
      }

      float dp = desvioJanela();
      float tend = tendenciaJanela();

      if (janelaCheia && dp < DESVIO_MAX_ESTAVEL && abs(tend) < TENDENCIA_MAX) {
        janelasEstaveis++;
      } else if (dp > DESVIO_MAX_ESTAVEL * 2) {
        janelasEstaveis = 0;
      }

      if (janelasEstaveis > 0) {
        Serial.print("  >> Janela estavel com peso (");
        Serial.print(janelasEstaveis);
        Serial.print("/");
        Serial.print(JANELAS_CONFIAVEIS);
        Serial.print(") Tend=");
        Serial.println(tend, 1);
      }

      if (janelasEstaveis >= JANELAS_CONFIAVEIS) {
        pesoFiltrado = mediaJanela() - offsetEstimado; // usa média da janela estável
        pesoKg = pesoFiltrado / FATOR_RAW_POR_KG;
        calcularGLP();
        Serial.print("  >> Peso capturado: ");
        Serial.print(pesoKg, 3);
        Serial.print(" kg | GLP: ");
        Serial.print(glpKg, 3);
        Serial.print(" kg (");
        Serial.print(glpPercentual, 0);
        Serial.println("%)");
        mudarEstado(MONITORAMENTO);
      }
      break;
    }

    // ── MONITORAMENTO ──────────────────────────────────────────────────────
    // Acompanhamento contínuo — detecta remoção do botijão
    case MONITORAMENTO: {
      pesoEstimado = rawHX - offsetEstimado;
      pesoFiltrado = pesoFiltrado + ALFA_FILTRO * (pesoEstimado - pesoFiltrado);
      pesoKg = pesoFiltrado / FATOR_RAW_POR_KG;
      calcularGLP();

      registrarAmostra();
      imprimirLeitura();

      // Detecta remoção do peso (queda brusca)
      if (pesoEstimado < LIMIAR_PESO_REMOVIDO) {
        Serial.println("  >> Peso removido da plataforma!");
        pesoFiltrado = 0.0;
        pesoKg = 0.0;
        glpKg = 0.0;
        glpPercentual = 0.0;
        mudarEstado(SEM_PESO);
      }
      break;
    }

    default:
      mudarEstado(ESTABILIZACAO);
      break;
  }

}
