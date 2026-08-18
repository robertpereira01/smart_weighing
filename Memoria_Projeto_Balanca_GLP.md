# Memória do Projeto -- Balança Inteligente para Botijão de GLP

**Versão:** 1.0\
**Objetivo:** Consolidar o histórico técnico do projeto para permitir
sua continuidade sem perda de contexto.

------------------------------------------------------------------------

# Objetivo

Desenvolver uma balança baseada em **ESP32 + HX711 + 4 células de carga
de 50 kg** para monitorar continuamente o peso de um botijão de GLP,
estimando a quantidade de gás restante.

O sistema deverá permanecer operando continuamente, compensando deriva
mecânica e eletrônica.

------------------------------------------------------------------------

# Hardware

-   ESP32
-   HX711
-   4 células de carga de 50 kg (meia ponte)
-   Junction Board
-   Protoboard (fase inicial)

Pinos:

``` cpp
DT  -> GPIO19
SCK -> GPIO18
```

Alimentação:

-   HX711 em 3,3 V
-   ESP32 em 3,3 V

------------------------------------------------------------------------

# Ponte de Wheatstone

Ligação atualmente validada:

-   E+ → A vermelho
-   E− → B vermelho
-   A− → C vermelho
-   A+ → D vermelho

Interligações:

-   A preto + D preto
-   B preto + C preto
-   A branco + C branco
-   B branco + D branco

Essa ligação foi validada experimentalmente.

------------------------------------------------------------------------

# Descobertas

## Problema inicial

O HX711 apresentava:

-   E+ / E− = 0 V

Após refazer a conexão:

-   funcionamento normal.

------------------------------------------------------------------------

## Resposta das células

Foi confirmado:

-   A e B reduzem a leitura.
-   C e D aumentam a leitura.

------------------------------------------------------------------------

## Estabilidade

Depois da remontagem mecânica:

-   estabilidade muito superior;
-   deriva bastante reduzida.

------------------------------------------------------------------------

# Evolução do firmware

## V0

Leitura simples:

``` cpp
scale.read();
```

------------------------------------------------------------------------

## V1

Leitura média:

``` cpp
scale.read_average(10);
```

Adicionado timestamp.

------------------------------------------------------------------------

## V1.1

Estabilização por:

-   média
-   desvio padrão

------------------------------------------------------------------------

## V1.2

Índice de confiança

A balança somente prossegue após várias janelas estáveis.

------------------------------------------------------------------------

## V1.3

Tara automática após estabilização.

------------------------------------------------------------------------

## V2

Incluído:

-   zero tracking
-   máquina de estados
-   preparação para calibração

------------------------------------------------------------------------

# Resultados importantes

Após corrigidas as conexões:

Sem peso:

-   PesoRAW variando aproximadamente entre -40 e +30 RAW.

Foi considerado excelente.

------------------------------------------------------------------------

# Calibração

Foi utilizado inicialmente um peso de aproximadamente 1 kg.

Depois passou-se a utilizar um peso conhecido de:

514 g.

A calibração definitiva ainda não foi realizada.

------------------------------------------------------------------------

# Conclusão importante

Durante os testes observou-se:

A tara não deve ser tratada como fixa.

Quando o botijão permanece semanas sobre a plataforma:

-   deriva mecânica
-   deriva térmica
-   deriva eletrônica

acabam deslocando lentamente o zero.

------------------------------------------------------------------------

# Nova arquitetura aprovada

Abandonar o conceito tradicional de tara.

Substituir por:

-   offset estimado
-   peso estimado

Modelo:

RAW = Offset + Peso

O firmware deverá estimar continuamente o Offset.

------------------------------------------------------------------------

# Arquitetura prevista para a V3

Estados:

1.  STARTUP
2.  ESTABILIZAÇÃO
3.  SEM PESO
4.  CAPTURA DE PESO
5.  MONITORAMENTO

Variáveis principais:

-   rawHX
-   offsetEstimado
-   pesoEstimado
-   pesoFiltrado

------------------------------------------------------------------------

# Próximos passos

1.  Implementar V3 completa.
2.  Calibrar com peso conhecido.
3.  Converter RAW → kg.
4.  Converter kg → GLP restante.
5.  Implementar OLED.
6.  Implementar Wi-Fi.
7.  Histórico de consumo.
8.  Estimativa de autonomia.

------------------------------------------------------------------------

# Forma de trabalho definida

A partir da V3:

-   cada versão será entregue como um único arquivo `.ino`;
-   nunca mais trechos isolados;
-   alterações sempre sobre a última versão completa.
