# SENSORES VL53L0X — Mini-Sumo (500g)

Documentação do driver de leitura e reconexão automática dos 3 sensores VL53L0X usados no robô, compartilhando o mesmo barramento I2C.

## 🖥 Hardware utilizado

- STM32 BluePill (STM32F103)
- 3x sensor VL53L0X (módulos prontos, pull-up já presente no barramento)

## 📏 Especificações do sensor

- Faixa de medição: 30mm ~ 8190mm
- Faixa máxima de medição ideal: 1200mm
- Alcance necessário no projeto: ~1000mm (suficiente para a arena)
- Pino XSHUT desliga o sensor se estiver em `LOW`
- `8190mm` não é uma medida real — é o valor retornado quando a leitura é inválida/fora de alcance. Sempre validar via `RangingData.RangeStatus == 0` antes de usar o valor lido.

## 📭 Endereçamento I2C

Todo VL53L0X sempre inicializa (boot) no endereço padrão, independente de qual endereço tinha antes de um reset:

- 7-bit: `0x29`
- 8-bit (HAL/API): `0x52`

Como três sensores não podem responder no mesmo endereço simultaneamente, cada um é ligado individualmente via XSHUT, checado em `0x52`, e reatribuído para um endereço fixo antes do próximo ser ligado:

| Sensor | Endereço final (8-bit) | Pino XSHUT |
|---|---|---|
| Sensor 1 | `0x30` | `Lidar_xShutdown_Pin` |
| Sensor 2 | `0x5A` | `PB4` |
| Sensor 3 | `0x64` | `PB5` |

## ⚙ Configuração I2C

- I2C Speed Mode: `Standard`
- Clock: `100 kHz`
- Modo de endereçamento: `7-bit`

## 🎯 Filtro de leituras

- Leituras só são usadas se `RangingData.RangeStatus == 0`.
- `SIGNAL_RATE_FINAL_RANGE` ajustado para `0.25` (padrão é `0.1`), reduzindo ruído em leituras de sinal fraco — aceitável dado o alcance curto necessário no projeto (~1m).

---

## 🔁 Rotina de reconexão automática — visão geral

Cada sensor é representado por um struct `LidarSensor_t`, com estado (`LIDAR_OK`, `LIDAR_FAULT`, `LIDAR_RECOVERING`), contadores de falha e de tentativas de reconexão.

Fluxo:

1. Falhas de leitura consecutivas (`LIDAR_FAIL_THRESHOLD`) marcam o sensor como `FAULT`.
2. Sensor em `FAULT` é isolado do barramento (XSHUT baixo) e entra em uma fila de espera.
3. Um dispatcher central (`RecoveryDispatcher`) processa **um sensor por vez** da fila, evitando que dois sensores fiquem simultaneamente no endereço `0x52` (o que causaria colisão de endereço no barramento).
4. O sensor sendo atendido passa por: reset via XSHUT → boot → checagem em `0x52` → reatribuição de endereço → reinicialização completa do driver.
5. Toda a máquina de estados usa `HAL_GetTick()` (não `HAL_Delay`), garantindo que o loop principal e os outros sensores continuem funcionando normalmente durante a reconexão.
6. Se um sensor não reconectar após várias tentativas (`recover_attempts >= 10`), ele cede a vez e volta ao fim da fila, evitando travar a reconexão dos demais.

---

## 📚 Estruturas de dados

### `LidarState_t`
Enum com os três estados possíveis de um sensor: `LIDAR_OK`, `LIDAR_FAULT`, `LIDAR_RECOVERING`. Evita usar múltiplas variáveis booleanas soltas pra representar a mesma condição.

### `LidarSensor_t`
Agrupa tudo que é necessário pra gerenciar um sensor individualmente, permitindo reaproveitar a mesma lógica de recovery para os três sem duplicar código.

| Campo | Função |
|---|---|
| `dev` | Ponteiro pro driver VL53L0X daquele sensor |
| `xshut_port` / `xshut_pin` | Pino de controle do reset físico (XSHUT) |
| `target_addr` | Endereço I2C final atribuído ao sensor |
| `state` | Estado atual (`LidarState_t`) |
| `fail_count` | Falhas de leitura consecutivas |
| `recover_tick` | Timestamp do último passo do recovery (não bloqueante) |
| `recover_step` | Passo atual da sequência de reconexão (0, 1 ou 2) |
| `recover_attempts` | Tentativas sem sucesso no ciclo de recovery atual |
| `in_queue` | Evita que o sensor entre duplicado na fila de espera |

### Fila de recovery (`recovery_queue`, `queue_head`, `queue_tail`, `queue_count`)
Fila circular de ponteiros para `LidarSensor_t`. Serializa a reconexão: só um sensor pode estar ativo no endereço `0x52` por vez, então quem falhou primeiro é atendido primeiro.

---

## 🔧 Funções implementadas

### `void LidarInit(VL53L0X_DEV Dev)`
Configura os parâmetros de operação de um sensor (calibração de referência, modo de medição, filtros de sinal, timing budget). Chamada tanto na inicialização inicial quanto dentro do recovery, já que um reboot via XSHUT apaga toda a configuração do sensor.

### `static uint8_t LidarIsAlive(uint8_t i2c_addr_shifted)`
Faz um "ping" I2C (`HAL_I2C_IsDeviceReady`) pra checar se existe algum dispositivo respondendo naquele endereço, sem precisar ler dado nenhum. Usada antes de tentar reinicializar o driver de um sensor em recovery.

### `static void QueuePush(LidarSensor_t *s)` / `QueuePeek(void)` / `QueuePop(void)`
Operações básicas da fila circular de recovery: inserir um sensor no fim, consultar quem está na frente, e remover quem está na frente.

### `static void LidarRecoveryStep(LidarSensor_t *s)`
Máquina de estados que avança um passo por chamada, usando `HAL_GetTick()` em vez de `HAL_Delay` para não bloquear o loop principal:

- **Passo 0** — desliga o sensor via XSHUT.
- **Passo 1** — liga o sensor de novo (ele reboot em `0x52`).
- **Passo 2** — checa se respondeu em `0x52`; se sim, reatribui o endereço final e reinicializa o driver por completo (`LidarInit`); se não, incrementa `recover_attempts` e tenta de novo no próximo ciclo.

### `static void LidarUpdate(LidarSensor_t *s)`
Detecta que um sensor acabou de falhar (`LIDAR_FAULT`) e, se ainda não estiver na fila, isola ele do barramento (XSHUT baixo) e o adiciona à fila de espera. Não faz o trabalho de reconexão em si, apenas prepara o sensor pra aguardar sua vez.

### `static void RecoveryDispatcher(void)`
Chamada uma vez por iteração do loop principal. Processa apenas o sensor na frente da fila, chamando `LidarRecoveryStep` nele. Remove da fila quando o sensor volta a `LIDAR_OK`, ou o realoca pro fim da fila se ele exceder o limite de tentativas sem sucesso.

### `static void LidarReportStatus(LidarSensor_t *s, VL53L0X_Error status)`
Chamada depois de cada medição. Conta falhas consecutivas e só marca o sensor como `LIDAR_FAULT` após ultrapassar `LIDAR_FAIL_THRESHOLD`, evitando que um erro isolado de leitura dispare todo o processo de reconexão.

---

## ❓ Teste de comunicação

Leitura do registrador de identificação:

```c
ret = HAL_I2C_Mem_Read(
    &hi2c1,
    0x52,
    0xC0,
    I2C_MEMADD_SIZE_8BIT,
    &test,
    1,
    100
);
```
Resultado esperado: `0xEE (0x238)`

Medição de distância:

```c
status = VL53L0X_PerformSingleRangingMeasurement(Dev, &RangingData);
```
Resultado esperado: `VL53L0X_ERROR_NONE (0)`

Variável que recebe os valores de leitura: `RangingData` (`RangingData.RangeMilliMeter`, validado por `RangingData.RangeStatus == 0`).

## REFERÊNCIAS

- [Repositório da Biblioteca](https://github.com/MasthElectronics/STMInterfaceWithLidar)
- [Vídeo - Como usar o VL53L0X com STM32](https://www.youtube.com/watch?v=2MKgkL_v8MA&t=712s)