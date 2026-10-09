# Robô Mini-Sumo 500g: Arquitetura do Firmware (STM32 Blue Pill)

MCU: STM32F103C8 (HAL, C). Sensores: 3x VL53L0X (I2C1), sensores de borda, 2 motores via ponte H com PWM.

## 1. Princípios

- Loop principal **não bloqueante** (sem `HAL_Delay`, apenas `HAL_GetTick`[Para não travar a execução do código]).
- Camadas com dependência em um único sentido: `app -> drivers -> HAL`.
- Estratégia (FSM) **não conhece hardware**: lê um `sensor_data_t` e escreve um `motor_cmd_t`.
- Todo driver tem teste próprio em `Testes/`.

```
+---------------------------------------------+
| app/    robot.c  strategy.c  (FSM)          |  decisão
+---------------------------------------------+
| mid/    sensors.c  drive.c                  |  fusão e abstração
+---------------------------------------------+
| drv/    vl53x.c  edge.c  motor.c  button.c  |  drivers
+---------------------------------------------+
| HAL (I2C, TIM PWM, GPIO, SysTick)           |
+---------------------------------------------+
```

## 2. Árvore de arquivos

```
Core/
  Inc/
    config.h
    types.h
    drv/   vl53x.h  edge.h  motor.h  button.h
    mid/   sensors.h  drive.h
    app/   strategy.h  robot.h
  Src/
    main.c
    drv/   vl53x.c  edge.c  motor.c  button.c
    mid/   sensors.c  drive.c
    app/   strategy.c  robot.c
testes/
```

## 3. O que vai em cada arquivo

### `config.h`
Arquivos de configuração.
- Endereços I2C: `0x30`, `0x5A`, `0x64`.
- Limiares: `ATTACK_DIST_MM`, `DETECT_DIST_MM` (~600), `MAX_RANGE_MM` (1000), `EDGE_THRESHOLD`.
- Velocidades (0 a 100%): `SPD_SEARCH`, `SPD_ATTACK`, `SPD_TURN`, `SPD_REVERSE`.
- Tempos (ms): `T_START_DELAY` (5000), `T_REVERSE`, `T_TURN`, `T_LOST_TIMEOUT`, `T_SENSOR_STALE`.
- Período do loop de sensores.

### `types.h`
```c
typedef enum { SIDE_LEFT, SIDE_CENTER, SIDE_RIGHT, SIDE_COUNT } side_t;

typedef struct {
    uint16_t dist_mm[SIDE_COUNT];   // 0xFFFF = sem leitura válida
    bool     valid[SIDE_COUNT];
    uint32_t stamp_ms[SIDE_COUNT];  // instante da última leitura boa
} range_data_t;

typedef struct {
    bool front_left, front_right;   // estrutura para guardar os estados dos sensores de borda
} edge_data_t;

// Estrutura que combina leituras de distância e borda
typedef struct {
    range_data_t range;
    edge_data_t  edge;
} sensor_data_t;

// Estrutura de comando para os motores
typedef struct { int8_t left, right; } motor_cmd_t; 
```

### `drv/vl53x.h/.c` (driver do ToF)
- Arquivo onde teremos a configuração e leitura dos sensores VL53L0X via I2C.

### `drv/edge.h/.c` (sensores de borda)
- Arquivo onde teremos a configuração e leitura dos sensores de borda (digital).

### `drv/motor.h/.c` (PWM + ponte H)
- Arquivo onde teremos a configuração e controle dos motores via PWM e ponte H.

### `drv/button.h/.c`
- Arquivo onde teremos a configuração e leitura do botão de start.

### `mid/sensors.h/.c` (fusão)
- `sensors_update(sensor_data_t *)`: coleta ToF + borda e preenche a struct.
- Filtro: Devemos tratar o sinal que recebemos do sensor para evitar leituras inválidas ou muito antigas.
- Marca `valid=false` se a leitura estiver mais velha que `T_SENSOR_STALE` ou acima de `MAX_RANGE_MM`.
- Helpers: `sensors_target_visible()`, `sensors_target_bearing()` (retorna `-1(Esquerda)`, `0(Centro)`, `+1(Direita)` ou `NONE` dependendo da posição do alvo).

### `mid/drive.h/.c` (movimentos)
Traduz intenção em `motor_cmd_t`:
- `drive_forward(spd)`, `drive_reverse(spd)`
- `drive_spin_left(spd)`, `drive_spin_right(spd)`
- `drive_arc(spd, bias)` para perseguir alvo com correção suave
- `drive_stop()`
- `drive_apply(motor_cmd_t)` chama `motor_set`.

### `app/strategy.h/.c` (FSM)
- `strategy_init()`
- `motor_cmd_t strategy_step(const sensor_data_t *in, uint32_t now_ms)`

### `app/robot.h/.c`
- `robot_init()`: chama todos os `*_init`.
- `robot_loop()`: um ciclo de `sensors_update -> strategy_step -> drive_apply`.
- Gerencia estado global: `WAIT_START`, `COUNTDOWN`, `RUNNING`, `STOPPED`.

### `main.c`
Só configuração de clock/HAL gerada pelo CubeMX, `robot_init()` e `while(1){ robot_loop(); }`.

## 4. Ciclo principal

```c
void robot_loop(void) {
    static sensor_data_t s;
    uint32_t now = HAL_GetTick();

    vl53x_task();                  // recuperação de sensores
    sensors_update(&s);            // ~ a cada 20 a 30 ms (ritmo do ToF)
    motor_cmd_t cmd = strategy_step(&s, now);
    drive_apply(cmd);
}
```

Borda tem prioridade sobre qualquer outra coisa. Se o ToF ficar lento, a leitura de borda deve rodar em ciclo mais rápido que a de distância (interrupções).

## 5. Estratégia

### 5.1 Estados

| Estado | Ação | Sai para |
|---|---|---|
| `WAIT_START` | Motores parados, aguarda botão/start module | `COUNTDOWN` |
| `COUNTDOWN` | Aguarda 5 s (regra), parado | `OPENING` |
| `OPENING` | Manobra inicial curta (ver 5.3) | `SEARCH` ou `ATTACK` |
| `SEARCH` | Gira procurando, ou avança em arco | `ATTACK` se vê alvo |
| `ATTACK` | Avança em direção ao alvo, vel. máxima | `SEARCH` se perde alvo |
| `LINE_ESCAPE` | Recua e gira para dentro da arena | `SEARCH` |
| `LOST` | Alvo sumiu há pouco, continua na última direção | `SEARCH` após timeout |

### 5.2 Prioridades (avaliadas nesta ordem, todo ciclo)

1. **Borda detectada** -> `LINE_ESCAPE` (sobrepõe tudo, inclusive `ATTACK`).
2. Alvo visível -> `ATTACK`.
3. Alvo perdido recentemente -> `LOST`.
4. Caso contrário -> `SEARCH`.

### 5.3 Abertura (`OPENING`)
Escolher uma opção antes da partida:
- **Frontal direto**: avançar reto por ~300 ms e cair em `SEARCH`.
- **Giro lateral**: girar ~90 graus e avançar, para flanquear. (Empurrar o oponente de lado)
- **Espera**: ficar parado e só reagir (útil contra robôs agressivos que se jogam para fora).

### 5.4 `SEARCH`
- Giro no lugar alternando o sentido (ex.: 600 ms para um lado, depois para o outro), evita ficar previsível.
- Variante: avançar em arco largo para cobrir a arena em vez de ficar só no centro.
- Se o último alvo foi visto à esquerda/direita, começa girando para esse lado.

### 5.5 `ATTACK`
Usa os 3 sensores:
- Só central vê: `drive_forward(SPD_ATTACK)`.
- Esquerdo vê (central não): arco para a esquerda (`bias` negativo).
- Direito vê (central não): arco para a direita.
- Central + lateral: avançar reto, corrigindo levemente para o lado do lateral.
- `dist < ATTACK_DIST_MM` (ex.: 150 mm): potência máxima, sem correção.
- Guarda `last_bearing` e `last_seen_ms` a cada leitura válida.

### 5.6 `LINE_ESCAPE`
Sub-máquina curta, tudo por tempo (`HAL_GetTick`):
1. Para.
2. Ré por `T_REVERSE` (se a borda é frontal) ou avanço (se traseira).
3. Giro de `T_TURN` para o lado oposto à borda detectada (esquerda viu: gira para a direita).
4. Retorna a `SEARCH`.

Se as duas bordas frontais dispararem, recuar e girar 180 graus.

### 5.7 `LOST`
- Mantém giro no sentido de `last_bearing` por até `T_LOST_TIMEOUT` (~300 ms).
- Se reencontrar, volta a `ATTACK`; senão `SEARCH`.

### 5.8 Casos especiais

- **Empurrão sem enxergar** (ToF cego colado no oponente): se estava em `ATTACK` e as leituras viram inválidas por menos de ~200 ms, **continuar empurrando** em vez de ir para `LOST`.
- **Sensor em falha** (`FAULT`): a estratégia ignora aquele lado (`valid=false`) e continua com os demais.
- **Anti-travamento**: se ficar em `ATTACK` por mais de X s sem alvo mudar de distância, fazer uma manobra lateral curta.
