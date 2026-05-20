# TESTE SENSOR VL53L0X

## ❗ Funcionalidades

- Comunicação I2C utilizando HAL STM32
- Inicialização do VL53L0X
- Leitura de distância
- Controle do pino XSHUT
- Teste de comunicação via registrador `0xC0`

## 🖥 Hardware utilizado

- STM32 BluePill
- Sensor VL53L0X

## 📏 Especificações do sensor

- Faixa de medição: 30mm ~ 8190mm
- Faixa máxima de medição ideal: 1200mm
- Pino xShut desliga o sensor se estiver LOW

## 📭 Endereço I2C

Endereço padrão do sensor:

- 7-bit: `0x29`
- 8-bit (HAL/API): `0x52`

## ⚙ Configuração I2C

- I2C Speed Mode: `Standard`
- Clock: `100 kHz`
- Modo de endereçamento: `7-bit`

## ❓ Teste de comunicação e variáveis importantes

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

Resultado esperado
```c
0xEE (0x238)
```
---

```c
status = VL53L0X_PerformSingleRangingMeasurement(Dev, &RangingData);
```

Resultado esperado
```c
VL53L0X_ERROR_NONE (0)
```

---

Variável que recebe os valores de leitura
```c
RangingData
```



## REFERENCIAS  
[Repositorio da Biblioteca](https://github.com/MasthElectronics/STMInterfaceWithLidar)

[Video - Como usar o VL53L0X com STM32](https://www.youtube.com/watch?v=2MKgkL_v8MA&t=712s)