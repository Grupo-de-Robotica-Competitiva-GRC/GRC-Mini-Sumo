# Teste de PWM 
## Objetivo
Fazer um código para STM32F103C8T6 com intuito de gerar um PWM para o driver de motor ZK-BM1 que ira controlar a velocidade dos motores.

## Definição do prescaler do timer
Para esse teste a resolução do PWM escolhida foi de 10 bits e a frequência do PWM é de 1000Hz já que o driver vai até 2000Hz e a frequência da MCU é de 72Mhz. Falta então apenas definir o prescaler. Usando a formuala $$\large\frac{\large f_{Hz}}{(ARR + 1)(PSC + 1)}$$ 
fazendo manipulações temos:

$$(PSC + 1) = \frac{f_{Hz}}{(ARR + 1)x10^3}$$
$$PSC = \frac{f_{Hz}}{(ARR + 1)x10^3} - 1$$
$$PRC = \frac{72x10^6}{2^10*10^3} - 1$$
$$PSC = 69,31$$

Como o PSC só aceita números inteiros o valor de $\large PSC = 69$  

<img width="349" height="417" alt="image" src="https://github.com/user-attachments/assets/ac7cdc4f-dd1e-442c-938f-447cc60a8fd8" />

## Portas usadas
Foram usadas as portas PA8 e PA9, portas essas que são corespondentes ao canal 1 e 2 do timer 1 do MCU respectivamente
<img width="461" height="444" alt="image" src="https://github.com/user-attachments/assets/f20838fd-6b47-426f-96b1-b3df38052067" />

## Código
O código foi feito apenas para gerar dois pusos PWM de mesma frequência e ao longo do tempo ir aumentando seu duty-cicle, quando chegar ao máximo o valor do duty-cicle é zerado começando o ciclo novamente.
```c

  HAL_TIM_PWM_start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);


  uint16_t dc1 = 0;
  uint16_t dc2 = 0;
  while (1)
  {
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dc1);
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dc2);

	  HAL_Delay(300);
	  dc1 += 30;
	  dc1 += 30;
	  if (dc1 == 1024) dc1 = 0;
	  if (dc2 == 1024) dc2 = 0;

  }

```
Inicialmente é feito a ativação dos PWMs do timer escolhido. Em seguida é definido duas variáveis, `dc1`  e `dc2` que são as variáveis que armazenam o duty-cocle de cada canal respectivamente. Dentro do loop principal é uzado a macro `__HAL_TTIMM_SET_COMPARE()` que diz para o comparador do timer compare o valor do duty-cicle com o valor do `ARR` gerando assim o PWM.

Sendo assim para gerar um PWM de 50% de ciclo ativo no canal 1, basta usar os valoror de `dc1` igual a 512, que é a metade do valor do `ARR`.

PS: Atenção ao colar o código. Certifique-se de colocar nas áreas de código de usuario gerado pelo `STMCubeMX`.
