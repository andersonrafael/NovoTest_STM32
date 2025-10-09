/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "cJSON.h"
#include "usart.h"
#include "gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for ProducerTask */
osThreadId_t ProducerTaskHandle;
const osThreadAttr_t ProducerTask_attributes = {
  .name = "ProducerTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ConsumerTask */
osThreadId_t ConsumerTaskHandle;
const osThreadAttr_t ConsumerTask_attributes = {
  .name = "ConsumerTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sensorDataQueue */
osMessageQueueId_t sensorDataQueueHandle;
const osMessageQueueAttr_t sensorDataQueue_attributes = {
  .name = "sensorDataQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// As funções da lógica do ADC são externas (estão em main.c)
extern uint16_t get_filtered_reading(void);
extern float raw_to_voltage_mV(uint16_t raw_value);
extern float voltage_to_current_mA(float voltage_mv);
/* USER CODE END FunctionPrototypes */

void StartProducerTask(void *argument);
void StartConsumerTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of sensorDataQueue */
  sensorDataQueueHandle = osMessageQueueNew (10, sizeof(SensorData_t), &sensorDataQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ProducerTask */
  ProducerTaskHandle = osThreadNew(StartProducerTask, NULL, &ProducerTask_attributes);

  /* creation of ConsumerTask */
  ConsumerTaskHandle = osThreadNew(StartConsumerTask, NULL, &ConsumerTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartProducerTask */
/**
  * @brief  Função da Tarefa Produtora (Medição).
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartProducerTask */
void StartProducerTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  SensorData_t data_to_send;

  // Loop infinito da tarefa
  for(;;)
  {
    // 1. Obter a leitura filtrada do ADC
    data_to_send.raw_adc = get_filtered_reading();

    // 2. Converter o valor bruto para tensão e depois para corrente
    data_to_send.voltage_mv = raw_to_voltage_mV(data_to_send.raw_adc);
    data_to_send.current_ma = voltage_to_current_mA(data_to_send.voltage_mv);

    // 3. Enviar a estrutura de dados completa para a fila
    osMessageQueuePut(sensorDataQueueHandle, &data_to_send, 0U, osWaitForever);

    // 4. Pausar a tarefa. A frequência de atualização será este delay
    // somado ao tempo total da medição (NUM_SAMPLES * 10ms).
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartConsumerTask */
/**
* @brief Função da Tarefa Consumidora (Formatação e Envio).
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartConsumerTask */
void StartConsumerTask(void *argument)
{
  /* USER CODE BEGIN StartConsumerTask */
  SensorData_t received_data;
  osStatus_t status;

  // Loop infinito da tarefa
  for(;;)
  {
    // 1. Esperar para receber um item da fila (a tarefa dorme aqui, economizando CPU)
    status = osMessageQueueGet(sensorDataQueueHandle, &received_data, NULL, osWaitForever);

    if (status == osOK)
    {
      // Pisca o LED azul (LD3) para indicar que está a processar e a enviar
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

      // 2. Calcular a percentagem (lógica de apresentação)
      float percentage = ((received_data.current_ma - MIN_CURRENT_MA) / (MAX_CURRENT_MA - MIN_CURRENT_MA)) * 100.0f;
      if (percentage < 0.0f) percentage = 0.0f;
      if (percentage > 100.0f) percentage = 100.0f;

      // 3. Criar o objeto JSON
      cJSON *root = cJSON_CreateObject();
      if (root != NULL)
      {
        cJSON_AddNumberToObject(root, "raw_adc", received_data.raw_adc);
        cJSON_AddNumberToObject(root, "voltage_mV", received_data.voltage_mv);
        cJSON_AddNumberToObject(root, "current_mA", received_data.current_ma);
        cJSON_AddNumberToObject(root, "percentage", percentage);

        // 4. Converter o objeto JSON para uma string
        char *json_string = cJSON_Print(root);
        if (json_string != NULL)
        {
          // 5. Enviar a string JSON pela UART3 usando printf
          printf("%s\r\n", json_string);

          // 6. LIBERAR A MEMÓRIA ALOCADA! (Muito importante)
          free(json_string);
        }

        // 7. Liberar a memória do objeto JSON
        cJSON_Delete(root);
      }
    }
  }
  /* USER CODE END StartConsumerTask */
}
