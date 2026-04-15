/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    double latitude;
    double longitude;
    float  speed_kmh;
    int    satellites;
    int    fix_valid;
} GNSS_Data_t;

GNSS_Data_t gnss_data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NMEA_MAX_LEN 128

#define IMU_ADDR (0x6B << 1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint8_t rx_byte_gps; // Il singolo byte che riceveremo


uint8_t g_gnss_byte; // Singolo byte ricevuto
char nmea_buffer[NMEA_MAX_LEN];
uint8_t idx = 0;
int debug = 0;

uint8_t data[6];
int16_t x, y, z = 90;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float convert_to_decimal(char *coord, char dir)
{
    float val = atof(coord);

    int degrees = (int)(val / 100);
    float minutes = val - (degrees * 100);

    float decimal = degrees + (minutes / 60.0);

    if (dir == 'S' || dir == 'W')
        decimal = -decimal;

    return decimal;
}
double latitude = 0.0;
double longitude = 0.0;
float  speed_kmh = 0.0;
float  altitude = 0.0;
int    satellites = 0;

// Funzione interna per estrarre i campi senza rompere nulla
static const char* get_field(const char* p, int field_num) {
    for (int i = 0; i < field_num && p; i++) {
        p = strchr(p, ',');
        if (p) p++;
    }
    return p;
}

void parse_nmea(char *sentence) {
    if (sentence[0] != '$') return;

    // --- PARSING GGA (Satelliti) ---
    if (strstr(sentence, "GGA")) {
        int comma = 0;
        for (char *p = sentence; *p != '\0'; p++) {
            if (*p == ',') {
                comma++;
                char *data = p + 1;

                if (comma == 7) { // Numero satelliti
                    satellites = (int)strtol(data, NULL, 10);
                }
                if (comma == 9) { // Altitudine (Metri sul livello del mare)
                    if (*data != ',') {
                        altitude = strtof(data, NULL);
                    }
                    break; // Possiamo uscire dal ciclo
                }
            }
        }
    }

    // --- PARSING RMC (Lat, Lon, Speed) ---
    if (strstr(sentence, "RMC")) {
        int comma = 0;
        for (char *p = sentence; *p != '\0'; p++) {
            if (*p == ',') {
                comma++;
                char *data = p + 1;
                switch (comma) {
                    case 2: if (*data == 'V') return; break;
                    case 3: if (*data != ',') {
                                double raw = strtod(data, NULL);
                                int deg = (int)(raw / 100);
                                latitude = deg + (raw - deg * 100) / 60.0;
                            } break;
                    case 4: if (*data == 'S') latitude = -latitude; break;
                    case 5: if (*data != ',') {
                                double raw = strtod(data, NULL);
                                int deg = (int)(raw / 100);
                                longitude = deg + (raw - deg * 100) / 60.0;
                            } break;
                    case 6: if (*data == 'W') longitude = -longitude; break;
                    case 7: if (*data != ',') speed_kmh = (float)strtod(data, NULL) * 1.85f; break;
                }
            }
        }
    }
    // NOTA: Ho tolto tutte le sprintf e HAL_UART_Transmit da qui!
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	HAL_Init();
	  SystemClock_Config();

	  // 1. PRIMA di MX_GPIO_Init e MX_USART6_Init
	  // Forza PG14 in INPUT senza pull-up (Alta Impedenza)
	  __HAL_RCC_GPIOG_CLK_ENABLE();
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	  GPIO_InitStruct.Pin = GPIO_PIN_14;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	  // 2. ASPETTA. Dai al GNSS il tempo di finire il boot elettrico
	  HAL_Delay(3000);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  uint8_t who = 0;

  HAL_I2C_Mem_Read(&hi2c1,
                   IMU_ADDR,
                   0x0F,
                   I2C_MEMADD_SIZE_8BIT,
                   &who,
                   1,
                   100);

  char msg[50];
  sprintf(msg, "WHO_AM_I = 0x%02X\r\n", who);
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
  uint8_t rst = 0x01;
  HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x12, 1, &rst, 1, 100);
  HAL_Delay(50);
  if (HAL_I2C_IsDeviceReady(&hi2c1, IMU_ADDR, 5, 100) == HAL_OK)
  {
      uint8_t data2;

      // Accel ON
      data2 = 0x40;
      HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x10, I2C_MEMADD_SIZE_8BIT, &data2, 1, 100);

      // Gyro ON
      data2 = 0x40;
      HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x11, I2C_MEMADD_SIZE_8BIT, &data2, 1, 100);

      HAL_Delay(50);

      HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
  }


  __HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);
  HAL_UART_Receive(&huart2, &g_gnss_byte, 1, 10); // Leggi un byte a vuoto per sbloccare
  HAL_UART_Receive_IT(&huart6, &g_gnss_byte, 1);  // Ora abilita davvero
  HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
  char *test_msg = "\r\n--- STM32 F767ZI Ready ---\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t*)test_msg, strlen(test_msg), 100);
  // Avvia la ricezione a interrupt (per 1 solo byte)
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET); // Impulso di sveglia
  HAL_Delay(100);
  for(int i = 0; i<10; i++){
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET); // Impulso di sveglia
	  HAL_Delay(100);
  }
  HAL_Delay(500); // Aspetta che il GNSS digerisca lo stimolo
  HAL_UART_Receive_IT(&huart6, &rx_byte_gps, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(satellites > 0){
		  	  char msg[120];
		  	if (HAL_I2C_IsDeviceReady(&hi2c1, IMU_ADDR, 5, 100) == HAL_OK)
		  	{
		  	    uint8_t data_acc[6];
		  	    uint8_t data_gyro[6];
		  	    char msg[120];

		  	    // ======================
		  	    // ACCELEROMETRO (0x28)
		  	    // ======================
		  	    HAL_I2C_Mem_Read(&hi2c1,
		  	                     IMU_ADDR,
		  	                     0x28 | 0x80,
		  	                     I2C_MEMADD_SIZE_8BIT,
		  	                     data_acc,
		  	                     6,
		  	                     100);

		  	    int16_t ax = (int16_t)(data_acc[0] | (data_acc[1] << 8));
		  	    int16_t ay = (int16_t)(data_acc[2] | (data_acc[3] << 8));
		  	    int16_t az = (int16_t)(data_acc[4] | (data_acc[5] << 8));

		  	    // ======================
		  	    // GIROSCOPIO (0x22)
		  	    // ======================
		  	    HAL_I2C_Mem_Read(&hi2c1,
		  	                     IMU_ADDR,
		  	                     0x22 | 0x80,
		  	                     I2C_MEMADD_SIZE_8BIT,
		  	                     data_gyro,
		  	                     6,
		  	                     100);

		  	    int16_t gx = (int16_t)(data_gyro[0] | (data_gyro[1] << 8));
		  	    int16_t gy = (int16_t)(data_gyro[2] | (data_gyro[3] << 8));
		  	    int16_t gz = (int16_t)(data_gyro[4] | (data_gyro[5] << 8));

		  	    // ======================
		  	    // OUTPUT UNICO
		  	    // ======================
		  	    if(debug){
		  	    sprintf(msg,
		  	            "ACC:%d,%d,%d | GYRO:%d,%d,%d\r\n",
		  	            ax, ay, az, gx, gy, gz);
		  	    } else{
			  	    sprintf(msg,
			  	            "%d,%d,%d,%d,%d,%d\r\n",
			  	            ax, ay, az, gx, gy, gz);
		  	    }
		  	    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

		  	    HAL_Delay(100);


		  	}

		  	else{
				  char *test_msg = "\r\n--- STA ANDANDO TUTTO MALE ---\r\n";
				  HAL_UART_Transmit(&huart3, (uint8_t*)test_msg, strlen(test_msg), 100);
		  	}

	      int lat_i = (int)latitude;
	      int lat_d = (long)fabs((latitude - lat_i) * 1000000);
	      int lon_i = (int)longitude;
	      int lon_d = (long)fabs((longitude - lon_i) * 1000000);
	      int speed_int = (int)speed_kmh;
	      int speed_dec = (int)((speed_kmh - speed_int) * 10); // due decimali
	      int alt_int = (int)altitude;
	      int alt_dec = (int)(altitude - alt_int * 1);
	      if(debug == 1){
	      sprintf(msg, "LAT:%d.%06d LON:%d.%06d ALT:%d.%d SPD:%d.%d km/h SAT:%d\r\n\n",
	              lat_i, lat_d, lon_i, lon_d, alt_int, alt_dec, speed_int, speed_dec, satellites);
	      }
	      else{
	    	  sprintf(msg, "%d.%06d,%d.%06d,%d.%d,%d.2%d,%d\r\n",
	    	  	              lat_i, lat_d, lon_i, lon_d, alt_int, alt_dec, speed_int, speed_dec, satellites);
	    	  	      }

	      HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

	      HAL_Delay(500); // Questo delay nel main NON blocca l'interrupt!


	  }
	  else{
		  char *test_msg = "\r\n--- PROBLEMA, STACCA E ATTACCA IL TX PD5 ---\r\n";
		  HAL_UART_Transmit(&huart3, (uint8_t*)test_msg, strlen(test_msg), 100);
		  HAL_Delay(500);

	  }
//	  uint8_t buffer;
//	      HAL_StatusTypeDef status;
//	      uint32_t err;
//	      char *test_msg = "\r\n PROVA \r\n";
//	      HAL_UART_Transmit(&huart4, (uint8_t*)test_msg, strlen(test_msg), 100);
//	      	 //HAL_Delay(1000);
//	      // 1. Leggiamo un byte con un timeout di 1 secondo
//	      status = HAL_UART_Receive(&huart4, &buffer, 1, 1000);
//
//	      // 2. Analizziamo il risultato della ricezione
//	      if (1)
//	      {
//	          // RICEZIONE CORRETTA: Lampeggio blu e invio al PC
//	          HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
//	          HAL_UART_Transmit(&huart3, &buffer, 1, 10);
//	      }
//	      else if (status == HAL_TIMEOUT)
//	      {
//	          // TIMEOUT: Nessun dato arrivato in 1 secondo
//	          char *msg = "Status: TIMEOUT (Nessun segnale su RX)\r\n";
//	          HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
//	          HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET); // Spegne rosso se era acceso
//	      }
//	      else if (status == HAL_ERROR)
//	      {
//	          // ERRORE HARDWARE: Qualcosa è andato storto elettricamente
//	          err = HAL_UART_GetError(&huart4);
//	          char err_msg[64];
//
//	          if (err & HAL_UART_ERROR_ORE)
//	              sprintf(err_msg, "Errore: OVERRUN (Dati troppo veloci)\r\n");
//	          else if (err & HAL_UART_ERROR_NE)
//	              sprintf(err_msg, "Errore: NOISE (Disturbo elettrico)\r\n");
//	          else if (err & HAL_UART_ERROR_FE)
//	              sprintf(err_msg, "Errore: FRAMING (Baud rate sbagliato?)\r\n");
//	          else
//	              sprintf(err_msg, "Errore: Codice %lu\r\n", err);
//
//	          HAL_UART_Transmit(&huart3, (uint8_t*)err_msg, strlen(err_msg), 100);
//	          HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET); // Accendi LED Rosso (LD3)
//
//	          // Reset della UART per riprovare
//	          HAL_UART_AbortReceive(&huart4);
//	          __HAL_UART_CLEAR_IT(&huart6, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);
//	      }
//	      else if (status == HAL_BUSY)
//	      {
//	          // BUSY: La periferica è bloccata in un'altra operazione
//	          char *msg = "Status: BUSY\r\n";
//	          HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
//	      }
//	  uint8_t test_byte = 'X';
//	  uint8_t rx_byte = 0;
//
//	  // Send 'X'
//	  HAL_UART_Transmit(&huart4, &test_byte, 1, 10);
//
//	  // Try to catch it immediately (no delay!)
//	  if (HAL_UART_Receive(&huart4, &rx_byte, 1, 10) == HAL_OK) {
//	      if (rx_byte == 'X') {
//	          HAL_GPIO_TogglePin(GPIOB, LD2_Pin); // BLINK BLUE = BOARD IS FINE
//	          char *test_msg = "\r\n--- FUNZIONA ---\r\n";
//	          HAL_UART_Transmit(&huart3, (uint8_t*)test_msg , strlen(test_msg), 100);
//	      }
//	  }
//	  HAL_Delay(100);


//		  if (HAL_UART_Receive(&huart2, &c, 1, 100) == HAL_OK)
//		      {
//		          // 1. SINCRONIZZAZIONE: Se arriva $, ricomincia da zero!
//		          if (c == '$')
//		          {
//		              idx = 0;
//		          }
//
//		          if (c == '\n')
//		          {
//		              nmea_buffer[idx] = '\0';
//		              // Non resettare idx qui, o meglio, l'if('$') lo farà per la prossima
//		              char *test_msg = "\r\n--- SONO QUI ---\r\n";
//		              HAL_UART_Transmit(&huart3, (uint8_t*)test_msg , strlen(test_msg), 100);
//		              //parse_nmea(nmea_buffer);
//		          }
//		          else
//		          {
//		              if (idx < NMEA_MAX_LEN - 1)
//		              {
//		                  nmea_buffer[idx++] = c;
//		              }
//		          }
//
//		          // Mantieni pure questo per vedere i dati grezzi sul PC
//		          HAL_UART_Transmit(&huart3, &c, 1, 100);
//		      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Questa funzione viene chiamata AUTOMATICAMENTE ogni volta che arriva un byte
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) { // Sostituisci con la tua istanza (es. USART2)
    	//HAL_UART_Transmit(&huart3, &g_gnss_byte	, 1, 100);
        // 1. Sincronizzazione immediata
        if (g_gnss_byte == '$') {
            idx = 0;
        }

        if (idx < 127) {
            nmea_buffer[idx++] = g_gnss_byte;

            // 2. Fine stringa: chiamiamo il parser
            if (g_gnss_byte == '\n') {
                nmea_buffer[idx] = '\0';
                parse_nmea(nmea_buffer);
                idx = 0;
            }
        } else {
            idx = 0; // Protezione overflow
        }

        // 3. RIATTIVA l'interrupt per il prossimo byte
        HAL_UART_Receive_IT(&huart6, &g_gnss_byte, 1);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
