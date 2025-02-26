/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//电机数据结构�?
typedef struct{
	int32_t raw_angle;
	int32_t total_angle;
	int16_t speed_rpm;
	int16_t torque_current;
	uint8_t temp;
	int32_t left_limit;
	int32_t right_limit;
}Motor_Status;
//工作状�?�枚�?
typedef enum {
	MODE_IDLE,
	MODE_PENDULUM,
	MODE_POSITION,
	MODE_INIT
}WorkMode;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX_CURRENT 700 //输入电调的最大电�?
#ifndef SIZE
#define SIZE
#define UART_BUF_SIZE 32
#endif

#define UART_CMD_MAX_LEN 16
//PID参数
#define PID_KP 10.0f
#define PID_KI 0.0f
#define PID_KD 6.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//全局变量
uint8_t MOTOR_ID=1;//电机ID

WorkMode work_mode=MODE_IDLE;
Motor_Status motor={0};

int32_t target_pos=0;
//uint8_t can_tx_data[8];

uint8_t uart_rx_buf[UART_BUF_SIZE];
uint8_t uart_cmd_buf[UART_CMD_MAX_LEN];
volatile uint8_t uart_cmd_ready = 0;


//PID结构�?
static arm_pid_instance_f32 pid;
static float32_t pid_output;
static float32_t pid_target;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//函数声明
void Update_Angle_Data(uint8_t* rx_data);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void Limit_Init();//左右端角度初始化
void Process_UART_Command(uint8_t* buf);
void Motor_Control_Task();
void PID_Init();
void PID_Control(int32_t target);
//void Send_Motor_Current(uint8_t id,int16_t current);
void CAN_Filter_Init(CAN_HandleTypeDef *hcan);
void cmd_motor(CAN_HandleTypeDef *hcan,uint32_t stdid,int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4);

//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t size);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//函数定义

//电机控制
void cmd_motor(CAN_HandleTypeDef *hcan,uint32_t stdid,int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4)
{
	CAN_TxHeaderTypeDef motor_tx_message;
    uint32_t send_mail_box;
	uint8_t can_tx_data[8];

	
    motor_tx_message.StdId=stdid;
    motor_tx_message.IDE=CAN_ID_STD;
    motor_tx_message.RTR=CAN_RTR_DATA;
    motor_tx_message.DLC=0x08;
    
    can_tx_data[0]=motor1>> 8;
    can_tx_data[1]=motor1;
    can_tx_data[2]=motor2 >>8;
    can_tx_data[3]=motor2;
    can_tx_data[4]=motor3>>8;
    can_tx_data[5]=motor3;
    can_tx_data[6]=motor4>>8;
    can_tx_data[7]=motor4;
	
    HAL_CAN_AddTxMessage(hcan, &motor_tx_message,can_tx_data,&send_mail_box);
}

//角度处理函数（CAN回调中调用）
void Update_Angle_Data(uint8_t* rx_data)
{
	static int32_t last_angle=0;
	int32_t new_angle=(rx_data[0]<<8)|rx_data[1];
	
	//处理突变
	int32_t delta=new_angle-last_angle;
	if(delta<-3000)delta+=8192;//正转过圈
	else if(delta>3000)delta-=8192;//逆转过圈
	
	motor.total_angle+=delta;
	motor.raw_angle=new_angle;
	last_angle=new_angle;
}

//CAN接收回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	
	HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data);
	
	//if(rx_header.StdId==0x200+MOTOR_ID){} 此处省略
	
	//数据更新
	Update_Angle_Data(rx_data);
	motor.speed_rpm=(rx_data[2]<<8)|rx_data[3];
	motor.torque_current=(rx_data[4]<<8)|rx_data[5];
	motor.temp=rx_data[6];
}

//左右端角度初始化
void Limit_Init()
{	
	HAL_TIM_Base_Stop_IT(&htim3);
	//左初始化
	int32_t last_angle=motor.total_angle;
	uint32_t last_tick=HAL_GetTick();
	while(1)
	{
		//向左发�?�最大电�?
		while(1)
		{
			cmd_motor(&hcan,0x200,-MAX_CURRENT,-MAX_CURRENT,-MAX_CURRENT,-MAX_CURRENT);
			HAL_Delay(1);
			if(HAL_GetTick()-last_tick>500)break;
		}
		last_tick=HAL_GetTick();
		int32_t delta=motor.total_angle-last_angle;
		if(abs(delta)<10)
		{
			motor.left_limit=0;
			motor.total_angle=0;
			break;
		}
		last_angle=motor.total_angle;
		HAL_Delay(10);
	}
	
	//右初始化
	while(1)
	{
		//向右发�?�最大电�?
		while(1)
		{
			cmd_motor(&hcan,0x200,MAX_CURRENT,MAX_CURRENT,MAX_CURRENT,MAX_CURRENT);
			HAL_Delay(1);
			if(HAL_GetTick()-last_tick>500)break;
		}
		last_tick=HAL_GetTick();
		int32_t delta=motor.total_angle-last_angle;
		if(abs(delta)<10)
		{
			motor.right_limit=motor.total_angle-10000;
			break;
		}
		last_angle=motor.total_angle;
		HAL_Delay(10);
	}
	
	cmd_motor(&hcan,0x200,0,0,0,0);//停止电机
	work_mode=MODE_IDLE;
	//HAL_TIM_Base_Start_IT(&htim3);
}

//UART指令处理（UART中断回调中调用）
void Process_UART_Command(uint8_t* buf)
{
	if(strstr((char*)buf,"MODE1"))
	{
		work_mode=MODE_PENDULUM;
	}else if(strstr((char*)buf,"STOP"))
	{
		work_mode=MODE_IDLE;
	}else if(strstr((char*)buf,"POS:"))
	{	
		float percent=atof((char*)(buf+4))/100.0f;
		target_pos=motor.left_limit+(int32_t)(percent*(motor.right_limit-motor.left_limit));
		work_mode=MODE_POSITION;
	}else if(strstr((char*)buf,"INIT"))
	{
		work_mode=MODE_INIT;
	}
}

//main中while循环
void Motor_Control_Task()
{
	//10tick执行�?�?
	static uint32_t last_tick=0;
	if(HAL_GetTick()-last_tick<10)return;
	last_tick=HAL_GetTick();
	
	switch(work_mode)
	{
		//钟摆 MODE1
		case MODE_PENDULUM:
		{
			static int8_t direction=-1;//1：向右；-1：向�?
			int32_t current_pos=motor.total_angle;
			//计算目标位置
			int32_t target=direction>0?(motor.right_limit-500):(motor.left_limit+500);
			PID_Control(target);
				
			//到达边界转向
			if(direction==-1&&current_pos<=motor.left_limit+500)direction=1;
			else if(direction==1&&current_pos>=motor.right_limit-500)direction=-1;
			last_tick=HAL_GetTick();	
			
			break;
		}
		//定位 POS:
		case MODE_POSITION:
		{
			PID_Control(target_pos);
			break;
		}
		//闲置 STOP
		case MODE_IDLE:
		{
//			Send_Motor_Current(MOTOR_ID,0);
			cmd_motor(&hcan,0x200,0,0,0,0);
			break;
		}
		case MODE_INIT:
		{
			Limit_Init();
		}
	}
}

//PID初始�?
void PID_Init()
{
	pid.Kp=PID_KP;
	pid.Ki=PID_KI;
	pid.Kd=PID_KD;
	pid.A0=1.0f;
	pid.A1=0.0f;
	pid.A2=0.0f;
	arm_pid_init_f32(&pid,1);
}

//实现PID电机控制
void PID_Control(int32_t target)
{
	
	pid_target=(float32_t)target;
	float error=pid_target-(float32_t)motor.total_angle;
	
	pid_output=arm_pid_f32(&pid,error);
	
	if(pid_output>0&&error>5000)
	{
		arm_clip_f32(&pid_output,&pid_output,500,MAX_CURRENT,1);
	}else if(pid_output<0&&error<-5000)
	{
		arm_clip_f32(&pid_output,&pid_output,-MAX_CURRENT,-500,1);
	}else
	{
		arm_clip_f32(&pid_output,&pid_output,-MAX_CURRENT,MAX_CURRENT,1);
	}	
	cmd_motor(&hcan,0x200,(int16_t)pid_output,(int16_t)pid_output,(int16_t)pid_output,(int16_t)pid_output);	
}

const char* WorkMode_To_Str(WorkMode mode) {
    const char* mode_str[] = {"IDLE", "PENDULUM", "POSITION", "INIT"};
    return (mode <= MODE_INIT) ? mode_str[mode] : "UNKNOWN";
}

//定时器中断回调（用于安全�?测）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	if(htim==&htim3)//10ms定时�?
//	{
//		static int32_t last_pos=0;
//		
//		//�?测堵转（位置变化过小
//		if(abs(motor.total_angle-last_pos)<5)
//		{
//			//触发保护停止
//			cmd_motor(&hcan,0x200,0,0,0,0);
//			work_mode=MODE_IDLE;
//		}
//		last_pos=motor.total_angle;
//	}
	if(htim==&htim4)
	{
		//发�?�状态数�?
		uint8_t uart_buf[64];
		_snprintf((char*)uart_buf,sizeof(uart_buf),
			"Pos:%ld L:%ld R:%ld mode:%s\r\n",
			motor.total_angle,
			motor.left_limit,
			motor.right_limit,
			WorkMode_To_Str(work_mode));
		HAL_UART_Transmit(&huart3,uart_buf,strlen((char*)uart_buf),10);
	}
}

//CAN通信初始�?
void CAN_Filter_Init(CAN_HandleTypeDef *hcan)
{
	CAN_FilterTypeDef can_filter;
	can_filter.FilterIdHigh=0x0000;
	can_filter.FilterIdLow=0x0000;
	can_filter.FilterMaskIdHigh=0x0000;
	can_filter.FilterMaskIdLow=0x0000;
	can_filter.FilterFIFOAssignment=CAN_RX_FIFO0;
	can_filter.FilterBank=0;
	can_filter.FilterMode=CAN_FILTERMODE_IDMASK;
	can_filter.FilterScale=CAN_FILTERSCALE_32BIT;
	can_filter.FilterActivation=ENABLE;
	
	HAL_CAN_ConfigFilter(hcan,&can_filter);
	HAL_CAN_Start(hcan);
	HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
}

//UART接收回调函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t size)
{
	if(huart==&huart3)
	{
		char *start = (char*)uart_rx_buf;
        char *end = strchr(start, '\n');
        
        while(end != NULL) {
            // 拷贝单条指令到处理缓冲区
            uint8_t len = end - start;
            len = (len > UART_CMD_MAX_LEN-1) ? UART_CMD_MAX_LEN-1 : len;
            memcpy(uart_cmd_buf, start, len);
            uart_cmd_buf[len] = '\0';
            
            // 处理下一条指令
            start = end + 1;
            end = strchr(start, '\n');
        }
		HAL_UART_Transmit_DMA(&huart3,uart_cmd_buf,sizeof(uart_cmd_buf));
        HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_15);
		Process_UART_Command(uart_cmd_buf);
        // 重启DMA接收
		memset(uart_rx_buf,0,UART_BUF_SIZE);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_rx_buf, UART_BUF_SIZE);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_CAN_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Stop(&htim3);
  
  CAN_Filter_Init(&hcan);
  PID_Init();
  
//  HAL_TIM_Base_Start_IT(&htim3);//启动10ms定时�?
  HAL_TIM_Base_Start_IT(&htim4);
  Limit_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  if(uart_data_ready)
//	  {
//		  Process_UART_Command(uart_rx_buf);
//		  HAL_UART_Transmit(&huart3,uart_rx_buf,UART_BUF_SIZE,100);
//		  uart_data_ready=0;
//		  memset(uart_rx_buf,0,UART_BUF_SIZE);
//	  }
	  Motor_Control_Task();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
