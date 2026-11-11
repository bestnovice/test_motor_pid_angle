/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS‑IS.
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drv_bsp.h"
#include "drv_can.h"
#include "dvc_motor.h"
#include "drv_uart.h"
#include "math.h"
#include <string.h>
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* MK15 v1帧结构体 */
#define MK15_FRAME_LEN 30U
#define MK15_TIMEOUT_MS 300U

#define ARM_MASS_KG    (0.1f)
#define ARM_LENGTH_M   (0.1f)
#define M3508_KT       (0.3f)     // N·m/A 标定值
// 水平位置最大补偿: m*g*L/Kt = 1.96/0.3 ≈ 6.5A → 换算counts
#define GRAV_FF_MAX    (ARM_MASS_KG * 9.81f * ARM_LENGTH_M / M3508_KT / 10.0f * 16384.0f)  // ≈10690
/* USER CODE END PD */

typedef struct
{
    uint8_t  version;
    uint8_t  flags;
    uint16_t seq;
    uint16_t payload_len;
    float Vx;
    float Vy;
    float w;
    float w_left;
    float w_right;
} MK15_Frame_t;
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//电机1/2: 每秒在0度与顺时针90度(输出轴)之间摆动, 换算成转子弧度(电机类Gearbox_Rate=1.0f, 角度为转子单位)
#define SWING_ANGLE_RAD     (0)
//电机3/4: 每秒2圈(输出轴), 换算成转子rad/s
#define TARGET_OMEGA_3_4    (0)

#define speed_max   (100)
#define roate_std   (1)
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
//电机1/2: 角度环控制(类内部角度环+速度环串级), 电机3/4: 速度环控制
Class_Motor_C620 motor1;
Class_Motor_C620 motor2;
Class_Motor_C620 motor3;
Class_Motor_C620 motor4;

//电机反馈状态(仅内存，不上报串口)
float Target_Angle1, Now_Angle1, Target_Omega1, Now_Omega1;
float Target_Angle2, Now_Angle2, Target_Omega2, Now_Omega2;
float Target_Omega3, Now_Omega3;
float Target_Omega4, Now_Omega4;

/* MK15 接收解析变量 */
MK15_Frame_t mk15_rx_frame;
uint8_t mk15_rx_buf[MK15_FRAME_LEN];
uint16_t mk15_rx_idx = 0U;
uint32_t mk15_last_valid_tick = 0U;

//由遥控量解算得到的电机目标
float tar_m1_angle;
float tar_m2_angle;
float tar_m3_omega;
float tar_m4_omega;

volatile uint32_t UART_Err_Cnt = 0;    //UART错误中断次数, Watch观察
volatile uint32_t UART_Rearm_Cnt = 0;  //接收重启次数, Watch观察

/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t crc16_ccitt_false(const uint8_t *data, uint16_t len);
uint8_t mk15_parse_frame(const uint8_t *raw, MK15_Frame_t *out);
void mk15_input_byte(uint8_t ch);
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief CAN报文回调函数, 分发给4个电机对象
 *
 * @param Rx_Buffer CAN接收的信息结构体
 */
void CAN_Motor_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    switch (Rx_Buffer->Header.StdId)
    {
    case (0x201):   //电机1 角度控制
    {
        motor1.CAN_RxCpltCallback(Rx_Buffer->Data);
    }
    break;
    case (0x202):   //电机2 角度控制
    {
        motor2.CAN_RxCpltCallback(Rx_Buffer->Data);
    }
    break;
    case (0x203):   //电机3 速度控制
    {
        motor3.CAN_RxCpltCallback(Rx_Buffer->Data);
    }
    break;
    case (0x204):   //电机4 速度控制
    {
        motor4.CAN_RxCpltCallback(Rx_Buffer->Data);
    }
    break;
    }
}

/**
 * @brief HAL库UART2 DMA空闲中断回调，接收MK15二进制字节流
 *
 * @param Buffer 接收缓冲区首地址
 * @param Length 本次收到字节数
 */
void UART_Serialplot_Call_Back(uint8_t *Buffer, uint16_t Length)
{
    for(uint16_t i = 0; i < Length; i++)
    {
        mk15_input_byte(Buffer[i]);
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
    MX_CAN1_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    BSP_Init(BSP_DC24_LU_ON | BSP_DC24_LD_ON | BSP_DC24_RU_ON | BSP_DC24_RD_ON);
    CAN_Init(&hcan1, CAN_Motor_Call_Back);

    //USART2 开启DMA空闲接收，仅用于MK15接收，不发送反馈
    UART_Init(&huart2, UART_Serialplot_Call_Back, 512);

    CAN_Filter_Mask_Config(&hcan1, CAN_FILTER(0) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);

    //电机1/2: 角度环+速度环串级
    motor1.PID_Angle.Init(0.0f, 0.0f, 0.0f, 0.0f, 500.0f * PI, 500.0f * PI);
    motor1.PID_Omega.Init(100.0f, 0.3f, 0.0f, 0.0f,5000.0f, 10000.0f);
    motor1.Init(&hcan1, CAN_Motor_ID_0x201, Control_Method_ANGLE, 1.0f);

    motor2.PID_Angle.Init(150.0f, 0.15f, 0.0f, 0.0f, 500.0f * PI, 500.0f * PI);
    motor2.PID_Omega.Init(100.0f, 0.3f, 0.0f, 0.0f, 5000.0f, 10000.0f);
    motor2.Init(&hcan1, CAN_Motor_ID_0x202, Control_Method_ANGLE, 1.0f);

    //电机3/4: 纯速度环
    motor3.PID_Omega.Init(100.0f, 0.5f, 0.0f, 0.0f, 2500.0f, 8000.0f);
    motor3.Init(&hcan1, CAN_Motor_ID_0x203, Control_Method_OMEGA, 1.0f);

    motor4.PID_Omega.Init(100.0f, 0.5f, 0.0f, 0.0f, 2500.0f, 8000.0f);
    motor4.Init(&hcan1, CAN_Motor_ID_0x204, Control_Method_OMEGA, 1.0f);

    motor3.Set_Target_Omega(TARGET_OMEGA_3_4);
    motor4.Set_Target_Omega(TARGET_OMEGA_3_4);

    //上电目标全部置零
    tar_m1_angle = 0.0f;
    tar_m2_angle = 0.0f;
    tar_m3_omega = 0.0f;
    tar_m4_omega = 0.0f;
    mk15_last_valid_tick = HAL_GetTick();
		
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);   // PE11 灭
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET); 
    /* USER CODE END 2 */
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    while (1)
    {
        //==== MK15 300ms超时保护 ====
        if((HAL_GetTick() - mk15_last_valid_tick) > MK15_TIMEOUT_MS)
        {
            tar_m1_angle  = 0.0f;
            tar_m2_angle  = 0.0f;
            tar_m3_omega  = 0.0f;
            tar_m4_omega  = 0.0f;
							HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);   // PE11 灭
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET); 
        }
        else
        {
            //上位机flags bit0=0：无效帧，强制归零
            if( !(mk15_rx_frame.flags & 0x01U) )
            {
                tar_m1_angle  = 0.0f;
                tar_m2_angle  = 0.0f;
                tar_m3_omega  = 0.0f;
                tar_m4_omega  = 0.0f;
							
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);   // PE11 灭
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);
            }
            else 
            {
                //########################
                // !!!此处填写你的运动学解算!!!
                // 输入：mk15_rx_frame.Vx, Vy, w, w_left, w_right
                // 输出赋值给 tar_m1_angle tar_m2_angle tar_m3_omega tar_m4_omega
                // 示例：
                tar_m1_angle = mk15_rx_frame.w_left*roate_std;
                tar_m2_angle = mk15_rx_frame.w_right*roate_std*(-1);
                 tar_m3_omega = 0.7*mk15_rx_frame.Vx*speed_max+0.3*mk15_rx_frame.Vy*speed_max;
                 tar_m4_omega = -0.7*mk15_rx_frame.Vx*speed_max+0.3*mk15_rx_frame.Vy*speed_max;
                //########################
                   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);   
                   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET); 
						}
            }
        //==== 串口接收自愈: 掉线或接收挂死时自动重启接收 ====
        if((HAL_GetTick() - mk15_last_valid_tick) > 500U)
        {
            if((huart2.RxState == HAL_UART_STATE_READY) && (huart2.ReceptionType != HAL_UART_RECEPTION_TOIDLE))
            {
                HAL_UART_AbortReceive(&huart2);
                if(HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART2_Manage_Object.Rx_Buffer, UART2_Manage_Object.Rx_Buffer_Length) == HAL_OK)
                {
                    UART_Rearm_Cnt++;
                }
            }
        }

        //设置电机目标
        motor1.Set_Target_Angle(tar_m1_angle);
        motor2.Set_Target_Angle(tar_m2_angle);
        motor3.Set_Target_Omega(tar_m3_omega);
        motor4.Set_Target_Omega(tar_m4_omega);

        //读取电机反馈(仅内存，不上报串口)
        Target_Angle1 = motor1.Get_Target_Angle();
        Now_Angle1    = motor1.Get_Now_Angle();
        Target_Omega1 = motor1.Get_Target_Omega();
        Now_Omega1    = motor1.Get_Now_Omega();

        Target_Angle2 = motor2.Get_Target_Angle();
        Now_Angle2    = motor2.Get_Now_Angle();
        Target_Omega2 = motor2.Get_Target_Omega();
        Now_Omega2    = motor2.Get_Now_Omega();

        Target_Omega3 = motor3.Get_Target_Omega();
        Now_Omega3    = motor3.Get_Now_Omega();
        Target_Omega4 = motor4.Get_Target_Omega();
        Now_Omega4    = motor4.Get_Now_Omega();

    //   TIM_UART_PeriodElapsedCallback();
       motor1.Feedforward =  GRAV_FF_MAX * cosf(3.14/6);
//        motor2.Feedforward = -GRAV_FF_MAX * cosf(Now_Angle2);
        //电机PID运算
        motor1.TIM_PID_PeriodElapsedCallback();
        motor2.TIM_PID_PeriodElapsedCallback();
        motor3.TIM_PID_PeriodElapsedCallback();
        motor4.TIM_PID_PeriodElapsedCallback();
				
        TIM_CAN_PeriodElapsedCallback();

        HAL_Delay(0);
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
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 6;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Activate the Over‑Drive mode
     */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/**
 * @brief UART错误回调, 上位机掉线/噪声导致溢出帧错时清错并重启接收,
 *        保证串口重新连接后能自动恢复, 无需复位下位机
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        UART_Err_Cnt++;
        __HAL_UART_CLEAR_OREFLAG(huart);   //只做清错+计数, 不在中断里重挂接收, 避免错误中断风暴饿死主循环; 接收重启由主循环自愈逻辑负责
    }
}


/**
 * @brief CRC‑16/CCITT‑FALSE，与python binascii.crc_hqx(body,0xffff)完全一致
 */
uint16_t crc16_ccitt_false(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    for(uint16_t i=0; i<len; i++)
    {
        crc ^= ((uint16_t)data[i]) << 8U;
        for(uint8_t b=0; b<8U; b++)
        {
            if(crc & 0x8000U)
                crc = (crc << 1U) ^ 0x1021U;
            else
                crc = crc << 1U;
        }
    }
    return crc;
}

/**
 * @brief 解析完整30字节MK15帧
 * @param raw 输入完整30字节缓冲区
 * @param out 输出解析结构体
 * @retval 1成功；0失败(帧头/CRC/版本/载荷长度错误)
 */
uint8_t mk15_parse_frame(const uint8_t *raw, MK15_Frame_t *out)
{
    if(raw[0] != 0xA5 || raw[1] != 0x5A)
        return 0;

    uint16_t calc_crc = crc16_ccitt_false(raw, 28);
    uint16_t recv_crc = ((uint16_t)raw[29] << 8) | raw[28];
    if(calc_crc != recv_crc)
        return 0;

    out->version      = raw[2];
    out->flags        = raw[3];
    out->seq          = ((uint16_t)raw[5] <<8) | raw[4];
    out->payload_len  = ((uint16_t)raw[7] <<8) | raw[6];

    memcpy(&out->Vx,      &raw[8],  4);
    memcpy(&out->Vy,      &raw[12], 4);
    memcpy(&out->w,       &raw[16], 4);
    memcpy(&out->w_left,  &raw[20], 4);
    memcpy(&out->w_right, &raw[24], 4);

    if(out->version != 1U || out->payload_len != 20U)
        return 0;

    return 1;
}

/**
 * @brief 逐字节送入，状态机搜索帧头0xA5 0x5A，收满30字节执行解析
 */
void mk15_input_byte(uint8_t ch)
{
    if(mk15_rx_idx == 0U)
    {
        if(ch != 0xA5)
            return;
        mk15_rx_buf[mk15_rx_idx++] = ch;
        return;
    }
    if(mk15_rx_idx == 1U)
    {
        if(ch != 0x5A)
        {
            mk15_rx_idx = 0U;
            return;
        }
        mk15_rx_buf[mk15_rx_idx++] = ch;
        return;
    }

    mk15_rx_buf[mk15_rx_idx++] = ch;
    if(mk15_rx_idx >= MK15_FRAME_LEN)
    {
        if(mk15_parse_frame(mk15_rx_buf, &mk15_rx_frame))
        {
            mk15_last_valid_tick = HAL_GetTick();
        }
        mk15_rx_idx = 0U;
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
#ifdef USE_FULL_ASSERT
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
