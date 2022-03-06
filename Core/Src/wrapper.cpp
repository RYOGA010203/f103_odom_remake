// 動作確認してないけど、一応リメイクした。
// 元のiocファイルの内容とmain.cppの内容が一致しない部分(ピン配置等)があったため、
// main.cppの記述を優先した。
#include "main.h"
#include "Odometry.h"
#include "stm32f1xx_hal.h"
#include "led.h"
#include <array>
#include "MadgwickAHRS.h"
#include "CanClass.hpp"

SPI_HandleTypeDef hspi2; //ジャイロとの通信用
TIM_HandleTypeDef htim2; //TIM3,4はエンコーダー用
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart1; //デバッグ用に残しておく

//周波数に合わせて下のとこいじって(丸投げ)
static constexpr uint32_t CAN_Freq = 200;
static constexpr uint16_t CAN_duration = 1000 / CAN_Freq;

float tx_header_x;
float tx_header_y;
float tx_header_yaw;

extern Madgwick MDGF;

Odometry *odom = new Odometry();

// CANで送るデータ入れ
static float X;
static float Y;
static float Yaw;

// CANで送るID
uint32_t send_data_x_id = 0x205;
uint32_t send_data_y_id = 0x206;
uint32_t send_data_yaw_id = 0x207;

static uint32_t last_time = 0;

// CANで受け取るデータ入れ
uint8_t receive_data;

// CANで受け取るID
uint32_t odom_board_id = 0x204;

CanClass can;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	can.receive(receive_data, 0x101);
	can.endit();//割り込み終了
}

/*
void TIM2_IRQHandler(void) {
	HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_SET);
	if (TIM2->SR & TIM_SR_UIF) {
		odom->Sample();

		TIM2->SR &= ~TIM_SR_UIF;
	}

	led_process();
}
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		odom->Sample();
}

void main_cpp(void)
{
	SPI2->CR1 |= SPI_CR1_SPE;	// SPIスタート
	HAL_TIM_Base_Start_IT(&htim2); //タイマー割り込み用
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); //エンコーダ用
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	// CAN初期化
	can.init(odom_board_id, 1000000);

	while(!(odom->Initialize())) {
		HAL_Delay(10);
	}

	MDGF.setBeta(1.0f);

	last_time = HAL_GetTick();
	while (HAL_GetTick() - last_time <= 1000){ //フィルターの上での初期姿勢{0,0,1G}が実際の姿勢に収束するまでの待ち時間
		asm("NOP");
	}

	odom->GetPose(&X, &Y, &Yaw);
	odom->SetOffsetYaw(Yaw);

	MDGF.setBeta(0.12f);

	while(1)
	{
		if (HAL_GetTick() - last_time >= CAN_duration){

			odom->GetPose(&X, &Y, &Yaw);

			can.send(send_data_x_id, X);
			asm("NOP");
			asm("NOP");
			asm("NOP");
			asm("NOP");
			asm("NOP");
			asm("NOP");
			asm("NOP");
			asm("NOP");
			asm("NOP");
			asm("NOP");
			can.send(send_data_y_id, Y);
			asm("NOP");
			asm("NOP");
			asm("NOP");
			asm("NOP");
			asm("NOP");
			asm("NOP");
			asm("NOP");
			asm("NOP");
			asm("NOP");
			asm("NOP");
			can.send(send_data_yaw_id, Yaw);

			HAL_GPIO_TogglePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);

			last_time = HAL_GetTick();
		}
	}
}
