/* Includes   -----------------------------------------------------------------*/
#include "main.h"

/**
 * @brief  The application entry point.
 * @retval int
 */

uint8_t header[2];
uint8_t rxbuffer[13];
uint8_t msg[13];
uint8_t sum;

char str[50];
uint16_t final;

int flag1 = 0;
int flag2 = 0;

int main(void) {
	set();
	HAL_UART_Receive_IT(&huart5, header, 2);
	while (1) {

//		HAL_UART_Receive(&huart2, rxbuffer, 15,100);
//

		if (flag2) {
			final = rxbuffer[2] << 8;
			final |= rxbuffer[1];
			sprintf(str, "received data -> %d \n",(int16_t)((final * 360) / 65536));
			HAL_UART_Transmit(&huart2, (uint8_t*) str, strlen(str), 100);
			flag2 = 0;

		}

	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == huart5.Instance) {

		if (!flag1) {
			if (header[0] == 0xAA) { //&& header[1] & 0x00) {
				flag1 = 1;
				HAL_UART_Receive_IT(&huart5, rxbuffer, 13);

			} else {
				header[0] = 0;
				HAL_UART_Receive_IT(&huart5, header, 2);
			}
		} else {
			for (int i = 0; i < 12; i++) {
				sum |= rxbuffer[i];
			}
			if (sum == rxbuffer[12]) {
				flag2 = 1;
			}
			flag1 = 0;
			header[0] = 0;
			HAL_UART_Receive_IT(&huart5, header, 2);

		}

	}

}

void TIM6_DAC_IRQHandler(void) {
	led1 = !led1;
	HAL_TIM_IRQHandler(&htim6);

}

/**
 * @brief  This function is executed in case of error occurrence.
 */
void Error_Handler(void) {

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

