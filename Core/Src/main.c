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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "BNO080.h"
#include "Quaternion.h"
#include "ICM20602.h"
#include "LPS22HH.h"
#include "FS-iA6B.h"
#include "PID control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//implement _write function so that printf will work
//takes the pointer location for the first char in the string to print
int _write(int file, char* p, int len)
{
	for (int i = 0; i < len; i++)
	{
		// Wait until transmit data register is empty
		while (!LL_USART_IsActiveFlag_TXE(USART6)) {}
		//adds to the initial pointer location for every char in string
		LL_USART_TransmitData8(USART6, *(p+i));
//		HAL_Delay(1);
	}
	return len;
}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//	flag to know if we've got data - initialized in stm32f4xx_it.c
extern uint8_t uart6_rx_flag;
//	variable to store the received data
extern uint8_t uart6_rx_data;

extern uint8_t ibus_rx_buf[32];//ibus message buffer
extern uint8_t ibus_rx_cplt_flag; //full message received flag

extern uint8_t tim7_1ms_flag;
extern uint8_t tim7_1000ms_flag;

unsigned char failsafe_flag = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int is_iBus_throttle_min(void);
void ESC_calibration(void);
int is_iBus_received(void);
void BNO080_Calibration(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  float q[4];
  float quatRadianAccuracy;
  unsigned short ccr1, ccr2, ccr3, ccr4;

  float yaw_heading_reference;

  unsigned char motor_arming_flag = 0;
  unsigned short iBus_SwA_Prev = 0;
  unsigned char iBus_rx_cnt = 0;



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
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_UART5_Init();
  MX_TIM5_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  LL_TIM_EnableCounter(TIM3);

  LL_USART_EnableIT_RXNE(USART6);//Interrupt for usart6 for terminal output
  LL_USART_EnableIT_RXNE(UART5);//Interrupt for uart5 for radio receiver data input

  //initialize BNO080 - includes SPI2 and GPIO configurations
  BNO080_Initialization();
  //Set output to be rotation vector and rate to 2500ms which is the max 400Hz output rate
//  BNO080_enableRotationVector(2500);
  //NOTE: Once ive implemented BNO080 calibration function, replace above line with the one below this comment. the line above will be in the BNO080 calibration function
  BNO080_enableGameRotationVector(2500);

  //initialize ICM-20602 - includes SPI1 and GPIO configurations
  //NOTE: This doesn't enable the accelerometer, to enable, go into
  //the function def and uncomment that line.
  ICM20602_Initialization();

  //initialize LPS22HH barometric pressure and temp sensor
  LPS22HH_Initialization();

  LL_TIM_EnableCounter(TIM5);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH4);


  LL_TIM_EnableCounter(TIM7);
  LL_TIM_EnableIT_UPDATE(TIM7);

  while (is_iBus_received() == 0)//wait until we get connection with the controller
  {
	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);

	  TIM3->PSC = 3000;
	  HAL_Delay(200);
	  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  HAL_Delay(200);
  }

  if (iBus.SwC == 2000)//check if we should calibrate
  {
	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  TIM3->PSC = 1500;
	  HAL_Delay(200);
	  TIM3->PSC = 2000;
	  HAL_Delay(200);
	  TIM3->PSC = 1500;
	  HAL_Delay(200);
	  TIM3->PSC = 2000;
	  HAL_Delay(200);
	  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);

	  ESC_calibration(); //calibrate ESCs

	  while (iBus.SwC != 1000)//wait until switch is turned off of calibration mode
	  {
		  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		  TIM3->PSC = 1500;
		  HAL_Delay(200);
		  TIM3->PSC = 2000;
		  HAL_Delay(200);
		  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		  is_iBus_received();//continue updating iBus message struct
	  }
  }
  else if (iBus.SwC == 1500)
  {
	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  TIM3->PSC = 1500;
	  HAL_Delay(200);
	  TIM3->PSC = 2000;
	  HAL_Delay(200);
	  TIM3->PSC = 1500;
	  HAL_Delay(200);
	  TIM3->PSC = 2000;
	  HAL_Delay(200);
	  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);

	  BNO080_Calibration(); //calibrate ESCs

	  while (iBus.SwC != 1000)//wait until switch is turned off of calibration mode
	  {
		  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		  TIM3->PSC = 1500;
		  HAL_Delay(200);
		  TIM3->PSC = 2000;
		  HAL_Delay(200);
		  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		  is_iBus_received();//continue updating iBus message struct
	  }
  }

  while (is_iBus_throttle_min() == 0 || iBus.SwA == 2000)//wait until throttle is at minimum and switch A is high (ie motors not armed) to continue
  {
	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  TIM3->PSC = 1000;
	  HAL_Delay(70);
	  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  HAL_Delay(70);
  }

  //play start up sound once throttle is at min
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
  TIM3->PSC = 2000;
  HAL_Delay(100);
  TIM3->PSC = 1500;
  HAL_Delay(100);
  TIM3->PSC = 1000;
  HAL_Delay(100);
  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);


  //set pid gains
  roll.in.kp = 5.0f;
  roll.in.ki = 5.0f;
  roll.in.kd = 1.7f;
  roll.out.kp = 45.0f;
  roll.out.ki = 3.0f;
  roll.out.kd = 4.0f;

  pitch.in.kp = 6.5f;
  pitch.in.ki = 5.0f;
  pitch.in.kd = 1.5f;
  pitch.out.kp = 45.0f;
  pitch.out.ki = 3.0f;
  pitch.out.kd = 4.0f;

  yaw_heading.kp = 50.0f;
  yaw_heading.ki = 0.0f;
  yaw_heading.kd = 20.0f;

  yaw_rate.kp = 15.0f;
  yaw_rate.ki = 0.0f;
  yaw_rate.kd = 2.0f;


  //set current heading as initial target for when drone boots up.
  //NOTE: change this to be inside motor arming condition once its implemented
  yaw_heading_reference = BNO080_Yaw;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */




	  //performs at 1kHz, set up for PID calculations
	  if (tim7_1ms_flag == 1)
	  {
		  tim7_1ms_flag = 0;

		  Double_Roll_Pitch_PID_Calculation(&pitch, ((iBus.RV - 1500) * 0.1f), BNO080_Pitch, ICM20602.gyro_x);
		  Double_Roll_Pitch_PID_Calculation(&roll, ((iBus.RH - 1500) * 0.1f), BNO080_Roll, ICM20602.gyro_y);

//		  if (iBus.LV <= 1030 || motor_arming _flag == 0) //REPLACE once ive implemented motor arming
		  //reset integrator values when landed
		  if (iBus.LV <= 1030)
		  {
			  Reset_All_PID_Integrator();
		  }

		  //if we try to change yaw position, use angular rate pid control loop.
		  if (iBus.LH < 1485 || iBus.LH > 1515)
		  {
			  yaw_heading_reference = BNO080_Yaw;//set new heading reference so that it maintains new position after each rotation

			  Single_Yaw_Rate_PID_Calculation(&yaw_rate, (iBus.LH - 1500), ICM20602.gyro_z);

			  ccr1 = (10500 + 500 + ((iBus.LV - 1000) * 10)) - pitch.in.pid_result + roll.in.pid_result - yaw_rate.pid_result;//convert controller value to pulse width (range of 10500 to 21000) and assign to CCR
			  ccr2 = (10500 + 500 + ((iBus.LV - 1000) * 10)) + pitch.in.pid_result + roll.in.pid_result + yaw_rate.pid_result;
			  ccr3 = (10500 + 500 + ((iBus.LV - 1000) * 10)) + pitch.in.pid_result - roll.in.pid_result - yaw_rate.pid_result;
	  		  ccr4 = (10500 + 500 + ((iBus.LV - 1000) * 10)) - pitch.in.pid_result - roll.in.pid_result + yaw_rate.pid_result;
		  }
		  else //else if stick is in center, hold position using heading pid control loop
		  {
			  Single_Yaw_Heading_PID_Calculation(&yaw_heading, yaw_heading_reference, BNO080_Yaw, ICM20602.gyro_z);

			  ccr1 = (10500 + 500 + ((iBus.LV - 1000) * 10)) - pitch.in.pid_result + roll.in.pid_result - yaw_heading.pid_result;//convert controller value to pulse width (range of 10500 to 21000) and assign to CCR
			  ccr2 = (10500 + 500 + ((iBus.LV - 1000) * 10)) + pitch.in.pid_result + roll.in.pid_result + yaw_heading.pid_result;
			  ccr3 = (10500 + 500 + ((iBus.LV - 1000) * 10)) + pitch.in.pid_result - roll.in.pid_result - yaw_heading.pid_result;
  	  		  ccr4 = (10500 + 500 + ((iBus.LV - 1000) * 10)) - pitch.in.pid_result - roll.in.pid_result + yaw_heading.pid_result;
		  }

		  //ccr values explained;
		  //(10500 + ((iBus.LV - 1000) * 10.5)) - this calculates throttle, shifts up iBus values from 1000 to 2000 into 0 to 1000, and the converts those to a range of 10500 t0 21000 which is what the pulse width range for ESC is.
		  //(10500 + 500 +((iBus.LV - 1000) * 10)) - changed the throttle to this to include a motor arming phase, full throttle down will have a slight motor spin.
		  //((iBus.RV - 1500) * 5) - this controls pitch - (- 1500) sets the right vertical stick to be 0 at center position, and -500 at full backward, and 500 at full forward. then multiplied by 5 for a gain (this makes each increase into a sizable portion of the 10500-21000 pulse width range) (too high gain and itll be way to angled, too little and wont pitch enough) this is subtracted from the front motors and added to the back ones. when the value is negative, this inverts, and the fronts are added to and the rears are subtracted from
		  //((iBus.RV - 1500) * 5) - this controls roll and yaw as well, just with different addition/subtraction applied to certain motors
//		  ccr1 = (10500 /* + 500 */ + ((iBus.LV - 1000) * 10)) - ((iBus.RV - 1500) * 5) + (iBus.RH - 1500 * 500) - (iBus.LH - 1500 * 500);//convert controller value to pulse width (range of 10500 to 21000) and assign to CCR
//		  ccr2 = (10500 /* + 500 */ + ((iBus.LV - 1000) * 10)) + ((iBus.RV - 1500) * 5) + (iBus.RH - 1500 * 500) + (iBus.LH - 1500 * 500);
//		  ccr3 = (10500 /* + 500 */ + ((iBus.LV - 1000) * 10)) + ((iBus.RV - 1500) * 5) - (iBus.RH - 1500 * 500) - (iBus.LH - 1500 * 500);
//		  ccr4 = (10500 /* + 500 */ + ((iBus.LV - 1000) * 10)) - ((iBus.RV - 1500) * 5) - (iBus.RH - 1500 * 500) + (iBus.LH - 1500 * 500);

//		  ccr1 = (10500 + 500 + ((iBus.LV - 1000) * 10));
//		  ccr2 = (10500 + 500 + ((iBus.LV - 1000) * 10));
//		  ccr3 = (10500 + 500 + ((iBus.LV - 1000) * 10));
//		  ccr4 = (10500 + 500 + ((iBus.LV - 1000) * 10));

//		  ccr1 = (10500 + ((iBus.LV - 1000) * 10));
//		  ccr2 = (10500 + ((iBus.LV - 1000) * 10));
//		  ccr3 = (10500 + ((iBus.LV - 1000) * 10));
//		  ccr4 = (10500 + ((iBus.LV - 1000) * 10));


//	  printf("%f\t%f\n", BNO080_Pitch, ICM20602.gyro_x);
//	  printf("%f\t%f\n", BNO080_Roll, ICM20602.gyro_y);
//	  printf("%f\t%f\n", BNO080_Yaw, ICM20602.gyro_z);
	  }

	  if (iBus.SwA == 2000 && iBus_SwA_Prev != 2000)//if were ready to arm the motors, allow controller access - this only applies to when we are switching from not armed to armed.
	  {
		  if (iBus.LV < 1010)//only arm motors if throttle is down while trying to arm
		  {
			  motor_arming_flag = 1;
		  }
		  else //wait till throttle is down to allow arming, make beeping warning
		  {
			  while (is_iBus_throttle_min() == 0 || iBus.SwA == 2000)//wait until throttle is at minimum and switch A is high (ie motors not armed) to continue
			  {
				  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
				  TIM3->PSC = 1000;
				  HAL_Delay(70);
				  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
				  HAL_Delay(70);
			  }
		  }

	  }

	  iBus_SwA_Prev = iBus.SwA;//update prev value of arming switch

	  if (iBus.SwA != 2000) //if switch is not down, set flag to not armed and kill all motors
	  {
		  motor_arming_flag = 0;
	  }

	  if (motor_arming_flag == 1)
	  {
		  if (failsafe_flag == 0)//check fail safe which monitors controller connection
		  {
			  TIM5->CCR1 = ccr1 > 21000 ? 21000 : ccr1 < 11000 ? 11000 : ccr1; //beware this makes throttle slightly positive as a baseline, meaning with throttle stick all the way down the motors will still spin slightly
			  TIM5->CCR2 = ccr2 > 21000 ? 21000 : ccr2 < 11000 ? 11000 : ccr2;
			  TIM5->CCR3 = ccr3 > 21000 ? 21000 : ccr3 < 11000 ? 11000 : ccr3;
			  TIM5->CCR4 = ccr4 > 21000 ? 21000 : ccr4 < 11000 ? 11000 : ccr4;
		  }
		  else
		  {
			  TIM5->CCR1 = 10500;
			  TIM5->CCR2 = 10500;
			  TIM5->CCR3 = 10500;
			  TIM5->CCR4 = 10500;
		  }

	  }
	  else //if we are not armed, kill all motors
	  {
		  TIM5->CCR1 = 10500;
		  TIM5->CCR2 = 10500;
		  TIM5->CCR3 = 10500;
		  TIM5->CCR4 = 10500;
	  }
//
//	  TIM5->CCR1 = ccr1 ;
//	  TIM5->CCR2 = ccr2 ;
//	  TIM5->CCR3 = ccr3 ;
//	  TIM5->CCR4 = ccr4 ;

//	  TIM5->CCR1 = ccr1 > 21000 ? 21000 : ccr1 < 10500 ? 10500 : ccr1;
//	  TIM5->CCR2 = ccr2 > 21000 ? 21000 : ccr2 < 10500 ? 10500 : ccr2;
//	  TIM5->CCR3 = ccr3 > 21000 ? 21000 : ccr3 < 10500 ? 10500 : ccr3;
//	  TIM5->CCR4 = ccr4 > 21000 ? 21000 : ccr4 < 10500 ? 10500 : ccr4;



//	  check if BNO080 has data for us
	  if (BNO080_dataAvailable() == 1) {
//		  //store the raw values of the quaternion
		  q[0] = BNO080_getQuatI();
		  q[1] = BNO080_getQuatJ();
		  q[2] = BNO080_getQuatK();
		  q[3] = BNO080_getQuatReal();

//		  //store the accuracy - not sure why but spark fun does
		  quatRadianAccuracy = BNO080_getQuatRadianAccuracy();

//		  //send raw values to be turned into Euler angles
//		  //this stores the roll, pitch, and yaw values globally
		  Quaternion_Update(&q[0]);



		  //flip signs to match expected - check with data output to make sure they are correct
		  BNO080_Roll = -BNO080_Roll;
		  BNO080_Pitch = -BNO080_Pitch;

//		  printf("%.2f\t%.2f\n", BNO080_Roll, BNO080_Pitch);


	  }
	  if (ICM20602_DataReady() == 1){

		  ICM20602_Get3AxisGyroRawData(&ICM20602.gyro_x_raw);//by passing the first struct member we want(gyro_x_raw)
		  	  	  	  	  	  	  	  	  	  	  	  	  	//as a reference (its address), the function can
		  	  	  	  	  	  	  	  	  	  	  	  	  	//just add to its address to get to all the other members
		  	  	  	  	  	  	  	  	  	  	  	  	    //kind of like passing the address of an array
		  //convert raw value to degrees per second (dps)
		  ICM20602.gyro_x = ICM20602.gyro_x_raw * 2000.f / 32768.f; //multiply by sensitivity, divide by resolution (16 bits signed)
		  ICM20602.gyro_y = ICM20602.gyro_y_raw * 2000.f / 32768.f; //cast to values to float
		  ICM20602.gyro_z = ICM20602.gyro_z_raw * 2000.f / 32768.f;

		  //flip sign to match expected - check with data output to make sure they are correct
		  ICM20602.gyro_x = -ICM20602.gyro_x;
		  ICM20602.gyro_z = -ICM20602.gyro_z;

		  //printf("A:%d X:%d Y:%d Z:%d B:%d\n", 4000, ICM20602.gyro_x_raw, ICM20602.gyro_y_raw, ICM20602.gyro_z_raw, -4000);
		  //print dps values, multiplied by 100 to save decimal values, divide outputted values by 100 to get actual dps
//		  printf("X:%.2f Y:%.2f Z:%.2f\n", ICM20602.gyro_x, ICM20602.gyro_y, ICM20602.gyro_z);

	  }

	  //if the data is ready
//	  if (LPS22HH_DataReady() == 1) {
//		  //get raw data and pass the addresses of the LPS22HH struct raw members to store it using pass by reference
//		  LPS22HH_GetPressure(&LPS22HH.pressure_raw);
//		  LPS22HH_GetTemperature(&LPS22HH.temperature_raw);
//
//		  //convert to altitude in meters using hPa units for pressure and °C for temperature
//		  //use float values in the equations to type cast parameters to floats
//		  //this is un-filtered pressure
//		  LPS22HH.baroAlt = getAltitude2(LPS22HH.pressure_raw/4096.f, LPS22HH.temperature_raw/100.f);//this returns altitude in meters
//
//		  //Low pass filter on the results, X closer to one means higher frequencies get filtered, closer to 0 means lower values get taken out.
//#define X 0.90f
//		  //DSP filter algorithm
//		  LPS22HH.baroAltFilt = LPS22HH.baroAltFilt * X + LPS22HH.baroAlt  * (1.0f - X);
//
//		  //print non filtered and filtered values
//		  printf("%d,%d\n", (int)(LPS22HH.baroAlt*100), (int)(LPS22HH.baroAltFilt*100));//output in centimeters since we multiply by 100 to save float values
//
//	  }

	  //radio transmitter receiving and parsing
	  if (ibus_rx_cplt_flag == 1) //if we have a full message
	  {
		  ibus_rx_cplt_flag = 0; //reset flag

		  if (ibus_Check_CHKSUM(&ibus_rx_buf[0], 32) == 1)
		  {
			  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_2);//toggle led for confirmation

			  iBus_Parsing(&ibus_rx_buf[0], &iBus);
			  iBus_rx_cnt++; //increment cnt to check if we are receiving data
//
			  if (iBus_isActiveFailsafe(&iBus) == 1)
			  {
				  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4); //make warning sound to indicate no connection
				  failsafe_flag = 1;
//				  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_2);//toggle led for confirmation
//				  HAL_Delay(100); //slight delay to see the led
			  }
			  else
			  {
				  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4); //turn off warning sound
				  failsafe_flag = 0;
//				  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_2);//toggle led for confirmation
//				  HAL_Delay(100); //slight delay to see the led
			  }

//			  printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\n", iBus.RH, iBus.RV, iBus.LV, iBus.LH, iBus.SwA, iBus.SwC, iBus.FailSafe);
//			  HAL_Delay(100); //slight delay to see the led
		  }
	  }
	  if (tim7_1000ms_flag == 1)
	  {
		  tim7_1000ms_flag = 0;
		  if (iBus_rx_cnt == 0)
		  {
			  failsafe_flag = 2;
		  }
		  iBus_rx_cnt = 0;//reset every one second
	  }




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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


int is_iBus_throttle_min(void)
{
	//radio transmitter receiving and parsing
	if (ibus_rx_cplt_flag == 1) //if we have a full message
	{
		ibus_rx_cplt_flag = 0; //reset flag
		if (ibus_Check_CHKSUM(&ibus_rx_buf[0], 32) == 1)
		{
			iBus_Parsing(&ibus_rx_buf[0], &iBus);
			//check if throttle is at minimum
			if (iBus.LV < 1010) //use 1010 since some throttle baselines have a bit extra
			{
				return 1;
			}
		}
	}
	return 0;
}


void ESC_calibration(void)
{
	//esc calibration
	//set PWM to max width for 7 seconds
	TIM5->CCR1 = 21000;
	TIM5->CCR2 = 21000;
	TIM5->CCR3 = 21000;
	TIM5->CCR4 = 21000;
	HAL_Delay(7000);
	//set PWM to min width for 8 seconds
	TIM5->CCR1 = 10500;
	TIM5->CCR2 = 10500;
	TIM5->CCR3 = 10500;
	TIM5->CCR4 = 10500;
	HAL_Delay(8000);
}

int is_iBus_received(void)
{
	//radio transmitter receiving and parsing
	if (ibus_rx_cplt_flag == 1) //if we have a full message
	{
		ibus_rx_cplt_flag = 0; //reset flag
		if (ibus_Check_CHKSUM(&ibus_rx_buf[0], 32) == 1)
		{
			iBus_Parsing(&ibus_rx_buf[0], &iBus);
			return 1;
		}
	}
	return 0;
}



void BNO080_Calibration(void)
{
	//Resets BNO080 to disable All output
	BNO080_Initialization();

	//BNO080/BNO085 Configuration
	//Enable dynamic calibration for accelerometer, gyroscope, and magnetometer
	//Enable Game Rotation Vector output
	//Enable Magnetic Field output
	BNO080_calibrateAll(); //Turn on cal for Accel, Gyro, and Mag
	BNO080_enableGameRotationVector(20000); //Send data update every 20ms (50Hz)
	BNO080_enableMagnetometer(20000); //Send data update every 20ms (50Hz)

	//Once magnetic field is 2 or 3, run the Save DCD Now command
  	printf("Calibrating BNO080. Pull up FS-i6 SWC to end calibration and save to flash\n");
  	printf("Output in form x, y, z, in uTesla\n\n");

	//while loop for calibration procedure
	//Iterates until iBus.SwC is mid point (1500)
	//Calibration procedure should be done while this loop is in iteration.
	while(iBus.SwC == 1500)
	{
		if(BNO080_dataAvailable() == 1)
		{
			//Observing the status bit of the magnetic field output
			float x = BNO080_getMagX();
			float y = BNO080_getMagY();
			float z = BNO080_getMagZ();
			unsigned char accuracy = BNO080_getMagAccuracy();

			float quatI = BNO080_getQuatI();
			float quatJ = BNO080_getQuatJ();
			float quatK = BNO080_getQuatK();
			float quatReal = BNO080_getQuatReal();
			unsigned char sensorAccuracy = BNO080_getQuatAccuracy();

			printf("%f,%f,%f,", x, y, z);
			if (accuracy == 0) printf("Unreliable\t");
			else if (accuracy == 1) printf("Low\t");
			else if (accuracy == 2) printf("Medium\t");
			else if (accuracy == 3) printf("High\t");

			printf("\t%f,%f,%f,%f,", quatI, quatI, quatI, quatReal);
			if (sensorAccuracy == 0) printf("Unreliable\n");
			else if (sensorAccuracy == 1) printf("Low\n");
			else if (sensorAccuracy == 2) printf("Medium\n");
			else if (sensorAccuracy == 3) printf("High\n");

			//Turn the LED and buzzer on when both accuracy and sensorAccuracy is high
			if(accuracy == 3 && sensorAccuracy == 3)
			{
				LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2);
				TIM3->PSC = 65000; //Very low frequency
				LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
			}
			else
			{
				LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2);
				LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
			}
		}

		is_iBus_received(); //Refreshes iBus Data for iBus.SwC
		HAL_Delay(100);
	}

	//Ends the loop when iBus.SwC is not mid point
	//Turn the LED and buzzer off
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2);
	LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);

	//Saves the current dynamic calibration data (DCD) to memory
	//Sends command to get the latest calibration status
	BNO080_saveCalibration();
	BNO080_requestCalibrationStatus();

	//Wait for calibration response, timeout if no response
	int counter = 100;
	while(1)
	{
		if(--counter == 0) break;
		if(BNO080_dataAvailable())
		{
			//The IMU can report many different things. We must wait
			//for the ME Calibration Response Status byte to go to zero
			if(BNO080_calibrationComplete() == 1)
			{
				printf("\nCalibration data successfully stored\n");
				LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
				TIM3->PSC = 2000;
				HAL_Delay(300);
				TIM3->PSC = 1500;
				HAL_Delay(300);
				LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
				HAL_Delay(1000);
				break;
			}
		}
		HAL_Delay(10);
	}
	if(counter == 0)
	{
		printf("\nCalibration data failed to store. Please try again.\n");
		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		TIM3->PSC = 1500;
		HAL_Delay(300);
		TIM3->PSC = 2000;
		HAL_Delay(300);
		LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		HAL_Delay(1000);
	}

	//BNO080_endCalibration(); //Turns off all calibration
	//In general, calibration should be left on at all times. The BNO080
	//auto-calibrates and auto-records cal data roughly every 5 minutes

	//Resets BNO080 to disable Game Rotation Vector and Magnetometer
	//Enables Rotation Vector
	BNO080_Initialization();
	BNO080_enableRotationVector(2500); //Send data update every 2.5ms (400Hz)
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
