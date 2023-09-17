/*
 * mpu6050_driver.c
 *
 *  Created on: Jul 23, 2023
 *      Author: pro
 */

// include all the prototypes and macros
#include "mpu6050_driver.h"
#define DEBUG_mpu 0
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim3;

void Self_test_mpu6050(I2C_HandleTypeDef *hi2c) {

	 uint8_t data[4] = {0} ;
	 uint8_t  Gyro_ST[3] = {0};
	 uint8_t  ACCL_ST[3] = {0} ;
	 float Gyro_FT[3] = {0} ;
	 float Accl_FT[3] = {0} ;

	 // begin test
	 data[0] = 0xE0;
	 HAL_I2C_Mem_Write(hi2c, MPU_ADDR, GYRO_CONFIG, 1,&data[0] , 1, HAL_MAX_DELAY); // enable gyro self test
	 data[0] = 0xF0 ;
	 HAL_I2C_Mem_Write(hi2c, MPU_ADDR, ACCEL_CONFIG, 1,&data[0] , 1, HAL_MAX_DELAY); // enable accel self test
	 HAL_Delay(150) ;
	 HAL_I2C_Mem_Read(hi2c, MPU_ADDR, SELF_TEST_X, 1, &data[0], 1, HAL_MAX_DELAY) ;
	 HAL_I2C_Mem_Read(hi2c, MPU_ADDR, SELF_TEST_Y, 1, &data[1], 1, HAL_MAX_DELAY) ;
	 HAL_I2C_Mem_Read(hi2c, MPU_ADDR, SELF_TEST_Z, 1, &data[2], 1, HAL_MAX_DELAY) ;
	 HAL_I2C_Mem_Read(hi2c, MPU_ADDR, SELF_TEST_A, 1, &data[3], 1, HAL_MAX_DELAY) ;

	 Gyro_ST[0] = (data[0] & 0x1F) ;	// X
	 Gyro_ST[1] = (data[1] & 0x1F) ;	// Y
	 Gyro_ST[2] = (data[2] & 0x1F) ;	// Z

	 ACCL_ST[0] = (((data[0] &  0xE0 ) >> 3) | ((data[3] & 0x30 ) >> 4));	//X
	 ACCL_ST[1] = (((data[1] &  0xE0 ) >> 3) | ((data[3] & 0x0C ) >> 2));	//Y
	 ACCL_ST[2] = (((data[2] &  0xE0 ) >> 3) | ((data[3] & 0x03 ) >> 0)); //Z

#if DEBUG_mpu
	printf("running mpu6050 test \n") ;

	printf("GYRO_ST[0] %u \n" , Gyro_ST[0]) ;
	printf("GYRO_ST[1] %u \n" , Gyro_ST[1]) ;
	printf("GYRO_ST[2] %u \n" , Gyro_ST[3]) ;

	printf("ACC_ST[0] %u \n" , ACCL_ST[0]) ;
	printf("ACC_ST[1] %u \n" , ACCL_ST[1]) ;
	printf("ACC_ST[2] %u \n" , ACCL_ST[3]) ;
#endif

	Gyro_FT[0] = 25.0*131.0*(powf(1.406,Gyro_ST[0]) - 1.0) ;
	Gyro_FT[1] = -25.0*131.0*(powf(1.406,Gyro_ST[1]) - 1.0) ;
	Gyro_FT[2] = 25.0*131.0*(powf(1.406,Gyro_ST[2]) - 1.0) ;

	Accl_FT[0] = 4096.0*0.34*(powf((0.92/0.34) ,
							  ((ACCL_ST[0] - 1.0)/(30)))); ;
	Accl_FT[1] = 4096.0*0.34*(powf((0.92/0.34 ),
							  ((ACCL_ST[1] - 1.0)/(30))));
	Accl_FT[2] = 4096.0*0.34*(powf((0.92/0.34) ,
							  ((ACCL_ST[2] - 1.0)/(30)))) ;
#if DEBUG_mpu
	printf("running mpu6050 test finished \n") ;

	printf("GYRO_FT[0] %F \n" , Gyro_FT[0]) ;
	printf("GYRO_FT[1] %F \n" , Gyro_FT[1]) ;
	printf("GYRO_FT[2] %F \n" , Gyro_FT[3]) ;

	printf("ACC_FT[0] %f \n" , Accl_FT[0]) ;
	printf("ACC_FT[1] %f \n" , Accl_FT[1]) ;
	printf("ACC_FT[2] %f \n" , Accl_FT[3]) ;
#endif

	float temp = 0 ;
	for (uint8_t i = 0; i < 3; i++ ) {
	temp = (100 + ((( Gyro_ST[i]- Gyro_FT[i] )/ Gyro_FT[i] )*100 ));
	printf("testing result Gyro %f \n", temp) ;
	}
temp = 0 ;
	for (uint8_t i = 0; i < 3; i++ ) {

	temp = (100 + ((( ACCL_ST[i]- Accl_FT[i] )/ Accl_FT[i] )*100 ));
	printf("testing result Accl %f \n", temp) ;
	}
}

void Mpu6050_Init(I2C_HandleTypeDef *hi2c ){
uint8_t data = 0x00;

//who am I
data = MPU_DATASHEET_ADDR ;
HAL_I2C_Mem_Read(hi2c, MPU_ADDR, WHO_AM_I, 1, &data, 1, HAL_MAX_DELAY) ;
if (data != MPU_DATASHEET_ADDR) {
	printf("who am i error \n");
}else if (data == MPU_DATASHEET_ADDR ) {
	printf("who am I value : %x \n", data) ;
}

// power mannagment 1
data = 0x01 ;
HAL_I2C_Mem_Write(hi2c, MPU_ADDR, PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY) ;
HAL_I2C_Mem_Read(hi2c, MPU_ADDR, PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY) ;
if (data != 0x01) {
	printf("pwr_mgmt1 error \n");
}else if (data == 0x01 ) {
	printf("pwr_mgmt1 value : %d \n", data) ;
}


// Configuration CONFIG 0x1A 26
data = 0x01;
HAL_I2C_Mem_Write(hi2c, MPU_ADDR, CONFIG, 1, &data, 1, HAL_MAX_DELAY) ;
HAL_I2C_Mem_Read(hi2c, MPU_ADDR, CONFIG, 1, &data, 1, HAL_MAX_DELAY) ;

if (data != 0x01) {
	printf("config error \n");
}else if (data == 0x01 ) {
	printf("config value : %d \n", data) ;
}

// Sample rate divider
data = 0x04;
HAL_I2C_Mem_Write(hi2c, MPU_ADDR, SMPLRT_DIV, 1, &data, 1, HAL_MAX_DELAY) ;
HAL_I2C_Mem_Read(hi2c, MPU_ADDR, SMPLRT_DIV, 1, &data, 1, HAL_MAX_DELAY) ;

if (data != 0x04) {
	printf("sample rate divider error \n");
}else if (data == 0x04 ) {
	printf("sample rate divider value : %d \n", data) ;
}


// Gyro Config
data = 0x08;
HAL_I2C_Mem_Write(hi2c, MPU_ADDR, GYRO_CONFIG, 1, &data, 1, HAL_MAX_DELAY) ;
HAL_I2C_Mem_Read(hi2c, MPU_ADDR, GYRO_CONFIG, 1, &data, 1, HAL_MAX_DELAY) ;

if (data != 0x08) {
	printf("Gyro config error \n ");
}else if (data == 0x08 ) {
	printf("gyro config value : %d \n", data) ;
}

// Accl Config
data = 0x00;
HAL_I2C_Mem_Write(hi2c, MPU_ADDR, ACCEL_CONFIG, 1, &data, 1, HAL_MAX_DELAY) ;
HAL_I2C_Mem_Read(hi2c, MPU_ADDR, ACCEL_CONFIG, 1, &data, 1, HAL_MAX_DELAY) ;

if (data != 0x00) {
	printf("Accl config error \n");
}else if (data == 0x00 ) {
	printf("Accl config value : %d \n", data) ;
}

// signal path reset
data = 0x07 ;
HAL_I2C_Mem_Write(hi2c, MPU_ADDR, SIGNAL_PATH_RESET, 1, &data, 1, HAL_MAX_DELAY) ;
HAL_I2C_Mem_Read(hi2c, MPU_ADDR, SIGNAL_PATH_RESET, 1, &data, 1, HAL_MAX_DELAY) ;
if (data != 0x07) {
	printf("(ignore) signal path reset error cannot read write only \n");
}else if (data == 0x07 ) {
	printf("signal path reset value : %d \n", data) ;
}

}

void get_Accl(I2C_HandleTypeDef *hi2c , MPU_Accl_Val_t * Accl_Data ){

	uint8_t data [6];
	int16_t accel_x = 0 ;
	int16_t accel_y = 0 ;
	int16_t accel_z = 0 ;

	HAL_I2C_Mem_Read(hi2c, MPU_ADDR,ACCEL_XOUT_H , 1, data, 6, HAL_MAX_DELAY) ;

	accel_x = data[0] << 8 | data[1] ;
	accel_y = data[2] << 8 | data[3] ;
    accel_z = data[4] << 8 | data[5] ;

    Accl_Data->pitch = (float)( (float)accel_x  / (float)16384 )*(float)9.8 ;
    Accl_Data->roll  = (float)( (float)accel_y  / (float)16384 )*(float)9.8 ;
    Accl_Data->yaw   = (float)( (float)accel_z  / (float)16384 )*(float)9.8 ;

#if DEBUG_mpu

	printf("[DEBUG] pitch_a: %0.1lf ,roll_a=  %0.1lf ,yaw_a=  %0.1lf \n" , Accl_Data->pitch , Accl_Data->roll, Accl_Data->yaw);
#endif


}

void get_gyro(I2C_HandleTypeDef *hi2c , MPU_Gyro_Val_t * Gyro_Data , MPU_Gyro_calib_t * Calib_Data) {

	uint8_t data [6];
	int16_t gyro_x = 0 ;
	int16_t gyro_y = 0 ;
	int16_t gyro_z = 0 ;

	HAL_I2C_Mem_Read(hi2c, MPU_ADDR,GYRO_XOUT_H , 1, data, 6, HAL_MAX_DELAY) ;

	gyro_x = data[0] << 8 | data[1] ;
    gyro_y = data[2] << 8 | data[3] ;
	gyro_z = data[4] << 8 | data[5] ;

	Gyro_Data->pitch = ( gyro_x  / 65.5 ) - Calib_Data->pitch ;
	Gyro_Data->roll  = ( gyro_y  / 65.5 ) - Calib_Data->roll;
	Gyro_Data->yaw   = ( gyro_z  / 65.5 ) - Calib_Data->yaw ;

#if DEBUG_mpu

	printf("[DEBUG] pitch: %0.1lf ,roll=  %0.1lf ,yaw=  %0.1lf \n" , Gyro_Data->pitch , Gyro_Data->roll, Gyro_Data->yaw);
#endif

}

void gyro_calibrate (I2C_HandleTypeDef *hi2c , MPU_Gyro_calib_t * Calib_Data){
	uint8_t data [6];
	int16_t gyro_x = 0 ;
	int16_t gyro_y = 0 ;
	int16_t gyro_z = 0 ;

	float pitch , yaw , roll ;
	double pitch_cal =  0, yaw_cal =  0, roll_cal = 0;
	__HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_4 ,16000  );
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) ;
	printf("/////////// CALIBRATING GYRO \\\\\\\\\\\\\\\\\\\\ \n") ;
	for (uint16_t i = 0; i  < 4000 ; i++) {

		// needed to communicate with i2c based devices like gyroscope mpu6050
	   HAL_I2C_Mem_Read(hi2c, MPU_ADDR,GYRO_XOUT_H , 1, data, 6, HAL_MAX_DELAY) ;

	   gyro_x = data[0] << 8 | data[1] ;
	   gyro_y = data[2] << 8 | data[3] ;
	   gyro_z = data[4] << 8 | data[5] ;

	   pitch = gyro_x  / 65.5 ; // change in x
	   roll  = gyro_y  / 65.5 ;	// change in y
	   yaw   = gyro_z  / 65.5 ;	// change in z

	   pitch_cal += pitch ;  // this means pitch_cal = pitch + pitch_cal
	   roll_cal += roll ;
	   yaw_cal += yaw ;
	   HAL_Delay(1) ;
	}

	// taking average
	pitch_cal = pitch_cal/ 4000  ;
	roll_cal  = roll_cal / 4000  ;
	yaw_cal   = yaw_cal  / 4000  ;

	printf("cal values are  %lf , %lf , %lf \n" , pitch_cal , roll_cal , yaw_cal);

	Calib_Data->pitch = pitch_cal ;
	Calib_Data->roll  = roll_cal;
	Calib_Data->yaw   = yaw_cal;

	printf("cal values are =  pitch : %lf ,roll: %lf ,yaw: %lf \n" , Calib_Data->pitch ,Calib_Data->roll , Calib_Data->yaw);

	__HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_4 ,0);

	__HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_3 ,32000);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) ;
printf("/////////////// DONE GYRO CALIBRATION \\\\\\\\\\\\\\\\\\\\ \n") ;

#if DEBUG_mpu
printf("[DEBUG] ///////// PRINTING 50 SAMPLES OF CALIBRATED GYRO DATA FOR TESTING PUROPOSE \\\\\\\\\\\ \n") ;
	for (uint8_t i = 0;  i < 50 ; i++) {
		  HAL_I2C_Mem_Read(hi2c, MPU_ADDR,GYRO_XOUT_H , 1, data, 6, HAL_MAX_DELAY) ;

		  gyro_x = data[0] << 8 | data[1] ;
		  gyro_y = data[2] << 8 | data[3] ;
		  gyro_z = data[4] << 8 | data[5] ;

		  pitch = ( gyro_x  / 65.5 ) - pitch_cal ;
		  roll  = ( gyro_y  / 65.5 ) - roll_cal;
		  yaw   = ( gyro_z  / 65.5 ) - yaw_cal ;

		  printf(" pitch:  %lf , roll:  %lf , yaw:  %lf \n" , pitch, roll, yaw);

	}
	printf("[DEBUG] ///////// SAMPLE DATA END \\\\\\\\\\\ \n") ;
#endif

}

