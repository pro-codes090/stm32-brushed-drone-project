/*
 * lora_sx1268.c
 *
 *  Created on: 06-Aug-2023
 *      Author: pro
 */

#include "lora_sx1278.h"


void Sx1278_spiwrite(SPI_HandleTypeDef *hspi1 , uint8_t LoRa_Addr, uint8_t* data ,uint8_t length ){
LoRa_Addr = LoRa_Addr | 0x80 ;
HAL_SPI_Transmit(hspi1 , &LoRa_Addr , 1, HAL_MAX_DELAY);

for (uint8_t i = 0;  i < length; i++) {
HAL_SPI_Transmit(hspi1, data, 1, HAL_MAX_DELAY) ;
data++ ;
}

__HAL_SPI_DISABLE(hspi1) ;
}

void Sx1278_spiread(SPI_HandleTypeDef *hspi1 , uint8_t LoRa_Addr , uint8_t* data ,uint8_t length ){

	LoRa_Addr = LoRa_Addr & 0x7F;
	HAL_SPI_Transmit(hspi1 , &LoRa_Addr , 1, HAL_MAX_DELAY);

	for (uint8_t i = 0;  i < length; i++) {
	HAL_SPI_Receive(hspi1, data, 1, HAL_MAX_DELAY) ;
		data++ ;
	}

	__HAL_SPI_DISABLE(hspi1) ;

}
static void LoRa_reset (){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, SET) ;
	HAL_Delay(10) ;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, RESET) ;
	HAL_Delay(10) ;

}
static void Setmode (SPI_HandleTypeDef *hspi1 , uint8_t mode){
	uint8_t temp = 0 ;
	Sx1278_spiread(hspi1, LR_RegOpMode,&temp , 1) ;

	if (mode == STATE_SLEEP) {
		temp = temp | STATE_SLEEP ;
	}else if (mode == STATE_STDBY) {
		temp = temp | STATE_STDBY ;
	}else if (mode == STATE_FSTX) {
		temp = temp | STATE_FSTX ;
	}else if (mode == STATE_TX ) {
		temp = temp | STATE_TX ;
	}else if (mode == STATE_FSRX) {
		temp = temp | STATE_FSRX ;
	}else if (mode == STATE_RXCONTINUOUS) {
		temp = temp | STATE_RXCONTINUOUS ;
	}else if (mode == STATE_RXSINGLE) {
		temp = temp | STATE_RXSINGLE ;
	}else if (mode == STATE_CAD) {
		temp = temp | STATE_CAD ;
	}
	// go to sleep here
	Sx1278_spiwrite(hspi1, LR_RegOpMode, &temp, 1) ;

}

static void set_spreadingfactor(SPI_HandleTypeDef *hspi1){
	uint8_t temp = 0x03 ;
	Sx1278_spiwrite(hspi1, 0x31 ,&temp , 1) ;
	temp = 0x0A ;
	Sx1278_spiwrite(hspi1, 0x37 ,&temp , 1) ;

	// read the value of the config2 reg
	Sx1278_spiread(hspi1, LR_RegModemConfig2, &temp, 1) ;
	temp = ((temp & 0x0F ) | (7 << 4)) ;
	Sx1278_spiwrite(hspi1, LR_RegModemConfig2, &temp, 1) ;

}

static void set_errorcoding(SPI_HandleTypeDef *hspi1){
	uint8_t temp = 0 ;
	Sx1278_spiread(hspi1, LR_RegModemConfig1, &temp, 1) ;
	temp = ((temp & 0xF1)|(1<<1)) ;
	Sx1278_spiread(hspi1, LR_RegModemConfig1, &temp, 1) ;

}

static void set_frequency(SPI_HandleTypeDef *hspi1){
// for 433mhz ;
	uint32_t freq = 0x00806c ;
	Sx1278_spiwrite(hspi1, LR_RegFrMsb, (uint8_t*)&freq, 3) ;
}

static void set_sync(SPI_HandleTypeDef *hspi1){
	uint8_t temp = 0x44 ;
	Sx1278_spiwrite(hspi1, RegSyncWord, &temp, 1);
}

static void setcrc(SPI_HandleTypeDef *hspi1){
	uint8_t temp = 0 ;
	Sx1278_spiread(hspi1, LR_RegModemConfig2, &temp, 1) ;
	temp = ((temp & 0xF5)|(1<<1)) ;
	Sx1278_spiwrite(hspi1, LR_RegModemConfig2, &temp, 1) ;

}

static void set_bandwidth(SPI_HandleTypeDef *hspi1){
	uint8_t temp = 0 ;
	Sx1278_spiread(hspi1, LR_RegModemConfig1, &temp, 1) ;
	temp = ((temp & 0x0F)|(1<<7)) ;
	Sx1278_spiwrite(hspi1, LR_RegModemConfig1, &temp, 1) ;

}

static void set_headermode(SPI_HandleTypeDef *hspi1) {
	uint8_t temp = 0 ;
	Sx1278_spiread(hspi1, LR_RegModemConfig1, &temp, 1) ;
	temp = ((temp & 0xFE)|(1<<0)) ;
	Sx1278_spiwrite(hspi1, LR_RegModemConfig1, &temp, 1) ;

}

static void set_symbTimeout(SPI_HandleTypeDef *hspi1){
	uint8_t temp = 0 ;
	Sx1278_spiread(hspi1, LR_RegModemConfig2, &temp, 1) ;
	// for bits 8-9
	temp = ((temp & 0xFE)|(3<<0)) ;
	Sx1278_spiwrite(hspi1, LR_RegModemConfig2, &temp, 1) ;

	Sx1278_spiread(hspi1, LR_RegSymbTimeoutLsb, &temp, 1) ;
	// for bits 0-7
	temp = ((temp & 0x00)|(0xFF));
	Sx1278_spiwrite(hspi1, LR_RegSymbTimeoutLsb, &temp, 1) ;

}

static void set_payloadlength(SPI_HandleTypeDef *hspi1) {
	uint8_t temp = 128 ;
	Sx1278_spiwrite(hspi1, LR_RegPayloadLength, &temp, 1) ;
}

static void set_lowdatarateoptimise(SPI_HandleTypeDef *hspi1) {
	uint8_t temp = 0 ;
	Sx1278_spiread(hspi1, LR_RegModemConfig3, &temp, 1) ;
	temp = ((temp & 0xF7)|(1<<3)) ;
	Sx1278_spiwrite(hspi1, LR_RegModemConfig3, &temp, 1) ;
}

static void goto_lora(SPI_HandleTypeDef *hspi1){
	uint8_t temp = 0 ;
	// got to sleep mode
	Setmode(hspi1, STATE_SLEEP) ;
	Sx1278_spiread(hspi1, LR_RegOpMode,&temp , 1) ;
	// enable the lora mode
	temp = temp | (0x80) ;
	// emable low frequencyy mode
	temp = temp | (0x08) ;
	// using the lora moderm with low fre here
	Sx1278_spiwrite(hspi1, LR_RegOpMode, &temp, 1) ;
	// go to standby mode
	Setmode(hspi1, STATE_STDBY) ;
	// do the diomapping
	temp = 0x00 ;
	Sx1278_spiwrite(hspi1, REG_LR_DIOMAPPING1, &temp, 1) ;
	temp = 0x00 ;
	Sx1278_spiwrite(hspi1, REG_LR_DIOMAPPING2, &temp, 1) ;

}

void Sx1278_Init(SPI_HandleTypeDef *hspi1 ){

// Reset the lora module over gpio pin
LoRa_reset();
// go into lora mode
goto_lora(hspi1) ;
// set the frequency
set_frequency(hspi1);
// set the sync word
set_sync(hspi1) ;
// set the spreading factor
set_spreadingfactor(hspi1) ;
// set the error coding
set_errorcoding(hspi1) ;
// set the crc setting
setcrc(hspi1) ;
// set bandwidth
set_bandwidth (hspi1) ;
// set headermode
set_headermode(hspi1) ;
// set symboltimeout
set_symbTimeout(hspi1);
// set payloadlength
set_payloadlength(hspi1) ;
// set datarate optimization
set_lowdatarateoptimise(hspi1) ;
// set set to standby mode
Setmode(hspi1, STATE_STDBY) ;
}

// uint8_t process (SPI_HandleTypeDef *hspi1 ){
////  get the process i.e what is the current state of lora module such as
////RF_RX_DONE 	0
////RF_RX_TIMEOUT 1
////RF_TX_DONE 	2
////RF_TX_TIMEOUT 3
////RF_LEN_ERROR 	4
//uint8_t temp = 0 ;
//Sx1278_spiwrite(hspi1,LR_RegIrqFlagsMask, &temp, 1);
//
//
// }
void Sx1278_start_rx(SPI_HandleTypeDef *hspi1 ){
	// got to rxcontinous mode
	Setmode(hspi1, STATE_RXCONTINUOUS) ;

}
void Sx1278_recieve(SPI_HandleTypeDef *hspi1 , uint8_t *buff){
	uint8_t temp = 0 ;
	// got to standdby mode
	Setmode(hspi1, STATE_STDBY) ;
	// read the irq flags
	Sx1278_spiread(hspi1, LR_RegIrqFlags, &temp, 1) ;
	if ((temp & 0x40 ) != 0 ){
		temp = 0xff ;
		Sx1278_spiwrite(hspi1, LR_RegIrqFlags, &temp, 1) ;
		// set the rx fifo buffer adddr
		Sx1278_spiread(hspi1, LR_RegFifoRxCurrentaddr, &temp, 1) ;
		// write the value of rx buffer addr to main addr pointer
		Sx1278_spiwrite(hspi1, LR_RegFifoAddrPtr, &temp, 1) ;
		// read the payload length recieved
		Sx1278_spiread(hspi1, LR_RegPayloadLength, &temp, 1) ;

		for (uint8_t i = 0; i < temp ; i++) {
				Sx1278_spiread(hspi1, LR_RegFifo, buff, 1) ;
				buff++ ;
		}
	}
	Setmode(hspi1, STATE_RXCONTINUOUS) ;
	return  ;
}

void Sx1278_send(SPI_HandleTypeDef *hspi1 , uint8_t *buff , uint8_t length){
uint8_t temp = 0;
uint16_t timeout = 1000 ;
// got to standby mode
Setmode(hspi1, STATE_STDBY) ;
// read the fifotxbaseaddr
Sx1278_spiread(hspi1, LR_RegFifoTxBaseAddr, &temp, 1) ;
// modify the fifo addr pointer
Sx1278_spiread(hspi1, LR_RegFifoAddrPtr, &temp, 1) ;
// put the amount of data you want to transmitt
temp = length ;
Sx1278_spiread(hspi1, LR_RegPayloadLength, &temp, 1) ;
// write to the fifo buffer
Sx1278_spiread(hspi1, LR_RegFifo, buff, length) ;

while(1){
	 Sx1278_spiread(hspi1, LR_RegIrqFlags, &temp, 1);
	if((temp & 0x08)!=0){
		temp = 0xff ;
		Sx1278_spiwrite(hspi1, LR_RegIrqFlags, &temp, 1);
		Setmode(hspi1, STATE_STDBY) ;
		return ;
	}
	else{
		if(--timeout==0){
			Setmode(hspi1, STATE_STDBY) ;
			return ;
		}
	}
	HAL_Delay(1);
}

}

