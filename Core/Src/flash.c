#include "flash.h"
#include "stm32wlxx_hal_conf.h"
#include "spi.h"




extern SPI_HandleTypeDef hspi1;


spi_write_t Win_Write_Enable(void){
	HAL_StatusTypeDef status;
	uint8_t cmd = WRITE_ENABLE;

	HAL_GPIO_WritePin(WIN_CS_PORT,WIN_CS_PIN,GPIO_PIN_RESET);

	status = HAL_SPI_Transmit(&hspi1,&cmd,1,HAL_MAX_DELAY);

	HAL_GPIO_WritePin(WIN_CS_PORT,WIN_CS_PIN,GPIO_PIN_SET);

	if(status != HAL_OK){
		return WIN_SPI_ERROR;
	}

	return WIN_SPI_OK;
}


spi_write_t Win_Write_Enable_Volatile(void){
	HAL_StatusTypeDef status;
	uint8_t cmd = VOLATILE_SR_WRITE_ENABLE;

	HAL_GPIO_WritePin(WIN_CS_PORT,WIN_CS_PIN,GPIO_PIN_RESET);

	status = HAL_SPI_Transmit(&hspi1,&cmd,1,HAL_MAX_DELAY);

	HAL_GPIO_WritePin(WIN_CS_PORT,WIN_CS_PIN,GPIO_PIN_SET);

	if(status != HAL_OK){
		return WIN_SPI_ERROR;
	}

	return WIN_SPI_OK;
}


spi_write_t Win_Write_Disable(void){
	HAL_StatusTypeDef status;
	uint8_t cmd = WRITE_DISABLE;

	HAL_GPIO_WritePin(WIN_CS_PORT,WIN_CS_PIN,GPIO_PIN_RESET);

	status = HAL_SPI_Transmit(&hspi1,&cmd,1,HAL_MAX_DELAY);

	HAL_GPIO_WritePin(WIN_CS_PORT,WIN_CS_PIN,GPIO_PIN_SET);

	if(status !=HAL_OK){
		return WIN_SPI_ERROR;
	}


	return WIN_SPI_OK;

}


spi_write_t Win_Read_Status_Register(uint8_t* sr_value, Status_Register_t sreg){
	HAL_StatusTypeDef status;

	uint8_t tx[2] = {sreg};
	uint8_t rx[2] = {0};


	HAL_GPIO_WritePin(WIN_CS_PORT, WIN_CS_PIN,GPIO_PIN_RESET);

	status = HAL_SPI_TransmitReceive(&hspi1,tx,rx,2,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(WIN_CS_PORT,WIN_CS_PIN,GPIO_PIN_SET);

	if(status != HAL_OK){
		return WIN_SPI_ERROR;
	}
	*sr_value = rx[1];
	return WIN_SPI_OK;
}




spi_write_t Win_Write_Status_Register(win_sreg_write_t sreg, const uint8_t sreg_bytes){
	HAL_StatusTypeDef status;

	 if(Win_Write_Enable()!=WIN_SPI_OK){
		return WIN_SPI_ERROR;
	 }

	 

	 uint8_t tx[2] = {sreg,sreg_bytes};
	 HAL_GPIO_WritePin(WIN_CS_PORT,WIN_CS_PIN,GPIO_PIN_RESET);
	 status = HAL_SPI_Transmit(&hspi1,tx,2,HAL_MAX_DELAY);


	 HAL_GPIO_WritePin(WIN_CS_PORT,WIN_CS_PIN,GPIO_PIN_SET);




	 if(status != HAL_OK){
			 return WIN_SPI_ERROR;
		 }

	 uint8_t sr_value;
	 do{
	 		 Win_Read_Status_Register(&sr_value,SREG1);
	 	 }
	 while( sr_value & 0x01);


	 return WIN_SPI_OK;

}


spi_write_t Win_Write_Status_Register_Volatile(win_sreg_write_t sreg, const uint8_t sreg_bytes){

	HAL_StatusTypeDef status;
	uint8_t tx[2]={sreg,sreg_bytes};
	Win_Write_Enable_Volatile();



	HAL_GPIO_WritePin(WIN_CS_PORT,WIN_CS_PIN,GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(&hspi1,tx,2,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(WIN_CS_PORT,WIN_CS_PIN,GPIO_PIN_SET);

	if(status != HAL_OK){
		return WIN_SPI_ERROR;
	}
	uint8_t sr_value;
	do {
			Win_Read_Status_Register(&sr_value,SREG1);
	}
	while(sr_value & 0x01);

	return WIN_SPI_OK;
}


spi_write_t Win_Read_Data(const uint8_t address[], uint8_t result[], int reads){
	HAL_StatusTypeDef status;
	uint8_t sr_value;
	Win_Read_Status_Register(&sr_value,SREG1);

	if(sr_value & 0x01){
		return WIN_SPI_BUSY;
	}

	uint8_t CMD[4] = {0};
	CMD[0]=READ_DATA;
	for(int i=0; i<3;i++){
		CMD[i+1]=address[i];
	}

	HAL_GPIO_WritePin(WIN_CS_PORT,WIN_CS_PIN,GPIO_PIN_RESET);

	status = HAL_SPI_Transmit(&hspi1,CMD,4,HAL_MAX_DELAY);
	if(status != HAL_OK){
			HAL_GPIO_WritePin(WIN_CS_PORT,WIN_CS_PIN,GPIO_PIN_SET);
			return WIN_SPI_ERROR;
	}

	status = HAL_SPI_Receive(&hspi1,result,reads,HAL_MAX_DELAY);

	if(status != HAL_OK){
			HAL_GPIO_WritePin(WIN_CS_PORT,WIN_CS_PIN,GPIO_PIN_SET);
			return WIN_SPI_ERROR;
		}


	HAL_GPIO_WritePin(WIN_CS_PORT,WIN_CS_PIN,GPIO_PIN_SET);



	return WIN_SPI_OK;
}

spi_write_t Win_Fast_Read_Data(const uint8_t address[], uint8_t result[], int reads){
	HAL_StatusTypeDef status;
	uint8_t sr_value;
	Win_Read_Status_Register(&sr_value, SREG1);

	if(sr_value & 0x01){
		return WIN_SPI_BUSY;
	}

	uint8_t CMD[5] = {0};
	CMD[0] = FAST_READ;
	CMD[4] = 0xFF;
	for(int i = 0; i < 3; i++){
		CMD[i+1] = address[i];
	}

	HAL_GPIO_WritePin(WIN_CS_PORT, WIN_CS_PIN, GPIO_PIN_RESET);

	status = HAL_SPI_Transmit(&hspi1, CMD, 5, HAL_MAX_DELAY);
	if(status != HAL_OK){
		HAL_GPIO_WritePin(WIN_CS_PORT, WIN_CS_PIN, GPIO_PIN_SET);
		return WIN_SPI_ERROR;
	}

	status = HAL_SPI_Receive(&hspi1, result, reads, HAL_MAX_DELAY);
	if(status != HAL_OK){
		HAL_GPIO_WritePin(WIN_CS_PORT, WIN_CS_PIN, GPIO_PIN_SET);
		return WIN_SPI_ERROR;
	}

	HAL_GPIO_WritePin(WIN_CS_PORT, WIN_CS_PIN, GPIO_PIN_SET);
	return WIN_SPI_OK;
}


spi_write_t WIN_Page_Program(const uint8_t address[], uint8_t data[], int n){
	HAL_StatusTypeDef status;

	if(n <= 0 || n > 256){
		return WIN_SPI_ERROR;
	}

	uint8_t tx[4 + 256] = {0};
	tx[0] = PAGE_PROGRAM;

	for(int i = 0; i < 3; i++){
		tx[i+1] = address[i];
	}
	for(int i = 0; i < n; i++){
		tx[4+i] = data[i];
	}

	if(Win_Write_Enable() != WIN_SPI_OK){
		return WIN_SPI_ERROR;
	}

	HAL_GPIO_WritePin(WIN_CS_PORT, WIN_CS_PIN, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(&hspi1, tx, 4+n, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(WIN_CS_PORT, WIN_CS_PIN, GPIO_PIN_SET);

	if(status != HAL_OK){
		return WIN_SPI_ERROR;
	}

	uint8_t sr_value;
	do {
		if(Win_Read_Status_Register(&sr_value, SREG1) != WIN_SPI_OK){
			return WIN_SPI_ERROR;
		}
	} while(sr_value & 0x01);

	return WIN_SPI_OK;
}


spi_write_t Win_Sector_Erase(const uint8_t address[]){

	HAL_StatusTypeDef status;

	uint8_t tx[4] = {0};

	tx[0]=SECTOR_ERASE;
	tx[1]=address[0];
	tx[2]=address[1];
	tx[3]=address[2];

	if(Win_Write_Enable() != WIN_SPI_OK){
			return WIN_SPI_ERROR;
		}

	HAL_GPIO_WritePin(WIN_CS_PORT,WIN_CS_PIN,GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(&hspi1, tx, 4,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(WIN_CS_PORT,WIN_CS_PIN,GPIO_PIN_SET);

	if(status != HAL_OK){
			return WIN_SPI_ERROR;
		}

		uint8_t sr_value;
		do {
			if(Win_Read_Status_Register(&sr_value, SREG1) != WIN_SPI_OK){
				return WIN_SPI_ERROR;
			}
		} while(sr_value & 0x01);


	return WIN_SPI_OK;
}


spi_write_t Block_Erase_32K(const uint8_t address[]){
	HAL_StatusTypeDef status;

		uint8_t tx[4] = {0};

		tx[0]=BLOCK_ERASE_32K;
		tx[1]=address[0];
		tx[2]=address[1];
		tx[3]=address[2];

		if(Win_Write_Enable() != WIN_SPI_OK){
				return WIN_SPI_ERROR;
			}

		HAL_GPIO_WritePin(WIN_CS_PORT,WIN_CS_PIN,GPIO_PIN_RESET);
		status = HAL_SPI_Transmit(&hspi1, tx, 4,HAL_MAX_DELAY);
		HAL_GPIO_WritePin(WIN_CS_PORT,WIN_CS_PIN,GPIO_PIN_SET);


		if(status != HAL_OK){
				return WIN_SPI_ERROR;
			}

			uint8_t sr_value;
			do {
				if(Win_Read_Status_Register(&sr_value, SREG1) != WIN_SPI_OK){
					return WIN_SPI_ERROR;
				}
			} while(sr_value & 0x01);


		return WIN_SPI_OK;
}




spi_write_t Block_Erase_64K(const uint8_t address[]){
	HAL_StatusTypeDef status;

		uint8_t tx[4] = {0};

		tx[0]=BLOCK_ERASE_64K;
		tx[1]=address[0];
		tx[2]=address[1];
		tx[3]=address[2];

		if(Win_Write_Enable() != WIN_SPI_OK){
				return WIN_SPI_ERROR;
			}

		HAL_GPIO_WritePin(WIN_CS_PORT,WIN_CS_PIN,GPIO_PIN_RESET);
		status = HAL_SPI_Transmit(&hspi1, tx, 4,HAL_MAX_DELAY);
		HAL_GPIO_WritePin(WIN_CS_PORT,WIN_CS_PIN,GPIO_PIN_SET);


		if(status != HAL_OK){
				return WIN_SPI_ERROR;
			}

			uint8_t sr_value;
			do {
				if(Win_Read_Status_Register(&sr_value, SREG1) != WIN_SPI_OK){
					return WIN_SPI_ERROR;
				}
			} while(sr_value & 0x01);


		return WIN_SPI_OK;
}


spi_write_t Win_Chip_Erase(){
	HAL_StatusTypeDef status;

	uint8_t CMD = CHIP_ERASE;


	if(Win_Write_Enable() != WIN_SPI_OK){
					return WIN_SPI_ERROR;
				}

			HAL_GPIO_WritePin(WIN_CS_PORT,WIN_CS_PIN,GPIO_PIN_RESET);
			status = HAL_SPI_Transmit(&hspi1, &CMD, 1,HAL_MAX_DELAY);
			HAL_GPIO_WritePin(WIN_CS_PORT,WIN_CS_PIN,GPIO_PIN_SET);


			if(status != HAL_OK){
					return WIN_SPI_ERROR;
				}

				uint8_t sr_value;
				do {
					if(Win_Read_Status_Register(&sr_value, SREG1) != WIN_SPI_OK){
						return WIN_SPI_ERROR;
					}
				} while(sr_value & 0x01);


			return WIN_SPI_OK;
}



spi_write_t Win_Erase_Program_Suspend(void){

	HAL_StatusTypeDef status;
	uint8_t sr1, sr2;
	uint8_t cmd = ERASE_PROGRAM_SUSPEND; // 0x75

	// Read Status Register-1 (contains BUSY at bit 0)
	if(Win_Read_Status_Register(&sr1, SREG1) != WIN_SPI_OK){
		return WIN_SPI_ERROR;
	}

	// Read Status Register-2 (contains SUS at bit 7, i.e. S15)
	if(Win_Read_Status_Register(&sr2, SREG2) != WIN_SPI_OK){
		return WIN_SPI_ERROR;
	}

	
	if(!(sr1 & 0x01) || (sr2 & 0x80)){
		return WIN_SPI_ERROR; 
	}

	HAL_GPIO_WritePin(WIN_CS_PORT, WIN_CS_PIN, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(WIN_CS_PORT, WIN_CS_PIN, GPIO_PIN_SET);

	if(status != HAL_OK){
		return WIN_SPI_ERROR;
	}

	// Datasheet: tSUS max = 20us -- BUSY clears from 1->0 and SUS sets 0->1
	// within this window. Poll rather than blind-delay for correctness.
	uint32_t start = HAL_GetTick();
	uint8_t sus_confirmed = 0;

	while((HAL_GetTick() - start) < 5){ // 5ms generous margin over 20us spec
		if(Win_Read_Status_Register(&sr2, SREG2) != WIN_SPI_OK){
			return WIN_SPI_ERROR;
		}
		if(sr2 & 0x80){ // SUS bit set -> suspend confirmed
			sus_confirmed = 1;
			break;
		}
	}

	if(!sus_confirmed){
		return WIN_SPI_ERROR; // suspend didn't take effect within expected window
	}

	return WIN_SPI_OK;
}