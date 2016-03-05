/*-----------------------------------------------------------------------------
 * Name:    stdout_USART.c
 * Purpose: STDOUT USART Template
 * Rev.:    1.0.0
 *-----------------------------------------------------------------------------*/

/* Copyright (c) 2013 - 2015 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
 *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/


#include "stdout_USART.h"
#include "globals.h"
#include "PWM.h"
//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

// <h>STDOUT USART Interface

//   <o>Connect to hardware via Driver_USART# <0-255>
//   <i>Select driver control block for USART interface
#define USART_DRV_NUM           3

//   <o>Baudrate
#define USART_BAUDRATE          57600

// </h>


#define _USART_Driver_(n)  Driver_USART##n
#define  USART_Driver_(n) _USART_Driver_(n)

extern ARM_DRIVER_USART  USART_Driver_(USART_DRV_NUM);
#define ptrUSART       (&USART_Driver_(USART_DRV_NUM))

#define BUFFERSIZE_IN 100
#define BUFFERSIZE_OUT 280
#define BUFFERSIZE_COM 10
#define MAX_SENDING_CODES 8

static uint8_t input_buf[BUFFERSIZE_IN];
static uint8_t output_buffer1[BUFFERSIZE_OUT];
static uint8_t output_buffer2[BUFFERSIZE_OUT];
static int32_t output_currentBufferNr = 1;
static int32_t output_currentPos = 0;
static uint8_t* output_currentBuffer = output_buffer1;
static uint8_t buffer[10];
static uint8_t sendingCodes[MAX_SENDING_CODES];
static uint8_t sendingCodesCurrent = 0;
static uint8_t command[BUFFERSIZE_COM];
static uint8_t kommuConnected = 0;
static uint8_t kommuNoPing = 0;
static const uint8_t maxPingsMissed = 40;

int buildCommand(uint8_t* buffer, uint8_t* command){
	static int currentPos = 0;
	static int commandPos = 0;
	static uint8_t reciving = 0;
	uint8_t currentChar = 0;
	static uint8_t gotStart = 0;
	uint8_t done = 0;
	int i;
	int recCount = ptrUSART->GetRxCount();
	while((recCount > currentPos || currentPos > recCount) & !done){
		currentChar = buffer[currentPos];
		if(currentChar == 0x02){
			gotStart = 1;
			for( i = 0; i < BUFFERSIZE_COM; i++){
				command[i] = 0;
			}
			commandPos = 0;
		}else if(!gotStart){
			//out of message
		}else if(currentChar == 0x03  && (command[0]!='x' || commandPos > 3)){
			gotStart = 0;
			if(commandPos > 0)
				done = 1;
		}else if(commandPos < BUFFERSIZE_COM){
			command[commandPos] = currentChar;
			commandPos++;
		}else{
			puts("command not closed :(");
		}
		currentPos++;
		if(currentPos >= 100){
			currentPos = 0;
		}
	}

	if(!reciving){
		startRec(buffer);
		reciving = 1;
	}		
	return done;
}



int sendCommand(uint8_t * toSend,int lenght){
	int i;
	if(output_currentPos+2+lenght > BUFFERSIZE_OUT)
		return -1;
	output_currentBuffer[output_currentPos++] = 0x02;
	for(i = 0; i < lenght; i++){
		output_currentBuffer[output_currentPos++] = toSend[i];
	}
	output_currentBuffer[output_currentPos++] = 0x03;
	return 0;
}

void myUSART_callback(uint32_t event){
	switch (event)
	{
	case ARM_USART_EVENT_RECEIVE_COMPLETE: 
		startRec(input_buf);
		break;
	case ARM_USART_EVENT_TRANSFER_COMPLETE:
	case ARM_USART_EVENT_SEND_COMPLETE:
	case ARM_USART_EVENT_TX_COMPLETE:
		break;

	case ARM_USART_EVENT_RX_TIMEOUT:
		break;

	case ARM_USART_EVENT_RX_OVERFLOW:
		puts("Lost data");
		break;
	case ARM_USART_EVENT_TX_UNDERFLOW:
		break;
	}
}

void sendCode(uint8_t codeToSend, uint8_t* dataBuffer){
	int lenght = 2;
	uint16_t currentTime = HAL_GetTick()/100;
	//currentTime = 2001;
	buffer[1] = codeToSend;
	if(codeToSend < 0x07){
		buffer[0] = 'V';
		helperShortToBuffer(buffer+2,acceltempgyroValsFiltered[codeToSend]);
		helperShortToBuffer(buffer+4,currentTime);
		lenght += 4;
	}else if(codeToSend < 0x0A){
		buffer[0] = 'W';
		helperIntToBuffer(buffer+2,angleGyro[codeToSend-0x07]);
		helperShortToBuffer(buffer+6,currentTime);
		lenght += 6;
	}
	else if(codeToSend < 0x0D){
		buffer[0] = 'W';
		helperIntToBuffer(buffer+2,angleAccel[codeToSend-0x0A]);
		helperShortToBuffer(buffer+6,currentTime);
		lenght += 6;
	}
	else if(codeToSend < 0x11){
		buffer[0] = 'W';
		helperIntToBuffer(buffer+2,angleComple[codeToSend-0x0D]);
		helperShortToBuffer(buffer+6,currentTime);
		lenght += 6;
	}else if(codeToSend < 0x12){
		buffer[0] = 'W';
		helperIntToBuffer(buffer+2,pidY_X);
		helperShortToBuffer(buffer+6,currentTime);
		lenght += 6;
	}else if(codeToSend < 0x14){
		buffer[0] = 'W';
		helperIntToBuffer(buffer+2,(int32_t)(angleKalman[codeToSend-0x12]*100));
		helperShortToBuffer(buffer+6,currentTime);
		lenght += 6;
	}
	sendCommand(buffer, lenght);
}

void startRecording(uint8_t code){
	uint8_t foundToDel = 255;
	uint8_t i;
	for(i = 0; i < sendingCodesCurrent; i++){
		if(sendingCodes[i] == code-'0'){
			foundToDel = i;
		}
	}
	if(foundToDel == 255 && sendingCodesCurrent < (MAX_SENDING_CODES-1)){
		sendingCodes[sendingCodesCurrent++] = code-'0';
	}
}

void stopRecordingAll(void){
	sendingCodesCurrent = 0;
}

void stopRecording(uint8_t code){
	uint8_t foundToDel = 255;
	uint8_t i;
	for(i = 0; i < sendingCodesCurrent; i++){
		if(sendingCodes[i] == code){
			foundToDel = i;
		}
	}
	if(foundToDel != 255){
		for(i = foundToDel; i < sendingCodesCurrent-1; i++){
			sendingCodes[i] = sendingCodes[i+1];
		}
		sendingCodesCurrent--;
	}
}

void changeValue(uint8_t* dataBuffer){
	uint8_t code = dataBuffer[0]-'0';
	if(code < 4){
		//bldc_set_power((dataBuffer[1]<<8)+dataBuffer[2],code);
		power[code] = helperBufferToInt(dataBuffer+1,2);
	}
	else if(code < 8){
		switch (code) {
			case 4:
				pidDataX->kp = helperBufferToInt(dataBuffer+1,2);
				break;
			case 5:
				pidDataX->ki = helperBufferToInt(dataBuffer+1,2);
				break;
			case 6:
				pidDataY->kp = helperBufferToInt(dataBuffer+1,2);
				break;
			case 7:
				pidDataY->ki = helperBufferToInt(dataBuffer+1,2);
				break;
			default:
				break;
		}
	}else if(code < 12){
		switch (code) {
					case 8:
						kalmanX.kalman_Q_angle = helperBufferToInt(dataBuffer+1,2) / 10000.f;
						break;
					case 9:
						kalmanX.kalman_Q_bias = helperBufferToInt(dataBuffer+1,2)/ 10000.f;
						break;
					case 10:
						kalmanX.kalman_R_measure = helperBufferToInt(dataBuffer+1,2)/ 10000.f;
						break;
					case 7:
						pidDataY->ki = helperBufferToInt(dataBuffer+1,2);
						break;
					default:
						break;
				}
	}

}

void kommuHandler(void){
	int i;
	while(buildCommand(input_buf,command)){
		switch(command[0]){
		case 'a':
			if(kommuConnected){
				sendCommand("a",1);
				kommuNoPing = 0;
			}
			break;
		case 'b':
			sendCommand("c",1);
			kommuConnected = 1;
			kommuNoPing = 0;
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
			break;
		case 's':
			stopRecording(command[1]);
			break;
		case 'r':
			if(kommuConnected){
				startRecording(command[1]);
			}
			break;
		case 'q':
			stopRecordingAll();
			break;
		case 'x':
			if(kommuConnected){
				changeValue(command+1);
			}
			break;
		}		
	}
	if(kommuConnected){
		for(i = 0; i < sendingCodesCurrent; i++){
			sendCode(sendingCodes[i],buffer);
		}
	}
	if(kommuNoPing > maxPingsMissed){ //Max ping loss?
		kommuConnected = 0;
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
	}else{
		kommuNoPing++;
	}

	if((output_currentPos>0) && !ptrUSART->GetStatus().tx_busy){
		ptrUSART->Send(output_currentBuffer,output_currentPos);
		if(output_currentBufferNr == 1){
			output_currentBuffer = output_buffer2;
			output_currentBufferNr = 2;
		}else{
			output_currentBuffer = output_buffer1;
			output_currentBufferNr = 1;
		}
		output_currentPos = 0;
	}
}



int startRec(uint8_t* buffer){
	if (ptrUSART->Receive(buffer, BUFFERSIZE_IN) != ARM_DRIVER_OK) {
		return (-1);
	}
	return 0;
}

/**
  Initialize stdout

  \return          0 on success, or -1 on error.
 */
int std_init (void) {
	int32_t status;

	status = ptrUSART->Initialize(myUSART_callback);
	if (status != ARM_DRIVER_OK) return (-1);

	status = ptrUSART->PowerControl(ARM_POWER_FULL);
	if (status != ARM_DRIVER_OK) return (-1);

	status = ptrUSART->Control(ARM_USART_MODE_ASYNCHRONOUS |
			ARM_USART_DATA_BITS_8       |
			ARM_USART_PARITY_NONE       |
			ARM_USART_STOP_BITS_1       |
			ARM_USART_FLOW_CONTROL_NONE,
			USART_BAUDRATE);
	if (status != ARM_DRIVER_OK) return (-1);

	status = ptrUSART->Control(ARM_USART_CONTROL_TX , 1);
	status = ptrUSART->Control(ARM_USART_CONTROL_RX , 1);
	if (status != ARM_DRIVER_OK) return (-1);

	return (0);
}

/**
  Put a character to the stdout

  \param[in]   ch  Character to output
  \return          The character written, or -1 on write error.
 */
int stdout_putchar (int ch) {
	uint8_t buf[1];

	buf[0] = ch;
	if (ptrUSART->Send(buf, 1) != ARM_DRIVER_OK) {
		return (-1);
	}
	while (ptrUSART->GetTxCount() != 1);
	return (ch);
}

/**
  Get a character from stdin

  \return     The next character from the input, or -1 on read error.
 */
int stdin_getchar (void) {
	uint8_t buf[1];

	if (ptrUSART->Receive(buf, 1) != ARM_DRIVER_OK) {
		return (-1);
	}
	while (ptrUSART->GetRxCount() != 1);
	return (buf[0]);
}

int stderr_putchar (int ch){
	return stdout_putchar(ch);
}

int helperBufferToInt(uint8_t* buffer, int lenght){
	return ( (buffer[0]<<8) + buffer[1]);
}

void helperIntToBuffer(uint8_t* buffer, int input){
	buffer[0] = (uint8_t)(input>>24);
	buffer[1] = (uint8_t)(input>>16);
	buffer[2] = (uint8_t)(input>>8);
	buffer[3] = (uint8_t)(input);
}

void helperShortToBuffer(uint8_t* buffer, uint16_t input){
	buffer[0] = (uint8_t)(input>>8);
	buffer[1] = (uint8_t)(input);
}




