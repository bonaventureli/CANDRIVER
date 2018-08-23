/*******************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only
* intended for use with Renesas products. No other uses are authorized. This
* software is owned by Renesas Electronics Corporation and is protected under
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
* AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software
* and to discontinue the availability of this software. By using this software,
* you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
* (c) 2014 Renesas Electronics Corporation All rights reserved.
*******************************************************************************/


/******************************************************************************
* File Name     : r_interrupt.c
* Version       : 1.0
* Device(s)     : R7F701035xAFP RH850/F1L
* Tool-Chain    : CubeSuite+(V2.01.00)
* Description   : This file is a sample of the interrupt.
* Operation     : -
*******************************************************************************
*******************************************************************************
* History       : DD.MM.YYYY Version Description
*               : 20.03.2014 1.00    First Release
******************************************************************************/
/******************************************************************************
Includes <System Includes> , ÅgProject IncludesÅh
******************************************************************************/

#pragma interrupt priority7_interrupt( enable=false , priority=EIINT_PRIORITY7 , callt=false , fpu=false )
#pragma interrupt FEINT_interrupt(priority=feint, callt=false, fpu=false)
#pragma interrupt INTOSTM0_interrupt( enable = false, channel = 76, callt = false, fpu = false )
//#pragma interrupt INTRCANGRECC_interrupt( enable = false, channel = 15, callt = false, fpu = false )
#pragma interrupt INTRCAN3TRX_interrupt( enable = false, channel = 214, callt = false, fpu = false )

#include    "r_typedefs.h"
#include    "iodefine.h"
#include    "rscan.h"

/******************************************************************************
Imported global variables and functions (from other files)
******************************************************************************/
extern uint8_t FlagNewData0;
extern uint8_t FlagNewData1;
extern uint8_t FlagNewData2;

/******************************************************************************
Macro definitions
******************************************************************************/


/******************************************************************************
Exported global variables and functions (to be accessed by other files)
******************************************************************************/
        void    priority7_interrupt( uint32_t );
		void 	FEINT_interrupt( unsigned long );
		void    INTOSTM0_interrupt(void);
		void 	INTRCANGRECC_interrupt( void );
		void 	INTRCAN3TRX_interrupt( void );
		
/******************************************************************************
Private global variables and functions
******************************************************************************/


/******************************************************************************
* Function Name : void priority7_interrupt( uint32_t )
* Description   : This function is priority (7) interrupt processing. (direct vector)
* Argument      : regEIIC_value     : Interrupt factor
* Return Value  : none
******************************************************************************/

extern Can_FrameType CAN_Trans_Data;

void priority7_interrupt( uint32_t regEIIC_value )
{
    switch ( regEIIC_value )
    {
        case 0x0000104CUL: /* OSTM0INT */
	    	P10 ^= (1<<3); 	//toggle LED1
		//P8 ^= (1<<5); 	//toggle LED2		
        break;
				
	case 0x0000100FUL: /* CANGRCC */
		INTRCANGRECC_interrupt();
        break;
				

        default:
        break;
    }
}

void FEINT_interrupt( unsigned long feic )
{
    if((FEINTF&0x4000)==0x4000)
    {
		FEINTFC |= 0x00004000;

		P10 ^= (1<<3); 	//toggle LED1
		P8 ^= (1<<5); 	//toggle LED2
		
		//Can_C3TrmByTxBuf(1,&CANTraStandData);
		__nop();

    }
}

/******************************************************************************
* Function Name : void INTRCANGRECC_interrupt( void )
* Description   : This function is INTRCANGRECC interrupt processing. (Table Reference)
* Argument      : none
* Return Value  : none
******************************************************************************/
void INTRCANGRECC_interrupt( void )
{
	Can_FrameType* pRxBuffer;
	RSCAN0.RFSTS0.UINT8[LL] &= 0x07;	//clear RFIF Receive FIFO Interrupt Request Flag
	while(RSCAN0.RFSTS0.UINT8[LH]  != 0)  // The receive FIFO buffer contains unread message.
	{
		FlagNewData0 = 1;	
		/* Read out message from Rx buffer */
		pRxBuffer = (Can_FrameType*) &(RSCAN0.RFID0);
		CANRecData = pRxBuffer[0];	//RF0
		RSCAN0.RFPCTR0.UINT8[LL] = 0xff;
	}
	RSCAN0.RFSTS1.UINT8[LL] &= 0x07;	//clear RFIF Receive FIFO Interrupt Request Flag
	while(RSCAN0.RFSTS1.UINT8[LH]  != 0)  // The receive FIFO buffer contains unread message.
	{
		FlagNewData1 = 1;	
		/* Read out message from Rx buffer */
		pRxBuffer = (Can_FrameType*) &(RSCAN0.RFID1);
		CANRecData = pRxBuffer[0];	//RF0
		RSCAN0.RFPCTR1.UINT8[LL] = 0xff;
	}
	RSCAN0.RFSTS2.UINT8[LL] &= 0x07;	//clear RFIF Receive FIFO Interrupt Request Flag
	while(RSCAN0.RFSTS2.UINT8[LH]  != 0)  // The receive FIFO buffer contains unread message.
	{
		FlagNewData2 = 1;	
		/* Read out message from Rx buffer */
		pRxBuffer = (Can_FrameType*) &(RSCAN0.RFID2);
		CANRecData = pRxBuffer[0];	//RF0
		RSCAN0.RFPCTR2.UINT8[LL] = 0xff;
	}
}
/******************************************************************************
* Function Name : void INTRCAN3TRX_interrupt( void )
* Description   : This function is INTRCAN3REC interrupt processing. (Table Reference)
* Argument      : none
* Return Value  : none
******************************************************************************/
void INTRCAN3TRX_interrupt( void )
{
	Can_FrameType* pTxBuffer;
	RSCAN0.CFSTS9.UINT8[LH] &= (~0x01);	//clear RFIF transmit FIFO Interrupt Request Flag
	if((RSCAN0.CFSTS9.UINT32&0x02)==0x0)  //FIFO not full
    {
	/* Store message to tx buffer */
	pTxBuffer = (Can_FrameType*) &(RSCAN0.CFID9.UINT32);
	pTxBuffer[0] = CAN_Trans_Data;
	RSCAN0.CFPCTR9.UINT32=0xFF;
    }
	
}

void INTOSTM0_interrupt( void )
{
	
		P10 ^= (1<<3); 	//toggle LED1
		P8 ^= (1<<5); 	//toggle LED2
	
}