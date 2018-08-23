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
* File Name     : r_main.c
* Version       : 1.0
* Device(s)     : R7F701035xAFP RH850/F1L
* Tool-Chain    : CubeSuite+(V2.01.00)
* Description   : This file is a sample of the peripheral function.
* Operation     : 1. Compile and download the sample code. Click 'Reset Go'
*               :    to start the software.
*******************************************************************************
*******************************************************************************
* History       : DD.MM.YYYY Version Description
*               : 20.03.2014 1.00    First Release
******************************************************************************/

/******************************************************************************
Includes <System Includes> , ?gProject Includes?h
******************************************************************************/
#include    "r_typedefs.h"
#include    "iodefine.h"
#include 	"rscan.h"
#include 	"can_table.h"
//#include 	"ostm0.h"

/******************************************************************************
Imported global variables and functions (from other files)
******************************************************************************/
extern  void        R_CLOCK_Init( void );

uint8_t FlagNewData0 = 0;
uint8_t FlagNewData1 = 0;
uint8_t FlagNewData2 = 0;
/******************************************************************************
Macro definitions
******************************************************************************/


/******************************************************************************
Exported global variables and functions (to be accessed by other files)
******************************************************************************/


/******************************************************************************
Private global variables and functions
******************************************************************************/
void        main( void );

/* Create Can_FrameType for send and receive data */
Can_FrameType CAN_Trans_Data={
  //CiTBpA
  0x111,
  0,
  0,
  1,        //ext ID

  //CiTBpB
  0x0000,                            
  0x000,                            
  0x8,    

  {
    0x12,                            //DB0
    0x34,                            //DB1
    0x56,                            //DB2
    0x78,                            //DB3
    //CiTBpD
    0x87,                            //DB4
    0x65,                            //DB5
    0x43,                            //DB6
    0x21                             //DB7
  }
};
Can_FrameType CAN_Trans_Data2={
  //CiTBpA
  0x222,
  0,
  0,
  1,        //ext ID

  //CiTBpB
  0x0000,                            
  0x000,                            
  0x8,    

  {
    0x12,                            //DB0
    0x34,                            //DB1
    0x56,                            //DB2
    0x78,                            //DB3
    //CiTBpD
    0x87,                            //DB4
    0x65,                            //DB5
    0x43,                            //DB6
    0x21                             //DB7
  }
};

/******************************************************************************
* Function Name : void main( void )
* Description   : This function is sample main processing.
* Argument      : none
* Return Value  : none
******************************************************************************/
void main( void )
{
  R_CLOCK_Init();                       /* Clock initialize    */
	RS_CAN_init();                        /* RS-CAN initialize   */
	
	__EI();
	
	Can_C0TrmByTxBuf(2,&CAN_Trans_Data);
	Can_C1TrmByTxBuf(4,&CAN_Trans_Data);
	Can_C2TrmByTxBuf(3,&CAN_Trans_Data2);
  
}


