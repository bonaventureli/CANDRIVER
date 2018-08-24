
/******************************************************************************
* file Name : rscan.c
* version   : 0.3
* Argument  : andy
* data  : 2018/8/24
* describe: 
*PIN Define:
	CAN1	CAN0_TX 		P0_0  2nd Alternative
	CAN1 	CAN0_RX 		P0_1  2nd Alternative
   
	CAN2 	CAN1_TX     		P10_7 4th Alternative
	CAN2	CAN1_RX			P10_6 4th Alternative
	Baud Rate: 500Kbps

	CAN0
	0010 2th output P0_0   CAN0TX
	0011 2th input  P0_1   CAN0RX
	0011 2th input  P10_0  CAN0RX
	0010 2th output P10_1  CAN0TX

	CAN1
	0110 4th output P10_7  CAN1TX
	0111 4th input  P10_6  CAN1RX

	CAN2
	0000 1st output P0_4   CAN2TX
	0001 1st input  P0_5   CAN2RX



	PFCAEn_m 	PFCEn_m 	PFCn_m 	PMn_m 	Function
	0 		0 		0 	1 	1st alternative function / Input
	0 		0 		0 	0 	1st alternative function / Output
	0 		0 		1 	1 	2nd alternative function / Input
	0 		0 		1 	0 	2nd alternative function / Output
	0 		1 		0 	1 	3rd alternative function / Input
	0 		1 		0 	0 	3rd alternative function / Output
	0 		1 		1 	1 	4th alternative function / Input
	0 		1 		1 	0 	4th alternative function / Output
	1 		0 		0 	1 	5th alternative function / Input
	1 		0 		0 	0 	5th alternative function / Output
	1 		0 		1 	X 	Setting prohibited
	1 		1 		X 	X 	Setting prohibited
	
******************************************************************************/
#include  	"r_typedefs.h"
#include  	"iodefine.h"
#include 	"rscan.h"
#include 	"can_table.h"
#define CANCLOCK 0
/******************************************************************************
* Function Name : void CanPinConfig( void )
* Description   : This function config the CAN Pin.
* Argument      : none
* Return Value  : none
******************************************************************************/
#define CAN0 1
void CanPinConfig(void)
{
#if CAN0
	/*CAN0 Set CAN0TX as P0_0 and CAN0RX as P0_1 / 
	0010 2th output P0_0   CAN0TX
	0011 2th input  P0_1   CAN0RX
	*/

	/*PFCEn — Port Function Control Expansion Register*/
	PFCE0&= ~(0x03<<0);
	/*PFCn / JPFC0 — Port Function Control Register*/
	PFC0 |= (0x03<<0); 
	/*PMn / APMn / JPM0 — Port Mode Register
	 * 0: Output mode (output enabled)
	 * 1: Input mode (output disabled)*/
	PM0 &= ~(1<<0);                                                       
	PM0 |= 1<<1;

	/*	PMCn / JPMC0 — Port Mode Control Register
	 *	1: Alternative mode
	 *	0: Port mode*/
	PMC0 |= (0x03<<0);
#else 
	/*CAN0 Set CAN0TX as P10_1 and CAN0RX as P10_0 / 
	0011 2th input  P10_0  CAN0RX
	0010 2th output P10_1  CAN0TX
	*/

	/*PFCEn — Port Function Control Expansion Register*/
	PFCE10&= ~(0x03<<0);
	/*PFCn / JPFC0 — Port Function Control Register*/
	PFC10 |= (0x03<<0); 
	/*PMn / APMn / JPM0 — Port Mode Register
	 * 0: Output mode (output enabled)
	 * 1: Input mode (output disabled)*/
	PM10 |= (1<<0);                                                       
	PM10 &= ~(1<<1);
	/*	PMCn / JPMC0 — Port Mode Control Register
	 *	1: Alternative mode
	 *	0: Port mode*/
	PMC10 |= (0x03<<0);
	
#endif
	/* CAN1 Set CAN1TX as P10_7 and CAN1RX as P10_6    
	 *0110 4th output P10_7  CAN1TX
	 *0111 4th input  P10_6  CAN1RX */

	/*PFCEn — Port Function Control Expansion Register*/
	PFCE10 |= (0x03<<6);
	/*PFCn / JPFC0 — Port Function Control Register*/
	PFC10  |= (0x03<<6);                                                      
	/*PMn / APMn / JPM0 — Port Mode Register
	 * 0: Output mode (output enabled)
	 * 1: Input mode (output disabled)*/
	PM10 &= ~(1<<7);                                                       
	PM10 |= 1<<6; 
	/*	PMCn / JPMC0 — Port Mode Control Register
	 *	1: Alternative mode
	 *	0: Port mode*/
	PMC10  |= (0x03<<6);



	/* CAN2 Set CAN2TX as P0_4 and CAN2RX as P0_5 
	 * 0000 1st output P0_4   CAN2TX
         * 0001 1st input  P0_5   CAN2RX */
	
	/*PFCEn — Port Function Control Expansion Register*/
	PFCE0&= ~(0x00003<<4);
	/*PFCn / JPFC0 — Port Function Control Register*/
	PFC0 &= ~(0x00003<<4);                                                      
	/*PMn / APMn / JPM0 — Port Mode Register
	 * 0: Output mode (output enabled)
	 * 1: Input mode (output disabled)*/
	PM0 &= ~(1<<4);                                                       
	PM0 |= 1<<5;
	/*	PMCn / JPMC0 — Port Mode Control Register
	 *	1: Alternative mode
	 *	0: Port mode*/
	PMC0 |= 0x00003<<4;
}
/*****************************************************************************
** Function:    RS_CAN_init
** Description: Configures the CAN0/CAN1 macros
** Parameter:   None
** Return:      None
******************************************************************************/
void RS_CAN_init(void)
{
#if CANCLOCK
    uint32_t    reg32_value;
    /* Source Clock Setting for   C_ISO_CAN
    CKSC_ICANS_CTL	 -   C_ISO_CAN   Source Clock Selection Register
    b31:b 2                        - Reserved set to 0
    b 1:b 0              ICANSCSID - Source Clock Setting for  C_ISO_CAN  - MainOSC X */
    do
    {
        reg32_value                = 0x00000001UL;
        PROTCMD1                   = 0x000000A5UL;   /* Protection release the CKSC_ILINS_CTL register */
        CKSC_ILINS_CTL             = reg32_value;
        CKSC_ILINS_CTL             = ~reg32_value;
        CKSC_ILINS_CTL             = reg32_value;
    } while ( PROTS1 != 0x00000000UL );
    while ( CKSC_ICANS_CTL != reg32_value )
    {
        /* Waiting for CKSC_ILINS_CTL to set. */
    }
#else
    /* Main Osc -> CAN */
    protected_write(PROTCMD1,CKSC_ICANOSCD_CTL,0x01)
    while (PROTS1 ==0x01);
#endif 
    /*	PMCn / JPMC0 — Port Mode Control Register
     *	1: Alternative mode
     *	0: Port mode
     *	Configure CAN0 GPIO-EnablePin P1_1 */
    PMC1 &= ~(1<<1);
    /*PMn / APMn / JPM0 — Port Mode Register
     * 0: Output mode (output enabled)
     * 1: Input mode (output disabled)*/
    PM1 &= ~(1<<1);
    /*Pn / APn / JP0 — Port Register
     * 0: Outputs low level
     * 1: Outputs high level
     * First disabled */
    P1 &= ~(1<<1);
    
    /* Configure CAN1 GPIO-EnablePin P2_6 */ 
    PMC2 &= ~(1<<6);
    PM2 &= ~(1<<6);
    P2 &= ~(1<<6);    //First disabled
    
    CanPinConfig();
        
    /*RSCAN0GSTS — Global Status Register
     * bit3	-GRAMINIT CAN RAM Initialization Status Flag
     * 0: CAN RAM initialization is completed.
     * 1: CAN RAM initialization is ongoing.*/
    while((RSCAN0.GSTS.UINT32 & 0x00000008)) ;

    /*RSCAN0GCTR — Global Control Register
     *bit2	-GSLPR Global Stop Mode
     *0: Other than global stop mode
     *1: Global stop mode
     *Switch to global/channel reset mode
     * */
    RSCAN0.GCTR.UINT32 &= ~(0x01<<2);
    /*RSCAN0CmCTR — Channel Control Register (m = 0 to 5)
     *bit2 CSLPR Channel Stop Mode
     *0: Other than channel stop mode
     *1: Channel stop mode
     * */
    RSCAN0.C0CTR.UINT32 &= ~(0x01<<2);
    RSCAN0.C1CTR.UINT32 &= ~(0x01<<2);
    RSCAN0.C2CTR.UINT32 &= ~(0x01<<2);

    /*RSCAN0GCFG — Global Configuration Register
     *bit4	DCS CAN Clock Source Select*2
     *0: clkc
     *1: clk_xincan
     *Configure clk_xincan
     * */
    RSCAN0.GCFG.UINT32 |= (0x01<<4);
    
    /* When fCAN is 16MHz, 
    Bitrate = fCAN/(BRP+1)/(1+TSEG1+TSEG2) = 16/2/16 = 0.5Mbps(500Kbps) */
    /*RSCAN0CmCFG — Channel Configuration Register (m = 0 to 5)
     *bit25-bit24	-SJW[1:0] Resynchronization Jump Width Control
     *1 0: 3 Tq
     *bit22-bit20	-TSEG2[2:0] Time Segment 2 Control
     *0 1 1: 4 Tq
     *bit19-bit16	-TSEG1[3:0] Time Segment 1 Control
     *1 0 1 0: 11 Tq
     *bit9-bit0		-BRP[9:0] Prescaler Division Ratio Set
     * */

    RSCAN0.C0CFG.UINT32 &= ~(0x03<<24);
    RSCAN0.C0CFG.UINT32 |= (0x02<<24);


    RSCAN0.C0CFG.UINT32 &= ~(0x01<<24);
    RSCAN0.C0CFG.UINT32 &= ~(0x01<<22);
    RSCAN0.C0CFG.UINT32 |= (0x03<<19);
    RSCAN0.C0CFG.UINT32 &= ~(0x01<<18);
    RSCAN0.C0CFG.UINT32 |= (0x03<<17);
    RSCAN0.C0CFG.UINT32 &= ~(0x01<<16);
    RSCAN0.C0CFG.UINT32 |= (0x01<<0);

    RSCAN0.C0CFG.UINT32 = 0x023a0001; //SJW =3TQ, TSEG1=11TQ, TSEG2=4TQ, BRP=1

    RSCAN0.C1CFG.UINT32 = 0x023a0001; //SJW =3TQ, TSEG1=11TQ, TSEG2=4TQ, BRP=1

    RSCAN0.C2CFG.UINT32 = 0x023a0001; //SJW =3TQ, TSEG1=11TQ, TSEG2=4TQ, BRP=1
		
    /* ==== Rx rule setting ==== */
    Can_SetRxRule();

    /* ==== buffer setting ==== */    
    //RSCAN0RMNB = 0x18;  //Can_SetGlobalBuffer--24
    RSCAN0.RMNB.UINT32 = 0x40;  //
    
    /*Receive FIFO buffer setting*/
    //RSCAN0.RFCC0.UINT32=0x0000F200; //8 buffer depth, FIFO 0 interrupt is disabled, RF0 enabled only in operation mode
    RSCAN0.RFCC0.UINT32=0x0000F202;	  //8 buffer depth, FIFO 0 interrupt at each reception is enabled
    RSCAN0.RFCC1.UINT32=0x0000F202;	  //8 buffer depth, FIFO 1 interrupt at each reception is enabled
    RSCAN0.RFCC2.UINT32=0x0000F202;	  //8 buffer depth, FIFO 1 interrupt at each reception is enabled
    
    /*TX RX FIFO buffer 0 setting, assigned to CH0*/
    //RSCAN0.CFCC9.UINT32=0xFF05F200;  //8 buffer depth, Tx Rx FIFO 9 interrupt is disabled, TX mode, link to buffer 0
    RSCAN0.CFCC0.UINT32=0xFF05F205;  //8 buffer depth, Tx Rx FIFO 9 interrupt is enable, TX mode, link to buffer 0
    //RSCAN0.CFCC9.UINT32=0xFF05F205;  //8 buffer depth, Tx Rx FIFO 9 interrupt is enable, TX mode, link to buffer 0

    /*TX RX FIFO buffer 3 setting, assigned to CH1*/
    RSCAN0.CFCC3.UINT32=0xFF06F200; //8 buffer depth, Tx Rx FIFO 12 interrupt is disabled, Gateway mode, link to buffer 0
	
		/*TX RX FIFO buffer 6 setting, assigned to CH2*/
    RSCAN0.CFCC6.UINT32=0xFF06F200; //8 buffer depth, Tx Rx FIFO 12 interrupt is disabled, Gateway mode, link to buffer 0
		
    /* Set THLEIE disabled, MEIE(FIFO Message Lost Interrupt disabled)  */
    RSCAN0.GCTR.UINT32 &= 0xfffff8ff;    

    /* If GlobalChannel in halt or reset mode */
    if (RSCAN0.GSTS.UINT32 & 0x03) 
    {
        RSCAN0.GCTR.UINT32 &= 0xfffffffc; //Switch to communication mode
        while ((RSCAN0.GSTS.UINT32 & 0x02) == 2) {
            /* While halt mode */
        }
        while ((RSCAN0.GSTS.UINT32 & 0x01) == 1) {
            /* While reset mode */
        }
    }

    /* If Channel 0 in halt or reset mode */
    if (RSCAN0.C0STS.UINT32 & 0x03) 
    {
        RSCAN0.C0CTR.UINT32 &= 0xfffffffc;    //Switch to communication mode
        while ((RSCAN0.C0STS.UINT32 & 0x02) == 2) {
            /* While halt mode */
        }
        while ((RSCAN0.C0STS.UINT32 & 0x01) == 1) {
            /* While reset mode */
        }
    }
    
    /* Enable CAN0 by setting P1_1 high */
    P1 |= 1<<1;    
     
    /* If Channel 1 in halt or reset mode */
    if (RSCAN0.C1STS.UINT32 & 0x03) 
    {
        RSCAN0.C1CTR.UINT32 &= 0xfffffffc;    //Switch to communication mode
        while ((RSCAN0.C1STS.UINT32 & 0x02) == 2) {
            /* While halt mode */
        }
        while ((RSCAN0.C1STS.UINT32 & 0x01) == 1) {
            /* While reset mode */
        }
    }
    
        /* If Channel 1 in halt or reset mode */
    if (RSCAN0.C2STS.UINT32 & 0x03) 
    {
        RSCAN0.C2CTR.UINT32 &= 0xfffffffc;    //Switch to communication mode
        while ((RSCAN0.C2STS.UINT32 & 0x02) == 2) {
            /* While halt mode */
        }
        while ((RSCAN0.C2STS.UINT32 & 0x01) == 1) {
            /* While reset mode */
        }
    }
    
    /* Enable CAN1 by setting P2_6 high */
    P2 |= 1<<6;

	RSCAN0.RFCC0.UINT32 |=0x01; //set RX FIFO on
	RSCAN0.RFCC1.UINT32 |=0x01; //set RX FIFO on
	RSCAN0.RFCC2.UINT32 |=0x01; //set RX FIFO on
	RSCAN0.CFCC0.UINT32 |=0x01; //set TX RX FIFO 0 on
	RSCAN0.CFCC3.UINT32 |=0x01; //set TX RX FIFO 3 on
	RSCAN0.CFCC6.UINT32 |=0x01; //set TX RX FIFO 6 on
	
	#if 0
	TBRCANGRECC = 1;          //Table interrupt is enable.
	MKRCANGRECC = 0;          //Enable the interrupt processing.
	#else
	TBRCANGRECC = 0;          //Table interrupt is enable.
	MKRCANGRECC = 0;          //Enable the interrupt processing.
	RFRCANGRECC = 0;          //Enable the interrupt processing.
	#endif

	TBRCAN0TRX = 1;          //Table interrupt is enable.
	MKRCAN0TRX = 0;          //Enable the interrupt processing.

	TBRCAN1TRX = 1;          //Table interrupt is enable.
	MKRCAN1TRX = 0;          //Enable the interrupt processing.

	TBRCAN2TRX = 1;          //Table interrupt is enable.
	MKRCAN2TRX = 0;          //Enable the interrupt processing.

}

/******************************************************************************
** Function:    Can_SetRxRule
** Description: Set all Rx rules
** Parameter:   None
** Return:      None
******************************************************************************/
static void Can_SetRxRule(void)
{
    U16 RxRuleIdx;
    U8 PageRxRuleIdx;
    volatile CAN_RX_RULE_TYPE* pCRE;

    /* Set Rx rule number per channel */
    RSCAN0.GAFLCFG0.UINT32 |= 0x10080800;   //Channel 0 rule number is 16 Channel 1 rule number is 8

    /* Get access base address of Rx rule */
    pCRE = (volatile CAN_RX_RULE_TYPE*)&(RSCAN0.GAFLID0.UINT32);

    /* Receive Rule Table Write Enable */
    RSCAN0.GAFLECTR.UINT32 |= 0x00000100;	//set bit8 to 1, Receive rule table write is enabled

    /* Copy Rx rule one by one */
    for (RxRuleIdx = 0U; RxRuleIdx < CAN_RX_RULE_NUM; RxRuleIdx++) //CAN_RX_RULE_NUM=12, refer to cab_table.h
    //for (RxRuleIdx = 0U; RxRuleIdx < 16; RxRuleIdx++)	//if it is more than 16 rules, go to another page
    {
        PageRxRuleIdx = (U8) (RxRuleIdx & CAN_PAGE_RX_RULE_IDX_MASK); //CAN_PAGE_RX_RULE_IDX_MASK= 0xF

        /* Update target Rx rule page if necessary. */
        if (PageRxRuleIdx == 0U) //RxRuleIdx=0, page=0; RxRuleIdx=16, page=1; RxRuleIdx=32,page=2;...
        {
            RSCAN0.GAFLECTR.UINT32 |= RxRuleIdx >> CAN_RX_RULE_PAGE_IDX_BIT_POS; //CAN_RX_RULE_PAGE_IDX_BIT_POS= 4U
        }

        /* Set a single Rx rule.*/
        pCRE[PageRxRuleIdx] = CAN_RX_RULE_TABLE[RxRuleIdx];
    }

    /* Rx rule write disable */
    RSCAN0.GAFLECTR.UINT32 &= 0xfffffeff;
}

/*****************************************************************************
** Function:    Can_ReadRx_buffer
** Description: This code shows how to read message from Rx buffer
** Parameter:   pRxBufIdx - Pointer to Rx buffer that receives frame
** Return:      CAN_RTN_OK           - A frame is successfully read out
**              CAN_RTN_BUFFER_EMPTY - No frame is read out   
******************************************************************************/
Can_RtnType Can_ReadRxBuffer(Can_FrameType* pFrame)
{
    U8 BufIdx;
    U8 CRBRCFiBufIdx;
    U8 OverwrittenFlag;
    U32 TempCRBRCF0;
    U32 TempCRBRCF1;
    U32 TempCRBRCF2;
    Can_FrameType* pRxBuffer;
    VU32* pCRBRCF;
    Can_RtnType RtnValue;

	//change to interrupt mode
	#if 0
    /*check data to RX FIFO buffer 0 */
    if((RSCAN0.RFSTS0.UINT32 & 0x00000001)==0x0)	//
    {
	pRxBuffer = (Can_FrameType*) &(RSCAN0.RFID0.UINT32);
	*pFrame = pRxBuffer[0];
	//BufIdx=0;
	RSCAN0.RFPCTR0.UINT32 |=0xFF;
    }
	#endif
	
    /* Judge if new messages to RX buffer are available */
    TempCRBRCF0 = RSCAN0.RMND0.UINT32;	//Receive Buffer New Data Register, if it is true, new data is coming
    TempCRBRCF1 = RSCAN0.RMND1.UINT32;
    TempCRBRCF2 = RSCAN0.RMND2.UINT32;
    if ((TempCRBRCF0 == CAN_CLR) && (TempCRBRCF1 == CAN_CLR)&& (TempCRBRCF2 == CAN_CLR)) //CAN_CLR==0
    {
        //RtnValue = CAN_RTN_BUFFER_EMPTY;	// buffer empty, no new data
    }
    else
    {
            /* Get Rx buffer that has new message */
            if (TempCRBRCF0 != CAN_CLR) 
            {
                pCRBRCF = &(RSCAN0.RMND0.UINT32);
                for (BufIdx = 0U; BufIdx < CAN_CRBRCF0_RX_BUF_NUM; ++BufIdx) //CAN_CRBRCF0_RX_BUF_NUM=32
                {
                    if ((TempCRBRCF0 & CAN_1_BIT_MASK) == CAN_SET) //CAN_1_BIT_MASK==0x1; CAN_SET=0x1
                    {
                        break;	//if checked bit is 1, that means there is a new message in corresponding receive buffer 
                    }
                    TempCRBRCF0 = TempCRBRCF0 >> CAN_B1_BIT_POS; //CAN_B1_BIT_POS=0x1
                }
            }
            else if (TempCRBRCF1 != CAN_CLR)
            {
                pCRBRCF = &(RSCAN0.RMND1.UINT32);
                for (BufIdx = 0U; BufIdx < CAN_CRBRCF1_RX_BUF_NUM; ++BufIdx) 
                {
                    if ((TempCRBRCF1 & CAN_1_BIT_MASK) == CAN_SET) 
                    {
                        break;
                    }
                    TempCRBRCF1 = TempCRBRCF1 >> CAN_B1_BIT_POS;
                }
                BufIdx += CAN_CRBRCF0_RX_BUF_NUM;	//CAN_CRBRCF0_RX_BUF_NUM ==32U
            }
            else if (TempCRBRCF2 != CAN_CLR)
            {
                pCRBRCF = &(RSCAN0.RMND2.UINT32);
                for (BufIdx = 0U; BufIdx < CAN_CRBRCF2_RX_BUF_NUM; ++BufIdx) {
                    if ((TempCRBRCF2 & CAN_1_BIT_MASK) == CAN_SET) {
                        break;
                    }
                    TempCRBRCF2 = TempCRBRCF2 >> CAN_B1_BIT_POS;
                }
                //BufIdx += CAN_CRBRCF1_RX_BUF_NUM;	//??? mabye +=64U
		BufIdx += (CAN_CRBRCF0_RX_BUF_NUM + CAN_CRBRCF1_RX_BUF_NUM);	// +=64U
            }
            /* Calculate index value in CRBRCFi */
            CRBRCFiBufIdx = BufIdx & CAN_5_BIT_MASK;	//CAN_5_BIT_MASK  0x1fU  0B11111

            do 
            {
                /* Clear Rx complete flag of corresponding Rx buffer */
                do 
                {
                    CLR_BIT(*pCRBRCF, CRBRCFiBufIdx);	//To clear a flag to 0, the program must write 0 to the flag
                } while (GET_BIT(*pCRBRCF, CRBRCFiBufIdx) == CAN_SET);

                /* Read out message from Rx buffer */
                pRxBuffer = (Can_FrameType*) &(RSCAN0.RMID0.UINT32);
                *pFrame = pRxBuffer[BufIdx];

                /* Judge if current message is overwritten */
                OverwrittenFlag = GET_BIT(*pCRBRCF, CRBRCFiBufIdx);
                /* message is overwritten */
                if (OverwrittenFlag == CAN_SET) 
                {
                    /* User process for message overwritten */
                    //Usr_HandleRxBufOverwritten(BufIdx);
                }
            } while (OverwrittenFlag == CAN_SET);

            RtnValue = CAN_RTN_OK;
    }

    return RtnValue;
}



Can_RtnType Can_C0TrmByTxBuf(U8 TxBufIdx, const Can_FrameType* pFrame)
{
    VU8* pTBSR;
    Can_FrameType* pTxBuffer;
    VU8* pTBCR;

    pTBSR = &(RSCAN0.TMSTS0);
    pTBCR = &(RSCAN0.TMC0);

    /* ---- Return if Tx Buffer is transmitting. ---- */    
    if( (pTBSR[TxBufIdx] & (VU8)0x01) == CAN_TBTST_TRANSMITTING )
    {
        return CAN_RTN_ERR;
    }

    /* Clear Tx buffer status */
    do 
    {
        pTBSR[TxBufIdx] = CAN_CLR;
    } while (pTBSR[TxBufIdx] != CAN_CLR);

    /* Store message to tx buffer */
    pTxBuffer = (Can_FrameType*) &(RSCAN0.TMID0);
    pTxBuffer[TxBufIdx] = *pFrame;

    /* Set transmission request */
    pTBCR[TxBufIdx] = CAN_TBCR_TRM;

    return CAN_RTN_OK;
}

Can_RtnType Can_TrmByTxRXFIFO_NO9(const Can_FrameType* pFrame)
{
    //VU8* pTBSR;
    Can_FrameType* pTxBuffer;
   // VU8* pTBCR;
    
    if((RSCAN0.CFSTS9.UINT32&0x02)==0x0)  //FIFO not full
    {
	/* Store message to tx buffer */
	pTxBuffer = (Can_FrameType*) &(RSCAN0.CFID9.UINT32);
	pTxBuffer[0] = *pFrame;
	RSCAN0.CFPCTR9.UINT32=0xFF;
    }
#if 0
    pTBSR = &(RSCAN0TMSTS0);
    pTBCR = &(RSCAN0TMC0);

    /* ---- Return if Tx Buffer is transmitting. ---- */    
    if( (pTBSR[TxBufIdx] & (VU8)0x01) == CAN_TBTST_TRANSMITTING )
    {
        return CAN_RTN_ERR;
    }

    /* Clear Tx buffer status */
    do 
    {
        pTBSR[TxBufIdx] = CAN_CLR;
    } while (pTBSR[TxBufIdx] != CAN_CLR);

    /* Store message to tx buffer */
    pTxBuffer = (Can_FrameType*) &(RSCAN0TMID0);
    pTxBuffer[TxBufIdx] = *pFrame;

    /* Set transmission request */
    pTBCR[TxBufIdx] = CAN_TBCR_TRM;
#endif
    return CAN_RTN_OK;
}

Can_RtnType Can_C1TrmByTxBuf(U8 TxBufIdx, const Can_FrameType* pFrame)
{
    VU8* pTBSR;
    Can_FrameType* pTxBuffer;
    VU8* pTBCR;

    pTBSR = &(RSCAN0.TMSTS16);
    pTBCR = &(RSCAN0.TMC16);

    /* ---- Return if Tx Buffer is transmitting. ---- */
    if( (pTBSR[TxBufIdx] & (VU8)0x01) == CAN_TBTST_TRANSMITTING )
    {
        return CAN_RTN_ERR;
    }

    /* Clear Tx buffer status */
    do 
    {
        pTBSR[TxBufIdx] = CAN_CLR;
    } while (pTBSR[TxBufIdx] != CAN_CLR);

    /* Store message to Tx buffer */
    pTxBuffer = (Can_FrameType*) &(RSCAN0.TMID16);
    pTxBuffer[TxBufIdx] = *pFrame;

    /* Set transmission request */
    pTBCR[TxBufIdx] = CAN_TBCR_TRM;

    return CAN_RTN_OK;
}

Can_RtnType Can_C2TrmByTxBuf(U8 TxBufIdx, const Can_FrameType* pFrame)
{
    VU8* pTBSR;
    Can_FrameType* pTxBuffer;
    VU8* pTBCR;

    pTBSR = &(RSCAN0.TMSTS32);
    pTBCR = &(RSCAN0.TMC32);

    /* ---- Return if Tx Buffer is transmitting. ---- */
    if( (pTBSR[TxBufIdx] & (VU8)0x01) == CAN_TBTST_TRANSMITTING )
    {
        return CAN_RTN_ERR;
    }

    /* Clear Tx buffer status */
    do 
    {
        pTBSR[TxBufIdx] = CAN_CLR;
    } while (pTBSR[TxBufIdx] != CAN_CLR);

    /* Store message to Tx buffer */
    pTxBuffer = (Can_FrameType*) &(RSCAN0.TMID32);
    pTxBuffer[TxBufIdx] = *pFrame;

    /* Set transmission request */
    pTBCR[TxBufIdx] = CAN_TBCR_TRM;

    return CAN_RTN_OK;
}

    /* Create Can_FrameType for send and receive data */
    const Can_FrameType CANTraStandData={
    //CiTBpA
    0x18,
    0,
    0,
    0,        
    
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
    
    Can_FrameType CANRecData={
    //CiTBpA
    0x00,
    0,
    0,
    0,
    
    //CiTBpB
    0x0000,                                
    0x000,                            
    0x0,                        

    //CiTBpC
    {
    0x00,                            //DB0
    0x00,                            //DB1
    0x00,                            //DB2
    0x00,                            //DB3
    //CiTBpD
    0x00,                            //DB4
    0x00,                            //DB5
    0x00,                            //DB6
    0x00                             //DB7
    }
    };


 /*****************************************************************************
** Function:    RS_CAN_error
** Description: This function sends/receives and compares data of the CAN-Channels
** Parameter:   None
** Return:      error 1
**              no error 0  
******************************************************************************/
int RS_CAN_error(void)
{
    int rs_count, error,i;

    if(Can_C0TrmByTxBuf(1,&CANTraStandData)== CAN_RTN_OK)
    {

        //Delay
        for(i=0;i<10000;i++);                 //Wait for CAN receive interrupt
                    
        Can_ReadRxBuffer(&CANRecData);        //Channel4 receive the Messages
    }
    
    /* Compare each sent and received value */
    error=0;
    for(rs_count=0; rs_count<8; rs_count++)
    {
        if(CANTraStandData.DB[rs_count] != CANRecData.DB[rs_count])
        {
            error=1;
        }
    }
    return error;
}

