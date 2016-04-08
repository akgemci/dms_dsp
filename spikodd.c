
#include "DSP2833x_Device.h"     // Device Headerfile and Examples Include File

extern void InitPieCtrl(void);
extern void InitSysCtrl(void);
extern void InitPieVectTable(void);
extern void InitAdc(void);
extern void InitSpiaGpio(void);
__interrupt void spiTxFifoIsr(void);
__interrupt void spiRxFifoIsr(void);
interrupt void adc1_isr(void);
interrupt void ePWM1A_compare_isr(void);
void Setup_ePWM(void);
void Setup_ADC(void);
void spi_fifo_init(void);
void Gpio_select(void);

float adc_chan_0;
float rdata[16];

float flag1 = 0;
float flag2 = 0;

float a[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
float b[] = {17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32};

void main(void)
{
	InitSysCtrl();
	InitSpiaGpio();
	Gpio_select();
	/*
	EALLOW;
	SysCtrlRegs.WDCR = 0x00AF; // re-enable the watchdog
	EDIS;
	 */
	DINT;
	IER = 0x0000;
	IFR = 0x0000;
	InitPieCtrl();
	InitPieVectTable();
	InitAdc();
	spi_fifo_init();
	Setup_ePWM();		// ePWM
	Setup_ADC();		// ADC setup
	EALLOW;	// This is needed to write to EALLOW protected registers
	PieVectTable.SPIRXINTA = &spiRxFifoIsr;
	PieVectTable.SPITXINTA = &spiTxFifoIsr;
	PieVectTable.SEQ1INT = &adc1_isr;
	PieVectTable.EPWM1_INT = &ePWM1A_compare_isr;
	EDIS;   // This is needed to disable write to EALLOW protected registers
	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
	PieCtrlRegs.PIEIER6.bit.INTx1 = 1;     // Enable PIE Group 6, INT 1
	PieCtrlRegs.PIEIER6.bit.INTx2 = 1;     // Enable PIE Group 6, INT 2
	PieCtrlRegs.PIEIER1.bit.INTx1 = 1;		 // adc1 (seq1 - pwm)
	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;		 // epwm1
	IER |= 25;                            // Enable CPU INT6
	EINT;                                // Enable Global Interrupts
	ERTM;
	while (1)
	{
		if (AdcRegs.ADCST.bit.INT_SEQ1 == 1) // ADC seq1 interrupt for pwm (at prd)
		{


			flag1 = 1;
			AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;

		}
		if (EPwm1Regs.ETFLG.bit.INT == 1)	// epwm interrupt for pwm of second converter (at prd)
		{


			flag2 = 1;
			EPwm1Regs.ETCLR.bit.INT = 1;
		}
	}
}


void Gpio_select(void)
{
	EALLOW;
	/*

	GpioCtrlRegs.GPAMUX1.all = 0;		// GPIO15 ... GPIO00 = General Purpose I/O
	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1; // activate epwm1a
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1; // activate epwm1b
	GpioCtrlRegs.GPAMUX2.all = 0;		// GPIO31 ... GPIO16 = General Purpose I/O
	GpioCtrlRegs.GPBMUX1.all = 0;		// GPIO47 ... GPIO32 = General Purpose I/O
	GpioCtrlRegs.GPBMUX2.all = 0;		// GPIO63 ... GPIO48 = General Purpose I/O
	GpioCtrlRegs.GPCMUX1.all = 0;		// GPIO79 ... GPIO64 = General Purpose I/O
	GpioCtrlRegs.GPCMUX2.all = 0;		// GPIO87 ... GPIO80 = General Purpose I/O
	GpioCtrlRegs.GPADIR.all = 0;
	GpioCtrlRegs.GPBDIR.all = 0;		// GPIO63-32 as inputs
	GpioCtrlRegs.GPCDIR.all = 0;		// GPIO87-64 as inputs
	EDIS;
	 */
}


void Setup_ePWM(void)
{
	EPwm1Regs.TBCTL.bit.CLKDIV = 0;		// CLKDIV = 1
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 1;	// HSCLKDIV = 2
	EPwm1Regs.TBCTL.bit.CTRMODE = 2; 	// up down mode
	EPwm1Regs.TBCTL.bit.SYNCOSEL = 1; 	// syncout if ctr = 0
	EPwm1Regs.AQCTLA.all = 0x00060; 	// zero = set; period = clear
	EPwm1Regs.TBPRD = 3750;			    // 1 kHz PWM signal (37500)
	EPwm1Regs.CMPA.half.CMPA = 0;		// 100% duty cycle first
	EPwm1Regs.ETPS.all = 0x0000;		// Configure ADC start by ePWM1
	EPwm1Regs.ETPS.bit.SOCAPRD = 1;		// 01 = generate SOCA on first event
	EPwm1Regs.ETPS.bit.INTPRD = 1;		// 01 = interrupt on first event
	EPwm1Regs.ETSEL.all = 0x0000;		// Enable SOCA to ADC
	EPwm1Regs.ETSEL.bit.SOCAEN = 1;		// 1 = enable SOCA
	EPwm1Regs.ETSEL.bit.SOCASEL = 2;	// 010 = SOCA on PRD event
	EPwm1Regs.ETSEL.bit.INTEN = 1;		// Enable EPWM1INT
	EPwm1Regs.ETSEL.bit.INTSEL = 7;		// 001 = Interrupt on Zero Event
}

void Setup_ADC(void)
{
	AdcRegs.ADCREFSEL.bit.REF_SEL = 0;
	AdcRegs.ADCTRL1.all = 0;
	AdcRegs.ADCTRL1.bit.SEQ_CASC = 0;		// dual sequentier mode
	AdcRegs.ADCTRL1.bit.CONT_RUN = 0;		// single run mode
	AdcRegs.ADCTRL1.bit.ACQ_PS = 7;			// 8 X ADC clock
	AdcRegs.ADCTRL1.bit.CPS = 0;			// divide by 1
	AdcRegs.ADCTRL2.all = 0;

	// clear SOC flags
	AdcRegs.ADCTRL2.bit.SOC_SEQ1=0;
	AdcRegs.ADCTRL2.bit.SOC_SEQ2=0;

	AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;	// ePWM_SOCA trigger (start of sequence)
	AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;	// enable ADC int for seq1
	AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1 = 0;	// interrupt after every EOS (end of sequence)
	AdcRegs.ADCTRL3.bit.ADCCLKPS = 3;		// set FCLK to 12.5 MHz

	AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 0;
	AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0;  // ADCINA0 - Vs

}

void spi_fifo_init()
{
	// Initialize SPI FIFO registers
	SpiaRegs.SPICCR.bit.SPISWRESET=0; // Reset SPI

	SpiaRegs.SPICCR.all=0x001F;       //16-bit character, no loopback mode
	SpiaRegs.SPICTL.all=0x0017;       //Interrupt enabled, Master/Slave XMIT enabled
	SpiaRegs.SPISTS.all=0x0000;
	SpiaRegs.SPICTL.bit.SPIINTENA = 1;       //Interrupt enabled, Master/Slave XMIT enabled
	SpiaRegs.SPIFFTX.bit.TXFFIENA = 1;

	//SpiaRegs.SPIBRR=0x0063;           // Baud rate
	SpiaRegs.SPIBRR=0x005;           // Baud rate
	SpiaRegs.SPIFFTX.all=0xC028;      // Enable FIFO's, set TX FIFO level to 8
	SpiaRegs.SPIFFRX.all=0x0028;      // Set RX FIFO level to 8
	SpiaRegs.SPIFFCT.all=0x00;
	SpiaRegs.SPIPRI.all=0x0010;

	SpiaRegs.SPICCR.bit.SPISWRESET=1;  // Enable SPI

	SpiaRegs.SPIFFTX.bit.TXFIFO=1;
	SpiaRegs.SPIFFRX.bit.RXFIFORESET=1;
}

interrupt void adc1_isr(void)
{
	adc_chan_0 = AdcMirror.ADCRESULT0;		// vs
	Uint16 i = 0;
	/*
	EALLOW;
	SysCtrlRegs.WDKEY = 0xAA; // servide WD #2
	EDIS;
	 */
	//SpiaRegs.SPITXBUF=32761;      // Send data
	//SpiaRegs.SPITXBUF=327;      // Send data

	for(i=0;i<16;i++)
	{
		//SpiaRegs.SPITXBUF=a[i];      // Send data
		SpiaRegs.SPITXBUF=a[i];      // Send data

	}

	//SpiaRegs.SPIFFTX.bit.TXFFIENA = 1;
	//SpiaRegs.SPICTL.bit.SPIINTENA = 0;       //Interrupt enabled, Master/Slave XMIT enabled
	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1; 		// reset adc sequencer1
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // acknowledge pie interrupt
}

interrupt void ePWM1A_compare_isr(void)
{
	SpiaRegs.SPIFFTX.bit.TXFFIENA = 0;
	/*
	EALLOW;
	SysCtrlRegs.WDKEY = 0x55; // service WD #1
	EDIS;
	 */
	EPwm1Regs.ETCLR.bit.INT = 1;
	PieCtrlRegs.PIEACK.all = 4;
}

__interrupt void spiTxFifoIsr(void)
{
	Uint16 i;

	for(i=0;i<16;i++)
	{
		//SpiaRegs.SPITXBUF=a[i];      // Send data
		SpiaRegs.SPITXBUF=32761;      // Send data

	}

	/*
 	if (i==0) SpiaRegs.SPITXBUF = a;
 	if (i==1) SpiaRegs.SPITXBUF = b;
 	i++;
 	if (i==2) i = 0;
	 */
	//SpiaRegs.SPITXBUF = a;

	SpiaRegs.SPIFFTX.bit.TXFFINTCLR=1;  // Clear Interrupt flag
	PieCtrlRegs.PIEACK.all|=0x20;  		// Issue PIE ACK
}

__interrupt void spiRxFifoIsr(void)
{
	Uint16 i;
	for(i=0;i<16;i++)
	{
		rdata[i]=SpiaRegs.SPIRXBUF;		// Read data
	}
	for(i=0;i<8;i++)                    // Check received data
	{
		//if(rdata[i] != rdata_point+i)
		//error();
	}

	SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
	SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1; 	// Clear Interrupt flag
	PieCtrlRegs.PIEACK.all|=0x20;       // Issue PIE ack
}


