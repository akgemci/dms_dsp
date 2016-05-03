
#include "DSP2833x_Device.h"

// external function prototypes
extern void InitSysCtrl(void);
extern void InitPieCtrl(void);
extern void InitPieVectTable(void);
extern void InitCpuTimers(void);
extern void ConfigCpuTimer(struct CPUTIMER_VARS *, float, float);
extern void InitAdc(void);
extern void InitAdcregs(void);
extern void InitSpiaa(void);
extern void InitEpwm(void);
extern void InitSpiaGpio(void);
// Prototype statements for functions found within this file.
void Gpio_select(void);
interrupt void cpu_timer0_isr(void);
interrupt void adc_isr(void);
//interrupt void spi_transmit_isr(void);
//interrupt void spi_recieve_isr(void);
int counter1=0;

int muxarr1[14]={0};
int muxarr2[14]={0};
int muxarr3[14]={0};
int muxarr4[14]={0};
float actual[16]={0};
float rdata[14];
float Actual1=0;
float Actual2=0;
float Actual3=0;
float Actual4=0;
float Actual5=0;
float Actual6=0;
float Actual7=0;
float Actual8=0;
float Actual9=0;
float Actual10=0;
float Actual11=0;
float Actual12=0;
float Actual13=0;
float Actual14=0;

float ADCV1=0;
float ADCV2=0;
float ADCV3=0;
float ADCV4=0;
float ADCI1=0;
float ADCI2=0;
float ADCI3=0;
float ADCI4=0;
float DCV1=0;
float DCV2=0;
float DCI1=0;
float DCI2=0;
float Torque=0;
float Speed=0;
//###########################################################################
//						main code
//###########################################################################
void main(void)
{

       	InitSysCtrl();	// Basic Core Init from DSP2833x_SysCtrl.c

	EALLOW;
	SysCtrlRegs.WDCR= 0x00AF;	// Re-enable the watchdog
	EDIS;			// 0x00AF  to NOT disable the Watchdog, Prescaler = 64

	DINT;				// Disable all interrupts
	IER=0x0000;
	IFR=0x0000;

	InitPieCtrl();		// basic setup of PIE table; from DSP2833x_PieCtrl.c
	InitPieVectTable();	// default ISR's in PIE
	Gpio_select();

	InitAdc();
	InitSpiaGpio();
	InitAdcregs();
	InitEpwm();
	InitSpiaa();


	EALLOW;
	PieVectTable.TINT0 = &cpu_timer0_isr;
	PieVectTable.ADCINT= &adc_isr;
	//PieVectTable.SPITXINTA = &spi_transmit_isr;
	//PieVectTable.SPIRXINTA = &spi_recieve_isr;

	EDIS;

	InitCpuTimers();	// basic setup CPU Timer0, 1 and 2

	ConfigCpuTimer(&CpuTimer0,150,25); // CPU - Timer0 at 100 milliseconds

	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;	//CPU Timer0
	PieCtrlRegs.PIEIER1.bit.INTx6 = 1;	//ADC
	//	PieCtrlRegs.PIEIER6.bit.INTx1 = 1; // SPI RX INTA
	//	PieCtrlRegs.PIEIER6.bit.INTx2 = 1; // SPI TX INTA interrupt
	IER |=1;
	EINT;
	ERTM;

	CpuTimer0Regs.TCR.bit.TSS = 0;	// start timer0

	while(1)
	{
		CpuTimer0.InterruptCount = 0;
	}
}


void Gpio_select(void)
{
	EALLOW;
	GpioCtrlRegs.GPAMUX1.all = 0;		// GPIO15 ... GPIO0 = General Puropse I/O
	GpioCtrlRegs.GPAMUX2.all = 0;		// GPIO31 ... GPIO16 = General Purpose I/O

	GpioCtrlRegs.GPBMUX1.all = 0;		// GPIO47 ... GPIO32 = General Purpose I/O
	GpioCtrlRegs.GPBMUX2.all = 0;		// GPIO63 ... GPIO48 = General Purpose I/O
	GpioCtrlRegs.GPCMUX1.all = 0;		// GPIO79 ... GPIO64 = General Purpose I/O
	GpioCtrlRegs.GPCMUX2.all = 0;		// GPIO87 ... GPIO80 = General Purpose I/O

	GpioCtrlRegs.GPADIR.all = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;
	GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;
	EDIS;
}

void InitAdcregs(void)
{

	AdcRegs.ADCTRL1.bit.SEQ_CASC=1;		//cascade mode
	AdcRegs.ADCTRL1.bit.CONT_RUN=0;		//stop after reaching end of sequence
	AdcRegs.ADCTRL1.bit.ACQ_PS=7;		//acquisition time=8/ADCCLK
	AdcRegs.ADCTRL1.bit.CPS=0;			//conversion prescale; ADCCLK=FCLK/1

	AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1=1;	//software trigger-start SEQ1
	AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1=1;		//int enable
	AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1=0;		//interrupt every EOS( end of sequence)

	AdcRegs.ADCTRL3.bit.ADCCLKPS=3;		// FCLK=HSPCLK/(2*3)

	AdcRegs.ADCMAXCONV.all= 0x000D;     //14 conversion
	AdcRegs.ADCCHSELSEQ1.bit.CONV00=0; //ADCINA0
	AdcRegs.ADCCHSELSEQ1.bit.CONV01=1;
	AdcRegs.ADCCHSELSEQ1.bit.CONV02=2;
	AdcRegs.ADCCHSELSEQ1.bit.CONV03=3;
	AdcRegs.ADCCHSELSEQ2.bit.CONV04=4;
	AdcRegs.ADCCHSELSEQ2.bit.CONV05=5;
	AdcRegs.ADCCHSELSEQ2.bit.CONV06=6;
	AdcRegs.ADCCHSELSEQ2.bit.CONV07=7; //ADCINA7
	AdcRegs.ADCCHSELSEQ3.bit.CONV08=8; //ADCINB0
	AdcRegs.ADCCHSELSEQ3.bit.CONV09=9;
	AdcRegs.ADCCHSELSEQ3.bit.CONV10=10;
	AdcRegs.ADCCHSELSEQ3.bit.CONV11=11;
	AdcRegs.ADCCHSELSEQ4.bit.CONV12=12;
	AdcRegs.ADCCHSELSEQ4.bit.CONV13=13; //ADCB5

}
void InitSpiaa(void)
{
	// Initialize SPI FIFO registers
	SpiaRegs.SPICCR.bit.SPISWRESET=0; // Reset SPI

	SpiaRegs.SPICCR.all=0x001F;       //16-bit character, no loopback mode
	SpiaRegs.SPICTL.all=0x0017;       //Interrupt enabled, Master/Slave XMIT enabled
	SpiaRegs.SPISTS.all=0x0000;
	SpiaRegs.SPICTL.bit.SPIINTENA = 1;       //Interrupt enabled, Master/Slave XMIT enabled
	SpiaRegs.SPIFFTX.bit.TXFFIENA = 1;

	SpiaRegs.SPIBRR=0x0005;           // Baud rate
	SpiaRegs.SPIFFTX.all=0xC028;      // Enable FIFO's, set TX FIFO level to 8
	SpiaRegs.SPIFFRX.all=0x0028;      // Set RX FIFO level to 8
	SpiaRegs.SPIFFCT.all=0x00;
	SpiaRegs.SPIPRI.all=0x0010;

	SpiaRegs.SPICCR.bit.SPISWRESET=1;  // Enable SPI

	SpiaRegs.SPIFFTX.bit.TXFIFO=1;
	SpiaRegs.SPIFFRX.bit.RXFIFORESET=1;
}
void InitEpwm(void)
{
		EPwm2Regs.TBCTL.all = 0xC030;

				// Configure timer control register
		/*
		 bit 15-14     11:     FREE/SOFT, 11 = ignore emulation suspend
		 bit 13        0:      PHSDIR, 0 = count down after sync event
		 bit 12-10     000:    CLKDIV, 000 => TBCLK = HSPCLK/1

		 bit 9-7       000:    HSPCLKDIV, 000 => HSPCLK = SYSCLKOUT/1
		 bit 6         0:      SWFSYNC, 0 = no software sync produced
		 bit 5-4       11:     SYNCOSEL, 11 = sync-out disabled
		 bit 3         0:      PRDLD, 0 = reload PRD on counter=0
		 bit 2         0:      PHSEN, 0 = phase control disabled
		 bit 1-0       00:     CTRMODE, 00 = count up mode
		*/
		EPwm2Regs.TBPRD = 3750;	// TPPRD +1  =  TPWM / (HSPCLKDIV * CLKDIV * TSYSCLK)
		EPwm2Regs.ETPS.all = 0x0100;			// Configure ADC start by ePWM2
		/*
	 	 bit 15-14     00:     EPWMxSOCB, read-only
		 bit 13-12     00:     SOCBPRD, don't care
		 bit 11-10     00:     EPWMxSOCA, read-only
		 bit 9-8       01:     SOCAPRD, 01 = generate SOCA on first event
		 bit 7-4       0000:   reserved
		 bit 3-2       00:     INTCNT, don't care
		 bit 1-0       00:     INTPRD, don't care
		*/
		EPwm2Regs.ETSEL.all = 0x0A00;			// Enable SOCA to ADC
		/*
		 bit 15        0:      SOCBEN, 0 = disable SOCB
		 bit 14-12     000:    SOCBSEL, don't care
		 bit 11        1:      SOCAEN, 1 = enable SOCA
		 bit 10-8      010:    SOCASEL, 010 = SOCA on PRD event
		 bit 7-4       0000:   reserved
		 bit 3         0:      INTEN, 0 = disable interrupt
		 bit 2-0       000:    INTSEL, don't care
		 */
}
interrupt void cpu_timer0_isr(void)
{
	CpuTimer0.InterruptCount++;
	EALLOW;
	SysCtrlRegs.WDKEY = 0xAA;	// service WD #2
	EDIS;

	counter1++;
		if(counter1==1)
		{
			GpioDataRegs.GPACLEAR.bit.GPIO9 = 1; 	// 00 case
			GpioDataRegs.GPACLEAR.bit.GPIO11= 1;
			muxarr1[0] = GpioDataRegs.GPADAT.bit.GPIO15;
			muxarr1[1] = GpioDataRegs.GPADAT.bit.GPIO16;
			muxarr1[2] = GpioDataRegs.GPADAT.bit.GPIO17;
			muxarr1[3] = GpioDataRegs.GPADAT.bit.GPIO18;
			muxarr1[4] = GpioDataRegs.GPADAT.bit.GPIO19;
			muxarr1[5] = GpioDataRegs.GPADAT.bit.GPIO20;
			muxarr1[6] = GpioDataRegs.GPADAT.bit.GPIO21;
			muxarr1[7] = GpioDataRegs.GPADAT.bit.GPIO22;
			muxarr1[8] = GpioDataRegs.GPADAT.bit.GPIO23;
			muxarr1[9] = GpioDataRegs.GPADAT.bit.GPIO24;
			muxarr1[10] = GpioDataRegs.GPADAT.bit.GPIO25;
			muxarr1[11] = GpioDataRegs.GPADAT.bit.GPIO26;
			muxarr1[12] = GpioDataRegs.GPADAT.bit.GPIO27;
			muxarr1[13] = GpioDataRegs.GPADAT.bit.GPIO28;

		}
		else if(counter1==2)
		{
			GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;	//01 case
			GpioDataRegs.GPASET.bit.GPIO11 = 1;
			muxarr2[0] = GpioDataRegs.GPADAT.bit.GPIO15;
			muxarr2[1] = GpioDataRegs.GPADAT.bit.GPIO16;
			muxarr2[2] = GpioDataRegs.GPADAT.bit.GPIO17;
			muxarr2[3] = GpioDataRegs.GPADAT.bit.GPIO18;
			muxarr2[4] = GpioDataRegs.GPADAT.bit.GPIO19;
			muxarr2[5] = GpioDataRegs.GPADAT.bit.GPIO20;
			muxarr2[6] = GpioDataRegs.GPADAT.bit.GPIO21;
			muxarr2[7] = GpioDataRegs.GPADAT.bit.GPIO22;
			muxarr2[8] = GpioDataRegs.GPADAT.bit.GPIO23;
			muxarr2[9] = GpioDataRegs.GPADAT.bit.GPIO24;
			muxarr2[10] = GpioDataRegs.GPADAT.bit.GPIO25;
			muxarr2[11] = GpioDataRegs.GPADAT.bit.GPIO26;
			muxarr2[12] = GpioDataRegs.GPADAT.bit.GPIO27;
			muxarr2[13] = GpioDataRegs.GPADAT.bit.GPIO28;
		}
		else if(counter1==3)
		{
			GpioDataRegs.GPASET.bit.GPIO9 = 1;		//10 case
			GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
			muxarr3[0] = GpioDataRegs.GPADAT.bit.GPIO15;
			muxarr3[1] = GpioDataRegs.GPADAT.bit.GPIO16;
			muxarr3[2] = GpioDataRegs.GPADAT.bit.GPIO17;
			muxarr3[3] = GpioDataRegs.GPADAT.bit.GPIO18;
			muxarr3[4] = GpioDataRegs.GPADAT.bit.GPIO19;
			muxarr3[5] = GpioDataRegs.GPADAT.bit.GPIO20;
			muxarr3[6] = GpioDataRegs.GPADAT.bit.GPIO21;
			muxarr3[7] = GpioDataRegs.GPADAT.bit.GPIO22;
			muxarr3[8] = GpioDataRegs.GPADAT.bit.GPIO23;
			muxarr3[9] = GpioDataRegs.GPADAT.bit.GPIO24;
			muxarr3[10] = GpioDataRegs.GPADAT.bit.GPIO25;
			muxarr3[11] = GpioDataRegs.GPADAT.bit.GPIO26;
			muxarr3[12] = GpioDataRegs.GPADAT.bit.GPIO27;
			muxarr3[13] = GpioDataRegs.GPADAT.bit.GPIO28;
		}
		else if(counter1==4)
		{
			GpioDataRegs.GPASET.bit.GPIO9 = 1;		//11 case
			GpioDataRegs.GPASET.bit.GPIO11 = 1;
			muxarr4[0] = GpioDataRegs.GPADAT.bit.GPIO15;
			muxarr4[1] = GpioDataRegs.GPADAT.bit.GPIO16;
			muxarr4[2] = GpioDataRegs.GPADAT.bit.GPIO17;
			muxarr4[3] = GpioDataRegs.GPADAT.bit.GPIO18;
			muxarr4[4] = GpioDataRegs.GPADAT.bit.GPIO19;
			muxarr4[5] = GpioDataRegs.GPADAT.bit.GPIO20;
			muxarr4[6] = GpioDataRegs.GPADAT.bit.GPIO21;
			muxarr4[7] = GpioDataRegs.GPADAT.bit.GPIO22;
			muxarr4[8] = GpioDataRegs.GPADAT.bit.GPIO23;
			muxarr4[9] = GpioDataRegs.GPADAT.bit.GPIO24;
			muxarr4[10] = GpioDataRegs.GPADAT.bit.GPIO25;
			muxarr4[11] = GpioDataRegs.GPADAT.bit.GPIO26;
			muxarr4[12] = GpioDataRegs.GPADAT.bit.GPIO27;
			muxarr4[13] = GpioDataRegs.GPADAT.bit.GPIO28;
			counter1=0;

		}

					EALLOW;
					SysCtrlRegs.WDKEY = 0x55;	// service WD #1
					EDIS;

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
interrupt void adc_isr(void)
{

	ADCV1=AdcMirror.ADCRESULT0;
	ADCV2=AdcMirror.ADCRESULT1;
	ADCV3=AdcMirror.ADCRESULT2;
	ADCV4=AdcMirror.ADCRESULT3;
	ADCI1=AdcMirror.ADCRESULT4;
	ADCI2=AdcMirror.ADCRESULT5;
	ADCI3=AdcMirror.ADCRESULT6;
	ADCI4=AdcMirror.ADCRESULT7;
	DCV1=AdcMirror.ADCRESULT8;
	DCV2=AdcMirror.ADCRESULT9;
	DCI1=AdcMirror.ADCRESULT10;
	DCI2=AdcMirror.ADCRESULT11;
	Torque=AdcMirror.ADCRESULT12;
	Speed=AdcMirror.ADCRESULT13;



	Actual1=(ADCV1-2048)*3/4095;
	Actual2=(ADCV2-2048)*3/4095;
	Actual3=(ADCV3-2048)*3/4095;
	Actual4=(ADCV4-2048)*3/4095;
	Actual5=(ADCI1-2048)*3/4095;
	Actual6=(ADCI2-2048)*3/4095;
	Actual7=(ADCI3-2048)*3/4095;
	Actual8=(ADCI4-2048)*3/4095;
	Actual9=(DCV1-2048)*3/4095;
	Actual10=(DCV2-2048)*3/4095;
	Actual11=(DCI1-2048)*3/4095;
	Actual12=(DCI2-2048)*3/4095;
	Actual13=(Torque-2048)*3/4095;
	Actual14=(Speed-2048)*3/4095;


	actual[3]=1;
	actual[4]=0;
	actual[5]=0;
	actual[6]=0;
	actual[7]=1;
	actual[8]=0;


	Uint16 i;
	for(i=0;i<16;i++)
	{

		SpiaRegs.SPITXBUF=actual[i];      // Send data

	}

	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

/*interrupt void spi_transmit_isr(void)
{

	Uint16 i;

	for(i=0;i<14;i++)
	{
		SpiaRegs.SPITXBUF=actual[i];      // Send data

	}

	SpiaRegs.SPIFFTX.bit.TXFFINTCLR=1;  // Clear Interrupt flag
	PieCtrlRegs.PIEACK.all|=0x20;  		// Issue PIE ACK
}

interrupt void spi_recieve_isr(void)
{
	Uint16 i;
		for(i=0;i<14;i++)
		{
			rdata[i]=SpiaRegs.SPIRXBUF;		// Read data
		}

		SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
		SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1; 	// Clear Interrupt flag
		PieCtrlRegs.PIEACK.all|=0x20;       // Issue PIE ack
}
 */
//===========================================================================
// End of SourceCode.
//===========================================================================
