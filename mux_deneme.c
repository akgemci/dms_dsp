
#include "DSP2833x_Device.h"

// external function prototypes
extern void InitSysCtrl(void);
extern void InitPieCtrl(void);
extern void InitPieVectTable(void);
extern void InitCpuTimers(void);
extern void ConfigCpuTimer(struct CPUTIMER_VARS *, float, float);
extern void InitAdc(void);

// Prototype statements for functions found within this file.
void Gpio_select(void);
interrupt void cpu_timer0_isr(void);
interrupt void adc_isr(void);

int counter1=0;

int muxarr1[14]={0};
int muxarr2[14]={0};
int muxarr3[14]={0};
int muxarr4[14]={0};

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

	Gpio_select();		// GPIO9, GPIO11, GPIO34 and GPIO49 as output
	// to 4 LEDs at Peripheral Explorer)

	InitPieCtrl();		// basic setup of PIE table; from DSP2833x_PieCtrl.c

	InitPieVectTable();	// default ISR's in PIE
	InitAdc();

	AdcRegs.ADCTRL1.bit.SEQ_CASC=1;
	AdcRegs.ADCTRL1.bit.CONT_RUN=0;
	AdcRegs.ADCTRL1.bit.ACQ_PS=7;
	AdcRegs.ADCTRL1.bit.CPS=0;

	AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1=1;
	AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1=1;
	AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1=0;

	AdcRegs.ADCTRL3.bit.ADCCLKPS= 3;

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



	EPwm2Regs.TBCTL.all = 0xC030;
	EPwm2Regs.TBPRD=3749;

	EPwm2Regs.ETPS.all = 0;
	EPwm2Regs.ETPS.bit.SOCAPRD = 1;
	EPwm2Regs.ETSEL.all = 0;
	EPwm2Regs.ETSEL.bit.SOCAEN=1;
	EPwm2Regs.ETSEL.bit.SOCASEL = 2;



	EALLOW;
	PieVectTable.TINT0 = &cpu_timer0_isr;
	PieVectTable.ADCINT= &adc_isr;
	EDIS;

	InitCpuTimers();	// basic setup CPU Timer0, 1 and 2

	ConfigCpuTimer(&CpuTimer0,150,100000); // CPU - Timer0 at 100 milliseconds

	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
	IER |=1;

	EINT;
	ERTM;

	CpuTimer0Regs.TCR.bit.TSS = 0;	// start timer0

	while(1)
	{    
		while(CpuTimer0.InterruptCount<10)
		{
			EALLOW;
			SysCtrlRegs.WDKEY = 0x55;	// service WD #1
			EDIS;
		}

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
	GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;	// peripheral explorer: LED LD1 at GPIO9
	GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;	// peripheral explorer: LED LD2 at GPIO11

	EDIS;
}   

interrupt void cpu_timer0_isr(void)
{
	CpuTimer0.InterruptCount++;
	EALLOW;
	SysCtrlRegs.WDKEY = 0xAA;	// service WD #2
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



	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}



//===========================================================================
// End of SourceCode.
//===========================================================================
