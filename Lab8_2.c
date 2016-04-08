
#include "DSP2833x_Device.h"

// external function prototypes
extern void InitSysCtrl(void);
extern void InitPieCtrl(void);
extern void InitPieVectTable(void);
extern void InitCpuTimers(void);
extern void ConfigCpuTimer(struct CPUTIMER_VARS *, float, float);
extern void display_ADC(unsigned int);
extern void InitAdc(void);

// Prototype statements for functions found within this file.
void Gpio_select(void);
interrupt void cpu_timer0_isr(void);
interrupt void adc_isr(void);



unsigned int Voltage_VR1;
		int	counter=0;
		

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

	AdcRegs.ADCMAXCONV.all= 0x0001;
	AdcRegs.ADCCHSELSEQ1.bit.CONV00=0;
	AdcRegs.ADCCHSELSEQ1.bit.CONV01=1;

	EPwm2Regs.TBCTL.all = 0xC030;
	EPwm2Regs.TBPRD=2999;

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
	PieCtrlRegs.PIEIER1.bit.INTx6=1;
	IER |=1;

	EINT;
	ERTM;

	CpuTimer0Regs.TCR.bit.TSS = 0;	// start timer0

	while(1)
	{ 
		while(CpuTimer0.InterruptCount == 0);
		{	
					EALLOW;
					SysCtrlRegs.WDKEY = 0x55;	// service WD #1
					SysCtrlRegs.WDKEY = 0xAA;
					EDIS;
		}
		CpuTimer0.InterruptCount = 0;
		ConfigCpuTimer(&CpuTimer0,100, 20000+Voltage_VR1*239.32);
		CpuTimer0Regs.TCR.bit.TSS=0;
		
			  		counter++;
					if(counter&1) GpioDataRegs.GPASET.bit.GPIO9 = 1;
						else GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
					if(counter&2) GpioDataRegs.GPASET.bit.GPIO11 = 1;
						else GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
					if(counter&4) GpioDataRegs.GPBSET.bit.GPIO34 = 1;
						else GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
					if(counter&8) GpioDataRegs.GPBSET.bit.GPIO49 = 1;
						else GpioDataRegs.GPBCLEAR.bit.GPIO49 = 1;
			
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

	GpioCtrlRegs.GPBDIR.all = 0;		// GPIO63-32 as inputs
	GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;	// peripheral explorer: LED LD3 at GPIO34
	GpioCtrlRegs.GPBDIR.bit.GPIO49 = 1; // peripheral explorer: LED LD4 at GPIO49
	GpioCtrlRegs.GPCDIR.all = 0;		// GPIO87-64 as inputs
	EDIS;
}   

interrupt void cpu_timer0_isr(void)
{
	CpuTimer0.InterruptCount++;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
interrupt void adc_isr(void)
{
	Voltage_VR1 = AdcMirror.ADCRESULT0;
	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}
//===========================================================================
// End of SourceCode.
//===========================================================================
