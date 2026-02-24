//#############################################################################
// FILE:   HWstarter_main.c
//
// TITLE:  HW Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void ADCA_ISR(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
int16_t updown = 1;
volatile int16_t  Adca4result = 0;
volatile float    Adca4volts  = 0.0f;

volatile int16_t Adca2result = 0;
volatile int16_t Adca3result = 0;

volatile float   Adca2volts  = 0.0f;
volatile float   Adca3volts  = 0.0f;



void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

	// Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

	// Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

	// LED1 and PWM Pin
//    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
//    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
//    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;


    // LED1 driven by EPWM12A (GPIO22)
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 5);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);   // fine to leave; pinmux is what matters

    // ----- EPWM12A SETUP -----

    EPwm12Regs.TBCTL.bit.CTRMODE = 0;     // Up count mode
    EPwm12Regs.TBCTL.bit.PHSEN = 0;       // Disable phase loading
    EPwm12Regs.TBCTL.bit.FREE_SOFT = 2;   // Free run during debug
    EPwm12Regs.TBCTL.bit.CLKDIV = 0;      // Divide by 1

    EPwm12Regs.TBCTR = 0;                 // Start counter at 0
    EPwm12Regs.TBPRD = 10000;             // 5kHz (200us period)

    EPwm12Regs.CMPA.bit.CMPA = 5000;      // 50% duty cycle

    EPwm12Regs.AQCTLA.bit.ZRO = 2;        // Set HIGH at zero
    EPwm12Regs.AQCTLA.bit.CAU = 1;        // Set LOW at CMPA

    EPwm12Regs.TBPHS.bit.TBPHS = 0;

    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1;   // Disable pull-up
    EDIS;


    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1;   // Disable pull-up for EPWM12A
    EDIS;





	// LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

	// LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

	// LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

	// LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

	// LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

	// LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

	// LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

	// LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

	// LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

	// LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

	// LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

	// LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

	// LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

	// LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

	// LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

	//SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

	//WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

	//Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.ADCA1_INT = &ADCA_ISR;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 1000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    //    init_serialSCIC(&SerialC,115200);
    //    init_serialSCID(&SerialD,115200);

    EALLOW;

    // EPWM4 will generate SOCA at TBCTR == TBPRD once every period
    EPwm4Regs.ETSEL.bit.SOCAEN  = 0;    // disable while configuring
    EPwm4Regs.TBCTL.bit.CTRMODE = 3;    // freeze counter while configuring

    EPwm4Regs.ETSEL.bit.SOCASEL = 2;    // 010: event when TBCTR = TBPRD  (period event)
    EPwm4Regs.ETPS.bit.SOCAPRD  = 1;    // 01: generate SOCA on 1st event (no prescale)

    EPwm4Regs.TBCTR = 0x0;             // clear counter
    EPwm4Regs.TBPHS.bit.TBPHS = 0x0000;
    EPwm4Regs.TBCTL.bit.PHSEN = 0;      // disable phase loading
    EPwm4Regs.TBCTL.bit.CLKDIV = 0;     // /1

    // TBCLK = 50 MHz (as handout states). 1 ms => 50,000 ticks.
    // In up-count, period is (TBPRD+1)/TBCLK, so use 49999 for exactly 1.000 ms.
    EPwm4Regs.TBPRD = 49999;

    EPwm4Regs.ETSEL.bit.SOCAEN  = 1;    // enable SOCA
    EPwm4Regs.TBCTL.bit.CTRMODE = 0;    // unfreeze, up-count mode

    EDIS;

    EALLOW;

    // ADCA clock/power setup (your handout gave these)
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; // ADCCLK divider
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1; // late interrupt pulse
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ    = 1; // power up
    DELAY_US(1000);                       // 1ms delay

    // SOC0: ADCINA4
    AdcaRegs.ADCSOC0CTL.bit.CHSEL   = 4;   // ADCIN4 (ADCINA4)
    AdcaRegs.ADCSOC0CTL.bit.ACQPS   = 99;  // sample window
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 11;  // ePWM4 SOCA  (trigger)

    // Interrupt when SOC0 completes:
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 2; // SOC0
    AdcaRegs.ADCINTSEL1N2.bit.INT1E   = 1; // enable ADCINT1
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // clear flag

    // SOC1: ADCINA2 (Joystick X)
    AdcaRegs.ADCSOC1CTL.bit.CHSEL   = 2;
    AdcaRegs.ADCSOC1CTL.bit.ACQPS   = 99;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 11;  // ePWM4 SOCA

    // SOC2: ADCINA3 (Joystick Y)
    AdcaRegs.ADCSOC2CTL.bit.CHSEL   = 3;
    AdcaRegs.ADCSOC2CTL.bit.ACQPS   = 99;
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 11;  // ePWM4 SOCA


    EDIS;


    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	// Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;   // ADCA1


    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
//        if (UARTPrint == 1 ) {
//				serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
//            UARTPrint = 0;
//        if (UARTPrint == 1) {
//            serial_printf(&SerialA,"ADCINA4: raw=%d, volts=%.3f\r\n", Adca4result, Adca4volts);
//            UARTPrint = 0;

//        }
        if (UARTPrint == 1) {
            serial_printf(&SerialA,
                "Photo(A4)=%.3fV  JoyX(A2)=%.3fV  JoyY(A3)=%.3fV\r\n",
                Adca4volts, Adca2volts, Adca3volts);
            UARTPrint = 0;
        }
    }
}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
	// making it lower priority than all other Hardware interrupts.
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts



    // Insert SWI ISR Code here.......


    numSWIcalls++;

    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }

//    if ((numTimer0calls%250) == 0) {
//        displayLEDletter(LEDdisplaynum);
//        LEDdisplaynum++;
//        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
//            LEDdisplaynum = 0;
//        }
//    }

	// Blink LaunchPad Red LED
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{


    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    CpuTimer2.InterruptCount++;

//    if (updown == 1)
//    {
//        EPwm12Regs.CMPA.bit.CMPA++;
//
//        if (EPwm12Regs.CMPA.bit.CMPA >= EPwm12Regs.TBPRD)
//        {
//            updown = 0;
//        }
//    }
//    else
//    {
//        EPwm12Regs.CMPA.bit.CMPA--;
//
//        if (EPwm12Regs.CMPA.bit.CMPA == 0)
//        {
//            updown = 1;
//        }
//    }
}

__interrupt void ADCA_ISR(void)
{
    static int32_t isr_count = 0;



    // Read results (SOC0->RESULT0, SOC1->RESULT1, SOC2->RESULT2)
    Adca4result = (int16_t)AdcaResultRegs.ADCRESULT0; // photo
    Adca2result = (int16_t)AdcaResultRegs.ADCRESULT1; // joy X
    Adca3result = (int16_t)AdcaResultRegs.ADCRESULT2; // joy Y

    // Convert to volts
    Adca4volts = ((float)Adca4result) * (3.0f / 4096.0f);
    Adca2volts = ((float)Adca2result) * (3.0f / 4096.0f);
    Adca3volts = ((float)Adca3result) * (3.0f / 4096.0f);

    // Every 100ms (since EPWM4 is 1kHz => ISR is 1000 Hz)
    isr_count++;
    if (isr_count >= 100)
    {
        UARTPrint = 1;
        isr_count = 0;
    }

    // Joystick -> direction LEDs (adjust thresholds if needed)
    // Assume center ~1.5V. Use a deadband.
    const float VLO = 1.20f;
    const float VHI = 1.80f;

    // Turn all direction LEDs off first
    GpioDataRegs.GPCCLEAR.bit.GPIO94  = 1;  // LED2
    GpioDataRegs.GPCCLEAR.bit.GPIO95  = 1;  // LED3
    GpioDataRegs.GPDCLEAR.bit.GPIO97  = 1;  // LED4
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;  // LED5
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;  // LED6
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;  // LED7
    GpioDataRegs.GPACLEAR.bit.GPIO25  = 1;  // LED8
    GpioDataRegs.GPACLEAR.bit.GPIO26  = 1;  // LED9
    GpioDataRegs.GPACLEAR.bit.GPIO27  = 1;  // LED10
    GpioDataRegs.GPBCLEAR.bit.GPIO60  = 1;  // LED11
    GpioDataRegs.GPBCLEAR.bit.GPIO61  = 1;  // LED12
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;  // LED13
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;  // LED14
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;  // LED15
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;  // LED16

    // Priority rule: if Y moved, show Y direction; else show X direction; else center LED8
    if (Adca3volts > VHI) {
        // Y toward buzzer -> LED6
        GpioDataRegs.GPESET.bit.GPIO130 = 1; // LED6
    } else if (Adca3volts < VLO) {
        // Y away from buzzer -> LED10
        GpioDataRegs.GPASET.bit.GPIO27 = 1;  // LED10
    } else if (Adca2volts > VHI) {
        // X "up" -> LED3
        GpioDataRegs.GPCSET.bit.GPIO95 = 1;  // LED3
    } else if (Adca2volts < VLO) {
        // X "down" -> LED13
        GpioDataRegs.GPESET.bit.GPIO157 = 1; // LED13
    } else {
        // Center -> LED8
        GpioDataRegs.GPASET.bit.GPIO25 = 1;  // LED8
    }

    // Scale photoresistor reading to LED1 (EPWM12A duty)
    // CMPA in [0, TBPRD]
    EPwm12Regs.CMPA.bit.CMPA = (uint16_t)(((float)Adca4result / 4095.0f) * (float)EPwm12Regs.TBPRD);

    // Clear ADC interrupt flag + acknowledge PIE
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
