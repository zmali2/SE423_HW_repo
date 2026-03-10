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
// #include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200


// --------------------
// Notes for buzzer song
// --------------------
#define C4NOTE        ((uint16_t)(((50000000/2)/2)/261.63))
#define D4NOTE        ((uint16_t)(((50000000/2)/2)/293.66))
#define E4NOTE        ((uint16_t)(((50000000/2)/2)/329.63))
#define F4NOTE        ((uint16_t)(((50000000/2)/2)/349.23))
#define G4NOTE        ((uint16_t)(((50000000/2)/2)/392.00))
#define A4NOTE        ((uint16_t)(((50000000/2)/2)/440.00))
#define B4NOTE        ((uint16_t)(((50000000/2)/2)/493.88))
#define C5NOTE        ((uint16_t)(((50000000/2)/2)/523.25))
#define D5NOTE        ((uint16_t)(((50000000/2)/2)/587.33))
#define E5NOTE        ((uint16_t)(((50000000/2)/2)/659.25))
#define F5NOTE        ((uint16_t)(((50000000/2)/2)/698.46))
#define G5NOTE        ((uint16_t)(((50000000/2)/2)/783.99))
#define A5NOTE        ((uint16_t)(((50000000/2)/2)/880.00))
#define B5NOTE        ((uint16_t)(((50000000/2)/2)/987.77))
#define F4SHARPNOTE   ((uint16_t)(((50000000/2)/2)/369.99))
#define G4SHARPNOTE   ((uint16_t)(((50000000/2)/2)/415.3))
#define A4FLATNOTE    ((uint16_t)(((50000000/2)/2)/415.3))
#define C5SHARPNOTE   ((uint16_t)(((50000000/2)/2)/554.37))
#define A5FLATNOTE    ((uint16_t)(((50000000/2)/2)/830.61))
#define OFFNOTE       0

// need at least 10 different notes for the final song.
// This one uses: C4 D4 E4 F4 G4 A4 B4 C5 D5 E5
// ZA Exercise 2: Hot Cross Buns melody
#define SONG_LENGTH 24

// ZA Exercise 2: Hot Cross Buns from sheet music (B A G version)

// ZA Exercise 2: Fur Elise opening melody (simplified)

// ZA Exercise 2: Fur Elise (10+ distinct notes version)

#define SONG_LENGTH 40

uint16_t songarray[SONG_LENGTH] = {

E5NOTE, D5NOTE, E5NOTE, D5NOTE,
E5NOTE, B4NOTE, D5NOTE, C5NOTE,

A4NOTE, OFFNOTE,
C4NOTE, E4NOTE, A4NOTE,
B4NOTE, OFFNOTE,

E4NOTE, G4NOTE, B4NOTE,
C5NOTE, OFFNOTE,

E4NOTE, E5NOTE, D5NOTE,
E5NOTE, D5NOTE, E5NOTE,

B4NOTE, D5NOTE, C5NOTE,
A4NOTE, OFFNOTE,

C5NOTE, B4NOTE, A4NOTE,
G4NOTE, F4NOTE, E4NOTE,

D4NOTE, C4NOTE, B4NOTE,
A4NOTE
};

// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void SPIB_isr(void); //exercise 3
void initEPWM9AforBuzzer(void);



// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
uint16_t songindex = 0;
uint16_t songdone = 0;

int16_t spivalue1 = 0;
int16_t spivalue2 = 0;
int16_t gyroz_raw = 0;          // ZA Exercise 4: raw Gyro Z reading from MPU-9250
int32_t spiisrcount = 0;        // ZA Exercise 4: counts SPIB interrupts for 100 ms printing


void initEPWM9AforBuzzer(void)
{
    EALLOW;

    // ZA Exercise 2: Route GPIO16 to EPWM9A
    GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 5);
    GPIO_SetupPinOptions(16, GPIO_OUTPUT, GPIO_PUSHPULL);

    // ZA Exercise 2: Configure EPWM9A for square-wave buzzer output
    EPwm9Regs.TBCTL.bit.CTRMODE = 2;        // up-down count mode
    EPwm9Regs.TBCTL.bit.PHSEN = 0;          // disable phase loading
    EPwm9Regs.TBCTL.bit.HSPCLKDIV = 0;      // /1
    EPwm9Regs.TBCTL.bit.CLKDIV = 1;         // /2
    EPwm9Regs.TBCTL.bit.PRDLD = 0;          // shadow load
    EPwm9Regs.TBCTL.bit.SYNCOSEL = 3;       // disable sync out
    EPwm9Regs.TBCTR = 0;

    // ZA Exercise 2: Pick one fixed note for testing
    EPwm9Regs.TBPRD = A4NOTE;

    EPwm9Regs.AQCTLA.all = 0;
    EPwm9Regs.AQCTLA.bit.CAU = 0;
    EPwm9Regs.AQCTLA.bit.CAD = 0;
    EPwm9Regs.AQCTLA.bit.ZRO = 2;           // set high at zero
    EPwm9Regs.AQCTLA.bit.PRD = 1;           // clear low at period

    EDIS;
}


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
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

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

    // Buzzer pin initially as GPIO low until EPWM9A is enabled
    GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(16, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

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
    PieVectTable.SPIB_RX_INT = &SPIB_isr;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers




    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();


    initEPWM9AforBuzzer();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 1000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 125000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    //    init_serialSCIC(&SerialC,115200);
    //    init_serialSCID(&SerialD,115200);

    // ZA Exercise 3: Configure GPIO66 as software slave select for SPIB
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Initially set GPIO66/SS high so MPU-9250 is not selected

    // ZA Exercise 3: Configure SPIB pins
    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); // Set GPIO63 pin to SPISIMOB
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); // Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); // Set GPIO65 pin to SPICLKB

    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // ZA Exercise 3: Enable pull-up on SPI pin
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0; // ZA Exercise 3: Enable pull-up on SPI pin
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0; // ZA Exercise 3: Enable pull-up on SPI pin
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // ZA Exercise 3: Set GPIO63 to asynchronous mode
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // ZA Exercise 3: Set GPIO64 to asynchronous mode
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // ZA Exercise 3: Set GPIO65 to asynchronous mode
    EDIS;

    // ZA Exercise 3: Configure SPIB
    SpibRegs.SPICCR.bit.SPISWRESET = 0;      // Put SPI in reset
    SpibRegs.SPICTL.bit.CLK_PHASE = 1;       // Mode 01
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0;     // Mode 01
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1;    // Set to SPI master
    SpibRegs.SPICCR.bit.SPICHAR = 15;        // 16-bit words
    SpibRegs.SPICTL.bit.TALK = 1;            // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1;            // Free run
    SpibRegs.SPICTL.bit.SPIINTENA = 0;       // Disable SPI interrupt during setup
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49;   // 50 MHz / (49+1) = 1 MHz
    SpibRegs.SPISTS.all = 0x0000;            // Clear status flags

    SpibRegs.SPIFFTX.bit.SPIRST = 1;         // Pull SPI FIFO out of reset
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1;       // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0;         // Reset TX FIFO pointer and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1;     // Clear TX FIFO interrupt flag

    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0;    // Reset RX FIFO pointer and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1;     // Clear RX overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1;     // Clear RX FIFO interrupt flag
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1;       // Enable RX FIFO interrupt
    SpibRegs.SPIFFCT.bit.TXDLY = 0;          // No delay between transmits

    SpibRegs.SPICCR.bit.SPISWRESET = 1;      // Pull SPI out of reset
    SpibRegs.SPIFFTX.bit.TXFIFO = 1;         // Release transmit FIFO from reset
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1;    // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1;       // Enable SPI interrupt
    SpibRegs.SPIFFRX.bit.RXFFIL = 16;        // Interrupt level initially set to 16 words



    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;
    IER |= M_INT6;   // ZA Exercise 3: Enable CPU interrupt group 6 for SPIB RX

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

    PieCtrlRegs.PIEIER6.bit.INTx3 = 1; // ZA Exercise 3: Enable SPIB_RX_INT in PIE group 6

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
//        if (UARTPrint == 1 ) {
//            serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
//            UARTPrint = 0;
//        }

         if (UARTPrint == 1 ) {
             serial_printf(&SerialA,"ZA Exercise 4: GyroZ raw = %d\r\n", gyroz_raw);
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

__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

//    // ZA Exercise 3: Every 1 ms, transmit two 16-bit SPI words
//    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;   // Clear GPIO66 low to act as slave select
//    SpibRegs.SPIFFRX.bit.RXFFIL = 2;        // Interrupt when two values are in RX FIFO
//    SpibRegs.SPITXBUF = 0x4A3B;             // First 16-bit word to transmit
//    SpibRegs.SPITXBUF = 0xB517;             // Second 16-bit word to transmit

     // ZA Exercise 4: Every 1 ms, read MPU-9250 Gyro Z through SPIB
     GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;           // ZA Exercise 4: Clear GPIO66 low to select MPU9250 chip
     SpibRegs.SPIFFRX.bit.RXFFIL = 2;                // ZA Exercise 4: Interrupt when two values are in RX FIFO
     SpibRegs.SPITXBUF = (0x8000 | 0x4600);          // ZA Exercise 4: Read starting at register 0x46
     SpibRegs.SPITXBUF = 0x0000;                     // ZA Exercise 4: Clock out second 16-bit word containing Gyro Z

        if ((numTimer0calls%50) == 0) {
            PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
        }

    if ((numTimer0calls%250) == 0) {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {
            LEDdisplaynum = 0;
        }
    }

    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
// ZA Exercise 2: CPU Timer1 ISR for continuous song playback
// ZA Exercise 2: CPU Timer1 ISR for continuous song playback
__interrupt void cpu_timer1_isr(void)
{
    CpuTimer1.InterruptCount++;

    if (songarray[songindex] == OFFNOTE)
    {
        // ZA Exercise 2: Silence for rests
        GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(16, GPIO_OUTPUT, GPIO_PUSHPULL);
        GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;
    }
    else
    {
        // ZA Exercise 2: Route GPIO16 to EPWM9A and play current note
        GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 5);
        GPIO_SetupPinOptions(16, GPIO_OUTPUT, GPIO_PUSHPULL);
        EPwm9Regs.TBPRD = songarray[songindex];
    }

    // ZA Exercise 2: Advance to next note
    songindex++;

    // ZA Exercise 2: Loop song back to beginning
    if (songindex >= SONG_LENGTH)
    {
        songindex = 0;
    }
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{


    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

    if ((CpuTimer2.InterruptCount % 50) == 0) {
        UARTPrint = 1;
    }
}

//// ZA Exercise 3: SPIB receive ISR
//__interrupt void SPIB_isr(void)
//{
//    spivalue1 = SpibRegs.SPIRXBUF;          // Read first 16-bit value off RX FIFO
//    spivalue2 = SpibRegs.SPIRXBUF;          // Read second 16-bit value off RX FIFO
//
//    GpioDataRegs.GPCSET.bit.GPIO66 = 1;     // Set GPIO66 high to end slave select
//
//    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1;    // Clear overflow flag
//    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1;    // Clear RX FIFO interrupt flag
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge PIE group 6 interrupt
//}

 // ZA Exercise 4: SPIB receive ISR for MPU-9250 Gyro Z reading
 __interrupt void SPIB_isr(void)
 {
     spivalue1 = SpibRegs.SPIRXBUF;          // ZA Exercise 4: First received 16-bit word, not used
     spivalue2 = SpibRegs.SPIRXBUF;          // ZA Exercise 4: Second received 16-bit word contains raw Gyro Z

     gyroz_raw = spivalue2;                  // ZA Exercise 4: Save raw Gyro Z reading
     spiisrcount++;                          // ZA Exercise 4: Count SPI interrupts

     if ((spiisrcount % 100) == 0) {
         UARTPrint = 1;                      // ZA Exercise 4: Print Gyro Z every 100 ms
     }

     GpioDataRegs.GPCSET.bit.GPIO66 = 1;     // ZA Exercise 4: Set GPIO66 high to end slave select

     SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1;    // ZA Exercise 4: Clear overflow flag
     SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1;    // ZA Exercise 4: Clear RX FIFO interrupt flag
     PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // ZA Exercise 4: Acknowledge PIE group 6 interrupt
 }
