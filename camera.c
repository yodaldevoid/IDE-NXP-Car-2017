/*
 * Freescale Cup linescan camera code
 *
 *  This method of capturing data from the line
 *  scan cameras uses a flex timer module, periodic
 *  interrupt timer, an ADC, and some GPIOs.
 *  CLK and SI are driven with GPIO because the FTM1
 *  module used doesn't have any output pins on the
 *  development board. The PIT timer is used to
 *  control the integration period. When it overflows
 *  it enables interrupts from the FTM1 module and then
 *  the FTM1 and ADC are active for 128 clock cycles to
 *  generate the camera signals and read the camera
 *  output.
 *
 *  PTB8        - camera CLK
 *  PTB23       - camera SI
 *  ADC0_DP0    - camera AOut
 */

#include "MK64F12.h"
#include "util.h"
#include "camera.h"

// Port B
#define CAM_CLK 9
// Port B
#define CAM_SI  23

void camera_init_FTM1(void);
void camera_init_GPIO(void);
void camera_init_PIT0(void);
void camera_init_ADC0(void);

int camReadDone = 0;
// line stores the current array of camera data
uint16_t line[128];

#ifdef DEBUG_CAM_DATA
int capCnt = 0;
#endif

// currentADC0 holds the current ADC value
uint16_t currentADC0;

void init_camera(void) {
    camera_init_GPIO(); // For CLK and SI output on GPIO
    camera_init_FTM1(); // To generate CLK, SI, and trigger ADC
    camera_init_ADC0();
    camera_init_PIT0(); // To trigger camera read based on integration time
}

/* ADC0 Conversion Complete ISR  */
void ADC0_IRQHandler(void) {
    // Reading ADC0_RA clears the conversion complete flag
    currentADC0 = ADC0->R[0];
}

/*
 * FTM1 handles the camera driving logic
 *   This ISR gets called once every integration period
 *       by the periodic interrupt timer 0 (PIT0)
 *   When it is triggered it gives the SI pulse,
 *       toggles clk for 128 cycles, and stores the line
 *       data from the ADC into the line variable
 */
void FTM1_IRQHandler(void) {
    // Pixel counter for camera logic
    // Starts at -2 so that the SI pulse occurs
    // ADC reads start
    static int pixcnt = -2;
    // clkval toggles with each FTM interrupt
    static int clkval = 0;

    // Clear interrupt
    FTM1->SC |= FTM_SC_TOF_MASK; //Pg 961 of K64P1M120SF5RM

    // Toggle clk
    if(clkval == 0) {
        clkval = 1;
        PTB->PSOR |= (1 << CAM_CLK); // CLK = 1
    } else {
        clkval = 0;
        PTB->PCOR |= (1 << CAM_CLK); // CLK = 0	
    }

    // Line capture logic
    if((pixcnt >= 2) && (pixcnt < 256)) {
        if(!clkval) {  // check for falling edge
            // ADC read (note that integer division is
            //  occurring here for indexing the array)
            line[pixcnt/2] = currentADC0;
        }
        pixcnt += 1;
    } else if(pixcnt < 2) {
        if(pixcnt == -1) {
            PTB->PSOR |= (1 << CAM_SI); // SI = 1
        } else if(pixcnt == 1) {
            PTB->PCOR |= (1 << CAM_SI); // SI = 0
            // ADC read
            line[0] = currentADC0;
        }
        pixcnt += 1;
    } else {
        PTB->PCOR |= (1 << CAM_CLK); // CLK = 0
        clkval = 0; // make sure clock variable = 0
        pixcnt = -2; // reset counter
        camReadDone = 1;
        // Disable FTM1 interrupts (until PIT0 overflows
        //   again and triggers another line capture)
        FTM1->SC &= ~(FTM_SC_TOIE_MASK);
    }
}

/* PIT0 determines the integration period
 * When it overflows, it triggers the clock logic from
 * FTM1. Note the requirement to set the MOD register
 * to reset the FTM counter because the FTM counter is
 * always counting, I am just enabling/disabling FTM1
 * interrupts to control when the line capture occurs
 */
void PIT0_IRQHandler(void) {
#ifdef DEBUG_CAM_DATA
        // Increment capture counter so that we can only
        //  send line data once every ~2 seconds
        capCnt += 1;
#endif
    // Clear interrupt
    PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK;

    // Setting mod resets the FTM counter
    FTM1->MOD = DEFAULT_SYSTEM_CLOCK/100000 - 1;

    // Enable FTM1 interrupts (camera)
    FTM1->SC |= FTM_SC_TOIE_MASK;
}

/* Initialization of FTM1 for camera */
static void camera_init_FTM1(void) {
    // Enable clock
    SIM->SCGC6 |= SIM_SCGC6_FTM1_MASK;

    // Disable Write Protection
    FTM1->MODE |= FTM_MODE_WPDIS_MASK;

    // Set output to '1' on init
    FTM1->OUTINIT |= FTM_OUTINIT_CH0OI_MASK;

    // Initialize the CNT to 0 before writing to MOD
    FTM1->CNT = 0;

    // Set the Counter Initial Value to 0
    FTM1->CNTIN = 0; // default

    // Set the period (~10us)
    FTM1->MOD = DEFAULT_SYSTEM_CLOCK/100000 - 1;

    // 50% duty
    FTM1->CONTROLS[0].CnV = (DEFAULT_SYSTEM_CLOCK/100000 - 1)/2;// Half of MOD

    // Set edge-aligned mode
    FTM1->CONTROLS[0].CnSC |= FTM_CnSC_MSB_MASK;

    // Enable High-true pulses
    // ELSB = 1, ELSA = 0
    FTM1->CONTROLS[0].CnSC |= FTM_CnSC_ELSB_MASK;

    // Enable hardware trigger from FTM1
    FTM1->EXTTRIG |= FTM_EXTTRIG_CH0TRIG_MASK;

    // Don't enable interrupts yet (disable)
    FTM1->SC &= ~(FTM_SC_TOIE_MASK);

    // No prescalar, system clock
    FTM1->SC |= FTM_SC_CLKS(0x1);

    // Set up interrupt
    NVIC_EnableIRQ(FTM1_IRQn);
}

/* Initialization of PIT timer to control
*       integration period
*/
static void camera_init_PIT0(void) {
    // Setup periodic interrupt timer (PIT)

    // Enable clock for timers
    SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;

    PIT->MCR &= ~(PIT_MCR_MDIS_MASK);

    // Enable timers to continue in debug mode
    // In case you need to debug
    PIT->MCR |= PIT_MCR_FRZ_MASK; //Pg 1086 K64 Ref Manual
    
    // PIT clock frequency is the system clock
    // Load the value that the timer will count down from
    PIT->CHANNEL[0].LDVAL = (int) (DEFAULT_SYSTEM_CLOCK * INTEGRATION_TIME);

    // Enable timer interrupts
    PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK;//Pg 1088 of K64 Ref Manual
    // Enable the timer
    PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;//Pg 1088 of K64 Ref Manual

    // Clear interrupt flag
    PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK;//Pg 1089 of K64 Ref Manual

    // Enable PIT interrupt in the interrupt controller
    NVIC_EnableIRQ(PIT0_IRQn);
}

/* Set up pins for GPIO
 * PTB9  - camera clk
 * PTB23 - camera SI
 */
static void camera_init_GPIO(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

    // Configure mux for Outputs
    PORTB->PCR[CAM_CLK] = PORT_PCR_MUX(1);
    PORTB->PCR[CAM_SI] = PORT_PCR_MUX(1);

    // Switch to output mode
    PTB->PDDR |= (1 << CAM_CLK);
    PTB->PDDR |= (1 << CAM_SI);
}

/* Set up ADC for capturing camera data */
static void camera_init_ADC0(void) {
    unsigned int calib;
    // Turn on ADC0
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;

    // Single ended 16 bit conversion, no clock divider
    ADC0->CFG1 = ADC_CFG1_MODE(0x3);

    // Do ADC Calibration for Singled Ended ADC. Do not touch.
    ADC0->SC3 = ADC_SC3_CAL_MASK;
    while ( (ADC0->SC3 & ADC_SC3_CAL_MASK) != 0 );
    calib = ADC0->CLP0;
    calib += ADC0->CLP1;
    calib += ADC0->CLP2;
    calib += ADC0->CLP3;
    calib += ADC0->CLP4;
    calib += ADC0->CLPS;
    calib = calib >> 1;
    calib |= 0x8000;
    ADC0->PG = calib;

    // Select hardware trigger.
    ADC0->SC2 = ADC_SC2_ADTRG_MASK;

    // Set to single ended mode
    ADC0->SC1[0] = ADC_SC1_AIEN_MASK | ADC_SC1_ADCH(0x00);

    // Set up FTM1 trigger on ADC0
    // FTM1 select
    // Alternative trigger en.
    // Pretrigger A
    SIM->SOPT7 |= SIM_SOPT7_ADC0TRGSEL(0x9) | SIM_SOPT7_ADC0ALTTRGEN_MASK;
    SIM->SOPT7 &= ~(SIM_SOPT7_ADC0PRETRGSEL_MASK);

    // Enable NVIC interrupt
    NVIC_EnableIRQ(ADC0_IRQn);
}
