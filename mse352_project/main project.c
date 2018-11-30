#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
//#include "PLL.h"

#define PWM_FREQUENCY 20000 //fan frequency = 20Mhz
#define TIMER0_TAILR_R
volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
volatile uint32_t ui32Adjust; //change to uint32_t for 32 bit registers
volatile double duty_cycle;
volatile uint32_t result;
volatile uint32_t ui32Period;

//Timer
void Timer3A_countCapture_init(void);
int Timer3A_countCapture(void);
volatile uint32_t count;

int main(void)
{

    //enable clock to run at 40MHz
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    //enable the GPIO peripheral and configure the pins as outputs
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //enable port E
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    //configures Timer 0 as a 32-bit timer in periodic mode
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    //First, calculate the number of clock cycles required for a ? Hz period by calling SysCtlClockGet() and dividing it by your desired frequency
    //Then divide that by two, since we want a count that is ½ of that for the interrupt.
    ui32Period = (SysCtlClockGet() / 0.5);
    //subtract 1 from the timer period since the interrupt fires at the zero count.
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period -1);

    //Enable the timer. This will start the timer and interrupts will begin triggering on the timeouts
    TimerEnable(TIMER0_BASE, TIMER_A);

    //********************************************************************************************************************
    /*
    while(1)
     {
        GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_3, GPIO_PIN_1); // turn on PE3
        SysCtlDelay(7000000); // wait a while
        GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_3, 0x00);   // turn PE3 off
        SysCtlDelay(7000000);
      }
     */
    //Initialize clock for result
    /* enable clocks */
    SYSCTL_RCGCGPIO_R |= 0x10;  /* enable clock to Port E (AIN0 is on PE3) */
    SYSCTL_RCGCADC_R |= 0x0001; /* enable clock to ADC0 */ //SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    /* initialize PE3 for AIN0 input  */
    GPIO_PORTE_DIR_R &= ~0x08;      // 2) make PE3 input
    GPIO_PORTE_AFSEL_R |= 0x08;     // 3) enable alternate function on PE3
    GPIO_PORTE_DEN_R &= ~0x08;      // 4) disable digital I/O on PE3
    GPIO_PORTE_AMSEL_R |= 0x08;     // 5) enable analog functionality on PE3

    /* initialize ADC0 */
    ADC0_ACTSS_R &= ~0x0008;        /* disable SS3 during configuration */
    ADC0_EMUX_R &= ~0xF000;    /* software trigger conversion */
    ADC0_SSMUX3_R = 0;         /* get input from channel 0 */
    ADC0_SSCTL3_R = 0x0006;        /* take one sample at a time, set flag at 1st sample */
    ADC0_ACTSS_R |= 0x0008;         /* enable ADC0 sequencer 3 */

    //Timer
    Timer3A_countCapture_init();

    //PWM Speed Control
    ui32Adjust = 500; //duty cycle = (1000-ui32Adjust)/10
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1); //enable PWM output on PD0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//enable GPIOD modules on PF0 and PF4
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //configure PWM output pin on PD0
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PD0_M1PWM0);

    //unlock the GPIO commit control register
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    //configures PF0 & 4 as inputs
    GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
    //configures the internal pull-up resistors on both pins
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    ui32PWMClock = SysCtlClockGet() / 64; //determine the count to be loaded into the Load register
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1; //subtract 1 since counter counts to 0
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN); //configure module 1 PWM generator 0 as a down-counter
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load); //load the count value

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui32Adjust * ui32Load / 1000); //set pulse width, PWM Load is divided by 1000 and multiplied by adjusting value
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);
    while(1)
    {
        duty_cycle = (1000-ui32Adjust)/10;
        // Get result from ADC
        ADC0_PSSI_R = 0x0008;         /* start a conversion sequence 3 */
        while((ADC0_RIS_R&0x08) == 0) ;   /* wait for conversion complete */
        result = ADC0_SSFIFO3_R&0xFFF; /* read conversion result */
        ADC0_ISC_R = 0x0008;          /* clear completion flag */
        //
        // Clear the timer interrupt
        TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
        // Read the current state of the GPIO pin and
        // write back the opposite state

        //Count
        count=0;
        count = Timer3A_countCapture();

        if(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_3))
        {
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
        }
        else
        {
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 4);
        }
        //PWM Adjustment
        if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)==0x00)
        {
                ui32Adjust--;
                if (ui32Adjust < 10)
                {
                    ui32Adjust = 10;
                }
                PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui32Adjust * ui32Load / 1000);
        }
        if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)==0x00)
        {
            ui32Adjust++;
            if (ui32Adjust > 1000)
            {
                ui32Adjust = 1000;
            }
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui32Adjust * ui32Load / 1000);
        }
        SysCtlDelay(100000);
    }
}

void Timer3A_countCapture_init(void)
{
    SYSCTL_RCGCTIMER_R |= 0x01;     /* enable clock to Timer Block 3 */ // Use Timer3 in 32-bit periodic mode to request interrupts at a periodic rate
    SYSCTL_RCGCGPIO_R |= 0x10;      /* enable clock to PORTE */  //SYSCTL_RCGC2_R |= 0x00000010;

    GPIO_PORTE_DIR_R &= ~0x08;        /* make PE3 an input pin */
    GPIO_PORTB_DEN_R |= 0x08;         /* make PE3 a digital pin */
    GPIO_PORTB_AFSEL_R |= 0x08;      /* enable alternate function on PE3 */
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF0FFF)+0x07000000; /* configure PE3 as T3CCP0 pin */

    TIMER0_CTL_R &= ~TIMER_CTL_TAEN;          /* disable TIMER3A during setup */
    TIMER0_CFG_R = TIMER_CFG_16_BIT;            /* configure as 16-bit timer mode */
    TIMER0_TAMR_R = (TIMER_TAMR_TACMR|TIMER_TAMR_TAMR_CAP);        /* up-count, edge-count, capture mode */
    //TIMER3-&amp;gt;TAMATCHR = 0xFFFF;  /* set the count limit */
    //TIMER3-&amp;gt;TAPMR = 0xFF; /* to 0xFFFFFF with prescaler */
    TIMER0_CTL_R &= ~(TIMER_CTL_TAEVENT_POS|0xC);
    //TIMER0_TAILR_R = TIMER_TAILR_TAILRL_M;
    TIMER0_TAPR_R = 0xFF;  /* to 0xFFFFFF with prescaler */
    TIMER0_IMR_R |= TIMER_IMR_CAEIM;        /* capture the rising edge */
    TIMER0_CTL_R |= TIMER_CTL_TAEN;           /* enable timer3A */
}
int Timer3A_countCapture(void)
{
    return TIMER0_TAR_R;
}
