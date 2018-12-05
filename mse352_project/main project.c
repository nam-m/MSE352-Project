#include <stdint.h>
#include <stdbool.h>
#include <math.h>
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
//#include "driverlib/interrupt.h"

#define PWM_FREQUENCY 20000 //fan frequency = 20Khz
//All are declared as 'volatile' so that each variable cannot be optimized out
//by the compiler and will be available to the
//'Expression' or 'Local' window(s) at run-time
volatile double ui32Load;
volatile double ui32PWMClock;
volatile double ui32Adjust;
volatile double duty_cycle;
volatile double result0; //tachometer reading from ADC0
volatile double result1; //12-bit digital potentiometer reading from analog signal (3.3V from MCU) ADC1
volatile double ui32Period;
volatile uint32_t first; //first digital value
volatile uint32_t next; //next digital value
volatile double start; //start timer value
volatile double end; //end timer value
volatile double count; //edge counts
volatile double period;
volatile double voltage;
volatile double speed; //in rpm
volatile double avg_speed; //average speed in rpm
volatile double ref_speed; //reference speed in rpm, set by potentiometer
volatile double max_speed; //max speed in rpm
volatile double total; //total speed before taking average
volatile double error; //difference between reference and signal speed
volatile double i = 1;
volatile double ratio;
volatile double Kp = 0.025; //proportional constant
volatile uint32_t dispAvg; //speed display in 32-bit integer

void writeTo7Seg(double num,uint32_t port1,uint32_t port2,uint32_t port3,uint32_t port4,uint8_t pin1,uint8_t pin2,uint8_t pin3,uint8_t pin4);
double digitAtPos(uint32_t binary, uint8_t pos, uint8_t base);
void P_duty_cycle(double result1, double ratio, double Kp, double ui32Adjust, double ui32Load, double avg_speed, double max_speed);
double constraints(double ui32Adjust);

int main(void)
{
    ratio = 2400.0/4095.0;
    //enable clock to run at 40MHz
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    //SYSCTL_SYSDIV_5: system clock divider 1-64
    //SYSCTL_USE_PLL:  generate an accurate output signal of frequency equal to, or a multiple of, the input signal frequency
    //SYSCTL_XTAL_16MHZ: external crystal frequency
    //SYSCTL_OSC_MAIN: oscillator source

    //enable the GPIO peripheral and configure the pins as outputs
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //enable port E
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //enable port A
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //enable port B
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_2);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); //enable port C
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_4);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); //enable port D
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_7);

    //PWM Speed Control
    ui32Adjust = 500;
    duty_cycle = (1000-ui32Adjust)/10; //inverted signal, as seen from oscilloscope
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
    ui32Load = (ui32PWMClock / PWM_FREQUENCY); //subtract 1 since counter counts to 0
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN); //configure module 1 PWM generator 0 as a down-counter
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load); //load the count value

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui32Adjust * ui32Load / 1000); //set pulse width, PWM Load is divided by 1000 and multiplied by adjusting value
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);

    //Initialize clock for result
    // enable clocks
    SYSCTL_RCGCGPIO_R |= 0x10;      // enable clock to Port E (AIN0 is on PE3)
    SYSCTL_RCGCADC_R |= 0x0001;     // enable clock to ADC0   //SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    // Wait for the ADC0 module to be ready.

    // initialize PE3 for AIN0 input
//  GPIO_PORTE_DIR_R &= ~0x08;      // make PE3 input //GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_3);
    GPIO_PORTE_AFSEL_R |= 0x08;     // enable alternate function on PE3
    GPIO_PORTE_DEN_R &= ~0x08;      // disable digital I/O on PE3
    GPIO_PORTE_AMSEL_R |= 0x08;     // enable analog functionality on PE3

    //initialize ADC0
    ADC0_ACTSS_R &= ~0x0008;        // disable SS3 during configuration
    ADC0_EMUX_R &= ~0xF000;         // software trigger conversion
    ADC0_SSMUX3_R = 0;              // get input from channel 0  //channel # = AIN#
    ADC0_SSCTL3_R |= 0x0006;        // take one sample at a time, set flag at 1st sample
    ADC0_ACTSS_R |= 0x0008;         // enable ADC0 sequencer 3 //ADCSequenceEnable(ADC0_BASE, 3);

    //ADC1 Configuration
    SYSCTL_RCGCGPIO_R |= 0x10;      // enable clock to Port E (AIN1 is on PE2)
    SYSCTL_RCGCADC_R |= 0x0002;     // enable clock to ADC1
    // initialize PE2 for AIN0 input
//  GPIO_PORTE_DIR_R &= ~0x04;
    GPIO_PORTE_AFSEL_R |= 0x04;
    GPIO_PORTE_DEN_R &= ~0x04;
    GPIO_PORTE_AMSEL_R |= 0x04;

    /* initialize ADC1 */
    ADC1_ACTSS_R &= ~0x0008;
    ADC1_EMUX_R &= ~0xF000;
    ADC1_SSMUX3_R = 1;              //channel 1 (AIN1)
    ADC1_SSCTL3_R |= 0x0006;
    ADC1_ACTSS_R |= 0x0008;

    //configures Timer 0 as a 32-bit timer in periodic mode
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    ui32Period = SysCtlClockGet(); //period for fan = period for crystal clock
    //subtract 1 from the timer period since the interrupt fires at the zero count.
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);

    // Get result from ADC0
    ADC0_PSSI_R |= 0x0008; /* start a conversion sequence 3 */
    while ((ADC0_RIS_R & 0x08) == 0); /* wait for conversion complete */
    result0 = ADC0_SSFIFO3_R; /* read conversion result */ //ADCSequenceOverflow
    ADC0_ISC_R = 0x0008; /* clear completion flag */

    //Get input digital signal before while loop
    if (result0 > 2000) //compare with average of max & min digital values
        first = 1;
    else
        first = 0;

    while(1)
    {
        if(avg_speed > 1000)
            dispAvg = avg_speed/10;
        else
            dispAvg = avg_speed;

        // Get result from ADC0
        ADC0_PSSI_R |= 0x0008; // start a conversion sequence 3
        while ((ADC0_RIS_R & 0x08) == 0); // wait for conversion complete
        result0 = ADC0_SSFIFO3_R; // read conversion result //ADCSequenceOverflow
        ADC0_ISC_R = 0x0008; // clear completion flag

        // Get result from ADC1
        ADC1_PSSI_R |= 0x0008; // start a conversion sequence 3
        while ((ADC1_RIS_R & 0x08) == 0); // wait for conversion complete
        result1 = ADC1_SSFIFO3_R; // read conversion result //ADCSequenceOverflow
        voltage = (3.3/4095)*result1; // 12-bit digital to analog signal voltage
        ADC1_ISC_R = 0x0008; // clear completion flag

        if (result0 > 2000)
            next = 1;
        else
            next = 0;
        if (next != first)
        {
            //First edge
            if (count == 0)
            {
                //Start timer
                TimerEnable(TIMER0_BASE, TIMER_A);
                start = TimerValueGet(TIMER0_BASE, TIMER_A); //Take time stamp of first edge
            }
            count++;
            if (count == 3) //3 edge changes = 1 period
            {
                end = TimerValueGet(TIMER0_BASE, TIMER_A); //Take time stamp of 5th rising edge
                count = 0;
                period = start - end; //Time between two edges calculated is the period
                speed = 60/(period/ui32Period); //rpm conversion
                total += speed;
                i++;
                if (i==3)
                {
                    avg_speed = total/i;
                    i=0;
                    total=0;
                }
                max_speed = 2400; //fan maximum speed in rpm
                P_duty_cycle(result1, ratio, Kp, ui32Adjust, ui32Load, avg_speed, 2400); //error calculation
                ui32Adjust = ui32Adjust - error;
                ui32Adjust = constraints(ui32Adjust); //set limits for duty cycle
                duty_cycle = (1000-ui32Adjust)/100;
                PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui32Adjust * ui32Load / 1000.0); //set duty cycle
                TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1); //set timer

                if (duty_cycle < 25) //set display to 000 when fan stops
                    avg_speed = 0;

                //Display speed on three 7-segments
                writeTo7Seg(digitAtPos(dispAvg, 0, 10), GPIO_PORTB_BASE, GPIO_PORTE_BASE, GPIO_PORTD_BASE, GPIO_PORTC_BASE, GPIO_PIN_2, GPIO_PIN_0, GPIO_PIN_7, GPIO_PIN_5);
                writeTo7Seg(digitAtPos(dispAvg, 1, 10), GPIO_PORTE_BASE, GPIO_PORTE_BASE, GPIO_PORTB_BASE, GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_4, GPIO_PIN_5);
                writeTo7Seg(digitAtPos(dispAvg, 2, 10), GPIO_PORTC_BASE, GPIO_PORTA_BASE, GPIO_PORTA_BASE, GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4, GPIO_PIN_3, GPIO_PIN_2);
            }
        }
        if (result0 > 2000) //compare with average of max & min digital values, repeat the comparison
            first = 1;
        else
            first = 0;

// writeTo7Seg() - Takes a single digit and outputs the binary representation into the set pins
// Input: single digit 0-9
void writeTo7Seg(double num, uint32_t port1, uint32_t port2, uint32_t port3, uint32_t port4, uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4)
{
    uint8_t base = 2;
    if(digitAtPos(num, 0, base))
        GPIOPinWrite(port1, pin1, pin1);
    else
        GPIOPinWrite(port1, pin1, 0x00);

    if(digitAtPos(num, 1,base))
        GPIOPinWrite(port2, pin2, pin2);
    else
        GPIOPinWrite(port2, pin2, 0x00);

    if(digitAtPos(num, 2, base))
        GPIOPinWrite(port3, pin3, pin3);
    else
        GPIOPinWrite(port3, pin3, 0x00);

    if(digitAtPos(num, 3, base))
        GPIOPinWrite(port4, pin4, pin4);
    else
        GPIOPinWrite(port4, pin4, 0x00);
}

// binaryDigitAtPos() - Takes a binary number and determines the digit at the specific position
// Input: binary - a byte value of the binary number (binary = B0110 ex)
//        pos - the position of the binary digit wanted
// Return: A 1/0 depending on the position in the binary representation of the byte number
double digitAtPos(uint32_t binary, uint8_t pos, uint8_t base)
{
    uint8_t denom = 1;
    uint8_t i = 0;
    for (i = 0; i < pos; i++)
        denom = denom * base;
    return ((binary / denom) % base);
}
void P_duty_cycle(double result1, double ratio, double Kp, double avg_speed, double max_speed)
{
    ref_speed = result1 * ratio;
    error = Kp * ((ref_speed - avg_speed) / max_speed) * 1000.0; //calculate error of control system
}
double constraints(double ui32Adjust)
{
    if (ui32Adjust < 10)
        ui32Adjust = 10;
    if (ui32Adjust > 1000)
        ui32Adjust = 1000;
    return ui32Adjust;
}
