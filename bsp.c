#include "TM4C123GH6PM.h"
#include "ti_bsp.h"

#define my_assert(cond) \
	do { \
		if(!(cond)) \
			__BKPT(3); \
	}while(0);

#define SYSCTL_RCC_BYPASS       0x00000800
#define SYSCTL_RCC_USESYSDIV    0x00400000  // Enable System Clock Divider
#define SYSCTL_RCC2_BYPASS2     0x00000800  // PLL Bypass 2
#define SYSCTL_RCC_MOSCDIS      0x00000001  // Main Oscillator Disable
#define SYSCTL_MAIN_OSC_DIS     0x00000001  // Disable main oscillator
#define SYSCTL_MISC_MOSCPUPMIS  0x00000100  // MOSC Power Up Masked Interrupt
                                            // Status
#define SYSCTL_RIS_MOSCPUPRIS   0x00000100  // MOSC Power Up Raw Interrupt
                                            // Status
#define SYSCTL_RCC_XTAL_M       0x000007C0  // Crystal Value
#define SYSCTL_RCC_OSCSRC_M     0x00000030  // Oscillator Source
#define SYSCTL_RCC2_USERCC2     0x80000000  // Use RCC2
#define SYSCTL_RCC2_OSCSRC2_M   0x00000070  // Oscillator Source 2
#define SYSCTL_RCC_PWRDN        0x00002000  // PLL Power Down
#define SYSCTL_RCC2_PWRDN2      0x00002000  // Power-Down PLL 2
#define SYSCTL_MISC_PLLLMIS     0x00000040  // PLL Lock Masked Interrupt Status
#define SYSCTL_RCC_SYSDIV_M     0x07800000  // System Clock Divisor
#define SYSCTL_RCC2_SYSDIV2_M   0x1F800000  // System Clock Divisor 2
#define SYSCTL_RCC2_DIV400      0x40000000  // Divide PLL as 400 MHz vs. 200
                                            // MHz
#define SYSCTL_RCC2_SYSDIV2LSB  0x00400000  // Additional LSB for SYSDIV2
#define SYSCTL_PLLSTAT_LOCK     0x00000001  // PLL Lock

#define GPIO_UNLOCK_KEY (uint32_t)0x4C4F434B

//*****************************************************************************
//
// Holds the current, debounced state of each button.  A 0 in a bit indicates
// that that button is currently pressed, otherwise it is released.
// We assume that we start with all the buttons released (though if one is
// pressed when the application starts, this will be detected).
//
//*****************************************************************************
static uint8_t g_ButtonStates = ALL_BUTTONS;

void bsp_delay(uint32_t delay)
{
	volatile uint32_t delay_cnt = delay;
	
	while(delay_cnt)
	{
		delay_cnt--;
	}
}
	
/***GPIO programming sequence from data sheet***************
1. Enable the clock to the port by setting the appropriate bits in the RCGCGPIO register (see
page 340). In addition, the SCGCGPIO and DCGCGPIO registers can be programmed in the
same manner to enable clocking in Sleep and Deep-Sleep modes.

2. Set the direction of the GPIO port pins by programming the GPIODIR register. A write of a 1
indicates output and a write of a 0 indicates input.

3. Configure the GPIOAFSEL register to program each bit as a GPIO or alternate pin. If an alternate
pin is chosen for a bit, then the PMCx field must be programmed in the GPIOPCTL register for
the specific peripheral required. There are also two registers, GPIOADCCTL and GPIODMACTL,
which can be used to program a GPIO pin as a ADC or µDMA trigger, respectively.

4. Set the drive strength for each of the pins through the GPIODR2R, GPIODR4R, and GPIODR8R
registers.

5. Program each pad in the port to have either pull-up, pull-down, or open drain functionality through
the GPIOPUR, GPIOPDR, GPIOODR register. Slew rate may also be programmed, if needed,
through the GPIOSLR register.

6. To enable GPIO pins as digital I/Os, set the appropriate DEN bit in the GPIODEN register. To
enable GPIO pins to their analog function (if available), set the GPIOAMSEL bit in the
GPIOAMSEL register.

7. Program the GPIOIS, GPIOIBE, GPIOEV, and GPIOIM registers to configure the type, event,
and mask of the interrupts for each port.
Note: To prevent false interrupts, the following steps should be taken when re-configuring
GPIO edge and interrupt sense registers:
	a. Mask the corresponding port by clearing the IME field in the GPIOIM register.
	b. Configure the IS field in the GPIOIS register and the IBE field in the GPIOIBE
			register.
	c. Clear the GPIORIS register.
	d. Unmask the port by setting the IME field in the GPIOIM register.

8. Optionally, software can lock the configurations of the NMI and JTAG/SWD pins on the GPIO
port pins, by setting the LOCK bits in the GPIOLOCK register.

When the internal POR signal is asserted and until otherwise clock_on, all GPIO pins are clock_on
to be undriven (tristate): GPIOAFSEL=0, GPIODEN=0, GPIOPDR=0, and GPIOPUR=0, except for
the pins shown in Table 10-1 on page 650. Table 10-3 on page 657 shows all possible configurations
of the GPIO pads and the control register settings required to achieve them. Table 10-4 on page 658
shows how a rising edge interrupt is clock_on for pin 2 of a GPIO port.
**********************************************/
	
static GPIOA_Type* const GPIO_base[GPIO_BUS_MAX][GPIO_PORT_MAX] =
{
	// GPIO access using APB
	{
		GPIOA,
		GPIOB,
		GPIOC,
		GPIOD,
		GPIOE,
		GPIOF
	},
	// GPIO access using AHB
	{
		GPIOA_AHB,
		GPIOB_AHB,
		GPIOC_AHB,
		GPIOD_AHB,
		GPIOE_AHB,
		GPIOF_AHB
	}		
};

typedef struct _GPIO_state{
	GPIOA_Type* GPIO_CTL;
	uint8_t clock_on:1,
					unused:7;
}GPIO_state;

static GPIO_state s_gpio_state[GPIO_PORT_MAX];

static void GPIO_set_pad_config(GPIOA_Type* gpio_ctrl, uint8_t pin_mask, uint32_t strength, uint32_t pin_type)
{
	my_assert(pin_mask & 0xF);
	my_assert(strength < GPIO_STRENGTH_MAX);
	my_assert(pin_type < GPIO_PIN_TYPE_MAX);
	
	//set the drive strength
	if(strength & GPIO_STRENGTH_2MA)
	{
		gpio_ctrl->DR2R |= (pin_mask & 0xF);
	}
	else
	{
		gpio_ctrl->DR2R &= ~(pin_mask & 0xF);
	}
	
	if(strength & GPIO_STRENGTH_4MA)
	{
		gpio_ctrl->DR4R |= (pin_mask & 0xF);
	}
	else
	{
		gpio_ctrl->DR4R &= ~(pin_mask & 0xF);
	}
	
	if(strength & GPIO_STRENGTH_8MA)
	{
		gpio_ctrl->DR8R |= (pin_mask & 0xF);
	}
	else
	{
		gpio_ctrl->DR8R &= ~(pin_mask & 0xFF);
	}
	
	if(strength & GPIO_STRENGTH_8MA)
	{
		gpio_ctrl->DR8R |= (pin_mask & 0xFF);
	}
	else
	{
		gpio_ctrl->DR8R &= ~(pin_mask & 0xF);
	}
	
	if(strength & GPIO_STRENGTH_8MA_SC)
	{
		gpio_ctrl->SLR |= (pin_mask & 0xF);
	}
	else
	{
		gpio_ctrl->SLR &= ~(pin_mask & 0xF);
	}
	
	//set the pintype configs
	if(pin_type & GPIO_PIN_TYPE_DEN) //push-pull confg, digital enable
	{
		gpio_ctrl->DEN |= (pin_mask & 0xF);
	}
	else
	{
		gpio_ctrl->DEN &= ~(pin_mask & 0xF);
	}
	
	if(pin_type & GPIO_PIN_TYPE_OD) //open drain confg.
	{
		gpio_ctrl->ODR |= (pin_mask & 0xF);
	}
	else
	{
		gpio_ctrl->ODR &= ~(pin_mask & 0xF);
	}
	
	if(pin_type & GPIO_PIN_TYPE_WPD) //weak pull down confg.
	{
		gpio_ctrl->PDR |= (pin_mask & 0xF);
	}
	else
	{
		gpio_ctrl->PDR &= ~(pin_mask & 0xF);
	}
	
	if(pin_type & GPIO_PIN_TYPE_WPU) //weak pullup confg.
	{
		gpio_ctrl->PUR |= (pin_mask & 0xF);
	}
	else
	{
		gpio_ctrl->PUR &= ~(pin_mask & 0xF);
	}
}

static void GPIO_enable_clock(uint32_t gpio_port)
{
	my_assert(gpio_port < GPIO_PORT_MAX);
	
	if(!s_gpio_state[gpio_port].clock_on)
	{
		uint32_t port_mask;
		
		port_mask = (1U << (gpio_port & 0xF));
		
		SYSCTL->RCGCGPIO |= port_mask;
		
		while(!(SYSCTL->RCGCGPIO & port_mask));
		
		s_gpio_state[gpio_port].clock_on = 1;
	}
}

static void GPIO_set_direction(GPIOA_Type* gpio_ctrl, uint32_t pin_mask, uint32_t direction)
{
	my_assert(pin_mask & 0xFF);
	
	if(direction & GPIO_DIR_OUT)
	{
		gpio_ctrl->DIR |= (pin_mask & 0xF);
	}
	else if (direction & GPIO_DIR_IN)
	{
		gpio_ctrl->DIR &= ~(pin_mask & 0xF);
	}
	
	if( direction & GPIO_DIR_AF)
	{
		gpio_ctrl->AFSEL |= (pin_mask & 0xF);
	}
	else
	{
		gpio_ctrl->AFSEL &= ~(pin_mask & 0xF);
	}
}

static GPIOA_Type* GPIO_get_ctrl_reg(uint32_t gpio_port, uint32_t bus_type) 
{	
	my_assert(gpio_port < GPIO_PORT_MAX);
	my_assert(bus_type < GPIO_BUS_MAX);
	
	if(NULL == (s_gpio_state[gpio_port].GPIO_CTL))
	{
		s_gpio_state[gpio_port].GPIO_CTL = GPIO_base[bus_type][gpio_port];
	}
	return s_gpio_state[gpio_port].GPIO_CTL;
}

static void GPIO_pin_config_std_output(GPIOA_Type* gpio_ctrl, uint32_t pin_mask)
{
	//set direction
	GPIO_set_direction(gpio_ctrl, pin_mask, GPIO_DIR_OUT);
	//configure pad
	GPIO_set_pad_config(gpio_ctrl, pin_mask, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_DEN);
}

static void GPIO_pin_config_std_intput(GPIOA_Type* gpio_ctrl, uint32_t pin_mask)
{
	//set direction
	GPIO_set_direction(gpio_ctrl, pin_mask, GPIO_DIR_IN);
	//configure pad
	GPIO_set_pad_config(gpio_ctrl, pin_mask, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_DEN|GPIO_PIN_TYPE_WPU);
}

void GPIO_pin_toggle(uint32_t gpio_port, uint32_t pin_mask)
{
	GPIOA_Type* GPIO_CTL;
	uint32_t data;
	
	pin_mask = pin_mask & 0xF;

	my_assert(gpio_port < GPIO_PORT_MAX);
	my_assert(s_gpio_state[gpio_port].clock_on && s_gpio_state[gpio_port].GPIO_CTL);
	my_assert(pin_mask < GPIO_PIN_MAX);
	
	GPIO_CTL = s_gpio_state[gpio_port].GPIO_CTL;

	data = GPIO_CTL->DATA;
	
	data ^= pin_mask;
	
	GPIO_CTL->DATA = data;
}

void GPIO_pin_write(uint32_t gpio_port, uint32_t pin_mask, uint8_t overwrite)
{
	GPIOA_Type* GPIO_CTL;
	
	pin_mask = pin_mask & 0xF;

	my_assert(gpio_port < GPIO_PORT_MAX);
	my_assert(s_gpio_state[gpio_port].clock_on && s_gpio_state[gpio_port].GPIO_CTL);
	
	GPIO_CTL = s_gpio_state[gpio_port].GPIO_CTL;
	
	if(overwrite)
	{
		GPIO_CTL->DATA = pin_mask;
	}
	else
	{
		GPIO_CTL->DATA |= pin_mask;
	}
}

uint32_t GPIO_pin_read(uint32_t gpio_port, uint32_t pin_mask)
{
	GPIOA_Type* GPIO_CTL;
	uint32_t data;
	
	pin_mask = pin_mask & 0xF;
	
	my_assert(gpio_port < GPIO_PORT_MAX);
	my_assert(s_gpio_state[gpio_port].clock_on && s_gpio_state[gpio_port].GPIO_CTL);
	
	GPIO_CTL = s_gpio_state[gpio_port].GPIO_CTL;
	
	data = GPIO_CTL->DATA;
	
	return ( data & pin_mask);
}

void GPIO_led_toggle(uint32_t led, uint32_t delay)
{	
	GPIO_pin_write(GPIO_PORT_F, led, 1);
	bsp_delay(delay);
	GPIO_pin_write(GPIO_PORT_F, 0, 1);
}

void GPIO_led_configure(uint8_t bus_type)
{
	GPIOA_Type* GPIO_CTL = GPIO_get_ctrl_reg(GPIO_LED_PORT, bus_type);
	
  //1. Enable the clock to the port by setting the appropriate bits in the RCGCGPIO register
	GPIO_enable_clock(GPIO_LED_PORT);
	
	//2. Set the direction of the GPIO port pins by programming the GPIODIR register
	GPIO_pin_config_std_output(GPIO_CTL, ALL_LED);
	
}

//*****************************************************************************
//
//! Polls the current state of the buttons and determines which have changed.
//!
//! \param pDelta points to a character that will be written to indicate
//! which button states changed since the last time this function was called.
//! This value is derived from the debounced state of the buttons.
//! \param pRawState points to a location where the raw button state will
//! be stored.
//!
//! This function should be called periodically by the application to poll the
//! pushbuttons.  It determines both the current debounced state of the buttons
//! and also which buttons have changed state since the last time the function
//! was called.
//!
//! In order for button debouncing to work properly, this function should be
//! caled at a regular interval, even if the state of the buttons is not needed
//! that often.
//!
//! If button debouncing is not required, the the caller can pass a pointer
//! for the \e pRawState parameter in order to get the raw state of the
//! buttons.  The value returned in \e pRawState will be a bit mask where
//! a 1 indicates the buttons is pressed.
//!
//! \return Returns the current debounced state of the buttons where a 1 in the
//! button ID's position indicates that the button is pressed and a 0
//! indicates that it is released.
//
//*****************************************************************************
uint8_t Buttons_poll(uint8_t *pDelta, uint8_t *pRawState)
{
    uint32_t Delta;
    uint32_t Data;
    static uint8_t SwitchClockA = 0;
    static uint8_t SwitchClockB = 0;

    //
    // Read the raw state of the push buttons.  Save the raw state
    // (inverting the bit sense) if the caller supplied storage for the
    // raw value.
    //
    Data = GPIO_pin_read(GPIO_BUTTON_PORT, ALL_BUTTONS);
	
    if(pRawState)
    {
        *pRawState = (uint8_t)~Data;
    }

    //
    // Determine the switches that are at a different state than the debounced
    // state.
    //
    Delta = Data ^ g_ButtonStates;

    //
    // Increment the clocks by one.
    //
    SwitchClockA ^= SwitchClockB;
    SwitchClockB = ~SwitchClockB; 

    //
    // Reset the clocks corresponding to switches that have not changed state.
    //
    SwitchClockA &= Delta;
    SwitchClockB &= Delta;

    //
    // Get the new debounced switch state.
    //
    g_ButtonStates &= SwitchClockA | SwitchClockB;
    g_ButtonStates |= (~(SwitchClockA | SwitchClockB)) & Data;

    //
    // Determine the switches that just changed debounced state.
    //
    Delta ^= (SwitchClockA | SwitchClockB);

    //
    // Store the bit mask for the buttons that have changed for return to
    // caller.
    //
    if(pDelta)
    {
        *pDelta = (uint8_t)Delta;
    }

    //
    // Return the debounced buttons states to the caller.  Invert the bit
    // sense so that a '1' indicates the button is pressed, which is a
    // sensible way to interpret the return value.
    //
    return ~g_ButtonStates;
}

void Buttons_configure(uint8_t bus_type)
{
	GPIOA_Type* GPIO_CTL = GPIO_get_ctrl_reg(GPIO_BUTTON_PORT, bus_type);
	
	GPIO_enable_clock(GPIO_BUTTON_PORT);
	
	GPIO_CTL->LOCK = GPIO_UNLOCK_KEY;
	
	GPIO_CTL->CR |= ALL_BUTTONS;
	
	GPIO_CTL->LOCK = 0;
	
	GPIO_pin_config_std_intput(GPIO_CTL, ALL_BUTTONS);
	
	//read the initial state of Buttons
	g_ButtonStates = GPIO_pin_read(GPIO_BUTTON_PORT, ALL_BUTTONS);
}

void bsp_clock_set(uint32_t Config)
{
    uint32_t Delay, RCC, RCC2;

    //
    // Get the current value of the RCC and RCC2 registers.
    //
    RCC = SYSCTL->RCC;
    RCC2 = SYSCTL->RCC2;

    //
    // Bypass the PLL and system clock dividers for now.
    //
    RCC |= SYSCTL_RCC_BYPASS;
    RCC &= ~(SYSCTL_RCC_USESYSDIV);
    RCC2 |= SYSCTL_RCC2_BYPASS2;

    //
    // Write the new RCC value.
    //
    SYSCTL->RCC = RCC;
    SYSCTL->RCC2 = RCC2;

    //
    // See if the oscillator needs to be enabled.
    //
    if((RCC & SYSCTL_RCC_MOSCDIS) && !(Config & SYSCTL_MAIN_OSC_DIS))
    {
        //
        // Make sure that the required oscillators are enabled.  For now, the
        // previously enabled oscillators must be enabled along with the newly
        // requested oscillators.
        //
        RCC &= (~SYSCTL_RCC_MOSCDIS | (Config & SYSCTL_MAIN_OSC_DIS));

        //
        // Clear the MOSC power up raw interrupt status to be sure it is not
        // set when waiting below.
        //
        SYSCTL->MISC = SYSCTL_MISC_MOSCPUPMIS;

        //
        // Write the new RCC value.
        //
        SYSCTL->RCC = RCC;

        //
        // Timeout using the legacy delay value.
        //
        Delay = 524288;

        while((SYSCTL->RIS & SYSCTL_RIS_MOSCPUPRIS) == 0)
        {
            Delay--;

            if(Delay == 0)
            {
                break;
            }
        }

        //
        // If the main oscillator failed to start up then do not switch to
        // it and return.
        //
        if(Delay == 0)
        {
            return;
        }

    }

    //
    // Set the new crystal value and oscillator source.  Because the OSCSRC2
    // field in RCC2 overlaps the XTAL field in RCC, the OSCSRC field has a
    // special encoding within Config to avoid the overlap.
    //
    RCC &= ~(SYSCTL_RCC_XTAL_M | SYSCTL_RCC_OSCSRC_M);
    RCC |= Config & (SYSCTL_RCC_XTAL_M | SYSCTL_RCC_OSCSRC_M);
    RCC2 &= ~(SYSCTL_RCC2_USERCC2 | SYSCTL_RCC2_OSCSRC2_M);
    RCC2 |= Config & (SYSCTL_RCC2_USERCC2 | SYSCTL_RCC_OSCSRC_M);
    RCC2 |= (Config & 0x00000008) << 3;

    //
    // Write the new RCC value.
    //
    SYSCTL->RCC = RCC;
    SYSCTL->RCC2 = RCC2;

    //
    // Set the PLL configuration.
    //
    RCC &= ~SYSCTL_RCC_PWRDN;
    RCC |= Config & SYSCTL_RCC_PWRDN;
    RCC2 &= ~SYSCTL_RCC2_PWRDN2;
    RCC2 |= Config & SYSCTL_RCC2_PWRDN2;

    //
    // Clear the PLL lock interrupt.
    //
    SYSCTL->MISC = SYSCTL_MISC_PLLLMIS;

    //
    // Write the new RCC value.
    //
    if(RCC2 & SYSCTL_RCC2_USERCC2)
    {
        SYSCTL->RCC2 = RCC2;
        SYSCTL->RCC = RCC;
    }
    else
    {
        SYSCTL->RCC = RCC;
        SYSCTL->RCC2 = RCC2;
    }

    //
    // Set the requested system divider and disable the appropriate
    // oscillators.  This value is not written immediately.
    //
    RCC &= ~(SYSCTL_RCC_SYSDIV_M | SYSCTL_RCC_USESYSDIV |
                 SYSCTL_RCC_MOSCDIS);
    RCC |= Config & (SYSCTL_RCC_SYSDIV_M | SYSCTL_RCC_USESYSDIV |
                             SYSCTL_RCC_MOSCDIS);
    RCC2 &= ~(SYSCTL_RCC2_SYSDIV2_M);
    RCC2 |= Config & SYSCTL_RCC2_SYSDIV2_M;
    if(Config & SYSCTL_RCC2_DIV400)
    {
        RCC |= SYSCTL_RCC_USESYSDIV;
        RCC2 &= ~(SYSCTL_RCC_USESYSDIV);
        RCC2 |= Config & (SYSCTL_RCC2_DIV400 | SYSCTL_RCC2_SYSDIV2LSB);
    }
    else
    {
        RCC2 &= ~(SYSCTL_RCC2_DIV400);
    }

    //
    // See if the PLL output is being used to clock the system.
    //
    if(!(Config & SYSCTL_RCC_BYPASS))
    {
        //
        // Wait until the PLL has locked.
        //
        for(Delay = 32768; Delay > 0; Delay--)
        {
            if((SYSCTL->PLLSTAT & SYSCTL_PLLSTAT_LOCK))
            {
                break;
            }
        }

        //
        // Enable use of the PLL.
        //
        RCC &= ~(SYSCTL_RCC_BYPASS);
        RCC2 &= ~(SYSCTL_RCC2_BYPASS2);
    }

    //
    // Write the final RCC value.
    //
    SYSCTL->RCC = RCC;
    SYSCTL->RCC2 = RCC2;

    //
    // Delay for a little bit so that the system divider takes effect.
    //
    bsp_delay(16);
}





