/**
 * "Copyright (c) 2009 The Regents of the University of California.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement
 * is hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 *
 * IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
 * OF CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 */

/**
 * Generic bit access for pins mapped into I/O space.
 *
 * TODO:
 *  - More configuration options for the IOs are possible, but
 *    currently not implemented.
 *
 * @author Thomas Schmid
 */
#include <stm32hardware.h>

generic module HplSTM32GeneralIOPinP (uint32_t port_addr, uint8_t bit) @safe()
{
    provides interface GeneralIO as IO;
}
implementation
{
    inline async command bool IO.get() {
        GPIO_TypeDef* port = (GPIO_TypeDef*)port_addr;
        // this depends on the mode
        if(call IO.isInput())
            return (port->IDR >> bit) & 0x01;
        else
            return (port->ODR >> bit) & 0x01;
    }
    inline async command void IO.set() { 
        GPIO_TypeDef* port = (GPIO_TypeDef*)port_addr;
	
	// setting the gpio pin
	port->BSRR = (1 << bit);
	//GPIOF->BSRR = (1 << bit);
    }

    inline async command void IO.clr() {
        GPIO_TypeDef* port = (GPIO_TypeDef*)port_addr;
	
	// resetting the gpio pin
	port->BRR = (1 << bit);
	//GPIOF->BRR = (1 << bit);
    }

    async command void IO.toggle() {
        GPIO_TypeDef* port = (GPIO_TypeDef*)port_addr;

	// set and reset depending on the status of the odr register	
	if(port->ODR & (1 << bit))
		port->BRR = (1 << bit);
	else
		port->BSRR = (1 << bit);
	
    }

    inline async command void IO.makeInput()  {
        GPIO_TypeDef* port = (GPIO_TypeDef*)port_addr;

        GPIO_InitTypeDef gpioi = {
            (uint16_t) 1 << bit, // select the pin
            GPIO_Speed_50MHz,
            GPIO_Mode_IN_FLOATING
        };
        GPIO_Init(port, &gpioi);
    }

    inline async command bool IO.isInput() {
       //GPIO_TypeDef* port = (GPIO_TypeDef*)port_addr;
	return TRUE;
    }

    inline async command void IO.makeOutput() {
        GPIO_TypeDef* port = (GPIO_TypeDef*)port_addr;

        GPIO_InitTypeDef gpioi = {
            (uint16_t) 1 << bit, // select the pin
            GPIO_Speed_50MHz,
            GPIO_Mode_Out_PP
        };
        GPIO_Init(port, &gpioi);
    }

    inline async command bool IO.isOutput() {
        GPIO_TypeDef* port = (GPIO_TypeDef*)port_addr;
	return FALSE;
    }
}

