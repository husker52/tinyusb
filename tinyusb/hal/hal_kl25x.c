/**************************************************************************/
/*!
    @file     hal_lpc11uxx.c
    @author   hathach (tinyusb.org)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, hathach (tinyusb.org)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    INCLUDING NEGLIGENCE OR OTHERWISE ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    This file is part of the tinyusb stack.
*/
/**************************************************************************/

#include "common/common.h"
#include "hal.h"

#if TUSB_CFG_MCU == MCU_KL25X
extern uint32_t *BD;

extern 
tusb_error_t hal_init(void)
{
	// TODO remove magic number
  /* Enable AHB clock to the USB block and USB RAM. */
//  LPC_SYSCON->SYSAHBCLKCTRL |= ((0x1<<14) | (0x1<<27));
//  LPC_SYSCON->PDRUNCFG &= ~( BIT_(8) | BIT_(10) ); // enable USB PLL & USB transceiver

  /* Pull-down is needed, or internally, VBUS will be floating. This is to
  address the wrong status in VBUSDebouncing bit in CmdStatus register.  */
  // set PIO0_3 as USB_VBUS
//  LPC_IOCON->PIO0_3   &= ~0x1F;
//  LPC_IOCON->PIO0_3   |= (0x01<<0)  | (1 << 3);            /* Secondary function VBUS */

  // set PIO0_6 as usb connect
//  LPC_IOCON->PIO0_6   &= ~0x07;
//  LPC_IOCON->PIO0_6   |= (0x01<<0);            /* Secondary function SoftConn */

    /* Enable all clocks needed for USB to function                             */
    /* Set USB clock to 48 MHz                                                  */
    SIM->SOPT2   |=   SIM_SOPT2_USBSRC_MASK     | /* MCGPLLCLK used as src      */
                      SIM_SOPT2_PLLFLLSEL_MASK  ; /* Select MCGPLLCLK as clock  */
#if defined(TARGET_MK20D5)
    SIM->CLKDIV2 &= ~(SIM_CLKDIV2_USBFRAC_MASK  | /* Clear CLKDIV2 FS values    */
                      SIM_CLKDIV2_USBDIV_MASK);
    SIM->CLKDIV2  =   SIM_CLKDIV2_USBDIV(0)     ; /* USB clk = (PLL*1/2)        */
    /*         = ( 48*1/1)=48     */
#endif // TARGET_MK20D5
    SIM->SCGC4   |=   SIM_SCGC4_USBOTG_MASK;      /* Enable USBOTG clock        */
    //NVIC_EnableIRQ(USB0_IRQn);            /* Enable OTG interrupt               */
    USB0->USBTRC0 |= USB_USBTRC0_USBRESET_MASK;

    while (USB0->USBTRC0 & USB_USBTRC0_USBRESET_MASK);

    USB0->ISTAT   = 0xFF;                 /* clear interrupt flags              */
    /* enable interrupts                                                        */
    USB0->INTEN = USB_INTEN_USBRSTEN_MASK |
            USB_INTEN_TOKDNEEN_MASK |
            USB_INTEN_SLEEPEN_MASK ;
#if 0
#ifdef __RTX
            ((USBD_RTX_DevTask   != 0) ? USB_INTEN_SOFTOKEN_MASK : 0) |
            ((USBD_RTX_DevTask   != 0) ? USB_INTEN_ERROREN_MASK  : 0) ;
#else
            ((USBD_P_SOF_Event   != 0) ? USB_INTEN_SOFTOKEN_MASK : 0) |
            ((USBD_P_Error_Event != 0) ? USB_INTEN_ERROREN_MASK  : 0) ;
#endif
#endif
    USB0->USBCTRL  = USB_USBCTRL_PDE_MASK;/* pull dawn on D+ and D-             */
    USB0->USBTRC0 |= (1 << 6);            /* bit 6 must be set to 1             */

  return TUSB_ERROR_NONE;
}


void USB0_IRQHandler(void)
{
  tusb_isr(0);
}

#endif
