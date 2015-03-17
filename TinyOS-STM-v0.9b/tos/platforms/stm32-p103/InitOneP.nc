// $Id: InitOneP.nc,v 1.4 2006/12/12 18:23:42 vlahan Exp $
/*
 * Copyright (c) 2005-2006 Intel Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached INTEL-LICENSE     
 * file. If you do not find these files, copies can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA, 
 * 94704.  Attention:  Intel License Inquiry.
 */
/**
 * Internal mica-family timer component. Sets up hardware timer 1 to run
 * at cpu clock / 256, at boot time. Assumes an ~8MHz CPU clock, replace
 * this component if you are running at a radically different frequency.
 *
 * @author David Gay
 */

//#include <MicaTimer.h>

configuration InitOneP { }
implementation {
  components PlatformC, HplStm32Timer3C as HWTimer,
    new Stm32TimerInitC(uint16_t, 0x2) as InitOne;

  PlatformC.SubInit -> InitOne;
  InitOne.Timer -> HWTimer;
}