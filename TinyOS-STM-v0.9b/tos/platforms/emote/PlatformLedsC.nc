/**
 * "Copyright (c) 2009 The Regents of the University  of California.  
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 * 
 * IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF
 * CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 */

/**
 * @author Thomas Schmid
 */
#include "hardware.h"

configuration PlatformLedsC
{
  provides 
  {
      interface GeneralIO as Led0;
      interface GeneralIO as Led1;
      interface GeneralIO as Led2;
      interface GeneralIO as Led3;
  }
  uses 
  {
      interface Init;
  }
}

implementation
{
  components HplSTM32GeneralIOC as GeneralIOC;
  components PlatformP;

  Init = PlatformP.MoteInit;

  Led0 = GeneralIOC.PortF6;
  Led1 = GeneralIOC.PortF7;
  Led2 = GeneralIOC.PortF8;
  Led3 = GeneralIOC.PortF9;
}
