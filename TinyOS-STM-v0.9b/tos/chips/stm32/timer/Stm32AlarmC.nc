/// $Id: Stm32AlarmC.nc,v 1.8 2008/06/26 04:39:05 regehr Exp $

/*
 * Copyright (c) 2004-2005 Crossbow Technology, Inc.  All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 * 
 * IN NO EVENT SHALL CROSSBOW TECHNOLOGY OR ANY OF ITS LICENSORS BE LIABLE TO 
 * ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL 
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN
 * IF CROSSBOW OR ITS LICENSOR HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH 
 * DAMAGE. 
 *
 * CROSSBOW TECHNOLOGY AND ITS LICENSORS SPECIFICALLY DISCLAIM ALL WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
 * AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS 
 * ON AN "AS IS" BASIS, AND NEITHER CROSSBOW NOR ANY LICENSOR HAS ANY 
 * OBLIGATION TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR 
 * MODIFICATIONS.
 */

/**
 * Build a TEP102 Alarm from an Atmega128 hardware timer and one of its
 * compare registers.
 * @param frequency_tag The frequency tag for this Alarm
 * @param timer_size The width of this Alarm
 * @param mindt The shortest time in the future this Alarm can be set
 *   (in its own time units). Has to be at least 2, as setting a compare
 *   register one above the current counter value is unreliable. Has to be
 *   large enough that the Alarm time does not pass between the computation
 *   of <code>expires</code> and actually setting the compare register.
 *   Check this (for high-frequency timers) by inspecting the generated
 *   assembly code...
 *
 * @author Martin Turon <mturon@xbow.com>
 * @author David Gay <david.e.gay@intel.com>
 */

generic module Stm32AlarmC(typedef frequency_tag, 
			    typedef timer_size @integer(),
			    int mindt) @safe()
{
  provides interface Alarm<frequency_tag, timer_size> as Alarm @atmostonce();

  uses interface HplStm32Timer<timer_size>;
  uses interface HplStm32Compare<timer_size>;
}
implementation
{
  async command timer_size Alarm.getNow() {
    return call HplStm32Timer.get();
  }

  async command timer_size Alarm.getAlarm() {
    return call HplStm32Compare.get();
  }

  async command bool Alarm.isRunning() {
    return call HplStm32Compare.isOn();
  }

  async command void Alarm.stop() {
    call HplStm32Compare.stop();
  }

  async command void Alarm.start( timer_size dt ) 
  {
    call Alarm.startAt( call HplStm32Timer.get(), dt);
  }

  async command void Alarm.startAt( timer_size t0, timer_size dt ) {
    /* We don't set an interrupt before "now" + mindt to avoid setting
       an interrupt which is in the past by the time we actually set
       it. mindt should always be at least 2, because you cannot
       reliably set an interrupt one cycle in the future. mindt should
       also be large enough to cover the execution time of this
       function. */
    atomic
      {
	timer_size now, elapsed, expires;

	dbg("Stm32AlarmC", "   starting timer at %llu with dt %llu\n",
	    (uint64_t)t0, (uint64_t) dt);

	now = call HplStm32Timer.get();
	elapsed = now + mindt - t0;
	if (elapsed >= dt)
	  expires = now + mindt;
	else
	  expires = t0 + dt;

	/* Setting the compare register to "-1" is a bad idea
	   (interrupt fires before counter overflow is detected, and all
	   the "current time" stuff goes bad) */
	if (expires == 0)
	  expires = 1;

	/* Note: all HplStm32Compare.set values have one subtracted,
	   because the comparisons are continuous, but the actual
	   interrupt is signalled at the next timer clock cycle. */
	call HplStm32Compare.set(expires - 1);
	call HplStm32Compare.reset();
	call HplStm32Compare.start();
      }
  }

  async event void HplStm32Compare.fired() {
    call HplStm32Compare.stop();
    dbg("Stm32AlarmC", " Compare fired, signal alarm above.\n");
    __nesc_enable_interrupt();
    signal Alarm.fired();
  }

  async event void HplStm32Timer.overflow() {
  }
}
