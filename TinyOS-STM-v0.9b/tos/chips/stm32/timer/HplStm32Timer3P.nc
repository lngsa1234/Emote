/**
 * Internal component of the HPL interface to Stm32 timer 1.
 *
 *	Nived Sivadas (nived.sivadas@samrkash.com)
 */

#include <STM32Timer.h>

// Channel Two or Comparator B in Input Capture Mode

module HplStm32Timer3P @safe()
{
  provides {
    // 16-bit Timers
    interface HplStm32Timer<uint16_t>   as Timer;
    interface HplStm32Capture<uint16_t> as Capture;
    interface HplStm32Compare<uint16_t> as CompareA;
    interface HplStm32Compare<uint16_t> as CompareC;
  }
  //uses interface HplStm32TimerCtrl8     as Timer0Ctrl;
}
implementation
{
  //=== Read the current timer value. ===================================
  async command uint16_t Timer.get() { return TIM_GetCounter(TIM3); }

  //=== Set/clear the current timer value. ==============================
  async command void Timer.set(uint16_t t) { TIM_SetCounter(TIM3,t); }

  //=== Read the current timer scale. ===================================
  async command uint8_t Timer.getScale() { return TIM_GetPrescalar(TIM3); }

  async command void Capture.setEdge(bool up){}

  //=== Turn off the timers. ============================================
  async command void Timer.off() {  }

  //=== Write a new timer scale. ========================================
  async command void Timer.setScale(uint8_t s)  { 

  }


  //=== Timer 16-bit implementation. ===================================
  async command void Timer.reset()    {  }
  async command void Capture.reset()  {  }
  async command void CompareA.reset() {  }
  async command void CompareC.reset() {  }

  async command void Timer.start()    {  }
  async command void Capture.start()  {  }
  async command void CompareA.start() {  }
  async command void CompareC.start() {  }

  async command void Timer.stop()    {  }
  async command void Capture.stop()  {  }
  async command void CompareA.stop() {  }
  async command void CompareC.stop() { }

  // Note: Many Timer interrupt flags are on Timer0 register
  async command bool Timer.test() { 
//    return (call Timer0Ctrl.getInterruptFlag()).bits.tov1; 
      return TRUE;
  }
  async command bool Capture.test()  { 
//    return (call Timer0Ctrl.getInterruptFlag()).bits.icf1; 
      return TRUE;
  }
  async command bool CompareA.test() { 
//    return (call Timer0Ctrl.getInterruptFlag()).bits.ocf1a; 
	return TRUE;
  }
  async command bool CompareC.test() { 
//    return (call TimerCtrl.getInterruptFlag()).bits.ocf1c; 
	return TRUE;
  }

  // Note: Many Timer interrupt mask bits are on Timer0 register
  async command bool Timer.isOn() {
//    return (call Timer0Ctrl.getInterruptMask()).bits.toie1;
	return TRUE;
  }
  async command bool Capture.isOn()  {
//    return (call Timer0Ctrl.getInterruptMask()).bits.ticie1;
	return TRUE;
  }
  async command bool CompareA.isOn() {
//    return (call Timer0Ctrl.getInterruptMask()).bits.ocie1a;
	return TRUE;
  }
  async command bool CompareC.isOn() {
    //return (call TimerCtrl.getInterruptMask()).bits.ocie1c;
	return TRUE;
  }

  //=== Read the compare registers. =====================================
  async command uint16_t CompareA.get() { return TIM_GetCapture1(TIM3); }
  async command uint16_t CompareC.get() { return TIM_GetCapture3(TIM3); }

  //=== Write the compare registers. ====================================
  async command void CompareA.set(uint16_t t) { TIM_SetCompare1(TIM3,t); }
  async command void CompareC.set(uint16_t t) { TIM_SetCompare3(TIM3,t); }

  //=== Read the capture registers. =====================================
  async command uint16_t Capture.get() { return TIM_GetCapture2(TIM3); }

  //=== Write the capture registers. ====================================
  async command void Capture.set(uint16_t t)  { 
	TIM_SetCompare2(TIM3,t); 
  }

  //=== Timer interrupts signals ========================================
  default async event void CompareA.fired() { }
  AVR_NONATOMIC_HANDLER(SIG_OUTPUT_COMPARE1A) {
    signal CompareA.fired();
  }
  default async event void CompareC.fired() { }
  AVR_NONATOMIC_HANDLER(SIG_OUTPUT_COMPARE1C) {
    signal CompareC.fired();
  }
  default async event void Capture.captured(uint16_t time) { }
  AVR_NONATOMIC_HANDLER(SIG_INPUT_CAPTURE1) {
    signal Capture.captured(call Timer.get());
  }
  default async event void Timer.overflow() { }
  AVR_NONATOMIC_HANDLER(SIG_OVERFLOW1) {
    signal Timer.overflow();
  }
}
