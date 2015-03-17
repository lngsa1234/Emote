/*
*	HPL Interface for STM32 Timer 3 
*     
*       @author Nived Sivadas <nived.sivadas@samraksh.com>
*
*/

configuration HplStm32Timer3C
{

	provides
	{
		interface HplStm32Timer<uint16_t> as Timer;
		//interface HplStm32TimerCtrl16 as TimerCtrl;
		interface HplStm32Capture<uint16_t> as Capture;
		interface HplStm32Compare<uint16_t> as Compare[uint8_t id];
	}
}
implementation
{
	//components HplStm32Timer0AsyncC, HplStm32Timer3P;
	components HplStm32Timer3P;


	Timer = HplStm32Timer3P;
	//TimerCtrl = HplStm32Timer3P;
	Capture = HplStm32Timer3P;


	Compare[0] = HplStm32Timer3P.CompareA;
	Compare[2] = HplStm32Timer3P.CompareC;


	//HplStm32Timer3P.Timer0Ctrl -> HplStm32Timer0AsyncC;



}
