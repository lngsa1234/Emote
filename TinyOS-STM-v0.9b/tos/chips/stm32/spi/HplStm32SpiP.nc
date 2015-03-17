
#include "Stm32Spi.h"
#include "/opt/tinyos-2.x/tos/chips/stm32/fwlib/inc/stm32f10x_map.h"
#include "/opt/tinyos-2.x/tos/chips/stm32/fwlib/inc/stm32f10x_spi.h"

module HplStm32SpiP @safe() {
  provides interface Stm32Spi as SPI;
  provides interface AsyncStdControl;
  
  uses {
    interface GeneralIO as SS;   // Slave set line
    interface GeneralIO as SCK;  // SPI clock line
    interface GeneralIO as MOSI; // Master out, slave in
    interface GeneralIO as MISO; // Master in, slave out
    interface McuPowerState as Mcu;
  }
}
implementation {

  async command error_t AsyncStdControl.start() {
    call SPI.enableSpi(TRUE);
  }

  async command error_t AsyncStdControl.stop() {
    call SPI.enableInterrupt(FALSE);
    call SPI.enableSpi(FALSE);
  }
  
  async command void SPI.initMaster() {
    call MOSI.makeOutput();
    call MISO.makeInput();
    call SCK.makeOutput();
    call SPI.setMasterBit(TRUE);
  }

  async command void SPI.initSlave() {
    call MISO.makeOutput();
    call MOSI.makeInput();
    call SCK.makeInput();
    call SS.makeInput();
    call SPI.setMasterBit(FALSE);
  }
  
  async command void SPI.sleep() {
//    call SS.set();	// why was this needed?
  }
  
  async command uint8_t SPI.read()        { return SPI_I2S_ReceiveData(SPI1); }
  async command void SPI.write(uint8_t d) { SPI_I2S_SendData(SPI1, d); }
  
  // Nived : Unsure how to write this functionality   
  default async event void SPI.dataReady(uint8_t d) {}
  /*
  AVR_ATOMIC_HANDLER(SIG_SPI) {
      signal SPI.dataReady(call SPI.read());
  }
  */

  //=== SPI Bus utility routines. ====================================
  async command bool SPI.isInterruptPending() {
//    return READ_BIT(SPSR, SPIF);
      return NVIC_GetPendingIRQ(SPI1_IRQn); 
  }

  async command bool SPI.isInterruptEnabled () {                
//    return READ_BIT(SPCR, SPIE);
      return NVIC_GetActive(SPI1_IRQn);
  }

  async command void SPI.enableInterrupt(bool enabled) {

	if(enabled)
	{
	
		NVIC_EnableIRQ(SPI1_IRQn);
		call Mcu.update();
	}
	else
	{
		NVIC_DisableIRQ(SPI1_IRQn);
		call Mcu.update();
	}	


	
    /*
    if (enabled) {
      SET_BIT(SPCR, SPIE);
      call Mcu.update();
    }
    else {
      CLR_BIT(SPCR, SPIE);
      call Mcu.update();
    }
	*/
  }

  async command bool SPI.isSpiEnabled() {
    //return READ_BIT(SPCR, SPE);
      return TRUE;
  }
  
  async command void SPI.enableSpi(bool enabled) {
    if (enabled) {
      //SET_BIT(SPCR, SPE);
      //call Mcu.update();
	SPI_Cmd(SPI1,ENABLE);
    }
    else {
      //CLR_BIT(SPCR, SPE);
      //call Mcu.update();
	SPI_Cmd(SPI1,DISABLE);
    }
  }

  /* DORD bit */
  async command void SPI.setDataOrder(bool lsbFirst) {
    if (lsbFirst) {
      //SET_BIT(SPCR, DORD);
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
    }
    else {
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
      //CLR_BIT(SPCR, DORD);
    }
  }
  
  async command bool SPI.isOrderLsbFirst() {
    return TRUE;
  }
  
  /* MSTR bit */
  async command void SPI.setMasterBit(bool isMaster) {
    if (isMaster) {
//      SET_BIT(SPCR, MSTR);
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    }
    else {
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
      //CLR_BIT(SPCR, MSTR);
    }
  }
  async command bool SPI.isMasterBitSet() {
    return TRUE;
  }
  
  /* CPOL bit */
  async command void SPI.setClockPolarity(bool highWhenIdle) {
    if (highWhenIdle) {
  //    SET_BIT(SPCR, CPOL);
    }
    else {
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
      //CLR_BIT(SPCR, CPOL);
    }
  }
  
  async command bool SPI.getClockPolarity() {
    //return READ_BIT(SPCR, CPOL);
	return TRUE;
  }
  
  /* CPHA bit */
  async command void SPI.setClockPhase(bool sampleOnTrailing) {
    if (sampleOnTrailing) {
      //SET_BIT(SPCR, CPHA);
    }
    else {
      SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
     // CLR_BIT(SPCR, CPHA);
    }
  }
  async command bool SPI.getClockPhase() {
    //return READ_BIT(SPCR, CPHA);
	return TRUE;
  }

  
  async command uint8_t SPI.getClock () {                
    return 0;
  }
  
  async command void SPI.setClock (uint8_t v) {
    //v &= (SPR1) | (SPR0);
    //SPCR = (SPCR & ~(SPR1 | SPR0)) | v;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  }

  async command bool SPI.hasWriteCollided() {
    return FALSE;
  }

  async command bool SPI.isMasterDoubleSpeed() {
    return TRUE;
  }

  async command void SPI.setMasterDoubleSpeed(bool on) {
   if (on) {
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
      //SET_BIT(SPSR, SPI2X);
    }
    else {
      //CLR_BIT(SPSR, SPI2X);
    }
  }
}
