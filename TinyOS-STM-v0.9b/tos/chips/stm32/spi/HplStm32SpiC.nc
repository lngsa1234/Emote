/**
 * Configuration encapsulating the basic SPI HPL for the stm32.
 *
 * @author Nived Sivadas
 */

configuration HplStm32SpiC {
	provides interface Stm32Spi as SpiBus;
}
implementation
{
	components HplSTM32GeneralIOC as IO, HplStm32SpiP as HplSpi;
	components McuSleepC;

	SpiBus = HplSpi;

	HplSpi.Mcu -> McuSleepC;
  	HplSpi.SS   -> IO.PortA4;  // Slave set line
  	HplSpi.SCK  -> IO.PortA5;  // SPI clock line
 	HplSpi.MOSI -> IO.PortA7;  // Master out, slave in
 	HplSpi.MISO -> IO.PortA6;  // Master in, slave out
	
}
