/*
 * XMEGA_ANALOG.h
 *
 * Created: 9/20/2014 12:10:10 AM
 *  Author: jpagel
 */ 


#ifndef XMEGA_ANALOG_H_
#define XMEGA_ANALOG_H_

#ifndef F_CPU
#error F_CPU must be defined.
#endif

#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/io.h>


#define ADCACAL0_offset   0x20
#define ADCACAL1_offset   0x21


void setupADC();

uint16_t analogreadPA4(void);

void analogWritePC0(uint8_t dutycyclepercent);



uint8_t SP_ReadCalibrationByte( uint8_t index )
{
	uint8_t result;

	/* Load the NVM Command register to read the calibration row. */
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	result = pgm_read_byte(index);

	/* Clean up NVM Command register. */
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	
	return result;
}

void setupADC()
{
	
	// ADCA is enabled
	// Resolution: 8 Bits
	// Current consumption: No limit
	// Conversion mode: Signed
	ADCA.CTRLB=ADC_CURRLIMIT_NO_gc | (1<<ADC_CONMODE_bp) | ADC_RESOLUTION_12BIT_gc;
	
	// Clock frequency: 500.000 kHz
	ADCA.PRESCALER=ADC_PRESCALER_DIV8_gc;
	
	
	ADCA.CALL = SP_ReadCalibrationByte( PROD_SIGNATURES_START + ADCACAL0_offset );
	ADCA.CALH = SP_ReadCalibrationByte( PROD_SIGNATURES_START + ADCACAL1_offset );
	ADCA.CALL = SP_ReadCalibrationByte( PROD_SIGNATURES_START + ADCACAL0_offset );
	ADCA.CALH = SP_ReadCalibrationByte( PROD_SIGNATURES_START + ADCACAL1_offset );
	
	// Sampling Time: 0.5 clock cycles = 4.0 us
	//ADCA.SAMPCTRL=0x00;
	
	// Reference: Internal 1.00 V
	// Temperature reference: On
	ADCA.REFCTRL=ADC_REFSEL_INT1V_gc | (1<<ADC_TEMPREF_bp) | (1<<ADC_BANDGAP_bp);
	
	// Initialize the ADC Compare register
	ADCA.CMPL=0x00;
	ADCA.CMPH=0x00;
	
	// ADC channel 0 gain: 1
	// ADC channel 0 input mode: Differential input signal
	ADCA.CH0.CTRL=(0<<ADC_CH_START_bp) | ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFF_gc;
	
	// ADC channel 0 positive input: ADC4 pin
	// ADC channel 0 negative input: GND
	ADCA.CH0.MUXCTRL=ADC_CH_MUXPOS_PIN5_gc | ADC_CH_MUXNEG_PIN1_gc;
	
	// ADC is in Free Running mode
	// Conversions are continuously performed on channel 0
	ADCA.EVCTRL=ADC_EVACT_NONE_gc;
	
	// Free Running mode: On
	ADCA.CTRLB|=ADC_FREERUN_bm;
	
	// Enable the ADC
	ADCA.CTRLA|=ADC_ENABLE_bm;
	// Insert a delay to allow the ADC common mode voltage to stabilize
	_delay_us(2);

	
	
}

uint16_t analogreadPA4(void)
{
	uint16_t data;

	// Wait for the AD conversion to complete
	while ((ADCA.CH0.INTFLAGS & ADC_CH_CHIF_bm)==0);
	// Clear the interrupt flag
	ADCA.CH0.INTFLAGS=ADC_CH_CHIF_bm;
	// Read the AD conversion result
	data=ADCA.CH0.RES;
	//data = (8<<data);
	
	return data;
}


void analogWritePC0(uint8_t dutycyclepercent)
{
	
	//int16_t incomingdutycycle = dutycyclepercent;
	//double convertedvalue = 65200 * (float)(incomingdutycycle / 100);
	
	//PWMPC0Value =  convertedValue / (float)PWMmultiBy;
	//float convertedValue = (uint16_t)PWMPC0Value / PWMDivisBy;
	//uint16_t temp = convertedValue * PWMmultiBy;
	PORTC_DIR = 0x01;             //Set PC.0 as the output port
	TCC0_PER = 4;            //Set the period of the waveform
	TCC0_CTRLB |= 0x03;           //Single slope mode
	TCC0_CTRLB |= 0x10;           //channel selection CCAEN
	TCC0_CTRLA |= 0x02;          //clock selection clk/2
	
	
	//TCC0_CCA = 0x2000;
	TCC0_CCABUF = 2;
	
	while((TCC0_INTFLAGS & 0x01) == 0);
	
	TCC0_INTFLAGS = 0x00;
}


#endif /* XMEGA_ANALOG_H_ */