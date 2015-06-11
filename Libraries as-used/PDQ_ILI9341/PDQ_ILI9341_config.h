//
// PDQ_ILI9341 configuration
//
// You need to include this file above #include "PDQ_ILI9341.h" in your sketch.
// Check settings on lines marked with "<=".

// NOTE: These are typical hookups individual boards will vary, please check your documentation.
// CAUTION: While Adafruit boards generally always come with needed level-converters, I find many
//          other LCD displays advertised as supporting 5V only support 5V power (with a regulator).
//          They still only have 3.3V safe logic (CS, DC, RESET, MOSI, SCK marked with * below).
//          If this is the case you will need a voltage level-converter (e.g., HC4050, divider circuit etc.).
//
// LCD PIN	Uno (328)	Leo (32u4)
// -------	---------	----------
// 1  VCC	 3.3V/5V	 3.3V/5V		// +3.3V or 5V with on-board regulator
// 2  GND	   GND		   GND
// 3* CS	   10		   10			// Could be any GPIO pin, but then need to make sure SS isn't a LOW input (or slave SPI mode)
// 4* RESET    0, 8 or RESET   0,8 or RESET		// 0 is soft-reset, RESET is Arduino reset.
// 5* DC/RS	    9  		    9			// Could be any GPIO pin
// 6* SDI/MOSI	   11		  ICSP4			// HW SPI pin (can't change)
// 7* SCK	   13		  ICSP3			// HW SPI pin (can't change) NOTE: On Uno this causes on-board LED to flicker during SPI use
// 8* LED	 3.3V/5V	 3.3V/5V		// LCD screen blanked when LOW (could use GPIO for PWM dimming)
// 9  SDO/MISO      -		    -			// (not used if present, LCD code is currently "write only")
//
//  * = Typically only 3.3V safe logic-line (unless board has level converter [ala Adafruit]). Be careful with 5V!

#define	ILI9341_CS_PIN		10			// <= /CS pin (chip-select, LOW to get attention of ST7735, HIGH and it ignores SPI bus)
#define	ILI9341_DC_PIN		9			// <= DC pin (1=data or 0=command indicator line) also called RS
//#define	ILI9341_RST_PIN		8			// <= RST pin (optional)
// (other pins used are dictated by AVR HW SPI used as shown above)

// other PDQ library options
#define	ST7735_SAVE_SPCR	0			// <= 0/1 with 1 to save/restore AVR SPI control register (to "play nice" when other SPI use)
