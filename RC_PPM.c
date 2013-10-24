//
//  RC_PPM.c
//  
//
//  Created by Sysadmin on 20.07.13
//  Copyright Ruedi Heimlicher 2013. All rights reserved.
//


#include <string.h>
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>
#include <math.h>

#include "lcd.c"
#include "adc.c"
#include "version.c"
#include "def.h"


#include "spi_adc.c"
#include "spi_ram.c"
#include "spi_eeprom.c"

// USB
#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

#define LOOPDELAY 1

#define INT 1    //Verwendung von Integer bei Berechnung

#define SERVOMAX  2000
#define SERVOMIN  1000

volatile uint8_t do_output=0;
static volatile uint8_t buffer[32]={};
static volatile uint8_t sendbuffer[32]={};


#define TIMER0_STARTWERT	0x40

volatile uint8_t timer0startwert=TIMER0_STARTWERT;
#define USB_DATENBREITE 32

void delay_ms(unsigned int ms);

static volatile uint8_t    adcstatus=0x00;
static volatile uint8_t    usbstatus=0x00;

static volatile uint8_t    eepromstatus=0x00;
static volatile uint8_t    potstatus=0x80; // Bit 7 gesetzt, Mittelwerte setzen
static volatile uint8_t    impulscounter=0x00;

static volatile uint8_t    masterstatus = 0;
/*
 RAM_READ   Bit 0 0x01
 RAM_WRITE  Bit 1 0x02
 EEPROM_READ    Bit 2 0x04
 EEPROM_WRITE   Bit 3 0x08
 
 */

#define USB_RECV  0 

volatile uint8_t status=0;

volatile uint8_t           PWM=0;
static volatile uint8_t    pwmposition=0;
static volatile uint8_t    pwmdivider=0;


volatile char              SPI_data='0';
volatile char              SPI_dataArray[SPI_BUFSIZE];
volatile uint16_t          POT_Array[SPI_BUFSIZE];
volatile uint16_t          Servo_Array[SPI_BUFSIZE];

volatile int16_t          Servo_ArrayInt[SPI_BUFSIZE]; // signed Int

volatile uint16_t          Mitte_Array[SPI_BUFSIZE];

volatile uint16_t          RAM_Array[SPI_BUFSIZE];

volatile uint8_t          Expo_Array[SPI_BUFSIZE]; // Expo und Richtung Einstellungen pro kanal
volatile uint8_t          Level_Array[SPI_BUFSIZE]; // Ausgangslevel-Einstellungen pro kanal
volatile uint8_t          Mix_Array[SPI_BUFSIZE]; // Mixing-Einstellungen pro model


volatile uint16_t          Batteriespannung =0;
volatile short int         received=0;

volatile uint16_t          abschnittnummer=0;
volatile uint16_t          usbcount=0;

volatile uint16_t          minwert=0xFFFF;
volatile uint16_t          maxwert=0;
volatile int16_t             canalwerta=0;
volatile int16_t             canalwertb=0;
volatile uint8_t           mixcanal=0;

volatile uint16_t adcdata=0;
volatile uint16_t diffdata=0;
volatile uint8_t diffdatalo=0;
volatile uint8_t diffdatahi=0;

volatile int16_t diffdataInt=0;

volatile uint16_t diff =0;

volatile uint8_t stufe=0;

volatile    uint8_t task_in=0; // Task von RC_LCD
volatile    uint8_t task_indata=0; // Taskdata von RC_LCD


volatile    uint8_t task_out=0; // Task an RC_LCD
volatile    uint8_t task_outdata=0; // Taskdata an RC_LCD
volatile    uint8_t task_counter=0;

void startTimer2(void)
{
   //timer2
   TCNT2   = 0; 
   //	TCCR2A |= (1 << WGM21);    // Configure timer 2 for CTC mode 
   TCCR2B |= (1 << CS20);     // Start timer at Fcpu/8 
   //	TIMSK2 |= (1 << OCIE2A);   // Enable CTC interrupt 
   TIMSK2 |= (1 << TOIE2);    // Enable OV interrupt 
   //OCR2A   = 5;             // Set CTC compare value with a prescaler of 64 
   TCCR2A = 0x00;
   
   sei();
}

void stopTimer2(void)
{
   TCCR2B = 0;
}

void Master_Init(void)
{
	//OSZIPORTDDR |= (1<<PULS);	// Output
	//OSZIPORT |= (1<<PULS);		// HI
	
	LOOPLED_DDR |=(1<<LOOPLED_PIN);
	LOOPLED_PORT |= (1<<LOOPLED_PIN);	// HI
	
	//Pin 0 von   als Ausgang fuer OSZI
	OSZIPORTDDR |= (1<<OSZI_PULS_A);	//Pin 0 von  als Ausgang fuer LED TWI
   OSZIPORT |= (1<<OSZI_PULS_A);		// HI
	
   OSZIPORTDDR |= (1<<OSZI_PULS_B);		//Pin 1 von  als Ausgang fuer LED TWI
   OSZIPORT |= (1<<OSZI_PULS_B);		//Pin   von   als Ausgang fuer OSZI
   
	
   /*
	TASTENDDR &= ~(1<<TASTE0);	//Bit 0 von PORT B als Eingang fŸr Taste 0
	TASTENPORT |= (1<<TASTE0);	//Pull-up
   */
 	
   
	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);		//Pin 4 von PORT D als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 5 von PORT D als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT D als Ausgang fuer LCD
   
	
   DDRD |= (1<<PORTD6);
   PORTD |= (1<<PORTD6);
   
   ADC_DDR &= ~(1<<ADC_0);
   
   
   
   MASTER_DDR &= ~(1<<SUB_BUSY_PIN); // Eingang fuer busy-Signal von Sub
   MASTER_PORT |= (1<<SUB_BUSY_PIN); // HI
   

   //DDRE |= (1<<PORTE0);
   //PORTE |= (1<<PORTE0);
   
   MASTER_DDR |= (1<<MASTER_EN_PIN);
   MASTER_PORT |= (1<<MASTER_EN_PIN);
   
   CMD_DDR |= (1 << ALARM_PIN); // Alarm
   
   
   /**
	 * Pin Change Interrupt enable on PCINT0 (PB6)
	 */
   /*
   PCIFR |= (1<<PCIF0);
   PCICR |= (1<<PCIE0);
	PCMSK0 |= (1<<PCINT6);
*/
  
}

void SPI_PORT_Init(void) // SPI-Pins aktivieren
{
   //http://www.atmel.com/dyn/resources/prod_documents/doc2467.pdf  page:165
   //Master init
   // Set MOSI and SCK output, all others input
   SPI_DDR &= ~(1<<SPI_MISO_PIN);
   SPI_PORT &= ~(1<<SPI_MISO_PIN); // HI
   
   SPI_DDR |= (1<<SPI_MOSI_PIN);
   
   
   SPI_DDR |= (1<<SPI_SCK_PIN);
   SPI_PORT &= ~(1<<SPI_SCK_PIN); // LO
   
   SPI_DDR |= (1<<SPI_SS_PIN);
   SPI_PORT |= (1<<SPI_SS_PIN); // HI
   
}

void SPI_ADC_init(void) // SS-Pin fuer EE aktivieren
{
   
   SPI_DDR |= (1<<SPI_SS_PIN);
   SPI_PORT |= (1<<SPI_SS_PIN); // HI
   
   
   
}


void SPI_RAM_init(void) // SS-Pin fuer RAM aktivieren
{
   SPI_RAM_DDR |= (1<<SPI_RAM_CS_PIN); // RAM-CS-PIN Ausgang
   SPI_RAM_PORT |= (1<<SPI_RAM_CS_PIN);// HI
   
   SPI_PORT &= ~(1<<SPI_MISO_PIN); // LO
}

void SPI_EE_init(void) // SS-Pin fuer EE aktivieren
{
   SPI_EE_DDR |= (1<<SPI_EE_CS_PIN); // EE-CS-PIN Ausgang
   SPI_EE_PORT |= (1<<SPI_EE_CS_PIN);// HI
}

void spi_start(void) // SPI-Pins aktivieren
{
   //http://www.atmel.com/dyn/resources/prod_documents/doc2467.pdf  page:165
   //Master init
   // Set MOSI and SCK output, all others input
   SPI_DDR &= ~(1<<SPI_MISO_PIN);
   SPI_PORT &= ~(1<<SPI_MISO_PIN); // LO
   
   SPI_DDR |= (1<<SPI_MOSI_PIN);
   
   SPI_DDR |= (1<<SPI_SCK_PIN);
   SPI_PORT &= ~(1<<SPI_SCK_PIN); // LO
   
   SPI_DDR |= (1<<SPI_SS_PIN);
   SPI_PORT |= (1<<SPI_SS_PIN); // HI
   
   // RAM-CS bereit
   SPI_RAM_DDR |= (1<<SPI_RAM_CS_PIN); // RAM-CS-PIN Ausgang
   SPI_RAM_PORT |= (1<<SPI_RAM_CS_PIN);// HI

   // EE-CS bereit
   SPI_EE_DDR |= (1<<SPI_EE_CS_PIN); // EE-CS-PIN Ausgang
   SPI_EE_PORT |= (1<<SPI_EE_CS_PIN);// HI
   
   
}

void spi_end(void) // SPI-Pins deaktivieren
{
   SPCR=0;
   
   SPI_DDR &= ~(1<<SPI_MOSI_PIN); // MOSI off
   SPI_DDR &= ~(1<<SPI_SCK_PIN); // SCK off
   SPI_DDR &= ~(1<<SPI_SS_PIN); // SS off
   
   SPI_RAM_DDR &= ~(1<<SPI_RAM_CS_PIN); // RAM-CS-PIN off
   SPI_EE_DDR &= ~(1<<SPI_EE_CS_PIN); // EE-CS-PIN off
   
   
}


void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}

// http://www.co-pylit.org/courses/COSC2425/lectures/AVRNetworks/index.html

void timer2_init(void)
{
   //timer2
	TCNT2   = 0;
   //	TCCR2A |= (1 << WGM21);    // Configure timer 2 for CTC mode
	TCCR2B |= (1 << CS20);     // Start timer at Fcpu/8
   //	TIMSK2 |= (1 << OCIE2A);   // Enable CTC interrupt
	TIMSK2 |= (1 << TOIE2);    // Enable OV interrupt
	//OCR2A   = 5;             // Set CTC compare value with a prescaler of 64
   TCCR2A = 0x00;

}



void timer1_init(void)
{
   
   // Quelle http://www.mikrocontroller.net/topic/103629
   
   //OSZI_A_HI ; // Test: data fuer SR
   //_delay_us(5);
   //#define FRAME_TIME 20 // msec
   KANAL_DDR |= (1<<KANAL_PIN); // Kanal Ausgang
      
   DDRD |= (1<<PORTD5); //  Ausgang
   PORTD |= (1<<PORTD5); //  Ausgang

   //TCCR1A = (1<<COM1A0) | (1<<COM1A1);// | (1<<WGM11);	// OC1B set on match, set on TOP
   //TCCR1B = (1<<WGM13) | (1<<WGM12) ;		// TOP = ICR1, clk = sysclk/8 (->1us)
   TCCR1B |= (1<<CS11); // f/8
  // TCCR1B |= (1<<CS10); // f
   TCNT1  = 0;														// reset Timer
   
                              // Impulsdauer
   OCR1B  = 0x80;				// Impulsdauer des Kanalimpulses
   
   TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt:
   TIMSK1 |= (1 << OCIE1B);  // enable timer compare interrupt:
   
   //KANAL_PORT |= (1<<PORTB5); // Ausgang HI
   //OSZI_A_LO ;
   //OSZI_B_LO ;
   
   
   KANAL_HI;
   
   impulscounter = 0;
   //OCR1A  = POT_FAKTOR*POT_Array[1];
   //OCR1A  = POT_FAKTOR*POT_Array[impulscounter];
   
   if (INT)
   {
      if (Servo_ArrayInt[impulscounter] > SERVOMAX)
      {
         Servo_ArrayInt[impulscounter] = SERVOMAX;
      }
      
      if (Servo_ArrayInt[impulscounter] < SERVOMIN)
      {
         Servo_ArrayInt[impulscounter] = SERVOMIN;
      }
      
      
      OCR1A  = Servo_ArrayInt[impulscounter]; // POT_Faktor schon nach ADC
      
   }
   else
   {
      
      if (Servo_Array[impulscounter] > SERVOMAX)
      {
         Servo_Array[impulscounter] = SERVOMAX;
      }
      
      if (Servo_Array[impulscounter]<  SERVOMIN)
      {
         Servo_Array[impulscounter] = SERVOMIN;
      }
      
      
      OCR1A  = Servo_Array[impulscounter]; //
   }
   


} // end timer1

void timer1_stop(void)
{
   TCCR1B = 0;
   
}




ISR(TIMER1_COMPA_vect)	 //Ende der Pulslaenge fuer einen Kanal
{
   
   
   impulscounter++;
   
   if (impulscounter < ANZ_POT)
   {
      // Start Impuls
      KANAL_HI;
      TCNT1  = 0;
      //KANAL_HI;
      
      // Laenge des naechsten Impuls setzen
      
      //OCR1A  = POT_FAKTOR*POT_Array[1]; // 18 us
      //OCR1A  = POT_FAKTOR*POT_Array[impulscounter]; // 18 us
      
      if (INT)
      {
         
         if (Servo_ArrayInt[impulscounter] > SERVOMAX)
         {
            Servo_ArrayInt[impulscounter] = SERVOMAX;
         }
         
         if (Servo_ArrayInt[impulscounter] < SERVOMIN)
         {
            Servo_ArrayInt[impulscounter] = SERVOMIN;
         }
         
         
         OCR1A  = Servo_ArrayInt[impulscounter]; // POT_Faktor schon nach ADC
         
      }
      else
      {
         
         if (Servo_Array[impulscounter] > SERVOMAX)
         {
            Servo_Array[impulscounter] = SERVOMAX;
         }
         
         if (Servo_Array[impulscounter]<  SERVOMIN)
         {
            Servo_Array[impulscounter] = SERVOMIN;
         }
         
         
         OCR1A  = Servo_Array[impulscounter]; //
      }
      
      
      
   }
   else
   {
      // Ende Impulspaket
      //OCR1A  = 0x4FF;
      //OSZI_A_HI ;
     // _delay_us(200);
      //KANAL_LO;

      // Alle Impulse gesendet, Timer1 stop. Timer1 wird bei Beginn des naechsten Paketes wieder gestartet
      timer1_stop();
      
      // SPI fuer device ausschalten
      spi_end();
      potstatus |= (1<<SPI_END); //       
      //OSZI_B_HI ;
      //OSZI_A_LO ;
      
      //MASTER_PORT &= ~(1<<MASTER_EN_PIN); // Master schickt Enable an Slave
      _delay_us(2);
      //PORTE |= (1<<PORTE0);
      MASTER_PORT |= (1<<MASTER_EN_PIN);
      
   }
   
   //OSZI_B_LO ;
   //_delay_us(10);
   
   
}


ISR(TIMER1_COMPB_vect)	 //Ende des Kanalimpuls. ca 0.3 ms
{
   //OSZI_A_LO ;
   //PORTB &= ~(1<<PORTB5); // OC1A Ausgang
   //OSZI_A_HI ;
   KANAL_LO;
   
   if (impulscounter < ANZ_POT)
   {
   }
   else
   {
      timer1_stop();
     // OSZI_B_HI ;
            
   }
}


void timer0 (void) // nicht verwendet
{
// Timer fuer Exp
	//TCCR0 |= (1<<CS01);						// clock	/8
	//TCCR0 |= (1<<CS01)|(1<<CS02);			// clock	/64
	TCCR0B |= (1<<CS02)| (1<<CS02);			// clock	/256
	//TCCR0 |= (1<<CS00)|(1<<CS02);			// clock /1024
	TCCR0B |= (1 << CS10); // Set up timer 
	OCR0A = 0x02;
	
	//TIFR |= (1<<TOV0);							//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK0 |= (1<<TOIE0);							//Overflow Interrupt aktivieren
	TCNT0 = TIMER0_STARTWERT;					//RŸcksetzen des Timers

}

/*
void timer2 (uint8_t wert) 
{ 
//	TCCR2 |= (1<<CS02);							//8-Bit Timer, Timer clock = system clock/256

//Takt fuer Servo
	TCCR2 |= (1<<CS20)|(1<<CS21);				//Takt /64	Intervall 64 us

	TCCR2 |= (1<<WGM21);							//	ClearTimerOnCompareMatch CTC

	//OC2 akt
//	TCCR2 |= (1<<COM20);						//	OC2 Pin zuruecksetzen bei CTC


	TIFR |= (1<<TOV2);							//Clear TOV2 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK0 |= (1<<OCIE2);							//CTC Interrupt aktivieren

	TCNT2 = 0x00;									//Zaehler zuruecksetzen
	
	OCR2 = wert;									//Setzen des Compare Registers auf Servoimpulsdauer
} 
*/

volatile uint16_t timer2Counter=0;
volatile uint16_t timer2BatterieCounter=0;

ISR (TIMER2_OVF_vect) 
{ 
	timer2Counter ++;
   if (timer2Counter >= 0x01A0) // Laenge des Impulspakets 20ms Atmega328
//	if (timer2Counter >= 0x474) // Laenge des Impulspakets 20ms teenysy
	{
      
      //if (MASTER_PIN & (1<<SUB_BUSY_PIN))
      {
         potstatus |= (1<<SPI_START); // Potentiometer messen
         
         masterstatus |= (1<< POT_READ);
         
         timer2BatterieCounter++; // Intervall fuer Messung der Batteriespannung
         if (timer2BatterieCounter >= 0xF)
         {
            adcstatus |= (1<<ADC_START); // Batteriespannung messen
            timer2BatterieCounter = 0;
         }
      }
		timer2Counter = 0;
      
	}
	TCNT2 = 10;							// ergibt 2 kHz fuer Timertakt
   //OSZI_A_LO ;
}

/*
ISR(TIMER2_COMP_vect) // Schaltet Impuls an SERVOPIN0 aus
{
//		lcd_clr_line(1);
//		lcd_puts("Timer2 Comp\0");
		TCCR20=0;
}
*/


//https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328


ISR (PCINT0_vect)
{
   
   if(INTERRUPT_PIN & (1<< SUB_BUSY_PIN))// LOW to HIGH pin change, Sub wieder ON
   {
      OSZI_B_HI;
      masterstatus &= ~(1<<SUB_BUSY_BIT); // Master ist wieder frei
      
   }
   else // HIGH to LOW pin change, Sub OFF
   {
      OSZI_B_LO;
      
      masterstatus |= (1<<SUB_BUSY_BIT); // Master muss warten

   }
   
}


void setMitte(void)
{
   for (uint8_t i=0;i< SPI_BUFSIZE;i++)
   {
      Mitte_Array[i] = POT_Array[i];
   }
}

// MARK: readSettings
void readSettings(uint8_t modelindex)
{
  
   uint8_t pos=0;
    
    // Level lesen
    uint16_t readstartadresse = TASK_OFFSET  + LEVEL_OFFSET + modelindex*SETTINGBREITE;
    // startadresse fuer Settings des models
    for (pos=0;pos<8;pos++)
    {
       Level_Array[pos] = spieeprom_rdbyte(readstartadresse+pos);
    }
    
    // Expo lesen
    readstartadresse = TASK_OFFSET  + EXPO_OFFSET + modelindex*SETTINGBREITE;
    for (pos=0;pos<8;pos++)
    {
       Expo_Array[pos] = spieeprom_rdbyte(readstartadresse+pos);
    }
   
   // Mix lesen
    readstartadresse = TASK_OFFSET  + MIX_OFFSET + modelindex*SETTINGBREITE;
    for (pos=0;pos<8;pos++)
    {
       Mix_Array[pos] = spieeprom_rdbyte(readstartadresse+pos); // data 0: kanaele data 1: mixart
    }
/*
   lcd_gotoxy(0,0);
   lcd_putc('E');
   lcd_putc(' ');
   lcd_puthex(Expo_Array[0]);
   lcd_puthex(Expo_Array[1]);
   
   lcd_putc(' ');
   lcd_putc('L');
   lcd_putc(' ');
   lcd_puthex(Level_Array[0]);
   lcd_puthex(Level_Array[1]);
   lcd_putc(' ');
 */

   lcd_gotoxy(0,1);
   lcd_putc('A');
   //lcd_putc(' ');
   lcd_putint12(readstartadresse);
   lcd_putc(' ');
   lcd_putc('M');
   //lcd_putc(' ');
   lcd_puthex(Mix_Array[0]);
   lcd_puthex(Mix_Array[1]);
   lcd_putc(' ');
   lcd_puthex(Mix_Array[2]);
   lcd_puthex(Mix_Array[3]);


   lcd_putc('+');
  
   uint8_t levela = Level_Array[0]& 0x07;
   uint8_t levelb = (Level_Array[0]& 0x70)>>4;
   
   /*
   lcd_gotoxy(6,1);
   lcd_putc('A');
   
   lcd_puthex(levela);
    lcd_putc(' ');
   lcd_putc('B');
   
   lcd_puthex(levelb);
    */

}

#pragma mark - main
int main (void)
{

   uint16_t count=0;
    
	// set for 16 MHz clock
	CPU_PRESCALE(0);
    
	// Initialize the USB, and then wait for the host to set configuration.
	// If the Teensy is powered without a PC connected to the USB port,
	// this will wait forever.
   
	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(100);

	sei();
	
	
	Master_Init();
	
   
   //SPI_PORT_Init(); //Pins fuer SPI aktivieren, incl. SS
   
   //SPI_RAM_init(); // SS-Pin fuer RAM aktivieren
   
   //SPI_EE_init();
   
      
   volatile    uint8_t outcounter=0;
   volatile    uint8_t testdata =0x00;
   volatile    uint8_t testaddress =0x00;
   volatile    uint8_t errcount =0x00;
   volatile    uint8_t ram_indata=0;
   

   volatile    uint8_t eeprom_indata=0;
   volatile    uint8_t eeprom_testdata =0x00;
   volatile    uint8_t eeprom_testaddress =0x00;

  	/* initialize the LCD */
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);

	lcd_puts("Guten Tag\0");
	delay_ms(100);
	lcd_cls();
	//lcd_puts("READY\0");
	lcd_puts("V: \0");
	lcd_puts(VERSION);
   lcd_clr_line(1);
   delay_ms(100);
   lcd_clr_line(0);
	uint8_t Tastenwert=0;
	uint8_t TastaturCount=0;
	
	uint16_t TastenStatus=0;
	uint16_t Tastencount=0;
	uint16_t Tastenprellen=0x01F;
	//timer0();
	
	initADC(0);
	//wdt_enable(WDTO_2S);
	
	uint16_t loopcount0=0;
	uint8_t loopcount1=0;

	
	
	
	/*
	Bit 0: 1 wenn wdt ausgelšst wurde
	 
	  */ 
	uint8_t i=0;
   
   timer2_init();

   sei();
   
   PWM = 0;
   uint8_t n=0;
   for (n=0;n<8;n++)
   {
      Mitte_Array[n] = MITTE;
   }
   
   char* versionstring = (char*) malloc(4);
   strncpy(versionstring, VERSION+9, 3);
   versionstring[3]='\0';
   volatile uint16_t versionint = atoi(versionstring);
   volatile uint8_t versionintl = versionint & 0x00FF;
   //versionint >>=8;
   volatile uint8_t versioninth = (versionint & 0xFF00)>>8;
   
   //eepromstatus |= (1<<EE_READ_SETTINGS); // Settings lesen beim Start
   
   uint8_t anzeigecounter=0;
   
// MARK: while
	while (1)
	{
      //OSZI_B_LO;
		//Blinkanzeige
		loopcount0+=1;
		if (loopcount0==0x4FFF)
		{
			loopcount0=0;
			loopcount1+=1;
			LOOPLED_PORT ^=(1<<LOOPLED_PIN);
         
         if  (masterstatus & (1<<ALARM_BIT))
         {
            CMD_PORT ^= (1<<ALARM_PIN);
         }
         else
         {
            CMD_PORT &= ~(1<<ALARM_PIN);
         }
          
         // Messung anzeigen
         if (loopcount1%0x2F == 0)
         {
            //eepromstatus |= (1<<EE_READ_SETTINGS); // Settings lesen beim Start
            //lcd_gotoxy(0,1);
            /*
            lcd_putint12(adcdata);
            
            lcd_putc('*');
             */
            //lcd_putint12(diff);
            //lcd_putc('*');
            
            //lcd_putint12(diffdata);
            //lcd_putc('*');
             
            /*
            lcd_gotoxy(0,1);
            lcd_putint12Bit(maxwert);
            lcd_putc('*');
            lcd_putint12Bit(minwert);
            lcd_putc('*');
             */
         }
         
         // neue Daten in sendbuffer
         for (int i=0;i<8;i++)
         {
            sendbuffer[8+2*i]=(POT_Array[i] & 0xFF);        // 8  10 12
            sendbuffer[8+2*i+1]= (POT_Array[i]>>8) & 0xFF;  // 9  11 13
         }
         //OSZI_B_LO;
         
          
		//OSZI_B_HI;
      } // if loopcount0
      
#pragma mark HALT
      
      if (MASTER_PIN & (1<<SUB_BUSY_PIN))
      {
         masterstatus &= ~(1<<HALT);
      }
      else
      {
         OSZI_A_TOGG ;
         
         spi_end();
         masterstatus |= (1<<HALT);
         MASTER_PORT |= (1<<MASTER_EN_PIN);
         //cli();
        
         timer2Counter=0;
         //continue;
     
      }
      
      
      /**	ADC	***********************/

     if (adcstatus & (1<< ADC_START)) // ADC starten
      {
       }
      
      if (potstatus & (1<< SPI_START)) // SPI starten, in TIMER2_OVF_vect gesetzt
      {
         masterstatus &= ~(1<< POT_READ);
         
         MASTER_PORT |= (1<<MASTER_EN_PIN); // Sub abstellen
         _delay_us(2);
         
         
         potstatus &= ~(1<< SPI_START);   // Bit zuruecksetzen
         
         uint8_t i=0;
         
         //if (MASTER_PIN & (1<<SUB_BUSY_PIN))
         {
            spi_start();
             SPI_ADC_init();
            spiadc_init();
            
            for(i=0;i< ANZ_POT;i++)
            {
               
               if ((i<2) )
               {
                  
                  //POT_Array[i] = 0x600;
                  
                  cli();
                  POT_Array[i] = POT_FAKTOR*MCP3208_spiRead(SingleEnd,i);
                  sei();
                  // POT_Array[i] = 0x600;
                  
                  
                  if (POT_Array[i]==0)
                  {
                     //OSZI_A_LO ;
                     errcount++;
                     masterstatus |= (1<<ALARM_BIT);
                     
                     POT_Array[i] = 0x600;
                     
                  }
                  
                  // Filter
                  //POT_Array[i] = 3*POT_Array[i]/4 + (MCP3208_spiRead(SingleEnd,i)/4);
                  //POT_Array[i] = POT_FAKTOR*(1*POT_Array[i]/2 + (MCP3208_spiRead(SingleEnd,i)/2));
                  _delay_us(2); // war mal 100
                  //OSZI_A_HI ;
               }
               else
               {
                  POT_Array[i] = 0x600;
               }
            }
            anzeigecounter++;
         }
         // Mittelwert speichern
         
         if (potstatus & (1<< POT_MITTE))
         {
            setMitte();
            potstatus &= ~(1<< POT_MITTE);
         }
         
         // TASK-Daten von RAM lesen
         
         // MARK: RAM
         
         // Daten zu POT-Werten aus eeprom
         //
         MASTER_PORT &= ~(1<<MASTER_EN_PIN);
         
         _delay_us(5);
         SPI_PORT_Init();
         
         SPI_EE_init();
         spieeprom_init();
         cli();
         
         // Servodaten aufbereiten ohne Mix
         for(i=0;i< 8;i++)
         {
            if ((i < 6) ) // 2 Steuerknueppel und 2 Schieber
            {
               uint8_t levela = Level_Array[i]& 0x0F;
               uint8_t levelb = (Level_Array[i]& 0xF0)>>4;
               
               uint8_t richtung = Expo_Array[i] & 0x80; // bit 7
               
               uint8_t stufea = Expo_Array[i] & 0x03;
               uint8_t stufeb = (Expo_Array[i] & 0x30) >>4;
                              
               uint16_t mitte = Mitte_Array[i];
               
               diff=mitte;
               adcdata = POT_Array[i];
               stufe = 0;
               
               if (adcdata > mitte) // Seite A
               {
                  diff = adcdata - mitte;
                  diffdatalo = (uint8_t)spieeprom_rdbyte(stufea*STUFENOFFSET + 2*diff);
                  diffdatahi = (uint8_t)spieeprom_rdbyte(stufea*STUFENOFFSET + 2*diff +1);
                  
                  
               }
               else // Seite B
               {
                  diff = mitte - adcdata;
                  diffdatalo = (uint8_t)spieeprom_rdbyte(stufeb*STUFENOFFSET + 2*diff);
                  diffdatahi = (uint8_t)spieeprom_rdbyte(stufeb*STUFENOFFSET + 2*diff +1);
               }
               // MARK: Integer
               
               // start signed int
               
               if (INT)
               {
                  diffdataInt = diffdatalo | (diffdatahi <<8);
                  
                  
                  if (adcdata > mitte) // positiver Wert
                  {
                     diffdataInt *= (8-levela);
                     diffdataInt /= 8;
                     
                  }
                  else // negativer Wert
                  {
                     diffdataInt *= (8-levelb);
                     diffdataInt /= 8;
                     
                     diffdataInt *= (-1);
                  }
                  if (richtung) // Richtung umkehren
                  {
                     diffdataInt *= (-1);
                  }
                  
                  Servo_ArrayInt[i] = diffdataInt; // mit Vorzeichen
                  
                  
                  // end signed int
               }
               else
               {
                  
                  diffdata = diffdatalo | (diffdatahi <<8);
                  
                  if ((adcdata > mitte) )
                  {
                     diffdata *= (8-levela);
                     diffdata /= 8;
                     
                     if (richtung)
                     {
                        Servo_Array[i] = mitte - diffdata;
                     }
                     else
                     {
                        Servo_Array[i] = mitte + diffdata;
                     }
                     
                     
                  }
                  else
                  {
                     diffdata *= (8-levelb);
                     diffdata /= 8;
                     
                     if (richtung)
                     {
                        Servo_Array[i] = mitte + diffdata;
                     }
                     else
                     {
                        Servo_Array[i] = mitte - diffdata;
                     }
                  }
               }
               
               if (diffdata == 0)
               {
                  //diffdata = 0x100;
               }
            }
            
         }// for i
         
         
         // Servodaten mit Mix verarbeiten
         // Mix_Array 0: canals
         // Mix_Array 1: mixart
         
         for (i=0;i<4;i++)
         {
            uint8_t mixcanal = Mix_Array[2*i];
            if (mixcanal ^ 0x88) // 88 bedeutet OFF
            {
               uint8_t mixart = Mix_Array[2*i+1] & 0x07;
               uint8_t canala = mixcanal & 0x07;
               uint8_t canalb = (mixcanal & 0x70) >>4;
               switch (mixart) // mixart ist gesetzt
               {
                  case 1: // V-Mix
                  {
                     canalwerta = Servo_ArrayInt[canala];// Wert fuer ersten Kanal
                     canalwertb = Servo_ArrayInt[canalb];// Wert fuer zweiten Kanal
                     
                     Servo_ArrayInt[canala] = canalwerta + canalwertb;
                     Servo_ArrayInt[canalb] = canalwerta - canalwertb;
                  
                  }break;
               } // switch
            }
         }
         
         
         // Mitte addieren
         for (i=0;i<8;i++)
         {
            Servo_ArrayInt[i] += Mitte_Array[i];
         }
         
         sei();
         
         
         if ((eepromstatus & (1<<EE_WRITE))) // write an eeprom
         {
            
            eepromstatus &= ~(1<<EE_WRITE);
            /*
             
             {
             SPI_PORT_Init();
             spieeprom_init();
             
             eeprom_testdata++;
             eeprom_testaddress--;
             
             
             
             lcd_gotoxy(19,1);
             lcd_putc((eeprom_testdata %10)+48);
             //lcd_putc(48);
             spieeprom_init();
             
             //OSZI_C_LO;
             // statusregister schreiben
             
             // WREN
             EE_CS_LO;
             _delay_us(LOOPDELAY);
             spieeprom_wren();
             _delay_us(LOOPDELAY);
             EE_CS_HI; // SS HI End
             _delay_us(LOOPDELAY);
             
             //Write status
             EE_CS_LO;
             _delay_us(LOOPDELAY);
             spieeprom_write_status();
             _delay_us(LOOPDELAY);
             EE_CS_HI; // SS HI End
             
             
             _delay_us(10);
             
             // Byte  write
             
             // WREN schicken 220 us
             EE_CS_LO;
             _delay_us(LOOPDELAY);
             spieeprom_wren();
             _delay_us(LOOPDELAY);
             EE_CS_HI; // SS HI End
             _delay_us(LOOPDELAY);
             
             
             // Data schicken 350 us
             EE_CS_LO;
             _delay_us(LOOPDELAY);
             spieeprom_wrbyte(eeprom_testaddress, eeprom_testdata);
             _delay_us(LOOPDELAY);
             EE_CS_HI; // SS HI End
             
             
             _delay_us(50);
             
             // Byte  read 270 us
             EE_CS_LO;
             _delay_us(LOOPDELAY);
             //     OSZI_B_LO;
             _delay_us(LOOPDELAY);
             eeprom_indata = spieeprom_rdbyte(eeprom_testaddress);
             _delay_us(LOOPDELAY);
             //     OSZI_B_HI;
             EE_CS_HI;
             //OSZI_C_HI;
             
             
             
             lcd_gotoxy(0,1);
             lcd_putc('*');
             lcd_puthex(eeprom_testdata);
             
             lcd_putc('*');
             lcd_puthex(eeprom_indata);
             lcd_putc('*');
             
             lcd_putc('*');
             lcd_puthex(errcount);
             
             sendbuffer[3] = eeprom_testaddress;
             sendbuffer[4] = eeprom_testdata;
             //sendbuffer[12] = eeprom_testdata;
             
             // end Daten an EEPROM
             }
             */
         } // EE_WRITE
         else
            
         {
            //if (MASTER_PIN & (1<<SUB_BUSY_PIN))
            {
               
               cli();
              
               SPI_RAM_init();
               
               spiram_init();
               
               _delay_us(1);
               // statusregister schreiben
               RAM_CS_LO;
               _delay_us(LOOPDELAY);
               spiram_write_status(0x00);
               _delay_us(LOOPDELAY);
               RAM_CS_HI; // SS HI End
               _delay_us(1);
               
               // Kontrolle: testdata in-out
               RAM_CS_LO;
               _delay_us(LOOPDELAY);
               //      OSZI_A_LO;
               spiram_wrbyte(testaddress, testdata);
               //     OSZI_A_HI;
               RAM_CS_HI;
               
               // Pot-Daten schicken
               //OSZI_A_LO ;
               for (i=0;i< 8;i++)
               {
                  if (INT)
                  {
                     RAM_CS_LO;
                     _delay_us(LOOPDELAY);
                     spiram_wrbyte(2*i, Servo_ArrayInt[i] & 0x00FF);
                     
                     RAM_CS_HI;
                     _delay_us(LOOPDELAY);
                     RAM_CS_LO;
                     _delay_us(LOOPDELAY);
                     spiram_wrbyte(2*i+1, (Servo_ArrayInt[i] & 0xFF00)>>8);
                     RAM_CS_HI;
                     
                     
                  }
                  
                  else
                  {
                     RAM_CS_LO;
                     _delay_us(LOOPDELAY);
                     spiram_wrbyte(2*i, Servo_Array[i] & 0x00FF);
                     RAM_CS_HI;
                     _delay_us(LOOPDELAY);
                     RAM_CS_LO;
                     _delay_us(LOOPDELAY);
                     spiram_wrbyte(2*i+1, (Servo_Array[i] & 0xFF00)>>8);
                     RAM_CS_HI;
                  }
                  
               }
               
               /*
               RAM_CS_LO;
               _delay_us(LOOPDELAY);
               spiram_wrbyte(16, Servo_ArrayInt[0] & 0x00FF);
               RAM_CS_HI;
               _delay_us(LOOPDELAY);
               
               RAM_CS_LO;
               _delay_us(LOOPDELAY);
               spiram_wrbyte(16+1, (Servo_ArrayInt[0] & 0xFF00)>>8);
               RAM_CS_HI;
               _delay_us(LOOPDELAY);
               
               RAM_CS_LO;
               _delay_us(LOOPDELAY);
               spiram_wrbyte(18, Servo_ArrayInt[1] & 0x00FF);
               RAM_CS_HI;
               _delay_us(LOOPDELAY);
               
               RAM_CS_LO;
               _delay_us(LOOPDELAY);
               spiram_wrbyte(18+1, (Servo_ArrayInt[1] & 0xFF00)>>8);
               RAM_CS_HI;
               _delay_us(LOOPDELAY);
               

               RAM_CS_LO;
               _delay_us(LOOPDELAY);
               spiram_wrbyte(20, (canalwerta+MITTE) & 0xFF);// canala

               RAM_CS_HI;
               _delay_us(LOOPDELAY);

               RAM_CS_LO;
               _delay_us(LOOPDELAY);
               spiram_wrbyte(20+1, ((canalwerta+MITTE) & 0xFF00)>>8);
               //spiram_wrbyte(20+1, (Mix_Array[0] & 0x70)>>4);// canalb
               
               RAM_CS_HI;
               _delay_us(LOOPDELAY);

               RAM_CS_LO;
               _delay_us(LOOPDELAY);
               spiram_wrbyte(22, ((canalwertb+MITTE) & 0xFF));
               //spiram_wrbyte(22, Mix_Array[0]);// canals
               RAM_CS_HI;
               _delay_us(LOOPDELAY);
               
               RAM_CS_LO;
               _delay_us(LOOPDELAY);
               spiram_wrbyte(22+1, ((canalwertb+MITTE) & 0xFF00)>>8);
               //spiram_wrbyte(22+1, Mix_Array[1]);// mixart
               RAM_CS_HI;
               _delay_us(LOOPDELAY);

               */
               
               
               // ADC-Daten schicken
               /*
               RAM_CS_LO;
               _delay_us(LOOPDELAY);
               OSZI_A_LO;
               spiram_wrbyte(0x3E, adcdata & 0xFF);
               //     OSZI_A_HI;
               RAM_CS_HI;
               _delay_us(LOOPDELAY);
               RAM_CS_LO;
               _delay_us(LOOPDELAY);
               OSZI_A_LO;
               spiram_wrbyte(0x3F, adcdata & 0xFF);
               //     OSZI_A_HI;
               RAM_CS_HI;
               */
               
               // Diff-Daten schicken
               /*
               RAM_CS_LO;
               _delay_us(LOOPDELAY);
               OSZI_A_LO;
               spiram_wrbyte(0x3C, diff & 0xFF);
               //spiram_wrbyte(0x20, 17);
               //     OSZI_A_HI;
               RAM_CS_HI;
               _delay_us(LOOPDELAY);
               RAM_CS_LO;
               _delay_us(LOOPDELAY);
               //      OSZI_A_LO;
               spiram_wrbyte(0x3D,  (diff & 0xFF00)>>8);
               //     OSZI_A_HI;
               RAM_CS_HI;
               
               _delay_us(LOOPDELAY);
               RAM_CS_LO;
               //      OSZI_A_LO;
               spiram_wrbyte(0x3E, diffdatalo);
               //     OSZI_A_HI;
               RAM_CS_HI;
               _delay_us(LOOPDELAY);
               RAM_CS_LO;
               
               //      OSZI_A_LO;
               spiram_wrbyte(0x3F, diffdatahi);
               OSZI_A_HI;
               RAM_CS_HI;
               */
               
               // Task lesen
                task_in=0;
               _delay_us(2);
               RAM_CS_LO;
               _delay_us(LOOPDELAY);
               //     OSZI_B_LO;
               _delay_us(LOOPDELAY);
               task_in = spiram_rdbyte(READ_TASKADRESSE);
               _delay_us(LOOPDELAY);
               //     OSZI_B_HI;
               RAM_CS_HI;
 
               
              
               if ((task_in & (1<<RAM_RECV_LCD_TASK)) || (eepromstatus & (1<<EE_READ_SETTINGS))) // Setting neu lesen
               {
                  eepromstatus &= ~(1<<EE_READ_SETTINGS);
                  
                  task_counter++;
                  // Taskdaten lesen
                  _delay_us(2);
                  RAM_CS_LO;
                  _delay_us(LOOPDELAY);
                  //     OSZI_B_LO;
                  _delay_us(LOOPDELAY);
                  task_indata = spiram_rdbyte(READ_TASKDATA);
                  _delay_us(LOOPDELAY);
                  //     OSZI_B_HI;
                  RAM_CS_HI;
                
                  
                  lcd_gotoxy(12,0);
                  lcd_puthex(task_in);
                  lcd_putc('+');
                  lcd_puthex(task_counter);
                  lcd_putc('+');
                  lcd_clr_line(1);
                  
                  
                  readSettings(task_indata);
                  
                  task_in &= ~(1<<RAM_RECV_LCD_TASK);
                  
                  // Taskdaten entfernen
                  _delay_us(LOOPDELAY);
                  RAM_CS_LO;
                  spiram_wrbyte(READ_TASKADRESSE, 0);
                  OSZI_A_HI;
                  RAM_CS_HI;

               
               }

               
               
               // Taskdaten schreiben
               _delay_us(LOOPDELAY);
               RAM_CS_LO;
               spiram_wrbyte(WRITE_TASKADRESSE, task_outdata);
               OSZI_A_HI;
               RAM_CS_HI;
               
               
               
               
               
               // Kontrolle
               _delay_us(2);
               RAM_CS_LO;
               _delay_us(LOOPDELAY);
               //     OSZI_B_LO;
               _delay_us(LOOPDELAY);
               ram_indata = spiram_rdbyte(testaddress);
               _delay_us(LOOPDELAY);
               //     OSZI_B_HI;
               RAM_CS_HI;
               
               // Fehler zaehlen
               if ((testdata == ram_indata))
               {
                  masterstatus &= ~(1<<ALARM_BIT);
               }
               else
               {
                  errcount++;
                  TCNT2 = 0;
                  masterstatus |= (1<<ALARM_BIT);
               }
               
               
               // statusregister schreiben
               RAM_CS_LO;
               _delay_us(LOOPDELAY);
               spiram_write_status(0x00);
               _delay_us(LOOPDELAY);
               RAM_CS_HI; // SS HI End
               _delay_us(2);
               
               // err
               RAM_CS_LO;
               
               _delay_us(LOOPDELAY);
               //      OSZI_A_LO;
               spiram_wrbyte(7, errcount);
               //     OSZI_A_HI;
               RAM_CS_HI;
               
               
               _delay_us(2);
               
               /*
                RAM_CS_LO;
                for (uint8_t k=0;k<32;k++)
                {
                spiram_wrbyte(testaddress, testdata);
                }
                RAM_CS_HI;
                */
               // Daten aendern
               if (outcounter%0x40 == 0)
               {
                  
                  // timer2Counter=0;
                  /*
                  lcd_gotoxy(6,0);
                  
                  lcd_putint12(Servo_ArrayInt[0]);
                  lcd_putc(' ');
                  lcd_putint12(Servo_ArrayInt[1]);
                  lcd_putc(' ');
                  //lcd_putc('*');
                  
                  //lcd_putint(testdata);
                  //lcd_putc('*');
                  //lcd_putint(ram_indata);
                  //lcd_putc('+');
                   */
                  if  (masterstatus & (1<<ALARM_BIT))
                  {
                     masterstatus &= ~(1<<ALARM_BIT);
                     lcd_gotoxy(0,0);
                     lcd_putc('+');
                     lcd_putint(errcount);
                     lcd_putc('+');
                     
                  }
                  
                  testdata++;
                  testaddress = 0xFF;
                  //testaddress--;
                  
                  
               }
               outcounter++;
               
               _delay_us(LOOPDELAY);
               
               // end Daten an RAM
               
               
               // EEPROM Test
               
               
               //spi_end();
               sei();
               // if (MASTER_PIN ) // Master blockiert mit LO das Summensignal bei Langen VorgŠngen
               {
                  
                  timer1_init(); // Kanaele starten
                  
               }
               //   else
               {
                  //      abschnittnummer=0; // nach Master-Vorgang Position des Counters fuer PageWrite zuruecksetzen
               }
               //OSZI_A_HI ;
               
            } // if busy_pin
            
         }
      } // end Pot messen
      
      
      if (potstatus & (1<< SPI_END)) // SPI auf diesem device beenden
      {
         potstatus &= ~(1<< SPI_END);
         //spi_end();
         _delay_us(20);
      }

      
      /**	END ADC	***********************/
      
      /**	End USB-routinen	***********************/
 		
		/* **** rx_buffer abfragen **************** */
		//rxdata=0;
		
#pragma mark Tasten		
		//	Daten von USB vorhanden
		 // rxdata
		
		//lcd_gotoxy(16,0);
        //lcd_putint(StepCounterA & 0x00FF);
		
		if (!(TASTENPIN & (1<<TASTE0))) // Taste 0
		{
			//lcd_gotoxy(8,1);
			//lcd_puts("T0 Down\0");
			
			if (!(TastenStatus & (1<<TASTE0))) //Taste 0 war noch nicht gedrueckt
			{
				//RingD2(5);
				TastenStatus |= (1<<TASTE0);
				
				Tastencount=0;
				//lcd_gotoxy(0,1);
				//lcd_puts("P0 \0");
				//lcd_putint(TastenStatus);
				//delay_ms(800);
			}
			else
			{
				
				
				Tastencount +=1;
				//lcd_gotoxy(7,1);
				//lcd_puts("TC \0");
				//lcd_putint(Tastencount);
				
				if (Tastencount >= Tastenprellen)
				{
               
					Tastencount=0;
               if (TastenStatus & (1<<TASTE0))
               {
                  //sendbuffer[0]=loopcount1;
                  //sendbuffer[1]=0xAB;
                  //usbstatus |= (1<<USB_RECV);
                  //lcd_gotoxy(2,1);
                  //lcd_putc('1');

                  //usb_rawhid_send((void*)sendbuffer, 50);
               }
					TastenStatus &= ~(1<<TASTE0);
               //lcd_gotoxy(3,1);
               //lcd_puts("ON \0");
               //delay_ms(400);
               //lcd_gotoxy(3,1);
              // lcd_puts("  \0");
               //lcd_putint(TastenStatus);
               
               
				}
			}//else
			
		}	// Taste 0
		
         
		
		if (!(TASTENPIN & (1<<TASTE1))) // Taste 1
		{
			//lcd_gotoxy(12,1);
			//lcd_puts("T1 Down\0");
			
			if (! (TastenStatus & (1<<TASTE1))) //Taste 1 war nicht nicht gedrueckt
			{
				TastenStatus |= (1<<TASTE1);
				Tastencount=0;
				//lcd_gotoxy(3,1);
				//lcd_puts("P1 \0");
				//lcd_putint(Servoimpulsdauer);
				//delay_ms(800);
				
			}
			else
			{
				//lcd_gotoxy(3,1);
				//lcd_puts("       \0");
				
				Tastencount +=1;
				if (Tastencount >= Tastenprellen)
				{
					
					
					Tastencount=0;
					TastenStatus &= ~(1<<TASTE1);
					
				}
			}//	else
			
		} // Taste 1
		
		/* ******************** */
		//		initADC(TASTATURPIN);
		//		Tastenwert=(uint8_t)(readKanal(TASTATURPIN)>>2);
		
		Tastenwert=0;
		
		//lcd_gotoxy(3,1);
		//lcd_putint(Tastenwert);
   
		//OSZI_B_HI;
      if (usbstatus & (1<< USB_RECV))
      {
         //lcd_gotoxy(10,1);
         //lcd_puthex(AbschnittCounter);
         //sendbuffer[3]= AbschnittCounter;
         //usb_rawhid_send((void*)sendbuffer, 50);
         //sendbuffer[0]=0;
         //sendbuffer[5]=0;
         //sendbuffer[6]=0;
         //usbstatus &= ~(1<< USB_RECV);
         
      }

	}//while
   //free (sendbuffer);

// return 0;
}
