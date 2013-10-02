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
#include "usb_rawhid.c"
#include "def.h"

//#include "spi.c"
#include "spi_adc.c"
#include "spi_ram.c"
#include "spi_eeprom.c"

// USB
#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

#define LOOPDELAY 1

#define SERVOMAX  4400
#define SERVOMIN  1400

volatile uint8_t do_output=0;
static volatile uint8_t buffer[32]={};
static volatile uint8_t sendbuffer[32]={};


#define TIMER0_STARTWERT	0x40

volatile uint8_t timer0startwert=TIMER0_STARTWERT;
#define USB_DATENBREITE 32
//volatile uint8_t rxbuffer[USB_DATENBREITE];

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
//volatile uint8_t txbuffer[USB_DATENBREITE];

//uint16_t EEMEM Brennerlaufzeit;	// Akkumulierte Laufzeit

void delay_ms(unsigned int ms);

static volatile uint8_t           adcstatus=0x00;
static volatile uint8_t    usbstatus=0x00;

static volatile uint8_t      eepromstatus=0x00;
static volatile uint8_t    potstatus=0x80; // Bit 7 gesetzt, Mittelwerte setzen
static volatile uint8_t    impulscounter=0x00;

static volatile uint8_t masterstatus = 0;

#define USB_RECV  0 

volatile uint8_t status=0;

volatile uint8_t           PWM=0;
static volatile uint8_t    pwmposition=0;
static volatile uint8_t    pwmdivider=0;


volatile char SPI_data='0';
volatile char SPI_dataArray[SPI_BUFSIZE];
volatile uint16_t POT_Array[SPI_BUFSIZE];
volatile uint16_t Mitte_Array[SPI_BUFSIZE];

volatile uint16_t RAM_Array[SPI_BUFSIZE];

volatile uint16_t Batteriespannung =0;
volatile short int received=0;

volatile uint16_t abschnittnummer=0;
volatile uint16_t usbcount=0;

volatile uint16_t minwert=0xFFFF;
volatile uint16_t maxwert=0;



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
	
	LOOPLEDDDR |=(1<<LOOPLED);
	LOOPLEDPORT |= (1<<LOOPLED);	// HI
	
	//Pin 0 von   als Ausgang fuer OSZI
	OSZIPORTDDR |= (1<<OSZI_PULS_A);	//Pin 0 von  als Ausgang fuer LED TWI
   OSZIPORT |= (1<<OSZI_PULS_A);		// HI
	
   OSZIPORTDDR |= (1<<OSZI_PULS_B);		//Pin 1 von  als Ausgang fuer LED TWI
   OSZIPORT |= (1<<OSZI_PULS_B);		//Pin   von   als Ausgang fuer OSZI
   
   
   OSZIPORTDDR |= (1<<OSZI_PULS_C);		//Pin 1 von  als Ausgang fuer LED TWI
   OSZIPORT |= (1<<OSZI_PULS_C);		//Pin   von   als Ausgang fuer OSZI
   
   OSZIPORTDDR |= (1<<OSZI_PULS_D);		//Pin 1 von  als Ausgang fuer LED TWI
   OSZIPORT |= (1<<OSZI_PULS_D);		//Pin   von   als Ausgang fuer OSZI
	
   /*
	TASTENDDR &= ~(1<<TASTE0);	//Bit 0 von PORT B als Eingang f�r Taste 0
	TASTENPORT |= (1<<TASTE0);	//Pull-up
   */
 	
   
	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);		//Pin 4 von PORT D als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 5 von PORT D als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT D als Ausgang fuer LCD
   
	
   DDRD |= (1<<PORTD6);
   PORTD |= (1<<PORTD6);
   
   ADC_DDR &= ~(1<<PORTF0);
   
   
   
   MASTER_EN_DDR |= (1<<MASTER_EN_PIN); // Ausgang fuer Enabel-Signal an Sub
   MASTER_EN_PORT |= (1<<MASTER_EN_PIN); // HI
   

   //DDRE |= (1<<PORTE0);
   //PORTE |= (1<<PORTE0);

   MEM_EN_DDR |= (1<<MEM_EN_PIN);
   MEM_EN_PORT |= (1<<MEM_EN_PIN);

  
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
   
   //MCP3208_spi_Init();
   //spiram_init();
   //spieeprom_init();
   //spiadc_init();
   
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





void timer1_init(void)
{
   
      OSZI_C_LO;
   // Quelle http://www.mikrocontroller.net/topic/103629
   
   OSZI_A_HI ; // Test: data fuer SR
   _delay_us(5);
   //#define FRAME_TIME 20 // msec
   KANAL_DDR |= (1<<KANAL_PIN); // Kanal Ausgang
      
   DDRD |= (1<<PORTD5); //  Ausgang
   PORTD |= (1<<PORTD5); //  Ausgang

   //TCCR1A = (1<<COM1A0) | (1<<COM1A1);// | (1<<WGM11);	// OC1B set on match, set on TOP
   //TCCR1B = (1<<WGM13) | (1<<WGM12) ;		// TOP = ICR1, clk = sysclk/8 (->1us)
   TCCR1B |= (1<<CS11);
   TCNT1  = 0;														// reset Timer
   
                              // Impulsdauer
   OCR1B  = 0xFF;				// Impulsdauer des Kanalimpulses
   
   TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt:
   TIMSK1 |= (1 << OCIE1B);  // enable timer compare interrupt:
   
   //KANAL_PORT |= (1<<PORTB5); // Ausgang HI
   //OSZI_A_LO ;
   OSZI_B_LO ;
   
   
   KANAL_HI;
   
   impulscounter = 0;
   //OCR1A  = POT_FAKTOR*POT_Array[1];
   //OCR1A  = POT_FAKTOR*POT_Array[impulscounter];
   
   /*
   if (POT_Array[impulscounter] > SERVOMAX)
   {
     POT_Array[impulscounter] = SERVOMAX;
   }
   
   if (POT_Array[impulscounter] < SERVOMIN)
   {
      POT_Array[impulscounter] = SERVOMIN;
   }
*/
   
   OCR1A  = POT_Array[impulscounter]; // POT_Faktor schon nach ADC
      
   


} // end timer1

void timer1_stop(void)
{
  // TCCR1A = 0;
   
}




ISR(TIMER1_COMPA_vect)	 //Ende der Pulslaenge fuer einen Kanal
{
   
   KANAL_HI;
   impulscounter++;
   
   if (impulscounter < ANZ_POT)
   {
      // Start Impuls
      
      TCNT1  = 0;
      //KANAL_HI;
            
      // Laenge des naechsten Impuls setzen
      
      //OCR1A  = POT_FAKTOR*POT_Array[1]; // 18 us
      //OCR1A  = POT_FAKTOR*POT_Array[impulscounter]; // 18 us
      
      /*
      if (POT_Array[impulscounter] > SERVOMAX)
      {
         POT_Array[impulscounter] = SERVOMAX;
      }
      
      if (POT_Array[impulscounter]<  SERVOMIN)
      {
         POT_Array[impulscounter] = SERVOMIN;
      }
       */
      OCR1A  = POT_Array[impulscounter]; // 
   }
   else
   {
      // Ende Impulspaket
      OCR1A  = 0x4FF;
      //OSZI_A_HI ;
     // _delay_us(200);
      KANAL_LO;

      // Alle Impulse gesendet, Timer1 stop. Timer1 wird bei Beginn des naechsten Paketes wieder gestartet
      timer1_stop();
      
      // SPI fuer device ausschalten
      spi_end();
      potstatus |= (1<<SPI_END); //       
      OSZI_B_HI ;
      //OSZI_A_LO ;
      
      MASTER_EN_PORT &= ~(1<<MASTER_EN_PIN); // Master schickt Enable an Slave
      _delay_us(2);
      //PORTE |= (1<<PORTE0);
      MEM_EN_PORT |= (1<<MEM_EN_PIN);
      OSZI_C_HI;
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
      OSZI_B_HI ;
            
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
	TCNT0 = TIMER0_STARTWERT;					//R�cksetzen des Timers

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
   
	if (timer2Counter >= 0x474) // Laenge des Impulspakets 20ms
//	if (timer2Counter >= 0x8E8) // Laenge des Impulspakets 20ms
	{
      
      //potstatus |= (1<<POT_START); // Potentiometer messen
      
      potstatus |= (1<<SPI_START); // Potentiometer messen
      
      
      timer2BatterieCounter++; // Intervall fuer Messung der Batteriespannung
      if (timer2BatterieCounter >= 0xF)
      {
         adcstatus |= (1<<ADC_START); // Batteriespannung messen
         timer2BatterieCounter = 0;
         
      }

		timer2Counter = 0;
       ;
	} 
	TCNT2 = 10;							// ergibt 2 kHz fuer Timertakt
}

/*
ISR(TIMER2_COMP_vect) // Schaltet Impuls an SERVOPIN0 aus
{
//		lcd_clr_line(1);
//		lcd_puts("Timer2 Comp\0");
		TCCR20=0;
}
*/

void setMitte(void)
{
   for (uint8_t i=0;i< SPI_BUFSIZE;i++)
   {
      Mitte_Array[i] = POT_Array[i];
   }
}


#pragma mark - main
int main (void) 
{
    int8_t r;

   uint16_t count=0;
    
	// set for 16 MHz clock
	CPU_PRESCALE(0);
    
	// Initialize the USB, and then wait for the host to set configuration.
	// If the Teensy is powered without a PC connected to the USB port,
	// this will wait forever.
	usb_init();
	while (!usb_configured()) /* wait */ ;
    
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

   
   //MCP3208_spi_Init();
   
  // SPI_RAM_PORT &= ~(1<<SPI_RAM_CS_PIN);
  // _delay_us(10);
   
  // spiram_init();
   
  // SPI_RAM_PORT |= (1<<SPI_RAM_CS_PIN);
  // _delay_us(10);
	/* initialize the LCD */
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);

	lcd_puts("Guten Tag\0");
	delay_ms(100);
	lcd_cls();
	//lcd_puts("READY\0");
	lcd_puts("V: \0");
	lcd_puts(VERSION);
   lcd_clr_line(1);

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
	Bit 0: 1 wenn wdt ausgel�st wurde
	 
	  */ 
	uint8_t i=0;
   
	//timer2
	TCNT2   = 0; 
//	TCCR2A |= (1 << WGM21);    // Configure timer 2 for CTC mode 
	TCCR2B |= (1 << CS20);     // Start timer at Fcpu/8 
//	TIMSK2 |= (1 << OCIE2A);   // Enable CTC interrupt 
	TIMSK2 |= (1 << TOIE2);    // Enable OV interrupt 
	//OCR2A   = 5;             // Set CTC compare value with a prescaler of 64 
    TCCR2A = 0x00;
   
   sei();
   
   PWM = 0;
   
   char* versionstring = (char*) malloc(4);
   strncpy(versionstring, VERSION+9, 3);
   versionstring[3]='\0';
   volatile uint16_t versionint = atoi(versionstring);
   volatile uint8_t versionintl = versionint & 0x00FF;
   //versionint >>=8;
   volatile uint8_t versioninth = (versionint & 0xFF00)>>8;
   
   uint8_t anzeigecounter=0;
   
#pragma mark while	  
	while (1)
	{
      //OSZI_B_LO;
		//Blinkanzeige
		loopcount0+=1;
		if (loopcount0==0x4FFF)
		{
			loopcount0=0;
			loopcount1+=1;
			LOOPLEDPORT ^=(1<<LOOPLED);
         //PORTD ^= (1<<PORTD6);
         
 			//
			//timer0();
         
         if (loopcount1%0x0F == 0)
         {
            //lcd_gotoxy(18,1);
            //lcd_puthex(hidstatus);
         }
         else if(loopcount1%8 == 4)
         {
            //lcd_gotoxy(18,1);
            //lcd_putc('*');
            //lcd_putc('*');
            
         }
         
         //sendbuffer[0]=0x33;
         sendbuffer[0]= abschnittnummer;
         sendbuffer[1]= abschnittnummer>>8;
         abschnittnummer++;
         sendbuffer[2]= usbcount & 0xFF;
         
         
         //sendbuffer[4] = loopcount1&0xFF;
         uint16_t adc0wert = adc_read(0);
         sendbuffer[5] = adc0wert & 0xFF;
         sendbuffer[6] = (adc0wert>>8) & 0xFF;

         
         
         
         // Messung anzeigen
         if (loopcount1%0xF == 0)
         {
            /*
            lcd_gotoxy(0,1);
            lcd_putint12Bit(POT_Array[0]);
            lcd_putc('*');
            lcd_putint12Bit(POT_Array[1]);
            lcd_putc('*');
             */
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
            sendbuffer[8+2*i]=(POT_Array[i] & 0xFF);    // 8 10
            sendbuffer[8+2*i+1]= (POT_Array[i]>>8) & 0xFF;  // 9  11
         }
         //OSZI_B_LO;
         
         // neue Daten abschicken
         usb_rawhid_send((void*)sendbuffer, 50); // 20 us
         
		//OSZI_B_HI;
      } // if loopcount0
      
      /**	ADC	***********************/

     if (adcstatus & (1<< ADC_START)) // ADC starten
      {
         
         Batteriespannung = adc_read(0);
         
         adcstatus &=  ~(1<< ADC_START);
         
         /*
         lcd_gotoxy(0,0);
         
         lcd_putint12Bit(Batteriespannung);
         
         lcd_putc('*');
         uint8_t high = (Batteriespannung & 0xFF00)>>8; // *0xFF rechnen und zu low dazuzaehlen
         lcd_putint(high);
         uint8_t low = (Batteriespannung & 0xFF);
         lcd_putc('*');
         lcd_putint(low);
         //lcd_puthex(loopcount1);
          */
         ;

      }
      
//      if (potstatus & (1<< POT_START)) // POT starten, in TIMER2_OVF_vect gesetzt
      if (potstatus & (1<< SPI_START)) // SPI starten, in TIMER2_OVF_vect gesetzt
      {
         MASTER_EN_PORT |= (1<<MASTER_EN_PIN); // Sub abstellen
         _delay_us(2);
          
         //potstatus &= ~(1<< POT_START); // Bit zuruecksetzen
         
         potstatus &= ~(1<< SPI_START);   // Bit zuruecksetzen
         
         OSZI_D_LO ;
         uint8_t i=0;
         
         // SPI fuer device einschschalten
         spi_start();
         

         
         SPI_ADC_init();
         spiadc_init();
         
         
         for(i=0;i< ANZ_POT;i++)
         {
            //PORTE &= ~(1<<PORTE0);
            /*
            MEM_EN_PORT &= ~(1<<MEM_EN_PIN);
            
            spiram_init();
            // RAM lesen als Test fuer Zeitverbrauch
            _delay_us(1);
            RAM_CS_LO;
            _delay_us(LOOPDELAY);
            //     OSZI_B_LO;
            _delay_us(LOOPDELAY);
            ram_indata = spiram_rdbyte(testaddress);
            _delay_us(LOOPDELAY);
            //     OSZI_B_HI;
            RAM_CS_HI;
            //PORTE |= (1<<PORTE0);
            MEM_EN_PORT |= (1<<MEM_EN_PIN);
            */
            //spiadc_init();
            
            
            //lcd_putint(i);
            if (i<2)
            {
               

               POT_Array[i] = POT_FAKTOR*MCP3208_spiRead(SingleEnd,i);
               if (POT_Array[i]==0)
               {
               POT_Array[i] = 0x800;
               
               }
               
               // Filter
               //POT_Array[i] = 3*POT_Array[i]/4 + (MCP3208_spiRead(SingleEnd,i)/4);
               //POT_Array[i] = POT_FAKTOR*(1*POT_Array[i]/2 + (MCP3208_spiRead(SingleEnd,i)/2));
               _delay_us(2); // war mal 100
            }
            else
            {
               POT_Array[i] = 0x800;
            }
         }
         OSZI_D_HI ;
         anzeigecounter++;
         
         // Mittelwert speichern
         if (potstatus & (1<< POT_MITTE))
         {
            setMitte();
            potstatus &= ~(1<< POT_MITTE);
         }
         
         if (anzeigecounter %0xF ==0)
         {
            //lcd_gotoxy(0,1);
            if (usbstatus & (1<<USB_RECV))
            {
               //lcd_putc('+');
               usbstatus &= ~(1<<USB_RECV);
            }
            
            //lcd_putint12Bit(POT_Array[0]);
            //lcd_putc(' ');
            //lcd_putint12Bit(POT_Array[1]);
            
         }
         
         
         
         
         ///usb_rawhid_send((void*)sendbuffer, 50); // in loopcount0
         
         // Daten an RAM
         //cli();
         //PORTE &= ~(1<<PORTE0);
         MEM_EN_PORT &= ~(1<<MEM_EN_PIN);
         
         _delay_us(100);
         SPI_PORT_Init();
         
         if (eepromstatus & (1<<EE_WRITE)) // write an eeprom
         {
            SPI_PORT_Init();
            eeprom_testdata++;
            eeprom_testaddress--;
            
            eepromstatus &= ~(1<<EE_WRITE);
            
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
            
         } // EE_WRITE
         else
            
         {
         
         
         
         SPI_RAM_init();
         
         spiram_init();
         
          _delay_us(100);
         // statusregister schreiben
         RAM_CS_LO;
         _delay_us(LOOPDELAY);
         spiram_write_status(0x00);
         _delay_us(LOOPDELAY);
         RAM_CS_HI; // SS HI End
         _delay_us(100);
     
         // testdata in-out
         RAM_CS_LO;
         
         _delay_us(LOOPDELAY);
         //      OSZI_A_LO;
         spiram_wrbyte(testaddress, testdata);
         //     OSZI_A_HI;
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
         if (!(testdata == ram_indata))
         {
            errcount++;
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
         spiram_wrbyte(0, errcount);
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
            //lcd_gotoxy(0,0);
            /*
            lcd_putint12(timer2Counter);
            
            lcd_putc('*');
            
            lcd_putint(testdata);
            lcd_putc('*');
            lcd_putint(ram_indata);
            lcd_putc('+');
             */
            //lcd_putint1(errcount);
            //lcd_putc('+');
            
            
            testdata++;
            testaddress--;
            
            
         }
         outcounter++;
         
         _delay_us(LOOPDELAY);
         
         // end Daten an RAM
         
         
         // EEPROM Test
         
           
         //spi_end();
         sei();
         //if (PINC & (1<<PC6))
         {
            
            timer1_init(); // Kanaele starten
            
         }
         //PORTD &= ~(1<<PORTD5); //  LO
         
         
         //timer1_stop();
         
         
         //OSZI_A_HI ;
            
         }
      } // end Pot messen
      
      
      if (potstatus & (1<< SPI_END)) // SPI auf diesem device beenden
      {
         OSZI_D_LO ;
         potstatus &= ~(1<< SPI_END);
         //spi_end();
         _delay_us(20);
         OSZI_D_HI ;
      }

      
      /**	END ADC	***********************/
      
       /**	Begin USB-routinen	***********************/
      
        // Start USB
      //lcd_putc('u');
      //OSZI_B_LO;
      r = usb_rawhid_recv((void*)buffer, 0);
      //OSZI_B_HI;
      if (r > 0) 
      {
         usbstatus |= (1<<USB_RECV);
        
         cli(); 
         
         uint8_t code = 0x00;
         code = buffer[31];
         lcd_gotoxy(14,0);
         lcd_puthex(code);
         lcd_putc('*');
         lcd_puthex(buffer[4]);
         switch (code)
         {   
               
            case 0xA0: // Write EEPROM
            {
               
               eepromstatus |= (1<<EE_WRITE);
               lcd_gotoxy(18,1);
               lcd_putc('E');
               lcd_putc(' ');
            }break;
               
               
             
#pragma mark default
            default:
            {
               // Abschnittnummer bestimmen
               uint8_t indexh=buffer[18];
               uint8_t indexl=buffer[19];
               
               for (int i=8;i<16;i++)
               {
                  //sendbuffer[i]=POT_Array[i];

               }
               //sendbuffer[6]=buffer[16];
               
               //sendbuffer[8]= versionintl;
               //sendbuffer[9]= versioninth;
               //usb_rawhid_send((void*)sendbuffer, 50); // nicht jedes Paket melden
               
             
               
                
            } // default
               
         } // switch code
         code=0;
         sei();
         
         
		} // r>0, neue Daten
      else
      {
         //OSZI_B_LO;
      }
      
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
