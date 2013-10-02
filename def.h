//Oszi
#define OSZIPORT           PORTD
#define OSZIPORTDDR        DDRD
#define OSZIPORTPIN        PIND
#define OSZI_PULS_A        0
#define OSZI_PULS_B        1


#define OSZI_A_LO OSZIPORT &= ~(1<<OSZI_PULS_A)
#define OSZI_A_HI OSZIPORT |= (1<<OSZI_PULS_A)
#define OSZI_A_TOGG OSZIPORT ^= (1<<OSZI_PULS_A)


#define OSZI_B_LO OSZIPORT &= ~(1<<OSZI_PULS_B)
#define OSZI_B_HI OSZIPORT |= (1<<OSZI_PULS_B)
#define OSZI_B_TOGG OSZIPORT ^= (1<<OSZI_PULS_B)

#define SPI_PORT     PORTB
#define SPI_PIN      PINB
#define SPI_DDR      DDRB

#define SPI_SS       2

#define SPI_MOSI     3
#define SPI_MISO     4
#define SPI_SCK      5

#define LOOPLED_PORT	PORTD
#define LOOPLED_DDR	DDRD
#define LOOPLED_PIN	4



//#define TASTE0				   2
//#define TASTE1             3

// SPI

#define SPI_BUFSIZE 8

#define STARTDELAYBIT       0
#define HICOUNTBIT          1

#define WDTBIT              7

// CNC12
#define CMD_PORT            PORTD   //    PORTB
#define CMD_DDR             DDRD    //    DDRB
#define CMD_PIN             PIND    //    PINB

#define EXTERN_PORT          PORTD
#define EXTERN_PIN           PIND

#define INT0_PIN           2



#define MASTER_PORT        PORTB   //    PORTB
#define MASTER_DDR         DDRB    //    DDRB
#define MASTER_PIN         PINB

#define MASTER_EN_PIN         7 // Mit PinChange-Interrupt
#define MASTER_EN_BIT         0 // Master erlaubt SPI
#define SUB_START_BIT       1 // Slave kann starten
#define SUB_END_BIT         2 // Slave ist fertig

#define ALARM_BIT         7 // toggle Alarm

#define SUB_TASK_BIT         3 // Slave hat Aufgaben

#define SUB_LCD_BIT         4 // Slave soll auf LCD schreiben


#define SUB_BUSY_PIN          6 // Sub ist busy

#define SUB_EN_PIN          7 // Sub ist enabled

         
#define MASTER_EN_HI           MASTER_PORT & (1<<MASTER_EN_PIN)


// ADC
// CNC12
#define ADC_PORT            PORTC   //
#define ADC_DDR             DDRC    //    
#define ADC_PIN             PINC    //    


// Bit

// EEPROM

#define EE_WREN   0
#define EE_WRITE  1
#define EE_READ   2
