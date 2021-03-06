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

#define SPI_BUFSIZE              8

#define STUFENOFFSET             0x800

#define TASK_OFFSET              0x2000 // Ort fuer Einstellungen

#define SETTINGBREITE            0x80; // Breite des Settingblocks fuer ein Device

#define MITTE_OFFSET             0x10
#define EXPO_OFFSET              0x20
#define LEVEL_OFFSET             0x30
#define MIX_OFFSET               0x40

#define READ_TASKADRESSE         0x1F0     // RAM_Adresse fuer  Task-Auftrag von RC_LCD
#define READ_TASKDATA            0x1F1

#define WRITE_TASKADRESSE        0x1FA     // RAM_Adresse fuer Task-Auftrag an RC_LCD
#define WRITE_TASKDATA           0x1FB

#define RAM_SEND_LCD_TASK         1 // LCD soll Task lesen (Auftrag AN LCD)
#define RAM_RECV_LCD_TASK         2 // RAM soll Task lesen (Auftrag VON PPM)


#define RAM_TASK_OK              7 // Task gelesen

#define STARTDELAYBIT            0
#define HICOUNTBIT               1

#define WDTBIT                   7

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

#define ALARM_PIN         7 // toggle Alarm
#define ALARM_BIT         6 // toggle Alarm

#define SUB_TASK_BIT         3 // Slave hat Aufgaben

#define SUB_LCD_BIT         4 // Slave soll auf LCD schreiben


#define SUB_BUSY_PIN          6 // Sub ist busy
#define SUB_BUSY_BIT          6 // Sub ist busy

#define SUB_EN_PIN          7 // Sub ist enabled

         
#define MASTER_EN_HI           MASTER_PORT & (1<<MASTER_EN_PIN)

// RAM
#define TASK_OFFSET       0x2000 // Ort fuer Einstellungen

#define SETTINGBREITE      0x100; // 256 Bytes, Breite des Settingblocks fuer ein model

#define  MITTE_OFFSET      0x10 // 16
#define  LEVEL_OFFSET      0x20 // 32
#define  EXPO_OFFSET       0x30 // 48
#define  MIX_OFFSET        0x40 // 64


// ADC

#define ADC_PORT            PORTC   //
#define ADC_DDR             DDRC    //    
#define ADC_PIN             PINC    //    
#define ADC_0               0 // Kanal fuer Batteriespannung

#define KANAL_PORT            PORTC   //    Ausgang Summensignal 
#define KANAL_DDR             DDRC    //    


#define KANAL_PIN          2                             // Ausgang fuer Summensignal
#define KANAL_LO           KANAL_PORT &= ~(1<<KANAL_PIN)
#define KANAL_HI           KANAL_PORT |= (1<<KANAL_PIN)


// Bit

// Masterstatus
#define  RAM_READ           0x01  //Bit 0 
#define  RAM_WRITE          0x02  //Bit 1  
#define  EEPROM_READ        0x04  //Bit 2
#define  EEPROM_WRITE       0x08  //Bit 3

#define  HALT                7  //Bit 7 

#define  POT_READ           4  //Bit 4

// EEPROM

#define EE_WREN   0
#define EE_WRITE  1
#define EE_READ   2

#define EE_READ_SETTINGS   4

#define MITTE 0x600


// Bit

#define ADC_START 0  //    Start Messung Batteriespannung mit internem ADC

#define POT_START 0  //    Start Messung Potentiometer

#define SPI_START 2  //    Start SPI auf diesem Device



#define SPI_END   3  //    End SPI auf diesem Device

#define POT_MITTE 7  //    Mittelwerte der Potentiometer speichern

#define ANZ_POT   6

#define POT_FAKTOR 0.6


#define TASTENDDR           DDRD
#define TASTENPORT          PORTD
#define TASTENPIN           PIND

#define TASTE0				   0
#define TASTE1             1

#define INTERRUPT_PORT            PORTB   //
#define INTERRUPT_DDR             DDRB    //
#define INTERRUPT_PIN             PINB    //


