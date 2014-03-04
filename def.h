//Oszi
/*
#define OSZIPORT           PORTD
#define OSZIPORTDDR        DDRD
#define OSZIPORTPIN        PIND
#define OSZI_PULS_A        0
#define OSZI_PULS_B        1
 */

// new PCB
#define OSZIPORT           PORTB
#define OSZIPORTDDR        DDRB
#define OSZIPORTPIN        PINB
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
#define EXPO_OFFSET              0x30
#define LEVEL_OFFSET             0x20
#define MIX_OFFSET               0x40
#define  TRIMM_OFFSET            0x50 // 80

#define READ_TASKADRESSE         0x1F0     // RAM_Adresse fuer  Task-Auftrag von RC_LCD
#define READ_TASKDATA            0x1F1

#define WRITE_TASKADRESSE        0x1FA     // RAM_Adresse fuer Task-Auftrag an RC_LCD
#define WRITE_TASKDATA           0x1FB

#define RAM_TRIMM_OFFSET         0x2F0    // Startadresse fuer Trimmdaten



#define RAM_SEND_LCD_TASK         1 // LCD soll Task lesen (Auftrag AN LCD)
#define RAM_RECV_LCD_TASK         2 // RAM soll Task lesen (Auftrag VON PPM)

// TRIMM_Tasks



#define RAM_SEND_DOGM_TASK 3

#define RAM_SEND_TRIMM_TASK      4

#define RAM_TASK_OK              7 // Task gelesen

#define STARTDELAYBIT            0
#define HICOUNTBIT               1

#define WDTBIT                   7

// CNC12
#define CMD_PORT              PORTD   //    PORTB
#define CMD_DDR               DDRD    //    DDRB
#define CMD_PIN               PIND    //    PINB

#define EXTERN_PORT           PORTD
#define EXTERN_PIN            PIND

#define INT0_PIN              2



#define MASTER_PORT           PORTB   //    PORTB
#define MASTER_DDR            DDRB    //    DDRB
#define MASTER_PIN            PINB

//defines fuer masterstatus

#define MASTER_EN_PIN         7 // 


#define MASTER_EN_BIT         0 // Master erlaubt SPI
#define SUB_START_BIT         1 // Slave kann starten
#define SUB_END_BIT           2 // Slave ist fertig
#define SUB_TASK_BIT          3  // Slave hat Aufgaben
#define SUB_LCD_BIT           4 // Slave soll auf LCD schreiben



#define ALARM_PIN             7 // toggle Alarm Pin







#define SUB_BUSY_PIN          6 // Sub ist busy

//#define SUB_EN_PIN            7 // Sub ist enabled

         
#define MASTER_EN_HI           MASTER_PORT & (1<<MASTER_EN_PIN)

// RAM
#define TASK_OFFSET           0x2000 // Speicherort fuer Einstellungen

//#define SETTINGBREITE      0x100; // 256 Bytes, Breite des Settingblocks fuer ein model

#define  MITTE_OFFSET         0x10 // 16
//#define  LEVEL_OFFSET      0x20 // 32
//#define  EXPO_OFFSET       0x30 // 48
#define  MIX_OFFSET           0x40 // 64



// ADC
/*
#define ADC_PORT            PORTC   //
#define ADC_DDR             DDRC    //    
#define ADC_PIN             PINC    //    
#define ADC_0               0 // Kanal fuer Batteriespannung
*/

#define KANAL_PORT            PORTC   //    Ausgang Summensignal 
#define KANAL_DDR             DDRC    //    


#define KANAL_PIN          2                             // Ausgang fuer Summensignal
#define KANAL_LO           KANAL_PORT &= ~(1<<KANAL_PIN)
#define KANAL_HI           KANAL_PORT |= (1<<KANAL_PIN)


// Bit

// Masterstatus

#define KANAL_EN_BIT          2 // Kanalimpuls setzen, nach Lesen von EEPROM beim Start
#define SUB_BUSY_BIT          3 // Sub ist busy
#define POT_READ              4  //Bit 4


#define RAM_ALARM_BIT         5 // toggle Alarm
#define ADC_ALARM_BIT         6 // toggle Alarm

#define  HALT                 7  //Bit 7


// loopstatus
#define KANAL_BIT             0     //Summensignal soll ausgegeben werden

// EEPROM

#define EE_WREN   0
#define EE_WRITE  1
#define EE_READ   2

#define EE_READ_SETTINGS   4

#define MITTE 0x5E0 // Neutralstellung 1.5ms

//#define MITTE 0x3B6 // Neutralstellung 1.5ms


// Bit

#define ADC_START 0  //    Start Messung Batteriespannung mit internem ADC

#define POT_START 0  //    Start Messung Potentiometer

#define SPI_START 2  //    Start SPI auf diesem Device



#define SPI_END   3  //    End SPI auf diesem Device

#define POT_MITTE 7  //    Mittelwerte der Potentiometer speichern

#define ANZ_POT   6

#define POT_FAKTOR 0.75


#define TASTENDDR           DDRD
#define TASTENPORT          PORTD
#define TASTENPIN           PIND

#define TASTE0				   0
#define TASTE1             1

#define INTERRUPT_PORT            PORTB   //
#define INTERRUPT_DDR             DDRB    //
#define INTERRUPT_PIN             PINB    //


