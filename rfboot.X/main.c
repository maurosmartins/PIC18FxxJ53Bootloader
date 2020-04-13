#include <xc.h>
#include <stdint.h>
#include <plib/usart.h>

#define _XTAL_FREQ 8000000

#define ERROR_RXTIMEOUT 1
#define ERROR_OK 0

#define ROMSTART 0x400u
#define ROMEND 0x1FFF8u


/*****************************************************************************************
* Config Bits
****************************************************************************************/
// CONFIG1L
#pragma config WDTEN = OFF      // Watchdog Timer (Disabled - Controlled by SWDTEN bit)
#pragma config PLLDIV = 4       // PLL Prescaler Selection (Divide by 4 (16 MHz oscillator input))
#pragma config CFGPLLEN = ON    // PLL Enable Configuration Bit (PLL Enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset (Enabled)
#pragma config XINST = OFF      // Extended Instruction Set (Disabled)

// CONFIG1H
#pragma config CPUDIV = OSC1    // CPU System Clock Postscaler (No CPU system clock divide)
#pragma config CP0 = OFF        // Code Protect (Program memory is not code-protected)

// CONFIG2L
//#pragma config OSC = HSPLL    // Oscillator (HS+PLL, USB-HS+PLL)
#pragma config OSC = INTOSC     // Intosc
#pragma config SOSCSEL = HIGH   // T1OSC/SOSC Power Selection Bits (High Power T1OSC/SOSC circuit selected)
#pragma config CLKOEC = OFF     // EC Clock Out Enable Bit  (CLKO output disabled on the RA6 pin)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor (Disabled)
#pragma config IESO = ON        // Internal External Oscillator Switch Over Mode (Enabled)

// CONFIG2H
#pragma config WDTPS = 32768    // Watchdog Postscaler (1:32768)

// CONFIG3L
#pragma config DSWDTOSC = INTOSCREF// DSWDT Clock Select (DSWDT uses INTRC)
#pragma config RTCOSC = T1OSCREF// RTCC Clock Select (RTCC uses T1OSC/T1CKI)
#pragma config DSBOREN = ON     // Deep Sleep BOR (Enabled)
#pragma config DSWDTEN = ON     // Deep Sleep Watchdog Timer (Enabled)
#pragma config DSWDTPS = G2     // Deep Sleep Watchdog Postscaler (1:2,147,483,648 (25.7 days))

// CONFIG3H
#pragma config IOL1WAY = ON     // IOLOCK One-Way Set Enable bit (The IOLOCK bit (PPSCON<0>) can be set once)
#pragma config ADCSEL = BIT12   // ADC 10 or 12 Bit Select (12 - Bit ADC Enabled)
#pragma config MSSP7B_EN = MSK7 // MSSP address masking (7 Bit address masking mode)

// CONFIG4L
#pragma config WPFP = PAGE_127  // Write/Erase Protect Page Start/End Location (Write Protect Program Flash Page 127)
#pragma config WPCFG = OFF      // Write/Erase Protect Configuration Region  (Configuration Words page not erase/write-protected)

// CONFIG4H
#pragma config WPDIS = OFF      // Write Protect Disable bit (WPFP<6:0>/WPEND region ignored)
#pragma config WPEND = PAGE_WPFP// Write/Erase Protect Region Select bit (valid when WPDIS = 0) (Pages WPFP<6:0> through Configuration Words erase/write protected)
#pragma config LS48MHZ = SYS48X8// Low Speed USB mode with 48 MHz system clock bit (System clock at 48 MHz USB CLKEN divide-by is set to 8)

void init(void);
void putch(uint8_t c);
uint8_t u8getch(uint8_t * error);
void vWriteFlashBlock(uint32_t addr, uint8_t u8FlashBlockLen);
void vEraseFlash(void);
void vEraseBlock(uint32_t addr);
uint8_t u8GetCodeLen(void);
uint8_t u8GetDataBlock(void);


volatile uint8_t FlashData[64];
union {
   uint32_t CodeLen;
   uint8_t CodeLena[sizeof(uint32_t)];
} u32CodeLen;
volatile uint32_t address;
enum sysStates {bootloaderMode = 0, errorMode = 0}; 
enum sysStates currentSysState = bootloaderMode;




void main(void) {
    uint8_t err, ch, counter, datablocklen;
    
    init();
    ch = RCREG2;
        
    /*
    #asm
        PSECT intcode
            GOTO ROMSTART+0x8
        PSECT intcodelo
            GOTO ROMSTART+0x18
    #endasm
    */
    
    counter = 0;
    
    //wait PC App and jump to code start if not received
    
    LATAbits.LATA0 = 0;
    while(1){        
        ch = u8getch(&err);
        if( (err == ERROR_OK) && (ch=='B') ){
            putch('A');
            break;
        }
        __delay_ms(10);
        __delay_ms(10);
        counter++;
        if(counter == 5){
            //asm("GOTO " ___mkstr(ROMSTART));
            asm("GOTO 0x400");
        }
    }
       
    u8GetCodeLen();    
    vEraseFlash();
    putch('B');
    
    address = ROMSTART;
    
    
    while( address < (address+u32CodeLen.CodeLen) ){
        datablocklen = u8GetDataBlock();
        if(datablocklen!=0){
            vWriteFlashBlock(address, datablocklen);
            putch('C');
            address += datablocklen;
        }else{
            break;
        }
        
    }
    
    
    //asm("GOTO " ___mkstr(ROMSTART));
    asm("GOTO 0x400");
}


uint8_t u8GetDataBlock(void){
    uint8_t j;
    uint8_t err;
    uint8_t ch;
    uint8_t datablocklen = 0;
    
    //dummy read
    //PIR3bits.RC2IF = 0;
    //ch=RCREG2;
    
    
    
    ch = u8getch(&err);      
    if(err == ERROR_OK){
        datablocklen = ch;
        
        for(j=0;j<datablocklen;j++){   
            LATAbits.LATA0 = 1;
            ch = u8getch(&err);
            LATAbits.LATA0 = 0;
            
            if(err == ERROR_OK){
                FlashData[j] = ch;
            } else{
                putch('N');
                break;
            }//if else
            
        }//for
        
    }//if
    
    //putch('C');
    return datablocklen;
}


//uint8_t u8GetDataBlock(void){
//    uint8_t j;
//    uint8_t err;
//    uint8_t datablocklen = 0;
//    
//    while(!PIR3bits.RC2IF){
//            asm("nop");
//        }
//    //datablocklen = u8getch(&err);
//    datablocklen = RCREG2;
//
//    PIR3bits.RC2IF = 0;
//    for(j=0;j<datablocklen;j++){        
//        while(!PIR3bits.RC2IF){
//            asm("nop");
//        }
//        FlashData[j] = RCREG2;
//
//    }
//    putch('K');
//    return datablocklen;
//}

uint8_t u8GetCodeLen(void){
    uint8_t j;
    uint8_t err;
    for(j=0;j<4;j++){
        u32CodeLen.CodeLena[j] = u8getch(&err);      
        if(err != ERROR_OK){
            putch('N');
            return ERROR_RXTIMEOUT;
        }
    }
    //putch('B');
    return ERROR_OK;
}


void init(void){
    OSCCONbits.IRCF = 0b111; //8MHz INTOSC
    
    /////////////////////////Unlock sequence for RPx pins (pg163 datasheet pic18f47j53)
	EECON2=0x55;							//first unlocking sytem step
	EECON2=0xAA;							//second unlocking sytem step
	PPSCONbits.IOLOCK=0;					//third unlocking sytem step

	//RB0 -> PIC TX2 (RP3)
	//RB1 -> PIC RX2 (RP4)
	RPINR16 = 4;							//RX2 on pin RP4 (RB1) RX2
	RPOR3 	= 6;							//TX2 on pin RP3 (RB0) TX2

	PPSCONbits.IOLOCK=1;					//lock remappable pins

/////////////////////////PIN Configuration
    ANCON0 = 0b00000001;                    //RA0 (AN0) as digital
	//CE  -> RB2
	//CS  -> RB3
	ANCON1=0b00011111;						//RC2, RB0, RB1, RB2, RB3 as Ditial IO Pins
											//AN8 a AN12 como IO digitais, o porto RB0, RB1, RB2 e RB3 incluem-se aqui
    
    TRISA = 0b11111110;                     //RA0 as output    
    
	TRISB=0b11110010;						//IO PIN RB2(RESET) and RB3(CS) and RB0 (Tx2) as output
											//The remainings as input
    
    Open2USART( USART_TX_INT_OFF & USART_RX_INT_OFF & USART_ADDEN_OFF & USART_ASYNCH_MODE & USART_EIGHT_BIT & USART_CONT_RX & USART_BRGH_HIGH,25);      //19200@8MHz
}

void vWriteFlashBlock(uint32_t addr, uint8_t u8FlashBlockLen){
    uint8_t i;
    
    addr = addr -1;
    
    TBLPTRU = (uint8_t)(addr>>16);
    TBLPTRH = (uint8_t)(addr>>8);
    TBLPTRL = (uint8_t)(addr);

    for(i=0;i<u8FlashBlockLen;i++){
        TABLAT = FlashData[i];        
        asm("TBLWT+*");
    }

    #asm
        BSF WREN_bit
        BCF GIE_bit
        MOVLW 0x55
        MOVWF EECON2
        MOVLW 0xAA
        MOVWF EECON2
        BSF WR_bit
        BSF GIE_bit
        BCF WREN_bit
    #endasm
}

void putch(uint8_t c){
    TX2IF = 0;
    TXREG2 = c;
    while(!TX2IF);
    
}

uint8_t u8getch(uint8_t * error){
    uint16_t i;
    uint8_t ch;
    for(i=0;i<10000;i++){
        if(PIR3bits.RC2IF == 1){
            *error = ERROR_OK;
            ch = RCREG2;
            return ch;    
        }
        //__delay_ms(1);
        __delay_us(200);
    }
    *error = ERROR_RXTIMEOUT;
    return 0;
}

void vEraseFlash(void){
    for(address=ROMSTART;address<(ROMEND-1024);address+=1024){
       vEraseBlock(address); 
    }
}

void vEraseBlock(uint32_t addr){
    
    TBLPTRU = (uint8_t)(addr>>16);
    TBLPTRH = (uint8_t)(addr>>8);
    TBLPTRL = (uint8_t)(addr);

    #asm
        BSF WREN_bit
        BSF FREE_bit
        BCF GIE_bit
        MOVLW 55h
        MOVWF EECON2
        MOVLW 0AAh
        MOVWF EECON2
        BSF WR_bit
        BSF GIE_bit
    #endasm
}
