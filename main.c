
// PIC18F4550 Configuration Bit Settings

//João Vitor de Oliveira Camara

//Tópico 3 - Interrupções e Timer


// 'C' source line config statements

// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = INTOSCIO_EC// Oscillator Selection bits (Internal oscillator, port function on RA6, EC used by USB (INTIO))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>     //biblioteca com muitas funções padrões
#include "stdint.h"

#define _XTAL_FREQ 8000000  //CONFIGURA FUNÇÃO DELAY PARA SUPOSTO CLOCK DE 8MHZ
#define disp1 PORTBbits.RB7 //definindo um novo nome para o PORTB
#define disp2 PORTBbits.RB6 //habilita a alimentação

//Variaveis auxiliares
uint8_t piscaLED = 0;
uint8_t aux = 1;


void __interrupt() rotinaAltaPrioridade(void);   //endereço da memória 0x008
void __interrupt(low_priority) rotinaBaixaPrioridade(void);  //endereço da memória 0x018


void main(void) {
    //Configuração do clock
    OSCCONbits.IRCF = 0b111;    //o registrador OSCCON configura o oscilador interno para 8mhz
    OSCCONbits.SCS = 0b11;      //o oscilador interno é definido como clock do sistema
    //while(OSCCONbits.IOFS != 1){ //Espera até que IOFS seja 1 garantindo a estabilidade de  frequência
    //} //só funciona em circuitos reais
    
    //Configuração das portas
    TRISB = 0b00001111;     //configurando as saidas e entradas
    TRISD = 0x00;           //configurando as saidas e entradas em hexa
    
    PORTB = 0;  //numero em decimal
    PORTD = 0xFF; //numero em hexa
    
    
    //Configurações do timer 
    T0CONbits.TMR0ON = 1;   //habilita o timer0
    T0CONbits.T08BIT = 1;   //seleciona o contador de 8 bits
    T0CONbits.T0CS = 0;     //seleciona o clock interno (ciclo de máquina, Fosc/4)
    T0CONbits.PSA = 0;      //o sinal de clock vem do prescaler
    T0CONbits.T0PS = 0b101; //divide o ciclo de máquina por 64
    TMR0 = 130;             //TMR0IF é setado quando TMR0 vai de 255 para 0 (255-125=130)
    //Incrementa o timer em 125*64*4*(1/8mhz) = 4ms
    
    
    
    //Configuração das interrupções
    RCONbits.IPEN = 1;      //Habilita a prioridade das interrupções
    
    INTCONbits.TMR0IE = 1;   //Habilita a interrupção pelo estouro do timer0
    INTCON2bits.TMR0IP = 1; //Alta prioridade no TMR0
    
    //RB0:
    INTCONbits.INT0IE = 1;     //Habilita a interrupção na porta RB0
    INTCON2bits.INTEDG0 = 0;    //Configura INT0 para SETAR a flag INT0IF na borda de descida
    
    INTCONbits.GIEH = 1;    //habilita as interrupções de alta prioridade
    INTCONbits.GIEL = 1;    //habilita as interrupções de baixa prioridade
    

    while(1){
       
        if(piscaLED>=250){  //250*4ms= 10002ms =~ 1seg pisca LED
            disp2 = ~disp2; //desabilita a alimentação no RB6
            piscaLED = 0;
        }
        
    }
    
    return;
}


void __interrupt() rotinaAltaPrioridade(void){   //endereço da memória 0x008
    INTCONbits.GIEH = 0;    //Disables all interrupts
    
    if(INTCONbits.INT0IF){ //testa flag do INT0 [LIGA DESLIGA]
        if(aux == 1){  //piscando
            T0CONbits.TMR0ON=1; // Timer0 On/Off Control bit [ON]
        }
        if(aux == 2){  //Ligado
            T0CONbits.TMR0ON=0; //// Timer0 On/Off Control bit [OFF]
            disp2=1;
        }
        if(aux == 3){ //Desligado
            disp2 = 0; //Desliga o display
            aux = 0;
        }
        
        aux++;  //aux retorna a valer 1 para o estado inicial
        //disp1 = ~disp1;
        INTCONbits.INT0IF = 0;  //Após executar o necessário limpa a flag para não entrar na função sem que ocorra o proximo evento
    }
    
    //função para o timer0
    if(INTCONbits.TMR0IF){  //testa a flag
        piscaLED++;
        INTCONbits.TMR0IF = 0;
        TMR0 = 130;
    }
    
    INTCONbits.GIEH = 1;    //Enable all unmasked interrupts
}

void __interrupt(low_priority) rotinaBaixaPrioridade(void){  //endereço da memória 0x018
    /*INTCONbits.GIEH = 0;
    
    if(INTCON3bits.INT1IF){ //testa flag do RB1
        disp2 = ~disp2;
        INTCON3bits.INT1IF = 0;  //Após executar o necessário limpa a flag para não entrar na função sem que ocorra o proximo evento
    }
    
    INTCONbits.GIEH = 1; */
}

