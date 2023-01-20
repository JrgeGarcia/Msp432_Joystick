#include "msp.h"
#include <stdio.h>
#include <math.h>
char bufferTerminal[50];
float Vx;
float Vy;
float voltagex = 0;
float voltagey = 0;
void ADC_init(void){
    //ADC Joystick channels
    ADC14->CTL0 = ADC14_CTL0_SHT0_2 | ADC14_CTL0_SHP | ADC14_CTL0_CONSEQ_1 | ADC14_CTL0_MSC| ADC14_CTL0_ON;
    ADC14->CTL1 = ADC14_CTL1_RES_2;
    ADC14->MCTL[0] |= ADC14_MCTLN_INCH_0;//channel A0
    ADC14->MCTL[3] |= ADC14_MCTLN_INCH_3 | ADC14_MCTLN_EOS; //A2
    ADC14->IER0 |= ADC14_IER0_IE0;
    P5->SEL1 |= (BIT2 | BIT5);
    P5->SEL0 |= (BIT2 | BIT5);
}
void UART_INIT(void){
    /*
     * P1.2->RX
     * P1.3->TX
     */
    P1->SEL0 |= 0X0C;
    P1->SEL1 &= ~0X0C;
    //UART0 CONFIGURATION
    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST;                               // Clear previous configuration of UART
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SSEL__SMCLK;                        // Select SMClock, no parity, 1 stop bit, 8 bits, LSB
    EUSCI_A0->BRW = 19;                                                  // Baudrate width, SMClock/16/DR -> 3000000/16/9600 = 19.53125
    EUSCI_A0->MCTLW = (9 << EUSCI_A_MCTLW_BRF_OFS | EUSCI_A_MCTLW_OS16); // // 19.53125 - 19 = 0.53125 * 16 = 8.5, round up to 9
    EUSCI_A0->CTLW0 &= ~(EUSCI_A_CTLW0_SWRST);                           // clear reset bit
    /*
     * P3.2->RX
     * P3.3->TX
     */
    P3->SEL1 |= 0X0C;
    P3->SEL0 &= ~0X0C;
    //UART2 CONFIGURATION
    EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST;                               // Clear previous configuration of UART
    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SSEL__SMCLK;                        // Select SMClock, no parity, 1 stop bit, 8 bits, LSB
    EUSCI_A2->BRW = 19;                                                  // Baudrate width, SMClock/16/DR -> 3000000/16/9600 = 19.53125
    EUSCI_A2->MCTLW = (9 << EUSCI_A_MCTLW_BRF_OFS | EUSCI_A_MCTLW_OS16); // // 19.53125 - 19 = 0.53125 * 16 = 8.5, round up to 9
    EUSCI_A2->CTLW0 &= ~(EUSCI_A_CTLW0_SWRST);
}
void UART0_puts(char *str){
    while(*str !='\0'){
    while(!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG));
    EUSCI_A2->TXBUF = *str++;
    }
}
void UART2_puts(char *str){
    while(*str !='\0'){
    while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
    EUSCI_A0->TXBUF = *str++;
    }
}
void main(void)
{
    WDT_A->CTL = 0x5A80;        // stop watchdog timer
    UART_INIT();
    ADC_init();
    /*
     * sampling time
     */
    while (1){
    /*
     * start the sampling conversion for joystick
     */
    ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;
    while(!ADC14->IFGR0);
    __delay_cycles(12e6);
    voltagex = ADC14->MEM[0];
    voltagey = ADC14->MEM[3];
    Vx = (3.3*voltagex)/(4095) - .647;
    Vy = (3.3*voltagey)/(4095) - .647;
    sprintf(bufferTerminal, "\rXVoltage: %.2f V\n\YVoltage: %.2f", Vx, Vy);
    UART0_puts(bufferTerminal);
    UART2_puts(bufferTerminal);
    }
}
