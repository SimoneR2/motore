/*==============================================================================
 *PROGRAM: MOTORE
 *WRITTEN BY: Simone Righetti
 *DATA: 18/01/2016
 *VERSION: 3.0
 *FILE SAVED AS: motore.c
 *FOR PIC: 18F4480
 *CLOCK FREQUENCY: 16 MHz
 *PROGRAM FUNCTION: Centralina che gestisce il motore della macchina. Con 
 *retroazione sulla velocità impostata rispetto al pwm. Il sistema utilizza
 *una rampa a parabola con reazione piuttosto rapida durante i transistori
 *e risposta più moderata a regime. Incorporata anche funzione di sicurezza
 *che ogni secondo verifica il corretto funzionamento delle centraline 
 *necessarie all'operatività in sicurezza del mezzo e che lo ferma in caso
 *di risposta negativa. La centralina inoltre si occupa, attraverso un partitore
 *con rapporto 1.8 : 1 e zener di protezione, di misurare la tensione della 
 *batteria per progetterla dalle scariche eccessive che la rovinerebbero.

======================================          
=         INPUT AND OUTPUTS          =            
=   RA0 => Tensione batteria (ADC)   =            
=   RA1 => Warning LED               =            
=   RD4-5-6-7 => ECCP (PWM Motore)   =   
=   RB2/RB3 => CANBus                =            
======================================

 * CODICI CENTRALINE PER RISPOSTA ECU_STATE
 *  -ABS! : 1
 *  -STERZO : 2
 *  -COMANDO : 3
 *Frequenza PWM: 20kHz.

 */
#include <xc.h>
#include "motore.h"
#include "CANlib.h"
#include "delay.h"
#include "delay.c"
#include <pwm.h>
#include <math.h>
#include <stdlib.h>
#include "idCan.h"
#include "timers.h"
#define _XTAL_FREQ 16000000
#define attesaRampa 15

void configurazione_iniziale(void);
void send_data(void);
void battery_measure(void);

CANmessage msg;
bit remote_frame = 0;
bit remote_frame1 = 0;
bit speed_fetched = 0;
bit message_sent = 0;
bit can_retry = 0;
bit request_sent = 0;
bit centralina_sterzo = 0;
bit centralina_abs = 0;
bit centralina_comando = 0;
volatile unsigned int dir = 1;
volatile unsigned char currentSpeed = 1;
volatile unsigned char requestSpeed = 0;
unsigned long counter = 0;
unsigned long id = 0;
unsigned long id1 = 0;
unsigned long timeCounter = 0; //1 = 10mS
unsigned long previousTimeCounter = 0;
unsigned long previousTimeCounter1 = 0;
unsigned long previousTimeCounter2 = 0;
int previousPwm = 200;
char left_speed = 0;
char right_speed = 0;
int duty_set = 200;
int duty_cycle = 0;
int errore = 0;
int vBatt = 0; //tensione batteria
unsigned int correzione = 0;
BYTE counter_array [8] = 0;
BYTE currentSpeed_array [8] = 0;
BYTE data_array [8] = 0;
BYTE data_array1 [8] = 0;
BYTE data_array_debug [8] = 0;

//****************************************************************
//ISR Gestione messaggi can e funzione di conteggio temporale
//****************************************************************

__interrupt(low_priority) void ISR_bassa(void) {
    if ((PIR3bits.RXB0IF == 1) || (PIR3bits.RXB1IF == 1)) {
        if (CANisRxReady()) { //Se il messaggio è arrivato
            CANreceiveMessage(&msg); //leggilo e salvalo
            if (msg.identifier == SPEED_CHANGE) { //variazione velocità 
                //TOGLIERE DUE COMMENTI!!!!
                requestSpeed = msg.data[0]; //velocità richiesta 
                dir = msg.data[1]; //direzione richiesta
                if (dir == 1) { //direzione avanti
                    SetOutputEPWM1(FULL_OUT_FWD, PWM_MODE_1);
                }
                if (dir == 0) { //direzione indietro
                    SetOutputEPWM1(FULL_OUT_REV, PWM_MODE_1);
                }
                //--------------------------
                // requestSpeed = 
                //previousTimeCounter = timeCounter;
            }
            if (msg.identifier == EMERGENCY) { //stop di emergenza
                requestSpeed = 0;
                PORTAbits.RA1 = 1;
            }
            if (msg.identifier == ACTUAL_SPEED) {

                left_speed = msg.data[0];
                right_speed = msg.data[1];
                speed_fetched = 1;

            }
            if (msg.identifier == ECU_STATE) { //funzione per presenza centraline
                switch (msg.data[0]) {
                    case 1: centralina_abs = 1;
                        break;
                    case 2: centralina_sterzo = 1;
                        centralina_comando = 1; //debug
                        break;
                    case 3: centralina_comando = 1;
                        break;
                }

            }
        }
        PIR3bits.RXB0IF = 0;
        PIR3bits.RXB1IF = 0;
    }
    if (PIR2bits.TMR3IF) { //interrupt timer, ogni 10mS
        timeCounter++; //incrementa di 1 la variabile timer
        TMR3H = 0x63; //reimposta il timer
        TMR3L = 0xC0; //reimposta il timer
        PIR2bits.TMR3IF = 0; //azzera flag interrupt timer
    }
}

int main(void) {
    unsigned char period;
    configurazione_iniziale();
    PORTAbits.RA1 = 1;
    PORTCbits.RC1 = 1;
    delay_ms(500);
    PORTAbits.RA1 = 0;
    PORTCbits.RC1 = 0;
    OpenTimer2(TIMER_INT_OFF & T2_PS_1_16 & T2_POST_1_1);
    period = 0xFE;
    OpenEPWM1(period);
    speed_fetched = 1;
    SetOutputEPWM1(FULL_OUT_FWD, PWM_MODE_1);
    SetDCEPWM1(300);
    delay_ms(500);
    SetDCEPWM1(0);
    while (1) {
        // if (PORTAbits.RA1 == 0) {
        if ((timeCounter - previousTimeCounter1 >= attesaRampa)) {
            CANsendMessage(ACTUAL_SPEED, data_array_debug, 8, CAN_CONFIG_STD_MSG & CAN_REMOTE_TX_FRAME & CAN_TX_PRIORITY_0);
            if (speed_fetched == 1) {
                PORTCbits.RC1 = ~PORTCbits.RC1; //debug
                speed_fetched = 0;
                currentSpeed = ((left_speed + right_speed) / 2);
                requestSpeed = 25; //DEBUG
                errore = abs((currentSpeed - requestSpeed)*10);
                correzione = ((errore / 17)*(errore / 17))*2;
                if (correzione > 3) {
                    if ((currentSpeed - requestSpeed) > 0) {
                        duty_set = previousPwm - correzione;
                        if (duty_set < 0) {
                            duty_set = 0;
                        }
                    } else {
                        duty_set = previousPwm + correzione;
                        if (duty_set > 1024) {
                            duty_set = 1023;
                        }
                    }
                } else {
                    duty_set = previousPwm;
                }
                previousPwm = duty_set;
            }
            previousTimeCounter1 = timeCounter;
            SetDCEPWM1(duty_set); //imposta pwm
        }

        if ((remote_frame == 1) || (can_retry == 1)) { //se è arrivato un remote frame la risposta è immediata
            send_data();
        }
        if ((CANisTXwarningON() == 1) || (CANisRXwarningON() == 1)) {
            //SetDCEPWM1(0); //ferma il mezzo DEBUG
            PORTAbits.RA1 = 1; //accendi led errore
            delay_ms(200);
            PORTAbits.RA1 = 0;
            delay_ms(200);
            PORTAbits.RA1 = 1;
            COMSTATbits.TXWARN = 0;
            COMSTATbits.RXWARN = 0;
        } else {
            PORTAbits.RA1 = 0;
        }

        //FUNZIONE DI SICUREZZA
        if ((timeCounter - previousTimeCounter) > 500) {
            if (request_sent == 0) {
                CANsendMessage(ECU_STATE, data_array, 8, CAN_CONFIG_STD_MSG & CAN_REMOTE_TX_FRAME & CAN_TX_PRIORITY_0); //remote frame per richiedere presenza centraline
                request_sent = 1;
            }
            if (request_sent == 1) {

                if ((centralina_abs == 1)&&(centralina_sterzo == 1)&&(centralina_comando == 1)) {
                    centralina_abs = 0;
                    centralina_sterzo = 0;
                    centralina_comando = 0;
                    PORTAbits.RA1 = 0;
                } else {
                    // SetDCEPWM1(0);
                    PORTAbits.RA1 = 1;
                    delay_ms(200);
                }
            }
            previousTimeCounter = timeCounter;
        }
        if ((timeCounter - previousTimeCounter2 >= 100)) { //misura la tensione della batteria ogni secondo

            battery_measure();
            previousTimeCounter2 = timeCounter;
        }
    }
}

void send_data(void) {
    if (CANisTxReady()) { //è disponibile almeno un buffer per l'invio dei dati
        if (remote_frame == 1) { //se è una risposta a un remote frame deve avere lo stesso id della richiesta
            CANsendMessage(id, data_array, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
            remote_frame = 0; //azzero flag risposta remote frame
        }
        if (can_retry == 1) {
            CANsendMessage(id1, data_array1, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
        }
    }
    if ((TXB0CONbits.TXABT) || (TXB1CONbits.TXABT)) { //se l'invio è stato abortito
        can_retry = 1;
        id1 = id;
        remote_frame1 = remote_frame;
        for (char i = 0; i < 8; i++) {
            data_array1[i] = data_array[i];
            TXB0CONbits.TXABT = 0;
            TXB1CONbits.TXABT = 0;
        }
    } else {
        can_retry = 0;
        if (remote_frame1 != 0) {
            remote_frame = remote_frame1;
            id = id1;
            message_sent = 1;
            remote_frame1 = 0;
        } else {

            message_sent = 0;
        }
    }
    remote_frame = 0; //azzero flag risposta remote frame
}

void battery_measure(void) {
    ADCON0bits.GO = 1; //abilita conversione ADC;
    while (ADCON0bits.GO);
    vBatt = ADRESH;
    vBatt = (vBatt * 14) / 255; //
    if (vBatt < 8) {
        while (!CANisTxReady());
        CANsendMessage(LOW_BATTERY, data_array, 8, CAN_CONFIG_STD_MSG & CAN_REMOTE_TX_FRAME & CAN_TX_PRIORITY_0);
        PORTCbits.RC1 = 1;
    } else {
        PORTCbits.RC1 = 0;

    }
}

void configurazione_iniziale(void) {

    //impostazione periferica can bus e azzeramento flag interrupt
    CANInitialize(4, 6, 5, 1, 3, CAN_CONFIG_LINE_FILTER_OFF & CAN_CONFIG_SAMPLE_ONCE & CAN_CONFIG_ALL_VALID_MSG & CAN_CONFIG_DBL_BUFFER_ON);
    RCONbits.IPEN = 1; //abilita priorità interrupt
    INTCONbits.INT0IF = 0; //azzera flag interrupt RB0
    PIR3bits.RXB1IF = 0; //azzera flag interrupt can bus buffer1
    PIR3bits.RXB0IF = 0; //azzera flag interrupt can bus buffer0

    IPR3bits.RXB1IP = 0; //interrupt bassa priorità per can
    IPR3bits.RXB0IP = 0; //interrupt bassa priorità per can
    INTCONbits.GIEH = 1; //abilita interrupt 
    INTCONbits.GIEL = 1; //abilita interrupt periferiche

    //impostazione timer3 per contatore========
    T3CON = 0x01; //abilita timer
    PIR2bits.TMR3IF = 0; //resetta flag interrupt timer 3
    IPR2bits.TMR3IP = 0; //interrupt bassa priorità timer 3
    TMR3H = 0x63;
    TMR3L = 0xC0;

    //==========================================

    //impostazione ADC ==================================
    ADCON0bits.ADON = 1; //attiva modulo ADC
    ADCON0bits.CHS0 = 0; //AN0 come input ADC
    ADCON0bits.CHS1 = 0; //AN0 come input ADC
    ADCON0bits.CHS2 = 0; //AN0 come input ADC
    ADCON0bits.CHS3 = 0; //AN0 come input ADC
    ADCON1bits.PCFG0 = 0; //AN0 input analogico
    ADCON1bits.PCFG1 = 1; //AN0 input analogico
    ADCON1bits.PCFG2 = 1; //AN0 input analogico
    ADCON1bits.PCFG3 = 1; //AN0 input analogico
    ADCON1bits.VCFG0 = 0; //reference interno
    ADCON1bits.VCFG1 = 0; //reference interno
    ADCON2bits.ADCS0 = 1; //Tad = Fosc/16
    ADCON2bits.ADCS1 = 0; //Tad = Fosc/16
    ADCON2bits.ADCS2 = 1; //Tad = Fosc/16
    ADCON2bits.ACQT0 = 0; //tempo di acquisizione 16TAD
    ADCON2bits.ACQT1 = 1; //tempo di acquisizione 16TAD
    ADCON2bits.ACQT2 = 1; //tempo di acquisizione 16TAD
    ADCON2bits.ADFM = 0; //Left Justified
    //=======================================================

    PIE3bits.RXB1IE = 1; //abilita interrupt ricezione can bus buffer1
    PIE3bits.RXB0IE = 1; //abilita interrupt ricezione can bus buffer0
    PIE2bits.TMR3IE = 1; //abilita interrupt timer 3 
    //impostazione porte
    LATA = 0x00;
    TRISA = 0b01111101;

    LATB = 0x00;
    TRISB = 0b11111011;

    LATC = 0x00;
    TRISC = 0x00;

    LATD = 0x00;
    TRISD = 0x00;

    LATE = 0x00;
    TRISE = 0xFF;
    delay_ms(2);
    SetOutputEPWM1(FULL_OUT_FWD, PWM_MODE_1);

}