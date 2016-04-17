//#define USE_AND_MASKS
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
//#include "motore.h" DEBUG!!
#include "motore4685.h"
#include "CANlib.h"
#include "delay.h"
#include "delay.c"
#include <pwm.h>
#include <math.h>
#include <stdlib.h>
#include "idCan.h"
#include "timers.h"
//#include <usart.h>//debug
#define _XTAL_FREQ 16000000
#define attesaRampa 20

void configurazione_iniziale(void);
void send_data(void);
void battery_measure(void);
void can_interpreter(void);
void rampe(void);

CANmessage msg;
volatile bit new_message = 0;
volatile bit remote_frame = 0;
volatile bit remote_frame1 = 0;
volatile bit speed_fetched = 0;
volatile bit message_sent = 0;
volatile bit can_retry = 0;
volatile bit request_sent = 0;
volatile bit centralina_sterzo = 0;
volatile bit centralina_abs = 0;
volatile bit centralina_comando = 0;
volatile unsigned int dir = 1;
volatile long currentSpeed = 0;
volatile long requestSpeed = 0;
volatile unsigned long id = 0;
volatile unsigned long timeCounter = 0; //1 = 10mS
unsigned long previousTimeCounter = 0;
unsigned long previousTimeCounter1 = 0;
unsigned long previousTimeCounter2 = 0;
unsigned int previousPwm = 0;
volatile unsigned int left_speed = 0;
volatile unsigned int right_speed = 0;
long duty_set = 0;
int duty_cycle = 0;
unsigned int errore = 0;
int vBatt = 0; //tensione batteria
long correzione = 0;
BYTE counter_array [8] = 0;
BYTE currentSpeed_array [8] = 0;
BYTE data_array [8] = 0;
BYTE data_array_debug [8] = 0;
volatile unsigned char current[] = 0;
volatile unsigned char scrittura = 0;
//====================================================================
//ISR Gestione messaggi can e funzione di conteggio temporale
//====================================================================

__interrupt(low_priority) void ISR_bassa(void) {
    if ((PIR3bits.RXB0IF == 1) || (PIR3bits.RXB1IF == 1)) {
        if (PIR3bits.RXB0IF == 1) {
            PORTCbits.RC1 = 1;
        }
        if (CANisRxReady()) { //Se il messaggio è arrivato
            CANreceiveMessage(&msg); //leggilo e salvalo
            remote_frame = msg.RTR;
            id = msg.identifier; //salvo il dato per interpretarlo
            for (char i = 0; i < 8; i++) {
                data_array[i] = msg.data[i]; //salvo il dato per interpretarlo
            }
            if (id == SPEED_CHANGE) { //variazione velocità 
                requestSpeed = data_array[1]; //velocità richiesta 
                requestSpeed = ((requestSpeed << 8) | data_array[0]);
                dir = data_array[2]; //direzione richiesta
            }
            new_message = 1;
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
    OpenTimer2(TIMER_INT_OFF & T2_PS_1_16 & T2_POST_1_16);
    period = 0xFA;
    OpenEPWM1(period);
    speed_fetched = 1;
    SetOutputEPWM1(FULL_OUT_FWD, PWM_MODE_1);
    while (1) {
        can_interpreter();
        if (PORTAbits.RA1 == 0) {
            // if ((PORTAbits.RA1 == 0)&&(PORTCbits.RC1 == 0)) {
            if ((timeCounter - previousTimeCounter1 >= attesaRampa)) {
                rampe();
            }
        } else {
            SetDCEPWM1(0);
        }
        if ((can_retry == 1)&&(remote_frame)) {
            send_data();
        }
        //        FUNZIONE DI SICUREZZA
        if (((timeCounter - previousTimeCounter) > 500) || (PORTAbits.RA1 == 1)&&((timeCounter - previousTimeCounter) > 5)) {
            if (request_sent == 0) {
                while (CANisTxReady() != 1) {
                }
                CANsendMessage(ECU_STATE, data_array, 8, CAN_CONFIG_STD_MSG & CAN_REMOTE_TX_FRAME & CAN_TX_PRIORITY_0); //remote frame per richiedere presenza centraline
                request_sent = 1;
            } else {
                if ((centralina_abs == 1)&&(centralina_sterzo == 1)&&(centralina_comando == 1)) {
                    centralina_abs = 0;
                    centralina_sterzo = 0;
                    centralina_comando = 0;
                    PORTAbits.RA1 = 0;
                    request_sent = 0;
                } else {
                    // SetDCEPWM1(0);
                    PORTAbits.RA1 = 1;
                    delay_ms(200);
                    PORTAbits.RA1 = 0;
                    delay_ms(200);
                    PORTAbits.RA1 = 1;
                    request_sent = 0;
                }
            }
            previousTimeCounter = timeCounter;
        }
        if ((timeCounter - previousTimeCounter2 > 100)) { //misura la tensione della batteria ogni secondo
            battery_measure();
            previousTimeCounter2 = timeCounter;
        }
    }
}

void rampe(void) {
    if (requestSpeed > 1) {
        while (CANisTxReady() != 1);
        CANsendMessage(ACTUAL_SPEED, data_array_debug, 8, CAN_CONFIG_STD_MSG & CAN_REMOTE_TX_FRAME & CAN_TX_PRIORITY_0);
        if (speed_fetched == 1) {
            if (dir == 1) { //direzione avanti
                SetOutputEPWM1(FULL_OUT_FWD, PWM_MODE_1);
            }
            if (dir == 0) { //direzione indietro
                SetOutputEPWM1(FULL_OUT_REV, PWM_MODE_1);
            }
            speed_fetched = 0;
            currentSpeed = ((left_speed + right_speed) / 2);
            if (currentSpeed == 0) {
                SetDCEPWM1(1000);
                previousPwm = 1000;
            }
            errore = abs((currentSpeed - requestSpeed));
            // 
            if (errore > 3000) {
                correzione = pow(2, (errore / 200)) - 1;
            } else if (errore > 2000) {
                correzione = pow(2, (errore / 150)) - 1;
            } else {
                correzione = pow(2, (errore / 100)) - 1;
                //                correzione = ((errore / 150)*(errore / 150))*2;
                //                correzione = correzione / 20;
            }

            if (currentSpeed - requestSpeed > 0) {
                duty_set = previousPwm - correzione;
                if (duty_set < 0) {
                    duty_set = 0;
                }
            } else {
                duty_set = previousPwm + correzione;
                if (duty_set > 1022) {
                    duty_set = 1023;
                }
            }
        }
        previousPwm = duty_set;
        previousTimeCounter1 = timeCounter;
        SetDCEPWM1(duty_set); //imposta pwm
    } else {
        SetDCEPWM1(0);
    }
}

void send_data(void) {
    if (CANisTxReady()) { //è disponibile almeno un buffer per l'invio dei dati
        if (remote_frame == 1) { //se è una risposta a un remote frame deve avere lo stesso id della richiesta
            CANsendMessage(id, data_array, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
            remote_frame = 0; //azzero flag risposta remote frame
        }
    }
    if ((TXB0CONbits.TXABT == 1) || (TXB1CONbits.TXABT == 1)) { //se l'invio è stato abortito
        can_retry = 1;
    } else {
        can_retry = 0;
    }
}

void battery_measure(void) {
    ADCON0bits.GO = 1; //abilita conversione ADC;
    while (ADCON0bits.GO);
    vBatt = ADRESH;
    vBatt = (vBatt * 14) / 255; //
    if (vBatt < 8) {
        while (CANisTxReady() != 1);
        CANsendMessage(LOW_BATTERY, data_array, 8, CAN_CONFIG_STD_MSG & CAN_REMOTE_TX_FRAME & CAN_TX_PRIORITY_0);
        PORTCbits.RC1 = 1;
    } else {
        PORTCbits.RC1 = 0;
    }
}

void can_interpreter(void) {
    if (new_message == 1) {


        if (id == EMERGENCY) { //stop di emergenza
            requestSpeed = 0;
            PORTAbits.RA1 = 1;
        }

        if (id == ACTUAL_SPEED) {
            /*==================================
             *La velocita' viene trasmessa a 2 BYTE
             *così facendo si ottiene una maggiore precisione
             *e puo' esprimere in mm/s
             ==================================*/
            left_speed = data_array[1];
            left_speed = ((left_speed << 8) | (data_array[0]));
            right_speed = data_array[3];
            right_speed = ((right_speed << 8) | (data_array[2]));
            speed_fetched = 1;
        }

        if (id == ECU_STATE) { //funzione per presenza centraline
            if (data_array[0] == 0x01) {
                centralina_abs = 1;
                // PORTCbits.RC5 = 0; //DEBUG
            }
            if (data_array[0] == 0x02) {
                centralina_sterzo = 1;
                //centralina_comando = 1; //DEBUG! <==
                //centralina_comando = 1; //debug
                //PORTCbits.RC4 = 0; //DEBUG
            }
            if (data_array[0] == 0x03) {
                centralina_comando = 1;
            }
        }
    }
    new_message = 0;
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