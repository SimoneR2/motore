#define USE_AND_MASKS
/*prova github
 *dato velocità espresso in km/h
 *interrupt can
 *PWM sul pin RC2
 *LED ROSSO su RA0 per segnalare errori can bus
 *RA1 sensore di tensione
 */

#include <xc.h>
#include "motore.h"
#include <CANlib.h>
#include "delay.h"
#include "delay.c"
#include <pwm.h>
#include "timers.h"
#include <math.h>
#include <stdlib.h>
#define _XTAL_FREQ 16000000

#define start_measure 0b00000000000000000000000000110 //da spostare a centralina freno
#define stop_measure 0b00000000000000000000000000101 //da spostare a centralina freno
#define speed_info 0b00000000000000000000000001000 
#define speed_change 0b00000000000000000000000000010
#define speed_frequency 0b00000000000000000000000000010 //da definire id
#define emergency 0b00000000000000000000000000001
#define soglia1 50
#define soglia2 30
#define soglia3 10
#define soglia4 5
#define rampa 10
void configurazione_iniziale(void);
void send_data(void);

CANmessage msg;
bit misura = 0;
bit remote_frame = 0;
unsigned int dir = 0;
unsigned char currentSpeed = 1;
unsigned char requestSpeed = 0;
unsigned long counter = 0;
unsigned long id = 0;
unsigned long timeCounter = 0; //1 = 10mS
unsigned long previousTimeCounter = 0;
unsigned long previousTimeCounter1 = 0;
char previousPwm = 0;
char left_speed = 0;
char right_speed = 0;
unsigned char duty_set = 0;
int duty_cycle = 0;
int errore = 0;
unsigned int correzione = 0;
BYTE counter_array [8] = 0;
BYTE currentSpeed_array [8] = 0;
BYTE data_array [8] = 0;

//*************************************
//ISR Alta priorità (gestione encoder)
//*************************************

__interrupt(high_priority) void ISR_alta(void) {
    if (INTCONbits.INT0IF == 1) { //Se c'è stato un interrupt su PORTB
        delay_ms(1);
        if (misura == 1) { //Se è richiesta la misura di spazio percorso
            counter++; //Aggiunge 1 allo spazio percorso
        }
        INTCONbits.INT0IF = 0; //azzera flag interrupt
    }
}

//*************************************
//ISR Bassa priorità (gestione can bus)
//*************************************

__interrupt(low_priority) void ISR_bassa(void) {
    if ((PIR3bits.RXB0IF == 1) || (PIR3bits.RXB1IF == 1)) {
        if (CANisRxReady()) { //Se il messaggio è arrivato
            CANreceiveMessage(&msg); //leggilo e salvalo
            if (msg.RTR == 1) { //Se il messaggio arrivato è un remote frame
                id = msg.identifier;
                remote_frame = msg.RTR;
            }
            if (msg.identifier == start_measure) { //richiesto inizio della misura
                misura = 1;
            }
            if (msg.identifier == stop_measure) { //fine della misura, manda il dato (remote frame)
                misura = 0; //smetti di contare
                data_array[0] = counter; //LSB
                data_array[1] = counter >> 8; //casting variabile
                data_array[2] = counter >> 8; //casting variabile MSB
                counter = 0; //azzera la variabile contatore
            }
            if (msg.identifier == speed_info) { //richiesta velocità (remote frame)
                data_array[0] = currentSpeed; //impacchettamento dati velocità
            }
            if (msg.identifier == speed_change) { //variazione velocità 
                currentSpeed = msg.data[0]; //velocità richiesta
                dir = msg.data[1]; //direzione richiesta
                previousTimeCounter = timeCounter;
            }
            if (msg.identifier == emergency) { //stop di emergenza
                currentSpeed = 0;
                PORTAbits.RA1 = 1;
            }
            if (msg.identifier == speed_frequency) {
                left_speed = msg.data[0];
                right_speed = msg.data[1];
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
}

int main(void) {
    unsigned char period;
    configurazione_iniziale();
    OpenTimer2(TIMER_INT_OFF & T2_PS_1_1 & T2_POST_1_1);
    period = 249;
    OpenEPWM1(period);

    //DEBUG SEQUENCE---------
    PORTD = 0xFF;
    delay_ms(500);
    PORTD = 0x00;
    delay_ms(500);
    PORTD = 0xFF;
    delay_ms(500);
    PORTD = 0x00;
    delay_ms(500);
    //-----------------------

    while (1) {
        if (dir == 1) { //direzione avanti
            SetOutputEPWM1(FULL_OUT_FWD, PWM_MODE_1);
        }
        if (dir == 0) { //direzione indietro
            SetOutputEPWM1(FULL_OUT_REV, PWM_MODE_1);
        }
        if ((timeCounter - previousTimeCounter1 >= rampa)) {
            CANsendMessage(speed_frequency, 0, 0, CAN_CONFIG_STD_MSG & CAN_REMOTE_TX_FRAME & CAN_TX_PRIORITY_0);
            currentSpeed = ((left_speed + right_speed) / 2);
            if (currentSpeed - requestSpeed > 0) {//RAMPE
                errore = currentSpeed - requestSpeed;
                errore = abs(errore);
                correzione = (errore / 17)*(errore / 17);
                correzione = correzione * 4; //conversione da 8 a 10 bit per il pwm
                if (correzione > 1) {
                    if ((currentSpeed - requestSpeed) > 0) {
                        if (previousPwm > correzione) {
                            duty_set = previousPwm - correzione;
                        }
                        if ((currentSpeed - requestSpeed) < 0) {
                            duty_set = previousPwm + correzione;
                        }
                    }
                    if (correzione < 1) {
                        duty_set = previousPwm;
                    }
                }
                previousPwm = duty_set;
                SetDCEPWM1(duty_set); //imposta pwm
                if (remote_frame == 1) { //se è arrivato un remote frame la rispsota è immediata
                    send_data();
                }
                previousTimeCounter1 = timeCounter;
            }
        }
        if ((CANisTXwarningON() == 1) || (CANisRXwarningON() == 1)) {
            SetDCEPWM1(0); //ferma il mezzo
            PORTAbits.RA0 = 1; //accendi led errore
        }

        /*
         *Funzione di sicurezza, se l'istruzione di velocità
         *non è ripetuta almeno 1 volta al secondo blocca il mezzo
         *perchè vi è stato un qualche errore sulle altre centraline. 
         */
        while ((timeCounter - previousTimeCounter) > 100) {
            SetDCEPWM1(0);
            PORTAbits.RA0 = 1;
            delay_ms(250);
            PORTAbits.RA0 = 0;
            delay_ms(250);
            previousTimeCounter = timeCounter;
        }
    }
}

void send_data(void) {
    if (CANisTxReady()) { //è disponibile almeno un buffer per l'invio dei dati
        if (remote_frame == 1) { //se è una risposta a un remote frame deve avere lo stesso id della richiesta
            CANsendMessage(id, data_array, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
            remote_frame = 0; //azzero flag risposta remote frame
        }
    }
    if ((TXB0CONbits.TXABT) || (TXB1CONbits.TXABT)) { //se l'invio è stato abortito
        delay_ms(5); //aspetta 5 millisecondi e ritenta l'invio
        if (remote_frame == 1) { //se è una risposta a un remote frame deve avere lo stesso id della richiesta
            CANsendMessage(id, data_array, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
            remote_frame = 0; //azzero flag risposta remote frame
        }
    }
}

void configurazione_iniziale(void) {

    //impostazione periferica can bus e azzeramento flag interrupt
    CANInitialize(4, 6, 5, 1, 3, CAN_CONFIG_LINE_FILTER_OFF & CAN_CONFIG_SAMPLE_ONCE & CAN_CONFIG_ALL_VALID_MSG & CAN_CONFIG_DBL_BUFFER_ON);
    RCONbits.IPEN = 1; //abilita priorità interrupt
    INTCONbits.INT0IF = 0; //azzera flag interrupt RB0
    INTCONbits.INT0IE = 1; //abilita interrupt RB0
    PIR3bits.RXB1IF = 0; //azzera flag interrupt can bus buffer1
    PIR3bits.RXB0IF = 0; //azzera flag interrupt can bus buffer0
    PIE3bits.RXB1IE = 1; //abilita interrupt ricezione can bus buffer1
    PIE3bits.RXB0IE = 1; //abilita interrupt ricezione can bus buffer0
    IPR3bits.RXB1IP = 0; //interrupt bassa priorità per can
    IPR3bits.RXB0IP = 0; //interrupt bassa priorità per can
    INTCONbits.GIEH = 1; //abilita interrupt alta priorità
    INTCONbits.GIEL = 1; //abilita interrupt bassa priorità periferiche
    INTCON2bits.INTEDG0 = 1; //interrupt su fronte di salita

    //impostazione timer3 per contatore
    T3CON = 0x01; //abilita timer
    PIR2bits.TMR3IF = 0; //resetta flag interrupt timer 3
    IPR2bits.TMR3IP = 0; //interrupt bassa priorità timer 3
    TMR3H = 0x63;
    TMR3L = 0xC0;
    PIE2bits.TMR3IE = 1; //abilita interrupt timer 3 

    //impostazione porte
    LATA = 0x00;
    TRISA = 0b11111100;

    LATB = 0x00;
    TRISB = 0b11111011;

    LATC = 0x00;
    TRISC = 0x00;

    LATD = 0x00;
    TRISD = 0b00000000;

    LATE = 0x00;
    TRISE = 0xFF;

    ADCON1 = 0xFF;
}