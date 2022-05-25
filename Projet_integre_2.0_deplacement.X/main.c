/*
 * Main file of Projet_intégré_2.0.X
 * Author: De Saulnier K. 
 *         Latte S.
 *
 * 
 */

#include "xc.h"         // Contains register variable definitions
#include "libpic30.h"   // Contains __delay_ms definition
#define pi acos(-1.0)
#define r 0.041
#define a_max 0.50 // m par seconde²
#define v_max = 0.50 // m par seconde
#define kp 3.69 // [m^-1]
#define E 0.134 // Empattement
#define correct_transl 1 // 1.025
#define correct_rot 1.1025
#define FCY 3685000     // µcycle frequency. Needed for __delay_ms
#define Period 2666      // Period = 40MHz/15000 -1 = 2666 donne la période du timer


void moteur_gauche(float dc);
void moteur_droite(float dc);
void regul(float* dc, float* err);
//float regul_transl(int* ordre);
//float regul_rotat();
float dc[2] = {0,0};
float err[4] = {10.0,-10.0,10.0,-10.0};
//float dc_rotat = 0;
//float dc_transl = 0;
float ordre = 33;
float param = 33;
float start = 254;
float stop = 253;
float led = 252;
//float err_transl = 10;
//float err_rot = 10;
float consigne_transl = 0; 
float consigne_rot = 0;
float distance[2] = {0,0};
float angle[2] = {0,0};
float distance_tot;
float angle_tot;
int delay = 1000;
   

int main(void){
    
    // paramètres et initialisation
    frcPll40MHzConfig();
    OC1RS = 0;
    OC2RS = 0;
    TMR2 = 0;
    T2CONbits.TON = 1; // On allume le timer2
    
    // PWM pour la roue de gauche 
    
    OC1CONbits.OCM = 0b110; // Selectionner le mode PWM d'output compare
    OC1CONbits.OCTSEL = 0; // On allume le timer2
    _RP15R = 0b10010; // Sur quelle patte mettre quel OCR 
    PR2 = Period;
    TRISBbits.TRISB13 = 0;
    
    // PWM pour la roue de droite
    
    OC2CONbits.OCM = 0b110; // Selectionner le mode PWM d'output compare
    OC2CONbits.OCTSEL = 0; // On allume le timer2
    _RP3R = 0b10011; // Sur quelle patte mettre quel OCR 
    TRISBbits.TRISB2 = 0;

    // Encodeur en quadrature QEI pour la roue gauche 
    
    QEI1CONbits.QEIM = 0b111;
    RPINR14bits.QEA1R = 6;
    RPINR14bits.QEB1R = 7;
    
    // Encodeur en quadrature QEI pour la roue droite 
    
    QEI2CONbits.QEIM = 0b111;
    RPINR16bits.QEA2R = 11;
    RPINR16bits.QEB2R = 12;
    
    // Définition des RX et TX 
    
	_RP8R = 3;  // TX sur pin 8
	_U1RXR = 5;  // RX sur pin 5  
    
    // Configuration de l'UART1 avec un format de trame 8N1, à 57600 bits/s
    U1MODEbits.PDSEL = 0;       // 8 bits, pas de parité
    U1MODEbits.STSEL = 0;       // 1 stop bit
    
    /*U1BRG = (3.685MHz / (16*57.6kHz)) - 1  =  2.998*/
    U1MODEbits.BRGH = 0;
    U1BRG = 42.4;
    
    U1MODEbits.UARTEN = 1;      // on active l'UART
    U1STAbits.UTXEN = 1;        // on active l'émission
    
    // Main (infinite) loop
    /*while(1) {
        if (ordre[0] == 0){
            dc_transl = regul_transl(ordre);
            //dc_rot = regul_rotat(ordre);
            moteur_gauche(dc_transl);
            moteur_droite(dc_transl); // Envoyer -dc pour rotation
        }
        if (ordre[0] == 1){
            regul(dc);
            moteur_gauche(dc[0]);
            moteur_droite(dc[1]); // Envoyer -dc pour rotation
        }
    }
    return 0;*/
    
    while (U1STAbits.URXDA) {U1RXREG;} // Vide le buffer de réception 
    while(1) {
        
        // Protocole de réception à l'aide d'une séquence de 4 octets : start - ordre - param - stop
        
        if(U1STAbits.URXDA){ // Partie réception du start
            if (U1RXREG == start){
                
                while (!U1STAbits.URXDA) {} // Partie réception 1 (tant qu'il n'a rien dans le buffer, reste sur la ligne)
                ordre = U1RXREG;
                
                while(U1STAbits.UTXBF){} // Partie émission 1
                U1TXREG = ordre;

                while (!U1STAbits.URXDA) {} // Partie réception 2 (tant qu'il n'a rien dans le buffer, reste sur la ligne)
                param = U1RXREG;

                while(U1STAbits.UTXBF){} // Partie émission 2
                U1TXREG = param;
                
                while (!U1STAbits.URXDA) {} // Partie réception du stop

                if (U1RXREG == stop){
                    if ((ordre == 0) || (ordre == 1)){ // Consigne translation
                        param = param*0.01;
                        if (ordre == 0){ // Avance
                            consigne_transl = param;
                            consigne_rot = 0;
                            while(err[0] > 0.015){
                            regul(dc, err);
                            moteur_gauche(dc[0]);
                            moteur_droite(dc[1]); // Changer dc[0] par dc[1] pour tourner dans autre sens
                            }
                        }
                        if (ordre == 1){                  // Recule
                            consigne_transl = -1*param;
                            consigne_rot = 0;
                            while(err[1] < -0.015){
                            regul(dc, err);
                            moteur_gauche(dc[0]);
                            moteur_droite(dc[1]); // Changer dc[0] par dc[1] pour tourner dans autre sens
                            }
                        }
                    }
                    if ((ordre == 2) || (ordre == 3)){ // Consigne rotation
                        param = (param*pi)/180;
                        if (ordre == 2){                      // Tourne à droite
                            consigne_rot = param;
                            consigne_transl = 0;
                            while(err[2] > 0.01){
                            regul(dc, err);
                            moteur_gauche(dc[0]);
                            moteur_droite(dc[1]); // Changer dc[0] par dc[1] pour tourner dans autre sens
                            }
                        }
                        if (ordre == 3){                      // Tourne à gauche
                            consigne_rot = -1*param;
                            consigne_transl = 0;
                            while(err[3] < -0.055){
                            regul(dc, err);
                            moteur_gauche(dc[0]);
                            moteur_droite(dc[1]); // Changer dc[0] par dc[1] pour tourner dans autre sens
                            }
                        }
                    }
                // On réinitialise les variables 
                dc[0] = 0;
                dc[1] = 0;
                moteur_gauche(dc[0]);
                moteur_droite(dc[1]); // Changer dc[0] par dc[1] pour tourner dans autre sens
                int a;
                a = 0;
                err[0] = 10.0;
                err[1] = -10.0;
                err[2] = 10.0;
                err[3] = -10.0;  
                distance_tot = 0;
                angle_tot = 0;
                POS1CNT = 0;
                POS2CNT = 0;
                while(U1STAbits.UTXBF){} // Partie émission LED
                U1TXREG = led;
                }
            }
        }
    }
    return 0;
}


void moteur_gauche(float dc){
    if (dc > 0){
        _LATB13 = 0;
    }
    else{
        dc = -dc;
        _LATB13 = 1;
    }
    OC1RS = dc*(Period+1);
    
}

void moteur_droite(float dc){
    if (dc > 0){
        _LATB2 = 0;
    }
    else{
        dc = -dc;
         _LATB2 = 1;
    }
    OC2RS = dc*(Period+1);
}

void regul(float* dc, float* err){
    float dc_transl;
    float dc_rot;
    float dc_somme; 
    float dc_diff;
    
    distance[0] = (2*pi*r*(int)POS1CNT)/1440; // Calcul de la distance de la roue gauche [m]
    distance[1] = (2*pi*r*(int)POS2CNT)/1440; // Calcul de la distance de la roue droite [m]
    distance_tot = (distance[0]+distance[1])/2;
        
    angle[0] = (2*pi*(int)POS1CNT)/1440; // Calcul de l'angle de la roue gauche [m]
    angle[1] = (2*pi*(int)POS2CNT)/1440; // Calcul de l'angle de la roue droite [m]
    angle_tot = r*(angle[0]-angle[1])/E;
    
    if (ordre == 0){ // Avancer
        err[0] = consigne_transl - distance_tot;
        err[2] = consigne_rot - angle_tot;
        dc_transl = err[0]*kp;
        dc_rot = err[2]*kp;
    }
    
    if (ordre == 1){ // Reculer
        err[1] = consigne_transl - distance_tot; // Voir si distrance_tot est négatif 
        err[3] = consigne_rot - angle_tot;
        dc_transl = err[1]*kp;
        dc_rot = err[3]*kp;
    }
    
    if (ordre == 2){ // Droite
        err[2] = consigne_rot - angle_tot;
        err[0] = consigne_transl - distance_tot;
        dc_rot = err[2]*kp;
        dc_transl = err[0]*kp;
    }
    
    if (ordre == 3){ // Gauche
        err[3] = consigne_rot - angle_tot; // Voir si distrance_tot est négatif 
        err[1] = consigne_transl - distance_tot; // Voir si distrance_tot est négatif
        dc_rot = err[3]*kp;
        dc_transl = err[1]*kp;
    }
    
    if (dc_transl > 0.3){
        dc_transl = 0.3;
    }
    
    if (dc_transl < -0.3){
        dc_transl = -0.3;
    }
    
    if (dc_rot > 0.25){
        dc_rot = 0.25;
    }
    if (dc_rot < -0.25){
        dc_rot = -0.25;
    }
    
    dc_somme = dc_transl + dc_rot;
    dc_diff = dc_transl - dc_rot;
    dc[0] = dc_somme;
    dc[1] = dc_diff;
}

/*float regul_transl(int*ordre){
    float distance[2] = {0,0};
    float consigne_transl = 1.5; // Distance de consigne
    float err;
    float distance_tot;
    float dc;
    distance[0] = (2*pi*r*(int)POS1CNT)/1440; // Calcul de la distance de la roue gauche [m]
    distance[1] = (2*pi*r*(int)POS2CNT)/1440; // Calcul de la distance de la roue droite [m]
    distance_tot = (distance[0]+distance[1])/2;
    err = consigne_transl - distance_tot;
    dc = err*kp;
    if (dc > 0.6){
        dc = 0.6;
    }
    if (dc < -0.6){
        dc = -0.6;
    }
    if (dc < 0.06){
        ordre[0] = 1;
        POS1CNT = 0;
        POS2CNT = 0;  
        OC1RS = 0;
        OC2RS = 0;
        __delay_ms(delay);
    }
    return dc;
}*/

/*float regul_rotat(){
    float err;
    float angle_tot;
    float dc;
    float angle[2] = {0,0};
    float consigne_rot = 2*pi;
    angle[0] = (2*pi*(int)POS1CNT)/1440; // Calcul de l'angle de la roue gauche [m]
    angle[1] = (2*pi*(int)POS2CNT)/1440; // Calcul de l'angle de la roue droite [m]
    angle_tot = r*(angle[0]-angle[1])/E;
    err = consigne_rot-angle_tot;
    dc = err*kp;
    if (dc > 0.3){
        dc = 0.3;
    }
    if (dc < -0.3){
        dc = -0.3;
    }
    /*if (dc < 0.06){
        ordre[0] = 1;
        POS1CNT = 0;
        POS2CNT = 0;  
        OC1RS = 0;
        OC2RS = 0;
        __delay_ms(delay);
        
    }
    return dc;
}
*/