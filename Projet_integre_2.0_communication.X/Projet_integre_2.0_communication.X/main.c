/*
 * Main file of Projet_intégré_2.0.X
 * Author: De Saulnier K. 
 *         Latte S.
 *
 * 
 */

#include "xc.h"
#include "adc.h"
#include "math.h"
#include "FskDetector.h"
#define D 1024
#define pi acos(-1.0)
#define FCY 3685000     // µcycle frequency. Needed for __delay_ms
#include "libpic30.h"   // Contains __delay_ms definition

void calcul(unsigned int sample, int* sortie);
void detect_enveloppe(int sortie0, int sortie1, int* envel);
//int calcul_900(unsigned int sample);
//int calcul_1100(unsigned int sample);
//int filtre(long g[4], long a1[4], long a2[4], long b1[4], long b2[4], long *inter[9]);
//void decalage(long *inter[9]);
//int detect_enveloppe_900(int sortie_900);
//int detect_enveloppe_1100(int sortie_1100);


int sortie[2] = {0,0};
int envel[2] = {0,0};


int main(void) {
    int a;
    unsigned int sample = 0, i = 0;
    int message;
    int param = 87;
    int delay = 2000;
    int ordre = 87;
    int param_recu = 77;
    int ordre_recu = 77;
    int err_ordre = 0;
    int err_param = 0;
    int err_stop = 0;
    int start = 254;
    int stop = 253;
    float error = 252;
    int led = 252;
    //int sortie_900;
    //int sortie_1100;
    //int envel_900;
    //int envel_1100;
   
    frcPll40MHzConfig();
    TRISBbits.TRISB3 = 0; // On initialise en sortie
    
    // Initialisation des LEDs
    TRISBbits.TRISB9 = 0; // On initialise en sortie
    TRISBbits.TRISB10 = 0; // On initialise en sortie
    TRISBbits.TRISB11 = 0; // On initialise en sortie
    TRISBbits.TRISB12 = 0; // On initialise en sortie

    
    TMR3 = 0;
    T3CONbits.TON = 1;
    
    // Configuration de L'ADC pour utilisation en polling sur AN0
    adcInit(ADC_TIMER3_SAMPLING);
    PR3 = 2666;                 // T=66.67µs=(PR1+1)/40MHz => PR1+1=2666.67

        /* Configuration du Peripheral Pin Select (PPS) pour connecter le signal
     * Rx de l'UART1 à RB6/RP6 et le signal Tx à RB7/RP7 */
    _U1RXR = 7;     // U1RX -> Réception de RB4 vers RB7
	_RP6R = 0b00011;    // U1TX -> Transmission de RB6 vers RB5


    // Configuration de l'UART1 avec un format de trame 8N1, à 57600 bits/s
    U1MODEbits.PDSEL = 0;       // 8 bits, pas de parité
    U1MODEbits.STSEL = 0;       // 1 stop bit
    /* L'UART peut fonctionner à 2 vitesses différentes. Le mode "high speed" 
     * est plus sensible au bruit et ne doit donc être utilisé que pour
     * des débits trop rapides pour le mode standard 
     * En mode standard, le débit est donné par :
     * baud rate = FCY / (16*(U1BRG+1))
     * => U1BRG = (3.685MHz / (16*57.6kHz)) - 1  =  2.998*/
    U1MODEbits.BRGH = 0;
    U1BRG = 42.4;
    
    U1MODEbits.UARTEN = 1;      // on active l'UART
    U1STAbits.UTXEN = 1;        // on active l'émission
    
       
    /*float t=0;
    while(1){
        if(U1STAbits.URXDA){
            sample = U1RXREG;
            if(sample == 's'){
                TMR3 = 0;
                T3CONbits.TON = 1;
                for(i=0; i<5000;i++){
                  while(!adcConversionDone()){}
                  _LATB15 = 1;
                  t = t+(1.0/15000.0);
                  sample = adcRead();
                  calcul(sample, sortie);
                  detect_enveloppe(sortie[0],sortie[1], envel);
                  _LATB15 = 0;
                  while(U1STAbits.UTXBF){}
                  U1TXREG = envel[1];
                }
                T3CONbits.TON = 0;
            }
        }
    }*/
    
    
    while (U1STAbits.URXDA) {U1RXREG;} // Vide le buffer de réception 
    while(1){
        while(!adcConversionDone()){}
        sample = adcRead();
        calcul(sample, sortie);
        detect_enveloppe(sortie[0],sortie[1], envel);
        message = FskDetector(envel[0], envel[1]);

        if (message != 0){
            ordre = (message&0xFF00)>>8;
            param = message&0x00FF;
            _LATB3 = 1;
            __delay_ms(delay);
            _LATB3 = 0;
            if (ordre == 0){
                _LATB9 = 1;
            }
            if (ordre == 1){
                _LATB10 = 1;
            }
            if (ordre == 2){
                _LATB11 = 1;
            }
            if (ordre == 3){
                _LATB12 = 1;
            }
            
            while(U1STAbits.UTXBF){} // Partie émission 1  (émet le start)    U1TXREG; était dans les crochets
            U1TXREG = start;
            
            while(U1STAbits.UTXBF){} // Partie émission 2  émet l'ordre
            U1TXREG = ordre;
            
            while(!U1STAbits.URXDA){} // Partie réception 1   reçoit l'ordre reçu
            ordre_recu = U1RXREG;
            if(ordre_recu == ordre){
                
                while(U1STAbits.UTXBF){} // Partie émission 3    émet le param
                U1TXREG = param;
                
                while(!U1STAbits.URXDA){} // Partie réception 2       reçoit le param reçu
                param_recu = U1RXREG;
                
                if(param_recu == param){
                    while(U1STAbits.UTXBF){} // Partie émission du stop correct
                    U1TXREG = stop;
                }
                
                else{          
                    while(U1STAbits.UTXBF){} // Partie émission  du mauvais stop car le param reçu est faux
                    U1TXREG = err_stop;
                    break;
                }
            }
            
            else{                
                while(U1STAbits.UTXBF){} // Partie émission du param d'erreur
                U1TXREG = error;
                break;
            }
            while(!U1STAbits.URXDA){} // Partie réception led
            if (U1RXREG == led){
                _LATB9 = 0;
                _LATB10 = 0;
                _LATB11 = 0;
                _LATB12 = 0;
            }
        }
    }
}


    static long g_900[4] = {D*0.002019, D*0.001964, D*0.0228, D*0.02294};
    static long a1_900[4] = {D*(-1.84602348), D*(-1.84997046), D*(-1.85002393), D*(-1.8593739)};
    static long a2_900[4] = {D*0.98747166, D*0.98763379, D*0.99474384, D*0.99490711};
    static long b1_900[4] = {D*2, D*2, D*(-2), D*(-2)};
    static long b2_900[4] = {D*1, D*1, D*1, D*1};
    /*static long Y1_900[3] = {0, 0, 0};
    static long Y2_900[3] = {0, 0, 0};
    static long Y3_900[3] = {0, 0, 0};
    static long Y4_900[3] = {0, 0, 0};
    static long y1_900[3] = {0, 0, 0};
    static long y2_900[3] = {0, 0, 0};
    static long y3_900[3] = {0, 0, 0};
    static long out_900[3] = {0, 0, 0};
    static long in_900[3] = {0, 0, 0};
    static long *inter_900[9] = {in_900, Y1_900, y1_900, Y2_900, y2_900, Y3_900, y3_900, Y4_900, out_900};*/

    static long g_1100[4] = {D*0.003035, D*0.002955, D*0.02263, D*0.02271};
    static long a1_1100[4] = {D*(-1.77495574  ), D*(-1.7807248), D*(-1.77882789  ), D*(-1.79257281)};
    static long a2_1100[4] = {D*0.98471132, D*0.98490396, D*0.99358215, D*0.99377646};
    static long b1_1100[4] = {D*2, D*2, D*(-2), D*(-2)};
    static long b2_1100[4] = {D*1, D*1, D*1, D*1};
    /*static long Y1_1100[3] = {0, 0, 0};
    static long Y2_1100[3] = {0, 0, 0};
    static long Y3_1100[3] = {0, 0, 0};
    static long Y4_1100[3] = {0, 0, 0};
    static long y1_1100[3] = {0, 0, 0};
    static long y2_1100[3] = {0, 0, 0};
    static long y3_1100[3] = {0, 0, 0};
    static long out_1100[3] = {0, 0, 0};
    static long in_1100[3] = {0, 0, 0};
    static long *inter_1100[9] = {in_1100, Y1_1100, y1_1100, Y2_1100, y2_1100, Y3_1100, y3_1100, Y4_1100, out_1100};*/
    
void calcul(unsigned int sample, int* sortie){
    static long inter_900[9][3];
    static long inter_1100[9][3];
    
    inter_900[0][2] = sample;
    
    inter_1100[0][2] = sample;
    
    int k;
    for(k=0;k<8;k++){
        inter_900[k+1][2] = (D*inter_900[k][2] + b1_900[k/2]*inter_900[k][1] + b2_900[k/2]*inter_900[k][0]) - a1_900[k/2]*(inter_900[k+1][1]/D) - a2_900[k/2]*(inter_900[k+1][0]/D); // On stocke les Y
        inter_900[k+2][2] = ((g_900[k/2]*inter_900[k+1][2])/D)/D; // On stocke les y
        
        inter_900[k][0]  =  inter_900[k][1];
        inter_900[k][1]  =  inter_900[k][2];
        inter_900[k+1][0] = inter_900[k+1][1];
        inter_900[k+1][1] = inter_900[k+1][2];
        k++;
    }
    inter_900[8][0]  =  inter_900[8][1];
    inter_900[8][1]  =  inter_900[8][2];
    
    for(k=0;k<8;k++){
        inter_1100[k+1][2] = (D*inter_1100[k][2] + b1_1100[k/2]*inter_1100[k][1] + b2_1100[k/2]*inter_1100[k][0]) - a1_1100[k/2]*(inter_1100[k+1][1]/D) - a2_1100[k/2]*(inter_1100[k+1][0]/D); // On stocke les Y
        inter_1100[k+2][2] = ((g_1100[k/2]*inter_1100[k+1][2])/D)/D; // On stocke les y
        inter_1100[k][0]  =  inter_1100[k][1];
        inter_1100[k][1]  =  inter_1100[k][2];
        inter_1100[k+1][0] = inter_1100[k+1][1];
        inter_1100[k+1][1] = inter_1100[k+1][2];
        k++;
    }
    inter_1100[8][0]  =  inter_1100[8][1];
    inter_1100[8][1]  =  inter_1100[8][2];
       
    sortie[0] = inter_900[8][2];
    sortie[1] = inter_1100[8][2]; 
}


void detect_enveloppe(int sortie0,int sortie1, int* envel){
    static int vect_900[15];    
    static int vect_1100[15];    
    static int max_1100 = 0;    
    static int max_900 = 0;
    static int i = 0;
    static int j = 0;
    static int output_900 = 0;
    static int output_1100 = 0;
    int a;

    int seuil_900 = 250;
    int seuil_1100 = 250;
   
    vect_900[i] = sortie0;
    vect_1100[j] = sortie1;
    
    if(max_900<vect_900[i]){
        max_900 = vect_900[i];
    }
    i++;
    if(i>15){
        i = 0;
        if(max_900 > seuil_900){
            output_900 = 1;
            max_900 = 0;
        }
        else{
            output_900 = 0;
            max_900 = 0;
        }
    }
    
    if(max_1100<vect_1100[j]){
        max_1100 = vect_1100[j];
    }
    j++;
    if(j>15){
        j = 0;
        if(max_1100 > seuil_1100){
            output_1100 = 1;
            max_1100 = 0;
        }
        else{
            output_1100 = 0;
            max_1100 = 0;
        }
    }
    envel[0] = output_900;
    envel[1] = output_1100;
}


/*int calcul_900(unsigned int sample){
    static long g[4] = {D*0.002019, D*0.001964, D*0.0228, D*0.02294};
    static long a1[4] = {D*(-1.84602348), D*(-1.84997046), D*(-1.85002393), D*(-1.8593739)};
    static long a2[4] = {D*0.98747166, D*0.98763379, D*0.99474384, D*0.99490711};
    static long b1[4] = {D*2, D*2, D*(-2), D*(-2)};
    static long b2[4] = {D*1, D*1, D*1, D*1};
    static long Y1[3] = {0, 0, 0};
    static long Y2[3] = {0, 0, 0};
    static long Y3[3] = {0, 0, 0};
    static long Y4[3] = {0, 0, 0};
    static long y1[3] = {0, 0, 0};
    static long y2[3] = {0, 0, 0};
    static long y3[3] = {0, 0, 0};
    static long out[3] = {0, 0, 0};
    static long in[3] = {0, 0, 0};
    long *inter[9] = {in, Y1, y1, Y2, y2, Y3, y3, Y4, out};
    
    in[2] = sample;
    inter[8][2] = filtre(g, a1, a2, b1, b2, inter);
    //decalage(inter);
    return inter[8][1];  
}

int calcul_1100(unsigned int sample){
    static long g[4] = {D*0.003035, D*0.002955, D*0.02263, D*0.02271};
    static long a1[4] = {D*(-1.77495574  ), D*(-1.7807248), D*(-1.77882789  ), D*(-1.79257281)};
    static long a2[4] = {D*0.98471132, D*0.98490396, D*0.99358215, D*0.99377646};
    static long b1[4] = {D*2, D*2, D*(-2), D*(-2)};
    static long b2[4] = {D*1, D*1, D*1, D*1};
    static long Y1[3] = {0, 0, 0};
    static long Y2[3] = {0, 0, 0};
    static long Y3[3] = {0, 0, 0};
    static long Y4[3] = {0, 0, 0};
    static long y1[3] = {0, 0, 0};
    static long y2[3] = {0, 0, 0};
    static long y3[3] = {0, 0, 0};
    static long out[3] = {0, 0, 0};
    static long in[3] = {0, 0, 0};
    long *inter[9] = {in, Y1, y1, Y2, y2, Y3, y3, Y4, out};
    
    in[2] = sample;
    inter[8][2] = filtre(g, a1, a2, b1, b2, inter);
    //decalage(inter);
    return inter[8][1];  
}

int filtre(long g[4], long a1[4], long a2[4], long b1[4], long b2[4], long *inter[9]){
    int k;
    for(k=0;k<8;k++){
        inter[k+1][2] = (D*inter[k][2] + b1[k/2]*inter[k][1] + b2[k/2]*inter[k][0]) - a1[k/2]*(inter[k+1][1]/D) - a2[k/2]*(inter[k+1][0]/D); // On stocke les Y
        inter[k+2][2] = ((g[k/2]*inter[k+1][2])/D)/D; // On stocke les y
        inter[k][0]  =  inter[k][1];
        inter[k][1]  =  inter[k][2];
        inter[k+1][0] = inter[k+1][1];
        inter[k+1][1] = inter[k+1][2];
        k++;
    }
    inter[8][0]  =  inter[8][1];
    inter[8][1]  =  inter[8][2];
    return inter[8][2];
}

void decalage(long *inter[9]){
    int j;
    int l;
    for (j=0;j<9;j++){ //on itère sur les colonnes 
        for (l=0;l<2;l++){ //on itère sur les lignes
            inter[j][l] = inter[j][l+1];
        }
    }
}

int detect_enveloppe_900(int sortie_900){
    static int vect_900[15];
    static int i = 0;
    int k = 0;
    int output;
    static int max = 0;
    int seuil = 255;
    vect_900[i] = sortie_900;
    max = 0;
    for(k=0;k<15;k++){
        if(vect_900[k]>max){
            max = vect_900[k];
        }
    }
    if(max>seuil){
        output = 1;
    }
    else{
        output =  0;
    }
    i++;
    if(i>=15){
        i=0;
    }
    return output;
}

int detect_enveloppe_1100(int sortie_1100){
    static int vect_1100[15];
    static int i = 0;
    int k = 0;
    int output;
    static int max = 0;
    int seuil = 255;
    vect_1100[i] = sortie_1100;
    max = 0;
    for(k=0;k<15;k++){
        if(vect_1100[k]>max){
            max = vect_1100[k];
        }
    }
    if(max>seuil){
        output = 1;
    }
    else{
        output =  0;
    }
    i++;
    if(i>=15){
        i=0;
    }
    return output;
}*/