#include <stdio.h>
#include <iostream>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <cstring>
#include <cmath>

#define SERVOLEN 2
#define PUMPENPIN 19//muss geändert werden
//C-Code für das Handling des Picos
class Servo;
int sped=50;
char entr[25];
Servo* Servos[SERVOLEN];
int Servoindex=0;
volatile bool timer_fired=false;
//1 Pumpe an
//2 Schrittmotor
//3 Servo 1
// 4 Servo 2
//anschlag arm 1 600 bois 2500
//2600=180°
//600:winkel 18°
//2600:Winkel 193
//450 hoch hür schrittmot
//3,700 danach
//4,1400
//1,150 runter?

class Servo{
    //handler für servo,der Servo arbeitet mit Pico-internen Timerfunktionen
    public:
    int servopin;
    bool aktiv=false;
    unsigned int dur=1500;
    int i;
    absolute_time_t dserv;
    Servo(int servpin){
        Servos[Servoindex]=this;
        Servoindex++;
        if(Servoindex>SERVOLEN){
            puts("ERROR:SERVO");
        }
        i=0;
        gpio_init(servpin);
        gpio_set_dir(servpin, GPIO_OUT);
        dserv=get_absolute_time();
        servopin=servpin;
    }
    bool stellen(int stell){
        //setzen der Position des Servos, soll nicht unter 500 Mikroseconden und nicht über 2600 liegen (Stellgrenzen)
        int z_temp=dur;
        servdel=0;
        if(stell>=500 && stell<=2600){
    while(abs(dur-stell)>2){
  dur=z_temp+(stell-z_temp)*pow(sin(((float)servdel*2*M_PI)/350),2);
  servdel++;
//std::cout<<dur<<","<<stell<<std::endl;
            sleep_ms(10);
    }
         return false;   
        }

        return true;
    }

    // void wackeln(){ nicht genutzt
    //     int stand=dur;
    //     if(stand<1000){
    //         dur=1000;
    //         sleep_ms(200);
    //     }
    //     if(stand>2100){
    //         dur=2100;
    //         sleep_ms(200);
    //     }
    //     stand=dur;
    //     for(float i=0;i<3000;i+=1){
    //         dur=stand+200*sin(i/600);
    //         sleep_us(40);
    //     }
    //     dur=stand;
    // }
    unsigned int servdel=10;

};

class Schrittmot{

    public:
    unsigned int schrittdel=5000;
    int schrittpin; 
    int dirpin;
    bool status;
    int schritte=0;
    bool vorwaerts=true;
    absolute_time_t dschritt;
    Schrittmot(int schrittpn,int dirpn){
        dirpin=dirpn;
        schrittpin=schrittpn;
        status=0;
            gpio_init(schrittpin);
    gpio_set_dir(schrittpin, GPIO_OUT);
                gpio_init(dirpin);
    gpio_set_dir(dirpin, GPIO_OUT);
        dschritt=get_absolute_time();
    }

    void setDir(bool state){
        gpio_put(dirpin,state);
    }
    void leveln(){
        //Greifarm mit endschalter versehen, auf diese weise kann die Position festgestellt werden, diese Funktion ist dafür veranrwortlich
        gpio_put(dirpin,0);
        schritte=1000;
        while(gpio_get(17))schritt();
        schritte=450;
        gpio_put(dirpin,1);
        while(schritt());
    }
    bool schritt(){
        //genutzt fürs bewegen des Schrittmotors. Die funktion kann über eine While schleife betrieben werden, ko können auh andere tasks gleichzeitig erledigt werden
        if(schritte==0){
        return false;
        }
            if(absolute_time_diff_us(dschritt,get_absolute_time())>=schrittdel){
                
                    //status=true;
                    dschritt=get_absolute_time();
                    status=status==0;
                    gpio_put(schrittpin,status);
                    schritte--;
            }
            return true;
    }
// bool fahrn(){
// 	if(absolute_time_diff_us(dschritt,get_absolute_time())>=schrittdel){
// 	dschritt=get_absolute_time();
// 	dschritt=get_absolute_time();
//                 status=status==0;
//                 gpio_put(schrittpin,status);
            
// }
// }
};
bool repeating_timer_callback(struct repeating_timer *t) {
    //Timerfunktion des Pico, kontrolliert die servos.
    // Der timer kann sich um mehrere Servos kümmern und setzt unterschiedliche pins iuns einem zeit raum von 20 millisekunden
    //(Periodendauer des Regelsignals eines Servos)
    static int akt=0;
    static absolute_time_t messen=get_absolute_time();

    char zu[9];
    unsigned long z=absolute_time_diff_us(messen,get_absolute_time());

    if(!timer_fired && akt==0){
    messen=get_absolute_time();
    }
    int del=timer_fired ? 3000-Servos[akt]->dur:Servos[akt]->dur; //500 extra delay
        gpio_put(Servos[akt]->servopin,!timer_fired);//anmachen, wenn zyklus noch nicht durch, sonst ausmachen
    timer_fired = timer_fired==0;
    if(!timer_fired){
        if(akt>=Servoindex-1){
            del=19990-absolute_time_diff_us(messen,get_absolute_time());
            akt=0;
            if(del<0){
                puts("ERRORTIMER");
                del=20000;
            }
        }
        else{
        akt++;
        }

    }

    t->delay_us=del;
    //printf("Repeat at %lld\n", time_us_64());
    return true;
}
enum derStatus{PUMPE,SCHRITTMOTOR_RAUF,SCHRITTMOTOR_RUNTER,SERVO_1,SERVO_2};
void set_State(int& Status,int&dur,Schrittmot* step){
    bool failed=false;
    switch(Status){
        case PUMPE:
        gpio_put(PUMPENPIN,dur);
        break;

        case SCHRITTMOTOR_RAUF:
        if (step==NULL) return;
        step->schritte=dur;
        step->setDir(0);//vielleicht stimmt die richtung nicht
        while(step->schritt());
        break;

        case SCHRITTMOTOR_RUNTER:
        if (step==NULL) return;
        step->schritte=dur;
        step->setDir(1);
        while(step->schritt());
        break;

        case SERVO_1:
        failed=Servos[0]->stellen(dur);

        break;

        case SERVO_2:
        failed=Servos[1]->stellen(dur);
        break;
    }
    if(!failed){
    puts("0");
    }
    else{
        puts("1");
    }
}
bool ser_proz(int& Status,int&dur,char* entr) {
    //warten auf input und speichern dieses in seperate chararray, um sie später als Kommandos zu verarbeiten 
    //char akt;
    int akt=getchar_timeout_us(1000);
    static int index=0;
    while (akt!=PICO_ERROR_TIMEOUT){

    
    //fgets(entr,8,stdin);
        if(akt==',' || akt>=48 && akt<=57){
        entr[index]=akt;
        index++;
        }
akt=getchar_timeout_us(1000);
    }
    if(index==0){
        return false;
    }
    index=0;
    sscanf(entr,"%i,%i",&Status,&dur);
    char erg[10];
    sprintf(erg,"%i,,%i",Status,dur);
memset(entr,'\0',25);


return true;
}



int main(void)
{
    int mov=0;  
    unsigned long i=0;

    int Status=0;
    int dur=0;
    stdio_init_all();
    gpio_init(10);
    gpio_init(PUMPENPIN);
    gpio_init(17);
    gpio_set_dir(17,GPIO_IN);
    gpio_set_dir(PUMPENPIN, GPIO_OUT);
    gpio_set_dir(10, GPIO_OUT);
    //gpio_put(10,0);
    Servo serv1(12);
    Servo serv2(13);
    
    char entr[25];
    struct repeating_timer timer;
    add_repeating_timer_us(20000, repeating_timer_callback, NULL, &timer);
    //printf("\n\nHello World\n");
    Schrittmot* step=NULL;
    Schrittmot schr1(11,10);
    schr1.leveln();
    //Servo serv1(18);
    //multicore_launch_core1(core1_entry);
    char printbuf[25];
    while (true)
    {
        //puts("beginn");
        //sleep_ms(1000);
        while(!ser_proz(Status,dur,entr));//wartet im prinzip die ganze zeit nur auf input
        set_State(Status,dur,&schr1);
        sprintf(printbuf,"%i",gpio_get(17));

    }
}

