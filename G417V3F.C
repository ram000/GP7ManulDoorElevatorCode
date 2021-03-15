#pragma regparms
#pragma objectextend


/*



G 4 to 17 :: #03-2021 # 


modified to keep dooropen when no calls

12/09/2010



modified from sion post on 20-feb-2010

file name salv3.c  copied from sionpost.c






Kanjawala G+ 6 :  03-11-2006  V3F : Man Door : 

Dadar Sircot  : modified G+2 ; announce on board : 26-02-2006
PF7 CSTM          20-08-2006 G+ 4 : Full V3F
SION POST         20-08-2006 G +4 : Full V3F 8574 used for UP colltg card
Chinh Bndr postr  20-08-2006 G+5  : Full V3F rest 8574 A 


G+13 Golden height 04-12-2005
G+3 Benner Bunglow 04-12-2005
G+10 FA            04-12-2005
G+6  Everest       03-12-2005
G+5  Baramathi     29-11-2005 : ic on 29-3-2006
G+7  NK Const      20-12-2005
Ganga Vihar        04-01-2005 
*/

#define FAN_OFF_DELAY  10;
// sionmtnl #define max_floors  7
// #define max_floors 5 dadar post 
#define max_floors 14

#define Call_Mask  0x7fff;   // for G + 7 : G+5 for Baramathi : 29-11-2005
#define Music 7;

#define _NB0804_

/*

FullColt.c    : 13-10-2004

# No of floors : G + 15 possible 

# timers CALLATDLY , WAITDLY , HOMEDLY , INSTIMER
# counters : FLRNO
# a0 : Car registration
# a1 : Landing call DOWN direction registration
# a4 : Landing call UP direction
# e0 : Car call buttons
# e1 : landing call DOWN buttons
# e4 : Landing call in UP direction 
# e2 : Att, DO , NS , STOP , KDN , KDN , DZ
# e3 : safety circuits

Functions :
        initializ all variables
        proceeds to home : upon LDN rached home is reset : floor ind set to
        Zero
        registers calls in landing & car
        direction lamp sets the diretion
        door closes and proceed to attend the call

        upon floor reached , slow speed activated
        stops exactly at door zone
        UP direction landing car calls attended in UP direction only
        DOWN direction ladning calls attenden in DOWN direction only 

e0 :  7        6     5      4       3     2    1    0   :      cop calls
e1 :  7        6     5      4       3     2    1    0   :      landing calls
e2 : PRXDN    DZ    KUP    KDN     STOP   NS    DO   ATT/AUTO
e3 : INSdN   INSUP  INS    PPRXUP   CG     LDN  LUP  FSW  
a0 :  7    6     5    4    3    2    1   0   :   COP  call register
a1 :  7    6     5    4    3    2    1   0   :   LC   call register
a2 :  A    B     C    D    E    F    G   H   :   Display No
a3 :      FAN    DOO  SLOW MTR  DIR  MTR DIR
    CALL         RLY       DN   DN   UP  UP
     ATT
*/

#include <reg51.h>
#include<intrins.h>
#include <stdio.h>


#ifdef _NB0804_
sbit SDA = P1^2;
sbit SCL = P1^3;
sbit WD = P1^0;
#endif

// with actual board 0408
sbit CS = P2^7;
sbit CLK = P2^6;
sbit DTA = P2^5;


/* include i2c.h */

#include <i2c.h>

char bdata a3,e2,e3, CH0,CH1,CH2,CH3,st, st1;
unsigned char data a2,a21, ff,PFLRNO,WAITDLY,HOMEDLY,tMtrOn ,dly,
        e00, e01, e02, e03,e04, e00h, e01h,e04h, MAX_FLR,
        e10, e11, e12, e13,e14, e10h, e11h,e14h, INSTMR,TEM,CH, STUKTMR,
        CALLMASK, tMtrOff ;

unsigned int   data REG_CALLS , BFLRNO,a0,a1,e0,e1,e4,a4,PlayTmr ; 
unsigned char e0_STK_TMR, e1_STK_TMR , e4_STK_TMR, TMR;
unsigned int  MASK_e0, MASK_e1, MASK_e4 ;


unsigned char Call_at_delay , Call_at_delay_i ;

/* sbit definitions */

sbit DIRUP  = a3^0 ;
sbit MTRUP  = a3^1 ;
sbit DIRDN  = a3^2 ;
sbit MTRDN  = a3^3 ;
sbit SLOW   = a3^4 ;
sbit DOORLY = a3^5 ;
sbit FAN    = a3^6 ;
sbit DC     = a3^7 ;
bit tbDC = 0 ; 
bit CALLAT = 0 ;
sbit ATT    = e2^0 ;
sbit DR     = e2^1 ;
sbit NS     = e2^2 ;
//sbit STOP   = e2^3 ;
sbit LG     = e2^3 ;

sbit KDN    = e2^4 ;
sbit KUP    = e2^5 ;
sbit DZ     = e2^6 ;
sbit PRXDN  = e2^7 ;

sbit FSW    = e3^0 ;
sbit LUP    = e3^1 ;
sbit LDN    = e3^2 ;
sbit CG     = e3^3 ;
sbit PRXUP  = e3^4 ;
sbit INS    = e3^5 ;
sbit INSUP  = e3^6 ;
sbit INSDN  = e3^7 ;

sbit NS_F       = st^7 ;
sbit CUDL      = st^6 ;
sbit CDDL      = st^5 ;

sbit LUPI      = st^3 ;
sbit acpbit    = st^2 ;
sbit LDNI      = st^1 ;
sbit bcpbit    = st^0 ;


sbit callatg0   = st^4 ;



bit FLRCALLED ; 

sbit R20  = st1^0;
sbit R21  = st1^1;
sbit R22  = st1^2;
sbit R23  = st1^3;
sbit pATT = st1^4 ;
sbit pMtrUp = st1^5 ;
sbit pMtrDn = st1^6;
sbit pFManSw = st1^7;

sbit  relay = P3^1;


bit sta,MVG, PPRXUP,PPRXDN, SEMG,HOME, bPLAY,bPLAYMUSIC ,pSLOW ,
//        R20, R21, R22, R23,

        R24, R25, BCP1,R30,R31,R32,R33, R34, R35, ACP1, ACP_C, BCP_C,
        WAIT , RUN ,STARTING, e0_STK_CNT ; 

//sbit  relay = P3^1;



code unsigned char NOTBL [] = {
      0x03f, 0x06, 0x05B, 0x04F, 0x066, 0x06D, 0x07D, 0x007,
//      0x03f,  0x05B, 0x04F, 0x066, 0x06D, 0x07D, 0x007,      
// floor number one removed for sircot Dadar 
        0x07F, 0x06F, 0x077, 0x07C, 0x039, 0x05E, 0x07B, 0x071,
        0x03D, 0x076 ,0x0E, 0x038, 0x037, 0x054, 0x05C, 0x073,
        0x031, 0x06C, 0x03E, 0x040, 0x06E, 0x0, 0x0
        } ;

data unsigned char doedly,A,R3 ,scount,i;
data unsigned char dcedly,dodly,dcdly, S1 , FLRNO, TIC,CALLATDLY,
                sta1, tFLRNO;
unsigned int secount = 0 ;
data unsigned int dely;

unsigned char STK_DELAY;
bit e0_stk, e1_stk, e4_stk;


void wd(void) { EA = 0 ; WD = 0 ; WD = 1 ;  EA = 1 ; }

void delay_1()
{
for(dely = 0;dely<=655; dely++) wd();
}



void delay_2()
{
for(dely = 0;dely<=2; dely++) wd();
}


/* sout_v()  for player */

void sout_v(unsigned char dly) {

  for (i=0;i<8;i++)
    {
     DTA =  (( dly >> i ) & 0x01 ) ; 
     CLK = 1;CLK = 1 ;
//     delay_2();
     CLK = 0  ;
    }
}

/* play()  */

void play(dly)
{
}

void play1(dly) {
}
/* serialin */

void serialin(void)
{

e00 = ~( i2c_in(0x40)) ;
e01 = ~( i2c_in(0x42)) ;

e02 = ~( i2c_in(0x44)) ;
e03 = ~( i2c_in(0x46)) ;

wd();


e00h= ~( i2c_in(0x48)) ;
e01h= ~( i2c_in(0x4a)) ;


e04 = ~( i2c_in(0x4c)) ;

e04h= ~( i2c_in(0x4e)) ;


wd();
}

/* serialout */

void serialout (void)
{


CH = ( a0 % 256 ) ; i2c_out (0x70,~CH);

CH = ( a0 / 256 ) ; i2c_out (0x78,~CH);


wd();
CH  = a1 % 256 ;    i2c_out (0x72,~CH);

CH  = a1 / 256 ;    i2c_out (0x7a,~CH);


                    i2c_out (0x74,~a2);

                    i2c_out (0x76,~a3);
wd();


CH  = a4 % 256 ;    i2c_out (0x7c,~CH);

CH  = a4 / 256 ;    i2c_out (0x7e,~CH);



}


/* timer setting */

void init(void)
{               // setting timer 10 msec mode
                TMOD=0x1;
                TH0 = 0xfb;
                TF0 = 1 ;
                TR0 = 1 ;
                ET0 = 1 ;
                EA  = 1 ;
}

void acp(void) {
      REG_CALLS = a1 | a0 | a4 |e0  | e1|e4       ;
      ACP1 = REG_CALLS >> (FLRNO + 1) ;
}

void bcp (void)
{
      REG_CALLS = a1 | a0 | a4 |e0 | e1 | e4         ;
      BCP1 = REG_CALLS << (16- FLRNO ) ;
wd();
}

void acpc (void) { ACP_C = ( a0 >> (FLRNO + 1) ) ;}

void bcpc (void)  { BCP_C = ( a0 << (16- FLRNO) ) ;}


/* main */

void main (void) {

    P0=255; P1=255; P2=255; P3=255;
    init();
    STUKTMR = 0 ;
    a0=a1=a2=a21= a3=e1=e2=e3=a4=e4 = 
    e00=e01=e02=e03=e10=e11=e12=e13= INSTMR= TEM = 0;
    FLRNO = PPRXUP = PRXUP =PPRXDN = PRXDN=  HOME = 0 ;
    HOMEDLY = 30;
wd();
    NS_F= sta=sta1=MVG=  FLRCALLED=SEMG=HOME= 0;
    R20= R21= R22= R23 =R24 =R25 =
    BCP1=R30= R31 =R32=R33=R34=R35=ACP1=WAIT = 0;
    ACP_C = BCP_C = 
    CUDL = CDDL = STARTING = 0 ;
    FLRNO =  BFLRNO = 0 ;
    bPLAYMUSIC = bPLAY = 0;
    MAX_FLR = max_floors ;
//  CALLMASK = Call_Mask ;

    e0_STK_TMR = e1_STK_TMR = e4_STK_TMR = 0 ;
    MASK_e0= MASK_e1 = MASK_e4 =  Call_Mask ;
    e0_STK_CNT = 0 ; st1 = tMtrOn = tMtrOff =  pFManSw = 0; 
     NS_F = TMR = 0 ;

    PlayTmr = 0 ;     STK_DELAY= 20;

serialout();

i2c_out(0x40,255);i2c_out(0x42,255);i2c_out(0x44,255);i2c_out(0x46,255);
wd();
i2c_out(0x48,255);i2c_out(0x4a,255);i2c_out(0x4c,255);
//i2c_out(0x4e,255);

delay_1();
#ifndef _TEST_
sta1 = 0 ;

while(1)
{

wd();


/* read input stat */
if (sta1==0)
        {
        serialin();
        e10  = e00 ;  e11  = e01;
        e10h = e00h;  e11h = e01h ;
        e14  = e04 ;  e14h = e04h ;
        e12  = e02 ;  e13  = e03 ;
        }
if (sta1==1)
{
        serialin();
        e0 = (e00h & e10h) * 256 ;        e0 +=  (e00 & e10 ) ;
        e1 = (e01h & e11h) * 256 ;        e1 +=  (e01 & e11 ) ;
        e4 = (e04h & e14h) * 256 ;        e4 +=  (e04 & e14 ) ;

//      e1 = ( e0 >> 4) & 0x000f ;
wd();
        e2 = e12 & e02 ;
        e3 = e13 & e03 ;
//        e0 &= MASK_e0   ;     e1 &= MASK_e1   ;  e4 &= MASK_e4  ;
//  e0 &= Call_Mask ;
//  e1 &= Call_Mask ;
//  e4 &= Call_Mask ; 

}
/* process */
if (sta1>=2)
{
 
// stuck process 
   // cancelling masks 

    MASK_e0 |= ~e0 ;
    MASK_e1 |= ~e1;
    MASK_e4 |= ~e4 ;

    MASK_e0 &= Call_Mask ;
    MASK_e1 &= Call_Mask ;
    MASK_e4 &= Call_Mask ;

    e0 &= MASK_e0 ;
    e1 &= MASK_e1 ;
    e4 &= MASK_e4 ;



     //masking 
 
    if ( (STK_DELAY ==0  ) && !ATT) 
      {

      if (e0 & BFLRNO ) {

                         MASK_e0 &= ~BFLRNO ;


                         e0 &= ~BFLRNO;


                         a0 &= ~BFLRNO;

                         }
    
      if (e1 & BFLRNO ) {

                         MASK_e1 &= ~BFLRNO ;


                         e1 &= ~BFLRNO;


                         a1 &= ~BFLRNO;

                         }
   
  
      }

















// Call_at_delay , call_at_delay_i setting from switch

/* read delay timings */

Call_at_delay = (P0 & 0x0f ) ;

Call_at_delay_i = ( P0& 0xf0 ) /16  ;

//Call_at_delay = (P0 & 0x07 )* 2;
//Call_at_delay_i = ( P0& 0x38 ) / 4  ;



//Call_at_delay = 8 ;
//Call_at_delay_i = 8;





LUP = ! (LUP) ;
LDN = ! (LDN) ; /* negate LUP & LDN */

/* modified on 05-FEB-04 */
/* clears calls which otherwise held by single wire */

// hold the call only input is true or clear 

       a0 &= e0 ;
       a1 &= e1 ;
       a4 &= e4 ;

/* cmnt   transition from Attendent to Auto mode cancels all calls */

if ( pATT & !ATT ) { e0=e1=e4 = a0 =a1= a4 =0;  }
   pATT = ATT ; 

wd();

/* Fireman swithc cancels all landing calls */

/* a five sec delay introduced to halt the lift give lead time for the
drive to stop fully & then start it towards ground floor

*/


if  (FSW ) { e1 = e4 = a1= a4 = 0 ; ATT = 0 ;  };

if (HOME & FSW & !pFManSw ) { e0=a0= HOME = 0 ; tMtrOn=5; SLOW = 0 ;secount = 0 ; }

 pFManSw = FSW ;


/* if homereached after firemean swithc is pressed put controller in Attend mode */

  if (HOME & FSW ) ATT = 1 ;

/* latches car , landing Dn, landing Up calls */
// separate only fresh buttons


       e0 &=  ~a0 ;
       e1 &=  ~a1;
       e4 &=  ~a4 ;


/* Limit down operated */

if (LDN) { FLRNO = 0 ; HOME = 1; HOMEDLY = 0 ;}
        LDNI = LDN ; LUPI = LUP ;
wd();

/* limit up operated : floor set to maximum floor */
//        if (LUP) { FLRNO = MAX_FLR ; HOME = 1;}
// Home removed from LUP to facilitate Fire Man Sw     : 16-01-2005 

        if (LUP) { FLRNO = MAX_FLR ; }

/* check any call pending above */

        bcp();
        bcpbit = BCP1 ;

/* check any call pending below */
        acp() ;
        acpbit = ACP1;
wd();
/* do not latch calls if car is moving within floor */

        if (MTRUP | MTRDN )
        {
        e0 &= ~BFLRNO ;
        wd();
        e1 &= ~BFLRNO ;
        e4 &= ~BFLRNO ;
        }

        if (HOME) {
                a0 |= e0 ;
                a1 |= e1 ;
                a4 |= e4 ;
                a0 &= MASK_e0   ;
                a1 &= MASK_e1   ;
                a4 &= MASK_e4   ;
                }



/* if door opened in moving condition */

/* defines emergency condition */

//        SEMG = (LG & CG &  (DOORLY==0) & !(INS) ) & HOME & !(STOP)
        SEMG = (LG & CG &  (DOORLY==0) & !(INS) ) & HOME  

        & ( (CALLATDLY == 0 ) | ((CALLATDLY > 3) & ! (DZ) ) );

// changed from 5 to 2 


/* counts up */
        if (FLRNO < MAX_FLR)
              FLRNO  += (!PPRXUP & PRXUP & DIRUP ) ;

  wd();
/* counts down */

        if (FLRNO > 0)
               {
                FLRNO  -= (!PPRXDN & PRXDN & DIRDN) ;
//                FLRNO  -= (!PPRXDN & PRXDN & DIRDN) ;
               }
/* defines decade indication of floor */


      BFLRNO = ( 1 << FLRNO) ;

//        if ((PPRXUP & !PRXUP & !NS &ATT ) ||
//           (PPRXDN & !PRXDN & !NS &ATT ) )
//           NS_F = 0 ;



/* register proximity position */

         PPRXUP = PRXUP;
         PPRXDN = PRXDN ;

/* defines run condition */
        RUN = ( DIRDN | DIRUP | MTRDN | MTRUP ) ;
          
/* floor called processing car calls  */
           R20 = ( a0 & BFLRNO ) ;
 
/* for down collecting */
wd();



  if ( ( !DIRUP && !NS_F ) ||
           (NS_F && !bcpbit) ||
          (DIRUP && !acpbit)
       )   

      R21 = ( a1 & BFLRNO ) ; else R21 = 0 ;

 if (DIRDN & !bcpbit) NS_F = 0 ;
 if (DIRUP & !acpbit) NS_F = 0 ;
 if (SLOW) NS_F = 0 ;
 if (!a0) NS_F = 0 ;

  wd();

// if (!NS_F&( !bcpbit | DIRDN ) )
//      R21 = ( a1 & BFLRNO ) ; else R21 = 0 ;

/* for UP collectiog */

  if ( ( !DIRDN && !NS_F ) ||
          (NS_F && !acpbit) ||
          (DIRDN && !bcpbit)
       )   

        R22 = ( a4 & BFLRNO ) ; else R22 = 0 ;

//     if (!NS_F&( !bcpbit | DIRUP ) )
//        R22 = ( a4 & BFLRNO ) ; else R22 = 0 ;

      R23 = !CG & !DZ ;

       FLRCALLED = R20 | R21 | R22 | R23 ;

/* attending the call */
        CALLAT = FLRCALLED ;
        FLRCALLED = 0 ;

/* safety */

     if  (R23) CALLATDLY = 3;

//        if ( CALLAT && CALLATDLY == 0)

/* modified on 19-11-2004  for not clearing the registerd call
   when lift idles in a floor
   it takes ordoubles the door timing */


//      if ( CALLATDLY > 1 & CALLATDLY< 5 &&  REG_CALLS == 0x0000 )
//          CALLATDLY = 4;
// 30-10-2010 : door left open when no calls are pending anywhere
// removed on 30-01-2017
 //       if ( REG_CALLS == 0x0000 && !INS & HOME & !SLOW) CALLATDLY = 2;


          if ( CALLAT )

           {
//                if (!RUN && CALLATDLY < 4 )  CALLATDLY = 2; else CALLATDLY = Call_at_delay ;
// modifies as delay demanded was more by M Kumar 13-10-2004 
            CALLATDLY = Call_at_delay_i ;
            CALLAT = 0 ;
            a0 &= ~BFLRNO ;
            NS_F = 0 ;

            if (DIRDN || !ACP1 || FLRNO == 0 )   a1 &= ~BFLRNO  ;


 //           if( !DIRUP & !DIRDN) { a1 &=~BFLRNO; a4 &= ~BFLRNO ; }

            if( !DIRUP ) { a1 &=~BFLRNO; }

            if( !DIRDN ) { a4 &=~BFLRNO; }
            


/* CG clear landing call if at GND */

            if (!CG && FLRNO == 0 )              a1 &= ~BFLRNO ;

 
/* changed on 14-10-04 */
/*  clearing up calls */

            if (DIRUP || !BCP1 )        a4 &= ~BFLRNO  ;

/* CG clear landing call if at GND */

            if (!CG && FLRNO == MAX_FLR ) a4 &=  ~BFLRNO ;

         }

/* end of flrcalled */


/* clearing calls special case */


    if (CALLATDLY > 0)
     {
       a0  &= ~BFLRNO  ;
       if (DIRDN || !ACP1 || FLRNO == 0 )   a1 &= ~BFLRNO  ;
       if (!CG && FLRNO == 0 )              a1 &= ~BFLRNO ;
       if (DIRUP || !BCP1 )        a4 &= ~BFLRNO  ;
       if (!CG && FLRNO == MAX_FLR ) a4 &=  ~BFLRNO ;
      }

   
   


/* direction down */

     wd();
        R20 = ! (ATT) & BCP1 & !(DIRUP) & !(INS) & (CALLATDLY == 0) ;
        R22 = DIRDN & ! (ATT) & BCP1 ;
        R21 = (INS & INSDN & !(INSUP) ) ;
//        R23 = ATT & BCP1 & ( KDN | DIRDN ) & !DIRUP & LG & CG ;

// for auto door 
        R23 = ATT & BCP1 & (( KDN & !CG ) | ( DIRDN & CG )  ) & !DIRUP ;


//for maual door

//      R23 = ATT & BCP1 & (( KDN & CG ) | ( DIRDN & CG )  ) & !DIRUP ;

        if (DIRDN && CALLATDLY >=3 ) R24 = 1 ; else R24 = 0;


DIRDN = ( R20 | R21 | R22 | R23 | R24 ) & !(LDN) & HOME ;

//CLOSE THE DOOR IN ATT & KUP OR DN PRESSED
/* added mtrup , mtrdn to remove effect of pressing buttons
        if motor is running or recognise only if doors are open */

//if (ATT && (KDN || KUP) && (CALLATDLY > 0) && !DR && !MTRUP && ! MTRDN )
if (ATT && (KDN || KUP) && !DR && !MTRUP && ! MTRDN )


        CALLATDLY = 0;

/* Direction Up */
        wd();
        R20 = !(ATT) & ACP1 & !(DIRDN) & ! (INS) & (CALLATDLY == 0);
        R21 = DIRUP & !(ATT) & ACP1 ;
//        R23 = ATT & ACP1 & ( KUP | DIRUP ) & !DIRDN & CG ; 

// for auto door


          R23 = ATT & ACP1 & ( (KUP & !CG) | (DIRUP & CG  )) & !DIRDN ;

//f or manual door 


//      R23 = ATT & ACP1 & (( KUP & CG ) | ( DIRUP & CG )  ) & !DIRDN ;

// dirdn added mtrdn removed CG added 04-02-2006
        if ( DIRUP && (CALLATDLY >=3)) R24 = 1 ; else R24 = 0 ;
        R22 = 0 ; R25 = 0 ;

        DIRUP = (R20 | R21 | R22 | R23 | R24 | R25 ) & !(LUP) & (HOME) ;

/* inspection */

/* MTRUP process */

        MTRUP = ( DIRUP & SEMG & !MTRDN) |
                ( INS & INSUP & !(INSDN) & !(LUP) & CG & LG   ) ;

//      ( !(ATT) | (ATT & DIRUP & ( KUP | MTRUP) ) ) ;

/* MTRDN process */

MTRDN = ( DIRDN & SEMG & !MTRUP ) |
        ( INS & INSDN & !(INSUP) & !(LDN) & CG & LG ) |
        ( !(LDN) & !(HOME) & CG &LG  & (HOMEDLY !=0 ) & !(INS) );
wd () ;


/* buzzer remoed as spare output not available */

//        BUZZER = !(INS) &
 //                 ( !(HOME) & (HOMEDLY == 0) ) |
//                ( e1 & ATT )   |
// above removed buzzer to be connected to HALL call card common

//        ( (CALLATDLY == 0 ) & !(CG) ) ;

// removed as bound to buzz when lights continuously lit
// if ( ( e1 > 0 && ATT 0 || BUZZER ) BUZZER = 1 ; else BUZZER = 0 ;

        wd();
        CUDL = ( ACP1 & ATT & !CDDL & !DIRDN) ;
        CDDL = ( BCP1 & ATT & !CUDL & !DIRUP) ;


/* stop resets not home reached delay */

if (  !(HOME) & ( HOMEDLY == 0 ) )
                HOMEDLY = 30 ;

// changed to include two proximity switches
/* with one proximity switch */









wd ();

/* Cancels registration if motor is moving within Door Zone */

// if (!RUN && (a1 && BFLRNO)) a1 &= ~BFLRNO ;

/* call being attended & door open pressed and auto mode keep door open till
DO released */

// keep the door open till KDN or KUP pressed

        if (ATT && !MTRUP && !MTRDN && !KUP && ! KDN )  CALLATDLY= 3;

/* Non sotp being registered */
        if (CALLATDLY) NS_F = 0 ;

        NS_F = ( ( ATT & NS ) | NS_F ) ;

        if (!ATT) NS_F = 0 ;

      acpc(); bcpc();

//        NS_F = ( ( ATT & NS ) | NS_F ) &
//                ( (DIRUP & ACP_C) | ( DIRDN & BCP_C ) ) ;


/* do not latch calls if car is moving within floor */
//
          if (MTRUP | MTRDN ) // do not latch call if car is moving within floor
          {
          e0 &= ~BFLRNO ;
          wd();
          e1 &= ~BFLRNO ;
          e4 &= ~BFLRNO ;
          }

//        FLRCALLED = 0 ;

/* modified to include CG & DZ 27.11.03 */

        if ( (CALLATDLY > 0) && DZ && !(MTRUP | MTRDN))
                DOORLY = 1 ; else DOORLY = 0 ;

/* Door Open taken care of */
        R31 = ( DR & DZ & !( MTRDN | MTRUP )  ) ;

        if (R31) CALLATDLY = Call_at_delay;

/* Latch calls only after home reached with masking */
wd();

    /*
        if (HOME) {
                a0 |= e0 ;
                a1 |= e1 ;
                a4 |= e4 ;
                a0 &= MASK_e0   ;
                a1 &= MASK_e1   ;
                a4 &= MASK_e4   ;
                }
      */
/* latch car calls when fireman alarm is pressed 
          if (FireManSw )
          {

           FireManFlag = 1 ;
             a0 = 1;
             a1 = 0;
             a4 = 0;
          }


           if (FireManSw && CG )
          {  a0 = e0 ;
             a0 &= Call_Mask ;
          }

*/
          wd();
            if (ATT && CALLATDLY == 1 ) CALLATDLY = 2 ;
   
//            FLRNO++ ;
// Disply G 4 to 17 only . 
         tFLRNO = FLRNO; 

          if ( tFLRNO > 0 )  tFLRNO = tFLRNO + 3 ;
              a2 = tFLRNO % 10 ; a2 = NOTBL[a2] ;

/* added for more floors */

//        if (FLRNO > 9 ) a2 |=128 ;
        if (FLRNO > 6 ) a2 |=128 ;

        if (!HOME) a2 = 118 ;
        if (LUP & LDN) a2 = 9; // if both limit are operated 


//        FLRNO--;
/* switch off fan if idle for morethan 30 seconds */

        wd();

//        WAIT = (MTRUP | MTRDN | DOORLY );

// modified for fan on delay for only car call


        WAIT = a0;

//        if (WAIT) WAITDLY = FAN_OFF_DELAY ;

        if (WAIT) { WAITDLY = FAN_OFF_DELAY ;} ;

        FAN = WAITDLY ;
//        if (relay) a3 |=128 ; else a3 &= 0x7f ; // blink 8th bit of a3

//        if (CALLATDLY != 0 & relay )  a21 = 0 ;
          if (!CG & relay )  a21 = 0 ;

        if (!relay) a21 = a2 ;

// 30-01-2017 door close function separated 
//  door close to operate to close the door until CG arrives 
//  when car is moving 

     
       if ( DOORLY) tbDC = 1;
       if ( CG ) tbDC = 0 ; 
       
       DC = !DOORLY & (tbDC |RUN | INSUP | INSDN | (HOMEDLY != 0)); 

//  if CG arrives DC to be cleared 
// if DOORLY arrives DC to be set 
// if NOT DOORLY & DC SET = operate DC 
// DORRLY is NOT DC 


// special for inspection
      wd();
        if (INS) {
                a0 = 0 ; a1 = 0 ; a2 = 64 ;
                INSTMR = 10 ; HOME = 0 ; CALLATDLY = 0; DOORLY = 0;
                FAN = 0 ;
                }

        if (!INS & (INSTMR == 0) & !(HOME) ) { HOMEDLY = 30 ; }

/* enable slow speed relay : maintain only when motor is running */
/* BUZZER e3^4 being used as Slow speed relay */

        if ( (CALLATDLY > 0)  && ( MTRDN | MTRUP ) )
        {
        SLOW = 1 ;
        }
        else
        SLOW  = 0 ;
        wd();
//        bPLAY = (SLOW & !pSLOW) ;


        if (SLOW & !pSLOW) {  PlayTmr = 200 ; }
        pSLOW = SLOW ;


        TEM = a3 ;

//        if (DIRUP & relay & !CG  )  a3  |= 1 ; else a3 &= 0xfe ;
// /       if (DIRDN & relay & !CG  )  a3  |= 4 ; else a3 &= 0xfb ;

            if (CDDL) DIRDN = 1 ; 
            if (CUDL) DIRUP = 1 ;


/* blinking for DIRUP & DIRDN indications  : 24-11-2005 */

// blinking arow effect removed


//     if ( CG & !relay ) {  DIRUP = 0 ; DIRDN = 0 ; }

/* delay for Mtrup & MtrDn */

        if ((tMtrOn ==  0 ) && ( (MTRUP & !pMtrUp ) || ( MTRDN & ! pMtrDn ) ))
           {  tMtrOn = 1 ; secount = 0 ; } 

          MTRUP =  ( MTRUP & (tMtrOn == 0 ) ) ;
          MTRDN = (  MTRDN & (tMtrOn == 0 ) ) ;


          if (MTRUP | MTRDN) STK_DELAY = 50;




/*delay for Door Open */

        if ((!MTRUP & pMtrUp ) || ( !MTRDN & pMtrDn ) )
          tMtrOff =  75 ;
          DOORLY  =  (DOORLY  & (tMtrOff == 0 ) ) ;
       
      serialout();
        a3 = TEM ;
        pMtrDn = MTRDN ;
        pMtrUp = MTRUP ;

        sta1 = 0;
/* stop the lift if door is opend other than door zone
  added on 14-10-2004
  */

    if (!DZ && !CG ) CALLATDLY = 5;


   wd();

/* announcer */

//    if ((FLRNO != PFLRNO )  && (CALLATDLY >  0 ) ) { bPLAY = 1 ; bPLAYMUSIC = 1 ; } 

//    if (DZ & bPLAY) { play(FLRNO); TMR=3 ;bPLAY = 0;}
// Door zone play removed replaced with a slow after delay


    if (PlayTmr ==1) { play(FLRNO); PlayTmr = 0 ; }



//    if (bPLAYMUSIC ) { play(20) ; bPLAYMUSIC = 0 ;TMR=3; }


   PFLRNO = FLRNO ;


   }

 } // end of while
#endif

}
/*END of Main */

void intr0(void) interrupt 0 {}
void intr1(void) interrupt 2 {}
void timar1(void ) interrupt 3 {} 

void timer0(void) interrupt 1 using 3
{
        sta1++;
        if (STARTING) {sta1 = 0; STARTING = 0 ; a0=0; a1=0; }

        if (tMtrOff) tMtrOff -- ;

        if (PlayTmr > 1 ) PlayTmr--;

        if (secount++>=150) {    // originally 115 
           sta = 1         ;
           if (tMtrOn ) tMtrOn--   ;

           if (TMR) TMR--;
           if ( CALLATDLY ) CALLATDLY--    ; // door close delay
           if ( HOMEDLY ) HOMEDLY --       ; // HomeDelay
           if ( INSTMR ) INSTMR--          ;
           if ( WAITDLY ) WAITDLY--        ;


           if ( CALLATDLY )

               if ( STK_DELAY ) STK_DELAY --   ;


           wd();



           relay = ~relay  ;
           secount = 0     ; // common delay


        }



        if (CALLATDLY>0 ) callatg0 = 1 ; else callatg0 = 0 ;

        TH0 = 0xe0 ;
        TL0 = 0x00 ;
}
     
   


/*
Fire man switch

    Stop the lift
    Go to Ground Floor
    Latch the call only if lift is moving
    process the car call only
    if any call close the door & latch the call after closing the door
    proceed in the normal way

    */

/*


15-3-2005  Amita : 89C52 G+13 Full collective
                   Old MB with 6 Old + 2 New Boards : total 9
                   

*/

/*



additions


flasihing of number when idle 

Everest   : G + 6 : Full : Auto Door : V3f

24-11-2005 : timer reduced to reduce door close timinigs foreverest : auto :
           : Lamda : V3f
           ; Blinking effect added for display

           : If both limits are operated show a & d segments


02-12-2005 : a delay for MTrdn , Mtrup during on & offcycle is needed
           : let us introduce Tmon & Tmoff
           : let us store Pmtrup  & Pmtrdn status
           : if there is a status change then start the timer
           : output the actual contents only after the specified
               delay is completed



timer increased from 100 to  150 increased dela n oscilation



*/
