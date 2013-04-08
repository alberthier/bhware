//redefinitions de fonctions de "communications"

#include <stdio.h>
#include <stdlib.h>

#include "position.h"
//#ifdef ASSER_POS
//    #include "asserv_position-2.h"
//#endif
    #include "asserv_trajectoire.h"
#include "task_asser.h"
#include "define.h"
#include "pic18.h"

/* variables pont entre le code du robot et le code du simulateur */
unsigned short simu_consigneMoteurG;
unsigned short simu_consigneMoteurD;


unsigned char   ASSER_Running   =   False; 

/** Flag de fragmentation de trajectoire */
unsigned char   ASSER_FragmentTraj  =   False;

//const float     PI              =   3.1415;

extern void SIMU_REDEF_ASSER_GoTo(unsigned char Mouvement, Data_Goto * Data)
{
    /* Cas de changement de trajectoire (evitement) */
    if (ASSER_Running == True)
    {
        ASSER_Running = False;
    }

    switch(Mouvement)
    {
        case ROTATE :

            /* Initialisation du sens de marche AVANT ou ARRIERE, du module POSITION */
            POS_InitialisationSensMarche(MARCHE_AVANT);

            break;

        case MOVE_CURVE :

            /* Initialisation du sens de marche AVANT ou ARRIERE, du module POSITION */
            POS_InitialisationSensMarche(Data->curve.Marche);

            break;

        case MOVE_LINE :

            /* Initialisation du sens de marche AVANT ou ARRIERE, du module POSITION */
            POS_InitialisationSensMarche(Data->line.Marche);

            break;

        case MOVE_ARC :

            /* Initialisation du sens de marche AVANT ou ARRIERE, du module POSITION */
            POS_InitialisationSensMarche(Data->arc.Marche);

            break;
    }

    ASSER_TRAJ_InitialisationTrajectoire(POS_GetPoseAsserRobot(), Mouvement, Data);

    /* Declenchement de l'asservissement */
    ASSER_Running = True;
}

extern void SIMU_REDEF_ASSER_SendConsigne(unsigned short ConsigneMoteurD, unsigned short ConsigneMoteurG)
{
    simu_consigneMoteurG = ConsigneMoteurG;
    simu_consigneMoteurD = ConsigneMoteurD;
}

float SIMU_RegulateurPI_BHT(char side, float erreurVitesse, float* integ, float Ki, float Kp, float vmax, float Te)
{
    signed int tensionPWM;
    float tensionPWMTemp;
    float I;
    float P;
    unsigned char flag_saturation = False;

    *integ = *integ + Te * erreurVitesse;
    I = Ki * (*integ);
    if (I > vmax)
    {
        *integ = vmax / Ki;
        I = vmax;
    }
    if (I < -vmax)
    {
        *integ = -vmax / Ki;
        I = -vmax;
    }

    P = Kp * erreurVitesse;
    tensionPWMTemp = P + I;

    if (tensionPWMTemp < -vmax)
    {
        tensionPWMTemp = -vmax;
        flag_saturation = True;
    }
    if (tensionPWMTemp > vmax)
    {
        tensionPWMTemp = vmax;
        flag_saturation = True;
    }

    if (flag_saturation == True)
    {
        /* Anti windup simplifie */
        *integ = *integ - Te * erreurVitesse;
        /* Flag de saturation*/
        if (side == GAUCHE)
        {
            SaturationPIGflag = True;
        }
        if (side == DROIT)
        {
            SaturationPIDflag = True;
        }
    }
    else
    {
        if (side == GAUCHE)
        {
            SaturationPIGflag = False;
        }
        if (side == DROIT)
        {
            SaturationPIDflag = False;
        }
    }
    /* NOLOG */
    //ASSER_TRAJ_LogAsserValPC("SaturationPIGflag", SaturationPIGflag);
    //ASSER_TRAJ_LogAsserValPC("SaturationPIDflag", SaturationPIDflag);

    //tensionPWM = (signed int)tensionPWMTemp;
    //return(tensionPWM);
    return(tensionPWMTemp);
}
