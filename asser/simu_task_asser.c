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

/* variables pont entre le code du robot et le code du simulateur */
unsigned short simu_consigneMoteurG;
unsigned short simu_consigneMoteurD;


unsigned char   ASSER_Running   =   False; 

/** Flag de fragmentation de trajectoire */
unsigned char   ASSER_FragmentTraj  =   False;

//const float     PI              =   3.1415;

//extern void SIMU_REDEF_ASSER_GoTo(Vecteur *pointInter, unsigned int nbrPtsInter, Pose poseArrivee, unsigned int Mouvement, signed int Marche)
extern void SIMU_REDEF_ASSER_GoTo(PtTraj *p_chemin, unsigned int nbrePtsChemin, unsigned int Mouvement, signed int Marche)
{
//    #ifdef ASSER_POS
//        Vecteur arrivee;
//    #endif

    /* Initialisation du module Position:couche d'abstraction des capteurs et actionneurs de positionnement */
    if (ASSER_Running == False)
    {
        POS_InitialisationSensMarche(Marche);
    }

//    /* Initialisation du module d'asservissement */
//    #ifdef ASSER_POS
//        arrivee.x = p_chemin[0].x;
//        arrivee.y = p_chemin[0].y;
//        ASSER_POS_Initialisation(arrivee, Mouvement);
//    #endif

    //ASSER_TRAJ_InitialisationTrajectoire(POS_GetPoseAsserRobot(), pointInter, nbrPtsInter, poseArrivee, Mouvement);
    ASSER_TRAJ_InitialisationTrajectoire(POS_GetPoseAsserRobot(), p_chemin, nbrePtsChemin, Mouvement);

    /* Declenchement de l'asservissement */
    ASSER_Running = True;
}

extern void SIMU_REDEF_ASSER_SendConsigne(unsigned short ConsigneMoteurD, unsigned short ConsigneMoteurG)
{
    simu_consigneMoteurG = ConsigneMoteurG;
    simu_consigneMoteurD = ConsigneMoteurD;
}

float SIMU_RegulateurPI_BHT(float erreurVitesse, float* integ, float Ki, float Kp, float vmax, float Te)
{
    signed int tensionPWM;
    float tensionPWMTemp;
    float I;
    float P;

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
    }
    if (tensionPWMTemp > vmax)
    {
        tensionPWMTemp = vmax;
    }
    //tensionPWM = (signed int)tensionPWMTemp;
    //return(tensionPWM);
    return(tensionPWMTemp);
}