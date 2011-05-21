#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "position.h"
#include "define.h"
#include "simuAsser.h"

//#ifdef ASSER_POS
//    #include "asserv_position-2.h"
//#endif
    #include "asserv_trajectoire.h"
#include "task_asser.h"

#ifndef TEMPS_SIMULATION
#define TEMPS_SIMULATION 15 /* secondes */
#endif

#define     SIMU_TM     0.3
#define     SIMU_KM     ((float)0.000985) // ((float)0.000997)
#define     SIMU_CR     0
#define     SIMU_PERIMETRE_GAUCHE   (PI * 0.0400)  /* longueur en mètre */
#define     SIMU_PERIMETRE_DROIT    (PI * 0.0400)  /* longueur en mètre 0.040*/
#define     TE_PI       0.001 //0.02 fait de grosses oscillations

/* coefficients des régulateurs de vitesse */
float   KP_GAUCHE   = 5.34;
float   KI_GAUCHE   = 17.8;
float   KP_DROIT    = 5.34;
float   KI_DROIT    = 17.8;
//float   KP_GAUCHE   = 6000.0;
//float   KI_GAUCHE   = 20000;
//float   KP_DROIT    = 6000;
//float   KI_DROIT    = 20000;

/* mesure deplacement de chaque roue en pas codeur */
signed int deltaPasCodeurG;
signed int deltaPasCodeurD;

/* paramètres des régulateurs de vitesse */
float integPI_G = 0.0;
float integPI_D = 0.0;

/* initialisation du robot à une vitesse nulle */
VitessesRobotUnicycle vitessesConsignes;
float vitesseMoteurG = 0.0, vitesseMoteurG_n2 = 0.0;
float vitesseMoteurD = 0.0, vitesseMoteurD_n2 = 0.0;

unsigned short g_ConsigneMoteurG;
unsigned short g_ConsigneMoteurD;

float erreurVitesseMoteurG = 0.0;
float erreurVitesseMoteurD = 0.0;

//float tensionPWM_G = 0.0, tensionPWM_G_n[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//float tensionPWM_D = 0.0, tensionPWM_D_n[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float tensionPWM_G = 0.0, tensionPWM_G_n[300];
float tensionPWM_D = 0.0, tensionPWM_D_n[300];

int compteurPeriodePI = 0;

char asserRunning_ant = False;

float L = 0.339;
Pose poseRobot;

extern void SIMU_SetGainsPI(float kp, float ki)
{
    KP_GAUCHE = kp / SIMU_KM;
    KP_DROIT = kp / SIMU_KM;
    KI_GAUCHE = ki / SIMU_KM;
    KI_DROIT = ki / SIMU_KM;
}

/*
Convertit la vitesse (en Udist_ent/s) des roues en nombre de pas codeur par période TE (en s)
*/
void SIMU_REDEF_ASSER_RecoverNbrPas(float vitesseRoueGauche, float vitesseRoueDroite, signed int* deltaPasGauche, signed int* deltaPasDroite)
{
    float deltaMesGauche= 0;
    float deltaMesDroite= 0;

    deltaMesGauche = vitesseRoueGauche * TE;
    deltaMesDroite = vitesseRoueDroite * TE;

    *deltaPasGauche = (signed int)((deltaMesGauche * (float)NBRE_PAS) / (float)SIMU_PERIMETRE_GAUCHE);
    *deltaPasDroite = (signed int)((deltaMesDroite * (float)NBRE_PAS) / (float)SIMU_PERIMETRE_DROIT);
}

float SIMU_ErreurVitesseConsPWMVToVitMS(unsigned int consPMW, float vitMS)
{ 
    float erreur;

    erreur = ( ((float)consPMW - (float)OffsetPWM) * (float)SIMU_KM) - vitMS;
    return(erreur);
}

/* en entrée, valeur du registre PWM, et en sortie la vitesse en m/s du moteur */
void SIMU_SimulationMoteurCC(signed int tensionPWM, float* vitesseMoteur, float constanteTemps, float gainStatique, float coupleResistant)
{
    float cste_exp, tensionPWM_eff;

    /* tension effective par application d'un couple resistant */
    if (tensionPWM < 0.0)
    {
        if ((tensionPWM + coupleResistant) > 0.0)
        {
            tensionPWM_eff = 0.0;
        }
        else
        {
            tensionPWM_eff = tensionPWM + coupleResistant;
        }
    }
    else
    {
        if ((tensionPWM - coupleResistant) < 0.0)
        {
            tensionPWM_eff = 0.0;
        }
        else
        {
            tensionPWM_eff = tensionPWM - coupleResistant;
        }
    }

    /* simulation au premier ordre du moteur CC */
    cste_exp = (float)exp(-TE_PI / constanteTemps);
    *vitesseMoteur = gainStatique * (1-cste_exp) * (float)tensionPWM_eff + cste_exp * (*vitesseMoteur);
}

void SIMU_SimulationMoteurCC_ordre2(signed int tensionPWM, float *tensionPWM_n, float* vitesseMoteur, float *vitesseMoteur_n2, float constanteTemps1, float constanteTemps2, float gainStatique, unsigned int nbPeriodRetard, float coupleResistant, float periode)
{
    float cste_exp1, cste_exp2, cste_exp12, tensionPWM_eff;
    unsigned int i;
    unsigned int nbPeriodRetardEff;
    float vitesseMoteurTemp;

    //limitation du retard
    if (nbPeriodRetard > 298)
    {
        nbPeriodRetardEff = 298;
    }
    else
    {
        nbPeriodRetardEff = nbPeriodRetard;
    }

    /* tension effective par application d'un couple resistant */
    if (tensionPWM < 0.0)
    {
        if ((tensionPWM + coupleResistant) > 0.0)
        {
            tensionPWM_eff = 0.0;
        }
        else
        {
            tensionPWM_eff = tensionPWM + coupleResistant;
        }
    }
    else
    {
        if ((tensionPWM - coupleResistant) < 0.0)
        {
            tensionPWM_eff = 0.0;
        }
        else
        {
            tensionPWM_eff = tensionPWM - coupleResistant;
        }
    }

    //memorisation des entrees anterieures
    if (nbPeriodRetardEff > 0)
    {
        for(i = 0; i < nbPeriodRetardEff; i++)
        {
            tensionPWM_n[nbPeriodRetardEff + 1 - i] = tensionPWM_n[nbPeriodRetardEff - i];
        }
    }
    tensionPWM_n[1] = tensionPWM_n[0];
    tensionPWM_n[0] = tensionPWM;

    /* simulation au second ordre du moteur CC */
    cste_exp1 = (float)exp(-periode / constanteTemps1);
    cste_exp2 = (float)exp(-periode / constanteTemps2);
    /*cste_exp12 = (float)exp(-(TE*(constanteTemps1 + constanteTemps2)) / (constanteTemps1 * constanteTemps2));*/
    vitesseMoteurTemp = *vitesseMoteur;
    *vitesseMoteur = (cste_exp1 + cste_exp2) * (*vitesseMoteur) - (cste_exp1 * cste_exp2) * (*vitesseMoteur_n2);
    *vitesseMoteur = *vitesseMoteur + gainStatique * (1 - cste_exp1) * (1 - cste_exp2) * tensionPWM_n[nbPeriodRetardEff + 1];

    //memorisation des sorties anterieures
    *vitesseMoteur_n2 = vitesseMoteurTemp;
}

void SIMU_InitialisationLogRobot(void)
{
    poseRobot = POS_GetPoseRobot();

    ASSER_TRAJ_LogAsser("vitesseMoteurGauche", NBR_ASSER_LOG_VALUE + 1, vitesseMoteurG);
    ASSER_TRAJ_LogAsser("vitesseMoteurDroit", NBR_ASSER_LOG_VALUE + 1, vitesseMoteurD);
    ASSER_TRAJ_LogAsser("erreurVitesseMoteurGauche", NBR_ASSER_LOG_VALUE + 1, erreurVitesseMoteurG);
    ASSER_TRAJ_LogAsser("erreurVitesseMoteurDroit", NBR_ASSER_LOG_VALUE + 1, erreurVitesseMoteurD);
    ASSER_TRAJ_LogAsser("tensionPWM_MoteurGauche", NBR_ASSER_LOG_VALUE + 1, tensionPWM_G);
    ASSER_TRAJ_LogAsser("tensionPWM_MoteurDroit", NBR_ASSER_LOG_VALUE + 1, tensionPWM_D);
//    ASSER_TRAJ_LogAsser("integPIVG", NBR_ASSER_LOG_VALUE + 1, integPI_G*KI_GAUCHE);
//    ASSER_TRAJ_LogAsser("integPIVD", NBR_ASSER_LOG_VALUE + 1, integPI_D*KI_DROIT);

    ASSER_TRAJ_LogAsser("ConsigneMoteurGauche", NBR_ASSER_LOG_VALUE + 1, 0.0);
    ASSER_TRAJ_LogAsser("ConsigneMoteurDroit", NBR_ASSER_LOG_VALUE + 1, 0.0);
    ASSER_TRAJ_LogAsser("vitLongitudinale", NBR_ASSER_LOG_VALUE + 1, 0.0);

    /* fichier de configuration pour l'affichage sous matlab */
    ASSER_TRAJ_LogAsser("periode", NBR_ASSER_LOG_VALUE + 1, TE);
    //ASSER_TRAJ_LogAsser("periode", NBR_ASSER_LOG_VALUE + 1, TE_PI);
    ASSER_TRAJ_LogAsser("Ki", NBR_ASSER_LOG_VALUE + 1, KI_GAUCHE*SIMU_KM);

//    /* debug boucle de vitesse 1ms */
//    ASSER_TRAJ_LogAsser("vitesseG1ms", NBR_ASSER_LOG_VALUE + 1, vitesseMoteurG);
//    ASSER_TRAJ_LogAsser("vitesseD1ms", NBR_ASSER_LOG_VALUE + 1, vitesseMoteurD);
}

void SIMU_LogRobot(void)
{
    ASSER_TRAJ_LogAsser("vitesseMoteurGauche", NBR_ASSER_LOG_VALUE, vitesseMoteurG);
    ASSER_TRAJ_LogAsser("vitesseMoteurDroit", NBR_ASSER_LOG_VALUE, vitesseMoteurD);

    ASSER_TRAJ_LogAsser("erreurVitesseMoteurGauche", NBR_ASSER_LOG_VALUE + 1, erreurVitesseMoteurG);
    ASSER_TRAJ_LogAsser("erreurVitesseMoteurDroit", NBR_ASSER_LOG_VALUE + 1, erreurVitesseMoteurD);

    ASSER_TRAJ_LogAsser("tensionPWM_MoteurGauche", NBR_ASSER_LOG_VALUE, tensionPWM_G);
    ASSER_TRAJ_LogAsser("tensionPWM_MoteurDroit", NBR_ASSER_LOG_VALUE, tensionPWM_D);
/*
    ASSER_TRAJ_LogAsser("integPIVG", NBR_ASSER_LOG_VALUE + 1, integPI_G*KI_GAUCHE);
    ASSER_TRAJ_LogAsser("integPIVD", NBR_ASSER_LOG_VALUE + 1, integPI_D*KI_DROIT);
*/
    ASSER_TRAJ_LogAsser("ConsigneMoteurGauche", NBR_ASSER_LOG_VALUE, (float)g_ConsigneMoteurG);
    ASSER_TRAJ_LogAsser("ConsigneMoteurDroit", NBR_ASSER_LOG_VALUE, (float)g_ConsigneMoteurD);
    ASSER_TRAJ_LogAsser("vitLongitudinale", NBR_ASSER_LOG_VALUE, vitessesConsignes.longitudinale);
//    ASSER_TRAJ_LogAsser("vitRotation", NBR_ASSER_LOG_VALUE + 1, vitessesConsignes.rotation);

}

/**********************************************************************/
/*! \brief ASSER_TRAJ_AfficheInfoFinAsser
 *
 *  \note   Affiche les info de fin d'asser
 *
 *  \return None
 */
/**********************************************************************/
extern void ASSER_TRAJ_AfficheInfoFinAsser(void)
{
    Pose poseRobot;

    poseRobot = POS_GetPoseRobot();
//    ASSER_TRAJ_LogAsser("time", ASSER_TRAJ_GetCompteur() * TE);
    printf("Motion accomplished. Pose x:%1.3fm y:%1.3fm theta:%1.3f time:%1.2fs.\n", poseRobot.x, poseRobot.y, ((poseRobot.angle * 180.0) / PI), (ASSER_TRAJ_GetCompteur() * TE));
}

/**********************************************************************/
/*! \brief SIMU_BoucleVitesse
 *
 *  \note   fonction periodique des asservissements de vitesse PI.
 *
 *  \return None
 */
/**********************************************************************/
void SIMU_BoucleVitesse(void)
{
    erreurVitesseMoteurG = SIMU_ErreurVitesseConsPWMVToVitMS(simu_consigneMoteurG, vitesseMoteurG);
    erreurVitesseMoteurD = SIMU_ErreurVitesseConsPWMVToVitMS(simu_consigneMoteurD, vitesseMoteurD);

    tensionPWM_G = (signed int)SIMU_RegulateurPI_BHT(erreurVitesseMoteurG, &integPI_G, KI_GAUCHE, KP_GAUCHE, OffsetPWM, TE_PI);

//    /* Mesure des instants de saturation du correcteur PI gauche */
//    if ((tensionPWM_G < -(OffsetPWM-0.1)) || (tensionPWM_G > (OffsetPWM-0.1)))
//    {
//        ASSER_TRAJ_LogAsser("saturationPIgauche", 1.0);
//    }
//    else
//    {
//        ASSER_TRAJ_LogAsser("saturationPIgauche", 0.0);
//    }
    tensionPWM_D = (signed int)SIMU_RegulateurPI_BHT(erreurVitesseMoteurD, &integPI_D, KI_DROIT, KP_DROIT, OffsetPWM, TE_PI);

    /* Mesure des instants de saturation du correcteur PI droit */
//    if ((tensionPWM_D < -(OffsetPWM-0.1)) || (tensionPWM_D > (OffsetPWM-0.1)))
//    {
//        ASSER_TRAJ_LogAsser("saturationPIdroit", 1.0);
//    }
//    else
//    {
//        ASSER_TRAJ_LogAsser("saturationPIdroit", 0.0);
//    }

    if (asserRunning_ant != ASSER_Running)
    {
        integPI_G = 0.0;
        integPI_D = 0.0;
        asserRunning_ant = ASSER_Running;
    }

    /* Simulation des deux moteurs de deplacement */

    // 1:gain statique, 2:premiere constante de temps, 3: deuxieme constante de temps
    // moteur gauche [0.90848238070313447, 0.001532263578663721, 0.14712688461925522]
    //SIMU_SimulationMoteurCC((signed int)tensionPWM_G, &vitesseMoteurG, SIMU_TM, SIMU_KM, SIMU_CR);
    SIMU_SimulationMoteurCC_ordre2((signed int)tensionPWM_G
                                   , tensionPWM_G_n
                                   , &vitesseMoteurG
                                   , &vitesseMoteurG_n2
                                   , 0.0178 //0.072
                                   , 0.3449 //0.431
                                   , SIMU_KM
                                   , 0 //20
                                   , SIMU_CR
                                   , TE_PI
                                   );

    // 1:gain statique, 2:premiere constante de temps, 3: deuxieme constante de temps
    // moteur droit [0.93041457097724989, 0.0018257971355764644, 0.20174006794915703] avec un retard pur de 0.02s
    //SIMU_SimulationMoteurCC((signed int)tensionPWM_D, &vitesseMoteurD, SIMU_TM, SIMU_KM, SIMU_CR);
    SIMU_SimulationMoteurCC_ordre2((signed int)tensionPWM_D
                                   , tensionPWM_D_n
                                   , &vitesseMoteurD
                                   , &vitesseMoteurD_n2
                                   , 0.0178 //0.072
                                   , 0.3449 //0.431
                                   , SIMU_KM
                                   , 0 //20
                                   , SIMU_CR
                                   , TE_PI
                                   );

//    if (ASSER_TRAJ_GetCompteur() == 1)
//    {
//        ASSER_TRAJ_LogAsser("vitesseG1ms", (float)tensionPWM_G);
//        ASSER_TRAJ_LogAsser("vitesseD1ms", (float)tensionPWM_D);
//    }

}

void SIMU_CalculPeriodique(void)
{
    int p;
        ASSER_TRAJ_LogAsser("ASSER_Running", NBR_ASSER_LOG_VALUE, ASSER_Running);
        asserRunning_ant = ASSER_Running;
        SIMU_REDEF_ASSER_RecoverNbrPas(vitesseMoteurG, vitesseMoteurD, &deltaPasCodeurG, &deltaPasCodeurD);
        POS_Positionnement(deltaPasCodeurD, deltaPasCodeurG);

        /*************************************************************************************************/
        /****** Bloc d'instructions reellement present dans la compilation du code du robot, teste par le simulateur **********************************************/
//        #ifdef ASSER_POS
//            ASSER_POS_AsservissementMouvementRobot(POS_GetPoseAsserRobot(), &vitessesConsignes);   /* ASSER_Running est modifié par cette fonction */
//        #endif
        /* Application de l'asservissement de trajectoire */
        ASSER_TRAJ_AsservissementMouvementRobot(POS_GetPoseAsserRobot(), &vitessesConsignes);

        //if (ASSER_Running == True)
        //{
            POS_ConversionVitessesLongRotToConsignesPWMRouesRobotUnicycle(vitessesConsignes.longitudinale, vitessesConsignes.rotation, &g_ConsigneMoteurG, &g_ConsigneMoteurD);
            /***********************************/
            /* Envois des consignes des boucles de vitesse au PIC asser */
            SIMU_REDEF_ASSER_SendConsigne(g_ConsigneMoteurD, g_ConsigneMoteurG);
            /*************************************************************************************************/
            /*************************************************************************************************/

            /* Boucle de vitesse, avec une periode de 1ms sachant que la periode de l'asser trajectoire est de 20x 1ms */
            for (p = 0; p < 20; p++)
            {
                SIMU_BoucleVitesse();
            }
        //}

        poseRobot = POS_GetPoseRobot(); // lecture de la position du robot dans le module POSITION
}

void SIMU_CalculPeriodiqueAsserVitessePI(void)
{
    //determination de la vitesse de consigne
    compteurPeriodePI = compteurPeriodePI + 1;
    if (compteurPeriodePI < floor(tempsAcc / TE_PI))
    {
        g_ConsigneMoteurG = (unsigned int)((float)Umax * ((float)compteurPeriodePI / (float)floor(tempsAcc / TE_PI))) + OffsetPWM;
    }
    if (compteurPeriodePI > (floor(5.0/TE_PI) - floor(tempsAcc / TE_PI)))
    {
        g_ConsigneMoteurG = (unsigned int)((float)Umax * ( (float)(floor(tempsAcc / TE_PI) - (compteurPeriodePI - (floor(5.0/TE_PI) - floor(tempsAcc / TE_PI)))) / (float)floor(tempsAcc / TE_PI))) + OffsetPWM;
    }
    SIMU_REDEF_ASSER_SendConsigne(g_ConsigneMoteurD, g_ConsigneMoteurG);

    erreurVitesseMoteurG = SIMU_ErreurVitesseConsPWMVToVitMS(simu_consigneMoteurG, vitesseMoteurG);

    //tensionPWM_G = SIMU_RegulateurPI_BHT(erreurVitesseMoteurG, &integPI_G, KI_GAUCHE, KP_GAUCHE, OffsetPWM * SIMU_KM, 0.001) / (float)SIMU_KM;
    tensionPWM_G = (float)floor(SIMU_RegulateurPI_BHT(erreurVitesseMoteurG, &integPI_G, KI_GAUCHE, KP_GAUCHE, OffsetPWM, TE_PI));
    if ((tensionPWM_G < -(OffsetPWM-0.1)) || (tensionPWM_G > (OffsetPWM-0.1)))
    {
        ASSER_TRAJ_LogAsser("saturationPIgauche", NBR_ASSER_LOG_VALUE + 1, 1.0);
    }
    else
    {
        ASSER_TRAJ_LogAsser("saturationPIgauche", NBR_ASSER_LOG_VALUE + 1, 0.0);
    }

    //SIMU_SimulationMoteurCC((signed int)tensionPWM_G, &vitesseMoteurG, SIMU_TM, SIMU_KM, SIMU_CR);

    // 1:gain statique, 2:premiere constante de temps, 3: deuxieme constante de temps
    // moteur gauche [0.90848238070313447, 0.001532263578663721, 0.14712688461925522]
    //SIMU_SimulationMoteurCC((signed int)tensionPWM_G, &vitesseMoteurG, SIMU_TM, SIMU_KM, SIMU_CR);
    SIMU_SimulationMoteurCC_ordre2((signed int)tensionPWM_G
                                   , tensionPWM_G_n
                                   , &vitesseMoteurG
                                   , &vitesseMoteurG_n2
                                   , 0.001532263578663721 //0.05
                                   , 0.14712688461925522 //0.177
                                   , SIMU_KM
                                   , 10 //20
                                   , SIMU_CR
                                   , TE_PI
                                   );

//    // 1:gain statique, 2:premiere constante de temps, 3: deuxieme constante de temps
//    // moteur droit [0.92917170491405932, 0.0017267357695696336, 0.2008518604808292] avec un retard pur de 0.02s
//    SIMU_SimulationMoteurCC_ordre2((signed int)tensionPWM_G
//                                   , tensionPWM_G_n
//                                   , &vitesseMoteurG
//                                   , &vitesseMoteurG_n2
//                                   , 0.0017267357695696336
//                                   , 0.2008518604808292
//                                   , SIMU_KM
//                                   , 20
//                                   , SIMU_CR
//                                   , TE_PI
//                                   );
}

int SIMU_AsserVitessePI(void)
{
    int p;

    compteurPeriodePI = 0;
    while (compteurPeriodePI < floor((3.0*tempsAcc)/TE)) /* simulation pendant 5 secondes */
    {
        //SIMU_CalculPeriodiqueAsserVitessePI();

        //determination de la vitesse de consigne
        compteurPeriodePI = compteurPeriodePI + 1;
        if (compteurPeriodePI < floor(tempsAcc / TE))
        {
            g_ConsigneMoteurG = (unsigned int)((float)Umax * ((float)compteurPeriodePI / (float)floor(tempsAcc / TE))) + OffsetPWM;
        }
      /*  if (compteurPeriodePI > (floor(5.0/TE) - floor(tempsAcc / TE)))
        {
            g_ConsigneMoteurG = (unsigned int)((float)Umax * ( (float)(floor(tempsAcc / TE) - (compteurPeriodePI - (floor(5.0/TE) - floor(tempsAcc / TE)))) / (float)floor(tempsAcc / TE))) + OffsetPWM;
        }*/
        SIMU_REDEF_ASSER_SendConsigne(g_ConsigneMoteurD, g_ConsigneMoteurG);
        /* Boucle de vitesse, avec une periode de 1ms sachant que la periode de l'asser trajectoire est de 20x 1ms */
        for (p = 0; p < 20; p++)
        {
            SIMU_BoucleVitesse();
        }

        SIMU_LogRobot();
    }
}

int SIMU_Mouvement(void)
{
    while( ((ASSER_Running == True) || (vitesseMoteurG > 10.1) || (vitesseMoteurD > 10.1)) && (ASSER_TRAJ_GetCompteur() < (unsigned int)(TEMPS_SIMULATION / TE)))
    {
        SIMU_CalculPeriodique();
        SIMU_LogRobot();
    }
    /* FIN 1er DEPLACEMENT */
    if (ASSER_Running == True)
    {
//        ASSER_TRAJ_LogAsser("time", -1.0);
        printf("Motion failed: timeout.\n");
        ASSER_Running = False;
        return 0;
    }
    else
    {
        ASSER_TRAJ_AfficheInfoFinAsser();
        return 1;
    }
}
