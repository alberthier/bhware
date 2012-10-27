#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "position.h"
#include "define.h"
#include "simuAsser.h"

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

/* parametres moteurs */
float   MASSE       = 1.2;
float   RAYON_ROUE  = 0.05;
float   FROTTEMENT_FLUIDE   = 0.0000504; //=0.000252*0.2;
float   FORCE_RESISTANTE    = 0.4;
float   RESISTANCE_INDUIT   = 2.18;
float   INDUCTANCE_INDUIT   = 0.00024;
float   CONSTANTE_COUPLE    = 0.0234;
float   CONSTANTE_VITESSE   = 0.02346;
float   RAPPORT_REDUCTION   = 20.0;
unsigned int NB_PERIODE_RETARD = 0u;

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

float tempsAcc  =   1.0;      /* Acceleration du deplacement sur la trajectoire,
                                 -> temps en secondes pour passer de 0 a la vitesse max retournee par POS_GetConsVitesseMax() */
float L = 0.339;
Pose poseRobot;

extern void SIMU_SetGainsPI(float kp, float ki)
{
    float KM;
    KM = SIMU_gain();
    KP_GAUCHE = kp / KM;
    KP_DROIT = kp / KM;
    KI_GAUCHE = ki / KM;
    KI_DROIT = ki / KM;
}

extern void SIMU_SetParamMoteur(float m, float R, float f, float Fr, float r, float L, float kc, float kv, float Rred)
{
    MASSE       = m;
    RAYON_ROUE  = R;
    FROTTEMENT_FLUIDE   = f;
    FORCE_RESISTANTE    = Fr;
    RESISTANCE_INDUIT   = r;
    INDUCTANCE_INDUIT   = L;
    CONSTANTE_COUPLE    = kc;
    CONSTANTE_VITESSE   = kv;
    RAPPORT_REDUCTION   = Rred;
}

extern void SIMU_SetParamProfilVitesse(float Amax, float Dmax)
{
    A_MAX = Amax;
    D_MAX = Dmax;   
}

extern void SIMU_SetConfigGeneraleProfilVitesse(float ratioAcc, float ratioDecc)
{
    Ratio_Acc = ratioAcc;
    Ratio_Decc = ratioDecc;
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

float SIMU_ErreurVitesseConsPWMVToVitMS(unsigned int consPMW, float vitMS, float K)
{ 
    float erreur;

    erreur = ( ((float)consPMW - (float)OffsetPWM) * K) - vitMS;
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

float SIMU_gain(void)
{
    float R, f, r, kc, kv, Rred;
    float K;

    R = RAYON_ROUE;
    f = FROTTEMENT_FLUIDE;
    r = RESISTANCE_INDUIT;
    kc = CONSTANTE_COUPLE;
    kv = CONSTANTE_VITESSE;
    Rred = RAPPORT_REDUCTION;

    K = (R/Rred) * (kc / (kc*kv + f*r));
    return((K*12.0)/1023.0);
}

void SIMU_SimulationMoteurCC_ordre2_complet(signed int tensionPWM,
                                            float *tensionPWM_n,
                                            float *vitesseMoteur,
                                            float *vitesseMoteur_n2,
                                            float masse,
                                            float rayon_roue,
                                            float frottement_fluide,
                                            float force_resistante,
                                            float resistance_induit,
                                            float inductance_induit,
                                            float constante_couple,
                                            float constante_vitesse,
                                            float rapport_reduction,
                                            unsigned int nbPeriodRetard,
                                            float periode)
{
    //float tensionPWM_eff;
    unsigned int i;
    unsigned int nbPeriodRetardEff;
    float m, R, f, Fr, r, L, kc, kv, Rred;

    //limitation du retard à la taille du tableau memorisant les tensions anterieures
    if (nbPeriodRetard > 298)
    {
        nbPeriodRetardEff = 298;
    }
    else
    {
        nbPeriodRetardEff = nbPeriodRetard;
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


    float K, S1, S2, T1, T2, alpha1, alpha2, C1, C2, N0, N1;
    float vTemp;
    float square_R;

    m = masse;
    R = rayon_roue;
    f = frottement_fluide;
    Fr = force_resistante;
    r = resistance_induit;
    L = inductance_induit;
    kc = constante_couple;
    kv = constante_vitesse;
    Rred = rapport_reduction;

    //K = (R/Rred) * (kc / (kc*kv + f*r));
    K = SIMU_gain();
    square_R = SQUARE(R);
    S1 = (square_R*L*m) / (Rred *(f*r + kc*kv));
    S2 = ( ( (square_R*r*m) / Rred) + f*L ) / (f*r + kc*kv);
    T1 = 0.5*(S2 + sqrt(SQUARE(S2) -4*S1));
    T2 = S2 - T1;
    alpha1 = exp(-periode/T1);
    alpha2 = exp(-periode/T2);
    C1 = T1 / (T2-T1);
    C2 = T2 / (T1-T2);
    N0 = alpha1*alpha2 + C1*alpha2 + C2*alpha1;
    N1 = -(alpha1 + alpha2 + C1 + C2 + C1*alpha2 + C2*alpha1);

    vTemp = (alpha1 + alpha2) * (*vitesseMoteur) - alpha1 * alpha2 * (*vitesseMoteur_n2);
    if ( fabs(tensionPWM_n[nbPeriodRetardEff]) < fabs(((R*r)/kc)* Fr) )
    {
        tensionPWM_n[nbPeriodRetardEff] = 0;
    }
    else
    {
        tensionPWM_n[nbPeriodRetardEff] = tensionPWM_n[nbPeriodRetardEff] - ((R*r)/kc)* Fr;
    }
    vTemp = vTemp + K * ( N1 * tensionPWM_n[nbPeriodRetardEff] + N0 * tensionPWM_n[nbPeriodRetardEff + 1u]);

    *vitesseMoteur_n2 = *vitesseMoteur;
    *vitesseMoteur = vTemp;
}

void SIMU_InitialisationLogRobot(void)
{
    //POS_SetGainStatiqueMoteur(SIMU_gain(), SIMU_gain());
    poseRobot = POS_GetPoseRobot();

    ASSER_TRAJ_LogAsserValPC("dist_parcourue",  0.0);
    ASSER_TRAJ_LogAsserValPC("VitesseReelleMesure", 0.0);

    ASSER_TRAJ_LogAsserValPC("vitesseMoteurGauche", vitesseMoteurG);
    ASSER_TRAJ_LogAsserValPC("vitesseMoteurDroit", vitesseMoteurD);
    ASSER_TRAJ_LogAsserValPC("erreurVitesseMoteurGauche", erreurVitesseMoteurG);
    ASSER_TRAJ_LogAsserValPC("erreurVitesseMoteurDroit", erreurVitesseMoteurD);
    ASSER_TRAJ_LogAsserValPC("tensionPWM_MoteurGauche", tensionPWM_G);
    ASSER_TRAJ_LogAsserValPC("tensionPWM_MoteurDroit", tensionPWM_D);
//    ASSER_TRAJ_LogAsserValPC("integPIVG", integPI_G*KI_GAUCHE);
//    ASSER_TRAJ_LogAsserValPC("integPIVD", integPI_D*KI_DROIT);

    ASSER_TRAJ_LogAsserValPC("ConsigneMoteurGauche", 0.0);
    ASSER_TRAJ_LogAsserValPC("ConsigneMoteurDroit", 0.0);
    ASSER_TRAJ_LogAsserValPC("vitLongitudinale", 0.0);
    ASSER_TRAJ_LogAsserValPC("vitLongitudinaleEffective", 0.0);

    /* fichier de configuration pour l'affichage sous matlab */
    ASSER_TRAJ_LogAsserValPC("periode", TE);
    //ASSER_TRAJ_LogAsserValPC("periode", TE_PI);
    ASSER_TRAJ_LogAsserValPC("Ki", KI_GAUCHE*SIMU_gain());

//    /* debug boucle de vitesse 1ms */
//    ASSER_TRAJ_LogAsserValPC("vitesseG1ms", vitesseMoteurG);
//    ASSER_TRAJ_LogAsserValPC("vitesseD1ms", vitesseMoteurD);

    ASSER_TRAJ_LogAsserValPC("ConsigneMoteurDroit_MS", 0.0);
    ASSER_TRAJ_LogAsserValPC("VitesseProfil", 0.0);
    ASSER_TRAJ_LogAsserValPC("VgASR", 0.0);
    ASSER_TRAJ_LogAsserValPC("Phase", 0.0);
    ASSER_TRAJ_LogAsserValPC("fFin", 0.0);
    ASSER_TRAJ_LogAsserValPC("VpiG", 0.0);
    ASSER_TRAJ_LogAsserValPC("VpiD", 0.0);
    ASSER_TRAJ_LogAsserValPC("VposG", 0.0);
    ASSER_TRAJ_LogAsserValPC("VposD", 0.0);        
}

void SIMU_LogRobot(void)
{
    ASSER_TRAJ_LogAsserValPC("vitesseMoteurGauche", vitesseMoteurG);
    ASSER_TRAJ_LogAsserValPC("vitesseMoteurDroit", vitesseMoteurD);

    ASSER_TRAJ_LogAsserValPC("erreurVitesseMoteurGauche", erreurVitesseMoteurG);
    ASSER_TRAJ_LogAsserValPC("erreurVitesseMoteurDroit", erreurVitesseMoteurD);

    ASSER_TRAJ_LogAsserValPC("tensionPWM_MoteurGauche", tensionPWM_G);
    ASSER_TRAJ_LogAsserValPC("tensionPWM_MoteurDroit", tensionPWM_D);
/*
    ASSER_TRAJ_LogAsserValPC("integPIVG", integPI_G*KI_GAUCHE);
    ASSER_TRAJ_LogAsserValPC("integPIVD", integPI_D*KI_DROIT);
*/
    ASSER_TRAJ_LogAsserValPC("ConsigneMoteurGauche", (float)g_ConsigneMoteurG);
    ASSER_TRAJ_LogAsserValPC("ConsigneMoteurDroit", (float)g_ConsigneMoteurD);
    ASSER_TRAJ_LogAsserValPC("vitLongitudinale", vitessesConsignes.longitudinale);
//    ASSER_TRAJ_LogAsserValPC("vitRotation", vitessesConsignes.rotation);

    ASSER_TRAJ_LogAsserValPC("ConsigneMoteurDroit_MS", (((float)g_ConsigneMoteurD)-((float)OffsetPWM))*SIMU_gain());
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
//    ASSER_TRAJ_LogAsserValPC("time", ASSER_compteurPeriode * TE);
    printf("Motion accomplished. Pose x:%1.3fm y:%1.3fm theta:%1.3f time:%1.2fs.\n", poseRobot.x, poseRobot.y, ((poseRobot.angle * 180.0) / PI), (ASSER_compteurPeriode * TE));
    ASSER_TRAJ_LogAsserValPC("Motion", 1.0);
    fflush(stdout);
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
    erreurVitesseMoteurG = SIMU_ErreurVitesseConsPWMVToVitMS(simu_consigneMoteurG, vitesseMoteurG, GAIN_STATIQUE_MOTEUR_G);
    erreurVitesseMoteurD = SIMU_ErreurVitesseConsPWMVToVitMS(simu_consigneMoteurD, vitesseMoteurD, GAIN_STATIQUE_MOTEUR_D);

    tensionPWM_G = (signed int)SIMU_RegulateurPI_BHT(GAUCHE, erreurVitesseMoteurG, &integPI_G, KI_GAUCHE, KP_GAUCHE, OffsetPWM, TE_PI);

//    /* Mesure des instants de saturation du correcteur PI gauche */
//    if ((tensionPWM_G < -(OffsetPWM-0.1)) || (tensionPWM_G > (OffsetPWM-0.1)))
//    {
//        ASSER_TRAJ_LogAsserValPC("saturationPIgauche", 1.0);
//    }
//    else
//    {
//        ASSER_TRAJ_LogAsserValPC("saturationPIgauche", 0.0);
//    }
    tensionPWM_D = (signed int)SIMU_RegulateurPI_BHT(DROIT, erreurVitesseMoteurD, &integPI_D, KI_DROIT, KP_DROIT, OffsetPWM, TE_PI);

    /* Mesure des instants de saturation du correcteur PI droit */
//    if ((tensionPWM_D < -(OffsetPWM-0.1)) || (tensionPWM_D > (OffsetPWM-0.1)))
//    {
//        ASSER_TRAJ_LogAsserValPC("saturationPIdroit", 1.0);
//    }
//    else
//    {
//        ASSER_TRAJ_LogAsserValPC("saturationPIdroit", 0.0);
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
                                   , 0.1707 //0.072
                                   , 0.003 //0.431
                                   , GAIN_STATIQUE_MOTEUR_G
                                   , 0 //20
                                   , SIMU_CR
                                   , TE_PI
                                   );

    /*
    SIMU_SimulationMoteurCC_ordre2_complet((signed int)tensionPWM_G
                                           , tensionPWM_G_n
                                           , &vitesseMoteurG
                                           , &vitesseMoteurG_n2
                                           , MASSE
                                           , RAYON_ROUE
                                           , FROTTEMENT_FLUIDE
                                           , FORCE_RESISTANTE
                                           , RESISTANCE_INDUIT
                                           , INDUCTANCE_INDUIT
                                           , CONSTANTE_COUPLE
                                           , CONSTANTE_VITESSE
                                           , RAPPORT_REDUCTION
                                           , NB_PERIODE_RETARD
                                           , TE_PI
                                           );
    */

    // 1:gain statique, 2:premiere constante de temps, 3: deuxieme constante de temps
    // moteur droit [0.93041457097724989, 0.0018257971355764644, 0.20174006794915703] avec un retard pur de 0.02s
    //SIMU_SimulationMoteurCC((signed int)tensionPWM_D, &vitesseMoteurD, SIMU_TM, SIMU_KM, SIMU_CR);

    SIMU_SimulationMoteurCC_ordre2((signed int)tensionPWM_D
                                   , tensionPWM_D_n
                                   , &vitesseMoteurD
                                   , &vitesseMoteurD_n2
                                   , 0.001 //0.072
                                   , 0.1763 //0.431
                                   , GAIN_STATIQUE_MOTEUR_D
                                   , 0 //20
                                   , SIMU_CR
                                   , TE_PI
                                   );

    /*
    SIMU_SimulationMoteurCC_ordre2_complet((signed int)tensionPWM_D
                                           , tensionPWM_D_n
                                           , &vitesseMoteurD
                                           , &vitesseMoteurD_n2
                                           , MASSE
                                           , RAYON_ROUE
                                           , FROTTEMENT_FLUIDE
                                           , FORCE_RESISTANTE
                                           , RESISTANCE_INDUIT
                                           , INDUCTANCE_INDUIT
                                           , CONSTANTE_COUPLE
                                           , CONSTANTE_VITESSE
                                           , RAPPORT_REDUCTION
                                           , NB_PERIODE_RETARD
                                           , TE_PI
                                           );
    */

//    if (ASSER_compteurPeriode == 1)
//    {
//        ASSER_TRAJ_LogAsserValPC("vitesseG1ms", (float)tensionPWM_G);
//        ASSER_TRAJ_LogAsserValPC("vitesseD1ms", (float)tensionPWM_D);
//    }

}

void SIMU_CalculPeriodique(void)
{
    int p;
        ASSER_TRAJ_LogAsserValPC("ASSER_Running", ASSER_Running);
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
            
         		ASSER_TRAJ_LogAsserValPC("VpiG", ((((float)(g_ConsigneMoteurG - 1023)) * 0.630) / 1023.0));
         		ASSER_TRAJ_LogAsserValPC("VpiD", ((((float)(g_ConsigneMoteurD - 1023)) * 0.606) / 1023.0));

            /***********************************/
            /* Envois des consignes des boucles de vitesse au PIC asser */
            SIMU_REDEF_ASSER_SendConsigne(g_ConsigneMoteurD, g_ConsigneMoteurG);
            /*************************************************************************************************/
            /*************************************************************************************************/

            /* Boucle de vitesse, avec une periode de 1ms sachant que la periode de l'asser trajectoire est de 20x 1ms */
            for (p = 0; p < floor(TE/TE_PI); p++)
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

    erreurVitesseMoteurG = SIMU_ErreurVitesseConsPWMVToVitMS(simu_consigneMoteurG, vitesseMoteurG, SIMU_gain());

    //tensionPWM_G = SIMU_RegulateurPI_BHT(GAUCHE, erreurVitesseMoteurG, &integPI_G, KI_GAUCHE, KP_GAUCHE, OffsetPWM * SIMU_KM, 0.001) / (float)SIMU_KM;
    tensionPWM_G = (float)floor(SIMU_RegulateurPI_BHT(GAUCHE, erreurVitesseMoteurG, &integPI_G, KI_GAUCHE, KP_GAUCHE, OffsetPWM, TE_PI));
    if ((tensionPWM_G < -(OffsetPWM-0.1)) || (tensionPWM_G > (OffsetPWM-0.1)))
    {
        ASSER_TRAJ_LogAsserValPC("saturationPIgauche", 1.0);
    }
    else
    {
        ASSER_TRAJ_LogAsserValPC("saturationPIgauche", 0.0);
    }

    //SIMU_SimulationMoteurCC((signed int)tensionPWM_G, &vitesseMoteurG, SIMU_TM, SIMU_KM, SIMU_CR);

    // 1:gain statique, 2:premiere constante de temps, 3: deuxieme constante de temps
    // moteur gauche [0.90848238070313447, 0.001532263578663721, 0.14712688461925522]
    //SIMU_SimulationMoteurCC((signed int)tensionPWM_G, &vitesseMoteurG, SIMU_TM, SIMU_KM, SIMU_CR);
    /*
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
                                   );*/

    SIMU_SimulationMoteurCC_ordre2_complet((signed int)tensionPWM_G
                                           , tensionPWM_G_n
                                           , &vitesseMoteurG
                                           , &vitesseMoteurG_n2
                                           , MASSE
                                           , RAYON_ROUE
                                           , FROTTEMENT_FLUIDE
                                           , FORCE_RESISTANTE
                                           , RESISTANCE_INDUIT
                                           , INDUCTANCE_INDUIT
                                           , CONSTANTE_COUPLE
                                           , CONSTANTE_VITESSE
                                           , RAPPORT_REDUCTION
                                           , NB_PERIODE_RETARD
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

#if 0

void SIMU_AsserVitessePI(void)
{
    int p;
    float len_parcours = 1.0;
    float distance_parcourue = 0.0;

    Pose    poseInit;
    PtTraj chemin[1];
    unsigned char mouvement;
    char marche;
    float angle_arrivee;
    unsigned int nbrePtsChemin;
    float vitesse_cons_mps = 0.0;

    poseInit.x = 0.0;
    poseInit.y = 0.0;
    poseInit.angle = 0.0;
    POS_InitialisationPoseRobot(poseInit);
    ASSER_TRAJ_InitialisationGenerale();
    chemin[0].x = CONVERT_FLOAT2SHORT_DISTANCE(poseInit.x + len_parcours);
    chemin[0].y = CONVERT_FLOAT2SHORT_DISTANCE(poseInit.y);
    angle_arrivee = 0.0;
    nbrePtsChemin = 1u;
    mouvement = DEPLACEMENT;
    marche = MARCHE_AVANT;
    SIMU_REDEF_ASSER_GoTo(chemin, nbrePtsChemin, mouvement, marche, angle_arrivee);

    compteurPeriodePI = 0;
    ASSER_TRAJ_LogAsserValPC("distPI", distance_parcourue);
    ASSER_TRAJ_LogAsserValPC("vitconsPI", vitesse_cons_mps);
    while ((distance_parcourue < len_parcours) && (compteurPeriodePI < (unsigned int)(TEMPS_SIMULATION / TE))) /* -> simulation pendant TEMPS_SIMULATION secondes au max */
    {
        compteurPeriodePI = compteurPeriodePI + 1;
        vitesse_cons_mps = ASSER_TRAJ_GabaritVitesse_getVitesse_vs_Distance(distance_parcourue);

        ASSER_TRAJ_LogAsserValPC("vitconsPI", vitesse_cons_mps);

        g_ConsigneMoteurG = (unsigned int)(vitesse_cons_mps / SIMU_gain()) + OffsetPWM;

        SIMU_REDEF_ASSER_SendConsigne(g_ConsigneMoteurD, g_ConsigneMoteurG);
        /* Boucle de vitesse, avec une periode de 1ms sachant que la periode de l'asser trajectoire est de 20x 1ms */
        for (p = 0; p < floor(TE/TE_PI); p++)
        {
            SIMU_BoucleVitesse();
            distance_parcourue += vitesseMoteurG * TE_PI;
        }
        ASSER_TRAJ_LogAsserValPC("distPI", distance_parcourue);
        SIMU_LogRobot();
    }
}

#endif

int SIMU_Mouvement(void)
{
    while( ((ASSER_Running == True) || (vitesseMoteurG > 10.1) || (vitesseMoteurD > 10.1)) && (ASSER_compteurPeriode < (unsigned int)(TEMPS_SIMULATION / TE)))
    {
        SIMU_CalculPeriodique();
        SIMU_LogRobot();
    }
    ASSER_TRAJ_LogAsserValPC("time", ASSER_compteurPeriode * TE);
    /* FIN 1er DEPLACEMENT */
    if (ASSER_Running == True)
    {
//        ASSER_TRAJ_LogAsserValPC("time", -1.0);
        printf("Motion failed: timeout.\n");
        ASSER_TRAJ_LogAsserValPC("Motion", -1.0);
        fflush(stdout);
        ASSER_Running = False;
        return 0;
    }
    else
    {
        ASSER_TRAJ_AfficheInfoFinAsser();
        return 1;
    }
}
