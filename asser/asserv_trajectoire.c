/************************************************************************************************/
/*! \brief asser_trajectoire.c
 *
 *  \author Jerome Poncin/Francois Ouvradou/Nicolas Chevillon
 *  \version 1.0.0
 *  \date    10/12/2008
 */
/************************************************************************************************/

/************************************************************************************************
 * asser_trajectoire management :   - la variable de pose du robot avec les fonctions de sa gestion
 *                                                  - les fonctions de gestion et de calculs de l'asservissement de trajectoire
 ************************************************************************************************/

/* Fichier *.h qui sont utilises par le module Asser_Trajectoire */

#include "define.h"
#ifdef PIC32_BUILD
#include "includes.h"
#include "tools.h"
#include "pic18.h"
#ifndef S_SPLINT_S
#include "math.h"
#endif /* S_SPLINT_S */
#include "task_evit_opt.h"
#include "HTTPpages.h"
#else /* PIC32_BUILD */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#endif /* PIC32_BUILD */
#include "task_asser.h"
#include "asserv_trajectoire.h"

/*! \addtogroup Task_Asser
 *  @{
 */

/*! \addtogroup Asser_Trajectoire
 *  @{
 */

/*! \addtogroup Asser_Trajectoire_Core
 *  @{
 */
    
/**********************************************************************
 * Definition dedicated to the local functions.
 **********************************************************************/

/***********************************************************************************************/
/**************************** Parametres de l'asservissement de trajectoire ****************************/
/***********************************************************************************************/
    
/** Parametres de l'asservissement par retour d'etat */
float                           gainRotation1                           =   - 6.0;   /* Gain de l'asservissement de la vitesse longitudinale */
float                           gainRotation2                           =   - 6.0;   /* Gain de l'asservissement de la vitesse de rotation */
float                           gainDeplacement1                        =   20.0;
float                           gainDeplacement2                        =   50.0;
float                           gainDeplacement3                        =   20.0;
    
/** Parametres de la generation de trajectoire */
float                           tempsAcc                                =   1.0;      /* Acceleration du deplacement sur la trajectoire,
                                                                                         -> temps en secondes pour passer de 0 a la vitesse max retournee par POS_GetConsVitesseMax() */
float                           A_MAX                                   =   4.0;
float                           D_MAX                                   =   -2.0;
float                           tempsDecc                               =   1.0;

float                           facteurVitesseAngulaireMax              =   4.0;

float                           COEFF_VI1                               =   0.95;
float                           VITESSE_SEUIL_DECC                      =   0.2;
float                           COEFF_DECC_FINALE                       =   0.08;
float                           DECC_MIN                                =   -0.3;       /* Decceleration en m/s^2 du profil de vitesse au point d'arrivee */

float                           RATIO_ACC                               =   1.0;
float                           RATIO_DECC                              =   1.0;

/***********************************************************************************************/
/**************************** Fin de la definition des parametres de l'asservissement de trajectoire *********/
/***********************************************************************************************/

/*----------------------------------------------------------------------------------------------*/

/** Variables globales */

/** Table pour le log asser */
float                           tabLogAsser[NBR_ASSER_LOG_VALUE];

/** Echnatillon de mesure pour le log asser */
unsigned char                   Sample                                  =   0;

/** Pour prendre les mesures 1/2 */
unsigned char                   TakeMesure                              =   False;

/*----------------------------------------------------------------------------------------------*/

/** Variables locales */

/** Le point du robot asservit a la trajectoire n'est pas le centre de l'axe des roues, mais un point sur la droite perpendiculaire a cet axe et passant par son centre, situe a la distance NORME_BARRE_SUIVI_TRAJ en avant de l'axe des roues */
static float                    NORME_BARRE_SUIVI_TRAJ;									/* Initialise dans ASSER_TRAJ_InitialisationGenerale() */

/** Variables globales de l'asservissement de trajectoire */
static unsigned int             compteurPeriode                         =   0;
static float                    errDist, memo_errDist, errAngle, memo_errAngle, memo_angleRobot;

/** Vecteurs associees au depart et a l'arrivee, et parametrage du profil de vitesse de la trajectoire */
static Trajectoire              chemin;
static unsigned int             segmentCourant                          =   0;

/** Matrice pour l'interpolation des points intermediaires de la trajectoire */
static float                    g_matriceT[(NBRE_MAX_PTS_TRAJ + 2)][(2 * (NBRE_MAX_PTS_TRAJ + 2))];
static Vecteur                  g_tableauH[(NBRE_MAX_PTS_TRAJ + 2)];
static Vecteur                  g_tableauF[(NBRE_MAX_PTS_TRAJ + 2)];
static Vecteur                  g_tableauPoints[(NBRE_MAX_PTS_TRAJ + 3)];
static unsigned int             g_tableauMask[(NBRE_MAX_PTS_TRAJ + 2)];

/** Pose de la trajectoire de consigne a l'instant t*/
static Pose                     poseReference                           =   {0.0,0.0,0.0};
static Pose                     poseReferenceRobot                      =   {0.0,0.0,0.0};
static Pose                     poseReferenceAv                         =   {0.0,0.0,0.0};
//static Pose                     poseReferenceRobotOld                   =   {0.0,0.0,0.0};
//static Pose                     poseReferenceRobotAv                   =   {0.0,0.0,0.0};
//static Pose                     poseReferenceOld                        =   {0.0,0.0,0.0};

/** Vitesse normalisee courante */
//static float                    vitesseNormaliseeCourante               =   0.0;  
//static float                    memoVitesseNormaliseeCourante           =   0.0;
/* distance a parcourir pendant la periode courante, distance qui est normalisee par la distance totale a parcourir. Elle sert a faire passer la vitesse d'une commande d'asser a l'autre en cas de changement de trajectoire en cours de mouvement */
static float                    vitesse_profil_consigne                 =   0.0;
static float                    parametrePositionSegmentTrajectoire     =   0.0;    
static float                    memo_vitesse_profil_consigne            =   0.0;
static unsigned char            shuntTestFinAsser                       =   False;

/** Tableaux du profil de vitesse */
static float                    g_tab_gabarit_vitesse[TAILLE_TAB_GABARIT_VITESSE];
static float                    g_tab_gabarit_acceleration[TAILLE_TAB_GABARIT_VITESSE];
static unsigned int             g_index_tab_gabarit_vitesse = 0;

// tableau de 1: distance des phases d'acc et de decc reunies, 2: vitesse max(de pointe), 3; distance de decc
static float                    g_tab_vpointe[5][3]; // = {{0.04, 0.2, 0.025}, {0.085, 0.3, 0.04}, {0.165, 0.4, 0.075}, {0.27, 0.5, 0.115}, {0.4, 0.6, 0.165}};

static Pose                     g_poseRobotTrajectoire;

/** Test */
//static float                    g_distance_suivante = 0.0;

/*----------------------------------------------------------------------------------------------*/

/* Prototypes des fonctions */

#if 0
static float                    ASSER_TRAJ_VitesseLimiteEnVirage(Trajectoire *traj, float diffThetaTrajectoire);
static float*                   ASSER_TRAJ_getGabaritVitesse(void);
#endif

static Pose                     ASSER_TRAJ_TrajectoireRotation(ParametresRotation *p_rotation, float t);
static void                     ASSER_TRAJ_GabaritVitesse(Trajectoire *traj);
static unsigned char            ASSER_TRAJ_ParcoursTrajectoire(Trajectoire *traj, float delta_distance, unsigned int *segmentCourant, float *paramPoseSegTraj);
static Pose                     ASSER_TRAJ_DiffTemporelleTrajectoire(Pose posePrecedente, Pose poseActuelle, float periode);
static void                     ASSER_TRAJ_MatriceRotation(Vecteur *matrice2p2, float angle);
static Vecteur                  ASSER_TRAJ_ProduitMatriceVecteur(Vecteur *matrice, Vecteur vecteur);
static Pose                     ASSER_TRAJ_ErreurPose(Pose poseRobot_P, Pose poseRef);
static VitessesRobotUnicycle    ASSER_TRAJ_RetourDetat(Pose erreurPose, Pose poseRef, Pose diffPoseRef, float longueurBarreSuivi, float *diagMatriceGain);
static VitessesRobotUnicycle    ASSER_TRAJ_RetourDetatOrientation(Pose erreurPose, Pose diffPoseRef, float gain[]);
static unsigned char            ASSER_TRAJ_TestFinAsservissement(Trajectoire *traj, float erDist, float memo_erDist, float tolDist, float erAngle, float memo_erAngle, float tolAngle);
static void                     ASSER_TRAJ_DistanceTrajectoire(segmentTrajectoireBS *segmentTraj);
static void                     ASSER_TRAJ_InitialisationMatriceInterpolation(unsigned int iPtI, unsigned int iPtF);
static void                     ASSER_TRAJ_InversionMatriceT(unsigned int nbrePts);
static Pose                     ASSER_TRAJ_TrajectoireBSpline(segmentTrajectoireBS *segmentTraj, float t);
static Vecteur                  ASSER_TRAJ_DiffCourbeBSpline(segmentTrajectoireBS *segmentTraj, float t);
static Vecteur                  ASSER_TRAJ_Diff2CourbeBSpline(segmentTrajectoireBS *segmentTraj, float t);
static float                    ASSER_TRAJ_DiffThetaBSpline(Vecteur diff1BS, Vecteur diff2BS);
static Pose                     ASSER_TRAJ_TrajectoireRemorqueBS(segmentTrajectoireBS *segmentTraj, float t, Vecteur diff1BS, float diffThetaBSRobot, Pose *poseTrajRobot);
static void                     ASSER_TRAJ_InitialisationCourbeBS_5(Vecteur ptI, Vecteur ptF, Vecteur deltaPtI, Vecteur deltaPtF, Vecteur qI, Vecteur qF, segmentTrajectoireBS *segmentTraj);
static void                     ASSER_TRAJ_InterpolationBSpline3(unsigned int iPtI, unsigned int iPtF);
static unsigned char            ASSER_TRAJ_ProfilAcceleration_2012(Trajectoire *traj, float vpointe, float *vitesse_n);
static unsigned char            ASSER_TRAJ_ProfilDecceleration_2012(Trajectoire *traj, float vpointe, float *vitesse_n);
static float                    ASSER_TRAJ_DistanceAcceleration(Trajectoire * traj, float vpointe, float vit_ini);
static float                    ASSER_TRAJ_DistanceDecceleration(Trajectoire * traj, float vpointe);
static void                     ASSER_TRAJ_InitTabVPointe(Trajectoire *traj, float coeff_vit_ini);
static unsigned char            ASSER_TRAJ_isDeplacement(Trajectoire *traj);

/*----------------------------------------------------------------------------------------------*/

/**********************************************************************/
/*! \brief ASSER_TRAJ_InitialisationGenerale
 * 
 *  \note  Fonction d'initialisation generale de l'asser trajectoire
 *
 *  \return None
 */
/**********************************************************************/

extern void ASSER_TRAJ_InitialisationGenerale(void)
{    
    /* Initialisation du profil de vitesse */
    chemin.profilVitesse.p = 0;

    NORME_BARRE_SUIVI_TRAJ = ECART_ROUE_LIBRE / 2.0;

    ASSER_TRAJ_ResetLogAsserTable();

    chemin.profilVitesse.tempsAcc = tempsAcc;

    POS_InitialisationConfigRobot();

    memo_angleRobot = m_poseRobot.angle;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_isDeplacement
 * 
 *  \note  Test le dÈplacement
 * 
 *  \param [in] traj              type de trajectoire
 *
 *  \return  True (Deplacement) or False (Rotation)
 */
/**********************************************************************/

static unsigned char ASSER_TRAJ_isDeplacement(Trajectoire * traj)
{
    if (traj->mouvement > ROTATION)
    {
        return True;
    }
    else
    {
        return False;
    }
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_GetCompteur
 * 
 *  \note  Fonction qui renvoie le compteur de periode
 *
 *  \return compteur de Periode
 */
/**********************************************************************/

extern unsigned int ASSER_TRAJ_GetCompteur(void)
{
    return compteurPeriode;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_GetSegmentCourant
 * 
 *  \note  Fonction qui renvoie le segment de traj courant
 *
 *  \return segment Courant
 */
/**********************************************************************/

extern unsigned int ASSER_TRAJ_GetSegmentCourant(void)
{
    return segmentCourant;
}

/**********************************************************************/
/*! \brief AsservissementMouvementRobot
 * 
 *  \note  Fonction centrale de l'asservissement de trajectoire: bloc d'instructions a executer periodiquement,
 *           contenant toute la loi de commande et la condition d'arret de l'asservissement.
 *
 *  \param [in]   poseRobot              pose du robot etant donne le choix de la marche AVANT ou ARRIERE pour l'orientation du robot
 *  \param [in]   vitessesConsignes   pointeur sur une structure pour fournir les vitesses longitudinale et de rotation, de consignes
 *
 *  \return         None
 */
/**********************************************************************/
 
extern void ASSER_TRAJ_AsservissementMouvementRobot(Pose poseRobot, VitessesRobotUnicycle *vitessesConsignes)
{
    float           diagonaleMatriceGain[2];
    float           gain[5];
    float           facteurCorrectionT                      = 1.0;
/*    float           vitesseNormaliseeSegmentCourantCorrigee = 0.0;
    float           vitesseSegmentCourant       = 0.0;
    float           vitLimiteCourbure           = 0.0; */
    Pose            differentielleTemporellePoseReference;
    Pose            differentielleTemporellePoseReferenceRobot;
    Pose            P_robot;                                        /* Point fixe du robot a asservir a la trajectoire de consigne */
    Pose            erreurPoseCentreRobot;
    Pose            erreur_P;
    Vecteur         diff1BS, diff2BS;
    float           vitesseAngulaire                        = 0.0;
    float           delta_distance                          = 0.0;
    float           parametrePositionSegmentTrajectoireAv   = 0.0;
    unsigned int    segmentCourantAv;
    float           delta_distance_Av                       = 0.0;
    float           diffThetaCourantAv                      = 0.0;
    Pose            poseReferenceRobotAv;

    ASSER_TRAJ_LogAsser("xRoueGauche", NBR_ASSER_LOG_VALUE, poseRobot.x + (ECART_ROUE_MOTRICE / 2.0) * cos(poseRobot.angle + (PI / 2)));
    ASSER_TRAJ_LogAsser("yRoueGauche", NBR_ASSER_LOG_VALUE, poseRobot.y + (ECART_ROUE_MOTRICE / 2.0) * sin(poseRobot.angle + (PI / 2)));
    ASSER_TRAJ_LogAsser("xRoueDroite", NBR_ASSER_LOG_VALUE, poseRobot.x + (ECART_ROUE_MOTRICE / 2.0) * cos(poseRobot.angle - (PI / 2)));
    ASSER_TRAJ_LogAsser("yRoueDroite", NBR_ASSER_LOG_VALUE, poseRobot.y + (ECART_ROUE_MOTRICE / 2.0) * sin(poseRobot.angle - (PI / 2)));
    ASSER_TRAJ_LogAsser("angle", NBR_ASSER_LOG_VALUE, poseRobot.angle);

    erreurPoseCentreRobot.x = 0.0;    
    erreurPoseCentreRobot.y = 0.0;

    /* Initialisation des variables locales pour que leur contenu soit defini partout dans la fonction */
    differentielleTemporellePoseReference.x = 0.0;
    differentielleTemporellePoseReference.y = 0.0;
    differentielleTemporellePoseReference.angle = 0.0;
    differentielleTemporellePoseReferenceRobot.x = 0.0;
    differentielleTemporellePoseReferenceRobot.y = 0.0;
    differentielleTemporellePoseReferenceRobot.angle = 0.0;
    
    erreur_P.x = 0.0;
    erreur_P.y = 0.0;
    erreur_P.angle = 0.0;
    
    //memoVitesseNormaliseeCourante = 0.0;
    memo_vitesse_profil_consigne = 0.0;

    memo_errDist = errDist;
    errDist = POS_ErreurDistance(poseRobot, chemin.posArrivee);
//    ASSER_TRAJ_LogAsser("errDist", errDist);
    memo_errAngle = errAngle;
    errAngle = POS_ErreurOrientation(poseRobot, chemin.posArrivee);

    if (compteurPeriode > 0)
    {
        if (ASSER_TRAJ_TestFinAsservissement(&chemin, errDist, memo_errDist, DIST_MIN, errAngle, memo_errAngle, ANGLE_MIN) == True)
        {
            if ((chemin.profilVitesse.distNormaliseeRestante * chemin.distance) < ((float)(DIST_MIN + 0.05)))
            {
                if (shuntTestFinAsser == False)
                {
                    ASSER_Running = False;
                }
            }
        }
        else
        {
            shuntTestFinAsser = False;
        }
    }
    else
    {
        if (ASSER_TRAJ_TestFinAsservissement(&chemin, errDist, memo_errDist, DIST_MIN, errAngle, memo_errAngle, ANGLE_MIN) == True)
        {
            if (chemin.nbreSegments == (unsigned int)1)
            {
                ASSER_Running = False;
            }
            else
            {
                shuntTestFinAsser = True;
            }
        }
    }

    if (ASSER_Running == True)
    {
        /* Comptage du nombre de periodes depuis le lancement de la commande d'asservissement */
        compteurPeriode++;
    
        /* Generation de la pose de trajectoire de consigne suivante */
        if ( (chemin.profilVitesse.p > 0) | (chemin.profilVitesse.etat == 1) )
        {
#ifdef PIC32_BUILD
            EVIT_ProcheFinAsser = False;
#endif /* PIC32_BUILD */

/************************************************************/
/* Debut de la dermination de la nouvelle position a suivre */
/************************************************************/            

            if (chemin.profilVitesse.p > 0)
            {                
                /* distance de deplacement sur la periode */
                //delta_distance = vitesse_profil_consigne * TE;
                //delta_distance = cos(g_poseRobotTrajectoire.angle) * (poseRobot.x - g_poseRobotTrajectoire.x) + sin(g_poseRobotTrajectoire.angle) * (poseRobot.y - g_poseRobotTrajectoire.y);
                if (ASSER_TRAJ_isDeplacement(&chemin) == True)
                {
                    delta_distance = cos(poseReferenceRobot.angle) * (poseRobot.x - poseReferenceRobot.x) + sin(poseReferenceRobot.angle) * (poseRobot.y - poseReferenceRobot.y);
                }
                else
                {
                    delta_distance = NORME_BARRE_SUIVI_TRAJ * fabs(POS_ModuloAngle(poseRobot.angle - memo_angleRobot));
                    memo_angleRobot = poseRobot.angle;
                    ASSER_TRAJ_LogAsser("angleRobot", NBR_ASSER_LOG_VALUE, poseRobot.angle);
                    ASSER_TRAJ_LogAsser("angleRef", NBR_ASSER_LOG_VALUE, poseReference.angle);
                }

                if (delta_distance < 0.0)
                {
                    delta_distance = 0.0;
                }

                chemin.profilVitesse.distance_parcourue = chemin.profilVitesse.distance_parcourue + delta_distance;
                //g_index_tab_gabarit_vitesse = floor(chemin.profilVitesse.distance_parcourue / chemin.profilVitesse.pas_echantillon_distance);
                /* if(g_index_tab_gabarit_vitesse < 30u)
                {
                    g_index_tab_gabarit_vitesse = 30u;
                }*/
                /*
                if(g_index_tab_gabarit_vitesse > (TAILLE_TAB_GABARIT_VITESSE-1u))
                {
                    g_index_tab_gabarit_vitesse = (TAILLE_TAB_GABARIT_VITESSE-1u);
                }*/

                ASSER_TRAJ_LogAsser("dist_parcourue", NBR_ASSER_LOG_VALUE, chemin.profilVitesse.distance_parcourue);
                ASSER_TRAJ_LogAsser("nbdepas", NBR_ASSER_LOG_VALUE, chemin.profilVitesse.pas_echantillon_distance);
                ASSER_TRAJ_LogAsser("index_tab_vit", NBR_ASSER_LOG_VALUE, g_index_tab_gabarit_vitesse);

                /* distance normalisee restant a parcourir */
                chemin.profilVitesse.distNormaliseeRestante = chemin.profilVitesse.distNormaliseeRestante - (delta_distance / chemin.distance);
                ASSER_TRAJ_ParcoursTrajectoire(&chemin, delta_distance, &segmentCourant, &parametrePositionSegmentTrajectoire);
            }
            else
            {   
                /* Commande de vitesse nulle, le point d'arrivee de consigne est atteint */
                parametrePositionSegmentTrajectoire = 1.0;
            }
/**********************************************************/
/* Fin de la dermination de la nouvelle position a suivre */
/**********************************************************/

            //poseReferenceOld = poseReference;
            //poseReferenceRobotOld = poseReferenceRobot;
            
            if (ASSER_TRAJ_isDeplacement(&chemin) == True)
            {
                diff1BS = ASSER_TRAJ_DiffCourbeBSpline(&chemin.segmentTrajBS[segmentCourant], parametrePositionSegmentTrajectoire);
                diff2BS = ASSER_TRAJ_Diff2CourbeBSpline(&chemin.segmentTrajBS[segmentCourant], parametrePositionSegmentTrajectoire);
                chemin.profilVitesse.diffThetaCourant = ASSER_TRAJ_DiffThetaBSpline(diff1BS, diff2BS);
                poseReference = ASSER_TRAJ_TrajectoireRemorqueBS(&chemin.segmentTrajBS[segmentCourant], parametrePositionSegmentTrajectoire, diff1BS, chemin.profilVitesse.diffThetaCourant, &poseReferenceRobot);
                //g_poseRobotTrajectoire = poseReference;
                //poseReference.angle = atan2((poseReference.y - poseReferenceOld.y), (poseReference.x - poseReferenceOld.x));

                parametrePositionSegmentTrajectoireAv = parametrePositionSegmentTrajectoire;
                segmentCourantAv = segmentCourant;
                delta_distance_Av = ASSER_TRAJ_GabaritVitesse_getVitesse_vs_Distance(chemin.profilVitesse.distance_parcourue) * TE; //g_tab_gabarit_vitesse[g_index_tab_gabarit_vitesse] * TE;
                ASSER_TRAJ_ParcoursTrajectoire(&chemin, delta_distance_Av, &segmentCourantAv, &parametrePositionSegmentTrajectoireAv);
                ASSER_TRAJ_LogAsser("segTrajAv", NBR_ASSER_LOG_VALUE, parametrePositionSegmentTrajectoireAv);
                diff1BS = ASSER_TRAJ_DiffCourbeBSpline(&chemin.segmentTrajBS[segmentCourantAv], parametrePositionSegmentTrajectoireAv);
                diff2BS = ASSER_TRAJ_Diff2CourbeBSpline(&chemin.segmentTrajBS[segmentCourantAv], parametrePositionSegmentTrajectoireAv);
                diffThetaCourantAv = ASSER_TRAJ_DiffThetaBSpline(diff1BS, diff2BS);
                poseReferenceAv = ASSER_TRAJ_TrajectoireRemorqueBS(&chemin.segmentTrajBS[segmentCourantAv], parametrePositionSegmentTrajectoireAv, diff1BS, diffThetaCourantAv, &poseReferenceRobotAv);
            }
            else /* if (chemin.mouvement == ROTATION) */
            {
                poseReference = ASSER_TRAJ_TrajectoireRotation(&(chemin.rotation), parametrePositionSegmentTrajectoire);
                //poseReference.angle = poseRobot.angle;

                //g_poseRobotTrajectoire = poseReference;
                ASSER_TRAJ_LogAsser("thetaPoseReference", NBR_ASSER_LOG_VALUE, poseReference.angle);

                parametrePositionSegmentTrajectoireAv = parametrePositionSegmentTrajectoire;
                segmentCourantAv = segmentCourant;
                delta_distance_Av = ASSER_TRAJ_GabaritVitesse_getVitesse_vs_Distance(chemin.profilVitesse.distance_parcourue) * TE;
                ASSER_TRAJ_ParcoursTrajectoire(&chemin, delta_distance_Av, &segmentCourantAv, &parametrePositionSegmentTrajectoireAv);
                poseReferenceAv = ASSER_TRAJ_TrajectoireRotation(&(chemin.rotation), parametrePositionSegmentTrajectoireAv);
            }
            ASSER_TRAJ_LogAsser("val_tab_vit", NBR_ASSER_LOG_VALUE, delta_distance_Av/TE);
            
            //differentielleTemporellePoseReference = ASSER_TRAJ_DiffTemporelleTrajectoire(poseReferenceOld, poseReference, TE);
            differentielleTemporellePoseReference = ASSER_TRAJ_DiffTemporelleTrajectoire(poseReference, poseReferenceAv, TE);

            //differentielleTemporellePoseReferenceRobot = ASSER_TRAJ_DiffTemporelleTrajectoire(poseReferenceRobotOld, poseReferenceRobot, TE);
            differentielleTemporellePoseReferenceRobot = ASSER_TRAJ_DiffTemporelleTrajectoire(poseReferenceRobot, poseReferenceRobotAv, TE);
        }
        else
        {
            vitesse_profil_consigne = 0.0;
            memo_vitesse_profil_consigne = 0.0;
            
#ifdef PIC32_BUILD
            /* Desactivation test antiblockage avant de chercher precisement le point d'arrivee pour eviter de passser en evitement */
            EVIT_ProcheFinAsser = True;
#endif /* PIC32_BUILD */
        }

        if (ASSER_TRAJ_isDeplacement(&chemin) == True)
        {
            erreurPoseCentreRobot = ASSER_TRAJ_ErreurPose(poseRobot, poseReferenceRobot);

            gain[0] = gainDeplacement1;
            gain[1] = gainDeplacement2;
            gain[2] = gainDeplacement3;
            gain[3] = gainRotation1;
            gain[4] = gainRotation2;
            
            *vitessesConsignes = ASSER_TRAJ_RetourDetatOrientation(erreurPoseCentreRobot, differentielleTemporellePoseReferenceRobot, gain);
        }
        else /* ROTATION */
        {
            /* Calcul du point deporte du centre des roues motrices a asservir a la trajectoire */
            P_robot.x = poseRobot.x + (NORME_BARRE_SUIVI_TRAJ * cos(poseRobot.angle));
            P_robot.y = poseRobot.y + (NORME_BARRE_SUIVI_TRAJ * sin(poseRobot.angle));
            P_robot.angle = poseRobot.angle;

            /* Calcul du vecteur d'erreur */
            erreur_P = ASSER_TRAJ_ErreurPose(P_robot, poseReference);

            /* Loi de commande par retour d'etat SANS orientation*/
            diagonaleMatriceGain[0] = gainRotation1;
            diagonaleMatriceGain[1] = gainRotation2;
            *vitessesConsignes = ASSER_TRAJ_RetourDetat(erreur_P, poseReference, differentielleTemporellePoseReference, NORME_BARRE_SUIVI_TRAJ, diagonaleMatriceGain);
        }

        /* Correction de la consigne de rotation une fois la trajectoire de consigne parcourue */
//         if (chemin.profilVitesse.p == 0)
//         {
// //             VmodCorrigee = sin(errAngle);
// //             VmodCorrigee = (fabs((float)vitesseMoyenne) * VmodCorrigee);
// //             VmodCorrigee = VmodCorrigee/errDist;
// //             vitModTemp = vitModTemp + VmodCorrigee;
//             vitessesConsignes->rotation = vitessesConsignes->rotation + (fabs(vitessesConsignes->longitudinale) * sin(errAngle) * 2.0) / (errDist * 0.3);
//         }

        /* Pour s'assurer d'une pure rotation, la vitesse longitudinale est forcee a zero */
        if (chemin.mouvement == ROTATION)
        {
            vitessesConsignes->longitudinale = 0.0;
        }
    }
    else
    {
        vitessesConsignes->longitudinale = 0.0;
        vitessesConsignes->rotation = 0.0;
        vitesse_profil_consigne = 0.0;
        memo_vitesse_profil_consigne = 0.0;
        chemin.profilVitesse.p = 0;
    }

    vitesseAngulaire = (chemin.profilVitesse.diffThetaCourant * ((memo_vitesse_profil_consigne) / facteurCorrectionT)) / TE;

    /* Log de valeurs */
    ASSER_TRAJ_LogAsser("xCCourant", NBR_ASSER_LOG_VALUE, ASSER_TRAJ_TrajectoireBSpline(&chemin.segmentTrajBS[segmentCourant], parametrePositionSegmentTrajectoire).x);
    ASSER_TRAJ_LogAsser("xPoseReferenceRobot", NBR_ASSER_LOG_VALUE, poseReferenceRobot.x);
    ASSER_TRAJ_LogAsser("yPoseReferenceRobot", NBR_ASSER_LOG_VALUE, poseReferenceRobot.y);
    ASSER_TRAJ_LogAsser("distNormaliseeRestante", NBR_ASSER_LOG_VALUE, chemin.profilVitesse.distNormaliseeRestante);
    //ASSER_TRAJ_LogAsser("yPoseReference", poseReference.y);
//    ASSER_TRAJ_LogAsser("xPoseReferenceRobot", poseReferenceRobot.x);
    //ASSER_TRAJ_LogAsser("orientationPoseReferenceRobot", poseReferenceRobot.angle);
    /*
    ASSER_TRAJ_LogAsser("poseReferenceRobot", segmentCourant);
    */
    ASSER_TRAJ_LogAsser("vitesseProfilConsigne", NBR_ASSER_LOG_VALUE, memo_vitesse_profil_consigne);

    ASSER_TRAJ_LogAsser("parametrePositionSegmentTrajectoire", NBR_ASSER_LOG_VALUE, parametrePositionSegmentTrajectoire);

    ASSER_TRAJ_LogAsser("segmentCourant", NBR_ASSER_LOG_VALUE, segmentCourant);
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_InitialisationTrajectoire
 * 
 *  \note   Initialisation d'une structure trajectoire definit en global dans le module ASSER_TRAJ
 *
 *  \param [in]     poseRobot       pose courante du robot integrant le choix de la marche (AVANT ou ARRIERE)
 *  \param [in]     point               pointeur du tableau des points imposes du chemin dont le point a atteindre en dernier
 *  \param [in]     nbrePts           nombre de points intermediaires du chemin par lesquelles passer +1 pour le point d'arrvee
 *  \param [in]     mouvement     type de mouvement a executer: un deplacement (DEPLACEMENT), une pure rotation (ROTATION) ou un d√©placement qui finit en ligne droite (DEPLACEMENT_LIGNE_DROITE)
 *
 *  \return           None
 */
/**********************************************************************/
extern void ASSER_TRAJ_InitialisationTrajectoire(Pose poseRobot, PtTraj * point, unsigned int nbrePts, unsigned int mouvement)
{
    Vecteur         diff1BS, diff2BS;
    unsigned int    iSegment, j, iCurrentGPoint_n0, iCurrentGPoint_n1;
    Vecteur         deltaI, deltaF, qI, qF;
    float           cordeInitiale, cordeFinale;
    float           angleInitial, angleFinal;
#ifndef PIC32_BUILD
    Pose            poseTest;
    int             intT;
    float           sommeMask;
#endif /* PIC32_BUILD */
    /* var des resultats de calcul de puissance */
    float           g_tableauH_iCurrentGPoint_n0_x_sq, g_tableauH_iCurrentGPoint_n0_y_sq;
    float           g_tableauH_iCurrentGPoint_n1_m1_x_sq, g_tableauH_iCurrentGPoint_n1_m1_y_sq;

    /* Temps d'acceleration / decceration par defaut */
    tempsAcc = chemin.profilVitesse.tempsAcc;
    tempsDecc = chemin.profilVitesse.tempsAcc * 6.0; //6.0;

    chemin.mouvement = mouvement;
    chemin.nbreSegments = nbrePts;
    parametrePositionSegmentTrajectoire = 0.0;
    segmentCourant = 0;
    shuntTestFinAsser = False;

    /* Constitution des points a interpoler */
    g_tableauPoints[0].x = poseRobot.x;
    g_tableauPoints[0].y = poseRobot.y;
    g_tableauMask[0] = 1;
    
    /* Points intermediaires transmis par l'utilisateur */
    for (j = 0; j < nbrePts; j++)
    {
        g_tableauPoints[(j + 1)].x = point[j].pose.x;
        g_tableauPoints[(j + 1)].y = point[j].pose.y;
        g_tableauMask[(j + 1)] = point[j].mask;
    }

    /* En cas de deplacement, l'angle final est de toute facon exploite --> soit calcule, soit defini */
    if (ASSER_TRAJ_isDeplacement(&chemin) == True)
    {
        g_tableauMask[nbrePts] = 1;
    }

    /* Liste des points terminee */
    if (ASSER_TRAJ_isDeplacement(&chemin) == True)
    {        
        /* Position a atteindre (condition d'arret du test de fin d'asservissment) */
        chemin.posArrivee.x = point[(nbrePts - 1)].pose.x;
        chemin.posArrivee.y = point[(nbrePts - 1)].pose.y;

        /* Configuration du profil de vitesse */
        chemin.profilVitesse.vmax = POS_GetConsVitesseMax();                /* UMAX * GAIN_STATIQUE_MOTEUR */

        if (nbrePts == 1)
        {
            /* Conditions aux limites des derivees */
            cordeInitiale = sqrt(SQUARE(g_tableauPoints[1].x - g_tableauPoints[0].x) + SQUARE(g_tableauPoints[1].y - g_tableauPoints[0].y));
            angleInitial = poseRobot.angle;

            deltaI.x =  (cordeInitiale * 1.0) * cos(angleInitial);
            deltaI.y =  (cordeInitiale * 1.0) * sin(angleInitial);
            angleFinal = point[0].pose.angle;

            if ( chemin.mouvement == DEPLACEMENT_LIGNE_DROITE)
            {
                angleFinal = atan2(point[0].pose.y - poseRobot.y, point[0].pose.x - poseRobot.x );
            }

            deltaF.x =  (cordeInitiale * 1.0) * cos(angleFinal);
            deltaF.y =  (cordeInitiale * 1.0) * sin(angleFinal);

            g_tableauH[0].x = 1.0;
            g_tableauH[0].y = 1.0;
            chemin.segmentTrajBS[0].t[0] = 0.0;
            chemin.segmentTrajBS[0].t[1] = 1.0;
            
            /* Definition de l'unique segment en B-Spline d'ordre 5, pour tenir compte des angles aux limites de la trajectoire */
            qI.x = 0.0;
            qI.y = 0.0;
            qF.x = 0.0;
            qF.y = 0.0;
            
            ASSER_TRAJ_InitialisationCourbeBS_5(g_tableauPoints[0], g_tableauPoints[1], deltaI, deltaF, qI, qF, &chemin.segmentTrajBS[0]);
            chemin.segmentTrajBS[0].ordre = 5;
        }
        else
        {
            /* Configuration de toute la trajectoire en spline d'ordre 3 */
            ASSER_TRAJ_InterpolationBSpline3(0, nbrePts);

            /* Reconfiguration en ordre 5 pour les segments dont au moins un des angles aux limites est impose. */
            angleInitial = poseRobot.angle;

            iCurrentGPoint_n0 = 0;
            iCurrentGPoint_n1 = 1;

            while (iCurrentGPoint_n1 < (nbrePts + 1))
            {   
                /* Recherche du prochain point sur lequel l'angle est impose (-> mask == 1) */
                if ((g_tableauMask[iCurrentGPoint_n1] == (unsigned int)1) /* | (iCurrentGPoint_n1 == nbrePts) */ )
                {
                    /* Configuration d'une portion de trajectoire */
                    angleFinal = point[(iCurrentGPoint_n1 - 1)].pose.angle;

                    /* Calcul de l'angle dans le cas deplacement ligne droite et dernier point de la traj */
                    if ((chemin.mouvement == DEPLACEMENT_LIGNE_DROITE) && (iCurrentGPoint_n1 == nbrePts))
                    {
                        angleFinal = atan2( (point[iCurrentGPoint_n1].pose.y - point[(iCurrentGPoint_n1 - 1)].pose.y),
                                            (point[iCurrentGPoint_n1].pose.x - point[(iCurrentGPoint_n1 - 1)].pose.x));
                    }

                    /*** Debut d'une boucle de config de portion de trajectoire ***/
                    /* Passage des segments de bords en B-Spline d'ordre 5, pour tenir compte des angles aux limites de chaque portion de trajectoire */
                    /* Conditions aux limites des derivees */
                    cordeInitiale = sqrt(SQUARE(g_tableauPoints[(iCurrentGPoint_n0 + 1)].x - g_tableauPoints[iCurrentGPoint_n0].x) + SQUARE(g_tableauPoints[(iCurrentGPoint_n0 + 1)].y - g_tableauPoints[iCurrentGPoint_n0].y));
                    deltaI.x = (cordeInitiale * 1.0) * cos(angleInitial);
                    deltaI.y = (cordeInitiale * 1.0) * sin(angleInitial);

                    if ((iCurrentGPoint_n1 - iCurrentGPoint_n0) == 1)
                    {
                        deltaF.x = (cordeInitiale * 1.0) * cos(angleFinal);
                        deltaF.y = (cordeInitiale * 1.0) * sin(angleFinal);

                        /* Redefinition du segment en BS d'ordre 5 */
                        /*
                        diff1BS.x = (chemin.segmentTrajBS[iCurrentGPoint_n0].mx[1] * g_tableauH[iCurrentGPoint_n0].x) / 2.0 + chemin.segmentTrajBS[iCurrentGPoint_n0].ax;
                        diff1BS.y = (chemin.segmentTrajBS[iCurrentGPoint_n0].my[1] * g_tableauH[iCurrentGPoint_n0].y) / 2.0 + chemin.segmentTrajBS[iCurrentGPoint_n0].ay;
                        diff1BS.x = diff1BS.x * g_tableauH[iCurrentGPoint_n0].x;
                        diff1BS.y = diff1BS.y * g_tableauH[iCurrentGPoint_n0].y;
                        */
                        //diff1BS = ASSER_TRAJ_DiffCourbeBSpline(&chemin.segmentTrajBS[0], chemin.segmentTrajBS[0].t[1]);
                        qI.x = 0.0; //chemin.segmentTrajBS[iCurrentGPoint_n0].mx[0] * pow(g_tableauH[iCurrentGPoint_n0].x, 2.0);
                        qI.y = 0.0; //chemin.segmentTrajBS[iCurrentGPoint_n0].my[0] * pow(g_tableauH[iCurrentGPoint_n0].y, 2.0);
                        qF.x = 0.0; //chemin.segmentTrajBS[iCurrentGPoint_n0].mx[1] * pow(g_tableauH[iCurrentGPoint_n0].x, 2.0);
                        qF.y = 0.0; //chemin.segmentTrajBS[iCurrentGPoint_n0].my[1] * pow(g_tableauH[iCurrentGPoint_n0].y, 2.0);

                        ASSER_TRAJ_InitialisationCourbeBS_5(g_tableauPoints[iCurrentGPoint_n0], g_tableauPoints[(iCurrentGPoint_n0 + 1)], deltaI, deltaF, qI, qF, &chemin.segmentTrajBS[iCurrentGPoint_n0]);
                        chemin.segmentTrajBS[iCurrentGPoint_n0].ordre = 5;
                    }
                    else
                    {
                        cordeFinale = sqrt(SQUARE(g_tableauPoints[iCurrentGPoint_n1].x - g_tableauPoints[(iCurrentGPoint_n1 - 1)].x) + SQUARE(g_tableauPoints[iCurrentGPoint_n1].y - g_tableauPoints[(iCurrentGPoint_n1 - 1)].y));
                        deltaF.x =  (cordeFinale * 1.0) * cos(angleFinal);
                        deltaF.y =  (cordeFinale * 1.0) * sin(angleFinal);

                        /* Redefinition du premier segment */
                        diff1BS.x = (chemin.segmentTrajBS[iCurrentGPoint_n0].mx[1] * (g_tableauH[iCurrentGPoint_n0].x) / 2.0) + chemin.segmentTrajBS[iCurrentGPoint_n0].ax;
                        diff1BS.y = (chemin.segmentTrajBS[iCurrentGPoint_n0].my[1] * (g_tableauH[iCurrentGPoint_n0].y) / 2.0) + chemin.segmentTrajBS[iCurrentGPoint_n0].ay;
                        diff1BS.x = diff1BS.x * g_tableauH[iCurrentGPoint_n0].x;
                        diff1BS.y = diff1BS.y * g_tableauH[iCurrentGPoint_n0].y;

                        g_tableauH_iCurrentGPoint_n0_x_sq = SQUARE(g_tableauH[iCurrentGPoint_n0].x);
                        g_tableauH_iCurrentGPoint_n0_y_sq = SQUARE(g_tableauH[iCurrentGPoint_n0].y);
                        qI.x = chemin.segmentTrajBS[iCurrentGPoint_n0].mx[0] * g_tableauH_iCurrentGPoint_n0_x_sq;
                        qI.y = chemin.segmentTrajBS[iCurrentGPoint_n0].my[0] * g_tableauH_iCurrentGPoint_n0_y_sq;
                        qF.x = chemin.segmentTrajBS[iCurrentGPoint_n0].mx[1] * g_tableauH_iCurrentGPoint_n0_x_sq;
                        qF.y = chemin.segmentTrajBS[iCurrentGPoint_n0].my[1] * g_tableauH_iCurrentGPoint_n0_y_sq;

                        ASSER_TRAJ_InitialisationCourbeBS_5(g_tableauPoints[iCurrentGPoint_n0], g_tableauPoints[(iCurrentGPoint_n0 + 1)], deltaI, diff1BS, qI, qF, &chemin.segmentTrajBS[iCurrentGPoint_n0]);
                        chemin.segmentTrajBS[iCurrentGPoint_n0].ordre = 5;

                        /* Redefinition du dernier segment en BS d'ordre 5 */
                        diff1BS.x = -(chemin.segmentTrajBS[(iCurrentGPoint_n1 - 1)].mx[0] * (g_tableauH[(iCurrentGPoint_n1 - 1)].x) / 2.0) + chemin.segmentTrajBS[(iCurrentGPoint_n1 - 1)].ax;
                        diff1BS.y = -(chemin.segmentTrajBS[(iCurrentGPoint_n1 - 1)].my[0] * (g_tableauH[(iCurrentGPoint_n1 - 1)].y) / 2.0) + chemin.segmentTrajBS[(iCurrentGPoint_n1 - 1)].ay;
                        diff1BS.x = diff1BS.x * g_tableauH[(iCurrentGPoint_n1 - 1)].x;
                        diff1BS.y = diff1BS.y * g_tableauH[(iCurrentGPoint_n1 - 1)].y;

                        g_tableauH_iCurrentGPoint_n1_m1_x_sq = SQUARE(g_tableauH[(iCurrentGPoint_n1 - 1)].x);
                        g_tableauH_iCurrentGPoint_n1_m1_y_sq = SQUARE(g_tableauH[(iCurrentGPoint_n1 - 1)].y);
                        qI.x = chemin.segmentTrajBS[(iCurrentGPoint_n1 - 1)].mx[0] * g_tableauH_iCurrentGPoint_n1_m1_x_sq;
                        qI.y = chemin.segmentTrajBS[(iCurrentGPoint_n1 - 1)].my[0] * g_tableauH_iCurrentGPoint_n1_m1_y_sq;
                        qF.x = chemin.segmentTrajBS[(iCurrentGPoint_n1 - 1)].mx[1] * g_tableauH_iCurrentGPoint_n1_m1_x_sq;
                        qF.y = chemin.segmentTrajBS[(iCurrentGPoint_n1 - 1)].my[1] * g_tableauH_iCurrentGPoint_n1_m1_y_sq;

                        ASSER_TRAJ_InitialisationCourbeBS_5(g_tableauPoints[(iCurrentGPoint_n1 - 1)], g_tableauPoints[iCurrentGPoint_n1], diff1BS, deltaF, qI, qF, &chemin.segmentTrajBS[(iCurrentGPoint_n1 - 1)]);
                        chemin.segmentTrajBS[(iCurrentGPoint_n1 - 1)].ordre = 5;
                    }

                    iCurrentGPoint_n0 = iCurrentGPoint_n1;
                    angleInitial = angleFinal;
                }
                
                /*** Test ***/
                /* ASSER_TRAJ_InitialisationDeriv2(iCurrentGPoint_n1, qI); */
                /************/

                iCurrentGPoint_n1 ++;
            }
        }

#ifndef PIC32_BUILD

        /* Enregistrement pour affichage de x(t) et de y(t) */

        sommeMask = 0;
        iSegment = 0;
        
        poseTest = ASSER_TRAJ_TrajectoireBSpline(&chemin.segmentTrajBS[iSegment], 0.0);
        poseTest = ASSER_TRAJ_TrajectoireBSpline(&chemin.segmentTrajBS[iSegment], 1.0);
        
        while (iSegment < (nbrePts))
        {
            for (intT = 0; intT < 100; intT++)
            {
                if ( (((float)intT / 100.0) > chemin.segmentTrajBS[iSegment].t[1]) && (iSegment < (nbrePts - 1)) )
                {
                    iSegment++;
                    poseTest = ASSER_TRAJ_TrajectoireBSpline(&chemin.segmentTrajBS[iSegment], 0.0);
                    poseTest = ASSER_TRAJ_TrajectoireBSpline(&chemin.segmentTrajBS[iSegment], 1.0);
                }
                
                poseTest = ASSER_TRAJ_TrajectoireBSpline(&chemin.segmentTrajBS[iSegment], ((float)intT / 100.0));
                
//                ASSER_TRAJ_LogAsser("tCourbeBS", (sommeMask + ((float)intT / 100.0)));
//                ASSER_TRAJ_LogAsser("xCourbeBS", poseTest.x);
//                ASSER_TRAJ_LogAsser("yCourbeBS", poseTest.y);
            }
            
            iSegment++;

            if (iSegment < nbrePts)
            {
                poseTest = ASSER_TRAJ_TrajectoireBSpline(&chemin.segmentTrajBS[iSegment], 0.0);
                poseTest = ASSER_TRAJ_TrajectoireBSpline(&chemin.segmentTrajBS[iSegment], 1.0);
            }
            
            sommeMask = sommeMask + 1.0;
        }

#endif /*PIC32_BUILD*/

        /* Determination des distances des segments de trajectoire, et de la distance totale */
        chemin.distance = 0.0;

        for (iSegment = 0; iSegment < nbrePts; iSegment++)
        {
            ASSER_TRAJ_DistanceTrajectoire(&chemin.segmentTrajBS[iSegment]);
            chemin.distance = chemin.distance + chemin.segmentTrajBS[iSegment].distance;
//            ASSER_TRAJ_LogAsser("log_distance", chemin.segmentTrajBS[iSegment].distance);
        }

        /* Calcul de la premiere pose de la trajectoire de consigne */
        diff1BS = ASSER_TRAJ_DiffCourbeBSpline(&chemin.segmentTrajBS[0], 0.0);
        diff2BS = ASSER_TRAJ_Diff2CourbeBSpline(&chemin.segmentTrajBS[0], 0.0);
        chemin.profilVitesse.diffThetaCourant = ASSER_TRAJ_DiffThetaBSpline(diff1BS, diff2BS);

        poseReference = ASSER_TRAJ_TrajectoireRemorqueBS(&chemin.segmentTrajBS[0], 0.0, diff1BS, chemin.profilVitesse.diffThetaCourant, &poseReferenceRobot);

    }
    else
    {
        /* Temps d'acceleration / decceration reduit pour la rotation */
        tempsDecc = tempsDecc * 2.0;

        /* Position a atteindre (condition d'arret du test de fin d'asservissment) */
        chemin.posArrivee.x = poseRobot.x + NORME_BARRE_SUIVI_TRAJ * cos(point[0].pose.angle);
        chemin.posArrivee.y = poseRobot.y + NORME_BARRE_SUIVI_TRAJ * sin(point[0].pose.angle);

        /* Configuration du profil de vitesse */
        chemin.profilVitesse.vmax = POS_GetConsVitesseAngulaireMax() * NORME_BARRE_SUIVI_TRAJ;

        chemin.rotation.poseDepartRobot = poseRobot;
        //chemin.segmentTraj[0].posDepart.x = poseRobot.x + (NORME_BARRE_SUIVI_TRAJ * cos(poseRobot.angle));
        //chemin.segmentTraj[0].posDepart.y = poseRobot.y + (NORME_BARRE_SUIVI_TRAJ * sin(poseRobot.angle));
        chemin.nbreSegments = 1;
        chemin.rotation.angle = POS_ModuloAngle(point[0].pose.angle - poseRobot.angle);

        /* Calcul du point d'arrivee de la trajectoire de consigne */
        //poseArriveeTemp = ASSER_TRAJ_TrajectoireRotation(&(chemin.rotation), 1.0);
        //chemin.segmentTraj[0].posArrivee.x = poseArriveeTemp.x;
        //chemin.segmentTraj[0].posArrivee.y = poseArriveeTemp.y;

        /* Calcul de la premiere pose de la trajectoire de consigne */
        chemin.segmentTrajBS[0].t[0] = 0.0;
        chemin.segmentTrajBS[0].t[1] = 1.0;
        poseReference = ASSER_TRAJ_TrajectoireRotation(&(chemin.rotation), 0.0);

        chemin.distance = (fabs(chemin.rotation.angle) * NORME_BARRE_SUIVI_TRAJ) + 0.000001;
    }

    /* Calcul du gabarit du profil de vitesse */
    g_poseRobotTrajectoire.x = poseRobot.x;
    g_poseRobotTrajectoire.y = poseRobot.y;
    g_poseRobotTrajectoire.angle = poseRobot.angle;
    chemin.profilVitesse.distance_parcourue = 0.0;
    
    /* Determination des coefficients a des profils de vitesse d'acceleration et de decceleration */
    chemin.profilVitesse.vitesse_courante = vitesse_profil_consigne;

    chemin.profilVitesse.p = 1;                                                                                 /* autorisation du profil de vitesse */
    chemin.profilVitesse.distNormaliseeRestante = 1.0;                                                          /* -> la distance totale normalisee */ 
    
    /* Initialisation du profil de vitessee */
    chemin.profilVitesse.pas_echantillon_distance = (chemin.distance * 1000.0) / ((float)(TAILLE_TAB_GABARIT_VITESSE - 1));
    chemin.profilVitesse.pas_echantillon_distance = (floor(chemin.profilVitesse.pas_echantillon_distance) + 1.0) / 1000.0;

    ASSER_TRAJ_LogAsser("pas_ech", NBR_ASSER_LOG_VALUE, chemin.profilVitesse.pas_echantillon_distance);
    g_index_tab_gabarit_vitesse = 0;

    /* Initialisation du profil de vitesse 2012 */
    chemin.profilVitesse.Amax = A_MAX * RATIO_ACC;
    chemin.profilVitesse.Dmax = D_MAX * RATIO_DECC;
    chemin.profilVitesse.vitesse_seuil_decc_finale = VITESSE_SEUIL_DECC;
    chemin.profilVitesse.coeff_decc_finale = COEFF_DECC_FINALE;
    chemin.profilVitesse.decc_min_finale = DECC_MIN;
    chemin.profilVitesse.coeff_vi1 = COEFF_VI1;

    ASSER_TRAJ_GabaritVitesse(&chemin);
    chemin.profilVitesse.etat = 1;

    compteurPeriode = 0;

    /* Initialisation de l'erreur de distance avant l'arrivee */
    errDist = POS_ErreurDistance(poseRobot, chemin.posArrivee);
    errAngle = POS_ErreurOrientation(poseRobot, chemin.posArrivee);
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_InitialisationCourbeBS_5
 * 
 *  \note   affectation des coefficients de B-Spline d'ordre 5 d'un segment de trajectoire,
 *             etant donnees les conditions aux limites du segment de trajectoire que sont les parametres d'entree.
 *
 *  Conditions aux limites au debut du segment
 *  \param [in]     ptI                 position
 *  \param [in]     deltaPtI          derivee premiere de la position (fixe l'angle de la tangente a la trajectoire)
 *  \param [in]     qI                  derivee seconde de la position (fixe la vitesse de variation de l'angle de la tangente a  la trajectoire)
 *
 *  Conditions aux limites a la fin du segment
 *  \param [in]     ptF                 position
 *  \param [in]     deltaPtF          derivee premiere de la position (fixe l'angle de la tangente a la trajectoire)
 *  \param [in]     qF                  derivee seconde de la position (fixe la vitesse de variation de l'angle de la tangente a  la trajectoire)
 *  \param [in]     segmentTraj
 *
 *  \return           None
 */
/**********************************************************************/
static void ASSER_TRAJ_InitialisationCourbeBS_5(Vecteur ptI, Vecteur ptF, Vecteur deltaPtI, Vecteur deltaPtF, Vecteur qI, Vecteur qF, segmentTrajectoireBS * segmentTraj)
{
    segmentTraj->bx = ptI.x - (qI.x / 20.0);
    segmentTraj->by = ptI.y - (qI.y / 20.0);

    segmentTraj->ax = deltaPtI.x + (qI.x / 4.0);
    segmentTraj->ay = deltaPtI.y + (qI.y / 4.0);

    segmentTraj->qx[0] = qI.x;
    segmentTraj->qx[1] = qF.x;
    segmentTraj->qy[0] = qI.y;
    segmentTraj->qy[1] = qF.y;

    segmentTraj->mx[0] = (60.0 * (ptF.x - segmentTraj->bx)) - (36.0 * segmentTraj->ax) - (24.0 * deltaPtF.x) + (3.0 * qF.x);
    segmentTraj->my[0] = (60.0 * (ptF.y - segmentTraj->by)) - (36.0 * segmentTraj->ay) - (24.0 * deltaPtF.y) + (3.0 * qF.y);

    segmentTraj->mx[1] = (12.0 * (deltaPtF.x - segmentTraj->ax)) - (3.0 * qF.x) - segmentTraj->mx[0];
    segmentTraj->my[1] = (12.0 * (deltaPtF.y - segmentTraj->ay)) - (3.0 * qF.y) - segmentTraj->my[0];
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_InitialisationMatriceInterpolation
 * 
 *  \note   Affectation de la matrice carre intervenant dans la determination des fonctions de la trajectoire selon des interpolations
 *
 *  \param [in]     iPtI         position
 *  \param [in]     iPtF         position
 *
 *  \return            None
 */
/**********************************************************************/
static void ASSER_TRAJ_InitialisationMatriceInterpolation(unsigned int iPtI, unsigned int iPtF)
{
    unsigned int i, j;
    unsigned int tailleMatrice;

    tailleMatrice = iPtF - iPtI - 1;

    /* RAZ de toute la matrice */
    for (i = 0; i < tailleMatrice; i++)
    {
        for (j = 0; j < (tailleMatrice * 2); j++)
        {
            g_matriceT[i][j] = 0.0;
        }
    }
    
    g_matriceT[0][0] = 2.0 * (g_tableauH[iPtI].x + g_tableauH[(iPtI + 1)].x);   /* Premier element de la diagonale de la matrice a inverser */
    g_matriceT[0][tailleMatrice] = 1.0;                                          /* Premier element de la diagonale de la matrice identite */
    
    for (i = (iPtI + 1); i < tailleMatrice; i++)
    {
        g_matriceT[i][i] = 2.0 * (g_tableauH[i].x + g_tableauH[(i + 1)].x);
        g_matriceT[(i - 1)][i] = g_tableauH[i].x;
        g_matriceT[i][(i - 1)] = g_tableauH[i].x;

        /* Construction de la matrice identite associee */
        g_matriceT[i][(tailleMatrice + i)] = 1.0;
    }
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_InversionMatriceT
 * 
 *  \note   Inversion de la matrice g_matriceT
 *
 *  \param [in]    nbrePts     nombre de points de la trajectoire pour la definition de la taille de la matrice
 *
 *  \return           None
 */
/**********************************************************************/
static void ASSER_TRAJ_InversionMatriceT(unsigned int nbrePts)
{
    int     inversible                      = 1;
    int     k, i,t, colonne, colonnebis;
    float   var, var1;

    t = nbrePts;
    k = 0;
    
    while((inversible == 1) && (k < t))
    {
        if (g_matriceT[k][k] != 0.0)
        {
            var = g_matriceT[k][k];
            
            for (colonne = 0; colonne < (2 * t); colonne++)
            {
                g_matriceT[k][colonne] = g_matriceT[k][colonne] / var;  /* Normalisation de la ligne contenant l'element diagonal */
            }
            
            for (i = 0; i < t; i++)
            {
                if (i != k)
                {
                    var1 = g_matriceT[i][k];
                    
                    for (colonnebis = 0; colonnebis < (2 * t); colonnebis++)
                    {
                        g_matriceT[i][colonnebis] = g_matriceT[i][colonnebis] - (g_matriceT[k][colonnebis] * var1);
                    }
                }
            }
            
            k++;
        }
        else
        {
            inversible = 0;
//            ASSER_TRAJ_LogAsser("inversionMatrice", 9999.0);
        }
    }
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_InterpolationBSpline3
 *
 *  \note   Determination des coefficients des B-Splines d'ordre 3 qui definissent chaque segment de trajectoire entre deux points imposes
 *
 *  \param [in]     iPtI         indice du premier point d'une portion de trajectoire pour la definition de la taille de la matrice
 *  \param [in]     iPtF         indice du dernier point d'une portion de trajectoire pour la definition de la taille de la matrice
 *
 *  \return            None
 */
/**********************************************************************/
static void ASSER_TRAJ_InterpolationBSpline3(unsigned int iPtI, unsigned int iPtF)
{
    float           distanceH       = 0.0;
    unsigned int    j, iSegment;
   
    for (j = iPtI; j < iPtF; j++)
    {
        g_tableauH[j].x = sqrt(SQUARE(g_tableauPoints[(j + 1)].x - g_tableauPoints[j].x) + SQUARE(g_tableauPoints[(j + 1)].y - g_tableauPoints[j].y));
        g_tableauH[j].y = g_tableauH[j].x;
        distanceH += g_tableauH[j].x;
    }

    /* Determination des parametres t de la trajectoire de B-Spline a chaque point de g_tableauPoints entre iPtI et iPtF */
    chemin.segmentTrajBS[iPtI].t[0] = 0.0;
    
    for (j = iPtI; j < iPtF; j++)
    {
        g_tableauH[j].x = g_tableauH[j].x / distanceH;
        g_tableauH[j].y = g_tableauH[j].x;
        
        chemin.segmentTrajBS[j].t[1] = chemin.segmentTrajBS[j].t[0] + g_tableauH[j].x;
        
        if (j < (iPtF - 1))
        {
            chemin.segmentTrajBS[(j + 1)].t[0] = chemin.segmentTrajBS[j].t[1];
        }
    }

    /* Matrice pour la determination des coefficients m des polynomes d'interpolation */
    ASSER_TRAJ_InitialisationMatriceInterpolation(iPtI, iPtF);                      /* iPtF - iPtI - 1); */
    ASSER_TRAJ_InversionMatriceT(iPtF - iPtI - 1);

    /* Determination des m avec le produit matriciel T^-1 * F */
    /* Calcul de la taille de l'interval entre deux points du parametre (des courbes parametriques) t */
    
    /* Calcul de F */
    for (j = iPtI; j < (iPtF - 1); j++)
    {
        g_tableauF[j].x = 6.0 * (((g_tableauPoints[(j + 2)].x - g_tableauPoints[(j + 1)].x) / g_tableauH[(j + 1)].x) - ((g_tableauPoints[(j + 1)].x - g_tableauPoints[j].x) / g_tableauH[j].x));
        g_tableauF[j].y = 6.0 * (((g_tableauPoints[(j + 2)].y - g_tableauPoints[(j + 1)].y) / g_tableauH[(j + 1)].y) - ((g_tableauPoints[(j + 1)].y - g_tableauPoints[j].y) / g_tableauH[j].y));
    }

    /* Calcul du produit matriciel */
    for (iSegment = (iPtI + 1); iSegment < iPtF; iSegment++)
    {
        chemin.segmentTrajBS[iSegment].mx[0] = 0.0;
        chemin.segmentTrajBS[iSegment].my[0] = 0.0;
        
        for (j = iPtI; j < (iPtF - 1); j++)
        {
            chemin.segmentTrajBS[iSegment].mx[0] = chemin.segmentTrajBS[iSegment].mx[0] + (g_matriceT[(iSegment - iPtI - 1)][(iPtF - iPtI - 1 + j)] * g_tableauF[j].x);
            chemin.segmentTrajBS[iSegment].my[0] = chemin.segmentTrajBS[iSegment].my[0] + (g_matriceT[(iSegment - iPtI - 1)][(iPtF - iPtI - 1 + j)] * g_tableauF[j].y);
        }
        
        chemin.segmentTrajBS[(iSegment - 1)].mx[1] = chemin.segmentTrajBS[iSegment].mx[0];
        chemin.segmentTrajBS[(iSegment - 1)].my[1] = chemin.segmentTrajBS[iSegment].my[0];
    }
    
    /* Les m aux bornes a zero (-> spline naturelle) */
    chemin.segmentTrajBS[iPtI].mx[0] = 0.0;
    chemin.segmentTrajBS[(iPtF - 1)].mx[1] = 0.0;
    chemin.segmentTrajBS[iPtI].my[0] = 0.0;
    chemin.segmentTrajBS[(iPtF - 1)].my[1] = 0.0;

    /* Calcul des a et b */
    for (iSegment = iPtI; iSegment < iPtF; iSegment++)
    {
        /* Calcul des ax */
        chemin.segmentTrajBS[iSegment].ax = ((g_tableauPoints[(iSegment + 1)].x - g_tableauPoints[iSegment].x) / g_tableauH[iSegment].x) - ((chemin.segmentTrajBS[iSegment].mx[1] - (chemin.segmentTrajBS[iSegment].mx[0] * g_tableauH[iSegment].x)) / 6.0);
        /* Calcul des bx */
        chemin.segmentTrajBS[iSegment].bx = g_tableauPoints[iSegment].x  - ((chemin.segmentTrajBS[iSegment].mx[0] * g_tableauH[iSegment].x * g_tableauH[iSegment].x) / 6.0);

        /* Calcul des ay */
        chemin.segmentTrajBS[iSegment].ay = ((g_tableauPoints[(iSegment + 1)].y - g_tableauPoints[iSegment].y) / g_tableauH[iSegment].y) - ((chemin.segmentTrajBS[iSegment].my[1] - (chemin.segmentTrajBS[iSegment].my[0] * g_tableauH[iSegment].y)) / 6.0);
        /* Calcul des by */
        chemin.segmentTrajBS[iSegment].by = g_tableauPoints[iSegment].y  - ((chemin.segmentTrajBS[iSegment].my[0] * g_tableauH[iSegment].y * g_tableauH[iSegment].y) / 6.0);

        chemin.segmentTrajBS[iSegment].i = iSegment;

        /* Ordre polynomial de la courbe B-Spline du segment */
        chemin.segmentTrajBS[iSegment].ordre = 3;
    }
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_TrajectoireRemorqueBS
 * 
 *  \note   exploitation d'une trajectoire desiree pour un asservissement par retour d'etat, trajectoire B-spline
 *
 *  \param [in]     segmentTraj          pointeur de structure definissant la trajectoire
 *  \param [in]     t                           parametre de la courbe de Bezier [0; 1]: 0-> depart, 1-> arrivee.
 *  \param [in]     diff1BS
 *  \param [in]     diffThetaBSRobot
 *  \param [in]     poseTrajRobot
 *
 *  \return            Pose                     pose de la courbe a laquelle doit etre asservi le point deporte du robot en fonction de t
 */
/**********************************************************************/
static Pose ASSER_TRAJ_TrajectoireRemorqueBS(segmentTrajectoireBS * segmentTraj, float t, Vecteur diff1BS, float diffThetaBSRobot, Pose * poseTrajRobot)
{   
    Pose    poseTraj;

    poseTraj = ASSER_TRAJ_TrajectoireBSpline(segmentTraj, t);
    *poseTrajRobot = poseTraj;

    poseTraj.x = poseTraj.x + (NORME_BARRE_SUIVI_TRAJ * cos(poseTraj.angle));
    poseTraj.y = poseTraj.y + (NORME_BARRE_SUIVI_TRAJ * sin(poseTraj.angle));
    poseTraj.angle = atan2((diff1BS.y + (NORME_BARRE_SUIVI_TRAJ * diffThetaBSRobot * cos(poseTraj.angle))), (diff1BS.x - (NORME_BARRE_SUIVI_TRAJ * diffThetaBSRobot * sin(poseTraj.angle))));

    ASSER_TRAJ_LogAsser("refX", NBR_ASSER_LOG_VALUE, poseTraj.x);
    
    return poseTraj;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_Trajectoire
 * 
 *  \note   equation parametrique de type B-Spline
 *
 *  \param [in]    t        parametre de la courbe: 0-> depart, 1-> arrivee.
 *
 *  \return          Pose   pose de la courbe fonction de l'argument t
 */
/**********************************************************************/
extern Pose ASSER_TRAJ_Trajectoire(float t)
{
    unsigned int    iSegment = 0;

    while ((iSegment < chemin.nbreSegments) && (t > chemin.segmentTrajBS[iSegment].t[1]))
    {
        iSegment++;
    }
    
    return (ASSER_TRAJ_TrajectoireBSpline(&chemin.segmentTrajBS[iSegment], t));
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_TrajectoireBSpline
 * 
 *  \note   Expression parametrique de type B-Spline de la position
 *
 *  \param [in]     segmentTraj  pointeur de structure definissant la trajectoire
 *  \param [in]     t                   parametre de la courbe de Bezier [0; 1]: 0-> depart, 1-> arrivee.
 *
 *  \return           Pose              pose de la courbe de Bezier fonction de l'argument t
 */
/**********************************************************************/
static Pose ASSER_TRAJ_TrajectoireBSpline(segmentTrajectoireBS * segmentTraj, float t)
{   
    float   theta_P, tl;
    Vecteur derivPositionTraj;
    Pose    poseTraj;
    /* var temporaire de resultats de calcul de puissance */
    float   t_m_t0_3, t1_m_t_3;
    float   one_m_tl_5, tl_3, tl_4, tl_5;

    if (segmentTraj->ordre == (unsigned int)3)
    {   
        /* Segment defini comme un polynome d'ordre 3 */
        /* Calcul des coordonnees de la courbe parametrique  (-> equations cubiques) au parametre t */
        t_m_t0_3 = CUBE(t - segmentTraj->t[0]);
        t1_m_t_3 = CUBE(segmentTraj->t[1] - t);
        poseTraj.x = ((segmentTraj->mx[1] * t_m_t0_3) / (6.0 * g_tableauH[segmentTraj->i].x))  + ((segmentTraj->mx[0] * t1_m_t_3) / (6.0 * g_tableauH[segmentTraj->i].x)) + (segmentTraj->ax * (t - segmentTraj->t[0])) + segmentTraj->bx;
        poseTraj.y = ((segmentTraj->my[1] * t_m_t0_3) / (6.0 * g_tableauH[segmentTraj->i].x))  + ((segmentTraj->my[0] * t1_m_t_3) / (6.0 * g_tableauH[segmentTraj->i].x)) + (segmentTraj->ay * (t - segmentTraj->t[0])) + segmentTraj->by;
    }
    else /* ordre == 5, segment defini comme un polynome d'ordre 5 */
    {
        /* Changement de variable sur t */
        tl = (t - segmentTraj->t[0]) / g_tableauH[segmentTraj->i].x;
        one_m_tl_5 = POWER5(1.0 - tl);
        tl_3 = CUBE(tl);
        tl_4 = POWER4(tl);
        tl_5 = POWER5(tl);
        poseTraj.x = ((segmentTraj->qx[0] / 20.0) * one_m_tl_5) + (segmentTraj->mx[0] * ((tl_3 / 6.0) - (tl_4 / 6.0) + (tl_5 / 20.0))) + (segmentTraj->mx[1] * ((tl_4 / 12.0) - (tl_5 / 20.0)) + ((segmentTraj->qx[1] * tl_5) / 20.0)) + (segmentTraj->ax * tl) + segmentTraj->bx;
        poseTraj.y = ((segmentTraj->qy[0] / 20.0) * one_m_tl_5) + (segmentTraj->my[0] * ((tl_3 / 6.0) - (tl_4 / 6.0) + (tl_5 / 20.0))) + (segmentTraj->my[1] * ((tl_4 / 12.0) - (tl_5 / 20.0)) + ((segmentTraj->qy[1] * tl_5) / 20.0)) + (segmentTraj->ay * tl) + segmentTraj->by;
    }

    /* Calcul de l'orientation de la trajectoire a t */
    derivPositionTraj = ASSER_TRAJ_DiffCourbeBSpline(segmentTraj, t);
    theta_P = atan2(derivPositionTraj.y, derivPositionTraj.x);
    poseTraj.angle = POS_ModuloAngle(theta_P);
    
    return poseTraj;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_DiffCourbeBSpline
 * 
 *  \note   Expression parametrique de la derivee premiere de l'expression parametrique de type B-Spline de la position
 *
 *  \param [in]     segmentTraj     pointeur de structure definissant un segment de trajectoire
 *  \param [in]     t                      parametre de la courbe parametrique [0; 1]: 0-> pose de depart, 1-> pose d'arrivee.
 *
 *  \return           derivP              derivee des coordonnees de la position sur la courbe de B-Spline fonction de t,
 *                                              par rapport au parametre t.
 */
/**********************************************************************/
static Vecteur ASSER_TRAJ_DiffCourbeBSpline(segmentTrajectoireBS * segmentTraj, float t)
{
    Vecteur derivP;
    float   tl;
    /* var temporaire de resultats de calcul de puissance */
    float   t_m_t0_2, t1_m_t_2;
    float   one_m_tl_4, tl_2, tl_3, tl_4;
    
    if (segmentTraj->ordre == (unsigned int)3)
    {   
        /* Segment defini comme un polynome d'ordre 3 */
        /* Derivee generique des composantes de la trajectoire par rapport a t */
        t_m_t0_2 = SQUARE(t - segmentTraj->t[0]);
        t1_m_t_2 = SQUARE(segmentTraj->t[1] - t);
        derivP.x = ((segmentTraj->mx[1] * t_m_t0_2) / (2.0 * g_tableauH[segmentTraj->i].x)) - ((segmentTraj->mx[0] * t1_m_t_2) / (2.0 * g_tableauH[segmentTraj->i].x)) + segmentTraj->ax;
        derivP.y = ((segmentTraj->my[1] * t_m_t0_2) / (2.0 * g_tableauH[segmentTraj->i].y)) - ((segmentTraj->my[0] * t1_m_t_2) / (2.0 * g_tableauH[segmentTraj->i].y)) + segmentTraj->ay;
    }
    else /* Ordre == 5, segment defini comme un polynome d'ordre 5 */
    {
        /* Changement de variable sur t */
        tl = (t - segmentTraj->t[0]) / g_tableauH[segmentTraj->i].x;
        one_m_tl_4 = POWER4(1.0 - tl);
        tl_2 = SQUARE(tl);
        tl_3 = CUBE(tl);
        tl_4 = POWER4(tl);

        derivP.x = (-(segmentTraj->qx[0] / 4.0) * one_m_tl_4) + (segmentTraj->mx[0] * ((tl_2 / 2.0) - ((2.0 * tl_3) / 3.0) + (tl_4 / 4.0))) + (segmentTraj->mx[1] * ((tl_3/ 3.0) - (tl_4 / 4.0))) + (segmentTraj->qx[1] * (tl_4 / 4.0)) + segmentTraj->ax;
        derivP.y = (-(segmentTraj->qy[0] / 4.0) * one_m_tl_4) + (segmentTraj->my[0] * ((tl_2 / 2.0) - ((2.0 * tl_3) / 3.0) + (tl_4 / 4.0))) + (segmentTraj->my[1] * ((tl_3/ 3.0) - (tl_4 / 4.0))) + (segmentTraj->qy[1] * (tl_4 / 4.0)) + segmentTraj->ay;

        derivP.x = derivP.x / g_tableauH[segmentTraj->i].x;
        derivP.y = derivP.y / g_tableauH[segmentTraj->i].y;
    }

    return derivP;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_Diff2CourbeBSpline
 * 
 *  \note   Expression parametrique de la derivee seconde de l'expression parametrique de type B-Spline de la position
 *
 *  \param [in]     segmentTraj  pointeur de structure definissant un segment de trajectoire
 *  \param [in]     t                   parametre de la courbe parametrique [0; 1]: 0-> pose de depart, 1-> pose d'arrivee.
 *
 *  \return            Vecteur        derivee des coordonnees de la position sur la courbe de B-Spline fonction de t,
 *                                           par rapport au parametre t.
 */
/**********************************************************************/
static Vecteur ASSER_TRAJ_Diff2CourbeBSpline(segmentTrajectoireBS * segmentTraj, float t)
{
    Vecteur deriv2P;
    float   tl;
    /* var temporaire de resultats de calcul de puissance */
    float   one_m_tl_3, one_m_tl_2, tl_2, tl_3;
    
    if (segmentTraj->ordre == 3)
    {   
        /* Segment defini comme un polynome d'ordre 3 */
        /* Derivee generique des composantes de la trajectoire par rapport a t */
        deriv2P.x = ((segmentTraj->mx[1] * (t - segmentTraj->t[0])) / g_tableauH[segmentTraj->i].x) + ((segmentTraj->mx[0] * (segmentTraj->t[1] - t)) / g_tableauH[segmentTraj->i].x);
        deriv2P.y = ((segmentTraj->my[1] * (t - segmentTraj->t[0])) / g_tableauH[segmentTraj->i].y) + ((segmentTraj->my[0] * (segmentTraj->t[1] - t)) / g_tableauH[segmentTraj->i].y);
    }
    else /* Ordre == 5, segment defini comme un polynome d'ordre 5 */
    {
        /* Changement de variable sur t */
        tl = (t - segmentTraj->t[0]) / g_tableauH[segmentTraj->i].x;
        one_m_tl_3 = CUBE(1.0 - tl);
        one_m_tl_2 = SQUARE(1.0 - tl);
        tl_2 = SQUARE(tl);
        tl_3 = CUBE(tl);

        deriv2P.x = (segmentTraj->qx[0] * one_m_tl_3) + (segmentTraj->mx[0] * tl * one_m_tl_2 + segmentTraj->mx[1] * tl_2 * (1.0 - tl)) + (segmentTraj->qx[1] * tl_3);
        deriv2P.y = (segmentTraj->qy[0] * one_m_tl_3) + (segmentTraj->my[0] * tl * one_m_tl_2 + segmentTraj->my[1] * tl_2 * (1.0 - tl)) + (segmentTraj->qy[1] * tl_3);

        deriv2P.x = deriv2P.x / SQUARE(g_tableauH[segmentTraj->i].x);
        deriv2P.y = deriv2P.y / SQUARE(g_tableauH[segmentTraj->i].y);
    }

    return deriv2P;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_DiffThetaBSpline
 * 
 *  \note   Expression parametrique de la derivee premiere de l'angle de la tangente a la trajectoire
 *
 *  \param [in]     diff1BS     
 *  \param [in]     diff2BS     
 *
 *  \return            diffTheta
 */
/**********************************************************************/
static float ASSER_TRAJ_DiffThetaBSpline(Vecteur diff1BS, Vecteur diff2BS)
{
    float diffTheta;
    
    diffTheta = ((diff2BS.y * diff1BS.x) - (diff1BS.y * diff2BS.x)) / (SQUARE(diff1BS.x) * (1.0 + SQUARE(diff1BS.y / diff1BS.x)));

    return diffTheta;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_TrajectoireRotation
 * 
 *  \note   Expression parametrique de la position a suivre pour realiser l'asservissement d'une pure rotation
 *
 *  \param [in]     p_rotation      pointeur de structure definissant la trajectoire
 *  \param [in]     t                   parametre de parcours de la trajectoire en arc de cercle [0; 1]: 0-> depart, 1-> arrivee.
 *
 *  \return           Pose              pose de la trajectoire fonction de l'argument t
 */
/**********************************************************************/

static Pose ASSER_TRAJ_TrajectoireRotation(ParametresRotation *p_rotation, float t)
{
    Pose poseTraj;

    poseTraj.angle = p_rotation->poseDepartRobot.angle + (p_rotation->angle * t);
    poseTraj.angle = POS_ModuloAngle(poseTraj.angle);
    poseTraj.x = p_rotation->poseDepartRobot.x + (NORME_BARRE_SUIVI_TRAJ * cos(poseTraj.angle));
    poseTraj.y = p_rotation->poseDepartRobot.y + (NORME_BARRE_SUIVI_TRAJ * sin(poseTraj.angle));
    
    return poseTraj;
}

#if 0

/**********************************************************************/
/*! \brief ASSER_TRAJ_VitesseLimiteEnVirage
 *
 *  \note   Determination de la vitesse longitudinale de consigne a appliquer en fonction de la courbure de la trajectoire
 *
 *  \param [in]     traj                     pointeur de structure definissant la trajectoire
 *  \param [in]     diffThetaTrajectoire     vitesse angulaire de la trajectoire
 *
 *  \return         vitesseLimite   vitesse en m/s
 */
/**********************************************************************/
static float ASSER_TRAJ_VitesseLimiteEnVirage(Trajectoire *traj, float diffThetaTrajectoire)
{
    float vitesseLimite = 0.0;

    vitesseLimite = traj->profilVitesse.vmax / (1.0 + (fabs(diffThetaTrajectoire) * facteurVitesseAngulaireMax));

    return(vitesseLimite);
}

#endif

/**********************************************************************/
/*! \brief ASSER_TRAJ_ParcoursTrajectoire
 *
 *  \note   Determination du gabarit de la vitesse longitudinale de consigne, dependant de la courbure de trajectoire
 *  et des fonctions d'acceleration et de decceleration.
 *
 *  \param [in]     traj                        pointeur de structure definissant la trajectoire
 *  \param [in]     delta_distance         distance de deplacement sur la periode en metre
 *  \param [in]     segmentCourant     
 *  \param [in]     paramPoseSegTraj    
 *
 *  \return            True or False
 */
/**********************************************************************/
static unsigned char ASSER_TRAJ_ParcoursTrajectoire(Trajectoire *traj, float delta_distance, unsigned int *segmentCourant, float *paramPoseSegTraj)
{
    unsigned int    memoSegmentCourant;
    Vecteur         diffD_T;
    float           facteurCorrectionT, delta_param_chemin;
    unsigned char   flag_trajectoireTerminee                = False;

    do
    {
        memoSegmentCourant = *segmentCourant;

        if (ASSER_TRAJ_isDeplacement(traj) == True)
        {
            /* Determination du ratio entre un deplacement en metre et un deplacement du parametre du chemin */
            diffD_T = ASSER_TRAJ_DiffCourbeBSpline(&traj->segmentTrajBS[*segmentCourant], *paramPoseSegTraj);
            facteurCorrectionT = sqrt(SQUARE(diffD_T.x) + SQUARE(diffD_T.y));

            /* Deplacement du parametre de pose du chemin */
            delta_param_chemin = delta_distance / facteurCorrectionT;
        }
        else /* if (chemin.mouvement == ROTATION) */
        {
            facteurCorrectionT = 1.0;
            delta_param_chemin = (delta_distance / traj->distance);
        }

        *paramPoseSegTraj += delta_param_chemin;

        if (*paramPoseSegTraj > traj->segmentTrajBS[*segmentCourant].t[1])
        {
            /* Passage au segment suivant */
            if (*segmentCourant < (traj->nbreSegments - 1))
            {
                /* part du deplacement du parametre du chemin au-dela du segment courant */
                *paramPoseSegTraj = *paramPoseSegTraj - traj->segmentTrajBS[*segmentCourant].t[1];

                /* part de la distance au-dela du segment courant */
                delta_distance = *paramPoseSegTraj / facteurCorrectionT;

                (*segmentCourant)++;
                *paramPoseSegTraj = traj->segmentTrajBS[*segmentCourant].t[0];
            }
            else
            {
                *paramPoseSegTraj = 1.0;
                flag_trajectoireTerminee = True;
            }
        }
    } while ((*segmentCourant != memoSegmentCourant) && (flag_trajectoireTerminee != True));

    return flag_trajectoireTerminee;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_GabaritVitesse
 *
 *  \note   Determination du gabarit de la vitesse longitudinale de consigne, dependant de la courbure de trajectoire
 *             et des fonctions d'acceleration et de decceleration.
 *
 *  \param [in]     traj            pointeur de structure definissant la trajectoire
 *
 *  \return           None
 */
/**********************************************************************/
static void ASSER_TRAJ_GabaritVitesse(Trajectoire * traj)
{
    unsigned int    numSegment              = 0;
    float           acc_courant             = 0.0;
    float           paramPosition           = 0.0;
    float           vitesse_consigne        = 0.0;
//    float           vitesse_limite          = 0.0;
    float           distanceRestante;
    unsigned int    flag_finTraj            = 0u;
    unsigned char   phase                   = 0u;
    unsigned char   flag_phase              = 0u;
    Vecteur         diff1BS, diff2BS;
    float           diffThetaTrajectoire    = 0.0;
    float           vitesse_consigne_gabarit= 0.0;
    float           vpointe                 = 0.0;
    unsigned int    i                       = 0u;
    unsigned int    N_plateau               = 0.0;
    float           coeff_vit_ini           = 0.4;

    ASSER_TRAJ_InitTabVPointe(traj, coeff_vit_ini);
    distanceRestante = traj->distance;
    ASSER_TRAJ_LogAsser("distanceRestante_FD", NBR_ASSER_LOG_VALUE, distanceRestante);

    for (i = 4; i >= 0; i--) // de la plus grande vitesse de pointe a la plus petite (de i=4 ‡ i=0)
    {
        if (g_tab_vpointe[i][0] < traj->distance)
        {
            vpointe = g_tab_vpointe[i][1];
            N_plateau = floor((traj->distance - g_tab_vpointe[i][0]) / traj->profilVitesse.pas_echantillon_distance) + 1;
            ASSER_TRAJ_LogAsser("init_gabarit", NBR_ASSER_LOG_VALUE, traj->distance);
            ASSER_TRAJ_LogAsser("init_gabarit", NBR_ASSER_LOG_VALUE, N_plateau);
            ASSER_TRAJ_LogAsser("init_gabarit", NBR_ASSER_LOG_VALUE, g_tab_vpointe[i][2]);
            break;
        }
    }

    /* Parcourir la trajectoire en l'echantillonnant sur N pas de distance pas_echantillon_distance */
    vitesse_consigne = coeff_vit_ini * vpointe;
    while (flag_finTraj == 0)
    {
        flag_finTraj = ASSER_TRAJ_ParcoursTrajectoire(traj, traj->profilVitesse.pas_echantillon_distance, &numSegment, &paramPosition);

        if (phase == 0)
        {
            flag_phase = ASSER_TRAJ_ProfilAcceleration_2012(traj, vpointe, &vitesse_consigne);

        }
        if (phase == 1)
        {
            //vitesse_consigne = vpointe;
            if (N_plateau > 0) 
            {
                N_plateau--;
            }
            if (N_plateau == 0)
            {
                flag_phase = 1;
            }
        }
        if (phase == 2)
        {
            flag_phase = ASSER_TRAJ_ProfilDecceleration_2012(traj, vpointe, &vitesse_consigne);

        }
        
        if (flag_phase == 1)
        {
            phase++;
            flag_phase = 0;
        }

        distanceRestante = distanceRestante - traj->profilVitesse.pas_echantillon_distance;
        ASSER_TRAJ_LogAsser("distanceRestante_FD", NBR_ASSER_LOG_VALUE, distanceRestante);

        diff1BS = ASSER_TRAJ_DiffCourbeBSpline(&traj->segmentTrajBS[numSegment], paramPosition);
        diff2BS = ASSER_TRAJ_Diff2CourbeBSpline(&traj->segmentTrajBS[numSegment], paramPosition);
        diffThetaTrajectoire = ASSER_TRAJ_DiffThetaBSpline(diff1BS, diff2BS);

        if (g_index_tab_gabarit_vitesse < TAILLE_TAB_GABARIT_VITESSE)
        {        
//            vitesse_limite = ASSER_TRAJ_VitesseLimiteEnVirage(traj, diffThetaTrajectoire);
//            if (vitesse_consigne > vitesse_limite)
//            {
//                vitesse_consigne_gabarit = vitesse_limite;
//            }
//            else
//            {
//                vitesse_consigne_gabarit = vitesse_consigne;
//            }
            if (vitesse_consigne < 0.05)
            {
                vitesse_consigne_gabarit = 0.05;
            }
            else
            {
                vitesse_consigne_gabarit = vitesse_consigne;
            }

            g_tab_gabarit_vitesse[g_index_tab_gabarit_vitesse] = vitesse_consigne_gabarit;
            ASSER_TRAJ_LogAsser("gabarit_vitesse", NBR_ASSER_LOG_VALUE, g_tab_gabarit_vitesse[g_index_tab_gabarit_vitesse]);
            g_tab_gabarit_acceleration[g_index_tab_gabarit_vitesse] = acc_courant;
            ASSER_TRAJ_LogAsser("gabarit_acceleration", NBR_ASSER_LOG_VALUE, g_tab_gabarit_acceleration[g_index_tab_gabarit_vitesse]);
            g_index_tab_gabarit_vitesse++;
        }

    }
    
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_InitTabVPointe
 *
 *  \note   Determination du gabarit de la vitesse longitudinale de consigne, dependant de la courbure de trajectoire
 *  et des fonctions d'acceleration et de decceleration.
 *
 *  \param [in]     traj            pointeur de structure definissant la trajectoire
 *
 *  \return None
 */
/**********************************************************************/
static void ASSER_TRAJ_InitTabVPointe(Trajectoire *traj, float coeff_vit_ini)
{   // tableau de 1: distance des phases d'acc et de decc reunies, 2: vitesse max(de pointe), 3; distance de decc
    unsigned int    i;
    float           vpointe;
    float           distanceAcc, distanceDecc;

    for(i = 0; i < 5; i++)
    {
        vpointe = 0.2 + i * 0.1;
        if (vpointe > traj->profilVitesse.vmax)
        {
            vpointe = traj->profilVitesse.vmax;
        }
        distanceAcc = ASSER_TRAJ_DistanceAcceleration(traj, vpointe, vpointe * coeff_vit_ini);
        distanceDecc = ASSER_TRAJ_DistanceDecceleration(traj, vpointe);
        g_tab_vpointe[i][0] = distanceAcc + distanceDecc;
        g_tab_vpointe[i][1] = vpointe;
        g_tab_vpointe[i][2] = distanceDecc;
    }
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_getGabaritVitesse_getVitesse_vs_Distance
 *
 *  \note   Determination du gabarit de la vitesse longitudinale de consigne, dependant de la courbure de trajectoire
 *             et des fonctions d'acceleration et de decceleration.
 *
 *  \param [in]     distance 
 *
 *  \return           vitesse
 */
/**********************************************************************/
extern float ASSER_TRAJ_GabaritVitesse_getVitesse_vs_Distance(float distance)
{
    unsigned int index;
    float vitesse, delta_distance;

    index = floor(distance / chemin.profilVitesse.pas_echantillon_distance);

    if (index > (TAILLE_TAB_GABARIT_VITESSE - 1))
    {
        index = (TAILLE_TAB_GABARIT_VITESSE - 1);
        vitesse = g_tab_gabarit_vitesse[index];
    }
    else
    {
        /* Interpolation de la vitesse de consigne */
        delta_distance = distance - ( ((float)index) * chemin.profilVitesse.pas_echantillon_distance );

        vitesse = g_tab_gabarit_vitesse[index] + (g_tab_gabarit_vitesse[(index + 1)] - g_tab_gabarit_vitesse[index]) * (delta_distance / chemin.profilVitesse.pas_echantillon_distance);
    }
    
    ASSER_TRAJ_LogAsser("index_get_vitesseGabarit", NBR_ASSER_LOG_VALUE, vitesse);

    return vitesse;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_ProfilAcceleration_2012
 *
 *  \note   Determination de la vitesse longitudinale de consigne en phase d'acceleration
 *
 *  \param [in]     traj             pointeur de structure definissant la trajectoire
 *  \param [in]     vpointe        vitesse de pointe du profil de vitesse en acceleration
 *  \param [in]     *vitesse_n   vitesse courante
 *
 *  \return            flag_fin        info de la fin de la phase d'acceleration
 */
/**********************************************************************/
static unsigned char ASSER_TRAJ_ProfilAcceleration_2012(Trajectoire * traj, float vpointe, float * vitesse_n)
{
    float           acc;
    float           dt;
    float           vitesse_np1;
    unsigned char   flag_fin     = False;

    acc = traj->profilVitesse.Amax - ((traj->profilVitesse.Amax / SQUARE(vpointe)) * SQUARE(*vitesse_n));
    vitesse_np1 = sqrt(SQUARE(*vitesse_n) + (2.0 * traj->profilVitesse.pas_echantillon_distance * acc));
    dt = (2.0 * traj->profilVitesse.pas_echantillon_distance) / (*vitesse_n + vitesse_np1);
    acc = (vitesse_np1 - *vitesse_n) / dt;

    if (vitesse_np1 > (vpointe - 0.005))
    {
        flag_fin = True;
    }
    
    *vitesse_n = vitesse_np1;
    
    return flag_fin;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_ProfilDecceleration_2012
 *
 *  \note   Determination de la vitesse longitudinale de consigne en phase de decceleration
 *
 *  \param [in]     traj              pointeur de structure definissant la trajectoire
 *  \param [in]     vpointe         vitesse de depart (et maximale) du profil de vitesse en decceleration
 *  \param [in]     vitesse_n      vitesse courante
 *
 *  \return            flag_fin        info de la fin de la phase de decceleration
 */
/**********************************************************************/
static unsigned char ASSER_TRAJ_ProfilDecceleration_2012(Trajectoire * traj, float vpointe, float * vitesse_n)
{
    float           vi1;
    float           decc, decc_limit, vitesse_np1;
//    float           dt;
    float           vcarre                  = 0.0;
    unsigned char   flag_fin                = False;

    vi1 = vpointe * traj->profilVitesse.coeff_vi1;
    if ( (*vitesse_n) > (vpointe - 0.005) )
    {
        (*vitesse_n) = vpointe - 0.005;
    }

    if ((*vitesse_n) < vi1)
    {
        decc = traj->profilVitesse.Dmax;
    }
    else
    {
        decc = traj->profilVitesse.Dmax * (1 - SQUARE( ((*vitesse_n) - vi1) / (vpointe - vi1) ) );
    }

    if ((*vitesse_n) < traj->profilVitesse.vitesse_seuil_decc_finale)
    {
        decc_limit = traj->profilVitesse.decc_min_finale - traj->profilVitesse.coeff_decc_finale * (*vitesse_n);
        if (decc  < decc_limit)
        {
            decc = decc_limit;
        }
    }

    vcarre = SQUARE((*vitesse_n)) + 2.0 * traj->profilVitesse.pas_echantillon_distance * decc;

    if (vcarre < 0.0)
    {
        vitesse_np1 = 0.0;
    }
    else
    {
        vitesse_np1 = sqrt(vcarre);
    }
/*
    dt = (2.0 * traj->profilVitesse.pas_echantillon_distance) / (*vitesse_n + vitesse_np1);
    decc = (vitesse_np1 - *vitesse_n) / dt;
*/
    if (vitesse_np1 < 0.001)
    {
        flag_fin = True;
    }
    
    *vitesse_n = vitesse_np1;

    return flag_fin;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_DistanceAcceleration
 *
 *  \note   Determination de la distance sur laquelle a lieu la phase d'acceleration
 *
 *  \param [in]     traj            pointeur de structure definissant la trajectoire
 *  \param [in]     vpointe         vitesse de pointe du profil de vitesse en acceleration
 *
 *  \return                         distance
 */
/**********************************************************************/
static float ASSER_TRAJ_DistanceAcceleration(Trajectoire * traj, float vpointe, float vit_ini)
{
    unsigned int    nbrePas = 0u;
    float           vitesse;
    unsigned char   flag_fin = False;

    vitesse = vit_ini;
    while( (flag_fin == False) && (nbrePas < 65535) )
    {
        flag_fin = ASSER_TRAJ_ProfilAcceleration_2012(traj, vpointe, &vitesse);
        nbrePas++;
    }

    return (nbrePas * traj->profilVitesse.pas_echantillon_distance);
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_DistanceDecceleration
 *
 *  \note   Determination de la distance sur laquelle a lieu la phase de deccelaration
 *
 *  \param [in]     traj            pointeur de structure definissant la trajectoire
 *  \param [in]     vpointe         vitesse de pointe du profil de vitesse en decceleration
 *
 *  \return                         distance
 */
/**********************************************************************/
static float ASSER_TRAJ_DistanceDecceleration(Trajectoire * traj, float vpointe)
{
    unsigned int    nbrePas = 0u;
    float           vitesse;
    unsigned char   flag_fin = False;

    vitesse = vpointe - 0.005;
    while( (flag_fin == False) && (nbrePas < 65535) )
    {
        flag_fin = ASSER_TRAJ_ProfilDecceleration_2012(traj, vpointe, &vitesse);
        nbrePas++;
    }

    return (nbrePas * traj->profilVitesse.pas_echantillon_distance);
}

#if 0

/**********************************************************************/
/*! \brief ASSER_TRAJ_ProfilConsigneAccelerationVitesse
 *
 *  \note   Determination de l'acceleration et de la vitesse longitudinale de consigne a appliquer
 *
 *  \param [in]     traj            pointeur de structure definissant la trajectoire
 *  \param [in]     paramTrajectoire    parametre de position de la position sur le segment de trajectoire courant
 *
 *  \return         vitFinale   vitesse en m/s
 */
/**********************************************************************/
static float ASSER_TRAJ_ProfilConsigneAccelerationVitesse(Trajectoire * traj, float distanceRestante, float * acc_courant, float * vitesse_consigne)
{

}

#endif

/**********************************************************************/
/*! \brief ASSER_TRAJ_DiffTemporelleTrajectoire
 * 
 *  \note              derivee temporelle discrete d'une pose
 *
 *  \param [in]     posePrecedente      pose a une periode i-1
 *  \param [in]     poseActuelle           pose a une periode i
 *  \param [in]     periode                  duree de la periode en seconde
 *
 *  \return           diffTemporellePose  Diff temporelle pose
 */
/**********************************************************************/
static Pose ASSER_TRAJ_DiffTemporelleTrajectoire(Pose posePrecedente, Pose poseActuelle, float periode)
{
    Pose diffTemporellePose;

    diffTemporellePose.x = (poseActuelle.x - posePrecedente.x) / periode;
    diffTemporellePose.y = (poseActuelle.y - posePrecedente.y) / periode;
    diffTemporellePose.angle = POS_ModuloAngle(poseActuelle.angle - posePrecedente.angle) / periode;

    return diffTemporellePose;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_MatriceRotation
 * 
 *  \note   affectation d'un tableau de deux vecteurs (considere comme une matrice 2x2), 
 *             par une matrice de passage de rotation
 *
 *  \param [in]     matrice2p2       pointeur d'un tableau de vecteurs
 *  \param [in]     angle               angle de la rotation en rd
 */
/**********************************************************************/
static void ASSER_TRAJ_MatriceRotation(Vecteur * matrice2p2, float angle)
{
    matrice2p2[0].x = cos(angle);
    matrice2p2[0].y = sin(angle);
    matrice2p2[1].x = - sin(angle);
    matrice2p2[1].y = cos(angle);
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_ProduitMatriceVecteur
 * 
 *  \note              produit matriciel entre une matrice 2x2 a gauche, et un vecteur a droite. 
 *                       Outil permettant un changement de repere suivant une rotation de la base du repere.
 *
 *  \param [in]     matrice     pointeur d'un tableau de vecteurs
 *  \param [in]     vecteur     coordonnees sur lesquelles est appliquee la matrice (une matrice de rotation initialisee par la fonction ASSER_TRAJ_MatriceRotation)
 *
 * \return             produit     nouvelles coordonnees
 */
/**********************************************************************/
static Vecteur ASSER_TRAJ_ProduitMatriceVecteur(Vecteur *matrice, Vecteur vecteur)
{
    Vecteur produit;
    
    produit.x = (matrice[0].x * vecteur.x) + (matrice[1].x * vecteur.y);
    produit.y = (matrice[0].y * vecteur.x) + (matrice[1].y * vecteur.y);

    return produit;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_ErreurPose
 * 
 *  \note   erreur de pose du robot par rapport a un point de reference, en mouvement 
 *             ou non, et selon le repere du point de reference
 *
 *  \param [in]  poseRobot_P    pose du robot
 *  \param [in]  poseRef        pose de reference
 *
 *  \return         erreurP
 */
/**********************************************************************/
static Pose ASSER_TRAJ_ErreurPose(Pose poseRobot_P, Pose poseRef)
{
    Vecteur matrice[2];
    Vecteur erreurPosition;
    Pose    erreurP;
    
    /* Erreur de position dans le repere absolu entre le point reference de la trajectoire et le point P deporte du robot */
    erreurPosition.x = poseRobot_P.x - poseRef.x;
    erreurPosition.y = poseRobot_P.y - poseRef.y;
    /* Affectation de la matrice par la matrice de rotation de moins l'angle de reference */
    ASSER_TRAJ_MatriceRotation(matrice, - poseRef.angle);
    erreurPosition = ASSER_TRAJ_ProduitMatriceVecteur(matrice, erreurPosition);

    erreurP.x = erreurPosition.x;
    erreurP.y = erreurPosition.y;
    /* Erreur d'orientation du robot par rapport a la tangente a la trajectoire a son point de reference */
    erreurP.angle = POS_ModuloAngle(poseRobot_P.angle - poseRef.angle);

    return erreurP;
}

 /**********************************************************************/
 /*! \brief ASSER_TRAJ_RetourDetat
  *
  *  \note  Loi de commande par retour d'etat du suivi de trajectoire
  *
  *  \param [in]     erreurPose              erreur calculee par la fonction ASSER_TRAJ_ErreurPose
  *  \param [in]     poseRef                  pose a laquelle on souhaite asservir la position du robot
  *  \param [in]     diffPoseRef              derivee temporelle de la pose de reference
  *  \param [in]     longueurBarreSuivi  distance entre le centre de l'axe des roues motrices/libres et le point du robot a asservir
  *  \param [in]     diagMatriceGain       gain matriciel du retour d'etat
  *
  *  \return            vitessesCons           Structure de vitesses avec les consignes de vitesse longitudinale (en m/s) et de vitesse de rotation (en rd/s)
  */
 /**********************************************************************/
 static VitessesRobotUnicycle ASSER_TRAJ_RetourDetat(Pose erreurPose, Pose poseRef, Pose diffPoseRef, float longueurBarreSuivi, float * diagMatriceGain)
 {
     VitessesRobotUnicycle   vitesseTemp;
     VitessesRobotUnicycle   vitessesCons;
     Vecteur                 vectTemp;
     Vecteur                 diffPositionRef;
     Vecteur                 matriceRot[2];

     /* v = K * erPos;  */
     vitesseTemp.longitudinale = diagMatriceGain[0] * erreurPose.x;
     vitesseTemp.rotation = diagMatriceGain[1] * erreurPose.y;

     /* v = v + diffPoseRef(3) * [-erPos(2); erPos(1)]; */
     vitesseTemp.longitudinale = vitesseTemp.longitudinale + (diffPoseRef.angle * (- erreurPose.y));
     vitesseTemp.rotation = vitesseTemp.rotation + (diffPoseRef.angle * erreurPose.x);

     /* v = v + matriceRotation(-thetaRef) * [diffPoseRef(1); diffPoseRef(2)]; */
     ASSER_TRAJ_MatriceRotation(matriceRot, - poseRef.angle);       /* Affectation d'une matrice de rotation de moins l'angle de reference */
     diffPositionRef.x = diffPoseRef.x;
     diffPositionRef.y = diffPoseRef.y;
     vectTemp = ASSER_TRAJ_ProduitMatriceVecteur(matriceRot, diffPositionRef);
     vitesseTemp.longitudinale = vitesseTemp.longitudinale + vectTemp.x;
     vitesseTemp.rotation = vitesseTemp.rotation + vectTemp.y;
     /* u = [cos(erTheta) sin(erTheta); -sin(erTheta)/l, cos(erTheta)/l]*v; */
     /* vectTemp.x = vitesseTemp.longitudinale;
        vectTemp.y = vitesseTemp.rotation; */
     vitessesCons.longitudinale = (cos(erreurPose.angle) * vitesseTemp.longitudinale) + (sin(erreurPose.angle) * vitesseTemp.rotation);
     vitessesCons.rotation = (- sin(erreurPose.angle) * vitesseTemp.longitudinale) + (cos(erreurPose.angle) * vitesseTemp.rotation);
     vitessesCons.rotation = vitessesCons.rotation / longueurBarreSuivi;

     return vitessesCons;
 }

/**********************************************************************/
/*! \brief ASSER_TRAJ_RetourDetatOrientation
 * 
 *  \note              loi de commande par retour d'etat du suivi de trajectoire avec asservissement de l'angle de la tangente a la trajectoire
 *
 *  \param [in]     erreurPose      erreur calculee par la fonction ASSER_TRAJ_ErreurPose
 *  \param [in]     diffPoseRef      derivee temporelle de la pose de reference
 *  \param [in]     gain                gain matriciel du retour d'etat
 *
 *  \return           vitessesCons    Structure de vitesses avec les consignes de vitesse longitudinale (en m/s) et de vitesse de rotation (en rd/s)
 */
/**********************************************************************/
static VitessesRobotUnicycle ASSER_TRAJ_RetourDetatOrientation(Pose erreurPose, Pose diffPoseRef, float *gain)
{
    VitessesRobotUnicycle   vitessesCons;
    float                   z[3];
    float                   w[2];
    float                   k1, k2, k3, uref1, uref2;
    
    k1 = gain[0];
    k2 = gain[1];
    k3 = gain[2];
    
    z[0] = erreurPose.x;
    z[1] = erreurPose.y;
    z[2] = tan(erreurPose.angle);
    
    uref1 = sqrt(SQUARE(diffPoseRef.x) + SQUARE(diffPoseRef.y));
    uref2 = diffPoseRef.angle;
    
    w[0] = -k1 * uref1 * fabs(uref1) * (z[0] + (z[1] * z[2]));
    w[1] = -(k2 * uref1 * z[1]) - (k3 * fabs(uref1) * z[2]);

    vitessesCons.longitudinale = (w[0] + uref1) / cos(erreurPose.angle);
    vitessesCons.rotation = (w[1] * SQUARE(cos(erreurPose.angle))) + uref2;

    return vitessesCons;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_TestFinAsservissement
 * 
 *  \note  Test de mouvement termine
 * 
 *  \param [in] traj                   trajectoire
 *  \param [in] erDist               erreur de de distance
 *  \param [in] memo_erDist    memo erreur de de distance
 *  \param [in] tolDist               tolerance de distance
 *  \param [in] erAngle             erreur d'angle  
 *  \param [in] memo_erAngle  memo erreur angle
 *  \param [in] tolAngle            tolerance d'angle
 *
 *  \return  True (si asser fini) or False (si asser non fini)
 */
/**********************************************************************/
static unsigned char ASSER_TRAJ_TestFinAsservissement(Trajectoire * traj, float erDist, float memo_erDist, float tolDist, float erAngle, float memo_erAngle, float tolAngle)
{
    unsigned char   ret                 = False;
    Pose            poseRobot_asserEnd;
    
    poseRobot_asserEnd = POS_GetPoseRobot();

    if (ASSER_TRAJ_isDeplacement(traj) == True)
    {
        /* Test pour un mouvement de deplacement */
        if ((erDist < tolDist) || (((erDist < (tolDist * 10.0)) && ((erDist - memo_erDist) > 0.0))))
        {
            ret = True;
            
            ASSER_TRAJ_LogAsser("vitesseAnglePtArret", NBR_ASSER_LOG_VALUE, (erAngle - memo_erAngle));
            ASSER_TRAJ_LogAsser("CPTvitesseAnglePtArret", NBR_ASSER_LOG_VALUE, ASSER_TRAJ_GetCompteur());
            
            ASSER_TRAJ_LogAsser("finAsser", NBR_ASSER_LOG_VALUE, poseRobot_asserEnd.x);
            ASSER_TRAJ_LogAsser("finAsser", NBR_ASSER_LOG_VALUE, poseRobot_asserEnd.y);
            ASSER_TRAJ_LogAsser("finAsser", NBR_ASSER_LOG_VALUE, poseRobot_asserEnd.angle);
        }
    }
    else
    {
        /* Test pour un mouvement de rotation */
        if ((erAngle < tolAngle) && ((erAngle > - tolAngle) || ((erAngle * memo_erAngle) < 0.0)))
        {
            ret = True;
        }
    }
    
    return ret;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_DistanceTrajectoire
 * 
 *  \note  calcul la longueur d'un segment de trajectoire
 *
 *  \param [in] segmentTraj      
 *
 *  \return None
 */
/**********************************************************************/
static void ASSER_TRAJ_DistanceTrajectoire(segmentTrajectoireBS * segmentTraj)
{
    float           distance = 0.000001;    /* toutes les divisions par la distance ne pourront pas etre des divisions par zero */
    unsigned char   param; 
    float           delta_D;
    Vecteur         diffD_T;

    for (param = 0; param < (unsigned char) 5; param++)
    {
        diffD_T = ASSER_TRAJ_DiffCourbeBSpline(segmentTraj, (segmentTraj->t[0] + (float)((float)param / 5.0) * g_tableauH[segmentTraj->i].x));
        delta_D = sqrt(SQUARE(diffD_T.x) + SQUARE(diffD_T.y)) * ((float)(1.0 / 5.0)) * g_tableauH[segmentTraj->i].x;
        distance += delta_D;
    }

    segmentTraj->distance = distance;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_LogAsser
 *
 *  \note  Prise des informations asser a logger
 * 
 *  \param [in] keyWord    Valeur de la table de log
 *  \param [in] index         Index de la table de log
 *  \param [in] val             Valeur a logguer
 *
 *  \return  nbAsserValue Nombre de donnees contenu dans le tabelau de log
 */
/**********************************************************************/
extern void ASSER_TRAJ_LogAsser(char * keyWord, unsigned char index, float val)
{
#ifdef PIC32_BUILD
    keyWord = keyWord;
    
    if (Test_mode == (unsigned long)2)
    {
        if ((int)index < NBR_ASSER_LOG_VALUE)
        {
            tabLogAsser[(int)index] = val;
        } 
    }
#else /* PIC32_BUILD */
    printf("log_%s: %1.5f\n", keyWord, val);
    fflush(stdout);
#endif /* PIC32_BUILD */
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_ResetLogAsserTable
 * 
 *  \note  Reset le tabelau de log
 * 
 *  \return None
 */
/**********************************************************************/
extern void ASSER_TRAJ_ResetLogAsserTable(void)
{
    unsigned char i;

    /* Reset de la table de log */
    for (i = 0; i < (unsigned char)NBR_ASSER_LOG_VALUE; i++)
    {
        tabLogAsser[(int)i] = 0.0;
    }  

    Sample = 0;
    TakeMesure = False;
}

/*! @} */

/*! @} */

/*! @} */

/* End of File : asserv_trajectoire.c */

