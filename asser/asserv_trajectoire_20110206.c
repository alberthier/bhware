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
float                           facteurVitesseAngulaireMax              =   4.0;
                            
/** Le point du robot asservit a la trajectoire n'est pas le centre de l'axe des roues, mais un point sur la droite perpendiculaire a cet axe et passant par son centre, situe a la distance NORME_BARRE_SUIVI_TRAJ en avant de l'axe des roues */
static const float              NORME_BARRE_SUIVI_TRAJ                  =   0.3;

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

/** Variables globales de l'asservissement de trajectoire */
static unsigned int             compteurPeriode                         =   0;
static float                    errDist, memo_errDist, errAngle, memo_errAngle;

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
static Pose                     poseReferenceOld                        =   {0.0,0.0,0.0};
static Pose                     poseReferenceRobotOld                   =   {0.0,0.0,0.0};

/** Vitesse normalisee courante */
//static float                    vitesseNormaliseeCourante               =   0.0;  /* distance a parcourir pendant la periode courante, distance qui est normalisee par la distance totale a parcourir. Elle sert a faire passer la vitesse d'une commande d'asser a l'autre en cas de changement de trajectoire en cours de mouvement */
static float                    vitesse_profil_consigne          =   0.0;
static float                    parametrePositionSegmentTrajectoire     =   0.0;    
//static float                    memoVitesseNormaliseeCourante           =   0.0;
static float                    memo_vitesse_profil_consigne     =   0.0;
static unsigned char            shuntTestFinAsser                       =   False;

/** Tableaux du profil de vitesse */
static float                    g_tab_gabarit_vitesse[20];

/*----------------------------------------------------------------------------------------------*/

/* Prototypes des fonctions */
static Pose                     ASSER_TRAJ_TrajectoireRotation(ParametresRotation *p_rotation, float t);
//static float                    ASSER_TRAJ_VitesseNormalisee_V3(Trajectoire *traj);
static float                    ASSER_TRAJ_VitesseLimiteEnVirage(Trajectoire *traj, float diffThetaTrajectoire);
static float                    ASSER_TRAJ_ProfilConsigneVitesse_V4(Trajectoire *traj, float paramTrajectoire);
static float                    ASSER_TRAJ_ParcoursTrajectoire(Trajectoire *traj, float delta_distance, unsigned int *segmentCourant, float *paramPoseSegTraj);
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
#if 0
static void                     ASSER_TRAJ_InitialisationDeriv2(unsigned int iPt, Vecteur q);
#endif

/*----------------------------------------------------------------------------------------------*/

/**********************************************************************/
/*! \brief ASSER_TRAJ_InitialisationGenerale
 * 
 *  \note  Fonction d'initialisation generale de l'asser traj
 *
 *  \return None
 */
/**********************************************************************/

extern void ASSER_TRAJ_InitialisationGenerale(void)
{    
    /* Initialisation du profil de vitesse */
    chemin.profilVitesse.p = 0;

    ASSER_TRAJ_ResetLogAsserTable();
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_GetCompteur
 * 
 *  \note  Fonction qui renvoie le compteur de periode
 *
 *  \return compteurPeriode
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
 *  \return segmentCourant
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
 * contenant toute la loi de commande et la condition d'arret de l'asservissement.
 *
 *  \param [in] poseRobot              pose du robot etant donne le choix de la marche AVANT ou ARRIERE pour l'orientation du robot
 *  \param [in] vitessesConsignes   pointeur sur une structure pour fournir les vitesses longitudinale et de rotation, de consignes
 *
 *  \return ASSER_Running             flag d'etat de l'asservissement
 */
/**********************************************************************/
 
extern void ASSER_TRAJ_AsservissementMouvementRobot(Pose poseRobot, VitessesRobotUnicycle *vitessesConsignes)
{
    float           diagonaleMatriceGain[2];
    float           gain[5];
    float           facteurCorrectionT          = 1.0;              //, delta_tmax_Limite;
/*    float           vitesseNormaliseeSegmentCourantCorrigee = 0.0;
    float           vitesseSegmentCourant       = 0.0;
    float           vitLimiteCourbure           = 0.0; */
    Pose            differentielleTemporellePoseReference;
    Pose            differentielleTemporellePoseReferenceRobot;
    Pose            P_robot;                                        /* Point fixe du robot a asservir a la trajectoire de consigne */
    Pose            erreurPoseCentreRobot;
    Pose            erreur_P;
    Vecteur         diffD_T, diff1BS, diff2BS;
    unsigned int    memoSegmentCourant;    
    float           vitesseAngulaire            = 0.0;
    float           delta_distance              = 0.0;
    float           delta_param_chemin          = 0.0;

#ifndef PIC32_BUILD
    unsigned int    cptEvt                      = 0;
    float           facteurCorrectionTordre2    = 0.0;
    Vecteur         derivPositionTraj;
#endif /* PIC32_BUILD */

    ASSER_TRAJ_LogAsser("xRoueGauche", NBR_ASSER_LOG_VALUE, poseRobot.x + (0.339 / 2.0) * cos(poseRobot.angle + (PI / 2)));
    ASSER_TRAJ_LogAsser("yRoueGauche", NBR_ASSER_LOG_VALUE, poseRobot.y + (0.339 / 2.0) * sin(poseRobot.angle + (PI / 2)));
    ASSER_TRAJ_LogAsser("xRoueDroite", NBR_ASSER_LOG_VALUE, poseRobot.x + (0.339 / 2.0) * cos(poseRobot.angle - (PI / 2)));
    ASSER_TRAJ_LogAsser("yRoueDroite", NBR_ASSER_LOG_VALUE, poseRobot.y + (0.339 / 2.0) * sin(poseRobot.angle - (PI / 2)));

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
/* Debut de la dermination de la nouvelle position à suivre */
/************************************************************/
            vitesse_profil_consigne = ASSER_TRAJ_ProfilConsigneVitesse_V4(&chemin, parametrePositionSegmentTrajectoire);
            memo_vitesse_profil_consigne = vitesse_profil_consigne;

            if (chemin.profilVitesse.p > 0)
            {
                /* distance de deplacement sur la periode */
                delta_distance = vitesse_profil_consigne * TE;
                chemin.profilVitesse.distNormaliseeRestante = chemin.profilVitesse.distNormaliseeRestante - delta_distance/chemin.distance;

                /* Forcage de la condition de la boucle while pour au moins une execution de la boucle */
                memoSegmentCourant = -1;
                while (segmentCourant != memoSegmentCourant)
                {
                    memoSegmentCourant = segmentCourant;
    
                    if (chemin.mouvement == DEPLACEMENT)
                    {
                        /* determination du ratio entre un deplacement en metre et un deplacement du parametre du chemin */
                        diffD_T = ASSER_TRAJ_DiffCourbeBSpline(&chemin.segmentTrajBS[segmentCourant], parametrePositionSegmentTrajectoire);
                        facteurCorrectionT = sqrt(pow(diffD_T.x, 2) + pow(diffD_T.y, 2));

                        /* deplacement du parametre de pose du chemin */
                        delta_param_chemin = delta_distance / facteurCorrectionT;
                    }
                    else /* if (chemin.mouvement == ROTATION) */
                    {
                        facteurCorrectionT = 1.0;
                        delta_param_chemin = delta_distance;
                    }

                    parametrePositionSegmentTrajectoire = parametrePositionSegmentTrajectoire + delta_param_chemin;                    

                    if (parametrePositionSegmentTrajectoire > chemin.segmentTrajBS[segmentCourant].t[1])
                    {  
                        /* Passage au segment suivant */
                        if (segmentCourant < (chemin.nbreSegments - 1))
                        {
                            /* part du deplacement du parametre du chemin au-dela du segment courant */
                            parametrePositionSegmentTrajectoire = parametrePositionSegmentTrajectoire - chemin.segmentTrajBS[segmentCourant].t[1];

                            /* part de la distance au-dela du segment courant */
                            delta_distance = parametrePositionSegmentTrajectoire / facteurCorrectionT;

                            segmentCourant++;
                            parametrePositionSegmentTrajectoire = chemin.segmentTrajBS[segmentCourant].t[0];
                        }
                        else
                        {
                            parametrePositionSegmentTrajectoire = 1.0;
                        }
                    }
                }
            }
            else
            {   
                /* Commande de vitesse nulle, le point d'arrivee de consigne est atteint */
                parametrePositionSegmentTrajectoire = 1.0;
            }
/**********************************************************/
/* Fin de la dermination de la nouvelle position à suivre */
/**********************************************************/

            poseReferenceOld = poseReference;
            poseReferenceRobotOld = poseReferenceRobot;
            
            if (chemin.mouvement == DEPLACEMENT)
            {
                diff1BS = ASSER_TRAJ_DiffCourbeBSpline(&chemin.segmentTrajBS[segmentCourant], parametrePositionSegmentTrajectoire);
                diff2BS = ASSER_TRAJ_Diff2CourbeBSpline(&chemin.segmentTrajBS[segmentCourant], parametrePositionSegmentTrajectoire);
                chemin.profilVitesse.diffThetaCourant = ASSER_TRAJ_DiffThetaBSpline(diff1BS, diff2BS);
                poseReference = ASSER_TRAJ_TrajectoireRemorqueBS(&chemin.segmentTrajBS[segmentCourant], parametrePositionSegmentTrajectoire, diff1BS, chemin.profilVitesse.diffThetaCourant, &poseReferenceRobot);
                //poseReference.angle = atan2((poseReference.y - poseReferenceOld.y), (poseReference.x - poseReferenceOld.x));
            }
            else /* if (chemin.mouvement == ROTATION) */
            {
                poseReference = ASSER_TRAJ_TrajectoireRotation(&(chemin.rotation), parametrePositionSegmentTrajectoire);
            }
            
            differentielleTemporellePoseReference = ASSER_TRAJ_DiffTemporelleTrajectoire(poseReferenceOld, poseReference, TE);
            differentielleTemporellePoseReferenceRobot = ASSER_TRAJ_DiffTemporelleTrajectoire(poseReferenceRobotOld, poseReferenceRobot, TE);
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

        if (chemin.mouvement == DEPLACEMENT)
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
    ASSER_TRAJ_LogAsser("distNormaliseeRestante", NBR_ASSER_LOG_VALUE, chemin.profilVitesse.distNormaliseeRestante);
    //ASSER_TRAJ_LogAsser("yPoseReference", poseReference.y);
//    ASSER_TRAJ_LogAsser("xPoseReferenceRobot", poseReferenceRobot.x);
//    ASSER_TRAJ_LogAsser("yPoseReferenceRobot", poseReferenceRobot.y);
    //ASSER_TRAJ_LogAsser("orientationPoseReferenceRobot", poseReferenceRobot.angle);
    /*
    ASSER_TRAJ_LogAsser("poseReferenceRobot", segmentCourant);
    */
    ASSER_TRAJ_LogAsser("vitesseProfilConsigne", NBR_ASSER_LOG_VALUE, memo_vitesse_profil_consigne);
    /*
    ASSER_TRAJ_LogAsser("Rinverse", chemin.profilVitesse.etat);
    ASSER_TRAJ_LogAsser("diffTheta", chemin.profilVitesse.diffThetaCourant);
    ASSER_TRAJ_LogAsser("vitesseAngulaire", vitesseAngulaire);
    */
    ASSER_TRAJ_LogAsser("parametrePositionSegmentTrajectoire", NBR_ASSER_LOG_VALUE, parametrePositionSegmentTrajectoire);

    /* enregistrement des vitesses de reference */
    /*
    ASSER_TRAJ_LogAsser("xDiffTemporellePoseRef", differentielleTemporellePoseReference.x);
    ASSER_TRAJ_LogAsser("yDiffTemporellePoseRef", differentielleTemporellePoseReference.y);
    ASSER_TRAJ_LogAsser("orientationDiffTemporellePoseRef", differentielleTemporellePoseReference.angle);
    */
//    ASSER_TRAJ_LogAsser("xDiffTemporellePoseRefRobot", differentielleTemporellePoseReferenceRobot.x);
//    ASSER_TRAJ_LogAsser("yDiffTemporellePoseRefRobot", differentielleTemporellePoseReferenceRobot.y);
    /*
    ASSER_TRAJ_LogAsser("orientationDiffTemporellePoseRefRobot", differentielleTemporellePoseReferenceRobot.angle);
*/
    ASSER_TRAJ_LogAsser("vitLongitudinale", NBR_ASSER_LOG_VALUE, vitessesConsignes->longitudinale);
    /*
    ASSER_TRAJ_LogAsser("vitLongitudinaleRef", sqrt(pow(differentielleTemporellePoseReference.x, 2) + pow(differentielleTemporellePoseReference.y, 2)));
    ASSER_TRAJ_LogAsser("facteurCorrectif", facteurCorrectionT);
    ASSER_TRAJ_LogAsser("facteurCorrectifOrdre2", facteurCorrectionTordre2);

    ASSER_TRAJ_LogAsser("xErreur_P", erreur_P.x);
    ASSER_TRAJ_LogAsser("yErreur_P", erreur_P.y);

    ASSER_TRAJ_LogAsser("xDerivPos", derivPositionTraj.x);
    ASSER_TRAJ_LogAsser("yDerivPos", derivPositionTraj.y);

    ASSER_TRAJ_LogAsser("xDiff2BS", diff2BS.x);
    ASSER_TRAJ_LogAsser("yDiff2BS", diff2BS.y);
    */
//    ASSER_TRAJ_LogAsser("xErreurPoseCentreRobot", erreurPoseCentreRobot.x);
//    ASSER_TRAJ_LogAsser("yErreurPoseCentreRobot", erreurPoseCentreRobot.y);
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_InitialisationTrajectoire
 * 
 *  \note   Initialisation d'une structure trajectoire definit en global dans le module ASSER_TRAJ
 *
 *  \param [in]     poseRobot       pose courante du robot integrant le choix de la marche (AVANT ou ARRIERE)
 *  \param [in]     point               pointeur du tableau des points imposes du chemin dont le point a atteindre en dernier
 *  \param [in]     nbrePts           nombre de points intermediaires du chemin par lesquelles passer +1 pour le point d'arrvee
 *  \param [in]     mouvement     type de mouvement a executer: un deplacement ou une pure rotation 
 */
/**********************************************************************/
extern void ASSER_TRAJ_InitialisationTrajectoire(Pose poseRobot, PtTraj *point, unsigned int nbrePts, unsigned int mouvement)
{
    Vecteur         diff1BS, diff2BS, posAPointer;
    unsigned int    iSegment, j, iCurrentGPoint_n0, iCurrentGPoint_n1;
    Vecteur         deltaI, deltaF, qI, qF;
    float           cordeInitiale, cordeFinale;
    float           angleInitial, angleFinal;
#ifndef PIC32_BUILD
    Pose            poseTest;
    int             intT;
    float           sommeMask;
#endif /* PIC32_BUILD */

    chemin.mouvement = mouvement;
    chemin.nbreSegments = nbrePts;
    parametrePositionSegmentTrajectoire = 0.0;
    segmentCourant = 0;
    shuntTestFinAsser = False;

    /* Position a atteindre (condition d'arret du test de fin d'asservissment) */
    chemin.posArrivee.x = point[(nbrePts - 1)].pose.x;
    chemin.posArrivee.y = point[(nbrePts - 1)].pose.y;

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

    /* Liste des points terminee */

    if (chemin.mouvement == DEPLACEMENT)
    {
        /* Configuration du profil de vitesse */
        chemin.profilVitesse.vmax = POS_GetConsVitesseMax();                /* UMAX * GAIN_STATIQUE_MOTEUR */
        chemin.profilVitesse.acc = chemin.profilVitesse.vmax / tempsAcc;

        if (nbrePts == 1)
        {
            /* Conditions aux limites des derivees */
            cordeInitiale = sqrt(pow((g_tableauPoints[1].x - g_tableauPoints[0].x), 2.0) + pow((g_tableauPoints[1].y - g_tableauPoints[0].y), 2.0));
            angleInitial = poseRobot.angle;
            deltaI.x =  (cordeInitiale * 1.0) * cos(angleInitial);
            deltaI.y =  (cordeInitiale * 1.0) * sin(angleInitial);
            angleFinal = point[0].pose.angle;
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

                    /*** Debut d'une boucle de config de portion de trajectoire ***/
                    /* Passage des segments de bords en B-Spline d'ordre 5, pour tenir compte des angles aux limites de chaque portion de trajectoire */
                    /* Conditions aux limites des derivees */
                    cordeInitiale = sqrt(pow((g_tableauPoints[(iCurrentGPoint_n0 + 1)].x - g_tableauPoints[iCurrentGPoint_n0].x), 2.0) + pow((g_tableauPoints[(iCurrentGPoint_n0 + 1)].y - g_tableauPoints[iCurrentGPoint_n0].y), 2.0));
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
                        cordeFinale = sqrt(pow((g_tableauPoints[iCurrentGPoint_n1].x - g_tableauPoints[(iCurrentGPoint_n1 - 1)].x), 2.0) + pow((g_tableauPoints[iCurrentGPoint_n1].y - g_tableauPoints[(iCurrentGPoint_n1 - 1)].y), 2.0));
                        deltaF.x =  (cordeFinale * 1.0) * cos(angleFinal);
                        deltaF.y =  (cordeFinale * 1.0) * sin(angleFinal);

                        /* Redefinition du premier segment */
                        diff1BS.x = (chemin.segmentTrajBS[iCurrentGPoint_n0].mx[1] * (g_tableauH[iCurrentGPoint_n0].x) / 2.0) + chemin.segmentTrajBS[iCurrentGPoint_n0].ax;
                        diff1BS.y = (chemin.segmentTrajBS[iCurrentGPoint_n0].my[1] * (g_tableauH[iCurrentGPoint_n0].y) / 2.0) + chemin.segmentTrajBS[iCurrentGPoint_n0].ay;
                        diff1BS.x = diff1BS.x * g_tableauH[iCurrentGPoint_n0].x;
                        diff1BS.y = diff1BS.y * g_tableauH[iCurrentGPoint_n0].y;

                        qI.x = chemin.segmentTrajBS[iCurrentGPoint_n0].mx[0] * pow(g_tableauH[iCurrentGPoint_n0].x, 2.0);
                        qI.y = chemin.segmentTrajBS[iCurrentGPoint_n0].my[0] * pow(g_tableauH[iCurrentGPoint_n0].y, 2.0);
                        qF.x = chemin.segmentTrajBS[iCurrentGPoint_n0].mx[1] * pow(g_tableauH[iCurrentGPoint_n0].x, 2.0);
                        qF.y = chemin.segmentTrajBS[iCurrentGPoint_n0].my[1] * pow(g_tableauH[iCurrentGPoint_n0].y, 2.0);

                        ASSER_TRAJ_InitialisationCourbeBS_5(g_tableauPoints[iCurrentGPoint_n0], g_tableauPoints[(iCurrentGPoint_n0 + 1)], deltaI, diff1BS, qI, qF, &chemin.segmentTrajBS[iCurrentGPoint_n0]);
                        chemin.segmentTrajBS[iCurrentGPoint_n0].ordre = 5;

                        /* Redefinition du dernier segment en BS d'ordre 5 */
                        diff1BS.x = -(chemin.segmentTrajBS[(iCurrentGPoint_n1 - 1)].mx[0] * (g_tableauH[(iCurrentGPoint_n1 - 1)].x) / 2.0) + chemin.segmentTrajBS[(iCurrentGPoint_n1 - 1)].ax;
                        diff1BS.y = -(chemin.segmentTrajBS[(iCurrentGPoint_n1 - 1)].my[0] * (g_tableauH[(iCurrentGPoint_n1 - 1)].y) / 2.0) + chemin.segmentTrajBS[(iCurrentGPoint_n1 - 1)].ay;
                        diff1BS.x = diff1BS.x * g_tableauH[(iCurrentGPoint_n1 - 1)].x;
                        diff1BS.y = diff1BS.y * g_tableauH[(iCurrentGPoint_n1 - 1)].y;

                        qI.x = chemin.segmentTrajBS[(iCurrentGPoint_n1 - 1)].mx[0] * pow(g_tableauH[(iCurrentGPoint_n1 - 1)].x, 2.0);
                        qI.y = chemin.segmentTrajBS[(iCurrentGPoint_n1 - 1)].my[0] * pow(g_tableauH[(iCurrentGPoint_n1 - 1)].y, 2.0);
                        qF.x = chemin.segmentTrajBS[(iCurrentGPoint_n1 - 1)].mx[1] * pow(g_tableauH[(iCurrentGPoint_n1 - 1)].x, 2.0);
                        qF.y = chemin.segmentTrajBS[(iCurrentGPoint_n1 - 1)].my[1] * pow(g_tableauH[(iCurrentGPoint_n1 - 1)].y, 2.0);

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
                    iSegment = iSegment + 1;
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
        /* Configuration du profil de vitesse */
        chemin.profilVitesse.vmax = POS_GetConsVitesseAngulaireMax() * NORME_BARRE_SUIVI_TRAJ;
        chemin.profilVitesse.acc = chemin.profilVitesse.vmax / tempsAcc;

        chemin.rotation.poseDepartRobot = poseRobot;
        //chemin.segmentTraj[0].posDepart.x = poseRobot.x + (NORME_BARRE_SUIVI_TRAJ * cos(poseRobot.angle));
        //chemin.segmentTraj[0].posDepart.y = poseRobot.y + (NORME_BARRE_SUIVI_TRAJ * sin(poseRobot.angle));
        chemin.nbreSegments = 1;
        posAPointer.x = point[0].pose.x;
        posAPointer.y = point[0].pose.y;
        chemin.rotation.angle = POS_ErreurOrientation(chemin.rotation.poseDepartRobot, posAPointer);

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
    chemin.profilVitesse.delta_tmax = ((TE * chemin.profilVitesse.vmax) / chemin.distance);
    chemin.profilVitesse.pmax = (unsigned int)(chemin.profilVitesse.vmax / (TE * chemin.profilVitesse.acc));    //floor(chemin.profilVitesse.delta_tmax / chemin.profilVitesse.delta_t)
    chemin.profilVitesse.delta_t = (((pow(TE, 2) * chemin.profilVitesse.acc)) / chemin.distance);              /* Delta_t a une dimension */
    chemin.profilVitesse.p = (unsigned int)((vitesse_profil_consigne / chemin.distance) / chemin.profilVitesse.delta_t);
    chemin.profilVitesse.distNormaliseeRestante = 1.0;                                                          /* -> la distance totale normalisee */
    chemin.profilVitesse.etat = 1;

//    ASSER_TRAJ_LogAsser("pmax", (float)(chemin.profilVitesse.pmax));
//    ASSER_TRAJ_LogAsser("pmax", chemin.profilVitesse.delta_tmax / chemin.profilVitesse.delta_t);
//    ASSER_TRAJ_LogAsser("pmax", chemin.profilVitesse.delta_tmax);
//    ASSER_TRAJ_LogAsser("pmax", ((float)chemin.profilVitesse.pmax) * chemin.profilVitesse.delta_t);

    compteurPeriode = 0;

    /* initialisation de l'erreur de distance avant l'arrivee */
    errDist = POS_ErreurDistance(poseRobot, chemin.posArrivee);
    errAngle = POS_ErreurOrientation(poseRobot, chemin.posArrivee);
}

#ifndef PIC32_BUILD

/**********************************************************************/
/*! \brief ASSER_TRAJ_InitialisationLogAsser
 * 
 *  \note   Initialisation du Log Asser 
 *
 *  \return None
 */
/**********************************************************************/
extern void ASSER_TRAJ_InitialisationLogAsser(void)
{
/*    ASSER_TRAJ_LogAsser("xPoseReference", poseReference.x);
    ASSER_TRAJ_LogAsser("yPoseReference", poseReference.y);

    ASSER_TRAJ_LogAsser("xPoseReferenceRobot", poseReferenceRobot.x);
    ASSER_TRAJ_LogAsser("yPoseReferenceRobot", poseReferenceRobot.y);
    ASSER_TRAJ_LogAsser("orientationPoseReferenceRobot", poseReferenceRobot.angle);
    ASSER_TRAJ_LogAsser("poseReferenceRobot", 0);
    ASSER_TRAJ_LogAsser("vitesseNormaliseeCourante", ((vitesseNormaliseeCourante * chemin.profilVitesse.vmax) / chemin.profilVitesse.delta_tmax));
    ASSER_TRAJ_LogAsser("Rinverse", 0.0);
    ASSER_TRAJ_LogAsser("diffTheta", chemin.profilVitesse.diffThetaCourant);
    ASSER_TRAJ_LogAsser("vitesseAngulaire", 0.0);
    ASSER_TRAJ_LogAsser("parametrePositionSegmentTrajectoire", 0.0);
    ASSER_TRAJ_LogAsser("xDiffTemporellePoseRef", 0.0);
    ASSER_TRAJ_LogAsser("yDiffTemporellePoseRef", 0.0);
    ASSER_TRAJ_LogAsser("orientationDiffTemporellePoseRef", 0.0);

    ASSER_TRAJ_LogAsser("xDiffTemporellePoseRefRobot", 0.0);
    ASSER_TRAJ_LogAsser("yDiffTemporellePoseRefRobot", 0.0);
    ASSER_TRAJ_LogAsser("orientationDiffTemporellePoseRefRobot", 0.0);

    ASSER_TRAJ_LogAsser("vitLongitudinale", 0.0);
    ASSER_TRAJ_LogAsser("vitLongitudinaleRef", 0.0);
    ASSER_TRAJ_LogAsser("facteurCorrectif", 1.0);
    ASSER_TRAJ_LogAsser("facteurCorrectifOrdre2", 0.0);

    ASSER_TRAJ_LogAsser("xErreur_P", 0.0);
    ASSER_TRAJ_LogAsser("yErreur_P", 0.0);

    ASSER_TRAJ_LogAsser("xDerivPos", 0.0);
    ASSER_TRAJ_LogAsser("yDerivPos", 0.0);

    ASSER_TRAJ_LogAsser("xDiff2BS", 0.0);
    ASSER_TRAJ_LogAsser("yDiff2BS", 0.0);*/
}

#endif /* PIC32_BUILD */

#if 0

/**********************************************************************/
/*! \brief ASSER_TRAJ_InitialisationDeriv2
 *
 *  \note   Reconfiguration de la derivee seconde de la trajectoire (Vecteur q) au point d'indice iPt, sans modifier autre chose.
 *
 *  \param [in]     iPt         indice du point auquel on impose une nouvelle derivee seconde
 *  \param [in]     q           valeurs de la derivee seconde de la trajectoire a imposer
 *
 *  \return None
 */
/**********************************************************************/
static void ASSER_TRAJ_InitialisationDeriv2(unsigned int iPt, Vecteur q)
{
    segmentTrajectoireBS *  segmentTraj;
    Vecteur                 qI, qF;
    Vecteur                 deltaI, deltaF;
    Vecteur                 posI, posF;
    Pose                    poseTemp;

    /* Reconfiguration du segment avant le point */
    if ((iPt > 0) && (iPt <= chemin.nbreSegments))
    {
        segmentTraj = &chemin.segmentTrajBS[(iPt - 1)];

        if (segmentTraj->ordre == 3u)
        {
            qI.x = segmentTraj->mx[0] * pow(g_tableauH[(iPt - 1)].x, 2.0);
            qI.y = segmentTraj->my[0] * pow(g_tableauH[(iPt - 1)].y, 2.0);

            qF.x = segmentTraj->mx[1] * pow(g_tableauH[(iPt - 1)].x, 2.0);
            qF.y = segmentTraj->my[1] * pow(g_tableauH[(iPt - 1)].y, 2.0);

            deltaI = ASSER_TRAJ_DiffCourbeBSpline(segmentTraj, segmentTraj->t[0]);
            deltaI.x = deltaI.x * g_tableauH[(iPt - 1)].x;
            deltaI.y = deltaI.y * g_tableauH[(iPt - 1)].y;

            deltaF = ASSER_TRAJ_DiffCourbeBSpline(segmentTraj, segmentTraj->t[1]);
            deltaF.x = deltaF.x * g_tableauH[(iPt - 1)].x;
            deltaF.y = deltaF.y * g_tableauH[(iPt - 1)].y;
        }
        else
        {
            qI = ASSER_TRAJ_Diff2CourbeBSpline(segmentTraj, segmentTraj->t[0]);
            qI.x = qI.x * pow(g_tableauH[(iPt - 1)].x, 2.0);
            qI.y = qI.y * pow(g_tableauH[(iPt - 1)].y, 2.0);
            
            qF = ASSER_TRAJ_Diff2CourbeBSpline(segmentTraj, segmentTraj->t[1]);
            qF.x = qF.x * pow(g_tableauH[(iPt - 1)].x, 2.0);
            qF.y = qF.y * pow(g_tableauH[(iPt - 1)].y, 2.0);

            deltaI = ASSER_TRAJ_DiffCourbeBSpline(segmentTraj, segmentTraj->t[0]);
            deltaI.x = deltaI.x * g_tableauH[(iPt - 1)].x;
            deltaI.y = deltaI.y * g_tableauH[(iPt - 1)].y;
            
            deltaF = ASSER_TRAJ_DiffCourbeBSpline(segmentTraj, segmentTraj->t[1]);
            deltaF.x = deltaF.x * g_tableauH[(iPt - 1)].x;
            deltaF.y = deltaF.y * g_tableauH[(iPt - 1)].y;
        }

        poseTemp = ASSER_TRAJ_TrajectoireBSpline(segmentTraj, segmentTraj->t[0]);
        posI.x = poseTemp.x;
        posI.y = poseTemp.y;

        poseTemp = ASSER_TRAJ_TrajectoireBSpline(segmentTraj, segmentTraj->t[1]);
        posF.x = poseTemp.x;
        posF.y = poseTemp.y;

        /* Application de la reconfiguration */
        ASSER_TRAJ_InitialisationCourbeBS_5(posI, posF, deltaI, deltaF, qI, qF, segmentTraj);
        segmentTraj->ordre = 5;
    }

    /* Reconfiguration du segment apres le point */
    if ((iPt >= (unsigned int)0u) && (iPt < chemin.nbreSegments))
    {
        segmentTraj = &chemin.segmentTrajBS[iPt];

        if (segmentTraj->ordre == (unsigned int)3)
        {
            qI.x = segmentTraj->mx[0] * pow(g_tableauH[iPt].x, 2.0);
            qI.y = segmentTraj->my[0] * pow(g_tableauH[iPt].y, 2.0);

            qF.x = segmentTraj->mx[1] * pow(g_tableauH[iPt].x, 2.0);
            qF.y = segmentTraj->my[1] * pow(g_tableauH[iPt].y, 2.0);

            deltaI = ASSER_TRAJ_DiffCourbeBSpline(segmentTraj, segmentTraj->t[0]);
            deltaI.x = deltaI.x * g_tableauH[iPt].x;
            deltaI.y = deltaI.y * g_tableauH[iPt].y;

            deltaF = ASSER_TRAJ_DiffCourbeBSpline(segmentTraj, segmentTraj->t[1]);
            deltaF.x = deltaF.x * g_tableauH[iPt].x;
            deltaF.y = deltaF.y * g_tableauH[iPt].y;
        }
        else
        {
            qI = ASSER_TRAJ_Diff2CourbeBSpline(segmentTraj, segmentTraj->t[0]);
            qI.x = qI.x * pow(g_tableauH[iPt].x, 2.0);
            qI.y = qI.y * pow(g_tableauH[iPt].y, 2.0);
            qF = ASSER_TRAJ_Diff2CourbeBSpline(segmentTraj, segmentTraj->t[1]);
            qF.x = qF.x * pow(g_tableauH[iPt].x, 2.0);
            qF.y = qF.y * pow(g_tableauH[iPt].y, 2.0);

            deltaI = ASSER_TRAJ_DiffCourbeBSpline(segmentTraj, segmentTraj->t[0]);
            deltaI.x = deltaI.x * g_tableauH[iPt].x;
            deltaI.y = deltaI.y * g_tableauH[iPt].y;
            deltaF = ASSER_TRAJ_DiffCourbeBSpline(segmentTraj, segmentTraj->t[1]);
            deltaF.x = deltaF.x * g_tableauH[iPt].x;
            deltaF.y = deltaF.y * g_tableauH[iPt].y;
        }

        poseTemp = ASSER_TRAJ_TrajectoireBSpline(segmentTraj, segmentTraj->t[0]);
        posI.x = poseTemp.x;
        posI.y = poseTemp.y;

        poseTemp = ASSER_TRAJ_TrajectoireBSpline(segmentTraj, segmentTraj->t[1]);
        posF.x = poseTemp.x;
        posF.y = poseTemp.y;

        /* Application de la reconfiguration */
        ASSER_TRAJ_InitialisationCourbeBS_5(posI, posF, deltaI, deltaF, qI, qF, segmentTraj);
        segmentTraj->ordre = 5;
    }
}

#endif

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
 *  \return None
 */
/**********************************************************************/
static void ASSER_TRAJ_InitialisationCourbeBS_5(Vecteur ptI, Vecteur ptF, Vecteur deltaPtI, Vecteur deltaPtF, Vecteur qI, Vecteur qF, segmentTrajectoireBS *segmentTraj)
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
 *  \param [in]     iPtI         
 *  \param [in]     iPtF
 *
 *  \reurn  None
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
 *  \param [in]     nbrePts     nombre de points de la trajectoire pour la definition de la taille de la matrice
 *
 *  \return None
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
 *  \return None
 */
/**********************************************************************/
static void ASSER_TRAJ_InterpolationBSpline3(unsigned int iPtI, unsigned int iPtF)
{
    float           distanceH;
    unsigned int    j, iSegment;

    distanceH = 0.0;
    
    for (j = iPtI; j < iPtF; j++)
    {
        g_tableauH[j].x = sqrt(pow((g_tableauPoints[(j + 1)].x - g_tableauPoints[j].x), 2) + pow((g_tableauPoints[(j + 1)].y - g_tableauPoints[j].y), 2));
        g_tableauH[j].y = g_tableauH[j].x;
        distanceH = distanceH + g_tableauH[j].x;
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
    ASSER_TRAJ_InitialisationMatriceInterpolation(iPtI, iPtF);                  //  iPtF - iPtI - 1);
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
 *  \return     poseTraj   pose de la courbe a laquelle doit etre asservi le point deporte du robot en fonction de t
 */
/**********************************************************************/
static Pose ASSER_TRAJ_TrajectoireRemorqueBS(segmentTrajectoireBS *segmentTraj, float t, Vecteur diff1BS, float diffThetaBSRobot, Pose *poseTrajRobot)
{   
    Pose    poseTraj;

    poseTraj = ASSER_TRAJ_TrajectoireBSpline(segmentTraj, t);
    *poseTrajRobot = poseTraj;

    poseTraj.x = poseTraj.x + (NORME_BARRE_SUIVI_TRAJ * cos(poseTraj.angle));
    poseTraj.y = poseTraj.y + (NORME_BARRE_SUIVI_TRAJ * sin(poseTraj.angle));
    poseTraj.angle = atan2((diff1BS.y + (NORME_BARRE_SUIVI_TRAJ * diffThetaBSRobot * cos(poseTraj.angle))), (diff1BS.x - (NORME_BARRE_SUIVI_TRAJ * diffThetaBSRobot * sin(poseTraj.angle))));
    
    return(poseTraj);
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_Trajectoire
 * 
 *  \note   equation parametrique de type B-Spline
 *
 *  \param [in]     t        parametre de la courbe: 0-> depart, 1-> arrivee.
 *
 *  \return     poseTraj   pose de la courbe fonction de l'argument t
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
static Pose ASSER_TRAJ_TrajectoireBSpline(segmentTrajectoireBS *segmentTraj, float t)
{   
    float   theta_P, tl;
    Vecteur derivPositionTraj;
    Pose    poseTraj;

    if (segmentTraj->ordre == (unsigned int)3)
    {   
        /* Segment defini comme un polynome d'ordre 3 */
        /* Calcul des coordonnees de la courbe parametrique  (-> equations cubiques) au parametre t */
        poseTraj.x = ((segmentTraj->mx[1] * pow((t - segmentTraj->t[0]), 3.0)) / (6.0 * g_tableauH[segmentTraj->i].x))  + ((segmentTraj->mx[0] * pow((segmentTraj->t[1] - t), 3.0)) / (6.0 * g_tableauH[segmentTraj->i].x)) + (segmentTraj->ax * (t - segmentTraj->t[0])) + segmentTraj->bx;
        poseTraj.y = ((segmentTraj->my[1] * pow((t - segmentTraj->t[0]), 3.0)) / (6.0 * g_tableauH[segmentTraj->i].x))  + ((segmentTraj->my[0] * pow((segmentTraj->t[1] - t), 3.0)) / (6.0 * g_tableauH[segmentTraj->i].x)) + (segmentTraj->ay * (t - segmentTraj->t[0])) + segmentTraj->by;
    }
    else /* ordre == 5, segment defini comme un polynome d'ordre 5 */
    {
        /* Changement de variable sur t */
        tl = (t - segmentTraj->t[0]) / g_tableauH[segmentTraj->i].x;
        poseTraj.x = ((segmentTraj->qx[0] / 20.0) * pow((1.0 - tl), 5.0)) + (segmentTraj->mx[0] * ((pow(tl, 3.0) / 6.0) - (pow(tl, 4.0) / 6.0) + (pow(tl, 5.0) / 20.0))) + (segmentTraj->mx[1] * ((pow(tl, 4.0) / 12.0) - (pow(tl, 5.0) / 20.0)) + ((segmentTraj->qx[1] * pow(tl, 5.0)) / 20.0)) + (segmentTraj->ax * tl) + segmentTraj->bx;
        poseTraj.y = ((segmentTraj->qy[0] / 20.0) * pow((1.0 - tl), 5.0)) + (segmentTraj->my[0] * ((pow(tl, 3.0) / 6.0) - (pow(tl, 4.0) / 6.0) + (pow(tl, 5.0) / 20.0))) + (segmentTraj->my[1] * ((pow(tl, 4.0) / 12.0) - (pow(tl, 5.0) / 20.0)) + ((segmentTraj->qy[1] * pow(tl, 5.0)) / 20.0)) + (segmentTraj->ay * tl) + segmentTraj->by;
    }
    
    /* Calcul de l'orientation de la trajectoire a t */
    derivPositionTraj = ASSER_TRAJ_DiffCourbeBSpline(segmentTraj, t);
    theta_P = atan2(derivPositionTraj.y, derivPositionTraj.x);
    poseTraj.angle = POS_ModuloAngle(theta_P);
    
    return(poseTraj);
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_DiffCourbeBSpline
 * 
 *  \note   Expression parametrique de la derivee premiere de l'expression parametrique de type B-Spline de la position
 *
 *  \param [in]     segmentTraj     pointeur de structure definissant un segment de trajectoire
 *  \param [in]     t                      parametre de la courbe parametrique [0; 1]: 0-> pose de depart, 1-> pose d'arrivee.
 *
 *  \return derivP  derivee des coordonnees de la position sur la courbe de B-Spline fonction de t,
 *                        par rapport au parametre t.
 */
/**********************************************************************/
static Vecteur ASSER_TRAJ_DiffCourbeBSpline(segmentTrajectoireBS *segmentTraj, float t)
{
    Vecteur derivP;
    float   tl;
    
    if (segmentTraj->ordre == (unsigned int)3)
    {   
        /* Segment defini comme un polynome d'ordre 3 */
        /* Derivee generique des composantes de la trajectoire par rapport a t */
        derivP.x = ((segmentTraj->mx[1] * pow((t - segmentTraj->t[0]),2.0)) / (2.0 * g_tableauH[segmentTraj->i].x)) - ((segmentTraj->mx[0] * pow((segmentTraj->t[1] - t), 2.0)) / (2.0 * g_tableauH[segmentTraj->i].x)) + segmentTraj->ax;
        derivP.y = ((segmentTraj->my[1] * pow((t - segmentTraj->t[0]),2.0)) / (2.0 * g_tableauH[segmentTraj->i].y)) - ((segmentTraj->my[0] * pow((segmentTraj->t[1] - t), 2.0)) / (2.0 * g_tableauH[segmentTraj->i].y)) + segmentTraj->ay;
    }
    else /* Ordre == 5, segment defini comme un polynome d'ordre 5 */
    {
        /* Changement de variable sur t */
        tl = (t - segmentTraj->t[0]) / g_tableauH[segmentTraj->i].x;
        
        derivP.x = (-(segmentTraj->qx[0] / 4.0) * pow((1.0 - tl), 4.0)) + (segmentTraj->mx[0] * ((pow(tl, 2.0) / 2.0) - ((2.0 * pow(tl, 3.0)) / 3.0) + (pow(tl, 4.0) / 4.0))) + (segmentTraj->mx[1] * ((pow(tl, 3.0)/ 3.0) - (pow(tl, 4.0) / 4.0))) + (segmentTraj->qx[1] * (pow(tl, 4.0) / 4.0)) + segmentTraj->ax;
        derivP.y = (-(segmentTraj->qy[0] / 4.0) * pow((1.0 - tl), 4.0)) + (segmentTraj->my[0] * ((pow(tl, 2.0) / 2.0) - ((2.0 * pow(tl, 3.0)) / 3.0) + (pow(tl, 4.0) / 4.0))) + (segmentTraj->my[1] * ((pow(tl, 3.0)/ 3.0) - (pow(tl, 4.0) / 4.0))) + (segmentTraj->qy[1] * (pow(tl, 4.0) / 4.0)) + segmentTraj->ay;

        derivP.x = derivP.x / g_tableauH[segmentTraj->i].x;
        derivP.y = derivP.y / g_tableauH[segmentTraj->i].y;
    }

    return(derivP);
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_Diff2CourbeBSpline
 * 
 *  \note   Expression parametrique de la derivee seconde de l'expression parametrique de type B-Spline de la position
 *
 *  \param [in]     segmentTraj  pointeur de structure definissant un segment de trajectoire
 *  \param [in]     t                   parametre de la courbe parametrique [0; 1]: 0-> pose de depart, 1-> pose d'arrivee.
 *
 *  \return  derivP     derivee des coordonnees de la position sur la courbe de B-Spline fonction de t,
 *                            par rapport au parametre t.
 */
/**********************************************************************/
static Vecteur ASSER_TRAJ_Diff2CourbeBSpline(segmentTrajectoireBS *segmentTraj, float t)
{
    Vecteur deriv2P;
    float   tl;
    
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
        
        deriv2P.x = (segmentTraj->qx[0] * pow((1.0 - tl), 3.0)) + (segmentTraj->mx[0] * tl * pow((1.0 - tl), 2.0) + segmentTraj->mx[1] * pow(tl, 2.0) * (1.0 - tl)) + (segmentTraj->qx[1] * pow(tl, 3.0));
        deriv2P.y = (segmentTraj->qy[0] * pow((1.0 - tl), 3.0)) + (segmentTraj->my[0] * tl * pow((1.0 - tl), 2.0) + segmentTraj->my[1] * pow(tl, 2.0) * (1.0 - tl)) + (segmentTraj->qy[1] * pow(tl, 3.0));

        deriv2P.x = deriv2P.x / pow(g_tableauH[segmentTraj->i].x, 2.0);
        deriv2P.y = deriv2P.y / pow(g_tableauH[segmentTraj->i].y, 2.0);
    }

    return(deriv2P);
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_DiffThetaBSpline
 * 
 *  \note   Expression parametrique de la derivee premiere de l'angle de la tangente a la trajectoire
 *
 *  \param [in]     diff1BS     
 *  \param [in]     diff2BS     
 *
 *  \return     
 */
/**********************************************************************/
static float ASSER_TRAJ_DiffThetaBSpline(Vecteur diff1BS, Vecteur diff2BS)
{
    float diffTheta;
    
    diffTheta = (diff2BS.y * diff1BS.x - diff1BS.y * diff2BS.x) / (pow(diff1BS.x, 2.0) * (1.0 + pow((diff1BS.y / diff1BS.x), 2.0)));

    return(diffTheta);
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
    
    return(poseTraj);
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_VitesseNormalisee_V3
 * 
 *  \note   Determination de la vitesse normalisee de consigne a appliquer (3ieme version: profil de vitesse trapezoidal)
 *
 *  \param [in]     traj            pointeur de structure definissant la trajectoire
 *
 *  \return         vitNormFinale   vitesse normalisee
 */
/**********************************************************************/
static float ASSER_TRAJ_VitesseNormalisee_V3(Trajectoire *traj)
{
    float   vitNormTemp = 0.0;
    float   distMinDecc;

    if (traj->profilVitesse.etat != -1)
    {   
        /* soit acceleration, soit vitesse constante */
        if (traj->profilVitesse.etat == 1)
        {   
            /* Acceleration */

            /* Calcul de la vitesse suivante probable */
            traj->profilVitesse.p++;

            /* Test de depassement de la vitesse maximale */
            if (traj->profilVitesse.p > traj->profilVitesse.pmax)
            {
                traj->profilVitesse.p = traj->profilVitesse.pmax;
                traj->profilVitesse.etat = 0;   /* Passage en vitesse constante */
            }

            vitNormTemp = ((float)traj->profilVitesse.p) * traj->profilVitesse.delta_t;
        }
        else /* traj->profilVitesse.etat est egal a 0, maintien de la vitesse */
        {   
            /* Vitesse constante */
            vitNormTemp = ((float)traj->profilVitesse.p) * traj->profilVitesse.delta_t;
        }

        /* Test de declenchement de la decceleration */
        if ((traj->profilVitesse.p > 0) && (ASSER_FragmentTraj == False))
        {
            /* Calcul de la distance normalisee minimale pour atteindre une vitesse nulle */
            distMinDecc = (traj->profilVitesse.delta_t * (float)((traj->profilVitesse.p  + traj->profilVitesse.etat) * (traj->profilVitesse.p + traj->profilVitesse.etat + 1))) / 2.0;
            if ( (traj->profilVitesse.distNormaliseeRestante - vitNormTemp) < distMinDecc )
            {
                traj->profilVitesse.p = traj->profilVitesse.p - traj->profilVitesse.etat; /* Retour au niveau de vitesse actuel */
                traj->profilVitesse.etat = -1;  /* Passage en decceleration */

                /* Calcul d'un delta d'acceleration supplementaire pour la decceleration */
                traj->profilVitesse.delta2t = (traj->profilVitesse.distNormaliseeRestante / ((float)(traj->profilVitesse.p - 1))) - ((((float)traj->profilVitesse.p) * traj->profilVitesse.delta_t) / 2.0);
                /* Decceleration */
                traj->profilVitesse.p--;
                vitNormTemp = ((float)traj->profilVitesse.p) * traj->profilVitesse.delta_t + traj->profilVitesse.delta2t;
                if (traj->profilVitesse.p == 0)
                {
                    traj->profilVitesse.etat = 0;
                    vitNormTemp = 0.0;
                }
            }
        }
    }
    else /* traj->profilVitesse.etat est egal a -1 */
    {  
        /* Decceleration */
        traj->profilVitesse.p = traj->profilVitesse.p--;
        vitNormTemp = (((float)traj->profilVitesse.p) * traj->profilVitesse.delta_t) + traj->profilVitesse.delta2t;
        
        if (traj->profilVitesse.p == 0)
        {
            traj->profilVitesse.etat = 0;
            vitNormTemp = 0.0;
        }
    }

    return vitNormTemp;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_VitesseLimiteEnVirage
 *
 *  \note   Determination de la vitesse longitudinale de consigne a appliquer en fonction de la courbure de la trajectoire
 *
 *  \param [in]     traj            pointeur de structure definissant la trajectoire
 *  \param [in]     diffThetaTrajectoire    vitesse angulaire de la trajectoire
 *
 *  \return         vitesseLimite   vitesse en m/s
 */
/**********************************************************************/
static float ASSER_TRAJ_VitesseLimiteEnVirage(Trajectoire *traj, float diffThetaTrajectoire)
{
    float vitesseLimite = 0.0;

    vitesseLimite = traj->profilVitesse.vmax / (1.0 + (diffThetaTrajectoire * facteurVitesseAngulaireMax));

    return(vitesseLimite);
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_ParcoursTrajectoire
 *
 *  \note   Determination du gabarit de la vitesse longitudinale de consigne, dependant de la courbure de trajectoire
 *  et des fonctions d'acceleration et de decceleration.
 *
 *  \param [in]     traj            pointeur de structure definissant la trajectoire
 *  \param [in]     delta_distance      distance de deplacement sur la periode
 *  \param [in]     paramTrajectoire    parametre de position de la position sur le segment de trajectoire courant
 *
 *  \return
 */
/**********************************************************************/
static float ASSER_TRAJ_ParcoursTrajectoire(Trajectoire *traj, float delta_distance, unsigned int *segmentCourant, float *paramPoseSegTraj)
{
    unsigned int    memoSegmentCourant = 0;
    Vecteur         diffD_T;
    float           facteurCorrectionT = 0.0;
    float           delta_param_chemin = 0.0;

    /* distance normalisee restant a parcourir */
    traj->profilVitesse.distNormaliseeRestante = traj->profilVitesse.distNormaliseeRestante - delta_distance/traj->distance;

    /* Forcage de la condition de la boucle while pour au moins une execution de la boucle */
    memoSegmentCourant = -1;
    while (*segmentCourant != memoSegmentCourant)
    {
        memoSegmentCourant = *segmentCourant;

        if (traj->mouvement == DEPLACEMENT)
        {
            /* determination du ratio entre un deplacement en metre et un deplacement du parametre du chemin */
            diffD_T = ASSER_TRAJ_DiffCourbeBSpline(&traj->segmentTrajBS[*segmentCourant], *paramPoseSegTraj);
            facteurCorrectionT = sqrt(pow(diffD_T.x, 2) + pow(diffD_T.y, 2));

            /* deplacement du parametre de pose du chemin */
            delta_param_chemin = delta_distance / facteurCorrectionT;
        }
        else /* if (chemin.mouvement == ROTATION) */
        {
            facteurCorrectionT = 1.0;
            delta_param_chemin = delta_distance;
        }

        *paramPoseSegTraj = *paramPoseSegTraj + delta_param_chemin;

        if (*paramPoseSegTraj > traj->segmentTrajBS[*segmentCourant].t[1])
        {
            /* Passage au segment suivant */
            if (*segmentCourant < (traj->nbreSegments - 1))
            {
                /* part du deplacement du parametre du chemin au-dela du segment courant */
                *paramPoseSegTraj = *paramPoseSegTraj - traj->segmentTrajBS[segmentCourant].t[1];

                /* part de la distance au-dela du segment courant */
                delta_distance = *paramPoseSegTraj / facteurCorrectionT;

                (*segmentCourant)++;
                *paramPoseSegTraj = traj->segmentTrajBS[*segmentCourant].t[0];
            }
            else
            {
                *paramPoseSegTraj = 1.0;
            }
        }
    }
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_GabaritVitesse
 *
 *  \note   Determination du gabarit de la vitesse longitudinale de consigne, dependant de la courbure de trajectoire
 *  et des fonctions d'acceleration et de decceleration.
 *
 *  \param [in]     traj            pointeur de structure definissant la trajectoire
 *  \param [in]     paramTrajectoire    parametre de position de la position sur le segment de trajectoire courant
 *
 *  \return
 */
/**********************************************************************/
static float ASSER_TRAJ_GabaritVitesse(Trajectoire *traj)
{
    // parcourir la trajectoire en l'echantillonnant


    g_tab_gabarit_vitesse
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_ProfilConsigneVitesse_V4
 *
 *  \note   Determination de la vitesse longitudinale de consigne a appliquer
 *
 *  \param [in]     traj            pointeur de structure definissant la trajectoire
 *  \param [in]     paramTrajectoire    parametre de position de la position sur le segment de trajectoire courant
 *
 *  \return         vitFinale   vitesse en m/s
 */
/**********************************************************************/
static float ASSER_TRAJ_ProfilConsigneVitesse_V4(Trajectoire *traj, float paramTrajectoire)
{
    float   vitNormTemp = 0.0;
    float   distMinDecc;

    if (traj->profilVitesse.etat != -1)
    {
        /* soit acceleration, soit vitesse constante */
        if (traj->profilVitesse.etat == 1)
        {
            /* Acceleration */

            /* Calcul de la vitesse suivante probable */
            traj->profilVitesse.p++;

            /* Test de depassement de la vitesse maximale */
            if (traj->profilVitesse.p > traj->profilVitesse.pmax)
            {
                traj->profilVitesse.p = traj->profilVitesse.pmax;
                traj->profilVitesse.etat = 0;   /* Passage en vitesse constante */
            }

            vitNormTemp = ((float)traj->profilVitesse.p) * traj->profilVitesse.delta_t;
        }
        else /* traj->profilVitesse.etat est egal a 0, maintien de la vitesse */
        {
            /* Vitesse constante */
            //vitNormTemp = vitNormInitiale;
            vitNormTemp = ((float)traj->profilVitesse.p) * traj->profilVitesse.delta_t;
        }

        /* Test de declenchement de la decceleration */
        if ((traj->profilVitesse.p > 0) && (ASSER_FragmentTraj == False))
        {
            /* Calcul de la distance normalisee minimale pour atteindre une vitesse nulle */
            distMinDecc = (traj->profilVitesse.delta_t * (float)((traj->profilVitesse.p  + traj->profilVitesse.etat) * (traj->profilVitesse.p + traj->profilVitesse.etat + 1))) / 2.0;
            if ( (traj->profilVitesse.distNormaliseeRestante - vitNormTemp) < distMinDecc )
            {
                traj->profilVitesse.p = traj->profilVitesse.p - traj->profilVitesse.etat; /* Retour au niveau de vitesse actuel */
                traj->profilVitesse.etat = -1;  /* Passage en decceleration */

                /* Calcul d'un delta d'acceleration supplementaire pour la decceleration */
                traj->profilVitesse.delta2t = (traj->profilVitesse.distNormaliseeRestante / ((float)(traj->profilVitesse.p - 1))) - ((((float)traj->profilVitesse.p) * traj->profilVitesse.delta_t) / 2.0);
                /* Decceleration */
                traj->profilVitesse.p--;
                vitNormTemp = ((float)traj->profilVitesse.p) * traj->profilVitesse.delta_t + traj->profilVitesse.delta2t;
                if (traj->profilVitesse.p == 0)
                {
                    traj->profilVitesse.etat = 0;
                    vitNormTemp = 0.0;
                }
            }
        }
    }
    else /* traj->profilVitesse.etat est egal a -1 */
    {
        /* Decceleration */
        traj->profilVitesse.p = traj->profilVitesse.p--;
        vitNormTemp = (((float)traj->profilVitesse.p) * traj->profilVitesse.delta_t) + traj->profilVitesse.delta2t;

        if (traj->profilVitesse.p == 0)
        {
            traj->profilVitesse.etat = 0;
            vitNormTemp = 0.0;
        }
    }

    return ((vitNormTemp * traj->distance)/TE);
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_DiffTemporelleTrajectoire
 * 
 *  \note           derivee temporelle discrete d'une pose
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
static void ASSER_TRAJ_MatriceRotation(Vecteur *matrice2p2, float angle)
{
    matrice2p2[0].x = cos(angle);
    matrice2p2[0].y = sin(angle);
    matrice2p2[1].x = - sin(angle);
    matrice2p2[1].y = cos(angle);
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_ProduitMatriceVecteur
 * 
 *  \note           produit matriciel entre une matrice 2x2 a gauche, et un vecteur a droite. 
 *  Outil permettant un changement de repere suivant une rotation de la base du repere.
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
    ASSER_TRAJ_MatriceRotation(matrice, - poseRef.angle); /* Affectation de la matrice par la matrice de rotation de moins l'angle de reference */
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
  *  \return     vitessesCons                  Structure de vitesses avec les consignes de vitesse longitudinale (en m/s) et de vitesse de rotation (en rd/s)
  */
 /**********************************************************************/
 static VitessesRobotUnicycle ASSER_TRAJ_RetourDetat(Pose erreurPose, Pose poseRef, Pose diffPoseRef, float longueurBarreSuivi, float *diagMatriceGain)
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
     ASSER_TRAJ_MatriceRotation(matriceRot, - poseRef.angle); /* Affectation d'une matrice de rotation de moins l'angle de reference */
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
 *  \note           loi de commande par retour d'etat du suivi de trajectoire avec asservissement de l'angle de la tangente a la trajectoire
 *
 *  \param [in]     erreurPose                erreur calculee par la fonction ASSER_TRAJ_ErreurPose
 *  \param [in]     diffPoseRef                derivee temporelle de la pose de reference
 *  \param [in]     gain                          gain matriciel du retour d'etat
 *
 *  \return     vitessesCons                    Structure de vitesses avec les consignes de vitesse longitudinale (en m/s) et de vitesse de rotation (en rd/s)
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
    
    uref1 = sqrt(pow(diffPoseRef.x, 2) + pow(diffPoseRef.y, 2));
    uref2 = diffPoseRef.angle;
    
    w[0] = -k1 * uref1 * fabs(uref1) * (z[0] + (z[1] * z[2]));
    w[1] = -(k2 * uref1 * z[1]) - (k3 * fabs(uref1) * z[2]);

    vitessesCons.longitudinale = (w[0] + uref1) / cos(erreurPose.angle);
    vitessesCons.rotation = (w[1] * pow(cos(erreurPose.angle), 2)) + uref2;

    return vitessesCons;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_TestFinAsservissement
 * 
 *  \note  Test de mouvement termine
 * 
 *  \param [in] traj          trajectoire
 *  \param [in] erDist       erreur de de distance
 *  \param [in] tolDist      tolerance de distance
 *  \param [in] erAngle    erreur d'angle 
 *  \param [in] tolAngle    tolerance d'angle
 *
 *  \return  True (si asser fini) or False (si asser non fini)
 */
/**********************************************************************/
static unsigned char ASSER_TRAJ_TestFinAsservissement(Trajectoire *traj, float erDist, float memo_erDist, float tolDist, float erAngle, float memo_erAngle, float tolAngle)
{
    unsigned char   ret                 = False;
    Pose            poseRobot_asserEnd;
    
    poseRobot_asserEnd = POS_GetPoseRobot();
    
    /* Test pour un mouvement de deplacement */
    if ((traj->mouvement == DEPLACEMENT) && ((erDist < tolDist) || (((erDist < (tolDist * 10.0)) && ((erDist - memo_erDist) > 0.0)))))
    {
        ret = True;
        
        ASSER_TRAJ_LogAsser("vitesseAnglePtArret", NBR_ASSER_LOG_VALUE, (erAngle - memo_erAngle));
        ASSER_TRAJ_LogAsser("CPTvitesseAnglePtArret", NBR_ASSER_LOG_VALUE, ASSER_TRAJ_GetCompteur());
        
        ASSER_TRAJ_LogAsser("finAsser", NBR_ASSER_LOG_VALUE, poseRobot_asserEnd.x);
        ASSER_TRAJ_LogAsser("finAsser", NBR_ASSER_LOG_VALUE, poseRobot_asserEnd.y);
        ASSER_TRAJ_LogAsser("finAsser", NBR_ASSER_LOG_VALUE, poseRobot_asserEnd.angle);
//        ASSER_TRAJ_LogAsser("ASSER_Running", 0.0);
//        ASSER_TRAJ_LogAsser("distErr", erDist);
    }

    /* Test pour un mouvement de rotation */
    if ((traj->mouvement == ROTATION) && ((erAngle < tolAngle) && ((erAngle > - tolAngle) || ((erAngle * memo_erAngle) < 0.0))))
    {
        ret = True;
        
 //       ASSER_TRAJ_LogAsser("ASSER_Running", 0.0);
 //       ASSER_TRAJ_LogAsser("thetaErr", erAngle);
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
static void ASSER_TRAJ_DistanceTrajectoire(segmentTrajectoireBS *segmentTraj)
{
    float           distance = 0.000001;    /* toutes les divisions par la distance ne pourront pas etre des divisions par zero */
    unsigned char   param; 
    float           delta_D;
    Vecteur         diffD_T;

    for (param = 0; param < (unsigned char) 5; param++)
    {
        diffD_T = ASSER_TRAJ_DiffCourbeBSpline(segmentTraj, (segmentTraj->t[0] + (float)((float)param / 5.0) * g_tableauH[segmentTraj->i].x));
        delta_D = sqrt(pow(diffD_T.x, 2) + pow(diffD_T.y, 2)) * ((float)(1.0 / 5.0)) * g_tableauH[segmentTraj->i].x;
        distance = distance + delta_D;
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
extern void ASSER_TRAJ_LogAsser(char *keyWord, unsigned char index, float val)
{
#ifdef PIC32_BUILD
    keyWord = keyWord;
    
    if (Test_mode == (unsigned long)True)
    {
        if ((int)index < NBR_ASSER_LOG_VALUE)
        {
            tabLogAsser[(int)index] = val;
        } 
    }
#else /* PIC32_BUILD */
    printf("log_%s: %1.5f\n", keyWord, val);
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

