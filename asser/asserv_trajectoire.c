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
#include "pic18.h"

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

/** Constantes */

#define                         Vitesse_Gain_ASR                        ((float) 0.002)

/*----------------------------------------------------------------------------------------------*/

/** Variables globales */

/** Parametres de l'asservissement par retour d'etat */
float                           gainRotation1                           =   - 6.0;   /* Gain de l'asservissement de la vitesse longitudinale */
float                           gainRotation2                           =   - 6.0;   /* Gain de l'asservissement de la vitesse de rotation */
float                           gainDeplacement1                        =   20.0;
float                           gainDeplacement2                        =   50.0;
float                           gainDeplacement3                        =   20.0;

/** Parametres de la generation de trajectoire */
float                           Ratio_Acc                               =   1.0;
float                           Ratio_Decc                              =   1.0;

float                           FacteurVitesseAngulaireMax              =   1.0;    /* Ce facteur doit etre positif */

float                           A_MAX                                   =   0.575;
float                           D_MAX                                   =   0.575;

float                           COEFF_VI1                               =   0.95;   /* COEFF_VI1 doit etre strictement positif et strictement inferieur a 1 (0.0 < COEFF_VI1 < 1.0) */
float                           VITESSE_SEUIL_DECC                      =   0.2;
float                           COEFF_DECC_FINALE                       =   0.08;
float                           DECC_MIN                                =   -0.3;   /* Decceleration en m/s^2 du profil de vitesse au point d'arrivee */

float                           k0_init                                 =   0.3; 
float                           C_init                                  =   0.2;

/***********************************************************************************************/
/**************************** Fin de la definition des parametres de l'asservissement de trajectoire *********/
/***********************************************************************************************/

/*----------------------------------------------------------------------------------------------*/

/** Variables locales */

/** Le point du robot asservit a la trajectoire n'est pas le centre de l'axe des roues, mais un point sur la droite perpendiculaire a cet axe et passant par son centre, situe a la distance NORME_BARRE_SUIVI_TRAJ en avant de l'axe des roues */
static float                    NORME_BARRE_SUIVI_TRAJ;                             /* Initialise dans ASSER_TRAJ_InitialisationGenerale() */

/** Variables globales de l'asservissement de trajectoire */
static unsigned int             compteurPeriode                         =   0;
static float                    errDist, memo_errDist, errAngle, memo_errAngle, memo_angleRobot;

/** Vecteurs associees au depart et a l'arrivee, et parametrage du profil de vitesse de la trajectoire */
static Trajectoire              chemin;
static unsigned int             segmentCourant                          =   1;

/** Contraintes d'affectation : ti doit etre compris entre 0 et 0.5, avec les valeurs 0.0 et 0.5 exclus */
static float                    ti                                      =   0.3;

/** Pose de la trajectoire de consigne a l'instant t*/
static Pose                     poseReference                           =   {0.0,0.0,0.0};
static Pose                     poseReferenceRobot                      =   {0.0,0.0,0.0};
static Pose                     poseReferenceAv                         =   {0.0,0.0,0.0};

/* Distance a parcourir pendant la periode courante, distance qui est normalisee par la distance totale a parcourir. Elle sert a faire passer la vitesse d'une commande d'asser a l'autre en cas de changement de trajectoire en cours de mouvement */
static float                    vitesse_profil_consigne                 =   0.0;
static float                    parametrePositionSegmentTrajectoire     =   0.0;
static unsigned char            shuntTestFinAsser                       =   False;

/** Tableaux du profil de vitesse */
static float                    g_tab_gabarit_vitesse[TAILLE_TAB_GABARIT_VITESSE];
static float                    g_tab_gabarit_acceleration[TAILLE_TAB_GABARIT_VITESSE];
static unsigned int             g_index_tab_gabarit_vitesse             = 0;
static unsigned int             g_index_tab_gabarit_acceleration        = 0;

/* tableau de 1: distance des phases d'acc et de decc reunies, 2: vitesse max(de pointe), 3; distance de decc */
static float                    g_tab_vpointe[10][3]; // = {{0.04, 0.2, 0.025}, {0.085, 0.3, 0.04}, {0.165, 0.4, 0.075}, {0.27, 0.5, 0.115}, {0.4, 0.6, 0.165}};
static unsigned int             iMaxVpointe                             = 9;

/* Variables profil de vitesse Scurve */
static float                    Vmax, VgASR, Vconsigne0, J, A, V, P, J0, A0, V0, P0, DistanceMin, JAmax, JDmax, PositionFinPhase4;
static unsigned char            Phase                                   = 0;
static unsigned char            ASRrunning                              = False;
/* Constantes  profil de vitesse Scurve */
static const float              VminMouv                                = 0.100;
static const float              EcartVitesse                            = 0.010;

/*----------------------------------------------------------------------------------------------*/

/* Prototypes des fonctions */

static void                     ASSER_TRAJ_MatriceRotation(Vecteur *matrice2p2, float angle);
static Pose                     ASSER_TRAJ_TrajectoireRotation(ParametresRotation *p_rotation, float t);
static void                     ASSER_TRAJ_GabaritVitesse(Trajectoire *traj);
static void                     ASSER_TRAJ_ParcoursTrajectoire(Trajectoire *traj, float delta_distance, unsigned int *segmentCourant, float *paramPoseSegTraj, unsigned char * pReturn);
static Pose                     ASSER_TRAJ_DiffTemporelleTrajectoire(Pose posePrecedente, Pose poseActuelle, float periode);
static Vecteur                  ASSER_TRAJ_ProduitMatriceVecteur(Vecteur *matrice, Vecteur vecteur);
static Pose                     ASSER_TRAJ_ErreurPose(Pose poseRobot_P, Pose poseRef);
static VitessesRobotUnicycle    ASSER_TRAJ_RetourDetat(Pose erreurPose, Pose poseRef, Pose diffPoseRef, float longueurBarreSuivi, float *diagMatriceGain);
static VitessesRobotUnicycle    ASSER_TRAJ_RetourDetatOrientation(Pose erreurPose, Pose diffPoseRef, float gain[]);
static unsigned char            ASSER_TRAJ_TestFinAsservissement(Trajectoire *traj, float erDist, float memo_erDist, float tolDist, float erAngle, float memo_erAngle, float tolAngle);
static void                     ASSER_TRAJ_DistanceTrajectoire(segmentTrajectoireBS *segmentTraj, unsigned int iSegment);
static Pose                     ASSER_TRAJ_TrajectoireBSpline(segmentTrajectoireBS *segmentTraj, unsigned int iSegment, float t);
static Vecteur                  ASSER_TRAJ_DiffCourbeBSpline(segmentTrajectoireBS *segmentTraj, unsigned int iSegment, float t);
static Vecteur                  ASSER_TRAJ_DiffTrajectoire(segmentTrajectoireBS * segmentTraj, unsigned int iSegment, float t);
static Vecteur                  ASSER_TRAJ_Diff2CourbeBSpline(segmentTrajectoireBS *segmentTraj, unsigned int iSegment, float t);
static Vecteur                  ASSER_TRAJ_Diff2Trajectoire(segmentTrajectoireBS *segmentTraj, unsigned int iSegment, float t);
static float                    ASSER_TRAJ_DiffThetaBSpline(Vecteur diff1BS, Vecteur diff2BS);
static float                    ASSER_TRAJ_DiffThetaBSplinePerLengthUnit(segmentTrajectoireBS * segmentTraj, unsigned int iSegment, float t);
static Pose                     ASSER_TRAJ_TrajectoireRemorqueBS(segmentTrajectoireBS *segmentTraj, unsigned int iSegment, float t, Vecteur diff1BS, float diffThetaBSRobot, Pose *poseTrajRobot);
static float                    ASSER_TRAJ_Acceleration_vs_Vitesse(Trajectoire * traj, float vpointe, float vitesse);
static float                    ASSER_TRAJ_Decceleration_vs_Vitesse(Trajectoire * traj, float vpointe, float vitesse);
static unsigned char            ASSER_TRAJ_ProfilAcceleration_2012(Trajectoire *traj, float vpointe, float *vitesse_n, float * acceleration);
static unsigned char            ASSER_TRAJ_ProfilDecceleration_2012(Trajectoire *traj, float vpointe, float *vitesse_n, float * acceleration);
static float                    ASSER_TRAJ_DistanceAcceleration(Trajectoire * traj, float vpointe, float vit_ini);
static float                    ASSER_TRAJ_DistanceDecceleration(Trajectoire * traj, float vpointe);
static void                     ASSER_TRAJ_TabGabarit_AddVitesse(float vitesse);
static void                     ASSER_TRAJ_TabGabarit_AddAcceleration(float acceleration);
static float                    ASSER_TRAJ_VitesseLimiteEnVirage(Trajectoire *traj, float diffThetaTrajectoire);
static void                     ASSER_TRAJ_SmoothGabaritAcceleration(Trajectoire * traj);
static void                     ASSER_TRAJ_CreateGabaritVitesseFromGabaritAcceleration(Trajectoire * traj);
static void                     ASSER_TRAJ_InitTabVPointe(Trajectoire *traj, float vit_ini);
static unsigned char            ASSER_TRAJ_isDeplacement(Trajectoire *traj);
static unsigned char            ASSER_TRAJ_isAngle(float angle);
static float                    ASSER_TRAJ_CalculTheta1(unsigned int iSegment, unsigned int nbrePts, PtTraj* point, float angle_rad, float prec_x, float prec_y);
static float                    ASSER_TRAJ_LongueurSegment(float x0, float y0, float x1, float y1);
static Vecteur                  ASSER_TRAJ_PortionEnd(float bx, float by, float ax, float ay, float qx, float qy);
static void                     ASSER_TRAJ_SolveSegment_v2(float x0, float y0, float theta0, float x1, float y1, float theta1, float k0, float* k1, float* out_q0x, float* out_q0y, float* out_q1x, float* out_q1y);
static unsigned char            ASSER_TRAJ_Profil_S_Curve(float * Vconsigne, float Distance, float VStart, float VEnd, float Amax, float Dmax, float gASR, float Pr, float Vr, unsigned char fSatPI);
#ifdef PIC32_BUILD
void                            ASSER_TRAJ_LogAsserPIC(char * keyWord, float Val1, float * pVal2, float * pVal3, float * pVal4, float * pVal5);
#define                         ASSER_TRAJ_LogAsserValPC(a, b)
#define                         ASSER_TRAJ_LogAsserMsgPC(a, b)
#else /* PIC32_BUILD */
#define                         ASSER_TRAJ_LogAsserPIC(a, b, c, d, e, f)
void                            ASSER_TRAJ_LogAsserValPC(char * keyWord, float Val);
void                            ASSER_TRAJ_LogAsserMsgPC(char * message, float Val);
#endif /* PIC32_BUILD */

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

    /* Calcul de la norme suivi trajectoire */
    NORME_BARRE_SUIVI_TRAJ = ECART_ROUE_LIBRE / 2.0;

    /* Initialisation de la position du robot */
    POS_InitialisationConfigRobot();
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_isDeplacement
 *
 *  \note  Test le deplacement
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
/*! \brief ASSER_TRAJ_isAngle
 *
 *  \note  Test si l'angle est valide
 *
 *  \param [in] angle              angle d'arrivee
 *
 *  \return  True (angle valide) or False (angle invalide)
 */
/**********************************************************************/
static unsigned char ASSER_TRAJ_isAngle(float angle)
{
    if (angle < ANGLE_INVALID)
    {
        return False;
    }
    else
    {
        return True;
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
 *  \param [in]   poseRobot               pose du robot etant donne le choix de la marche AVANT ou ARRIERE pour l'orientation du robot
 *  \param [out] vitessesConsignes    pointeur sur une structure pour fournir les vitesses longitudinale et de rotation, de consignes
 *
 *  \return         None
 */
/**********************************************************************/

extern void ASSER_TRAJ_AsservissementMouvementRobot(Pose poseRobot, VitessesRobotUnicycle *vitessesConsignes)
{
    float           diagonaleMatriceGain[2];
    float           gain[5];
    Pose            differentielleTemporellePoseReference;
    Pose            differentielleTemporellePoseReferenceRobot;
    Pose            P_robot;                                        /* Point fixe du robot a asservir a la trajectoire de consigne */
    Pose            erreurPoseCentreRobot;
    Pose            erreur_P;
    Vecteur         diff1BS, diff2BS;
    float           delta_distance                          = 0.0;
    float           parametrePositionSegmentTrajectoireAv   = 0.0;
    unsigned int    memo_segmentCourant                     = 0.0;
    unsigned int    segmentCourantAv;
    float           delta_distance_Av                       = 0.0;
    float           diffThetaCourantAv                      = 0.0;
    Pose            poseReferenceRobotAv;
    float           VitesseProfil                           = 0.0;
    float           distanceTotale_Profil                   = 0.0;
    float           distanceParcourue_Profil                = 0.0;
    unsigned char   iSegment                                = 1u;
    float           Vr;
    
    /* Log de valeurs */
    ASSER_TRAJ_LogAsserValPC("xRoueGauche", m_poseRobot.x + (ECART_ROUE_MOTRICE / 2.0) * cosf(m_poseRobot.angle + (PI / 2)));
    ASSER_TRAJ_LogAsserValPC("yRoueGauche", m_poseRobot.y + (ECART_ROUE_MOTRICE / 2.0) * sinf(m_poseRobot.angle + (PI / 2)));
    ASSER_TRAJ_LogAsserValPC("xRoueDroite", m_poseRobot.x + (ECART_ROUE_MOTRICE / 2.0) * cosf(m_poseRobot.angle - (PI / 2)));
    ASSER_TRAJ_LogAsserValPC("yRoueDroite", m_poseRobot.y + (ECART_ROUE_MOTRICE / 2.0) * sinf(m_poseRobot.angle - (PI / 2)));
    ASSER_TRAJ_LogAsserValPC("angle", m_poseRobot.angle);

    /* Initialisation des variables locales pour que leur contenu soit defini partout dans la fonction */
    differentielleTemporellePoseReference.x = 0.0;
    differentielleTemporellePoseReference.y = 0.0;
    differentielleTemporellePoseReference.angle = 0.0;
    differentielleTemporellePoseReferenceRobot.x = 0.0;
    differentielleTemporellePoseReferenceRobot.y = 0.0;
    differentielleTemporellePoseReferenceRobot.angle = 0.0;

    /* Initialisation de l'erreur */
    erreur_P.x = 0.0;
    erreur_P.y = 0.0;
    erreur_P.angle = 0.0;    
    erreurPoseCentreRobot.x = 0.0;
    erreurPoseCentreRobot.y = 0.0;

    /* Memorisation de l'erreur */
    memo_errDist = errDist;
    memo_errAngle = errAngle;

    /* Calcul de l'erreur */
    errDist = POS_ErreurDistance(poseRobot, chemin.posArrivee);
    errAngle = POS_ErreurOrientation(poseRobot, chemin.posArrivee);

    /* Test fin d'asser */
    if (compteurPeriode > 0)
    {
        if (ASSER_TRAJ_TestFinAsservissement(&chemin, errDist, memo_errDist, DIST_MIN, errAngle, memo_errAngle, ANGLE_MIN) == True)
        {
            if ((chemin.profilVitesse.distNormaliseeRestante * chemin.distance) < ((float)(DIST_MIN + 0.05)))
            {
                if (shuntTestFinAsser == False)
                {
                    Phase = 0;
                                
                    ASSER_Running = False;
                    
                    Vr = POS_GetVitesseRelle();
                
                    if (Vr > (VminMouv + EcartVitesse))
                    {
#ifdef PIC32_BUILD                        
                        TOOLS_LogFault(AsserPosErr, True, FLOAT, (float *)&Vr, True, "Asserv_traj : Vr > VminMouv a l'arrivee en position !");
#else /* PIC32_BUILD */
                        ASSER_TRAJ_LogAsserMsgPC("Asserv_traj : Vr > VminMouv a l'arrivee en position !", Vr);
#endif /* PIC32_BUILD */  
                    }
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
                Phase = 0;
            
                ASSER_Running = False;
                
                Vr = POS_GetVitesseRelle();
                
                if (POS_GetVitesseRelle() > (VminMouv + EcartVitesse))
                {
#ifdef PIC32_BUILD                        
                    TOOLS_LogFault(AsserPosErr, True, FLOAT, (float *)&Vr, True, "Asserv_traj : Vr > VminMouv a l'arrivee en position !");
#else /* PIC32_BUILD */
                    ASSER_TRAJ_LogAsserMsgPC("Asserv_traj : Vr > VminMouv a l'arrivee en position !", Vr);
#endif /* PIC32_BUILD */  
                }
            }
            else
            {
                shuntTestFinAsser = True;
            }
        }
    }

    /* distance de deplacement sur la periode */
    if (ASSER_TRAJ_isDeplacement(&chemin) == True)
    {
        delta_distance = cosf(poseReferenceRobot.angle) * (poseRobot.x - poseReferenceRobot.x) + sinf(poseReferenceRobot.angle) * (poseRobot.y - poseReferenceRobot.y);
    }
    else
    {
        delta_distance = NORME_BARRE_SUIVI_TRAJ * fabsf(POS_ModuloAngle(poseRobot.angle - memo_angleRobot));

        memo_angleRobot = poseRobot.angle;

        ASSER_TRAJ_LogAsserValPC("angleRobot", poseRobot.angle);
        ASSER_TRAJ_LogAsserValPC("angleRef", poseReference.angle);
    }

    if (delta_distance < 0.0)
    {
        delta_distance = 0.0;
    }

    chemin.profilVitesse.distance_parcourue += delta_distance;

    ASSER_TRAJ_LogAsserValPC("dist_parcourue",  chemin.profilVitesse.distance_parcourue);

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
            /* Debut de la dermination de la nouvelle position a suivre                     */
            /************************************************************/

            if (chemin.profilVitesse.p > 0)
            {
                ASSER_TRAJ_LogAsserValPC("nbdepas",  chemin.profilVitesse.pas_echantillon_distance);
                ASSER_TRAJ_LogAsserValPC("index_tab_vit",  g_index_tab_gabarit_vitesse);

                /* distance normalisee restant a parcourir */
                chemin.profilVitesse.distNormaliseeRestante -= (delta_distance / chemin.distance);
                
                memo_segmentCourant = segmentCourant;
                ASSER_TRAJ_ParcoursTrajectoire(&chemin, delta_distance, &segmentCourant, &parametrePositionSegmentTrajectoire, NULL);
                if (memo_segmentCourant != segmentCourant)
                {
                    ASSER_TRAJ_LogAsserValPC("periodeNewSeg", ASSER_TRAJ_GetCompteur());
                }
            }
            else
            {
                /* Commande de vitesse nulle, le point d'arrivee de consigne est atteint */
                parametrePositionSegmentTrajectoire = chemin.nbreSegments;
            }
            
            /**********************************************************/
            /* Fin de la dermination de la nouvelle position a suivre                      */
            /**********************************************************/

            if (ASSER_TRAJ_isDeplacement(&chemin) == True)
            {
                diff1BS = ASSER_TRAJ_DiffTrajectoire(chemin.segmentTrajBS, segmentCourant, parametrePositionSegmentTrajectoire);
                diff2BS = ASSER_TRAJ_Diff2Trajectoire(chemin.segmentTrajBS, segmentCourant, parametrePositionSegmentTrajectoire);
                
                chemin.profilVitesse.diffThetaCourant = ASSER_TRAJ_DiffThetaBSpline(diff1BS, diff2BS);
                poseReference = ASSER_TRAJ_TrajectoireRemorqueBS(chemin.segmentTrajBS, segmentCourant, parametrePositionSegmentTrajectoire, diff1BS, chemin.profilVitesse.diffThetaCourant, &poseReferenceRobot);

                parametrePositionSegmentTrajectoireAv = parametrePositionSegmentTrajectoire;
                segmentCourantAv = segmentCourant;

                distanceTotale_Profil = chemin.segmentTrajBS[segmentCourant].distance;
                distanceParcourue_Profil = chemin.profilVitesse.distance_parcourue;
                for (iSegment = 1; iSegment < segmentCourant; iSegment++)
                {
                    distanceParcourue_Profil -= chemin.segmentTrajBS[iSegment].distance;
                }
                ASSER_Running = ASSER_TRAJ_Profil_S_Curve(&VitesseProfil, distanceTotale_Profil, 0.0, 0.0, chemin.profilVitesse.Amax, chemin.profilVitesse.Dmax, Vitesse_Gain_ASR, distanceParcourue_Profil, POS_GetVitesseRelle(), (SaturationPIDflag | SaturationPIGflag));
                if (segmentCourant < chemin.nbreSegments)
                {
                    ASSER_Running = True;
                }
                //ASSER_Running = ASSER_TRAJ_Profil_S_Curve(&VitesseProfil, chemin.distance, 0.0, 0.0, chemin.profilVitesse.Amax, chemin.profilVitesse.Dmax, Vitesse_Gain_ASR, chemin.profilVitesse.distance_parcourue, POS_GetVitesseRelle(), (SaturationPIDflag | SaturationPIGflag));

                delta_distance_Av = VitesseProfil * TE;
                //delta_distance_Av = ASSER_TRAJ_GabaritVitesse_getVitesse_vs_Distance(chemin.profilVitesse.distance_parcourue) * TE;
                
                ASSER_TRAJ_ParcoursTrajectoire(&chemin, delta_distance_Av, &segmentCourantAv, &parametrePositionSegmentTrajectoireAv, NULL);
                
                ASSER_TRAJ_LogAsserValPC("segTrajAv",  parametrePositionSegmentTrajectoireAv);

                diff1BS = ASSER_TRAJ_DiffTrajectoire(chemin.segmentTrajBS, segmentCourantAv, parametrePositionSegmentTrajectoireAv);
                diff2BS = ASSER_TRAJ_Diff2Trajectoire(chemin.segmentTrajBS, segmentCourantAv, parametrePositionSegmentTrajectoireAv);

                diffThetaCourantAv = ASSER_TRAJ_DiffThetaBSpline(diff1BS, diff2BS);
                poseReferenceAv = ASSER_TRAJ_TrajectoireRemorqueBS(chemin.segmentTrajBS, segmentCourantAv, parametrePositionSegmentTrajectoireAv, diff1BS, diffThetaCourantAv, &poseReferenceRobotAv);
            }
            else /* if (chemin.mouvement == ROTATION) */
            {
                poseReference = ASSER_TRAJ_TrajectoireRotation(&(chemin.rotation), parametrePositionSegmentTrajectoire);

                ASSER_TRAJ_LogAsserValPC("thetaPoseReference",  poseReference.angle);

                parametrePositionSegmentTrajectoireAv = parametrePositionSegmentTrajectoire;
                segmentCourantAv = segmentCourant;

                ASSER_Running = ASSER_TRAJ_Profil_S_Curve(&VitesseProfil, chemin.distance, 0.0, 0.0, chemin.profilVitesse.Amax, chemin.profilVitesse.Dmax, Vitesse_Gain_ASR, chemin.profilVitesse.distance_parcourue, POS_GetVitesseRelle(), (SaturationPIDflag | SaturationPIGflag));
                delta_distance_Av = VitesseProfil * TE;
                //delta_distance_Av = ASSER_TRAJ_GabaritVitesse_getVitesse_vs_Distance(chemin.profilVitesse.distance_parcourue) * TE;
                
                ASSER_TRAJ_ParcoursTrajectoire(&chemin, delta_distance_Av, &segmentCourantAv, &parametrePositionSegmentTrajectoireAv, NULL);

                poseReferenceAv = ASSER_TRAJ_TrajectoireRotation(&(chemin.rotation), parametrePositionSegmentTrajectoireAv);
            }
            
            ASSER_TRAJ_LogAsserValPC("val_tab_vit",  (delta_distance_Av / TE));

            differentielleTemporellePoseReference = ASSER_TRAJ_DiffTemporelleTrajectoire(poseReference, poseReferenceAv, TE);

            differentielleTemporellePoseReferenceRobot = ASSER_TRAJ_DiffTemporelleTrajectoire(poseReferenceRobot, poseReferenceRobotAv, TE);
        }
        else
        {
            vitesse_profil_consigne = 0.0;

#ifdef PIC32_BUILD
            /* Desactivation test antiblockage avant de chercher precisement le point d'arrivee pour eviter de passser en evitement */
            EVIT_ProcheFinAsser = True;
#endif /* PIC32_BUILD */

            Phase = 0;
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
            P_robot.x = poseRobot.x + (NORME_BARRE_SUIVI_TRAJ * cosf(poseRobot.angle));
            P_robot.y = poseRobot.y + (NORME_BARRE_SUIVI_TRAJ * sinf(poseRobot.angle));            
            P_robot.angle = poseRobot.angle;

            /* Calcul du vecteur d'erreur */
            erreur_P = ASSER_TRAJ_ErreurPose(P_robot, poseReference);

            /* Loi de commande par retour d'etat SANS orientation*/
            diagonaleMatriceGain[0] = gainRotation1;
            diagonaleMatriceGain[1] = gainRotation2;
            
            *vitessesConsignes = ASSER_TRAJ_RetourDetat(erreur_P, poseReference, differentielleTemporellePoseReference, NORME_BARRE_SUIVI_TRAJ, diagonaleMatriceGain);

            vitessesConsignes->longitudinale = 0.0;
        }
    }
    else
    {
        vitessesConsignes->longitudinale = 0.0;
        vitessesConsignes->rotation = 0.0;
        
        vitesse_profil_consigne = 0.0;
        
        chemin.profilVitesse.p = 0;

        Phase = 0;
    }

    /* Log de valeurs */
    ASSER_TRAJ_LogAsserValPC("xCCourant",  ASSER_TRAJ_Trajectoire(chemin.segmentTrajBS, segmentCourant, parametrePositionSegmentTrajectoire).x);
    ASSER_TRAJ_LogAsserValPC("xPoseReferenceRobot",  poseReferenceRobot.x);
    ASSER_TRAJ_LogAsserValPC("yPoseReferenceRobot",  poseReferenceRobot.y);
    ASSER_TRAJ_LogAsserValPC("distNormaliseeRestante",  chemin.profilVitesse.distNormaliseeRestante);
    ASSER_TRAJ_LogAsserValPC("orientationPoseReferenceRobot",  poseReferenceRobot.angle);
    ASSER_TRAJ_LogAsserValPC("consRotation",  vitessesConsignes->rotation);
    ASSER_TRAJ_LogAsserValPC("parametrePositionSegmentTrajectoire",  parametrePositionSegmentTrajectoire);
    ASSER_TRAJ_LogAsserValPC("segmentCourant",  segmentCourant);
    ASSER_TRAJ_LogAsserValPC("VitesseProfil", VitesseProfil);
    ASSER_TRAJ_LogAsserValPC("VgASR", VgASR);
    ASSER_TRAJ_LogAsserValPC("Phase", (Phase/10.0));
    ASSER_TRAJ_LogAsserValPC("fFin", (ASSER_Running/10.0));
}


/**********************************************************************/
/*! \brief ASSER_TRAJ_InitialisationTrajectoire
 *
 *  \note   Initialisation d'une structure trajectoire definit en global dans le module ASSER_TRAJ
 *
 *  \param [in]     poseRobot       pose courante du robot integrant le choix de la marche (AVANT ou ARRIERE)
 *  \param [in]     point               pointeur du tableau des points imposes du chemin dont le point a atteindre en dernier
 *  \param [in]     nbrePts           nombre de points intermediaires du chemin par lesquelles passer +1 pour le point d'arrvee
 *  \param [in]     mouvement     type de mouvement a executer: un deplacement (DEPLACEMENT), une pure rotation (ROTATION) ou un dÃ©placement qui finit en ligne droite (DEPLACEMENT_LIGNE_DROITE)
 *
 *  \return           None
 */
/**********************************************************************/
extern void ASSER_TRAJ_InitialisationTrajectoire(Pose poseRobot, PtTraj * point, unsigned int nbrePts, unsigned int mouvement, float angle_rad)
{
    Vecteur                 diff1BS, diff2BS, pos_ti, pos_tj;
    float                   k0, k1, theta0, theta1, C, prec_x, prec_y, cos_theta1, sin_theta1;
    unsigned int            iseg_portion, iSegment;
    segmentTrajectoireBS *  segmentTraj;
    float                   lenSeg, angle_fin_rotation;
#ifndef PIC32_BUILD
    Pose                    poseTest;
    Vecteur                 diffTest, diff2Test;
    float                   diffTheta;
    unsigned int            intT;
    float                   sommeMask;
#endif /* PIC32_BUILD */

    memo_angleRobot = m_poseRobot.angle;

    chemin.mouvement = mouvement;
    chemin.nbreSegments = nbrePts;

    parametrePositionSegmentTrajectoire = 0.0;
    segmentCourant = 1;
    shuntTestFinAsser = False;

    /* Liste des points terminee */
    if (ASSER_TRAJ_isDeplacement(&chemin) == True)
    {
        if (ASSER_TRAJ_isAngle(angle_rad) == True)
        {
            if (m_sensMarcheMouvement == MARCHE_ARRIERE)
            {
                angle_rad = POS_ModuloAngle(angle_rad + PI);
            }
        }

        /* Position a atteindre (condition d'arret du test de fin d'asservissment) */
        chemin.posArrivee.x = CONVERT_DISTANCE(point[(nbrePts - 1)].x);
        chemin.posArrivee.y = CONVERT_DISTANCE(point[(nbrePts - 1)].y);

        /* Configuration du profil de vitesse */
        chemin.profilVitesse.vmax = POS_GetConsVitesseMax();                /* UMAX * GAIN_STATIQUE_MOTEUR */

        if (mouvement == DEPLACEMENT_LIGNE_DROITE)
        {
            chemin.segmentTrajBS[1].aix = (CONVERT_DISTANCE(point[0].x) - poseRobot.x);
            chemin.segmentTrajBS[1].aiy = (CONVERT_DISTANCE(point[0].y) - poseRobot.y);
            chemin.segmentTrajBS[1].bix = poseRobot.x;
            chemin.segmentTrajBS[1].biy = poseRobot.y;
            chemin.segmentTrajBS[1].distance = sqrt(SQUARE(chemin.segmentTrajBS[1].aix) + SQUARE(chemin.segmentTrajBS[1].aiy));
            chemin.distance = chemin.segmentTrajBS[1].distance;
            ASSER_TRAJ_LogAsserValPC("distance", chemin.distance);
        }
        else
        {
            lenSeg = ASSER_TRAJ_LongueurSegment(poseRobot.x, poseRobot.y, CONVERT_DISTANCE(point[0].x), CONVERT_DISTANCE(point[0].y));
            C = C_init * lenSeg;
            k0 = k0_init * lenSeg;

            chemin.segmentTrajBS[0].bx = poseRobot.x;
            chemin.segmentTrajBS[0].by = poseRobot.y;
            chemin.segmentTrajBS[1].bx = CONVERT_DISTANCE(point[0].x);
            chemin.segmentTrajBS[1].by = CONVERT_DISTANCE(point[0].y);

            theta0 = poseRobot.angle;
            
            chemin.segmentTrajBS[0].ax = k0 * cosf(theta0);            
            chemin.segmentTrajBS[0].ay = k0 * sinf(theta0);            

            k1 = k0;
            theta1 = ASSER_TRAJ_CalculTheta1(1, nbrePts, point, angle_rad, poseRobot.x, poseRobot.y);
            
            ASSER_TRAJ_LogAsserValPC("theta1", theta1);
            ASSER_TRAJ_LogAsserValPC("angle_rad", angle_rad);
            
            chemin.segmentTrajBS[1].ax = k1 * cosf(theta1);            
            chemin.segmentTrajBS[1].ay = k1 * sinf(theta1);

            ASSER_TRAJ_LogAsserValPC("pti", C);

            ASSER_TRAJ_SolveSegment_v2(poseRobot.x,
                                    poseRobot.y,
                                    theta0,
                                    CONVERT_DISTANCE(point[0].x),
                                    CONVERT_DISTANCE(point[0].y),
                                    theta1,
                                    k0,
                                    &k1,
                                    &chemin.segmentTrajBS[1].qx[0],
                                    &chemin.segmentTrajBS[1].qy[0],
                                    &chemin.segmentTrajBS[1].qx[1],
                                    &chemin.segmentTrajBS[1].qy[1]);

            ASSER_TRAJ_LogAsserValPC("q", chemin.segmentTrajBS[1].qx[0]);
            ASSER_TRAJ_LogAsserValPC("q", chemin.segmentTrajBS[1].qy[0]);
            ASSER_TRAJ_LogAsserValPC("q", chemin.segmentTrajBS[1].qx[1]);
            ASSER_TRAJ_LogAsserValPC("q", chemin.segmentTrajBS[1].qy[1]);
            ASSER_TRAJ_LogAsserValPC("q", 3000.0);

            iSegment = 1;
            iseg_portion = iSegment - 1;
            segmentTraj = chemin.segmentTrajBS;
            pos_ti = ASSER_TRAJ_PortionEnd(segmentTraj[iseg_portion].bx, segmentTraj[iseg_portion].by, segmentTraj[iseg_portion].ax, segmentTraj[iseg_portion].ay, segmentTraj[iSegment].qx[0], segmentTraj[iSegment].qy[0]);

            iseg_portion = iSegment;
            segmentTraj = chemin.segmentTrajBS;
            pos_tj = ASSER_TRAJ_PortionEnd(segmentTraj[iseg_portion].bx, segmentTraj[iseg_portion].by, -segmentTraj[iseg_portion].ax, -segmentTraj[iseg_portion].ay, segmentTraj[iSegment].qx[1], segmentTraj[iSegment].qy[1]);

            chemin.segmentTrajBS[iSegment].aix = (pos_tj.x - pos_ti.x) / (1.0 - (2.0 * ti));
            chemin.segmentTrajBS[iSegment].aiy = (pos_tj.y - pos_ti.y) / (1.0 - (2.0 * ti));
            chemin.segmentTrajBS[iSegment].bix = pos_ti.x;
            chemin.segmentTrajBS[iSegment].biy = pos_ti.y;

            ASSER_TRAJ_LogAsserValPC("pti", CONVERT_DISTANCE(point[0].x));
            ASSER_TRAJ_LogAsserValPC("pti", CONVERT_DISTANCE(point[0].y));
            ASSER_TRAJ_LogAsserValPC("pti", theta1);
            ASSER_TRAJ_LogAsserValPC("pti", k0);
            ASSER_TRAJ_LogAsserValPC("pti", k1);
            ASSER_TRAJ_LogAsserValPC("pti", ti);
            ASSER_TRAJ_LogAsserValPC("pti", ((segmentTraj[0].ax * ti) + segmentTraj[0].bx));
            ASSER_TRAJ_LogAsserValPC("pti", pos_ti.x);
            ASSER_TRAJ_LogAsserValPC("pti", pos_ti.y);
            ASSER_TRAJ_LogAsserValPC("pti", pos_tj.x);
            ASSER_TRAJ_LogAsserValPC("pti", pos_tj.y);

            prec_x = CONVERT_DISTANCE(point[0].x);
            prec_y = CONVERT_DISTANCE(point[0].y);

            for(iSegment = 2; iSegment <= nbrePts; iSegment++)
            {
                chemin.segmentTrajBS[iSegment].bx = CONVERT_DISTANCE(point[(iSegment - 1)].x);
                chemin.segmentTrajBS[iSegment].by = CONVERT_DISTANCE(point[(iSegment - 1)].y);

                lenSeg = ASSER_TRAJ_LongueurSegment(prec_x, prec_y, CONVERT_DISTANCE(point[(iSegment - 1)].x), CONVERT_DISTANCE(point[(iSegment - 1)].y));
                C = C_init * lenSeg;
                k0 = k1;
                k1 = k0_init * lenSeg;

                /* Calcul de l'angle au point intermediaire */
                theta0 = theta1;

                theta1 = ASSER_TRAJ_CalculTheta1(iSegment, nbrePts, point, angle_rad, prec_x, prec_y);
                
                cos_theta1 = cosf(theta1);                
                sin_theta1 = sinf(theta1);                

                chemin.segmentTrajBS[iSegment].ax = k1 * cos_theta1;
                chemin.segmentTrajBS[iSegment].ay = k1 * sin_theta1;

                ASSER_TRAJ_SolveSegment_v2(prec_x,
                                        prec_y,
                                        theta0,
                                        CONVERT_DISTANCE(point[(iSegment - 1)].x),
                                        CONVERT_DISTANCE(point[(iSegment - 1)].y),
                                        theta1,
                                        k0,
                                        &k1,
                                        &chemin.segmentTrajBS[iSegment].qx[0],
                                        &chemin.segmentTrajBS[iSegment].qy[0],
                                        &chemin.segmentTrajBS[iSegment].qx[1],
                                        &chemin.segmentTrajBS[iSegment].qy[1]);

                ASSER_TRAJ_LogAsserValPC("q2", k0);
                ASSER_TRAJ_LogAsserValPC("q2", k1);
                ASSER_TRAJ_LogAsserValPC("q2", chemin.segmentTrajBS[iSegment].qx[0]);
                ASSER_TRAJ_LogAsserValPC("q2", chemin.segmentTrajBS[iSegment].qy[0]);
                ASSER_TRAJ_LogAsserValPC("q2", chemin.segmentTrajBS[iSegment].qx[1]);
                ASSER_TRAJ_LogAsserValPC("q2", chemin.segmentTrajBS[iSegment].qy[1]);


                iseg_portion = iSegment - 1;
                segmentTraj = chemin.segmentTrajBS;ASSER_TRAJ_LogAsserValPC("distance", chemin.distance);
                pos_ti = ASSER_TRAJ_PortionEnd(segmentTraj[iseg_portion].bx, segmentTraj[iseg_portion].by, segmentTraj[iseg_portion].ax, segmentTraj[iseg_portion].ay, segmentTraj[iSegment].qx[0], segmentTraj[iSegment].qy[0]);

                iseg_portion = iSegment;
                segmentTraj = chemin.segmentTrajBS;
                pos_tj = ASSER_TRAJ_PortionEnd(segmentTraj[iseg_portion].bx, segmentTraj[iseg_portion].by, -segmentTraj[iseg_portion].ax, -segmentTraj[iseg_portion].ay, segmentTraj[iSegment].qx[1], segmentTraj[iSegment].qy[1]);

                chemin.segmentTrajBS[iSegment].aix = (pos_tj.x - pos_ti.x) / (1.0 - (2.0 * ti));
                chemin.segmentTrajBS[iSegment].aiy = (pos_tj.y - pos_ti.y) / (1.0 - (2.0 * ti));
                chemin.segmentTrajBS[iSegment].bix = pos_ti.x;
                chemin.segmentTrajBS[iSegment].biy = pos_ti.y;

                prec_x = CONVERT_DISTANCE(point[(iSegment - 1)].x);
                prec_y = CONVERT_DISTANCE(point[(iSegment - 1)].y);
            }
        }

#ifndef PIC32_BUILD

        /* Enregistrement pour affichage de x(t) et de y(t) */
        for (intT = 0; intT < (100 * nbrePts); intT++)
        {            
            poseTest = ASSER_TRAJ_Trajectoire(chemin.segmentTrajBS, floor( (((float)intT)*((float)1)) / 100.0) + 1,  ((float)intT) / 100.0 );

            ASSER_TRAJ_LogAsserValPC("def_i", (floor( (((float)intT)*((float)1)) / 100.0) + 1));
            ASSER_TRAJ_LogAsserValPC("def_xTraj", poseTest.x);
            ASSER_TRAJ_LogAsserValPC("def_yTraj", poseTest.y);
            ASSER_TRAJ_LogAsserValPC("def_angleTraj", poseTest.angle);

            diffTest = ASSER_TRAJ_DiffTrajectoire(chemin.segmentTrajBS, floor( (((float)intT)*((float)1)) / 100.0) + 1, ((float)intT) / 100.0);

            ASSER_TRAJ_LogAsserValPC("def_diff_xTraj", diffTest.x);
            ASSER_TRAJ_LogAsserValPC("def_diff_yTraj", diffTest.y);

            diff2Test = ASSER_TRAJ_Diff2Trajectoire(chemin.segmentTrajBS, floor( (((float)intT)*((float)1)) / 100.0) + 1, ((float)intT) / 100.0);
            ASSER_TRAJ_LogAsserValPC("def_diff2_xTraj", diff2Test.x);
            ASSER_TRAJ_LogAsserValPC("def_diff2_yTraj", diff2Test.y);

            diffTheta = ASSER_TRAJ_DiffThetaBSpline(diffTest, diff2Test);
            ASSER_TRAJ_LogAsserValPC("def_diff_ThetaTraj", diffTheta);
        }

#endif /*PIC32_BUILD*/

        /* Determination des distances des segments de trajectoire, et de la distance totale */

        if (mouvement != DEPLACEMENT_LIGNE_DROITE)
        {
            chemin.distance = 0.0;
            for (iSegment = 1; iSegment <= nbrePts; iSegment++)
            {
                ASSER_TRAJ_DistanceTrajectoire(chemin.segmentTrajBS, iSegment);

                chemin.distance += chemin.segmentTrajBS[iSegment].distance;

                ASSER_TRAJ_LogAsserValPC("distance_seg", chemin.segmentTrajBS[iSegment].distance);
            }
        }
        else
        {
            ASSER_TRAJ_LogAsserValPC("distance_seg", chemin.distance);
        }

        /* Calcul de la premiere pose de la trajectoire de consigne */
        diff1BS = ASSER_TRAJ_DiffTrajectoire(chemin.segmentTrajBS, 1, 0.0);
        diff2BS = ASSER_TRAJ_Diff2Trajectoire(chemin.segmentTrajBS, 1, 0.0);
        
        chemin.profilVitesse.diffThetaCourant = ASSER_TRAJ_DiffThetaBSpline(diff1BS, diff2BS);

        poseReference = ASSER_TRAJ_TrajectoireRemorqueBS(chemin.segmentTrajBS, 1, 0.0, diff1BS, chemin.profilVitesse.diffThetaCourant, &poseReferenceRobot);
    }
    else
    {
        /* Position a atteindre (condition d'arret du test de fin d'asservissment) */
        angle_fin_rotation = POS_ModuloAngle(angle_rad);
        
        chemin.posArrivee.x = poseRobot.x + NORME_BARRE_SUIVI_TRAJ * cosf(angle_fin_rotation);        
        chemin.posArrivee.y = poseRobot.y + NORME_BARRE_SUIVI_TRAJ * sinf(angle_fin_rotation);        
        
        ASSER_TRAJ_LogAsserValPC("angleFinRotation", angle_fin_rotation);

        /* Configuration du profil de vitesse */
        chemin.profilVitesse.vmax = POS_GetConsVitesseAngulaireMax() * NORME_BARRE_SUIVI_TRAJ;

        chemin.rotation.poseDepartRobot = poseRobot;
        chemin.nbreSegments = 1;
        chemin.rotation.angle = POS_ModuloAngle(angle_fin_rotation - poseRobot.angle);

        ASSER_TRAJ_LogAsserValPC("plageAngleRotation", chemin.rotation.angle);

        /* Calcul de la premiere pose de la trajectoire de consigne */
        poseReference = ASSER_TRAJ_TrajectoireRotation(&(chemin.rotation), 0.0);

        chemin.distance = (fabsf(chemin.rotation.angle) * NORME_BARRE_SUIVI_TRAJ);
        ASSER_TRAJ_LogAsserValPC("distance_seg", chemin.distance);
    }

    /* Garantie que la distance totale de la trajectoire est positive et non nulle */
    if (chemin.distance < ((float)1e-3))
    {
        chemin.distance = ((float)1e-3);
    }
    ASSER_TRAJ_LogAsserValPC("distance", chemin.distance);

    /* Calcul du gabarit du profil de vitesse */
    chemin.profilVitesse.distance_parcourue = 0.0;

    /* Determination des coefficients a des profils de vitesse d'acceleration et de decceleration */
    chemin.profilVitesse.vitesse_courante = vitesse_profil_consigne;

    chemin.profilVitesse.p = 1;                                                                                 /* autorisation du profil de vitesse */
    chemin.profilVitesse.distNormaliseeRestante = 1.0;                                                          /* -> la distance totale normalisee */

    /* Initialisation du profil de vitessee */
    chemin.profilVitesse.pas_echantillon_distance = (chemin.distance * 1000.0) / ((float)(TAILLE_TAB_GABARIT_VITESSE - 1));    
    chemin.profilVitesse.pas_echantillon_distance = (floorf(chemin.profilVitesse.pas_echantillon_distance) + 1.0) / 1000.0;

    ASSER_TRAJ_LogAsserValPC("pas_ech", chemin.profilVitesse.pas_echantillon_distance);
    
    g_index_tab_gabarit_vitesse = 0;
    g_index_tab_gabarit_acceleration = 0;

    /* Initialisation du profil de vitesse 2012 */
    chemin.profilVitesse.Amax = A_MAX * Ratio_Acc;
    chemin.profilVitesse.Dmax = D_MAX * Ratio_Decc;
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
/*! \brief ASSER_TRAJ_PortionEnd
 *
 *  \note   
 *
 *  \param [in]     bx 
 *  \param [in]     by
 *  \param [in]     ax
 *  \param [in]     ay 
 *  \param [in]     qx
 *  \param [in]     qy
 *
 *  \return      
 */
/**********************************************************************/
static Vecteur ASSER_TRAJ_PortionEnd(float bx, float by, float ax, float ay, float qx, float qy)
{
    Vecteur pos_ti;
    float   ti_3, ti_2;

    ti_3 = CUBE(ti);
    ti_2 = SQUARE(ti);
    
    pos_ti.x = (((- qx) / (6.0 * ti)) * ti_3) + ((qx / 2.0) * ti_2) + (ax * ti) + bx;
    pos_ti.y = (((- qy) / (6.0 * ti)) * ti_3) + ((qy / 2.0) * ti_2) + (ay * ti) + by;

    return pos_ti;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_SolveSegment_v2
 *
 *  \note   
 *
 *  \param [in]     x0
 *  \param [in]     y0
 *  \param [in]     theta0 
 *  \param [in]     x1
 *  \param [in]     y1
 *  \param [in]     theta1
 *  \param [in]     k0
 *  \param [in]     k1
 *  \param [in]     out_q0x
 *  \param [in]     out_q0y
 *  \param [in]     out_q1x 
 *  \param [in]     out_q1y
 *
 *  \return      
 */
/**********************************************************************/
static void ASSER_TRAJ_SolveSegment_v2(float x0, float y0, float theta0, float x1, float y1, float theta1, float k0, float* k1, float* out_q0x, float* out_q0y, float* out_q1x, float* out_q1y)
{
    float F2, F3, F4, F5;

    F2 = (- k0 * cosf(theta0)) + (*k1 * cosf(theta1));    
    F3 = (- k0 * sinf(theta0)) + (*k1 * sinf(theta1));    
    F4 = (((- *k1 * cosf(theta1)) - (k0 * cosf(theta0))) * ti) + x1 - x0;    
    F5 = (((- *k1 * sinf(theta1)) - (k0 * sinf(theta0))) * ti) + y1 - y0;    

    *out_q0x = ((2.0 * ti * F2)/(3.0 * (1.0 - (2.0 * ti))) + F4/(1.0 - (2.0 * ti)) - (k0 * cosf(theta0))) / ((ti / 2.0)  + ((2.0 * pow(ti, 2.0)))/(3.0 *( 1.0 - (2.0 * ti))));
    *out_q0y = ((2.0 * ti * F3)/(3.0 * (1.0 - (2.0 * ti))) + F5/(1.0 - (2.0 * ti)) - (k0 * sinf(theta0))) / ((ti / 2.0)  + ((2.0 * pow(ti, 2.0)))/(3.0 * (1.0 - (2.0 * ti))));

    *out_q1x = ((2.0 * F2) / ti) - (*out_q0x);
    *out_q1y = ((2.0 * F3) / ti) - (*out_q0y);
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_CalculTheta1
 *
 *  \note   
 *
 *  \param [in]     iSegment 
 *  \param [in]     nbrePts
 *  \param [in]     point
 *  \param [in]     angle_rad 
 *  \param [in]     prec_x
 *  \param [in]     prec_y
 *
 *  \return      
 */
/**********************************************************************/
static float ASSER_TRAJ_CalculTheta1(unsigned int iSegment, unsigned int nbrePts, PtTraj* point, float angle_rad, float prec_x, float prec_y)
{
    float theta1, thetaS1, thetaS2;

    if (iSegment < nbrePts)
    {        
        thetaS1 = atan2f((CONVERT_DISTANCE(point[(iSegment - 1)].y) - prec_y), (CONVERT_DISTANCE(point[(iSegment - 1)].x) - prec_x));        

        /* Workarround library math */
        if ((CONVERT_DISTANCE(point[(iSegment - 1)].y) - prec_y) < 0.0)
        {
            if (thetaS1 > 0.0)
            {
                thetaS1 = - thetaS1;
            }
        }
        else
        {
            if (thetaS1 < 0.0)
            {
                thetaS1 = - thetaS1;
            }
        }

        thetaS2 = atan2f((CONVERT_DISTANCE(point[iSegment].y) - CONVERT_DISTANCE(point[(iSegment - 1)].y)), (CONVERT_DISTANCE(point[iSegment].x) - CONVERT_DISTANCE(point[(iSegment - 1)].x)));

        /* Workarround library math */
        if ((CONVERT_DISTANCE(point[iSegment].y) - CONVERT_DISTANCE(point[(iSegment - 1)].y)) < 0.0)
        {
            if (thetaS2 > 0.0)
            {
                thetaS2 = - thetaS2;
            }
        }
        else
        {
            if (thetaS2 < 0.0)
            {
                thetaS2 = - thetaS2;
            }
        }

        theta1 = thetaS1 - ((thetaS1 - thetaS2) / 2.0);
    }
    else
    {
        if (ASSER_TRAJ_isAngle(angle_rad) == True)
        {
            theta1 = angle_rad;
        }
        else
        {            
            theta1 = atan2f((CONVERT_DISTANCE(point[(nbrePts - 1)].y) - prec_y), (CONVERT_DISTANCE(point[(nbrePts - 1)].x) - prec_x));            

            /* Workarround library math */
            if ((CONVERT_DISTANCE(point[(nbrePts - 1)].y) - prec_y) < 0.0)
            {
                if (theta1 > 0.0)
                {
                    theta1 = - theta1;
                }
            }
            else
            {
                if (theta1 < 0.0)
                {
                    theta1 = - theta1;
                }
            }
        }
    }

    return theta1;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_LongueurSegment
 *
 *  \note   
 *
 *  \param [in]     x0 
 *  \param [in]     y0
 *  \param [in]     x1
 *  \param [in]     y1
 *
 *  \return      
 */
/**********************************************************************/
static float ASSER_TRAJ_LongueurSegment(float x0, float y0, float x1, float y1)
{   
    return sqrtf(SQUARE(x1 - x0) + SQUARE(y1 - y0));
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
static Pose ASSER_TRAJ_TrajectoireRemorqueBS(segmentTrajectoireBS * segmentTraj, unsigned int iSegment, float t, Vecteur diff1BS, float diffThetaBSRobot, Pose * poseTrajRobot)
{
    Pose    poseTraj;

    poseTraj = ASSER_TRAJ_Trajectoire(segmentTraj, iSegment, t);
    
    *poseTrajRobot = poseTraj;

    poseTraj.x = poseTraj.x + (NORME_BARRE_SUIVI_TRAJ * cosf(poseTraj.angle));    
    poseTraj.y = poseTraj.y + (NORME_BARRE_SUIVI_TRAJ * sinf(poseTraj.angle));    
    
    poseTraj.angle = atan2f((diff1BS.y + (NORME_BARRE_SUIVI_TRAJ * diffThetaBSRobot * cosf(poseTraj.angle))), (diff1BS.x - (NORME_BARRE_SUIVI_TRAJ * diffThetaBSRobot * sinf(poseTraj.angle))));    

    /* Workarround library math */
    if ((diff1BS.y + (NORME_BARRE_SUIVI_TRAJ * diffThetaBSRobot * cosf(poseTraj.angle))) < 0.0)
    {
        if (poseTraj.angle > 0.0)
        {
            poseTraj.angle = - poseTraj.angle;
        }
    }
    else
    {
        if (poseTraj.angle < 0.0)
        {
            poseTraj.angle = - poseTraj.angle;
        }
    }

    ASSER_TRAJ_LogAsserValPC("refX", poseTraj.x);

    return poseTraj;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_Trajectoire
 *
 *  \note   equation parametrique de type B-Spline
 *
 *  \param [in]     segmentTraj  pointeur de structure definissant la trajectoire
 *  \param [in]     iSegment       segment
 *  \param [in]     t                   parametre de la courbe de Bezier [0; 1]: 0-> depart, 1-> arrivee.
 *
 *  \return           Pose   pose de la courbe fonction de l'argument t
 */
/**********************************************************************/
extern Pose ASSER_TRAJ_Trajectoire(segmentTrajectoireBS * segmentTraj, unsigned int iSegment, float t)
{
    return ASSER_TRAJ_TrajectoireBSpline(segmentTraj, iSegment, (t  - ((float)(iSegment - 1))));
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_TrajectoireBSpline
 *
 *  \note   Expression parametrique de type B-Spline de la position
 *
 *  \param [in]     segmentTraj  pointeur de structure definissant la trajectoire
 *  \param [in]     iSegment       segment
 *  \param [in]     t                   parametre de la courbe de Bezier [0; 1]: 0-> depart, 1-> arrivee.
 *
 *  \return           Pose              pose de la courbe de Bezier fonction de l'argument t
 */
/**********************************************************************/
static Pose ASSER_TRAJ_TrajectoireBSpline(segmentTrajectoireBS * segmentTraj, unsigned int iSegment, float t)
{
    float           theta_P, tl;
    Vecteur         derivPositionTraj;
    Pose            poseTraj;
    unsigned int    iseg_portion;
    char            num_portion;
    float           sign_a_portion      = 1.0;
    /* var temporaire de resultats de calcul de puissance */
    float           tl_2, tl_3;

    if (t < ti)
    {
        num_portion = 1;
        tl = t;
        iseg_portion = iSegment - 1;
        sign_a_portion = 1.0;
    }
    else
    {
        if (t > (1.0 - ti))
        {
            num_portion = 3;
            tl = 1.0 - t;
            iseg_portion = iSegment;
            sign_a_portion = -1.0;
        }
        else
        {
            num_portion = 2;
            iseg_portion = iSegment;
            tl = t - ti;
        }
    }

    if (num_portion != 2)
    {
        tl_3 = CUBE(tl);
        tl_2 = SQUARE(tl);

        poseTraj.x = (((- segmentTraj[iSegment].qx[(iseg_portion - iSegment + 1)]) / (6.0 * ti)) * tl_3) + ((segmentTraj[iSegment].qx[(iseg_portion - iSegment + 1)] / 2.0) * tl_2) + (sign_a_portion * segmentTraj[iseg_portion].ax * tl) + segmentTraj[iseg_portion].bx;
        poseTraj.y = (((- segmentTraj[iSegment].qy[(iseg_portion - iSegment + 1)]) / (6.0 * ti)) * tl_3) + ((segmentTraj[iSegment].qy[(iseg_portion - iSegment + 1)] / 2.0) * tl_2) + (sign_a_portion * segmentTraj[iseg_portion].ay * tl) + segmentTraj[iseg_portion].by;
    }
    else
    {
        poseTraj.x = (segmentTraj[iSegment].aix * tl) + segmentTraj[iSegment].bix;
        poseTraj.y = (segmentTraj[iSegment].aiy * tl) + segmentTraj[iSegment].biy;
    }

    if (chemin.mouvement == DEPLACEMENT_LIGNE_DROITE)
    {
        poseTraj.x = (segmentTraj[1].aix * t) + segmentTraj[1].bix;
        poseTraj.y = (segmentTraj[1].aiy * t) + segmentTraj[1].biy;
    }

    /* Calcul de l'orientation de la trajectoire a t */
    derivPositionTraj = ASSER_TRAJ_DiffCourbeBSpline(segmentTraj, iSegment, t);  
    
    theta_P = atan2f(derivPositionTraj.y, derivPositionTraj.x);    

    /* Workarround library math */
    if (derivPositionTraj.y < 0.0)
    {
        if (theta_P > 0.0)
        {
            theta_P = - theta_P;
        }
    }
    else
    {
        if (theta_P < 0.0)
        {
            theta_P = - theta_P;
        }
    }

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
static Vecteur ASSER_TRAJ_DiffCourbeBSpline(segmentTrajectoireBS * segmentTraj, unsigned int iSegment, float t)
{
    Vecteur         derivP;
    float           tl;
    unsigned int    iseg_portion;
    char            num_portion;
    float           sign_a_portion = 1.0;
    /* var temporaire de resultats de calcul de puissance */
    float           tl_2;

    if (t < ti)
    {
        num_portion = 1;
        tl = t;
        iseg_portion = iSegment - 1;
        sign_a_portion = 1.0;
    }
    else
    {
        if (t > (1.0 - ti))
        {
            num_portion = 3;
            tl = 1.0 - t;
            iseg_portion = iSegment;
            sign_a_portion = -1.0;
        }
        else
        {
            num_portion = 2;
            iseg_portion = iSegment;
            tl = t - ti;
        }
    }

    if (num_portion != 2)
    {
        tl_2 = SQUARE(tl);
        derivP.x = ((- segmentTraj[iSegment].qx[iseg_portion - iSegment + 1]) / (2.0 * ti)) * tl_2 + segmentTraj[iSegment].qx[(iseg_portion - iSegment + 1)] * tl + sign_a_portion * segmentTraj[iseg_portion].ax;
        derivP.y = ((- segmentTraj[iSegment].qy[iseg_portion - iSegment + 1]) / (2.0 * ti)) * tl_2 + segmentTraj[iSegment].qy[(iseg_portion - iSegment + 1)] * tl + sign_a_portion * segmentTraj[iseg_portion].ay;

        if (num_portion == 3)
        {
            derivP.x = - derivP.x;
            derivP.y = - derivP.y;
        }
    }
    else
    {
        derivP.x = segmentTraj[iSegment].aix;
        derivP.y = segmentTraj[iSegment].aiy;
    }

    if (chemin.mouvement == DEPLACEMENT_LIGNE_DROITE)
    {
        derivP.x = segmentTraj[1].aix;
        derivP.y = segmentTraj[1].aiy;
    }

    return derivP;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_DiffTrajectoire
 *
 *  \note   
 *
 *  \param [in]     segmentTraj  pointeur de structure definissant un segment de trajectoire
 *  \param [in]     iSegment       Segment
 *  \param [in]     t                   parametre de la courbe parametrique [0; 1]: 0-> pose de depart, 1-> pose d'arrivee.
 *
 *  \return            Vecteur        derivee des coordonnees de la position sur la courbe de B-Spline fonction de t,
 *                                           par rapport au parametre t.
 */
/**********************************************************************/
static Vecteur ASSER_TRAJ_DiffTrajectoire(segmentTrajectoireBS * segmentTraj, unsigned int iSegment, float t)
{
    return ASSER_TRAJ_DiffCourbeBSpline(segmentTraj, iSegment, (t - ((float)(iSegment - 1))));
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_Diff2CourbeBSpline
 *
 *  \note   Expression parametrique de la derivee seconde de l'expression parametrique de type B-Spline de la position
 *
 *  \param [in]     segmentTraj  pointeur de structure definissant un segment de trajectoire
 *  \param [in]     iSegment       Segment
 *  \param [in]     t                   parametre de la courbe parametrique [0; 1]: 0-> pose de depart, 1-> pose d'arrivee.
 *
 *  \return            Vecteur        derivee des coordonnees de la position sur la courbe de B-Spline fonction de t,
 *                                           par rapport au parametre t.
 */
/**********************************************************************/
static Vecteur ASSER_TRAJ_Diff2CourbeBSpline(segmentTrajectoireBS * segmentTraj, unsigned int iSegment, float t)
{
    Vecteur         deriv2P;
    float           tl;
    unsigned char   num_portion;
    unsigned int    iseg_portion;
    /* var temporaire de resultats de calcul de puissance */
    float           tl_2;

    if (t < ti)
    {
        num_portion = 1;
        tl = t;
        iseg_portion = iSegment - 1;
    }
    else
    {
        if (t > (1.0 - ti))
        {
            num_portion = 3;
            tl = 1.0 - t;
            iseg_portion = iSegment;
        }
        else
        {
            num_portion = 2;
            iseg_portion = iSegment;
            tl = t - ti;
        }
    }

    if (num_portion != 2)
    {
        tl_2 = SQUARE(tl);
        deriv2P.x = ((- segmentTraj[iSegment].qx[(iseg_portion - iSegment + 1)]) / ti) * tl + segmentTraj[iSegment].qx[(iseg_portion - iSegment + 1)];
        deriv2P.y = ((- segmentTraj[iSegment].qy[(iseg_portion - iSegment + 1)]) / ti) * tl + segmentTraj[iSegment].qy[(iseg_portion - iSegment + 1)];
    }
    else
    {
        deriv2P.x = 0.0;
        deriv2P.y = 0.0;
    }

    if (chemin.mouvement == DEPLACEMENT_LIGNE_DROITE)
    {
        deriv2P.x = 0.0;
        deriv2P.y = 0.0;
    }

    return deriv2P;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_Diff2Trajectoire
 *
 *  \note    
 *
 *  \param [in]     segmentTraj
 *  \param [in]     iSegment
 *  \param [in]     t
 *
 *  \return             
 */
/**********************************************************************/
static Vecteur ASSER_TRAJ_Diff2Trajectoire(segmentTrajectoireBS * segmentTraj, unsigned int iSegment, float t)
{
    return ASSER_TRAJ_Diff2CourbeBSpline(segmentTraj, iSegment, (t - ((float)(iSegment - 1))));
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

    diffTheta = SQUARE(diff1BS.x) + SQUARE(diff1BS.y);

    /* Restriction de la somme precedente a une valeur minimale pour eviter la division par zero et maximiser le resultat final de diffTheta */
    if (diffTheta < ((float)1e-9))
    {
        diffTheta = 1e-9;
    }

    diffTheta = ((diff2BS.y * diff1BS.x) - (diff1BS.y * diff2BS.x)) / diffTheta;

    return diffTheta;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_DiffThetaBSplinePerLenghtUnit
 *
 *  \note   Expression parametrique de la derivee premiere de l'angle de la tangente a la trajectoire
 *
 *  \param [in]     segmentTraj  pointeur de structure definissant un segment de trajectoire
 *  \param [in]     iSegment       Segment
 *  \param [in]     t                   parametre de la courbe parametrique [0; 1]: 0-> pose de depart, 1-> pose d'arrivee.
 *
 *  \return            diffTheta
 */
/**********************************************************************/
static float ASSER_TRAJ_DiffThetaBSplinePerLenghtUnit(segmentTrajectoireBS * segmentTraj, unsigned int iSegment, float t)
{
    Vecteur         diff1BS, diff2BS;
    float           diffTheta;
    float           facteurCorrectionT;

    diff1BS = ASSER_TRAJ_DiffTrajectoire(segmentTraj, iSegment, t );
    diff2BS = ASSER_TRAJ_Diff2Trajectoire(segmentTraj, iSegment, t );

    facteurCorrectionT = sqrtf(SQUARE(diff1BS.x) + SQUARE(diff1BS.y));

    diffTheta = SQUARE(diff1BS.x) + SQUARE(diff1BS.y);

    /* Restriction de la somme precedente a une valeur minimale pour eviter la division par zero et maximiser le resultat final de diffTheta */
    if (diffTheta < ((float)1e-9))
    {
        diffTheta = 1e-9;
    }

    diffTheta = ((diff2BS.y * diff1BS.x) - (diff1BS.y * diff2BS.x)) / diffTheta;

    diffTheta = diffTheta / facteurCorrectionT;

    return diffTheta;
}

/*****************************facteurCorrectionT = sqrtf(SQUARE(diffD_T.x) + SQUARE(diffD_T.y));*****************************************/
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
    
    poseTraj.x = p_rotation->poseDepartRobot.x + (NORME_BARRE_SUIVI_TRAJ * cosf(poseTraj.angle));    
    poseTraj.y = p_rotation->poseDepartRobot.y + (NORME_BARRE_SUIVI_TRAJ * sinf(poseTraj.angle));    

    return poseTraj;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_VitesseLimiteEnVirage
 *
 *  \note   Determination de la vitesse longitudinale limite de consigne a appliquer en fonction de la courbure de la trajectoire
 *
 *  \param [in]     traj                     pointeur de structure definissant la trajectoire
 *  \param [in]     diffThetaTrajectoire     vitesse angulaire de la trajectoire
 *
 *  \return         vitesseLimite   vitesse en m/s
 */
/**********************************************************************/
static float ASSER_TRAJ_VitesseLimiteEnVirage(Trajectoire *traj, float diffThetaTrajectoire)
{
    //return ((float)(traj->profilVitesse.vmax / (1.0 + (fabsf(diffThetaTrajectoire) * FacteurVitesseAngulaireMax))));

    return ((float)(traj->profilVitesse.vmax / (1.0 + (fabsf(diffThetaTrajectoire) * (ECART_ROUE_MOTRICE / 2.0) ))));
}

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
 *  \param [in]     pReturn                 Flag de fin de trajectoire
 *
 *  \return            None
 */
/**********************************************************************/
static void ASSER_TRAJ_ParcoursTrajectoire(Trajectoire *traj, float delta_distance, unsigned int *segmentCourant, float *paramPoseSegTraj, unsigned char * pReturn)
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
            diffD_T = ASSER_TRAJ_DiffTrajectoire(traj->segmentTrajBS, *segmentCourant, *paramPoseSegTraj);            
            facteurCorrectionT = sqrtf(SQUARE(diffD_T.x) + SQUARE(diffD_T.y));            

            /* Deplacement du parametre de pose du chemin */
            delta_param_chemin = delta_distance / facteurCorrectionT;
        }
        else /* if (chemin.mouvement == ROTATION) */
        {
            facteurCorrectionT = 1.0;
            delta_param_chemin = (delta_distance / traj->distance);
        }

        *paramPoseSegTraj += delta_param_chemin;

        if (*paramPoseSegTraj > ((float)*segmentCourant))
        {
            /* Passage au segment suivant */
            if (*segmentCourant < traj->nbreSegments)
            {
                /* part du deplacement du parametre du chemin au-dela du segment courant */
                *paramPoseSegTraj = *paramPoseSegTraj - ((float)*segmentCourant);

                /* part de la distance au-dela du segment courant */
                delta_distance = *paramPoseSegTraj * facteurCorrectionT;

                (*segmentCourant)++;
                *paramPoseSegTraj = ((float)(*segmentCourant - 1));
                Phase = 0;
            }
            else
            {
                *paramPoseSegTraj = ((float)traj->nbreSegments);
                flag_trajectoireTerminee = True;
            }
        }
    } while ((*segmentCourant != memoSegmentCourant) && (flag_trajectoireTerminee != True));

    if (pReturn != NULL)
    {
        *pReturn = flag_trajectoireTerminee;
    }
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_TabGabarit_AddVitesse
 *
 *  \note   ajout d'une vitesse et de son acceleration correspondante dans les tableaux du profil de vitesse
 *
 *  \param [in]     vitesse         vitesse suivante du profil de consigne de vitesse
 *
 *  \return None
 */
/**********************************************************************/
static void ASSER_TRAJ_TabGabarit_AddVitesse(float vitesse)
{
    if (g_index_tab_gabarit_vitesse < TAILLE_TAB_GABARIT_VITESSE)
    {
        g_tab_gabarit_vitesse[g_index_tab_gabarit_vitesse] = vitesse;
        
        ASSER_TRAJ_LogAsserValPC("gabarit_vitesse", g_tab_gabarit_vitesse[g_index_tab_gabarit_vitesse]);

        g_index_tab_gabarit_vitesse++;
    }
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_TabGabarit_AddAcceleration
 *
 *  \note   ajout d'une vitesse et de son acceleration correspondante dans les tableaux du profil de vitesse
 *
 *  \param [in]     acceleration    acceleration suivante du profil de consigne de vitesse
 *
 *  \return None
 */
/**********************************************************************/
static void ASSER_TRAJ_TabGabarit_AddAcceleration(float acceleration)
{
    if (g_index_tab_gabarit_acceleration < TAILLE_TAB_GABARIT_VITESSE)
    {
        g_tab_gabarit_acceleration[g_index_tab_gabarit_acceleration] = acceleration;
        
        ASSER_TRAJ_LogAsserValPC("gabarit_acceleration", g_tab_gabarit_acceleration[g_index_tab_gabarit_acceleration]);

        g_index_tab_gabarit_acceleration++;
    }
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
    unsigned int    numSegment                      = 1;
    float           acc_courant                     = 0.0;
    float           paramPosition                   = 0.0;
    float           vitesse_consigne                = 0.0;
    float           vitesse_limite                  = 0.0;
    float           distanceRestante;
    unsigned char   flag_finTraj                    = False;
    float           diffThetaTrajectoire            = 0.0;
    float           vitesse_consigne_gabarit        = 0.0;
    float           acceleration_consigne_gabarit   = 0.0;
    float           vpointe                         = 0.0;
    unsigned int    i                               = 0u;
    float           vit_ini                         = 0.05;

    distanceRestante = traj->distance;
    
    ASSER_TRAJ_LogAsserValPC("distanceRestante_FD", distanceRestante);

    /* Parcourir la trajectoire en l'echantillonnant sur N pas de distance pas_echantillon_distance */
    vitesse_consigne = vit_ini;
    
    ASSER_TRAJ_TabGabarit_AddVitesse(traj->profilVitesse.vmax);
    
    while (flag_finTraj == 0)
    {
        ASSER_TRAJ_ParcoursTrajectoire(traj, traj->profilVitesse.pas_echantillon_distance, &numSegment, &paramPosition, &flag_finTraj);
        distanceRestante -= traj->profilVitesse.pas_echantillon_distance;
        
        ASSER_TRAJ_LogAsserValPC("distanceRestante_FD", distanceRestante);

        if (ASSER_TRAJ_isDeplacement(traj) == True)
        {
            diffThetaTrajectoire = ASSER_TRAJ_DiffThetaBSplinePerLenghtUnit(traj->segmentTrajBS, numSegment, paramPosition);

            vitesse_limite = ASSER_TRAJ_VitesseLimiteEnVirage(traj, diffThetaTrajectoire);

            vitesse_consigne_gabarit = vitesse_limite;
        }
        else
        {
            vitesse_consigne_gabarit = vitesse_consigne;
        }

        if (vitesse_consigne_gabarit < VminMouv)
        {
            vitesse_consigne_gabarit = VminMouv;
        }

        acceleration_consigne_gabarit = (powf(vitesse_consigne_gabarit, 2.0) - powf(g_tab_gabarit_vitesse[g_index_tab_gabarit_vitesse - 1], 2.0)) / (2.0 * traj->profilVitesse.pas_echantillon_distance);

        ASSER_TRAJ_TabGabarit_AddAcceleration(acceleration_consigne_gabarit);
        ASSER_TRAJ_TabGabarit_AddVitesse(vitesse_consigne_gabarit);

    }
/*
    if (ASSER_TRAJ_isDeplacement(traj) == True)
    {
        ASSER_TRAJ_SmoothGabaritAcceleration(traj);
        ASSER_TRAJ_CreateGabaritVitesseFromGabaritAcceleration(traj);
    }
*/
#ifndef PIC32_BUILD
    if (ASSER_TRAJ_isDeplacement(traj) == False)
    {
        for(i = 0; i < g_index_tab_gabarit_acceleration; i++)
        {
            ASSER_TRAJ_LogAsserValPC("gabarit_acceleration_new", g_tab_gabarit_acceleration[i]);
            ASSER_TRAJ_LogAsserValPC("gabarit_vitesse_new", g_tab_gabarit_vitesse[i]);
        }
    }
#endif /* PIC32_BUILD */
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_InitTabVPointe
 *
 *  \note   Determination du gabarit de la vitesse longitudinale de consigne, dependant de la courbure de trajectoire
 *  et des fonctions d'acceleration et de decceleration.
 *
 *  \param [in]     traj            pointeur de structure definissant la trajectoire
 *  \param [in]     coeff_vit_ini   coefficient applique a la vitesse de pointe pour determiner la vitesse de consigne au depart
 *
 *  \return None
 */
/**********************************************************************/
static void ASSER_TRAJ_InitTabVPointe(Trajectoire * traj, float vit_ini)
{   
    /* tableau de 1: distance des phases d'acc et de decc reunies, 2: vitesse max(de pointe), 3; distance de decc */
    unsigned int    i;
    float           vpointe, distanceAcc, distanceDecc;

    vpointe = 0.05;
    ASSER_TRAJ_LogAsserValPC("vpointe", vpointe);
    
    distanceAcc = ASSER_TRAJ_DistanceAcceleration(traj, vpointe, vit_ini);
    distanceDecc = ASSER_TRAJ_DistanceDecceleration(traj, vpointe);
    
    g_tab_vpointe[0][0] = distanceAcc + distanceDecc;
    g_tab_vpointe[0][1] = vpointe;
    g_tab_vpointe[0][2] = distanceDecc;

    for(i = 1; i < 10; i++)
    {
        vpointe = 0.1 + (((float)( i - 1)) * 0.1);
        
        if (vpointe > traj->profilVitesse.vmax)
        {
            iMaxVpointe = i;
            vpointe = traj->profilVitesse.vmax;
        }
        
        ASSER_TRAJ_LogAsserValPC("vpointe", vpointe);
        
        distanceAcc = ASSER_TRAJ_DistanceAcceleration(traj, vpointe, vit_ini);
        distanceDecc = ASSER_TRAJ_DistanceDecceleration(traj, vpointe);
        
        g_tab_vpointe[i][0] = distanceAcc + distanceDecc;
        g_tab_vpointe[i][1] = vpointe;
        g_tab_vpointe[i][2] = distanceDecc;

        if ((0.1 + (((float)(i - 1)) * 0.1)) > traj->profilVitesse.vmax)
        {
            break;
        }
    }
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_SmoothGabaritAcceleration
 *
 *  \note   lisser le profil d'acceleration
 *
 *  \param [in]     traj            pointeur de structure definissant la trajectoire
 *
 *  \return None
 */
/**********************************************************************/
static void ASSER_TRAJ_SmoothGabaritAcceleration(Trajectoire * traj)
{
    int             i, m, n;
    float           acc_max_vsVitesse   = 0.0;
    float           acc                 = 1.0;
    float           acc_max_cste        = 0.0;
    float           sum_acc             = 0.0;
    unsigned int    cpt                 = 0;
    float           vitesse             = 0.0;
    int             flag_acc            = 1;
    unsigned char   flag_fin            = False;
    float           vcarre              = 0.0;
    
    vitesse = 0.0;
    
    acc_max_cste = powf(vitesse, 2.0) / (2.0 * traj->profilVitesse.pas_echantillon_distance);

    i = 0;
    
    while(i < (g_index_tab_gabarit_acceleration - 1u))
    {
        acc = g_tab_gabarit_acceleration[i];
        vitesse = g_tab_gabarit_vitesse[i];

        if (acc > 0.0)
        {
            flag_acc = 1;

            flag_fin = ASSER_TRAJ_ProfilAcceleration_2012(traj, traj->profilVitesse.vpointe, &vitesse, &acc_max_vsVitesse);

            if (i > 0)
            {
                if (g_tab_gabarit_acceleration[i-1] < 0.0)
                {
                    acc_max_vsVitesse = 0.0;
                    
                    vitesse = sqrtf( SQUARE((g_tab_gabarit_vitesse[i])) + (2.0 * traj->profilVitesse.pas_echantillon_distance * acc_max_vsVitesse) );                    
                }
            }

            if ((acc > (acc_max_vsVitesse*1.01)) )
            {
                m = i;

                cpt = 0;
                
                do
                {
                    n = m + flag_acc;

                    sum_acc = g_tab_gabarit_acceleration[m] + g_tab_gabarit_acceleration[n];

                    g_tab_gabarit_acceleration[m] = acc_max_vsVitesse;
                    g_tab_gabarit_acceleration[n] = sum_acc - acc_max_vsVitesse;

                    if ( ( ((float)(n-i)) * traj->profilVitesse.pas_echantillon_distance < 0.04 ) && (ASSER_TRAJ_Acceleration_vs_Vitesse(traj, traj->profilVitesse.vpointe, vitesse) > acc_max_cste))
                    {
                        acc_max_vsVitesse = acc_max_cste;

                        vitesse = sqrtf( SQUARE(vitesse) + (2.0 * traj->profilVitesse.pas_echantillon_distance * acc_max_cste));                        
                    }
                    else
                    {
                        flag_fin = ASSER_TRAJ_ProfilAcceleration_2012(traj, traj->profilVitesse.vpointe, &vitesse, &acc_max_vsVitesse);
                    }

                    m = m + flag_acc;

                    cpt++;
                }
                while ((g_tab_gabarit_acceleration[n] > (acc_max_vsVitesse * 1.01)) && (m < (g_index_tab_gabarit_acceleration - 1u)));
                
                i = n;

                ASSER_TRAJ_LogAsserValPC("debug_smooth", i);
                ASSER_TRAJ_LogAsserValPC("debug_smooth", n);
            }
        }
        i++;
    }

    i = (g_index_tab_gabarit_acceleration - 1u);
    
    while(i > 0)
    {
        acc = g_tab_gabarit_acceleration[i];
        vitesse = g_tab_gabarit_vitesse[i];

        if (acc < 0.0)
        {
            flag_acc = -1;
            acc_max_vsVitesse = ASSER_TRAJ_Decceleration_vs_Vitesse(traj, traj->profilVitesse.vpointe, vitesse);

            if (acc < (acc_max_vsVitesse * 1.01))
            {
                m = i;

                cpt = 0;
                
                do
                {
                    n = m + flag_acc;

                    sum_acc = g_tab_gabarit_acceleration[m] + g_tab_gabarit_acceleration[n];

                    g_tab_gabarit_acceleration[m] = acc_max_vsVitesse;
                    g_tab_gabarit_acceleration[n] = sum_acc - acc_max_vsVitesse;

                    vcarre = SQUARE(vitesse) - (2.0 * traj->profilVitesse.pas_echantillon_distance * acc_max_vsVitesse);

                    if (vcarre > 0.0)
                    {                        
                        vitesse = sqrtf(vcarre);                        
                    }
                    else
                    {
                        vitesse = 0.0;
                    }
                    
                    acc_max_vsVitesse = ASSER_TRAJ_Decceleration_vs_Vitesse(traj, traj->profilVitesse.vpointe, vitesse);

                    m = m + flag_acc;

                    cpt++;
                }
                while ((g_tab_gabarit_acceleration[n] < (acc_max_vsVitesse * 1.01)) && (m > 0));
                
                i = m;

                ASSER_TRAJ_LogAsserValPC("debug_smooth", i);
                ASSER_TRAJ_LogAsserValPC("debug_smooth", n);
            }
        }
        
        i--;
    }

#ifndef PIC32_BUILD
    for(i = 0; i < g_index_tab_gabarit_acceleration; i++)
    {
        ASSER_TRAJ_LogAsserValPC("gabarit_acceleration_new", g_tab_gabarit_acceleration[i]);
    }
#endif /* PIC32_BUILD */
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_CreateGabaritVitesseFromGabaritAcceleration
 *
 *  \note   lisser le profil d'acceleration
 *
 *  \param [in]     traj            pointeur de structure definissant la trajectoire
 *
 *  \return None
 */
/**********************************************************************/
static void ASSER_TRAJ_CreateGabaritVitesseFromGabaritAcceleration(Trajectoire * traj)
{
    unsigned int    i;
    
    ASSER_TRAJ_LogAsserValPC("gabarit_vitesse_new", g_tab_gabarit_vitesse[0]);

    for (i = 0; i < g_index_tab_gabarit_acceleration; i++)
    {
        g_tab_gabarit_vitesse[(i + 1)] = sqrtf(powf(g_tab_gabarit_vitesse[i], 2.0) + (2.0 * traj->profilVitesse.pas_echantillon_distance * g_tab_gabarit_acceleration[i]));

        ASSER_TRAJ_LogAsserValPC("gabarit_vitesse_new", g_tab_gabarit_vitesse[(i + 1)]);
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
    unsigned int    index;
    float           vitesse, delta_distance;

    index = floorf(distance / chemin.profilVitesse.pas_echantillon_distance);

    if (index > (TAILLE_TAB_GABARIT_VITESSE - 1))
    {
        index = (TAILLE_TAB_GABARIT_VITESSE - 1);
        
        vitesse = g_tab_gabarit_vitesse[index];
    }
    else
    {
        /* Interpolation de la vitesse de consigne */
        delta_distance = distance - (((float)index) * chemin.profilVitesse.pas_echantillon_distance);

        vitesse = g_tab_gabarit_vitesse[index] + ((g_tab_gabarit_vitesse[(index + 1)] - g_tab_gabarit_vitesse[index]) * (delta_distance / chemin.profilVitesse.pas_echantillon_distance));
    }

    ASSER_TRAJ_LogAsserValPC("index_get_vitesseGabarit", vitesse);

    return vitesse;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_Acceleration_vs_Vitesse
 *
 *  \note   
 *
 *  \param [in]     traj 
 *  \param [in]     vpointe
 *  \param [in]     vitesse
 *
 *  \return           acc
 */
/**********************************************************************/
static float ASSER_TRAJ_Acceleration_vs_Vitesse(Trajectoire * traj, float vpointe, float vitesse)
{
    float acc;

    acc = traj->profilVitesse.Amax - ((traj->profilVitesse.Amax / SQUARE(vpointe)) * SQUARE(vitesse));

    return acc;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_Decceleration_vs_Vitesse
 *
 *  \note   
 *
 *  \param [in]     traj 
 *  \param [in]     vpointe
 *  \param [in]     vitesse
 *
 *  \return           decc
 */
/**********************************************************************/
static float ASSER_TRAJ_Decceleration_vs_Vitesse(Trajectoire * traj, float vpointe, float vitesse)
{
    float   decc;
    float   vi1, decc_limit;

    vi1 = vpointe * traj->profilVitesse.coeff_vi1;
    
    if (vitesse < vi1)
    {
        decc = traj->profilVitesse.Dmax;
    }
    else
    {
        decc = traj->profilVitesse.Dmax * (1.0 - SQUARE((vitesse - vi1) / (vpointe - vi1)));
    }

    if (sqrtf(SQUARE(vitesse) + (2.0 * traj->profilVitesse.pas_echantillon_distance * decc)) < traj->profilVitesse.vitesse_seuil_decc_finale)
    {
        decc_limit = traj->profilVitesse.decc_min_finale - (traj->profilVitesse.coeff_decc_finale * vitesse);

        if (decc  < decc_limit)
        {
            decc = decc_limit;
        }
    }

    return decc;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_ProfilAcceleration_2012
 *
 *  \note   Determination de la vitesse longitudinale de consigne en phase d'acceleration
 *
 *  \param [in]     traj              pointeur de structure definissant la trajectoire
 *  \param [in]     vpointe         vitesse de pointe du profil de vitesse en acceleration
 *  \param [in]     *vitesse_n    vitesse courante 
 *  \param [in]     acceleration  acceleration  
 *
 *  \return            flag_fin        info de la fin de la phase d'acceleration
 */
/**********************************************************************/
static unsigned char ASSER_TRAJ_ProfilAcceleration_2012(Trajectoire * traj, float vpointe, float * vitesse_n, float * acceleration)
{
    float           acc, vitesse_np1;
    unsigned char   flag_fin                = False;

    acc = ASSER_TRAJ_Acceleration_vs_Vitesse(traj, vpointe, *vitesse_n);

    vitesse_np1 = sqrtf(SQUARE(*vitesse_n) + (2.0 * traj->profilVitesse.pas_echantillon_distance * acc));

    if (vitesse_np1 > (vpointe - 0.005))
    {
        flag_fin = True;
    }

    *vitesse_n = vitesse_np1;
    *acceleration = acc;

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
 *  \param [in]     acceleration  acceleration  
 *
 *  \return            flag_fin        info de la fin de la phase de decceleration
 */
/**********************************************************************/
static unsigned char ASSER_TRAJ_ProfilDecceleration_2012(Trajectoire * traj, float vpointe, float * vitesse_n, float * acceleration)
{
    float           decc, vitesse_np1;
    float           vcarre                              = 0.0;
    unsigned char   flag_fin                            = False;

    if (*vitesse_n > (vpointe - 0.005))
    {
        *vitesse_n = vpointe - 0.005;
    }

    decc = ASSER_TRAJ_Decceleration_vs_Vitesse(traj, vpointe, *vitesse_n);

    vcarre = SQUARE(*vitesse_n) + (2.0 * traj->profilVitesse.pas_echantillon_distance * decc);

    if (vcarre < 0.0)
    {
        vitesse_np1 = 0.0;
    }
    else
    {
        vitesse_np1 = sqrtf(vcarre);        
    }

    if (vitesse_np1 < 0.001)
    {
        flag_fin = True;
    }

    *vitesse_n = vitesse_np1;
    *acceleration = decc;

    return flag_fin;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_DistanceAcceleration
 *
 *  \note   Determination de la distance sur laquelle a lieu la phase d'acceleration
 *
 *  \param [in]     traj            pointeur de structure definissant la trajectoire
 *  \param [in]     vpointe       vitesse de pointe du profil de vitesse en acceleration
 *
 *  \return           distance
 */
/**********************************************************************/
static float ASSER_TRAJ_DistanceAcceleration(Trajectoire * traj, float vpointe, float vit_ini)
{
    unsigned int    nbrePas = 0u;
    float           vitesse = vit_ini;
    float           acc;
    unsigned char   flag_fin;

    do 
    {
        flag_fin = ASSER_TRAJ_ProfilAcceleration_2012(traj, vpointe, &vitesse, &acc);
        
        nbrePas++;
    } while ((flag_fin == False) && (nbrePas < 65535));

    return ((float)(nbrePas * traj->profilVitesse.pas_echantillon_distance));
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_DistanceDecceleration
 *
 *  \note   Determination de la distance sur laquelle a lieu la phase de deccelaration
 *
 *  \param [in]     traj            pointeur de structure definissant la trajectoire
 *  \param [in]     vpointe       vitesse de pointe du profil de vitesse en decceleration
 *
 *  \return           distance
 */
/**********************************************************************/
static float ASSER_TRAJ_DistanceDecceleration(Trajectoire * traj, float vpointe)
{
    unsigned int    nbrePas         = 0u;
    float           vitesse, acc;
    unsigned char   flag_fin;

    vitesse = vpointe - 0.005;
    
    do
    {
        flag_fin = ASSER_TRAJ_ProfilDecceleration_2012(traj, vpointe, &vitesse, &acc);
        
        nbrePas++;
    } while((flag_fin == False) && (nbrePas < 65535));

    return ((float)(nbrePas * traj->profilVitesse.pas_echantillon_distance));
}

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
    matrice2p2[0].x = cosf(angle);    
    matrice2p2[0].y = sinf(angle);    
    matrice2p2[1].x = - sinf(angle);    
    matrice2p2[1].y = cosf(angle);    
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
        
     vitessesCons.longitudinale = (cosf(erreurPose.angle) * vitesseTemp.longitudinale) + (sinf(erreurPose.angle) * vitesseTemp.rotation);     
     vitessesCons.rotation = (- sinf(erreurPose.angle) * vitesseTemp.longitudinale) + (cosf(erreurPose.angle) * vitesseTemp.rotation);     
     vitessesCons.rotation = vitessesCons.rotation / longueurBarreSuivi;

     ASSER_TRAJ_LogAsserValPC("errPoseAngle", erreurPose.angle);

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
    float                   z, uref1, uref2, erreurAngle;
    float                   w[2];

    erreurAngle = POS_ModuloAngle(erreurPose.angle);
    
    if (fabsf(erreurAngle) > ((PI / 2.0) - 0.1))
    {
        if (erreurAngle > 0.0)
        {
            erreurAngle = (PI / 2.0) - 0.1;
        }
        else
        {
            erreurAngle = -((PI / 2.0) - 0.1);
        }
    }

    z = tanf(erreurAngle);

    uref1 = sqrtf(SQUARE(diffPoseRef.x) + SQUARE(diffPoseRef.y));

    uref2 = diffPoseRef.angle;

    w[0] = - gain[0] * uref1 * fabsf(uref1) * (erreurPose.x + (erreurPose.y * z));
    w[1] = -(gain[1] * uref1 * erreurPose.y) - (gain[2] * fabsf(uref1) * z);

    vitessesCons.longitudinale = (w[0] + uref1) / cosf(erreurAngle);
    vitessesCons.rotation = (w[1] * SQUARE(cosf(erreurAngle))) + uref2;

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

    if (ASSER_TRAJ_isDeplacement(traj) == True)
    {
        /* Test pour un mouvement de deplacement */
        if ((erDist < tolDist) || (((erDist < (tolDist * 10.0)) && ((erDist - memo_erDist) > 0.0))))
        {
            ret = True;

            ASSER_TRAJ_LogAsserValPC("vitesseAnglePtArret", (erAngle - memo_erAngle));
            ASSER_TRAJ_LogAsserValPC("CPTvitesseAnglePtArret", ASSER_TRAJ_GetCompteur());

#ifndef PIC32_BUILD

            Pose    poseRobot_asserEnd;
            
            poseRobot_asserEnd = POS_GetPoseRobot();

            ASSER_TRAJ_LogAsserValPC("finAsser", poseRobot_asserEnd.x);
            ASSER_TRAJ_LogAsserValPC("finAsser", poseRobot_asserEnd.y);
            ASSER_TRAJ_LogAsserValPC("finAsser", poseRobot_asserEnd.angle);
#endif /* PIC32_BUILD */
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
 *  \param [in] iSegment
 *
 *  \return None
 */
/**********************************************************************/
static void ASSER_TRAJ_DistanceTrajectoire(segmentTrajectoireBS * segmentTraj, unsigned int iSegment)
{
    float           distance = 0.000001;    /* toutes les divisions par la distance ne pourront pas etre des divisions par zero */
    unsigned char   param;
    Vecteur         diffD_T;

    for (param = 0; param < (unsigned char) 5; param++)
    {
        diffD_T = ASSER_TRAJ_DiffCourbeBSpline(segmentTraj, iSegment, ((float)(ti * (((float)param) / 5.0))));
        
        distance += sqrtf(SQUARE(diffD_T.x) + (SQUARE(diffD_T.y))) * (ti * ((float)(1.0 / 5.0)));        
    }
    
    ASSER_TRAJ_LogAsserValPC("disti", distance);

    distance += sqrtf(((float)SQUARE(segmentTraj[iSegment].aix) + SQUARE(segmentTraj[iSegment].aiy))) * ((float)(1.0 - (2.0 * ti)));

    for (param = 0; param < (unsigned char) 5; param++)
    {
        diffD_T = ASSER_TRAJ_DiffCourbeBSpline(segmentTraj, iSegment, ((float)(((1.0 - ti) * 1.001) + (ti * (((float)param) / 5.0)))));

        distance += sqrtf(((float)(SQUARE(diffD_T.x) + SQUARE(diffD_T.y)))) * ((float)(ti * ((float)(1.0 / 5.0))));
    }

    segmentTraj[iSegment].distance = distance;
}

#ifndef PIC32_BUILD

/**********************************************************************/
/*! \brief ASSER_TRAJ_LogAsserValPC
 *
 *  \note  Log d'une variable interne de l'asser
 * 
 *  \param [in] keyWord    Nom de la variable logguer
 *  \param [in] Val        Valeur a logguer
 *
 *  \return  None
 */
/**********************************************************************/
void ASSER_TRAJ_LogAsserValPC(char * keyWord, float Val)
{
    printf("log_%s: %1.5f\n", keyWord, Val);
    fflush(stdout);
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_LogAsserMsgPC
 *
 *  \note  Log d'un message associe a une valeur
 *
 *  \param [in] Message    Message a logguer
 *  \param [in] Val            Valeur a logguer
 *
 *  \return  None
 */
/**********************************************************************/
void ASSER_TRAJ_LogAsserMsgPC(char * message, float Val)
{
    printf("msg_%s: %1.5f\n", message, Val);
    fflush(stdout);
}

#else /* PIC32_BUILD */


/**********************************************************************/
/*! \brief ASSER_TRAJ_LogAsserPIC
 *
 *  \note  Log Asser
 * 
 *  \param [in] keyWord    Valeur de la table de log
 *  \param [in] Val1           Valeur a logguer 
 *  \param [in] pVal2         Valeur a logguer
 *  \param [in] pVal3         Valeur a logguer
 *  \param [in] pVal4         Valeur a logguer
 *  \param [in] pVal5         Valeur a logguer
 *
 *  \return  None
 */
/**********************************************************************/
void ASSER_TRAJ_LogAsserPIC(char * keyWord, float Val1, float * pVal2, float * pVal3, float * pVal4, float * pVal5)
{
    if (pVal2 == NULL)
    {
        TOOLS_Printf("%s:%1.5f\n", keyWord, Val1);
    }
    else
    {
        if (pVal3 == NULL)
        {
            TOOLS_Printf("%s:%1.5f,%1.5f\n", keyWord, Val1, *pVal2);
        }
        else
        {
            if (pVal4 == NULL)
            {
                TOOLS_Printf("%s:%1.5f,%1.5f,%1.5f\n", keyWord, Val1, *pVal2, *pVal3);
            }
            else
            {
                if (pVal5 == NULL)
                {
                    TOOLS_Printf("%s:%1.5f,%1.5f,%1.5f,%1.5f\n", keyWord, Val1, *pVal2, *pVal3, *pVal4);
                }
                else
                {
                    TOOLS_Printf("%s:%1.5f,%1.5f,%1.5f,%1.5f,%1.5f\n", keyWord, Val1, *pVal2, *pVal3, *pVal4, *pVal5);
                }
            }
        }
    }
}

#endif /* PIC32_BUILD */

/**********************************************************************/
/*! \brief ASSER_TRAJ_Profil_S_Curve
 *
 *  \note  Log Asser
 * 
 *  \param [out] Vconsigne Vitesse de consigne (m/s)
 *  \param [in] Distance     Distance a parcourir (m)
 *  \param [in] VStart        Vitesse de debut de profil de vitesse (m/s)
 *  \param [in] VEnd          Vitesse de fin de profil de vitesse (m/s)
 *  \param [in] Amax         Acceleration de debut de profil de vitesse (m/s2)
 *  \param [in] Dmax         Deceleration de fin de profil de vitesse (m/s2)
 *  \param [in] gASR         Gain d'ASR (m/s)
 *  \param [in] Pr              Postion reelle longitudinale (m)          
 *  \param [in] Vr              Vitesse relle (m/s)
 *  \param [in] fSatPI         Flag de saturation du PI
 *
 *  \return  Flag d'arrivee au point (False arrivee / True continuer le profil)
 */
/**********************************************************************/
static unsigned char ASSER_TRAJ_Profil_S_Curve(float * Vconsigne, float Distance, float VStart, float VEnd, float Amax, float Dmax, float gASR, float Pr, float Vr, unsigned char fSatPI)
{
    unsigned char   AsserRunningFlag = True;

    Vr = fabsf(Vr);
    
    if (Phase == 0)
    {
        /* Phase 0 (Initialisation) */
            
        /* Forcage de la vitesse minim de deplacement minimum */
        if (VStart < (VminMouv / 2))
        {
            VStart = VminMouv / 2;
        }

        /* Initilisation de la vitesse maximum */
        Vmax = MIN(DonneeVmaxGauche, DonneeVmaxDroite);
        
        /* Ajustement de Vmax suivant la distance si necessaire */      
        DistanceMin = (((Vmax * Vmax) - (VStart * VStart)) / Amax) + (((Vmax * Vmax) - (VEnd * VEnd)) / Dmax) + (2 * (Vmax * TE)) + (VStart * TE) + (VEnd * TE);

        while (DistanceMin > Distance)
        {
            Vmax -= 0.001;
            DistanceMin = (((Vmax * Vmax) - (VStart * VStart)) / Amax) + (((Vmax * Vmax) - (VEnd * VEnd)) / Dmax) + (2 * (Vmax * TE)) + (VStart * TE) + (VEnd * TE);
        }
        
        /* Verification des parametres */
        if (VEnd > Vmax)
        {
            VEnd = Vmax;

#ifdef PIC32_BUILD        
            TOOLS_LogFault(AsserPosErr, False, INTEGER, 0, True, "Asserv_traj : Reajustement de la vitesse de fin de profil ! Vitesse de fin impossible sur cette distance");
#else /* PIC32_BUILD */
            ASSER_TRAJ_LogAsserMsgPC("Asserv_traj : Reajustement de la vitesse de fin de profil ! Vitesse de fin impossible sur cette distance", VEnd);
#endif /* PIC32_BUILD */
        }
        
        /* Ajustement de VStart et VEnd pour eviter les division par zero */
        if ((Vmax - VStart) <= 0.0)
        {
            VStart = Vmax - 0.000001;
        }
        
        if ((Vmax - VEnd) <= 0.0)
        {
            VEnd = Vmax - 0.000001;
        }
        
        /* Calcul du Jerk Max */
        JAmax = (Amax * Amax) / (Vmax - VStart);
        JDmax = (Dmax * Dmax) / (Vmax - VEnd);
                
        /* Initialisation des variables */
        A0 = 0.0;
        V0 = VStart;
        P0 = 0.0;
        J0 = JAmax;
        
        A = 0.0;
        V = 0.0;
        P = 0.0;
        J = 0.0;

        VgASR = 0.0;
        ASRrunning = False;
        Vconsigne0 = 0.0;
    
        /* Fin de la Phase 0 */
        Phase = 1;
    }
    
    switch(Phase)
    {
        /* Phase I ou Phase V */
        case 1:
        case 5:  
            
            /* Calcul S-Curve Profil */

            /* Position */
            P = P0 + (V0 * TE) + (0.5 * A0 * SQUARE(TE)) + ((1.0 / 6.0) * J0 * CUBE(TE));

            /* Vitesse */            
            V = V0 + (A0 * TE) + (0.5 * J0 * SQUARE(TE));

            /* Verification de la vitesse */
            if (V > Vmax)
            {
                V = Vmax;
            }
            
            if (V < 0.0)
            {
                V = 0.0;
            }
            
            /* Acceleration */
            A = A0 + (J0 * TE);
            
            if (A > Amax)
            {
                A = Amax;
            }
            else if (A < -Dmax)
            {
                A = -Dmax;
            }
            
            /* Jerk */
            J = A / TE;
            
            if (J > JAmax)
            {
                J = JAmax;
            }
            else if (J < -JDmax)
            {
                J = -JDmax;
            }

            /* ASR */

            /* Phase d'acceleration */
            if (Phase == 1)
            {
                if (ASRrunning == False)
                {
                    /* Vitesse Reelle atteinte => delcanchement de l'ASR pour plus de performance */
                    if ((Vr >= (V0 - EcartVitesse)) && (fSatPI == False))
                    {
                       ASRrunning = True;
                    }
                }
                
                if (ASRrunning == True)
                {
                    /* Vitesse Reelle atteinte => delcanchement de l'ASR pour plus de performance */
                    if ((Vr >= (V0 - EcartVitesse)) && (fSatPI == False))
                    {
                        /* Incrementation de la vitesse gain ASR */
                        VgASR = VgASR + gASR;
                        /* Modification de la vitesse suivant la vitesse precedente */
                        *Vconsigne = Vconsigne0 + VgASR;
                        
                        /* Verifiaction et ajustement de la vitesse de consigne */
                        if (*Vconsigne > Vmax)
                        {
                            *Vconsigne = Vmax;
                        }
                        else if (*Vconsigne < VminMouv)
                        {
                            *Vconsigne = VminMouv;
                        }    
                    }   
                    else
                    {
                        /* Vitesse Reelle non atteinte => correction de l'ASR */
                        if (VgASR > 0.0)
                        {
                            /* Decrementation de la vitesse gain ASR */
                            VgASR = VgASR - gASR;
                            /* Modification de la vitesse suivant la vitesse precedente */
                            *Vconsigne = Vconsigne0 + VgASR;
                            
                            /* Verifiaction et ajustement de la vitesse de consigne */
                            if (*Vconsigne > Vmax)
                            {
                                *Vconsigne = Vmax;
                            }
                            else if (*Vconsigne < VminMouv)
                            {
                                *Vconsigne = VminMouv;
                            }
                        }  
                        else
                        {
                            /* Correction de la vitesse pour eviter les sauts de vitesse en cours de profil */
                            if (V >= Vconsigne0)
                            {
                                /* Consigne de vitesse */
                                *Vconsigne = V;
                            }
                            else
                            {
                                *Vconsigne = Vconsigne0;
                            }                                
                            
                            /* Verifiaction et ajustement de la vitesse de consigne */
                            if (*Vconsigne > Vmax)
                            {
                                *Vconsigne = Vmax;
                            }
                            else if (*Vconsigne < VminMouv)
                            {
                                *Vconsigne = VminMouv;
                            }                                                        

#ifdef PIC32_BUILD
                            TOOLS_LogFault(AsserPosErr, True, FLOAT, (float *)&Amax, True, "Asserv_traj : Amax / Dmax > aux capacitees du robot !");
#else /* PIC32_BUILD */
                            ASSER_TRAJ_LogAsserMsgPC("Asserv_traj : Amax / Dmax > aux capacitees du robot !", Amax);
#endif /* PIC32_BUILD */     
                        }
                    }
                }
                else
                {            
                    /* Consigne de vitesse */
                    *Vconsigne = V;  
                   
                    /* Verifiaction et ajustement de la vitesse de consigne */
                    if (*Vconsigne > Vmax)
                    {
                        *Vconsigne = Vmax;
                    }
                    else if (*Vconsigne < VminMouv)
                    {
                        *Vconsigne = VminMouv;
                    }                                                                                                                                         
                }
            }       
            /* Phase de decceleration */
            else
            {
                if (ASRrunning == False)
                {
                    /* Vitesse Reelle superieure => delcanchement de l'ASR pour mieux deccelerer */
                    if ((Vr > (V0 + EcartVitesse)) && (fSatPI == False))
                    {                       
                       ASRrunning = True;
                    }
                }
                
                if (ASRrunning == True)
                {
                    /* Vitesse Reelle superieure => delcanchement de l'ASR pour mieux deccelerer */
                    if ((Vr > (V0 + EcartVitesse)) && (fSatPI == False))
                    {
                        /* Decrementation de la vitesse gain ASR */
                        VgASR = VgASR - gASR;
                        /* Modification de la vitesse suivant la vitesse du profil deceleration */
                        *Vconsigne = V + VgASR;

                        /* Verifiaction et ajustement de la vitesse de consigne */
                        if (*Vconsigne > Vmax)
                        {
                            *Vconsigne = Vmax;
                        }
                        else if (*Vconsigne < VminMouv)
                        {
                            *Vconsigne = VminMouv;
                        }
                    }
                    else
                    {
                        /* Vitesse Reelle atteinte => correction de l'ASR */
                        if (VgASR < 0.0)
                        {
                            /* Incrementation de la vitesse gain ASR */
                            VgASR = VgASR + gASR;
                            /* Modification de la vitesse suivant la vitesse du profil deceleration */
                            *Vconsigne = V + VgASR;
                            
                            /* Verifiaction et ajustement de la vitesse de consigne */
                            if (*Vconsigne > Vmax)
                            {
                                *Vconsigne = Vmax;
                            }                              
                            else if (*Vconsigne < VminMouv)
                            {
                                *Vconsigne = VminMouv;
                            }
                        }
                        else
                        {
                            /* Correction de la vitesse pour eviter les sauts de vitesse en cours de profil et pourvoir freiner */
                            if (Vconsigne0 < V)
                            {
                                *Vconsigne = Vconsigne0;
                            }
                            else
                            {
                                /* Consigne de vitesse*/
                                *Vconsigne = V;  
                                
                                /* Verifiaction et ajustement de la vitesse de consigne */
                                if (*Vconsigne > Vmax)
                                {
                                    *Vconsigne = Vmax;
                                }
                                else if (*Vconsigne < VminMouv)
                                {
                                    *Vconsigne = VminMouv;
                                }                                                                                        
                            } 
                        }
                    }
                }
                else
                {
                    /* Consigne de vitesse */
                    *Vconsigne = V;
                    
                    /* Verifiaction et ajustement de la vitesse de consigne */
                    if (*Vconsigne > Vmax)
                    {
                        *Vconsigne = Vmax;
                    }
                    else if (*Vconsigne < VminMouv)
                    {
                        *Vconsigne = VminMouv;
                    }                                          
                }
            }
                   
            /* Sauvergarde de valeur pour la porchaine iteration */
            A0 = A;
            V0 = V;
            P0 = P;
            J0 = J;

            /* Test fin de Phase */
            if ((Phase == 1) && ((A >= Amax) || (Pr >= (((Vmax * Vmax) - (VStart * VStart))/Amax) + (VStart * TE))))
            {
                if (A >= Amax)
                {
                    Phase = 3;
                }
                else
                {
                    /* Initialisation des variables pour la phase 4 */
                    A = 0.0;
                    A0 = 0.0;
                    J = 0.0;
                    J0 = 0.0;
                    PositionFinPhase4 = 0.0;
                
                    Phase = 4;
                }
            }
            else if ((Phase == 5) && (A <= -Dmax))
            {           
                Phase = 7;
            }

            break;

        /* Phase III ou Phase VII */   
        case 3:
        case 7:  
            
            /* Calcul S-Curve Profil */
        
            /* Position */
            P = P0 + (V0 * TE) + (0.5 * A0 * SQUARE(TE)) - ((1.0 / 6.0) * J0 * CUBE(TE));
        
            /* Vitesse */
            V = V0 + (A0 * TE) - (0.5 * J0 * SQUARE(TE));

            if (V > Vmax)
            {
                V = Vmax;
            }
            
            if (V < 0.0)
            {
               V = 0.0;
            }
                        
            /* Acceleration  */
            A = A0 - (J0 * TE);

            if (A > Amax)
            {
                A = Amax;
            }
            else if (A < -Dmax)
            {
                A = -Dmax;
            }
        
            /* Jerk */
            J = A / TE;              

            if (J > JAmax)
            {
                J = JAmax;
            }
            else if (J < -JDmax)
            {
                J = -JDmax;
            }
        
            /* ASR */
        
            /* Phase de decceleration */
            if (Phase == 7)
            {    
                if (ASRrunning == False)
                {
                    /* Vitesse Reelle superieure => delcanchement de l'ASR pour mieux deccelerer */
                    if ((Vr > (V0 + EcartVitesse)) && (fSatPI == False))
                    {
                        ASRrunning = True;
                    }
                }
                
                if (ASRrunning == True)
                {
                    /* Vitesse Reelle superieure => delcanchement de l'ASR pour mieux deccelerer */
                    if ((Vr > (V0 + EcartVitesse)) && (Vr > VminMouv) && (fSatPI == False))
                    {
                        /* Decrementation de la vitesse gain ASR */
                        VgASR = VgASR - gASR;
                        /* Modification de la vitesse suivant la vitesse du profil deceleration */
                        *Vconsigne = V + VgASR;
    
                        /* Verifiaction et ajustement de la vitesse de consigne */
                        if (*Vconsigne > Vmax)
                        {
                            *Vconsigne = Vmax;
                        } 
                        else if (*Vconsigne < -Vmax)
                        {
                            *Vconsigne = -Vmax;
                        }
                    }
                    else
                    {
                        /* Vitesse Reelle atteinte => correction de l'ASR */
                        if (VgASR < 0.0)
                        {   
                            /* Incrementation de la vitesse gain ASR */
                            VgASR = VgASR + gASR;
                            /* Modification de la vitesse suivant la vitesse du profil deceleration */
                            *Vconsigne = V + VgASR;
                                
                            /* Verifiaction et ajustement de la vitesse de consigne */
                            if (*Vconsigne > Vmax)
                            {
                                *Vconsigne = Vmax;
                            }
                            else if (Vr < VminMouv)
                            {
                                *Vconsigne = Vconsigne0 + gASR;
                            }
                        }
                        else
                        {
                            /* Correction de la vitesse pour eviter les sauts de vitesse en cours de profil et pourvoir freiner */
                            if (Vconsigne0 < V)
                            {
                                *Vconsigne = Vconsigne0;
                                  
                                /* Verifiaction et ajustement de la vitesse de consigne */                              
                                if (*Vconsigne > Vmax)
                                {
                                    *Vconsigne = Vmax;
                                }
                                else if (Vr < VminMouv)
                                {
                                    *Vconsigne = Vconsigne0 + gASR;
                                }                                                                
                            }
                            else
                            {
                                /* Consigne de vitesse*/
                                *Vconsigne = V;  
                                
                                /* Verifiaction et ajustement de la vitesse de consigne */
                                if (*Vconsigne > Vmax)
                                {
                                    *Vconsigne = Vmax;
                                }
                                else if (Vr < VminMouv)
                                {
                                    *Vconsigne = Vconsigne0 + gASR;
                                }                                                                                      
                            }                                                                                       
                        }
                    }
                }
                else
                {
                    /* Consigne de vitesse */
                    *Vconsigne = V;
                    
                    /* Verifiaction et ajustement de la vitesse de consigne */
                    if (*Vconsigne > Vmax)
                    {
                        *Vconsigne = Vmax;
                    }
                    else if (Vr < VminMouv)
                    {
                        *Vconsigne = Vconsigne0 + gASR;
                    }                                        
                }
            }
            /* Phase de d'acceleration */
            else
            {
                if (ASRrunning == False)
                {
                    /* Vitesse Reelle atteinte => delcanchement de l'ASR pour plus de performance */
                    if ((Vr >= (V0 - EcartVitesse)) && (fSatPI == False))
                    {
                        ASRrunning = True;
                    }
                }
                
                if (ASRrunning == True)
                {
                    /* Vitesse Reelle atteinte => delcanchement de l'ASR pour plus de performance */
                    if ((Vr >= (V0 - EcartVitesse)) && (fSatPI == False))
                    {
                        /* Incrementation de la vitesse gain ASR */
                        VgASR = VgASR + gASR;
                        /* Modification de la vitesse suivant la vitesse precedente */
                        *Vconsigne = Vconsigne0 + VgASR;
                            
                        /* Verifiaction et ajustement de la vitesse de consigne */
                        if (*Vconsigne > Vmax)
                        {
                            *Vconsigne = Vmax;
                        }
                        else if (*Vconsigne < VminMouv)
                        {
                            *Vconsigne = VminMouv;
                        }
                    }
                    else
                    {
                        /* Vitesse Reelle non atteinte => correction de l'ASR */
                        if (VgASR > 0.0)
                        {
                            /* Decrementation de la vitesse gain ASR */
                            VgASR = VgASR - gASR;
                            /* Modification de la vitesse suivant la vitesse precedente */
                            *Vconsigne = Vconsigne0 + VgASR;
                            
                            /* Verifiaction et ajustement de la vitesse de consigne */
                            if (*Vconsigne > Vmax)
                            {
                                *Vconsigne = Vmax;
                            }
                            else if (*Vconsigne < VminMouv)
                            {
                                *Vconsigne = VminMouv;
                            }
                        }
                        else
                        {
                            if (fSatPI == False)
                            {
                                /* Correction de la vitesse pour eviter les sauts de vitesse en cours de profil */
                                if (V >= Vconsigne0)
                                {
                                    /* Consigne de vitesse */
                                    *Vconsigne = V;
                                }
                                else
                                {
                                    *Vconsigne = Vconsigne0;
                                } 
                            }
                            else
                            {
                                /* Decrementation de la vitesse gain ASR */
                                VgASR = VgASR - gASR;
                                /* Modification de la vitesse suivant la vitesse precedente */
                                *Vconsigne = Vconsigne0 + VgASR;
                            }
                            
                            /* Verifiaction et ajustement de la vitesse de consigne */
                            if (*Vconsigne > Vmax)
                            {
                                *Vconsigne = Vmax;
                            }
                            else if (*Vconsigne < VminMouv)
                            {
                                *Vconsigne = VminMouv;
                            }                                                        

#ifdef PIC32_BUILD                                
                            TOOLS_LogFault(AsserPosErr, True, FLOAT, (float *)&Dmax, True, "Asserv_traj : Amax / Dmax > aux capacitees du robot !");
#else /* PIC32_BUILD */
                            ASSER_TRAJ_LogAsserMsgPC("Asserv_traj : Amax / Dmax > aux capacitees du robot !", Dmax);
#endif /* PIC32_BUILD */                            
                        }
                    }
                }
                else
                {
                    /* Consigne de vitesse */
                    *Vconsigne = V; 
                    
                    /* Verifiaction et ajustement de la vitesse de consigne */
                    if (*Vconsigne > Vmax)
                    {
                        *Vconsigne = Vmax;
                    }
                    else if (*Vconsigne < VminMouv)
                    {
                        *Vconsigne = VminMouv;
                    }                                        
                }
            }
            
            /* Sauvergarde de valeur pour la porchaine iteration */
            A0 = A;
            V0 = V;
            P0 = P;
            J0 = J;
                                   
            /* Test fin de Phase */
            if ((Phase == 3) && ((A <= 0.0) || (Pr >= (((Vmax * Vmax) - (VStart * VStart))/Amax) + (VStart * TE))))
            {
                /* Initialisation des variables pour la phase 4 */
                A = 0.0;
                A0 = 0.0;
                J = 0.0;
                J0 = 0.0;
                PositionFinPhase4 = 0.0;
        
                /* Reset gain en vitesse ASR */
                VgASR = 0.0;
                ASRrunning = False;
                        
                Phase = 4;
             } 
             /* Phase de decceleration */          
             else
             {
#ifdef PIC32_BUILD                 
                if (Pr >= (Distance - (Vr * TE)))
#else /* PIC32_BUILD */
                if (Pr >= Distance)
#endif /* PIC32_BUILD */  

                {
                    Phase = 8;
                    
                    if ((Vr > (VminMouv + EcartVitesse)) && (VEnd == 0.0))
                    {
#ifdef PIC32_BUILD                        
                        TOOLS_LogFault(AsserPosErr, True, FLOAT, (float *)&Vr, True, "Asserv_traj : Vr > VminMouv a l'arrivee en position !");
#else /* PIC32_BUILD */
                        ASSER_TRAJ_LogAsserMsgPC("Asserv_traj : Vr > VminMouv a l'arrivee en position !", Vr);
#endif /* PIC32_BUILD */  
                    }
                }
             }

             break;

        case 4:
                        
            /* Calcul S-Curve Profil */

            /* Position */
            P = P0 + (V0 * TE) + (0.5 * A0 * SQUARE(TE)) + ((1.0 / 6.0) * J0 * CUBE(TE));

            /* ASR */
            if (ASRrunning == True)
            {
                ASRrunning = False;
            }
                    
            /* Consigne de vitesse */
            *Vconsigne = Vmax;
            
            /* Sauvergarde de valeur pour la porchaine iteration */
            P0 = Pr;
            V0 = Vmax;
            
            /* Calcul distance fin de Phase */
            if (PositionFinPhase4 <= 0.0)
            {
                PositionFinPhase4 = Distance - (((Vmax * Vmax) - (VEnd * VEnd))/Dmax) - (VEnd * TE) -  (Vmax * TE);
            }
          
            /* Test fin de Phase */
            if ((PositionFinPhase4 > 0.0) && (Pr >= PositionFinPhase4))
            {
                J0 = -JDmax;
                Phase = 5;
            } 

            break;

        default:

#ifdef PIC32_BUILD
            TOOLS_LogFault(AsserPosErr, True, INTEGER, (int *)&Phase, True, "Asserv_traj : Phase de profil non gere !");
#else /* PIC32_BUILD */
            ASSER_TRAJ_LogAsserMsgPC("Asserv_traj : Phase de profil non gere !", (float)Phase);
#endif /* PIC32_BUILD */  

            break;
    }

    /* Sauvegarde de la vitesse precedente */
    Vconsigne0 = *Vconsigne;

    /* Phase VIII (fin d'asser) */
    if (Phase == 8)
    {           
        /* Reset Phase */
        Phase = 0;
                   
        /* Stop */
        AsserRunningFlag = False;
    }

    return AsserRunningFlag;
}

/*! @} */

/*! @} */

/*! @} */

/* End of File : asserv_trajectoire.c */

