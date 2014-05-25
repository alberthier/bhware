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
#include "task_main.h"
#else /* PIC32_BUILD */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#endif /* PIC32_BUILD */
#include "task_asser.h"
#include "asserv_trajectoire.h"
#include "pic18.h"

/*! \addtogroup Asser
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

/* MACROS */
#define                         SQUARE(a)                               ((a)*(a))
#define                         CUBE(a)                                 ((a)*(a)*(a))
#define                         POWER4(a)                               ((a)*(a)*(a)*(a))
#define                         POWER5(a)                               ((a)*(a)*(a)*(a)*(a))

#define                         TEST_BIT(a, n)                          ((a & (1 << n)) >>  n)

/* Pour t = t1 = 0.1 */
#define                         SQUARE_t1                               ((float) 0.01)
#define                         CUBE_t1                                 ((float) 0.001)
#define                         POWER4_t1                               ((float) 0.0001)
#define                         POWER5_t1                               ((float) 0.00001)
#define                         POWER6_t1                               ((float) 0.000001)
#define                         POWER7_t1                               ((float) 0.0000001)
#define                         POWER8_t1                               ((float) 0.00000001)
#define                         POWER9_t1                               ((float) 0.000000001)

/* Pour t = t1/2.0 = 0.05 */
#define                         SQUARE_t1_2                             ((float) 0.0025)
#define                         CUBE_t1_2                               ((float) 0.000125)
#define                         POWER4_t1_2                             ((float) 0.00000625)
#define                         POWER5_t1_2                             ((float) 0.0000003125)

/* Constantes */

#define                         Vitesse_Gain_ASR                        ((float) 0.002)
#define                         DISTANCE_SPLINE3                        ((float) 0.018)
#define                         ANGLE_STEP                              ((float) 0.02 * PI)
#define                         R_INV                                   ((float)(1.0 / (ECART_ROUE_MOTRICE / 2.0)))
#define                         AMAX_REDUCTION                          ((float) 0.6)
#define                         VMAX_REDUCTION                          ((float) 0.6)

/*----------------------------------------------------------------------------------------------*/

/* Variables globales */

/** Parametres de l'asservissement par retour d'etat */
float                           gainCentreRot                           =   20.0;

float                           gainDeplacement1                        =   20.0;
float                           gainDeplacement2                        =   50.0;
float                           gainDeplacement3                        =   20.0;

/** Parametres de la generation de trajectoire */
float                           Ratio_Acc                               =   1.0;
float                           Ratio_Decc                              =   1.0;
float                           Ratio_Acc_Rot                           =   0.5;
float                           Ratio_Decc_Rot                          =   0.8;
float                           Vmax_limit                              =   0.7;

/** Vitesse minimum de mouvement de reference*/
float                           VminMouvRef                             =   0.1; 

float                           A_MAX                                   =   1.0;
float                           AmaxTemp                                =   1.0;
float                           D_MAX                                   =   1.0;


unsigned int                    ASSER_compteurPeriode                   =   0;
unsigned int                    ASSER_segmentCourant                    =   1;
unsigned int                    ASSER_subSegmentCourant                 =   0;

/** Structure chemin contenant le definition de la trajectoire a realiser */
Deplacement                     chemin;

/* Vitesse minimum de mouvement et maximale d'arret */
float                           VminMouv                                = 0.1;
const float                     EcartVitesseDecc                        = 0.001;
float                           EcartVitesseAcc                         = 0.035;

unsigned char                   VmaxArrivedInAcc                        = False;

/***********************************************************************************************/
/**************************** Fin de la definition des parametres de l'asservissement de trajectoire *********/
/***********************************************************************************************/

/*----------------------------------------------------------------------------------------------*/

/* Variables locales */

/** Le point du robot asservit a la trajectoire n'est pas le centre de l'axe des roues, mais un point sur la droite perpendiculaire 
a cet axe et passant par son centre, situe a la distance NORME_BARRE_SUIVI_TRAJ en avant de l'axe des roues */
static float                    NORME_BARRE_SUIVI_TRAJ;         /* Initialise dans ASSER_TRAJ_InitialisationGenerale() */

/** Variables globales de l'asservissement de trajectoire */
static float                    errDist, memo_errDist, errAngle, memo_errAngle, memo_angleRobot;

/** Vecteurs associees au depart et a l'arrivee, et parametrage du profil de vitesse de la trajectoire */
static unsigned int             g_iSeg                                  =   0u;
static unsigned int             g_iSubSeg                               =   0u;
static unsigned int             g_memoSegmentCourant                    =   0;
static unsigned char            g_memoSubSegmentCourant                 =   0;
static char                     m_sensDeplacement                       =   MARCHE_AVANT;
static float                    distanceTotale_Profil                   =   0.0;
static float                    vitesse_debut_profil                    =   0.0;
static float                    vitesse_fin_profil                      =   0.0;

/** Contraintes d'affectation : ti doit etre compris entre 0 et 0.5, avec les valeurs 0.0 et 0.5 exclus */
static const float              ti                                      =   0.1;

/** Config Spline3 */
static const float              Db                                      =   0.017;
static const float              Da                                      =   0.001;//0.0006;

/** Pose de la trajectoire de consigne a l'instant t*/
static Pose                     poseReference                           =   {0.0,0.0,0.0};
static Pose                     poseReferenceRobot                      =   {0.0,0.0,0.0};
static Pose                     poseReferenceAv                         =   {0.0,0.0,0.0};

/** Distance a parcourir pendant la periode courante, distance qui est normalisee par la distance totale a parcourir.
Elle sert a faire passer la vitesse d'une commande d'asser a l'autre en cas de changement de trajectoire en cours de mouvement */
static float                    vitesse_profil_consigne                 =   0.0;
static unsigned char            shuntTestFinAsser                       =   False;

/** Variables profil de vitesse Scurve */
static float                    Vmax, VgASR, Vconsigne0, J, A, V, P, J0, A0, V0, P0, DistanceMin, JAmax, JDmax, PositionFinPhase4;
static unsigned char            Phase                                   = 0;
static unsigned char            ASRrunning                              = False;

static float                    distanceParcourue_Profil                = 0.0;

static float                    erreurAngleMemo                         = 0.0;
static float                    tempwMemo                               = 0.0;


/** Structure de configurationd de spline de type 3 */
static ConfigSpline3            cfgSp3;

/*----------------------------------------------------------------------------------------------*/

/* Prototypes des fonctions */

static void                     ASSER_TRAJ_MatriceRotation(Vecteur *matrice2p2, float angle);
static Pose                     ASSER_TRAJ_TrajectoireRotation(ParametresRotation *p_rotation, float t);
static Pose                     ASSER_TRAJ_DiffTemporelleTrajectoire(Pose posePrecedente, Pose poseActuelle, float periode);
static Vecteur                  ASSER_TRAJ_ProduitMatriceVecteur(Vecteur *matrice, Vecteur vecteur);
static Pose                     ASSER_TRAJ_ErreurPose(Pose poseRobot_P, Pose poseRef);
static VitessesRobotUnicycle    ASSER_TRAJ_RetourDetatOrientation(Pose erreurPose, Pose diffPoseRef, float gain[]);
static unsigned char            ASSER_TRAJ_TestFinAsservissement(Deplacement *traj, float erDist, float memo_erDist, float tolDist, float erAngle, float memo_erAngle, float tolAngle);
static void                     ASSER_TRAJ_DistanceTrajectoire(Trajectoire * traj, unsigned int iSegment);
static void                     ASSER_TRAJ_Trajectoire_SubSegment(segmentTrajectoire * segmentTraj, unsigned char subSeg, float pp, Pose *poseTraj, Vecteur *diff1Traj, Vecteur *diff2Traj);
static float                    ASSER_TRAJ_DiffThetaBSpline(Vecteur diff1BS, Vecteur diff2BS);
static Pose                     ASSER_TRAJ_TrajectoireRemorqueBS(Trajectoire * traj, unsigned int iSegment, unsigned char iSubSegment, float t, Pose * poseTrajRobot);
static float                    ASSER_TRAJ_CalculTheta1(unsigned int iSegment, unsigned int nbrePts, PtTraj* point, float angle_rad, float prec_x, float prec_y);
static float                    ASSER_TRAJ_Rinv_courbure(segmentTrajectoire * segmentTraj, unsigned char subSeg, float t);
static int                      ASSER_TRAJ_sds_ab(ConfigSegment * cfgSeg, segmentTrajectoire * segmentTraj);
static void                     ASSER_TRAJ_Rotation_coord(float theta, float x, float y, float * x_n, float * y_n);
static int                      ASSER_TRAJ_sds_ab_base(float x1, float y1, float theta1, float x2, float theta2, float t1, unsigned char n1, unsigned char n2, float qx1_0, float qy1_0, float qx2_0, float qy2_0, float bx1, ConfigSpline34 * cfgSp1, ConfigSpline34 * cfgSp2);
static void                     ASSER_TRAJ_Test_courbure(Deplacement *traj, ConfigSegment * cfgSeg, segmentTrajectoire  * segmentTraj);
static int                      ASSER_TRAJ_Generation_curvatureForced(ConfigSegment * cfgSeg, segmentTrajectoire  * segmentTraj);
static void                     ASSER_TRAJ_InitialSplineForCircle(ConfigSegment * cfgSeg, segmentTrajectoire  * segmentTraj, unsigned char subSeg);
static void                     ASSER_TRAJ_Rotation_config(ConfigSegment * cfgSeg, segmentTrajectoire  * segmentTraj, unsigned char subSeg, ConfigArc * arc);
static float                    ASSER_TRAJ_Spline34(float t, float t1, unsigned char n, float q, float a, float b, float c, unsigned char deriv);
static void                     ASSER_TRAJ_IdentificationPoly4(float y0, float dy0, float yi, float y1, float dy1, float * a, float * b, float * c, float * d, float * e);
#ifdef PIC32_BUILD
void                            ASSER_TRAJ_LogAsserPIC(char * keyWord, float Val1, float * pVal2, float * pVal3, float * pVal4, float * pVal5);
#else /* PIC32_BUILD */
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
    erreurAngleMemo = 0.0;
    tempwMemo = 0.0;

    /* Initialisation du profil de vitesse */
    chemin.profilVitesse.p = 0;

    /* Calcul de la norme suivi trajectoire */
    NORME_BARRE_SUIVI_TRAJ = ECART_ROUE_MOTRICE / 2.0;

    /* Initialisation de la position du robot */
    POS_InitialisationConfigRobot();

    /* Initialisation de la vitesse max du robot */
    Vmax_limit = MIN(DonneeVmaxGauche, DonneeVmaxDroite);

    /* Initialisation de la spline 3 de transition entre une ligne droite et un arc de cercle */
    cfgSp3.bx = Db / ti;
    cfgSp3.by = 0.0;

    cfgSp3.ax = 0.0;
    cfgSp3.ay = (6.0 * Da) / SQUARE_t1;
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
extern unsigned char ASSER_TRAJ_isDeplacement(Deplacement * traj)
{
    if (traj->mouvement > ROTATE)
    {
        return True;
    }
    else
    {
        return False;
    }
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
    float           gain[3], Vr;
    Pose            differentielleTemporellePoseReference, differentielleTemporellePoseReferenceRobot, erreurPoseCentreRobot, erreur_P;
    Vecteur         diff1Traj                               = {0.0, 0.0};
    Vecteur         diff2Traj                               = {0.0, 0.0};
    unsigned int    segmentCourantAv;
    unsigned char   subSegmentCourantAv; 
    Pose            poseReferenceRobotAv                    = {0.0, 0.0, 0.0};
    Pose            poseTraj                                = {0.0, 0.0, 0.0};
    float           delta_distance_Av                       = 0.0;
    float           vitesse_fin_profil_0                    = 0.0;
    float           VitesseProfil                           = 0.0;
    float           distSupp                                = 0.0;
    float           diffThetaTrajectoire                    = 0.0;
    unsigned char   iSubSeg                                 = 0u;
    float           dl_dt_0, dl_dt_1, d2l_dt2_0, d2l_dt2_1, dl_dt_i;
    float           a_dlc                                   = 0.0;
    float           b_dlc                                   = 0.0;
    float           c_dlc                                   = 0.0;
    float           d_dlc                                   = 0.0;
    float           e_dlc                                   = 0.0;
    float           Rinv_0, Rinv_1, dRinv_0, dRinv_1, Rinv_i;
    float           a_Rinv                                  = 0.0;
    float           b_Rinv                                  = 0.0;
    float           c_Rinv                                  = 0.0;
    float           d_Rinv                                  = 0.0;
    float           e_Rinv                                  = 0.0;
    float           delta_distance                          = 0.0;
    float           parametrePositionSegmentTrajectoireAv   = 0.0;
    unsigned int    memo_segmentCourant                     = 0;
    unsigned char   flag_ASSER_TRAJ_TestFinAsservissement   = False;
#ifndef PIC32_BUILD
    unsigned char   memo_subSegmentCourant                  = 0;
#endif

#ifdef PIC32_BUILD  
    INT8U           OS_Status                               = OS_ERR_NONE;  
#endif /* PIC32_BUILD */  
    
    /* Log de valeurs */
#ifdef PLOTS_SIMU
    ASSER_TRAJ_LogAsserValPC("xRoueGauche", m_poseRobot.x + NORME_BARRE_SUIVI_TRAJ * cosf(m_poseRobot.angle + (PI / 2)));
    ASSER_TRAJ_LogAsserValPC("yRoueGauche", m_poseRobot.y + NORME_BARRE_SUIVI_TRAJ * sinf(m_poseRobot.angle + (PI / 2)));
    ASSER_TRAJ_LogAsserValPC("xRoueDroite", m_poseRobot.x + NORME_BARRE_SUIVI_TRAJ * cosf(m_poseRobot.angle - (PI / 2)));
    ASSER_TRAJ_LogAsserValPC("yRoueDroite", m_poseRobot.y + NORME_BARRE_SUIVI_TRAJ * sinf(m_poseRobot.angle - (PI / 2)));
    ASSER_TRAJ_LogAsserValPC("angle", m_poseRobot.angle);
#endif /* PLOTS_SIMU */

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

    ASSER_TRAJ_LogAsserValPC("ASSER_compteurPeriode", ASSER_compteurPeriode);

    /* Test fin d'asser */
    if (ASSER_compteurPeriode > 0)
    {
        flag_ASSER_TRAJ_TestFinAsservissement = ASSER_TRAJ_TestFinAsservissement(&chemin, errDist, memo_errDist, DIST_MIN, errAngle, memo_errAngle, ANGLE_MIN);
        if (flag_ASSER_TRAJ_TestFinAsservissement == True)
        {
            if ((chemin.profilVitesse.distNormaliseeRestante * chemin.distance) < ((float)0.05))
            {
                if (shuntTestFinAsser == False)
                {
                    Phase = 0;
                                
                    ASSER_Running = False;
                
                    if (ASSER_TRAJ_isDeplacement(&chemin) == True)
                    {    
                        if (Vmax >= MIN(DonneeVmaxGauche, DonneeVmaxDroite))
                        {
                            Vr = fabs(POS_GetVitesseRelle());

                            if (Vr > (VminMouv + EcartVitesseDecc))
                            {
#ifdef PIC32_BUILD      
                                if (Test_mode == (unsigned long)1)
                                {
                                    TOOLS_LogFault(AsserPosErr, True, FLOAT, (float *)&Vr, True, "Vr>VminMouv a l'arrivee");
                                }
#else /* PIC32_BUILD */
                                ASSER_TRAJ_LogAsserMsgPC("Asser: Vr>VminMouv a l'arrivee", Vr);
#endif /* PIC32_BUILD */  
                            }
                        }
                    }
                    else
                    {
                        Vr = fabs((((float)m_sensDeplacement) * POS_GetVitesseRotation() * (ECART_ROUE_MOTRICE / 2.0)));
                        if (Vr > (VminMouv + EcartVitesseDecc))
                        {
#ifdef PIC32_BUILD      
                            if (Test_mode == (unsigned long)1)
                            {
                                TOOLS_LogFault(AsserPosErr, True, FLOAT, (float *)&Vr, True, "Vr>VminMouv a l'arrivee");
                            }
#else /* PIC32_BUILD */
                            ASSER_TRAJ_LogAsserMsgPC("Asser: Vr>VminMouv a l'arrivee", Vr);
#endif /* PIC32_BUILD */  
                        }
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
        flag_ASSER_TRAJ_TestFinAsservissement = ASSER_TRAJ_TestFinAsservissement(&chemin, errDist, memo_errDist, DIST_MIN, errAngle, memo_errAngle, ANGLE_MIN);
        if (flag_ASSER_TRAJ_TestFinAsservissement == True)
        {
            if ((ASSER_TRAJ_isDeplacement(&chemin) == True) & (chemin.trajectoire.subTrajs.nbreSegments == (unsigned int)1) \
                    | (ASSER_TRAJ_isDeplacement(&chemin) == False))
            {
                Phase = 0;
            
                ASSER_Running = False;
                
                if (ASSER_TRAJ_isDeplacement(&chemin) == True)
                {    
                    if (Vmax >= MIN(DonneeVmaxGauche, DonneeVmaxDroite))
                    {
                        Vr = fabs(POS_GetVitesseRelle());

                        if (Vr > (VminMouv + EcartVitesseDecc))
                        {
#ifdef PIC32_BUILD      
                            if (Test_mode == (unsigned long)1)
                            {
                                TOOLS_LogFault(AsserPosErr, True, FLOAT, (float *)&Vr, True, "Vr>VminMouv a l'arrivee");
                            }
#else /* PIC32_BUILD */
                            ASSER_TRAJ_LogAsserMsgPC("Asser: Vr>VminMouv a l'arrivee", Vr);
#endif /* PIC32_BUILD */  
                        }
                    }
                }
                else
                {
                    Vr = fabs((((float)m_sensDeplacement) * POS_GetVitesseRotation() * (ECART_ROUE_MOTRICE / 2.0)));

                    if (Vr > (VminMouv + EcartVitesseDecc))
                    {
#ifdef PIC32_BUILD      
                        if (Test_mode == (unsigned long)1)
                        {
                            TOOLS_LogFault(AsserPosErr, True, FLOAT, (float *)&Vr, True, "Vr>VminMouv a l'arrivee");
                        }
#else /* PIC32_BUILD */
                        ASSER_TRAJ_LogAsserMsgPC("Asser: Vr>VminMouv a l'arrivee", Vr);
#endif /* PIC32_BUILD */  
                    }
                }
            }
            else
            {
                shuntTestFinAsser = True;
            }
        }
    }
    ASSER_TRAJ_LogAsserValPC("flag_ASSER_TRAJ_TestFinAsservissement", flag_ASSER_TRAJ_TestFinAsservissement);

    /* distance de deplacement sur la periode */
    if (ASSER_TRAJ_isDeplacement(&chemin) == True)
    {
        delta_distance = cosf(poseReferenceRobot.angle) * (poseRobot.x - poseReferenceRobot.x) + sinf(poseReferenceRobot.angle) * (poseRobot.y - poseReferenceRobot.y);
        ASSER_TRAJ_LogAsserValPC("delta_distance", delta_distance);
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
        ASSER_compteurPeriode++;

        /* Generation de la pose de trajectoire de consigne suivante */
        if ( (chemin.profilVitesse.p > 0) || (chemin.profilVitesse.etat == 1) )
        {
            if (ASSER_TRAJ_isDeplacement(&chemin) == True)
            {
                /************************************************************/
                /* Debut de la dermination de la nouvelle position a suivre                     */
                /************************************************************/

                if (chemin.profilVitesse.p > 0)
                {
                    /* distance normalisee restant a parcourir */
                    chemin.profilVitesse.distNormaliseeRestante -= (delta_distance / chemin.distance);

                    memo_segmentCourant = chemin.trajectoire.subTrajs.segmentCourant;
    #ifndef PIC32_BUILD
                    memo_subSegmentCourant = chemin.trajectoire.subTrajs.subSegmentCourant;
    #endif /* PIC32_BUILD */
                    ASSER_TRAJ_LogAsserValPC("segmentCourant", chemin.trajectoire.subTrajs.segmentCourant);
                    ASSER_TRAJ_LogAsserValPC("subSegmentCourant", chemin.trajectoire.subTrajs.subSegmentCourant);
                    ASSER_TRAJ_LogAsserValPC("paramPoseSubSegCourant", chemin.trajectoire.subTrajs.paramPoseSubSegCourant);

                    ASSER_TRAJ_ParcoursTrajectoire(&chemin \
                                                   , delta_distance \
                                                   , &chemin.trajectoire.subTrajs.segmentCourant \
                                                   , &chemin.trajectoire.subTrajs.subSegmentCourant \
                                                   , &chemin.trajectoire.subTrajs.paramPoseSubSegCourant \
                                                   , NULL);

                    if (memo_segmentCourant != chemin.trajectoire.subTrajs.segmentCourant)
                    {
                        ASSER_TRAJ_LogAsserValPC("periodeNewSeg", ASSER_compteurPeriode);

    #ifdef PIC32_BUILD
                        MAIN_Way_Point_Index = memo_segmentCourant;

                        /* Arrivee au point intermediaire */
                        MAIN_event = OSFlagPost(MAIN_event_gpr, MAIN_GOTO_WAYPOINT_REACHED, OS_FLAG_SET, &OS_Status);
                        if(OS_Status != OS_ERR_NONE)
                        {
                            TOOLS_LogFault(OS_Err, True, INTEGER, &OS_Status, True, "Post flag error");
                        }
    #endif /* PIC32_BUILD */
                    }
    #ifndef PIC32_BUILD
                    if ((memo_subSegmentCourant != chemin.trajectoire.subTrajs.subSegmentCourant) \
                        | (memo_segmentCourant != chemin.trajectoire.subTrajs.segmentCourant))
                    {
                        ASSER_TRAJ_LogAsserValPC("periodeNewSubSeg", ASSER_compteurPeriode);
                        ASSER_TRAJ_LogAsserValPC("xNewSubSeg", poseRobot.x);
                        ASSER_TRAJ_LogAsserValPC("yNewSubSeg", poseRobot.y);
                    }
    #endif /* PIC32_BUILD */
                }
                else
                {
                    /* Commande de vitesse nulle, le point d'arrivee de consigne est atteint */
                    chemin.trajectoire.subTrajs.paramPoseSubSegCourant = ti;
                }

                /**********************************************************/
                /* Fin de la dermination de la nouvelle position a suivre                      */
                /**********************************************************/


                ASSER_TRAJ_Trajectoire(&chemin.trajectoire \
                                       , chemin.trajectoire.subTrajs.segmentCourant \
                                       , chemin.trajectoire.subTrajs.subSegmentCourant \
                                       , chemin.trajectoire.subTrajs.paramPoseSubSegCourant \
                                       , &poseTraj, &diff1Traj, &diff2Traj);
                
                chemin.profilVitesse.diffThetaCourant = ASSER_TRAJ_DiffThetaBSpline(diff1Traj, diff2Traj);
                
                poseReference = ASSER_TRAJ_TrajectoireRemorqueBS(&chemin.trajectoire \
                                                                 , chemin.trajectoire.subTrajs.segmentCourant \
                                                                 , chemin.trajectoire.subTrajs.subSegmentCourant \
                                                                 , chemin.trajectoire.subTrajs.paramPoseSubSegCourant \
                                                                 , &poseReferenceRobot);

                ASSER_TRAJ_LogAsserValPC("Vend_01", vitesse_fin_profil);
                ASSER_TRAJ_LogAsserValPC("Vstart_01", vitesse_debut_profil);

                if (chemin.trajectoire.subTrajs.subSegmentCourant != g_memoSubSegmentCourant)
                {
                    g_memoSubSegmentCourant = chemin.trajectoire.subTrajs.subSegmentCourant;

                    if ((chemin.trajectoire.subTrajs.segmentCourant == g_iSeg) \
                    &&  (chemin.trajectoire.subTrajs.subSegmentCourant == g_iSubSeg))
                    {
                        g_iSeg = chemin.trajectoire.subTrajs.segmentCourant;
                        g_iSubSeg = chemin.trajectoire.subTrajs.subSegmentCourant;
                        distanceTotale_Profil = 0.0;

                        do
                        {
                            /* Determination de la longueur du sous-segment a l'exterieur*/
                            switch(g_iSubSeg)
                            {
                                case SPLINE31 :
                                case SPLINE341 :
                                case SPLINE342 :
                                case SPLINE32 :

                                    ASSER_TRAJ_Trajectoire_SubSegment(&chemin.trajectoire.subTrajs.segmentTraj[g_iSeg], g_iSubSeg, 0.0, &poseTraj, &diff1Traj, &diff2Traj);
                                    dl_dt_0 = sqrt(SQUARE(diff1Traj.x) + SQUARE(diff1Traj.y));
                                    d2l_dt2_0 = sqrt(SQUARE(diff2Traj.x) + SQUARE(diff2Traj.y));
                                    ASSER_TRAJ_Trajectoire_SubSegment(&chemin.trajectoire.subTrajs.segmentTraj[g_iSeg], g_iSubSeg, (ti / 2.0), &poseTraj, &diff1Traj, &diff2Traj);
                                    dl_dt_i = sqrt(SQUARE(diff1Traj.x) + SQUARE(diff1Traj.y));
                                    ASSER_TRAJ_Trajectoire_SubSegment(&chemin.trajectoire.subTrajs.segmentTraj[g_iSeg], g_iSubSeg, ti, &poseTraj, &diff1Traj, &diff2Traj);
                                    dl_dt_1 = sqrt(SQUARE(diff1Traj.x) + SQUARE(diff1Traj.y));
                                    d2l_dt2_1 = sqrt(SQUARE(diff2Traj.x) + SQUARE(diff2Traj.y));

                                    ASSER_TRAJ_IdentificationPoly4(dl_dt_0, d2l_dt2_0, dl_dt_i, dl_dt_1, d2l_dt2_1, &a_dlc, &b_dlc, &c_dlc, &d_dlc, &e_dlc);

                                    /* Longueur centrale de la spline */
                                    distanceTotale_Profil += ((a_dlc * POWER5_t1) / 5.0) + ((b_dlc * POWER4_t1) / 4.0) + ((c_dlc * CUBE_t1) / 3.0) + ((d_dlc * SQUARE_t1) / 2.0) + (e_dlc * ti);

                                    Rinv_0 = fabsf(ASSER_TRAJ_Rinv_courbure(&chemin.trajectoire.subTrajs.segmentTraj[g_iSeg] \
                                                                      , g_iSubSeg \
                                                                      , 0.0));
                                    dRinv_0 = fabsf(ASSER_TRAJ_Rinv_courbure(&chemin.trajectoire.subTrajs.segmentTraj[g_iSeg] \
                                                                       , g_iSubSeg \
                                                                       , (0.01 * ti)));
                                    dRinv_0 = (dRinv_0 - Rinv_0) / (0.01 * ti);
                                    Rinv_i = fabsf(ASSER_TRAJ_Rinv_courbure(&chemin.trajectoire.subTrajs.segmentTraj[g_iSeg] \
                                                                      , g_iSubSeg \
                                                                      , (ti / 2.0)));
                                    Rinv_1 = fabsf(ASSER_TRAJ_Rinv_courbure(&chemin.trajectoire.subTrajs.segmentTraj[g_iSeg] \
                                                                      , g_iSubSeg \
                                                                      , ti));
                                    dRinv_1 = fabsf(ASSER_TRAJ_Rinv_courbure(&chemin.trajectoire.subTrajs.segmentTraj[g_iSeg] \
                                                                       , g_iSubSeg \
                                                                       , (0.99 * ti)));
                                    dRinv_1 = (Rinv_1 - dRinv_1) / (0.01 * ti);

                                    ASSER_TRAJ_IdentificationPoly4(Rinv_0, dRinv_0, Rinv_i, Rinv_1, dRinv_1, &a_Rinv, &b_Rinv, &c_Rinv, &d_Rinv, &e_Rinv);

                                    distSupp = ((a_dlc * a_Rinv) * POWER9_t1) / 9.0;
                                    distSupp += (((a_dlc * b_Rinv) + (a_Rinv * b_dlc)) * POWER8_t1) / 8.0;
                                    distSupp += (((a_dlc * c_Rinv) + (a_Rinv * c_dlc) + (b_dlc * b_Rinv)) * POWER7_t1) / 7.0;
                                    distSupp += (((a_dlc * d_Rinv) + (a_Rinv * d_dlc) + (b_dlc * c_Rinv) + (b_Rinv * c_dlc)) * POWER6_t1) / 6.0;
                                    distSupp += (((a_dlc * e_Rinv) + (a_Rinv * e_dlc) + (b_dlc * d_Rinv) + (b_Rinv * d_dlc) + (c_dlc * c_Rinv)) * POWER5_t1) / 5.0;
                                    distSupp += (((b_dlc * e_Rinv) + (b_Rinv * e_dlc) + (c_dlc * d_Rinv) + (c_Rinv * d_dlc)) * POWER4_t1) / 4.0;
                                    distSupp += (((c_dlc * e_Rinv) + (c_Rinv * e_dlc) + (d_dlc * d_Rinv)) * CUBE_t1) / 3.0;
                                    distSupp += (((d_dlc * e_Rinv) + (d_Rinv * e_dlc)) * SQUARE_t1) / 2.0;
                                    distSupp += (e_dlc * e_Rinv) * ti;
                                    distSupp = distSupp * (ECART_ROUE_MOTRICE / 2.0);
                                    ASSER_TRAJ_LogAsserValPC("distSupp",  distSupp);

                                    distanceTotale_Profil += distSupp;

                                    AmaxTemp = chemin.profilVitesse.Amax * AMAX_REDUCTION;

                                    break;

                                case ARC1 :
                                    distanceTotale_Profil += (NORME_BARRE_SUIVI_TRAJ + (1.0 / chemin.trajectoire.subTrajs.segmentTraj[g_iSeg].arc1.rayon_inverse) ) * fabs(chemin.trajectoire.subTrajs.segmentTraj[chemin.trajectoire.subTrajs.segmentCourant].arc1.angle);
                                    AmaxTemp = chemin.profilVitesse.Amax * AMAX_REDUCTION;
                                    break;

                                case ARC2 :
                                    distanceTotale_Profil += (NORME_BARRE_SUIVI_TRAJ + (1.0 / chemin.trajectoire.subTrajs.segmentTraj[g_iSeg].arc2.rayon_inverse) ) * fabs(chemin.trajectoire.subTrajs.segmentTraj[chemin.trajectoire.subTrajs.segmentCourant].arc2.angle);
                                    AmaxTemp = chemin.profilVitesse.Amax * AMAX_REDUCTION;
                                    break;

                                case LINE :
                                    distanceTotale_Profil += sqrtf(((float)SQUARE(chemin.trajectoire.subTrajs.segmentTraj[g_iSeg].line.ax) + SQUARE(chemin.trajectoire.subTrajs.segmentTraj[g_iSeg].line.ay))) * ti;
                                    AmaxTemp = chemin.profilVitesse.Amax;
                                    break;

                                default :
                                    AmaxTemp = chemin.profilVitesse.Amax;
#ifdef PIC32_BUILD
                                    TOOLS_LogFault(AsserPosErr, True, INTEGER, (int *)&iSubSeg, True, "dist supp");
#else /* PIC32_BUILD */
                                    ASSER_TRAJ_LogAsserMsgPC("Asser: dist supp pour vitesse en courbe non gere", (float)iSubSeg);
#endif /* PIC32_BUILD */
                                    return;
                            }


                            /* Passage au sous-segment suivant */
                            do
                            {
                                g_iSubSeg++;
                                if (g_iSubSeg > SPLINE32)
                                {
                                    /* Passage au segment suivant */
                                    g_iSeg++;
                                    if (g_iSeg < chemin.trajectoire.subTrajs.nbreSegments)
                                    {
                                        g_iSubSeg = chemin.trajectoire.subTrajs.segmentTraj[g_iSeg].subSeg_firstUsed;
                                    }
                                }
                            } while ((TEST_BIT(chemin.trajectoire.subTrajs.segmentTraj[g_iSeg].subSeg_used.Info, g_iSubSeg) == False) && (g_iSeg < chemin.trajectoire.subTrajs.nbreSegments));
                        } while ( (g_iSubSeg != LINE) && ((chemin.trajectoire.subTrajs.subSegmentCourant != LINE) || (g_iSeg == (chemin.trajectoire.subTrajs.nbreSegments - 1))) && (g_iSeg < chemin.trajectoire.subTrajs.nbreSegments));

                        /* Init */
                        distanceParcourue_Profil = 0.0;

                        /* determination des vitesses initiale et finale de chaque profil S-curve */
                        if ((chemin.trajectoire.subTrajs.segmentCourant == 0)
                        &&  (chemin.trajectoire.subTrajs.subSegmentCourant == chemin.trajectoire.subTrajs.segmentTraj[chemin.trajectoire.subTrajs.segmentCourant].subSeg_firstUsed))
                        {
                            vitesse_debut_profil = 0.0;
                        }
                        else
                        {
                            vitesse_debut_profil = MIN(vitesse_fin_profil, (((float)m_sensDeplacement) * POS_GetVitesseRelle()));
                        }

                        if (g_iSeg == chemin.trajectoire.subTrajs.nbreSegments)
                        {
                            vitesse_fin_profil = 0.0;
                        }
                        else
                        {
                            diffThetaTrajectoire = ASSER_TRAJ_DiffThetaBSplinePerLenghtUnit(&chemin.trajectoire \
                                                                                            , g_iSeg \
                                                                                            , g_iSubSeg \
                                                                                            , 0.0);

                            vitesse_fin_profil_0 = ASSER_TRAJ_VitesseLimiteEnVirage(&chemin, diffThetaTrajectoire);

                            delta_distance_Av = 0.02;
                            parametrePositionSegmentTrajectoireAv = 0.0;
                            segmentCourantAv = g_iSeg;
                            subSegmentCourantAv = g_iSubSeg;

                            ASSER_TRAJ_ParcoursTrajectoire(&chemin \
                                                           , delta_distance_Av \
                                                           , &segmentCourantAv \
                                                           , &subSegmentCourantAv \
                                                           , &parametrePositionSegmentTrajectoireAv \
                                                           , NULL);

                            diffThetaTrajectoire = ASSER_TRAJ_DiffThetaBSplinePerLenghtUnit(&chemin.trajectoire \
                                                                                            , segmentCourantAv \
                                                                                            , subSegmentCourantAv \
                                                                                            , parametrePositionSegmentTrajectoireAv);

                            vitesse_fin_profil = ASSER_TRAJ_VitesseLimiteEnVirage(&chemin, diffThetaTrajectoire);

                            vitesse_fin_profil = MIN(vitesse_fin_profil_0, vitesse_fin_profil);
                        }

                        Phase = 0;

                        ASSER_TRAJ_LogAsserValPC("vitesse_debut_profil",  vitesse_debut_profil);
                        ASSER_TRAJ_LogAsserValPC("vitesse_fin_profil",  vitesse_fin_profil);

                    }
                } /* Fin d'init du nouveau profil de vitesse */

                /* Ajout de distance pour obtenir la distance parcourue par la roue (pas toujours la même) a l'exterieur des virages */
                distanceParcourue_Profil += delta_distance * (1.0 + NORME_BARRE_SUIVI_TRAJ * ASSER_TRAJ_Rinv_courbure(&chemin.trajectoire.subTrajs.segmentTraj[chemin.trajectoire.subTrajs.segmentCourant] \
                                                                                                                       , chemin.trajectoire.subTrajs.subSegmentCourant \
                                                                                                                        , chemin.trajectoire.subTrajs.paramPoseSubSegCourant));

                ASSER_TRAJ_LogAsserValPC("Vend_0", vitesse_fin_profil);
                ASSER_TRAJ_LogAsserValPC("Vstart_0", vitesse_debut_profil);
                
                ASSER_Running = ASSER_TRAJ_Profil_S_Curve(&VitesseProfil, distanceTotale_Profil, vitesse_debut_profil, POS_GetConsVitesseMax(), vitesse_fin_profil, AmaxTemp, chemin.profilVitesse.Dmax, Vitesse_Gain_ASR, distanceParcourue_Profil, (((float)m_sensDeplacement) * POS_GetVitesseRelle()), (SaturationPIDflag | SaturationPIGflag));

                if ((chemin.trajectoire.subTrajs.segmentCourant < (chemin.trajectoire.subTrajs.nbreSegments - 1)) \
                        | ((chemin.trajectoire.subTrajs.segmentCourant == (chemin.trajectoire.subTrajs.nbreSegments - 1)) & (g_iSubSeg < chemin.trajectoire.subTrajs.segmentTraj[(chemin.trajectoire.subTrajs.nbreSegments - 1)].subSeg_lastUsed)))
                {
                    ASSER_Running = True;
                }

                /* Passage de la vitesse exterieure a la vitesse centrale si besoin */
                VitesseProfil = VitesseProfil / (1.0 + NORME_BARRE_SUIVI_TRAJ * fabsf(ASSER_TRAJ_Rinv_courbure(&chemin.trajectoire.subTrajs.segmentTraj[chemin.trajectoire.subTrajs.segmentCourant] \
                                                                                                            , chemin.trajectoire.subTrajs.subSegmentCourant \
                                                                                                            , chemin.trajectoire.subTrajs.paramPoseSubSegCourant)));
                switch(chemin.trajectoire.subTrajs.subSegmentCourant)
                {
                    case SPLINE31 :
                    case SPLINE32 :
                        if (VitesseProfil > (chemin.profilVitesse.vmax / (1.0 + (NORME_BARRE_SUIVI_TRAJ * R_INV))))
                        {
                            VitesseProfil = (chemin.profilVitesse.vmax / (1.0 + (NORME_BARRE_SUIVI_TRAJ * R_INV)));
                        }
                        break;

                    case ARC1 :
                        if ( VitesseProfil > ((chemin.profilVitesse.vmax / VMAX_REDUCTION) / (1.0 + (NORME_BARRE_SUIVI_TRAJ * chemin.trajectoire.subTrajs.segmentTraj[chemin.trajectoire.subTrajs.segmentCourant].arc1.rayon_inverse))))
                        {
                            VitesseProfil = ((chemin.profilVitesse.vmax / VMAX_REDUCTION) / (1.0 + (NORME_BARRE_SUIVI_TRAJ * chemin.trajectoire.subTrajs.segmentTraj[chemin.trajectoire.subTrajs.segmentCourant].arc1.rayon_inverse)));
                        }
                        break;

                    case ARC2 :
                        if (VitesseProfil > ((chemin.profilVitesse.vmax / VMAX_REDUCTION) / (1.0 + (NORME_BARRE_SUIVI_TRAJ * chemin.trajectoire.subTrajs.segmentTraj[chemin.trajectoire.subTrajs.segmentCourant].arc2.rayon_inverse))))
                        {
                            VitesseProfil = ((chemin.profilVitesse.vmax / VMAX_REDUCTION) / (1.0 + (NORME_BARRE_SUIVI_TRAJ * chemin.trajectoire.subTrajs.segmentTraj[chemin.trajectoire.subTrajs.segmentCourant].arc2.rayon_inverse)));
                        }
                        break;

                    case SPLINE341 :
                        diffThetaTrajectoire = ASSER_TRAJ_DiffThetaBSplinePerLenghtUnit(&chemin.trajectoire \
                                                                                        , chemin.trajectoire.subTrajs.segmentCourant \
                                                                                        , chemin.trajectoire.subTrajs.subSegmentCourant \
                                                                                        , 0.0);
                        vitesse_fin_profil_0 = ASSER_TRAJ_VitesseLimiteEnVirage(&chemin, diffThetaTrajectoire);
                        if (VitesseProfil > vitesse_fin_profil_0)
                        {
                            VitesseProfil = vitesse_fin_profil_0;
                        }
                        break;

                    case SPLINE342 :
                        diffThetaTrajectoire = ASSER_TRAJ_DiffThetaBSplinePerLenghtUnit(&chemin.trajectoire \
                                                                                        , chemin.trajectoire.subTrajs.segmentCourant \
                                                                                        , chemin.trajectoire.subTrajs.subSegmentCourant \
                                                                                        , ti);
                        vitesse_fin_profil_0 = ASSER_TRAJ_VitesseLimiteEnVirage(&chemin, diffThetaTrajectoire);
                        if ( VitesseProfil > vitesse_fin_profil_0 )
                        {
                            VitesseProfil = vitesse_fin_profil_0;
                        }
                        break;

                   case LINE :
                        /* VitesseProfill not changed */                            
                        break;
                   default:                    
#ifdef PIC32_BUILD
                        TOOLS_LogFault(AsserPosErr, True, INTEGER, (int *)&chemin.trajectoire.subTrajs.subSegmentCourant, True, "subSegmentCourant non gere");
#else /* PIC32_BUILD */
                        ASSER_TRAJ_LogAsserMsgPC("Asser: subSegmentCourant non gere", (float)iSubSeg);
#endif /* PIC32_BUILD */
                        return;
                }

                ASSER_TRAJ_LogAsserValPC("VitesseProfil", VitesseProfil);


                ASSER_TRAJ_LogAsserValPC("Vend", vitesse_fin_profil);
                ASSER_TRAJ_LogAsserValPC("Vstart", vitesse_debut_profil);
                ASSER_TRAJ_LogAsserValPC("distanceTotale_Profil", distanceTotale_Profil);
                ASSER_TRAJ_LogAsserValPC("cfgAsser",  chemin.profilVitesse.Amax);
                ASSER_TRAJ_LogAsserValPC("cfgAsser",  chemin.profilVitesse.Dmax);

                delta_distance_Av = VitesseProfil * TE;

                parametrePositionSegmentTrajectoireAv = chemin.trajectoire.subTrajs.paramPoseSubSegCourant;
                segmentCourantAv = chemin.trajectoire.subTrajs.segmentCourant;
                subSegmentCourantAv = chemin.trajectoire.subTrajs.subSegmentCourant;
                
                ASSER_TRAJ_ParcoursTrajectoire(&chemin \
                                               , delta_distance_Av \
                                               , &segmentCourantAv \
                                               , &subSegmentCourantAv \
                                               , &parametrePositionSegmentTrajectoireAv \
                                               , NULL);

                poseReference = ASSER_TRAJ_TrajectoireRemorqueBS(&chemin.trajectoire \
                                                                 , segmentCourantAv \
                                                                 , subSegmentCourantAv \
                                                                 , parametrePositionSegmentTrajectoireAv \
                                                                 , &poseReferenceRobotAv);
            }
            
            differentielleTemporellePoseReference = ASSER_TRAJ_DiffTemporelleTrajectoire(poseReference, poseReferenceAv, TE);
            differentielleTemporellePoseReferenceRobot = ASSER_TRAJ_DiffTemporelleTrajectoire(poseReferenceRobot, poseReferenceRobotAv, TE);
            ASSER_TRAJ_LogAsserValPC("differentielleTemporellePoseReferenceRobot", differentielleTemporellePoseReferenceRobot.x);
        }
        else
        {
            vitesse_profil_consigne = 0.0;


            Phase = 0;
        }

        if (ASSER_TRAJ_isDeplacement(&chemin) == True)
        {
            erreurPoseCentreRobot = ASSER_TRAJ_ErreurPose(poseRobot, poseReferenceRobot);
            erreur_P.x = erreurPoseCentreRobot.x;
            erreur_P.y = erreurPoseCentreRobot.y;
            erreur_P.angle = erreurPoseCentreRobot.angle;

            gain[0] = gainDeplacement1;
            gain[1] = gainDeplacement2;
            gain[2] = gainDeplacement3;

            ASSER_TRAJ_LogAsserValPC("vitLongitudinaleTest", differentielleTemporellePoseReferenceRobot.x);

            *vitessesConsignes = ASSER_TRAJ_RetourDetatOrientation(erreurPoseCentreRobot, differentielleTemporellePoseReferenceRobot, gain);

            ASSER_TRAJ_LogAsserValPC("vitessesConsignes_longitudinale", vitessesConsignes->longitudinale);
        }
        else /* ROTATION */
        {
            if (fabsf(poseRobot.angle - chemin.trajectoire.rotation.poseDepartRobot.angle) > PI)
            {
                chemin.profilVitesse.distance_parcourue = fabsf(fabsf(poseRobot.angle - chemin.trajectoire.rotation.poseDepartRobot.angle) - (2.0 * PI)) * NORME_BARRE_SUIVI_TRAJ;
            }
            else
            {
                chemin.profilVitesse.distance_parcourue = fabsf(poseRobot.angle - chemin.trajectoire.rotation.poseDepartRobot.angle) * NORME_BARRE_SUIVI_TRAJ;
            }
            chemin.profilVitesse.distNormaliseeRestante = (chemin.distance - chemin.profilVitesse.distance_parcourue) / chemin.distance;
            ASSER_TRAJ_LogAsserValPC("dist_parcourue_rot",  chemin.profilVitesse.distance_parcourue);

            ASSER_Running = ASSER_TRAJ_Profil_S_Curve(&VitesseProfil, chemin.distance, 0.0, POS_GetConsVitesseMax(), 0.0, chemin.profilVitesse.AmaxRot, chemin.profilVitesse.DmaxRot, Vitesse_Gain_ASR, chemin.profilVitesse.distance_parcourue, (((float)m_sensDeplacement) * POS_GetVitesseRotation() * (ECART_ROUE_MOTRICE / 2.0)), (SaturationPIDflag | SaturationPIGflag));
            if (chemin.trajectoire.rotation.angle > 0.0)
            {
                vitessesConsignes->rotation = VitesseProfil / NORME_BARRE_SUIVI_TRAJ;
            }
            else
            {
                vitessesConsignes->rotation = (- 1.0) * VitesseProfil / NORME_BARRE_SUIVI_TRAJ;
            }

            poseReference.x = chemin.trajectoire.rotation.poseDepartRobot.x;
            poseReference.y = chemin.trajectoire.rotation.poseDepartRobot.y;
            poseReference.angle = poseRobot.angle;
            
            erreur_P = ASSER_TRAJ_ErreurPose(poseRobot, poseReference);
            vitessesConsignes->longitudinale = - gainCentreRot * erreur_P.x;
            ASSER_TRAJ_LogAsserValPC("gainCentreRot", gainCentreRot);
        } 
    }
    else
    {
        ASSER_TRAJ_LogAsserValPC("ASSER_Running_TvF", ASSER_compteurPeriode);
        vitessesConsignes->longitudinale = 0.0;
        vitessesConsignes->rotation = 0.0;
        
        vitesse_profil_consigne = 0.0;
        
        chemin.profilVitesse.p = 0;

        Phase = 0;
    }

    /* Log de valeurs */
#ifndef PIC32_BUILD
    if (ASSER_TRAJ_isDeplacement(&chemin) == True)
    {
        ASSER_TRAJ_Trajectoire(&chemin.trajectoire \
                               , chemin.trajectoire.subTrajs.segmentCourant \
                               , chemin.trajectoire.subTrajs.subSegmentCourant \
                               , chemin.trajectoire.subTrajs.paramPoseSubSegCourant \
                               , &poseTraj, &diff1Traj, &diff2Traj);
    }

    ASSER_TRAJ_LogAsserValPC("p", chemin.profilVitesse.p);
#endif /* PIC32_BUILD */

#ifdef PLOTS_SIMU
    ASSER_TRAJ_LogAsserValPC("vitLongMvt", vitessesConsignes->longitudinale);
    ASSER_TRAJ_LogAsserValPC("vitAngulaireRotation", vitessesConsignes->rotation);
    ASSER_TRAJ_LogAsserValPC("xCCourant",  poseTraj.x);
    ASSER_TRAJ_LogAsserValPC("xPoseReferenceRobot",  poseReferenceRobot.x);
    ASSER_TRAJ_LogAsserValPC("yPoseReferenceRobot",  poseReferenceRobot.y);
    ASSER_TRAJ_LogAsserValPC("erreurPose_x",  erreur_P.x);
    ASSER_TRAJ_LogAsserValPC("erreurPose_y",  erreur_P.y);
    ASSER_TRAJ_LogAsserValPC("erreurPose_angle",  erreur_P.angle);
    ASSER_TRAJ_LogAsserValPC("distNormaliseeRestante",  chemin.profilVitesse.distNormaliseeRestante);
    ASSER_TRAJ_LogAsserValPC("orientationPoseReferenceRobot",  poseReferenceRobot.angle);
    ASSER_TRAJ_LogAsserValPC("consRotation",  vitessesConsignes->rotation);
    ASSER_TRAJ_LogAsserValPC("parametrePositionSegmentTrajectoire",  chemin.trajectoire.subTrajs.paramPoseSubSegCourant);
    ASSER_TRAJ_LogAsserValPC("segmentCourant",  chemin.trajectoire.subTrajs.segmentCourant);
    ASSER_TRAJ_LogAsserValPC("VgASR", VgASR);
    ASSER_TRAJ_LogAsserValPC("Phase", (Phase / 10.0));
    ASSER_TRAJ_LogAsserValPC("fFin", (ASSER_Running / 10.0));
    ASSER_TRAJ_LogAsserValPC("distanceParcourue_Profil",  distanceParcourue_Profil);
#endif /* PLOTS_SIMU */
}


/**********************************************************************/
/*! \brief ASSER_TRAJ_InitialisationTrajectoire
 *
 *  \note   Initialisation d'une structure trajectoire definit en global dans le module ASSER_TRAJ
 *
 *  \param [in]     poseRobot       pose courante du robot integrant le choix de la marche (AVANT ou ARRIERE)
 *  \param [in]     Mouvement     Mouvement demande      
 *  \param [in]     Data               Donnees de la trajectoire      
 *
 *  \return           None
 */
/**********************************************************************/
extern void ASSER_TRAJ_InitialisationTrajectoire(Pose poseRobot, unsigned char Mouvement, Data_Goto * Data)
{
    Pose                    poseTraj;
    Vecteur                 diff1BS, diff2BS;
    unsigned int            iSubSeg, iSegment;
    float                   segCourant_x1, segCourant_y1, segCourant_theta1;
    float                   angle_fin_rotation, theta_seg, theta_seg_next;
    ConfigSegment           cfgSeg;
    unsigned char           curvature_forced_2_prec;
    unsigned char           curve2_prec                 = False;
    float                   angle_rad                   = 0.0;
    int                     traj_sds = TRAJ_OK, traj_sds_prev = TRAJ_OK;
#ifndef PIC32_BUILD
    Pose                    poseTest;
    Vecteur                 diffTest, diff2Test;
    float                   diffTheta, Rinv, dl_dt;
    unsigned int            intT;
#endif /* PIC32_BUILD */

    memo_angleRobot = m_poseRobot.angle;

    chemin.mouvement = Mouvement;

    shuntTestFinAsser = False;

    switch(Mouvement)
    {
        case ROTATE :
            
            /* Position a atteindre (condition d'arret du test de fin d'asservissement) */
            angle_fin_rotation = POS_ModuloAngle(Data->rotate.Angle);
            chemin.trajectoire.rotation.angle_final = angle_fin_rotation;
            ASSER_TRAJ_LogAsserValPC("angle_final_rot", chemin.trajectoire.rotation.angle_final / PI);

            chemin.posArrivee.x = poseRobot.x + (NORME_BARRE_SUIVI_TRAJ * cosf(angle_fin_rotation));
            chemin.posArrivee.y = poseRobot.y + (NORME_BARRE_SUIVI_TRAJ * sinf(angle_fin_rotation));

            ASSER_TRAJ_LogAsserValPC("angleFinRotation", angle_fin_rotation);

            /* Configuration du profil de vitesse */
            chemin.profilVitesse.vmax = POS_GetConsVitesseAngulaireMax() * NORME_BARRE_SUIVI_TRAJ;

            chemin.trajectoire.rotation.poseDepartRobot = poseRobot;
            chemin.trajectoire.rotation.angle = POS_ModuloAngle(angle_fin_rotation - poseRobot.angle);

            ASSER_TRAJ_LogAsserValPC("plageAngleRotation", chemin.trajectoire.rotation.angle / PI);

            /* Calcul de la premiere pose de la trajectoire de consigne */
            poseReference = ASSER_TRAJ_TrajectoireRotation(&(chemin.trajectoire.rotation), 0.0);

            chemin.distance = fabsf(chemin.trajectoire.rotation.angle) * NORME_BARRE_SUIVI_TRAJ;
            ASSER_TRAJ_LogAsserValPC("distance_seg", chemin.distance);

            break;

        case MOVE_CURVE :
            
            chemin.trajectoire.subTrajs.use_angle = Data->curve.Use_Angle;
            ASSER_TRAJ_LogAsserValPC("use_angle", chemin.trajectoire.subTrajs.use_angle);
            angle_rad = Data->curve.Angle;
            chemin.trajectoire.subTrajs.nbreSegments = Data->curve.NbrePtsChemin;
            ASSER_TRAJ_LogAsserValPC("NbSegCurve", chemin.trajectoire.subTrajs.nbreSegments);
            m_sensDeplacement = Data->curve.Marche;

            if (Data->curve.Use_Angle == ANGLE)
            {
                if (Data->curve.Marche == MARCHE_ARRIERE)
                {
                    angle_rad = POS_ModuloAngle(angle_rad + PI);
                }
                else
                {
                    angle_rad = POS_ModuloAngle(angle_rad);
                }
            }

            /* Position a atteindre (condition d'arret du test de fin d'asservissment) */
            chemin.posArrivee.x = CONVERT_DISTANCE(Data->curve.Chemin[(Data->curve.NbrePtsChemin - 1)].x);
            chemin.posArrivee.y = CONVERT_DISTANCE(Data->curve.Chemin[(Data->curve.NbrePtsChemin - 1)].y);

            /* Configuration du profil de vitesse */
            chemin.profilVitesse.vmax = POS_GetConsVitesseMax(); /* UMAX * GAIN_STATIQUE_MOTEUR */

            cfgSeg.curve1 = False;
            cfgSeg.curve2 = False;
            cfgSeg.curvature_forced = False;
            cfgSeg.curvature_forced_1 = False;
            cfgSeg.curvature_forced_2 = False;
            cfgSeg.Rb_prec = - 1.0;
            cfgSeg.qx0 = 0.0;
            cfgSeg.qy0 = 0.0;
            cfgSeg.inflexion_point = False;
            cfgSeg.spline4rot = True;
            cfgSeg.angle_step = ANGLE_STEP;
            cfgSeg.Rinv_ref = 1.0 / (ECART_ROUE_MOTRICE / 2.0);
            cfgSeg.v_Rinv_ref =  ASSER_TRAJ_VitesseLimiteEnVirage(&chemin, cfgSeg.Rinv_ref);

            segCourant_x1 = poseRobot.x;
            segCourant_y1 = poseRobot.y;
            segCourant_theta1 = poseRobot.angle;
            cfgSeg.x1 = segCourant_x1;
            cfgSeg.y1 = segCourant_y1;
            cfgSeg.theta1 = segCourant_theta1;

            curvature_forced_2_prec = False;
            ASSER_TRAJ_LogAsserValPC("angle_rad", angle_rad);

            for(iSegment = 0; iSegment < chemin.trajectoire.subTrajs.nbreSegments; iSegment++)
            {
                ASSER_TRAJ_LogAsserValPC("segConfigured", iSegment);

                if (iSegment >= (sizeof(chemin.trajectoire.subTrajs.segmentTraj) / sizeof(segmentTrajectoire)))
                {
#ifdef PIC32_BUILD
                    TOOLS_LogFault(AsserPosErr, True, INTEGER, (int *)&iSegment, True, "depassement nombre point");
#else /* PIC32_BUILD */
                    ASSER_TRAJ_LogAsserMsgPC("Asser: depassement du nombre de point possible", (float)iSegment);
#endif /* PIC32_BUILD */ 
                    return;
                }  

                do
                {
                    chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Flags.SPLINE31_USED = False;
                    chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Flags.ARC1_USED = False;
                    chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Flags.SPLINE341_USED = False;
                    chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Flags.LINE_USED = True;
                    chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Flags.SPLINE342_USED = False;
                    chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Flags.ARC2_USED = False;
                    chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Flags.SPLINE32_USED = False;

                    /* Init des arcs de cercle */
                    chemin.trajectoire.subTrajs.segmentTraj[iSegment].arc1.angle = 0.0;
                    chemin.trajectoire.subTrajs.segmentTraj[iSegment].arc2.angle = 0.0;

                    cfgSeg.x1 = segCourant_x1;
                    cfgSeg.y1 = segCourant_y1;
                    cfgSeg.theta1 = segCourant_theta1;

                    cfgSeg.x1_n = cfgSeg.x1;
                    cfgSeg.y1_n = cfgSeg.y1;
                    cfgSeg.theta1_n = cfgSeg.theta1;
                    cfgSeg.qx1 = 0.0;
                    cfgSeg.qy1 = 0.0;

                    cfgSeg.x2 = CONVERT_DISTANCE(Data->curve.Chemin[iSegment].x);
                    cfgSeg.y2 = CONVERT_DISTANCE(Data->curve.Chemin[iSegment].y);
                    cfgSeg.theta2 = ASSER_TRAJ_CalculTheta1(iSegment, Data->curve.NbrePtsChemin, Data->curve.Chemin, angle_rad, cfgSeg.x1, cfgSeg.y1);
                    cfgSeg.x2_n = cfgSeg.x2;
                    cfgSeg.y2_n = cfgSeg.y2;
                    cfgSeg.theta2_n = cfgSeg.theta2;
                    cfgSeg.qx2 = 0.0;
                    cfgSeg.qy2 = 0.0;

                    theta_seg = atan2f((cfgSeg.y2 - cfgSeg.y1), (cfgSeg.x2 - cfgSeg.x1));

                    /* Workarround library math */
                    if ((cfgSeg.y2 - cfgSeg.y1) < 0.0)
                    {
                        if (theta_seg > 0.0)
                        {
                            theta_seg = - theta_seg;
                        }
                    }
                    else
                    {
                        if (theta_seg < 0.0)
                        {
                            theta_seg = - theta_seg;
                        }
                    }

                    chemin.trajectoire.subTrajs.segmentTraj[iSegment].theta_seg = theta_seg;

                    if (iSegment < (chemin.trajectoire.subTrajs.nbreSegments - 1))
                    {
                        theta_seg_next = atan2f((CONVERT_DISTANCE(Data->curve.Chemin[(iSegment + 1)].y) - cfgSeg.y2), (CONVERT_DISTANCE(Data->curve.Chemin[(iSegment + 1)].x) - cfgSeg.x2));

                        /* Workarround library math */
                        if ((CONVERT_DISTANCE(Data->curve.Chemin[(iSegment + 1)].y) - cfgSeg.y2) < 0.0)
                        {
                            if (theta_seg_next > 0.0)
                            {
                                theta_seg_next = - theta_seg_next;
                            }
                        }
                        else
                        {
                            if (theta_seg_next < 0.0)
                            {
                                theta_seg_next = - theta_seg_next;
                            }
                        }

                        if (((cfgSeg.theta2 - theta_seg) * (theta_seg_next - cfgSeg.theta2)) < 0.0)
                        {
                            cfgSeg.inflexion_point = True;
                        }
                    }

                    if (chemin.trajectoire.subTrajs.nbreSegments == 1)
                    {
                        cfgSeg.curve1 = False;
                        cfgSeg.curve2 = False;
                        cfgSeg.sp1_type = SPLINE4;
                        cfgSeg.sp2_type = SPLINE4;
                    }
                    else /* (chemin.trajectoire.subTrajs.nbreSegments > 1) */
                    {
                        if (iSegment == 0)
                        {
                            cfgSeg.curve1 = False;
                            cfgSeg.curve2 = True;
                            cfgSeg.sp1_type = SPLINE4;
                            cfgSeg.sp2_type = SPLINE3;
                        }
                        else
                        {
                            /* Initialiser la courbure induite par la fin du segment precedent */
                            cfgSeg.qx1 = cfgSeg.qx0;
                            cfgSeg.qy1 = cfgSeg.qy0;

                            ASSER_TRAJ_LogAsserValPC("curvature_forced_2_prec", ((float)curvature_forced_2_prec));

                            if (curvature_forced_2_prec == True)
                            {
                                cfgSeg.Rb_prec = - 1.0;
                                cfgSeg.spline4rot = False;
                            }
                            else
                            {
                                cfgSeg.spline4rot = True;
                            }

                            /* Dernier segment */
                            if (iSegment == (chemin.trajectoire.subTrajs.nbreSegments - 1))
                            {
                                cfgSeg.curve1 = True;
                                cfgSeg.curve2 = False;
                                cfgSeg.sp1_type = SPLINE4_n;
                                cfgSeg.sp2_type = SPLINE4;
                            }
                            else
                            {
                                cfgSeg.curve1 = True;
                                cfgSeg.curve2 = True;
                                cfgSeg.sp1_type = SPLINE4_n;
                                cfgSeg.sp2_type = SPLINE3;
                            }

                            if (curve2_prec == False)
                            {
                                cfgSeg.curve1 = False;
                            }
                        }

                        ASSER_TRAJ_LogAsserValPC("inflexion_point", ((float)cfgSeg.inflexion_point));
                        if (cfgSeg.inflexion_point == True)
                        {
                            cfgSeg.curve2 = False;
                            cfgSeg.sp2_type = SPLINE4;
                        }
                    }

                    traj_sds = ASSER_TRAJ_sds_ab(&cfgSeg, &chemin.trajectoire.subTrajs.segmentTraj[iSegment]);
                    ASSER_TRAJ_LogAsserValPC("traj_sds_0", traj_sds);

                    if (traj_sds_prev == TRAJ_OK)
                    {
                        ASSER_TRAJ_Test_courbure(&chemin, &cfgSeg, &chemin.trajectoire.subTrajs.segmentTraj[iSegment]);
                        if (cfgSeg.curvature_forced == True)
                        {
                            traj_sds = ASSER_TRAJ_Generation_curvatureForced(&cfgSeg, &chemin.trajectoire.subTrajs.segmentTraj[iSegment]);
                        }
                    }
                    traj_sds_prev = traj_sds;
                }while (traj_sds == TRAJ_ARCNOK);

                segCourant_x1 = cfgSeg.x2;
                segCourant_y1 = cfgSeg.y2;
                segCourant_theta1 = cfgSeg.theta2;
                cfgSeg.x1 = segCourant_x1;
                cfgSeg.y1 = segCourant_y1;
                cfgSeg.theta1 = segCourant_theta1;

                curvature_forced_2_prec = cfgSeg.curvature_forced_2;

                chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_lastUsed = 0;
                chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_firstUsed = SPLINE32 + 1;

                for (iSubSeg = SPLINE31; iSubSeg < (SPLINE32 + 1); iSubSeg++)
                {       
                    if (TEST_BIT(chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Info, iSubSeg) == True)
                    {
                        if (chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_firstUsed > SPLINE32)
                        {
                            chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_firstUsed = iSubSeg;
                        }
                        chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_lastUsed = iSubSeg;
                    }
                }

                ASSER_TRAJ_LogAsserValPC("flag_subSeg_used", TEST_BIT(chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Info, 0));
                ASSER_TRAJ_LogAsserValPC("flag_subSeg_used", TEST_BIT(chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Info, 1));
                ASSER_TRAJ_LogAsserValPC("flag_subSeg_used", TEST_BIT(chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Info, 2));
                ASSER_TRAJ_LogAsserValPC("flag_subSeg_used", TEST_BIT(chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Info, 3));
                ASSER_TRAJ_LogAsserValPC("flag_subSeg_used", TEST_BIT(chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Info, 4));
                ASSER_TRAJ_LogAsserValPC("flag_subSeg_used", TEST_BIT(chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Info, 5));
                ASSER_TRAJ_LogAsserValPC("flag_subSeg_used", TEST_BIT(chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Info, 6));

                /**************************************/
            }

            chemin.trajectoire.subTrajs.segmentCourant = 0;
            chemin.trajectoire.subTrajs.subSegmentCourant = chemin.trajectoire.subTrajs.segmentTraj[0].subSeg_firstUsed;
            chemin.trajectoire.subTrajs.paramPoseSubSegCourant = 0.0;

            chemin.profilVitesse.vmax = chemin.profilVitesse.vmax * VMAX_REDUCTION;

            /* Determination des distances des segments de trajectoire, et de la distance totale */

            chemin.distance = 0.0;
            for (iSegment = 0; iSegment < chemin.trajectoire.subTrajs.nbreSegments; iSegment++)
            {
                ASSER_TRAJ_DistanceTrajectoire(&chemin.trajectoire, iSegment);

                chemin.distance += chemin.trajectoire.subTrajs.segmentTraj[iSegment].distance;

                ASSER_TRAJ_LogAsserValPC("distance_seg", chemin.trajectoire.subTrajs.segmentTraj[iSegment].distance);
            }

            
            break;

        case MOVE_LINE :
            chemin.trajectoire.subTrajs.nbreSegments = Data->line.NbrePtsChemin;
            m_sensDeplacement = Data->line.Marche;

            /* Position a atteindre (condition d'arret du test de fin d'asservissment) */
            chemin.posArrivee.x = CONVERT_DISTANCE(Data->line.Chemin[(Data->line.NbrePtsChemin - 1)].x);
            chemin.posArrivee.y = CONVERT_DISTANCE(Data->line.Chemin[(Data->line.NbrePtsChemin - 1)].y);

            /* Configuration du profil de vitesse */
            chemin.profilVitesse.vmax = POS_GetConsVitesseMax();                /* UMAX * GAIN_STATIQUE_MOTEUR */

            chemin.distance = 0.0;
            for(iSegment = 0; iSegment < chemin.trajectoire.subTrajs.nbreSegments; iSegment++)
            {

                chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Flags.SPLINE31_USED = False;
                chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Flags.ARC1_USED = False;
                chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Flags.SPLINE341_USED = False;
                chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Flags.LINE_USED = True;
                chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Flags.SPLINE342_USED = False;
                chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Flags.ARC2_USED = False;
                chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Flags.SPLINE32_USED = False;
                chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_firstUsed = LINE;
                chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_lastUsed = LINE;

                if (iSegment == 0)
                {
                    chemin.trajectoire.subTrajs.segmentTraj[iSegment].line.ax = (CONVERT_DISTANCE(Data->line.Chemin[0].x) - poseRobot.x) / ti;
                    chemin.trajectoire.subTrajs.segmentTraj[iSegment].line.ay = (CONVERT_DISTANCE(Data->line.Chemin[0].y) - poseRobot.y) / ti;
                    chemin.trajectoire.subTrajs.segmentTraj[iSegment].line.bx = poseRobot.x;
                    chemin.trajectoire.subTrajs.segmentTraj[iSegment].line.by = poseRobot.y;
                }
                else
                {
                    chemin.trajectoire.subTrajs.segmentTraj[iSegment].line.ax = (CONVERT_DISTANCE(Data->line.Chemin[iSegment].x) - CONVERT_DISTANCE(Data->line.Chemin[(iSegment - 1)].x)) / ti;
                    chemin.trajectoire.subTrajs.segmentTraj[iSegment].line.ay = (CONVERT_DISTANCE(Data->line.Chemin[iSegment].y) - CONVERT_DISTANCE(Data->line.Chemin[(iSegment - 1)].y)) / ti;
                    chemin.trajectoire.subTrajs.segmentTraj[iSegment].line.bx = CONVERT_DISTANCE(Data->line.Chemin[(iSegment - 1)].x);
                    chemin.trajectoire.subTrajs.segmentTraj[iSegment].line.by = CONVERT_DISTANCE(Data->line.Chemin[(iSegment - 1)].y);
                }

                ASSER_TRAJ_DistanceTrajectoire(&chemin.trajectoire, iSegment);
                chemin.distance += chemin.trajectoire.subTrajs.segmentTraj[iSegment].distance;
                ASSER_TRAJ_LogAsserValPC("distance_seg", chemin.trajectoire.subTrajs.segmentTraj[iSegment].distance);
            }

            chemin.trajectoire.subTrajs.segmentCourant = 0;
            chemin.trajectoire.subTrajs.subSegmentCourant = LINE;
            chemin.trajectoire.subTrajs.paramPoseSubSegCourant = 0.0;

            ASSER_TRAJ_LogAsserValPC("distance", chemin.distance);
            ASSER_TRAJ_LogAsserValPC("paramPoseSubSegCourant", chemin.trajectoire.subTrajs.paramPoseSubSegCourant);

            break;

        case MOVE_ARC :
            
            chemin.trajectoire.subTrajs.nbreSegments = Data->arc.NbrePtsChemin;
            m_sensDeplacement = Data->arc.Marche;

            /* Position a atteindre (condition d'arret du test de fin d'asservissment) */
            chemin.posArrivee.x = CONVERT_DISTANCE(Data->arc.Centre_rotation.x) + ( Data->arc.Rayon * cosf(Data->arc.Chemin[(Data->arc.NbrePtsChemin - 1)]) );
            chemin.posArrivee.y = CONVERT_DISTANCE(Data->arc.Centre_rotation.y) + ( Data->arc.Rayon * sinf(Data->arc.Chemin[(Data->arc.NbrePtsChemin - 1)]) );

            /* Configuration du profil de vitesse */
            chemin.profilVitesse.vmax = POS_GetConsVitesseMax();                /* UMAX * GAIN_STATIQUE_MOTEUR */

            chemin.distance = 0.0;
            
            for(iSegment = 0; iSegment < chemin.trajectoire.subTrajs.nbreSegments; iSegment++)
            {
                chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Flags.SPLINE31_USED = False;
                chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Flags.ARC1_USED = True;
                chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Flags.SPLINE341_USED = False;
                chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Flags.LINE_USED = False;
                chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Flags.SPLINE342_USED = False;
                chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Flags.ARC2_USED = False;
                chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Flags.SPLINE32_USED = False;
                chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_firstUsed = ARC1;
                chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_lastUsed = ARC1;

                chemin.trajectoire.subTrajs.segmentTraj[iSegment].arc1.xc = CONVERT_DISTANCE(Data->arc.Centre_rotation.x);
                chemin.trajectoire.subTrajs.segmentTraj[iSegment].arc1.yc = CONVERT_DISTANCE(Data->arc.Centre_rotation.y);
                chemin.trajectoire.subTrajs.segmentTraj[iSegment].arc1.rayon_inverse = (1.0 / Data->arc.Rayon);

                if (iSegment == 0)
                {
                    chemin.trajectoire.subTrajs.segmentTraj[iSegment].arc1.theta0 = atan2f((poseRobot.y - chemin.trajectoire.subTrajs.segmentTraj[iSegment].arc1.yc) \
                                                                                           , (poseRobot.x - chemin.trajectoire.subTrajs.segmentTraj[iSegment].arc1.xc));
                    chemin.trajectoire.subTrajs.segmentTraj[iSegment].arc1.angle = POS_ModuloAngle(Data->arc.Chemin[iSegment] - chemin.trajectoire.subTrajs.segmentTraj[iSegment].arc1.theta0);
                }
                else
                {
                    chemin.trajectoire.subTrajs.segmentTraj[iSegment].arc1.theta0 = Data->arc.Chemin[(iSegment - 1)];
                    chemin.trajectoire.subTrajs.segmentTraj[iSegment].arc1.angle = POS_ModuloAngle(Data->arc.Chemin[iSegment] - Data->arc.Chemin[(iSegment - 1)]);
                }
                ASSER_TRAJ_LogAsserValPC("theta0_arc", chemin.trajectoire.subTrajs.segmentTraj[iSegment].arc1.theta0);
                ASSER_TRAJ_LogAsserValPC("angle_arc", chemin.trajectoire.subTrajs.segmentTraj[iSegment].arc1.angle);

                ASSER_TRAJ_DistanceTrajectoire(&chemin.trajectoire, iSegment);
                chemin.distance += chemin.trajectoire.subTrajs.segmentTraj[iSegment].distance;
                ASSER_TRAJ_LogAsserValPC("distance_seg", chemin.trajectoire.subTrajs.segmentTraj[iSegment].distance);
            }

            chemin.trajectoire.subTrajs.segmentCourant = 0;
            chemin.trajectoire.subTrajs.subSegmentCourant = ARC1;
            chemin.trajectoire.subTrajs.paramPoseSubSegCourant = 0.0;
            
            break;

        default :

#ifdef PIC32_BUILD
            TOOLS_LogFault(AsserPosErr, True, INTEGER, (int *)&Mouvement, True, "Mouvement non gere");
#else /* PIC32_BUILD */
            ASSER_TRAJ_LogAsserMsgPC("Asser: Mouvement non gere", (float)Mouvement);
#endif /* PIC32_BUILD */  

            return;
    }

    if ((Mouvement == MOVE_CURVE) || (Mouvement == MOVE_LINE) || (Mouvement == MOVE_ARC))
    {
#ifndef PIC32_BUILD
        for(iSegment = 0; iSegment < chemin.trajectoire.subTrajs.nbreSegments; iSegment++)
        {
            for (iSubSeg = SPLINE31; iSubSeg <= SPLINE32; iSubSeg++)
            {
                for (intT = 0; intT < 31; intT++)
                {
                    if (TEST_BIT(chemin.trajectoire.subTrajs.segmentTraj[iSegment].subSeg_used.Info, iSubSeg) == True)
                    {
                        ASSER_TRAJ_Trajectoire(&chemin.trajectoire \
                                               , iSegment \
                                               , iSubSeg \
                                               , (((float)intT) / 30.0) * ti \
                                               , &poseTest, &diffTest, &diff2Test);

                        dl_dt = sqrtf(SQUARE(diffTest.x) + SQUARE(diffTest.y));

                        diffTheta = ASSER_TRAJ_DiffThetaBSpline(diffTest, diff2Test);


                        Rinv = ASSER_TRAJ_Rinv_courbure(&chemin.trajectoire.subTrajs.segmentTraj[iSegment] \
                                                        , iSubSeg \
                                                        , (((float)intT) / 30.0) * ti);

                        ASSER_TRAJ_LogAsserValPC("def_Rinv", Rinv);
                        ASSER_TRAJ_LogAsserValPC("def_vLim", ASSER_TRAJ_VitesseLimiteEnVirage(&chemin, fabsf(Rinv)));
                        ASSER_TRAJ_LogAsserValPC("def_dl_dt", dl_dt);

                        ASSER_TRAJ_LogAsserValPC("def_xTraj", poseTest.x);
                        ASSER_TRAJ_LogAsserValPC("def_yTraj", poseTest.y);
                        ASSER_TRAJ_LogAsserValPC("def_angleTraj", poseTest.angle);

                        switch(iSubSeg)
                        {
                            case SPLINE31 :
                                ASSER_TRAJ_LogAsserValPC("def_xTraj_SP31", poseTest.x);
                                ASSER_TRAJ_LogAsserValPC("def_yTraj_SP31", poseTest.y);
                                break;

                            case ARC1 :
                                ASSER_TRAJ_LogAsserValPC("def_xTraj_ARC1", poseTest.x);
                                ASSER_TRAJ_LogAsserValPC("def_yTraj_ARC1", poseTest.y);
                                break;

                            case SPLINE341 :
                                ASSER_TRAJ_LogAsserValPC("def_xTraj_SP341", poseTest.x);
                                ASSER_TRAJ_LogAsserValPC("def_yTraj_SP341", poseTest.y);
                                break;

                            case LINE :
                                ASSER_TRAJ_LogAsserValPC("def_xTraj_LINE", poseTest.x);
                                ASSER_TRAJ_LogAsserValPC("def_yTraj_LINE", poseTest.y);
                                break;

                            case SPLINE342:
                                ASSER_TRAJ_LogAsserValPC("def_xTraj_SP342", poseTest.x);
                                ASSER_TRAJ_LogAsserValPC("def_yTraj_SP342", poseTest.y);
                                break;

                            case ARC2 :
                                ASSER_TRAJ_LogAsserValPC("def_xTraj_ARC2", poseTest.x);
                                ASSER_TRAJ_LogAsserValPC("def_yTraj_ARC2", poseTest.y);
                                break;

                            case SPLINE32 :
                                ASSER_TRAJ_LogAsserValPC("def_xTraj_SP32", poseTest.x);
                                ASSER_TRAJ_LogAsserValPC("def_yTraj_SP32", poseTest.y);
                                break;

                            default:
                                break;
                        }


                        ASSER_TRAJ_LogAsserValPC("def_diff_xTraj", diffTest.x);
                        ASSER_TRAJ_LogAsserValPC("def_diff_yTraj", diffTest.y);

                        ASSER_TRAJ_LogAsserValPC("def_diff2_xTraj", diff2Test.x);
                        ASSER_TRAJ_LogAsserValPC("def_diff2_yTraj", diff2Test.y);

                        ASSER_TRAJ_LogAsserValPC("def_diff_ThetaTraj", diffTheta);

                    }
                }
            }
        }
#endif /*PIC32_BUILD*/

        /* Calcul de la premiere pose de la trajectoire de consigne */
        ASSER_TRAJ_Trajectoire(&chemin.trajectoire \
                               , 0 \
                               , chemin.trajectoire.subTrajs.segmentTraj[0].subSeg_firstUsed \
                               , 0.0 \
                               , &poseTraj, &diff1BS, &diff2BS);

        chemin.profilVitesse.diffThetaCourant = ASSER_TRAJ_DiffThetaBSpline(diff1BS, diff2BS);

        poseReference = ASSER_TRAJ_TrajectoireRemorqueBS(&chemin.trajectoire, 0, chemin.trajectoire.subTrajs.segmentTraj[0].subSeg_firstUsed, 0.0, &poseReferenceRobot);
        ASSER_TRAJ_LogAsserValPC("poseReferenceRobot", poseReferenceRobot.x);
        ASSER_TRAJ_LogAsserValPC("poseReferenceRobot", poseReferenceRobot.y);
        ASSER_TRAJ_LogAsserValPC("poseReferenceRobot", poseReferenceRobot.angle);
        ASSER_TRAJ_LogAsserValPC("xNewSubSeg", poseRobot.x);
        ASSER_TRAJ_LogAsserValPC("yNewSubSeg", poseRobot.y);

        g_memoSegmentCourant = (NBRE_MAX_PTS_TRAJ  + 1);
        g_memoSubSegmentCourant = (SPLINE32  + 1);
        g_iSeg = 0;
        g_iSubSeg = chemin.trajectoire.subTrajs.segmentTraj[g_iSeg].subSeg_firstUsed;
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

    chemin.profilVitesse.p = 1;                        /* autorisation du profil de vitesse */
    chemin.profilVitesse.distNormaliseeRestante = 1.0; /* -> la distance totale normalisee */
 
    /* Initialisation du profil de vitesse */
    chemin.profilVitesse.Amax = A_MAX * Ratio_Acc;
    chemin.profilVitesse.Dmax = D_MAX * Ratio_Decc;    
    chemin.profilVitesse.AmaxRot = A_MAX * Ratio_Acc_Rot;
    chemin.profilVitesse.DmaxRot = D_MAX * Ratio_Decc_Rot;   
    chemin.profilVitesse.etat = 1;

    ASSER_compteurPeriode = 0;

    Phase = 0;

    /* Initialisation de l'erreur de distance avant l'arrivee */
    errDist = POS_ErreurDistance(poseRobot, chemin.posArrivee);
    errAngle = POS_ErreurOrientation(poseRobot, chemin.posArrivee);
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_sds_ab
 *
 *  \note
 *
 *  \param [in]     cfgSeg  Pointeur vers une structure de configuration de segment
 *  \param [in]     segmentTraj  Pointeur vers une structure de segment
 *
 *  \return
 */
/**********************************************************************/
static int ASSER_TRAJ_sds_ab(ConfigSegment * cfgSeg, segmentTrajectoire * segmentTraj)
{
    int     trajok = TRAJ_OK;
    float   x1, y1, theta1, x2, y2, theta2;
    float   dl_dt, d2l_dt2;
    unsigned char n1, n2;
    float   qx1, qy1, qx2, qy2, bx1;
    float   theta_seg, theta1_base, theta2_base;
    Pose    poseTraj    = {0.0, 0.0, 0.0};
    float   x2_base     = 0.0;
    float   y2_base     = 0.0;
    float   qx1_base    = 0.0;
    float   qy1_base    = 0.0;
    float   qx2_base    = 0.0;
    float   qy2_base    = 0.0;
    Vecteur diff1Traj   = {0.0, 0.0};
    Vecteur diff2Traj   = {0.0, 0.0};

    switch (cfgSeg->sp1_type)
    {
        case SPLINE3 :
            n1 = 0;
            x1 = cfgSeg->x1;
            y1 = cfgSeg->y1;
            theta1 = cfgSeg->theta1;
            qx1 = 0.0;
            qy1 = 0.0;
            break;

        case SPLINE4 :
            n1 = 1;
            x1 = cfgSeg->x1;
            y1 = cfgSeg->y1;
            theta1 = cfgSeg->theta1;
            qx1 = 0.0;
            qy1 = 0.0;
            break;

        case SPLINE4_n :
            n1 = 1;
            x1 = cfgSeg->x1_n;
            y1 = cfgSeg->y1_n;
            theta1 = cfgSeg->theta1_n;
            qx1 = cfgSeg->qx1;
            qy1 = cfgSeg->qy1;
            break;

        default :
#ifdef PIC32_BUILD
            TOOLS_LogFault(AsserPosErr, True, INTEGER, (int *)&cfgSeg->sp1_type, True, "spline non gere");
#else /* PIC32_BUILD */
            ASSER_TRAJ_LogAsserMsgPC("spline non gere", (float)cfgSeg->sp1_type);
#endif /* PIC32_BUILD */  
            return;
    }

    switch (cfgSeg->sp2_type)
    {
        case SPLINE3 :
            n2 = 0;
            x2 = cfgSeg->x2;
            y2 = cfgSeg->y2;
            theta2 = cfgSeg->theta2;
            qx2 = 0.0;
            qy2 = 0.0;
            break;

        case SPLINE4 :
            n2 = 1;
            x2 = cfgSeg->x2;
            y2 = cfgSeg->y2;
            theta2 = cfgSeg->theta2;
            qx2 = 0.0;
            qy2 = 0.0;
            break;

        case SPLINE4_n :
            n2 = 1;
            x2 = cfgSeg->x2_n;
            y2 = cfgSeg->y2_n;
            theta2 = cfgSeg->theta2_n;
            qx2 = cfgSeg->qx2;
            qy2 = cfgSeg->qy2;
            break;

        default :
#ifdef PIC32_BUILD
            TOOLS_LogFault(AsserPosErr, True, INTEGER, (int *)&cfgSeg->sp2_type, True, "spline non gere");
#else /* PIC32_BUILD */
            ASSER_TRAJ_LogAsserMsgPC("Asser: spline non gere", (float)cfgSeg->sp2_type);
#endif /* PIC32_BUILD */  
            return;
    }

    theta_seg = atan2f((y2 - y1), (x2 - x1));
    
    /* Workarround library math */
    if ((y2 - y1) < 0.0)
    {
        if (theta_seg > 0.0)
        {
            theta_seg = - theta_seg;
        }
    }
    else
    {
        if (theta_seg < 0.0)
        {
            theta_seg = - theta_seg;
        }
    }

    segmentTraj->spline41.cx = x1;
    segmentTraj->spline41.cy = y1;
    segmentTraj->spline41.theta_seg = theta_seg;
    segmentTraj->spline42.cx = x2;
    segmentTraj->spline42.cy = y2;
    segmentTraj->spline42.theta_seg = theta_seg;

    ASSER_TRAJ_LogAsserValPC("data_sds_ab", qx1);
    ASSER_TRAJ_LogAsserValPC("data_sds_ab", qy1);
    ASSER_TRAJ_LogAsserValPC("data_sds_ab", qx2);
    ASSER_TRAJ_LogAsserValPC("data_sds_ab", qy2);

    theta1_base = POS_ModuloAngle(theta1 - theta_seg);
    theta2_base = POS_ModuloAngle(POS_ModuloAngle(theta2 - PI) - theta_seg);
    ASSER_TRAJ_Rotation_coord(- theta_seg, (x2 - x1), (y2 - y1), &x2_base, &y2_base);
    x2_base = x1 + x2_base;
    y2_base = y1 + y2_base;
    ASSER_TRAJ_Rotation_coord(- theta_seg, qx1, qy1, &qx1_base, &qy1_base);
    ASSER_TRAJ_Rotation_coord(- theta_seg, qx2, qy2, &qx2_base, &qy2_base);

    bx1 = cfgSeg->Rb_prec * cosf(theta1_base);

    trajok = ASSER_TRAJ_sds_ab_base(x1, y1, theta1_base, x2_base, theta2_base, ti, n1, n2, qx1_base, qy1_base, qx2_base, qy2_base, bx1, &segmentTraj->spline41, &segmentTraj->spline42);

    if (trajok == TRAJ_OK)
    {
        segmentTraj->subSeg_used.Flags.SPLINE341_USED = True;
        segmentTraj->subSeg_used.Flags.SPLINE342_USED = True;

        cfgSeg->theta_seg = theta_seg;

        cfgSeg->Rb_prec = segmentTraj->spline42.bx / cosf(theta2_base); /* To be used for the next segment */

        if ((theta1 - theta_seg) > 0.0)
        {
            cfgSeg->s_Rinv_1 = - 1.0;
        }
        else
        {
            cfgSeg->s_Rinv_1 = 1.0;
        }

        if ((theta2 - theta_seg) > 0.0)
        {
            cfgSeg->s_Rinv_2 = - 1.0;
        }
        else
        {
            cfgSeg->s_Rinv_2 = 1.0;
        }

        ASSER_TRAJ_Trajectoire_SubSegment(segmentTraj, SPLINE342, ti, &poseTraj, &diff1Traj, &diff2Traj);
        dl_dt = sqrtf(SQUARE(diff1Traj.x) + SQUARE(diff1Traj.y));
        d2l_dt2 = sqrtf(SQUARE(diff2Traj.x) + SQUARE(diff2Traj.y));
        cfgSeg->qx0 = ((diff2Traj.x * dl_dt) - (diff1Traj.x * d2l_dt2)) / SQUARE(dl_dt);
        cfgSeg->qy0 = ((diff2Traj.y * dl_dt) - (diff1Traj.y * d2l_dt2)) / SQUARE(dl_dt);

        /* Configuration de la ligne droite joignant les deux splines */
        ASSER_TRAJ_Trajectoire_SubSegment(segmentTraj, SPLINE341, ti, &poseTraj, &diff1Traj, &diff2Traj);
        segmentTraj->line.bx = poseTraj.x;
        segmentTraj->line.by = poseTraj.y;
        ASSER_TRAJ_Trajectoire_SubSegment(segmentTraj, SPLINE342, 0.0, &poseTraj, &diff1Traj, &diff2Traj);
        segmentTraj->line.ax = (poseTraj.x - segmentTraj->line.bx) / ti;
        segmentTraj->line.ay = (poseTraj.y - segmentTraj->line.by) / ti;
    }

    return trajok;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_Rotation_coord
 *
 *  \note
 *
 *  \param [in]     theta   Angle de rotation du repere
 *  \param [in]     x       Abscisse initiale
 *  \param [in]     y       Ordonnee initiale
 *  \param [in]     x_n     Pointeur vers l'abscisse finale apres changement de repere
 *  \param [in]     y_n     Pointeur vers l'ordonnee finale apres changement de repere
 *
 *  \return
 */
/**********************************************************************/
static void ASSER_TRAJ_Rotation_coord(float theta, float x, float y, float * x_n, float * y_n)
{
    *x_n = (x * cosf(theta)) - (y * sinf(theta));
    *y_n = (x * sinf(theta)) + (y * cosf(theta));
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_sds_ab_base
 *
 *  \note
 *
 *  \param [in]     x1      Abscisse de la pose initiale du segment (pose initiale de la spline 1)
 *  \param [in]     y1      Ordonnee de la pose initiale du segment (pose initiale de la spline 1)
 *  \param [in]     theta1  Angle de la pose initiale du segment (pose initiale de la spline 1)
 *  \param [in]     x2      Abscisse de la pose finale du segment (pose initiale de la spline 2)
 *  \param [in]     theta2  Angle de la pose finale du segment (pose initiale de la spline 2)
 *  \param [in]     t1      Longueur parametrique de la spline
 *  \param [in]     n1      Ordre de la spline 1
 *  \param [in]     n2      Ordre de la spline 2
 *  \param [in]     qx1_0   Derivee seconde de l'abscisse initiale de la spline 1
 *  \param [in]     qy1_0   Derivee seconde de l'ordonnee initiale de la spline 1
 *  \param [in]     qx2_0   Derivee seconde de l'abscisse initiale de la spline 2
 *  \param [in]     qy2_0   Derivee seconde de l'ordonnee initiale de la spline 2
 *  \param [in]     bx1     Parametre b de la spline 1
 *  \param [in]     cfgSp1  Pointeur vers une structure des parametres de la spline 1
 *  \param [in]     cfgSp2  Pointeur vers une structure des parametres de la spline 2
 *
 *  \return                 Etat de validite du parametrage de trajectoire [1 : ok; -1 : non valide]
 */
/**********************************************************************/
static int ASSER_TRAJ_sds_ab_base(float x1, float y1, float theta1, float x2, float theta2, float t1, unsigned char n1, unsigned char n2, float qx1_0, float qy1_0, float qx2_0, float qy2_0, float bx1, ConfigSpline34 * cfgSp1, ConfigSpline34 * cfgSp2)
{
    float   y2, D, C, Cmax, Cmin, CoefC;
    float   qx1, qy1, qx2, qy2;
    float   S1, S2;
    static float   Tx1, Tx2, Ty1, Ty2, T2x1, T2x2, T2y1, T2y2;
    static float   A31, A32, A41, A42;
    static float   A01, A02, A11, A12, A13, A14, A15, A16, A17, A18, A19;
    static float   A20, A21, A22, A23, A24, A25, A26, A27, A28, A29;
    static float   A30, A50, A51, A52, A53, A54, A55, A56, A57, A58, A59, A60;
    static float   delta_k, k, k1, k2;
    static int     trajok = TRAJ_OK;

    /* TODO => trop de float alloué dans la stack => refactoring */

    y2 = y1;
    D = fabsf(x2 - x1);

    Cmax = 0.06;
    Cmin = 0.03;

    if ((0.05 * D) > Cmax)
    {
        C = Cmax;
    }
    else
    {
        C = (0.05 * D);
        if (C < Cmin)
        {
            C = Cmin;
        }
    }

    CoefC = (C / t1) * 1.0;
    qx1 = qx1_0 * CoefC;
    qy1 = qy1_0 * CoefC;
    qx2 = qx2_0 * CoefC;
    qy2 = qy2_0 * CoefC;

    S1 = 1.0;
    if (theta1 < 0.0)
    {
        S1 = - S1;
    }

    S2 = 1.0;
    if (theta2 < 0.0)
    {
        S2 = - S2;
    }

    Tx1 = (t1 / 2.0) * qx1;
    Tx2 = (t1 / 2.0) * qx2;
    Ty1 = (t1 / 2.0) * qy1;
    Ty2 = (t1 / 2.0) * qy2;
    T2x1 = (SQUARE_t1 / 3.0) * qx1;
    T2x2 = (SQUARE_t1 / 3.0) * qx2;
    T2y1 = (SQUARE_t1 / 3.0) * qy1;
    T2y2 = (SQUARE_t1 / 3.0) * qy2;
    A31 = - pow(t1, (((float)n1) + 2.0)) / ((((float)n1) + 2.0) * (((float)n1) + 1.0));
    A32 = - pow(t1, (((float)n2) + 2.0)) / ((((float)n2) + 2.0) * (((float)n2) + 1.0));
    A41 = - (2.0 * pow(t1, (((float)n1) + 3.0))) / ((((float)n1) + 1.0) * (((float)n1) + 2.0) * (((float)n1) + 3.0));
    A42 = - (2.0 * pow(t1, (((float)n2) + 3.0))) / ((((float)n2) + 1.0) * (((float)n2) + 2.0) * (((float)n2) + 3.0));
    A01 = T2x1 + (S1 * T2y1);
    A02 = (A42 / A32) * ((Tx1 + Tx2) - (S2 * (Ty1 + Ty2))) - T2x2 + (S2 * T2y2);
    A11 = t1 * (1.0 + (S1 * tanf(theta1)));
    A12 = (A42 / A32) * (1.0 - (S2 * tanf(theta1)));
    A13 = ((A42 / A32) - t1) * (1.0 - (S2 * tanf(theta2)));
    A14 = (A42 * A31) / A32;
    A15 = (1.0 - cosf(theta1)) / (A31 * cosf(theta1));
    A16 = Tx1 / A31;
    A17 = (A41 * A15) + A11;
    A18 = A01 - (A41 * A16);
    A19 = (A14 * A15) + A12;
    A20 = A02 - (A14 * A16);
    A21 = 1.0 + (A31 * A15);
    A22 = (A31 * A16) - (Tx1 + Tx2);
    A23 = - (((S2 * A14 * A17) / (S1 * A41)) + A19) / A13;
    A24 = (C - A20 + (S2 * A14 * (C - A18)) / (S1 * A41)) / A13;
    A25 = - (A21 + A23) / A32;
    A26 = (A22 - A24) / A32;
    A27 = - A17 / (S1 * A41);
    A28 = (C - A18) / (S1 * A41);
    A29 = - ((A31 * A27) + tanf(theta1) + (tanf(theta2) * A23)) / A32;
    A30 = - ((A31 * A28) + (tanf(theta2) * A24) + (Ty1 + Ty2)) / A32;
    A50 = 1.0 + (A31 * A15);
    A51 = - (A42 * A25) - (t1 * A23) + (A41 * A15) + t1;
    A52 = Tx1 - (A31 * A16);
    A53 = - (A42 * A26) - (t1 * A24) - (A41 * A16) + T2x1 - T2x2 + x1 - x2;
    A54 = (A31 * A27) + tanf(theta1);
    A55 = - (A42 * A29) - (t1 * tanf(theta2) * A23) + (A41 * A27) + (t1 * tanf(theta1));
    A56 = (A31 * A28) + Ty1;
    A57 = - (A42 * A30) - (t1 * tanf(theta2) * A24) + (A41 * A28) - T2y2 - y2 + T2y1 + y1;

    bx1 = - 1.0; /* TODO => On passe toujours que dans un cas ???? */
    if (bx1 > 0.0)
    {
        /* determination de k a partir de bx1(=x_prime(t=0)=-x_prime_precedent(t=0)) */
        k = - (A56 + (A54 * bx1)) / (A57 + (A55 * bx1));
    }
    else
    {
        A58 = (A53 * A55) - (A51 * A57);
        A59 = (A52 * A55) + (A53 * A54) - (A50 * A57) - (A51 * A56);
        A60 = (A52 * A54) - (A50 * A56);

        delta_k = SQUARE(A59) - (4.0 * A58 * A60);
        k1 = (- A59 + sqrtf(delta_k)) / (2.0 * A58);
        k2 = (- A59 - sqrtf(delta_k)) / (2.0 * A58);
        if (k1 > 0.0)
        {
            k = k1;
        }
        else
        {
            k = k2;
        }
    }
    ASSER_TRAJ_LogAsserValPC("data_sds_ab_base_k", k);

    if (k < 0.0)
    {
        trajok = TRAJ_NOK;
    }
    else
    {
        trajok = TRAJ_OK;
        cfgSp1->bx = - (A56 + (k * A57)) / (A54 + (k * A55));

        cfgSp1->n = n1;
        cfgSp2->n = n2;
        cfgSp1->qx = qx1;
        cfgSp1->qy = qy1;
        cfgSp2->qx = qx2;
        cfgSp2->qy = qy2;

        cfgSp2->ay = A29 * cfgSp1->bx + A30;
        cfgSp2->ax = A25 * cfgSp1->bx + A26;
        cfgSp2->bx = A23 * cfgSp1->bx + A24;
        cfgSp1->ay = A27 * cfgSp1->bx + A28;
        cfgSp1->ax = A15 * cfgSp1->bx - A16;

        cfgSp1->by = cfgSp1->bx * tanf(theta1);
        cfgSp2->by = cfgSp2->bx * tanf(theta2);

        //ax1, bx1, ay1, by1, qx1, qy1, ax2, bx2, ay2, by2, qx2, qy2
        ASSER_TRAJ_LogAsserValPC("data_sds_ab_base", cfgSp1->ax);
        ASSER_TRAJ_LogAsserValPC("data_sds_ab_base", cfgSp1->bx);
        ASSER_TRAJ_LogAsserValPC("data_sds_ab_base", cfgSp1->ay);
        ASSER_TRAJ_LogAsserValPC("data_sds_ab_base", cfgSp1->by);
        ASSER_TRAJ_LogAsserValPC("data_sds_ab_base", cfgSp1->qx);
        ASSER_TRAJ_LogAsserValPC("data_sds_ab_base", cfgSp1->qy);
        ASSER_TRAJ_LogAsserValPC("data_sds_ab_base", cfgSp2->ax);
        ASSER_TRAJ_LogAsserValPC("data_sds_ab_base", cfgSp2->bx);
        ASSER_TRAJ_LogAsserValPC("data_sds_ab_base", cfgSp2->ay);
        ASSER_TRAJ_LogAsserValPC("data_sds_ab_base", cfgSp2->by);
        ASSER_TRAJ_LogAsserValPC("data_sds_ab_base", cfgSp2->qx);
        ASSER_TRAJ_LogAsserValPC("data_sds_ab_base", cfgSp2->qy);
    }

    return (trajok);
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_Rinv_courbure
 *
 *  \note
 *
 *  \param [in]     segmentTraj  Pointeur vers une structure de segment
 *  \param [in]     subSeg         Sous-segment
 *  \param [in]     t                   Position parametrique sur le sous-segment
 *
 *  \return
 */
/**********************************************************************/
static float ASSER_TRAJ_Rinv_courbure(segmentTrajectoire * segmentTraj, unsigned char subSeg, float t)
{
    Pose    poseTraj;
    float   diff1l_sq, dl_dt, diffTheta, Rinv_courbure;
    Vecteur diff1BS   = {0.0, 0.0};
    Vecteur diff2BS   = {0.0, 0.0};

    ASSER_TRAJ_Trajectoire_SubSegment(segmentTraj, subSeg, t, &poseTraj, &diff1BS, &diff2BS);

    diff1l_sq = SQUARE(diff1BS.x) + SQUARE(diff1BS.y);
    dl_dt = sqrtf(diff1l_sq);

    diffTheta = diff1l_sq;

    /* Restriction de la somme precedente a une valeur minimale pour eviter la division par zero et maximiser le resultat final de diffTheta */
    if (diffTheta < 1e-9)
    {
        diffTheta = 1e-9;
    }

    diffTheta = ((diff2BS.y * diff1BS.x) - (diff1BS.y * diff2BS.x)) / diffTheta;

    Rinv_courbure = diffTheta / dl_dt;

    return Rinv_courbure;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_Test_courbure
 *
 *  \note
 *
 *  \param [in]     traj    Pointeur vers une structure de deplacement
 *  \param [in]     cfgSeg  Pointeur vers une structure de configuration de segment 
 *  \param [in]     segmentTraj  Pointeur vers une structure de segment
 *
 *  \return
 */
/**********************************************************************/
static void ASSER_TRAJ_Test_courbure(Deplacement *traj, ConfigSegment * cfgSeg, segmentTrajectoire * segmentTraj)
{
    float   t_test, Rinv_sp_1, Rinv_sp_2, v_Rinv_sp_1, v_Rinv_sp_2;

    switch(cfgSeg->sp1_type)
    {
        case SPLINE3 :
            t_test = 0.0;
            break;

        case SPLINE4 :
        case SPLINE4_n :
            t_test = ti / 2.0;
            break;

        default :
#ifdef PIC32_BUILD
            TOOLS_LogFault(AsserPosErr, True, INTEGER, (int *)&cfgSeg->sp1_type, True, "spline non gere");
#else /* PIC32_BUILD */
            ASSER_TRAJ_LogAsserMsgPC("Asser: spline non gere", (float)cfgSeg->sp1_type);
#endif /* PIC32_BUILD */
        return;
    }

    Rinv_sp_1 = ASSER_TRAJ_Rinv_courbure(segmentTraj, SPLINE341, t_test);

    switch(cfgSeg->sp2_type)
    {
        case SPLINE3 :
            t_test = ti;
            break;

        case SPLINE4 :
        case SPLINE4_n :
            t_test = ti / 2.0;
            break;

        default :
#ifdef PIC32_BUILD
            TOOLS_LogFault(AsserPosErr, True, INTEGER, (int *)&cfgSeg->sp1_type, True, "spline non gere");
#else /* PIC32_BUILD */
            ASSER_TRAJ_LogAsserMsgPC("Asser: spline non gere", (float)cfgSeg->sp1_type);
#endif /* PIC32_BUILD */
            return;
    }

    Rinv_sp_2 = ASSER_TRAJ_Rinv_courbure(segmentTraj, SPLINE342, t_test);

    v_Rinv_sp_1 = ASSER_TRAJ_VitesseLimiteEnVirage(traj, fabsf(Rinv_sp_1));
    v_Rinv_sp_2 = ASSER_TRAJ_VitesseLimiteEnVirage(traj, fabsf(Rinv_sp_2));

    cfgSeg->curvature_forced = False;

    if (v_Rinv_sp_1 < cfgSeg->v_Rinv_ref)
    {
        cfgSeg->curvature_forced_1 = True;
        cfgSeg->angle_r1 = 0.0;
        cfgSeg->curvature_forced = True;
    }
    else
    {
        cfgSeg->curvature_forced_1 = False;
        cfgSeg->angle_r1 = 0.0;
    }

    if (v_Rinv_sp_2 < cfgSeg->v_Rinv_ref)
    {
        cfgSeg->curvature_forced_2 = True;
        cfgSeg->angle_r2 = 0.0;
        cfgSeg->curvature_forced = True;
    }
    else
    {
        cfgSeg->curvature_forced_2 = False;
        cfgSeg->angle_r2 = 0.0;
    }
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_Generation_curvatureForced
 *
 *  \note
 *
 *  \param [in]     cfgSeg  Pointeur vers une structure de configuration de segment
 *  \param [in]     segmentTraj  Pointeur vers une structure de segment
 *
 *  \return
 */
/**********************************************************************/
static int ASSER_TRAJ_Generation_curvatureForced(ConfigSegment * cfgSeg, segmentTrajectoire * segmentTraj)
{
    unsigned char   flag_rotation_angle_value_1;
    unsigned char   flag_rotation_angle_value_2;
    unsigned char   cpt;
    float           Rinv_sp4_1_n_0, Rinv_sp4_1_n_1, dRinv_sp4_1_n;
    float           Rinv_sp4_2_n_0, Rinv_sp4_2_n_1, dRinv_sp4_2_n;
    int             traj_sds = TRAJ_OK;

    /* Initialisation */
    if (cfgSeg->curvature_forced_1 == True)
    {
        flag_rotation_angle_value_1 = False;

        if (cfgSeg->spline4rot == True)
        {
            ASSER_TRAJ_LogAsserValPC("test_x_init", SPLINE31);
            ASSER_TRAJ_InitialSplineForCircle(cfgSeg, segmentTraj, SPLINE31);

            if (segmentTraj->spline31.split == True)
            {
                cfgSeg->s_Rinv_1 = -1.0;
            }
            else
            {
                cfgSeg->s_Rinv_1 = 1.0;
            }
        }
        cfgSeg->sp1_type = SPLINE4_n;
    }
    else
    {
        flag_rotation_angle_value_1 = True;
    }

    if (cfgSeg->curvature_forced_2 == True)
    {
        flag_rotation_angle_value_2 = False;
        if (cfgSeg->curve2 == False)
        {
            ASSER_TRAJ_LogAsserValPC("test_x_init", SPLINE32);
            ASSER_TRAJ_InitialSplineForCircle(cfgSeg, segmentTraj, SPLINE32);

            if (segmentTraj->spline32.split == True)
            {
                cfgSeg->s_Rinv_2 = -1.0;
            }
            else
            {
                cfgSeg->s_Rinv_2 = 1.0;
            }
        }
        cfgSeg->sp2_type = SPLINE4_n;
    }
    else
    {
        flag_rotation_angle_value_2 = True;
    }

    /* TODO refactoring => mal codé! */
    /* Determination des angles de rotation */
    cpt = 0;
    while ( ((flag_rotation_angle_value_1 == False) || (flag_rotation_angle_value_2 == False)) && (cpt < 10) && (traj_sds == TRAJ_OK)) /* TODO */
    {
        if (flag_rotation_angle_value_1 == False)
        {
            /* rotation 1 */
            ASSER_TRAJ_Rotation_config(cfgSeg, segmentTraj, ARC1, &segmentTraj->arc1);
        }

        if (flag_rotation_angle_value_2 == False)
        {
            /* rotation 2 */
            ASSER_TRAJ_Rotation_config(cfgSeg, segmentTraj, ARC2, &segmentTraj->arc2);
        }

        /* nouvelle s4s4 */
        traj_sds = ASSER_TRAJ_sds_ab(cfgSeg, segmentTraj);
        if (traj_sds != TRAJ_OK)
        {
            traj_sds = TRAJ_ARCNOK;
        }

        if (flag_rotation_angle_value_1 == False)
        {
            Rinv_sp4_1_n_0 = ASSER_TRAJ_Rinv_courbure(segmentTraj, SPLINE341, 0.0);
            Rinv_sp4_1_n_1 = ASSER_TRAJ_Rinv_courbure(segmentTraj, SPLINE341, (0.01 * ti));
            dRinv_sp4_1_n = fabsf(Rinv_sp4_1_n_1) - fabsf(Rinv_sp4_1_n_0);

            if (dRinv_sp4_1_n < 0.0)
            {
                flag_rotation_angle_value_1 = True;
            }
            else
            {
                cfgSeg->angle_r1 = cfgSeg->angle_r1 + (cfgSeg->s_Rinv_1 * cfgSeg->angle_step);
                segmentTraj->subSeg_used.Flags.ARC1_USED = True;
            }
        }

        if (flag_rotation_angle_value_2 == False)
        {
            Rinv_sp4_2_n_0 = ASSER_TRAJ_Rinv_courbure(segmentTraj, SPLINE342, ti);
            Rinv_sp4_2_n_1 = ASSER_TRAJ_Rinv_courbure(segmentTraj, SPLINE342, (0.99 * ti));
            dRinv_sp4_2_n = fabsf(Rinv_sp4_2_n_1) - fabsf(Rinv_sp4_2_n_0);

            if (dRinv_sp4_2_n < 0.0)
            {
                flag_rotation_angle_value_2 = True;
            }
            else
            {
                cfgSeg->angle_r2 = cfgSeg->angle_r2 + (cfgSeg->s_Rinv_2 * cfgSeg->angle_step);
                segmentTraj->subSeg_used.Flags.ARC2_USED = True;
            }
        }
        ASSER_TRAJ_LogAsserValPC("angle_r1", cfgSeg->angle_r1);
        ASSER_TRAJ_LogAsserValPC("angle_r2", cfgSeg->angle_r2);

        cpt = cpt + 1;
    }

    return traj_sds;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_InitialSplineForCircle
 *
 *  \note
 *
 *  \param [in]     cfgSeg  Pointeur vers une structure de configuration de segment
 *  \param [in]     segmentTraj  Pointeur vers une structure de segment
 *  \param [in]     subSeg  Sous-segment
 *
 *  \return
 */
/**********************************************************************/
static void ASSER_TRAJ_InitialSplineForCircle(ConfigSegment * cfgSeg, segmentTrajectoire * segmentTraj, unsigned char subSeg)
{
    float   delta_theta;

    if (subSeg == SPLINE31)
    {
        segmentTraj->subSeg_used.Flags.SPLINE31_USED = True;

        segmentTraj->spline31.x = cfgSeg->x1;
        segmentTraj->spline31.y = cfgSeg->y1;
        segmentTraj->spline31.theta = cfgSeg->theta1;

        delta_theta = cfgSeg->theta1 - cfgSeg->theta_seg;
        delta_theta = POS_ModuloAngle(delta_theta);

        if (delta_theta > 0.0)
        {
            segmentTraj->spline31.split = True;
        }
        else
        {
            segmentTraj->spline31.split = False;
        }
    }

    else if (subSeg == SPLINE32)
    {
        segmentTraj->subSeg_used.Flags.SPLINE32_USED = True;

        segmentTraj->spline32.x = cfgSeg->x2;
        segmentTraj->spline32.y = cfgSeg->y2;
        segmentTraj->spline32.theta = POS_ModuloAngle(cfgSeg->theta2 + PI);

        delta_theta = cfgSeg->theta2 - cfgSeg->theta_seg;
        delta_theta = POS_ModuloAngle(delta_theta);

        if (delta_theta > 0.0)
        {
            segmentTraj->spline32.split = True;
        }
        else
        {
            segmentTraj->spline32.split = False;
        }
    }
    else
    {
#ifdef PIC32_BUILD
        TOOLS_LogFault(AsserPosErr, True, INTEGER, (int *)&subSeg, True, "segment non gere");
#else /* PIC32_BUILD */
        ASSER_TRAJ_LogAsserMsgPC("Asser: segment non gere", (float)subSeg);
#endif /* PIC32_BUILD */
            return;
    }
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_Rotation_config
 *
 *  \note
 *
 *  \param [in]     traj    Pointeur vers une structure de deplacement
 *  \param [in]     segmentTraj  Pointeur vers une structure de segment
 *  \param [in]     cfgSeg  Pointeur vers une structure de configuration de segment
 *
 *  \return
 */
/**********************************************************************/
static void ASSER_TRAJ_Rotation_config(ConfigSegment * cfgSeg, segmentTrajectoire * segmentTraj, unsigned char subSeg, ConfigArc * arc)
{
    unsigned char   sp;
    float           tf;
    Pose            poseTrajR;
    Vecteur         pos_fr;
    float           Rinv_sp3;
    float           signe_Rinv_sp3;
    float           angle;
    float           theta_f;
    float           theta_cr, theta_fr;
    float           dl_dt, d2l_dt2;
    Vecteur         Diff1BS_f   =   {0.0, 0.0};
    Vecteur         Diff2BS_f   =   {0.0, 0.0};
    Vecteur         diff2TrajR  =   {0.0, 0.0};
    Vecteur         diff1TrajR  =   {0.0, 0.0};
    Pose            pos_f       =   {0.0, 0.0, 0.0};

    if ( ((cfgSeg->spline4rot == True) && (subSeg == ARC1)) || ((cfgSeg->curve2 == False) && (subSeg == ARC2)) )
    {
        switch (subSeg)
        {
            case ARC1 :
                sp = SPLINE31;
                angle = cfgSeg->angle_r1;
                tf = ti;

                if (segmentTraj->spline31.split == True)
                {
                    signe_Rinv_sp3 = - 1.0;
                }
                else
                {
                    signe_Rinv_sp3 = 1.0;
                }

                ASSER_TRAJ_Trajectoire_SubSegment(segmentTraj, sp, tf, &pos_f, &Diff1BS_f, &Diff2BS_f);
                theta_f = atan2f(Diff1BS_f.y, Diff1BS_f.x);

                /* Workarround library math */
                if (Diff1BS_f.y < 0.0)
                {
                    if (theta_f > 0.0)
                    {
                        theta_f = - theta_f;
                    }
                }
                else
                {
                    if (theta_f < 0.0)
                    {
                        theta_f = - theta_f;
                    }
                }

                break;

            case ARC2 :
                sp = SPLINE32;
                angle = cfgSeg->angle_r2;
                tf = 0.0;

                if (segmentTraj->spline32.split == True)
                {
                    signe_Rinv_sp3 = - 1.0;
                }
                else
                {
                    signe_Rinv_sp3 = 1.0;
                }

                ASSER_TRAJ_Trajectoire_SubSegment(segmentTraj, sp, tf, &pos_f, &Diff1BS_f, &Diff2BS_f);
                theta_f = atan2f(Diff1BS_f.y, Diff1BS_f.x);

                /* Workarround library math */
                if (Diff1BS_f.y < 0.0)
                {
                    if (theta_f > 0.0)
                    {
                        theta_f = - theta_f;
                    }
                }
                else
                {
                    if (theta_f < 0.0)
                    {
                        theta_f = - theta_f;
                    }
                }

                theta_f = POS_ModuloAngle(theta_f + PI);

                break;

            default :
#ifdef PIC32_BUILD
                TOOLS_LogFault(AsserPosErr, True, INTEGER, (int *)&subSeg, True, "arc non gere");
#else /* PIC32_BUILD */
                ASSER_TRAJ_LogAsserMsgPC("Asser: arc non gere", (float)subSeg);
#endif /* PIC32_BUILD */
                return;
        }

        /* Forcage au rayon inverse de reference */
        Rinv_sp3 = cfgSeg->Rinv_ref;
    }
    else
    {
        switch (subSeg)
        {
            case ARC1 :
                angle = cfgSeg->angle_r1;
                pos_f.x = cfgSeg->x1;
                pos_f.y = cfgSeg->y1;
                theta_f = cfgSeg->theta1;
                signe_Rinv_sp3 = cfgSeg->s_Rinv_1;
                break;

            case ARC2 :
                angle = cfgSeg->angle_r2;
                pos_f.x = cfgSeg->x2;
                pos_f.y = cfgSeg->y2;
                theta_f = POS_ModuloAngle(cfgSeg->theta2 + PI);
                signe_Rinv_sp3 = cfgSeg->s_Rinv_2;
                break;

            default :
#ifdef PIC32_BUILD
                TOOLS_LogFault(AsserPosErr, True, INTEGER, (int *)&subSeg, True, "arc non gere");
#else /* PIC32_BUILD */
                ASSER_TRAJ_LogAsserMsgPC("Asser: arc non gere", (float)subSeg);
#endif /* PIC32_BUILD */
                return;
        }

        Rinv_sp3 = cfgSeg->Rinv_ref;
    }

    if (signe_Rinv_sp3 < 0.0)
    {
        theta_cr = POS_ModuloAngle(theta_f - (PI / 2.0));
    }
    else
    {
        theta_cr = POS_ModuloAngle(theta_f + (PI / 2.0));
    }

    arc->xc = pos_f.x + (cosf(theta_cr) / Rinv_sp3);
    arc->yc = pos_f.y + (sinf(theta_cr) / Rinv_sp3);
    arc->rayon_inverse = R_INV;
    arc->theta0 = POS_ModuloAngle(theta_cr + PI);
    arc->angle = angle;

    theta_fr = POS_ModuloAngle(theta_f + angle);
    pos_fr.x = arc->xc + (cosf(POS_ModuloAngle(arc->theta0 + angle)) / Rinv_sp3);
    pos_fr.y = arc->yc + (sinf(POS_ModuloAngle(arc->theta0 + angle)) / Rinv_sp3);

    switch(subSeg)
    {
        case ARC1 :
            ASSER_TRAJ_Trajectoire_SubSegment(segmentTraj, ARC1, ti, &poseTrajR, &diff1TrajR, &diff2TrajR);
            dl_dt = sqrtf(SQUARE(diff1TrajR.x) + SQUARE(diff1TrajR.y));
            d2l_dt2 = sqrtf(SQUARE(diff2TrajR.x) + SQUARE(diff2TrajR.y));
            cfgSeg->qx1 = ((diff2TrajR.x * dl_dt) - (diff1TrajR.x * d2l_dt2)) / SQUARE(dl_dt);
            cfgSeg->qy1 = ((diff2TrajR.y * dl_dt) - (diff1TrajR.y * d2l_dt2)) / SQUARE(dl_dt);
            cfgSeg->x1_n = pos_fr.x;
            cfgSeg->y1_n = pos_fr.y;
            cfgSeg->theta1_n = theta_fr;

            ASSER_TRAJ_LogAsserValPC("data_Rotation_config_arc1", segmentTraj->arc1.angle);
            ASSER_TRAJ_LogAsserValPC("data_Rotation_config_arc1", cfgSeg->qx1);
            ASSER_TRAJ_LogAsserValPC("data_Rotation_config_arc1", cfgSeg->qy1);
            break;

        case ARC2 :
            ASSER_TRAJ_Trajectoire_SubSegment(segmentTraj, ARC2, 0.0, &poseTrajR, &diff1TrajR, &diff2TrajR);
            dl_dt = sqrtf(SQUARE(diff1TrajR.x) + SQUARE(diff1TrajR.y));
            d2l_dt2 = sqrtf(SQUARE(diff2TrajR.x) + SQUARE(diff2TrajR.y));
            cfgSeg->qx2 = ((diff2TrajR.x * dl_dt) - (diff1TrajR.x * d2l_dt2)) / SQUARE(dl_dt);
            cfgSeg->qy2 = ((diff2TrajR.y * dl_dt) - (diff1TrajR.y * d2l_dt2)) / SQUARE(dl_dt);
            cfgSeg->x2_n = pos_fr.x;
            cfgSeg->y2_n = pos_fr.y;
            cfgSeg->theta2_n = POS_ModuloAngle(theta_fr + PI);

            ASSER_TRAJ_Trajectoire_SubSegment(segmentTraj, ARC2, ti, &poseTrajR, &diff1TrajR, &diff2TrajR);
            dl_dt = sqrtf(SQUARE(diff1TrajR.x) + SQUARE(diff1TrajR.y));
            d2l_dt2 = sqrtf(SQUARE(diff2TrajR.x) + SQUARE(diff2TrajR.y));
            cfgSeg->qx0 = ((diff2TrajR.x * dl_dt) - (diff1TrajR.x * d2l_dt2)) / SQUARE(dl_dt);
            cfgSeg->qy0 = ((diff2TrajR.y * dl_dt) - (diff1TrajR.y * d2l_dt2)) / SQUARE(dl_dt);
            break;

        default :
#ifdef PIC32_BUILD
            TOOLS_LogFault(AsserPosErr, True, INTEGER, (int *)&subSeg, True, "arc non gere");
#else /* PIC32_BUILD */
            ASSER_TRAJ_LogAsserMsgPC("Asser: arc non gere", (float)subSeg);
#endif /* PIC32_BUILD */
            return;
    }
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

    if (iSegment < (nbrePts - 1))
    {        
        thetaS1 = atan2f((CONVERT_DISTANCE(point[iSegment].y) - prec_y), (CONVERT_DISTANCE(point[iSegment].x) - prec_x));

        /* Workarround library math */
        if ((CONVERT_DISTANCE(point[iSegment].y) - prec_y) < 0.0)
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

        thetaS2 = atan2f((CONVERT_DISTANCE(point[(iSegment + 1)].y) - CONVERT_DISTANCE(point[iSegment].y)), (CONVERT_DISTANCE(point[(iSegment + 1)].x) - CONVERT_DISTANCE(point[iSegment].x)));

        /* Workarround library math */
        if ((CONVERT_DISTANCE(point[(iSegment + 1)].y) - CONVERT_DISTANCE(point[iSegment].y)) < 0.0)
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
        if (chemin.trajectoire.subTrajs.use_angle == ANGLE)
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
/*! \brief ASSER_TRAJ_Spline34
 *
 *  \note
 *
 *  \param [in]     t
 *  \param [in]     t1
 *  \param [in]     n
 *  \param [in]     q
 *  \param [in]     a
 *  \param [in]     b
 *  \param [in]     c
 *  \param [in]     deriv
 *
 *  \return
 */
/**********************************************************************/
static float ASSER_TRAJ_Spline34(float t, float t1, unsigned char n, float q, float a, float b, float c, unsigned char deriv)
{
    float ret;

    switch (deriv)
    {
        case 0 :
            ret  = ((a * pow(t, ((float)n) + 3.0)) / ((((float)n) + 2.0) * (((float)n) + 3.0))) - ((q * CUBE(t)) / (6.0 * t1)) - ((a * t1 * pow(t, ((float)n) + 2.0)) / ((((float)n) + 1.0) * (((float)n) + 2.0))) + ((q * SQUARE(t)) / 2.0) + (b * t) + c;
            break;

        case 1 :
            ret  = ((a * pow(t, ((float)n) + 2.0)) / (((float)n) + 2.0)) - ((q * SQUARE(t)) / (2.0 * t1)) - ((a * t1 * pow(t, ((float)n) + 1.0)) / (((float)n) + 1.0)) + (q * t) + b;
            break;

        case 2 :
            ret  = (a * pow(t, ((float)n) + 1.0)) - ((q * t) / t1) - (a * t1 * pow(t, ((float)n))) + q;
            break;
            
        default :
#ifdef PIC32_BUILD
            TOOLS_LogFault(AsserPosErr, True, INTEGER, (int *)&deriv, True, "deriv non gere");
#else /* PIC32_BUILD */
            ASSER_TRAJ_LogAsserMsgPC("Asser: deriv non gere", (float)deriv);
#endif /* PIC32_BUILD */
            ret  = 0.0;
            break;
    }

    return ret;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_Spline3
 *
 *  \note
 *
 *  \param [in]     t
 *  \param [in]     t1
 *  \param [in]     a
 *  \param [in]     b
 *  \param [in]     c
 *  \param [in]     deriv
 *
 *  \return
 */
/**********************************************************************/
static float ASSER_TRAJ_Spline3(float t, float t1, float a, float b, float c, unsigned char deriv)
{
    float ret;

    switch (deriv)
    {
        case 2 :
            ret  = (a / t1) * t;
            break;
            
        case 1 :
            ret  = ((a / (2.0 * t1)) * SQUARE(t)) + b;
            break;
            
        case 0 :
            ret = ((a / (6.0 * t1)) * CUBE(t)) + (b * t) + c;
            break;
            
        default :
#ifdef PIC32_BUILD
            TOOLS_LogFault(AsserPosErr, True, INTEGER, (int *)&deriv, True, "deriv non gere");
#else /* PIC32_BUILD */
            ASSER_TRAJ_LogAsserMsgPC("Asser: deriv non gere", (float)deriv);
#endif /* PIC32_BUILD */
            ret  = 0.0;
            break;
    }

    return ret;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_Trajectoire_SubSegment
 *
 *  \note   Expression parametrique de type B-Spline de la position
 *
 *  \param [in]     segmentTraj     pointeur de structure definissant la trajectoire
 *  \param [in]     subSeg          nom du sous-segment
 *  \param [in]     t               position parametrique sur le sous-segment [0; t1]: 0-> depart, t1-> arrivee.
 *  \param [in]     poseTraj        sortie de la pose correspondant a la position parametrique t par pointeur
 *  \param [in]     diff1Traj       sortie du vecteur de la derivee premiere correspondant a la position parametrique t par pointeur
 *  \param [in]     diff2Traj       sortie du vecteur de la derivee seconde correspondant a la position parametrique t par pointeur
 *
 *  \return         Pose            pose correspondant a la position parametrique t
 */
/**********************************************************************/
static void ASSER_TRAJ_Trajectoire_SubSegment(segmentTrajectoire * segmentTraj, unsigned char subSeg, float pp, Pose *poseTraj, Vecteur *diff1Traj, Vecteur *diff2Traj)
{
    ConfigSpline3R  *spline3R =   NULL;
    ConfigSpline34  *spline34 =   NULL;
    ConfigArc       *arc    =   NULL;
    float           t;
    unsigned char   subSegClass, deriv;
    float           x, y;
    float           theta, angle;
    float           theta_P;
    float           x_r     =   0.0;
    float           y_r     =   0.0;

    switch (subSeg)
    {
        case SPLINE341 :
            spline34 = &segmentTraj->spline41;
            subSegClass = C_SPLINE34;
            t = pp;
            break;
            
        case SPLINE342 :
            spline34 = &segmentTraj->spline42;
            subSegClass = C_SPLINE34;
            t = ti - pp;
            break;

        case SPLINE31 :
            spline3R = &segmentTraj->spline31;
            subSegClass = C_SPLINE3;
            t = pp;
            break;
    
        case SPLINE32 :
            spline3R = &segmentTraj->spline32;
            subSegClass = C_SPLINE3;
            t = ti - pp;
            break;
   
        case ARC1 :
            arc = &segmentTraj->arc1;
            subSegClass = C_ARC;
            t = pp;
            break;
    
        case ARC2 :
            arc = &segmentTraj->arc2;
            subSegClass = C_ARC;
            t = ti - pp;
            break;

        case LINE :
            subSegClass = C_LINE;
            t = pp;
            break;

        default :
#ifdef PIC32_BUILD
            TOOLS_LogFault(AsserPosErr, True, INTEGER, (int *)&subSeg, True, "subSeg non gere");
#else /* PIC32_BUILD */
            ASSER_TRAJ_LogAsserMsgPC("Asser: subSeg non gere", (float)subSeg);
#endif /* PIC32_BUILD */
            return;
    }

    switch (subSegClass)
    {
        case C_SPLINE3 :
            for (deriv = 0; deriv < 3; deriv++)
            {
                if (spline3R != NULL)
                {
                    x = ASSER_TRAJ_Spline3(t, ti, cfgSp3.ax, cfgSp3.bx, 0.0, deriv);
                    y = (1.0 + ((float) spline3R->split) * (- 2.0)) * ASSER_TRAJ_Spline3(t, ti, cfgSp3.ay, cfgSp3.by, 0.0, deriv);
                    ASSER_TRAJ_Rotation_coord(spline3R->theta, x, y, &x_r, &y_r);

                    switch (deriv)
                    {
                        case 0 :
                            poseTraj->x = spline3R->x + x_r;
                            poseTraj->y = spline3R->y + y_r;
                            break;

                        case 1 :
                            diff1Traj->x = x_r;
                            diff1Traj->y = y_r;
                            if (subSeg == SPLINE32)
                            {
                                diff1Traj->x = - diff1Traj->x;
                                diff1Traj->y = - diff1Traj->y;
                            }
                            break;

                        case 2 :
                            diff2Traj->x = x_r;
                            diff2Traj->y = y_r;
                            break;

                        default :
#ifdef PIC32_BUILD
                            TOOLS_LogFault(AsserPosErr, True, INTEGER, (int *)&deriv, True, "deriv non gere");
#else /* PIC32_BUILD */
                            ASSER_TRAJ_LogAsserMsgPC("Asser: deriv non gere", (float)deriv);
#endif /* PIC32_BUILD */
                            return;
                    }
                }
                else
                {
#ifdef PIC32_BUILD
                    TOOLS_LogFault(AsserPosErr, False, INTEGER, NULL, True, "NULL pointer");
#else /* PIC32_BUILD */
                    ASSER_TRAJ_LogAsserMsgPC("Asser: NULL pointer", 0.0);
#endif /* PIC32_BUILD */
                    return;
                }
            }
            break;

        case C_SPLINE34 :
            for (deriv = 0; deriv < 3; deriv++)
            {
                if (spline34 != NULL)
                {
                    x = ASSER_TRAJ_Spline34(t, ti, spline34->n, spline34->qx, spline34->ax, spline34->bx, spline34->cx, deriv);
                    y = ASSER_TRAJ_Spline34(t, ti, spline34->n, spline34->qy, spline34->ay, spline34->by, spline34->cy, deriv);

                    switch (deriv)
                    {
                        case 0 :
                            ASSER_TRAJ_Rotation_coord(spline34->theta_seg, (x - spline34->cx), (y - spline34->cy), &x_r, &y_r);
                            poseTraj->x = spline34->cx + x_r;
                            poseTraj->y = spline34->cy + y_r;
                            break;

                        case 1 :
                            ASSER_TRAJ_Rotation_coord(spline34->theta_seg, x, y, &x_r, &y_r);
                            diff1Traj->x = x_r;
                            diff1Traj->y = y_r;
                            if (subSeg == SPLINE342)
                            {
                                diff1Traj->x = - diff1Traj->x;
                                diff1Traj->y = - diff1Traj->y;
                            }
                            break;

                        case 2 :
                            ASSER_TRAJ_Rotation_coord(spline34->theta_seg, x, y, &x_r, &y_r);
                            diff2Traj->x = x_r;
                            diff2Traj->y = y_r;
                            break;

                        default :
#ifdef PIC32_BUILD
                            TOOLS_LogFault(AsserPosErr, True, INTEGER, (int *)&deriv, True, "deriv non gere");
#else /* PIC32_BUILD */
                            ASSER_TRAJ_LogAsserMsgPC("Asser: deriv non gere", (float)deriv);
#endif /* PIC32_BUILD */
                            return;
                    }
                }
                else
                {
#ifdef PIC32_BUILD
                    TOOLS_LogFault(AsserPosErr, False, INTEGER, NULL, True, "NULL pointer");
#else /* PIC32_BUILD */
                    ASSER_TRAJ_LogAsserMsgPC("Asser: NULL pointer", 0.0);
#endif /* PIC32_BUILD */
                    return;
                }
            }
            break;
    
        case C_ARC :
            if (arc!= NULL)
            {
                angle = arc->angle;
                theta = arc->theta0 + ((t * angle) / ti);
                if (fabsf(angle) < (ANGLE_STEP / 2.0))
                {
                    angle = angle + (ANGLE_STEP * 0.1);
                }

                for (deriv = 0; deriv < 3; deriv++)
                {
                    switch(deriv)
                    {
                        case 0 :
                            poseTraj->x = arc->xc + (cosf(theta) / arc->rayon_inverse);
                            poseTraj->y = arc->yc + (sinf(theta) / arc->rayon_inverse);
                            break;

                        case 1 :
                            diff1Traj->x = (angle / ti) * (- sinf(theta) / arc->rayon_inverse);
                            diff1Traj->y = (angle / ti) * (cosf(theta) / arc->rayon_inverse);
                            if (subSeg == ARC2)
                            {
                                diff1Traj->x = - diff1Traj->x;
                                diff1Traj->y = - diff1Traj->y;
                            }
                            break;

                        case 2 :
                            diff2Traj->x = SQUARE(angle / ti) * (- cosf(theta) / arc->rayon_inverse);
                            diff2Traj->y = SQUARE(angle / ti) * (- sinf(theta) / arc->rayon_inverse);
                            break;

                        default :
#ifdef PIC32_BUILD
                            TOOLS_LogFault(AsserPosErr, True, INTEGER, (int *)&deriv, True, "deriv non gere");
#else /* PIC32_BUILD */
                            ASSER_TRAJ_LogAsserMsgPC("Asser: deriv non gere", (float)deriv);
#endif /* PIC32_BUILD */
                            return;
                    }
                }
            }
            else
            {
#ifdef PIC32_BUILD
                TOOLS_LogFault(AsserPosErr, False, INTEGER, NULL, True, "NULL pointer arc");
#else /* PIC32_BUILD */
                ASSER_TRAJ_LogAsserMsgPC("Asser: Asser: NULL pointer arc", (float)0.0);
#endif /* PIC32_BUILD */
            }
            break;

        case C_LINE :  
            poseTraj->x = (segmentTraj->line.ax * t) + segmentTraj->line.bx;
            poseTraj->y = (segmentTraj->line.ay * t) + segmentTraj->line.by;

            diff1Traj->x = segmentTraj->line.ax;
            diff1Traj->y = segmentTraj->line.ay;

            diff2Traj->x = 0.0;
            diff2Traj->y = 0.0;
            break;

        default :
#ifdef PIC32_BUILD
            TOOLS_LogFault(AsserPosErr, True, INTEGER, (int *)&subSegClass, True, "subSegClass non gere");
#else /* PIC32_BUILD */
            ASSER_TRAJ_LogAsserMsgPC("Asser: subSegClass non gere", (float)subSegClass);
#endif /* PIC32_BUILD */
            return;
    }

    theta_P = atan2f(diff1Traj->y, diff1Traj->x);

    /* Workarround library math */
    if (diff1Traj->y < 0.0)
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

    poseTraj->angle = POS_ModuloAngle(theta_P);
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
static Pose ASSER_TRAJ_TrajectoireRemorqueBS(Trajectoire * traj, unsigned int iSegment, unsigned char iSubSegment, float t, Pose * poseTrajRobot)
{
    Pose    poseTraj    =   {0.0, 0.0, 0.0};
    Vecteur diff1Traj   =   {0.0, 0.0};
    Vecteur diff2Traj   =   {0.0, 0.0};
    float   diffTheta;

    ASSER_TRAJ_Trajectoire(traj \
                           , iSegment \
                           , iSubSegment \
                           , t \
                           , &poseTraj, &diff1Traj, &diff2Traj);
    
    *poseTrajRobot = poseTraj;

    poseTraj.x = poseTraj.x + (NORME_BARRE_SUIVI_TRAJ * cosf(poseTraj.angle));    
    poseTraj.y = poseTraj.y + (NORME_BARRE_SUIVI_TRAJ * sinf(poseTraj.angle));    

    diffTheta = ASSER_TRAJ_DiffThetaBSpline(diff1Traj, diff2Traj);
    poseTraj.angle = atan2f((diff1Traj.y + (NORME_BARRE_SUIVI_TRAJ * diffTheta * cosf(poseTraj.angle))), (diff1Traj.x - (NORME_BARRE_SUIVI_TRAJ * diffTheta * sinf(poseTraj.angle))));

    /* Workarround library math */
    if ((diff1Traj.y + (NORME_BARRE_SUIVI_TRAJ * diffTheta * cosf(poseTraj.angle))) < 0.0)
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

    return poseTraj;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_Trajectoire
 *
 *  \note   equation parametrique de type B-Spline
 *
 *  \param [in]     traj            pointeur de structure definissant la trajectoire
 *  \param [in]     iSegment    segment
 *  \param [in]     subSeg
 *  \param [in]     t                 parametre de la courbe de Bezier [0; 1]: 0-> depart, 1-> arrivee.
 *  \param [in]     poseTraj
 *  \param [in]     diff1Traj 
 *  \param [in]     diff2Traj
 *
 *  \return           Pose   pose de la courbe fonction de l'argument t
 */
/**********************************************************************/
extern void ASSER_TRAJ_Trajectoire(Trajectoire * traj, unsigned int iSegment, unsigned char subSeg, float t, Pose * poseTraj, Vecteur * diff1Traj, Vecteur * diff2Traj)
{
    ASSER_TRAJ_Trajectoire_SubSegment(&traj->subTrajs.segmentTraj[iSegment], subSeg, t, poseTraj, diff1Traj, diff2Traj);
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
 *  \param [in]     traj               pointeur de structure definissant un segment de trajectoire
 *  \param [in]     iSegment       Segment
 *  \param [in]     iSubSegment sous-segment
 *  \param [in]     t                   parametre de la courbe parametrique [0; 1]: 0-> pose de depart, 1-> pose d'arrivee.
 *
 *  \return            diffTheta
 */
/**********************************************************************/
extern float ASSER_TRAJ_DiffThetaBSplinePerLenghtUnit(Trajectoire * traj, unsigned int iSegment, unsigned char iSubSegment, float t)
{
    Pose            poseTraj;
    float           diffTheta;
    float           facteurCorrectionT;
    Vecteur         diff1Traj   =   {0.0, 0.0};
    Vecteur         diff2Traj   =   {0.0, 0.0};

    ASSER_TRAJ_Trajectoire(traj \
                           , iSegment \
                           , iSubSegment \
                           , t \
                           , &poseTraj, &diff1Traj, &diff2Traj);

    facteurCorrectionT = sqrtf(SQUARE(diff1Traj.x) + SQUARE(diff1Traj.y));

    diffTheta = SQUARE(diff1Traj.x) + SQUARE(diff1Traj.y);

    /* Restriction de la somme precedente a une valeur minimale pour eviter la division par zero et maximiser le resultat final de diffTheta */
    if (diffTheta < ((float)1e-9))
    {
        diffTheta = 1e-9;
    }

    diffTheta = ((diff2Traj.y * diff1Traj.x) - (diff1Traj.y * diff2Traj.x)) / diffTheta;

    diffTheta = diffTheta / facteurCorrectionT;

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
extern float ASSER_TRAJ_VitesseLimiteEnVirage(Deplacement *traj, float diffThetaTrajectoire)
{
    return ((float)(traj->profilVitesse.vmax / (1.0 + (fabsf(diffThetaTrajectoire) * (ECART_ROUE_MOTRICE / 2.0) ))));
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_ParcoursTrajectoire
 *
 *  \note   Determination du gabarit de la vitesse longitudinale de consigne, dependant de la courbure de trajectoire
 *  et des fonctions d'acceleration et de decceleration.
 *
 *  \param [in]     traj                          pointeur de structure definissant la trajectoire
 *  \param [in]     delta_distance          distance de deplacement sur la periode en metre
 *  \param [in]     subSegmentCourant sous-segment courant
 *  \param [in]     segmentCourant       segment courant
 *  \param [in]     paramPoseSegTraj   parametre du segment de trajectoire
 *  \param [in]     pReturn                   flag de fin de trajectoire
 *
 *  \return            None
 */
/**********************************************************************/
extern void ASSER_TRAJ_ParcoursTrajectoire(Deplacement *traj, float delta_distance, unsigned int *segmentCourant, unsigned char *subSegmentCourant, float *paramPoseSegTraj, unsigned char * pReturn)
{
    unsigned int   memoSegmentCourant = 0;
    unsigned char   memoSubSegmentCourant;
    Pose            poseTraj;
    Vecteur         diff2Traj;
    float           facteurCorrectionT, delta_param_chemin;
    unsigned char   flag_trajectoireTerminee                = False;
    Vecteur         diff1Traj   =   {0.0, 0.0};

    do
    {
        memoSegmentCourant = *segmentCourant;
        memoSubSegmentCourant = *subSegmentCourant;

        if (ASSER_TRAJ_isDeplacement(traj) == True)
        {
            /* Determination du ratio entre un deplacement en metre et un deplacement du parametre du chemin */
            ASSER_TRAJ_Trajectoire(&traj->trajectoire, *segmentCourant, *subSegmentCourant, *paramPoseSegTraj, &poseTraj, &diff1Traj, &diff2Traj);
            facteurCorrectionT = sqrtf(SQUARE(diff1Traj.x) + SQUARE(diff1Traj.y));

            ASSER_TRAJ_LogAsserValPC("facteurCorrectionT", facteurCorrectionT);

            /* Deplacement du parametre de pose du chemin */
            delta_param_chemin = delta_distance / facteurCorrectionT;
        }
        else /* if (chemin.mouvement == ROTATION) */
        {
            facteurCorrectionT = 1.0;
            delta_param_chemin = (delta_distance / traj->distance);
        }

        *paramPoseSegTraj += delta_param_chemin;

        if (*paramPoseSegTraj > ti)
        {
            if (ASSER_TRAJ_isDeplacement(traj) == True)
            {
                /* Passage au sous-segment suivant */

                /* Recherche du sous-segment suivant */
                do
                {
                    *subSegmentCourant = *subSegmentCourant + 1;
                    if (*subSegmentCourant > SPLINE32)
                    {
                        /* Passage au segment suivant */
                        *segmentCourant = *segmentCourant + 1;
                        if (*segmentCourant < traj->trajectoire.subTrajs.nbreSegments)
                        {
                            *subSegmentCourant = traj->trajectoire.subTrajs.segmentTraj[*segmentCourant].subSeg_firstUsed;
                        }
                    }
                } while ((TEST_BIT(traj->trajectoire.subTrajs.segmentTraj[*segmentCourant].subSeg_used.Info, *subSegmentCourant) == False) && (*segmentCourant < traj->trajectoire.subTrajs.nbreSegments));

                if (*segmentCourant < traj->trajectoire.subTrajs.nbreSegments)
                {
                    /* part du deplacement du parametre du chemin au-dela du sous-segment courant */
                    *paramPoseSegTraj = *paramPoseSegTraj - ti;

                    /* part de la distance au-dela du sous-segment courant */
                    delta_distance = *paramPoseSegTraj * facteurCorrectionT;

                    *paramPoseSegTraj = 0.0;
                }
                else
                {
                    *segmentCourant = (traj->trajectoire.subTrajs.nbreSegments - 1);
                    *subSegmentCourant = traj->trajectoire.subTrajs.segmentTraj[*segmentCourant].subSeg_lastUsed;
                    *paramPoseSegTraj = ti;
                    flag_trajectoireTerminee = True;
                }
            }
            else
            {
                *segmentCourant = (traj->trajectoire.subTrajs.nbreSegments - 1);
                *paramPoseSegTraj = ti;
                flag_trajectoireTerminee = True;
            }
        }
    } while ( ((*subSegmentCourant != memoSubSegmentCourant) || (*segmentCourant != memoSegmentCourant)) && (flag_trajectoireTerminee != True));

    if (pReturn != NULL)
    {
        *pReturn = flag_trajectoireTerminee;
    }
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
    float                   tempw[2];

    erreurAngle = POS_ModuloAngle(erreurPose.angle);
    
    if (fabsf(erreurAngle) > ((PI / 2.0) - 0.1))
    {
        if (erreurAngle > 0.0)
        {
            erreurAngle = (PI / 2.0) - 0.1;
        }
        else
        {
            erreurAngle = - ((PI / 2.0) - 0.1);
        }
    }

    z = tanf(erreurAngle);

    uref1 = sqrtf(SQUARE(diffPoseRef.x) + SQUARE(diffPoseRef.y));

    uref2 = diffPoseRef.angle;

    w[0] = - gain[0] * uref1 * fabsf(uref1) * (erreurPose.x + (erreurPose.y * z));
    w[1] = - (gain[1] * uref1 * erreurPose.y) - (gain[2] * fabsf(uref1) * z);
    tempw[0] = -(gain[1] * uref1 * erreurPose.y);
    tempw[1] = - (gain[2] * fabs(uref1) * z);
    w[1] = tempw[0] + tempw[1];

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
static unsigned char ASSER_TRAJ_TestFinAsservissement(Deplacement * traj, float erDist, float memo_erDist, float tolDist, float erAngle, float memo_erAngle, float tolAngle)
{
    unsigned char   ret = False;

    if (ASSER_TRAJ_isDeplacement(traj) == True)
    {
        /* Test pour un mouvement de deplacement */
        if ((erDist < tolDist) || (((erDist < (tolDist * 10.0)) && ((erDist - memo_erDist) > 0.0))))
        {
            ret = True;

            ASSER_TRAJ_LogAsserValPC("vitesseAnglePtArret", (erAngle - memo_erAngle));
            ASSER_TRAJ_LogAsserValPC("CPTvitesseAnglePtArret", ASSER_compteurPeriode);

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
        if ( ((erAngle < tolAngle) && (erAngle > - tolAngle)) || ((erAngle * memo_erAngle) < 0.0) )
        {
            ret = True;
        }
    }

    ASSER_TRAJ_LogAsserValPC("testFinAsservissement", ret);

    return ret;
}

/**********************************************************************/
/*! \brief ASSER_TRAJ_IdentificationPoly4
 *
 *  \note  Calcul des coefficients du polynome d'ordre 4 a partir de conditions initiales
 *
 *  \param [in] y0
 *  \param [in] dy0
 *  \param [in] yi
 *  \param [in] y1
 *  \param [in] dy1
 *  \param [in] a
 *  \param [in] b
 *  \param [in] c
 *  \param [in] d
 *  \param [in] e
 *
 *  \return None
 */
/**********************************************************************/
static void ASSER_TRAJ_IdentificationPoly4(float y0, float dy0, float yi, float y1, float dy1, float * a, float * b, float * c, float * d, float * e)
{
    float   A1, A2, A3, B1, B2, B3, C1, C2, C3, D1, D2, D3;

    A1 = POWER4_t1;
    A2 = 4.0 * CUBE_t1;
    A3 = POWER4_t1_2;
    B1 = CUBE_t1;
    B2 = 3.0 * SQUARE_t1;
    B3 = CUBE_t1_2;
    C1 = SQUARE_t1;
    C2 = 2.0 * ti;
    C3 = SQUARE_t1_2;
    D1 = (dy0 * ti) + y0 - y1;
    D2 = dy0 - dy1;
    D3 = dy0 * (ti / 2.0) + y0 - yi;

    *e = y0;
    *d = dy0;
    *a = - ((((B3 - (C3 / C1) * B1) / (B2 - (C2 / C1) * B1)) * ((C2 / C1) * D1 - D2)) + D3 - (C3 / C1) * D1) / (A3 - (C3 / C1) * A1  + (((B3 - (C3 / C1) * B1) / (B2 - (C2 / C1) * B1)) * ((C2 / C1) * A1 - A2)));
    *b = ((*a) * ((C2 / C1) * A1 - A2) + (C2 / C1) * D1 - D2) / (B2 - (C2 / C1) * B1);
    *c = - (A1 / C1) * (*a) - (B1 / C1) * (*b) - (D1 / C1);
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
static void ASSER_TRAJ_DistanceTrajectoire(Trajectoire * traj, unsigned int iSegment)
{
    float           distance    = 0.000001;    /* toutes les divisions par la distance ne pourront pas etre des divisions par zero */
    unsigned char   iSubSeg;
    Pose            poseTraj;
    Vecteur         diff1Traj   = {0.0, 0.0};
    Vecteur         diff2Traj   = {0.0, 0.0};
    float           dl_dt_0, dl_dt_1, d2l_dt2_0, d2l_dt2_1, dl_dt_i;
    float           a           = 0.0;
    float           b           = 0.0;
    float           c           = 0.0;
    float           d           = 0.0;
    float           e           = 0.0;

    for (iSubSeg = SPLINE31; iSubSeg <= SPLINE32; iSubSeg++)
    {
        if (TEST_BIT(traj->subTrajs.segmentTraj[iSegment].subSeg_used.Info, iSubSeg) == True)
        {
            switch (iSubSeg)
            {
                case SPLINE31 :
                case SPLINE32 :
                    distance += DISTANCE_SPLINE3;
                    break;

                case LINE :
                    distance += sqrtf(((float)SQUARE(traj->subTrajs.segmentTraj[iSegment].line.ax) + SQUARE(traj->subTrajs.segmentTraj[iSegment].line.ay))) * ti;
                    break;

                case ARC1 :
                    distance += fabsf(traj->subTrajs.segmentTraj[iSegment].arc1.angle) / traj->subTrajs.segmentTraj[iSegment].arc1.rayon_inverse;
                    break;

                case ARC2 :
                    distance += fabsf(traj->subTrajs.segmentTraj[iSegment].arc2.angle) / traj->subTrajs.segmentTraj[iSegment].arc2.rayon_inverse;
                    break;

                case SPLINE341 :
                case SPLINE342 :
                    ASSER_TRAJ_Trajectoire_SubSegment(&traj->subTrajs.segmentTraj[iSegment], iSubSeg, 0.0, &poseTraj, &diff1Traj, &diff2Traj);
                    dl_dt_0 = sqrt(SQUARE(diff1Traj.x) + SQUARE(diff1Traj.y));
                    d2l_dt2_0 = sqrt(SQUARE(diff2Traj.x) + SQUARE(diff2Traj.y));
                    ASSER_TRAJ_Trajectoire_SubSegment(&traj->subTrajs.segmentTraj[iSegment], iSubSeg, (ti / 2.0), &poseTraj, &diff1Traj, &diff2Traj);
                    dl_dt_i = sqrt(SQUARE(diff1Traj.x) + SQUARE(diff1Traj.y));
                    ASSER_TRAJ_Trajectoire_SubSegment(&traj->subTrajs.segmentTraj[iSegment], iSubSeg, ti, &poseTraj, &diff1Traj, &diff2Traj);
                    dl_dt_1 = sqrt(SQUARE(diff1Traj.x) + SQUARE(diff1Traj.y));
                    d2l_dt2_1 = sqrt(SQUARE(diff2Traj.x) + SQUARE(diff2Traj.y));

                    ASSER_TRAJ_IdentificationPoly4(dl_dt_0, d2l_dt2_0, dl_dt_i, dl_dt_1, d2l_dt2_1, &a, &b, &c, &d, &e);

                    distance += (a * POWER5_t1) / 5.0 + (b * POWER4_t1) / 4.0 + (c * CUBE_t1) / 3.0 + (d * SQUARE_t1) / 2.0 + e * ti;
                    break;
                    
                default :
#ifdef PIC32_BUILD
                    TOOLS_LogFault(AsserPosErr, True, INTEGER, (int *)&iSubSeg, True, "Calcul dist sous-seg non gere");
#else /* PIC32_BUILD */
                    ASSER_TRAJ_LogAsserMsgPC("Asser: Calcul distance sous-segment non gere", (float)iSubSeg);
#endif /* PIC32_BUILD */
                    return;

            }
        }
    }

    traj->subTrajs.segmentTraj[iSegment].distance = distance;
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
extern void ASSER_TRAJ_LogAsserValPC(char * keyWord, float Val)
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
extern void ASSER_TRAJ_LogAsserMsgPC(char * message, float Val)
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
extern void ASSER_TRAJ_LogAsserPIC(char * keyWord, float Val1, float * pVal2, float * pVal3, float * pVal4, float * pVal5)
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
extern unsigned char ASSER_TRAJ_Profil_S_Curve(float * Vconsigne, float Distance, float VStart, float Vmax_SCurve, float VEnd, float Amax, float Dmax, float gASR, float Pr, float Vr, unsigned char fSatPI)
{
    unsigned char   AsserRunningFlag = True;
    float           Vmax_tmpMin;    
    float           Vmax_tmpMax;  

    Vr = fabsf(Vr);
    
    if (Phase == 0)
    {
        /* Phase 0 (Initialisation) */

        VmaxArrivedInAcc = False;
        
        /* Forcage de la vitesse de deplacement minimum */
        if (VStart < (VminMouv / 2.0))
        {
            VStart = VminMouv / 2.0;
        }

        /* Initilisation de la vitesse maximum */
        Vmax = Vmax_SCurve;
        
        /* Ajustement de VStart et VEnd pour eviter les division par zero */
        if ((Vmax - VStart) <= 0.0)
        {
            VStart = Vmax - 0.001;
        }
        
        if ((Vmax - VEnd) <= 0.0)
        {
            VEnd = Vmax - 0.001;
        }
        
        /* Ajustement de Vmax suivant la distance si necessaire */      
        DistanceMin = (((Vmax * Vmax) - (VStart * VStart)) / Amax) + (((Vmax * Vmax) - (VEnd * VEnd)) / Dmax) + (2 * (Vmax * TE)) + (VStart * TE) + (VEnd * TE);

        if (DistanceMin > Distance)
        {
            Vmax_tmpMin = 0.0;
            Vmax_tmpMax = Vmax;
            
            while ((Vmax_tmpMax - Vmax_tmpMin) > 0.001)
            {
                Vmax = (Vmax_tmpMin + Vmax_tmpMax) / 2.0;
                  
                DistanceMin = (((Vmax * Vmax) - (VStart * VStart)) / Amax) + (((Vmax * Vmax) - (VEnd * VEnd)) / Dmax) + (2 * (Vmax * TE)) + (VStart * TE) + (VEnd * TE);
                
                if (DistanceMin < Distance)
                {
                    Vmax_tmpMin = Vmax;
                }
                else
                {
                    Vmax_tmpMax = Vmax;
                }
            }
            Vmax = Vmax_tmpMin;
        }
        
        if ((Vmax / 2.0) < VminMouvRef)
        {
            VminMouv = (Vmax / 2.0);
        }
        else
        {
            VminMouv = VminMouvRef;
        }
        
        /* Verification des parametres */
        if (VEnd > Vmax)
        {
            VEnd = Vmax;

#ifdef PIC32_BUILD 
            if (Test_mode == (unsigned long)1)
            {
                TOOLS_LogFault(AsserPosErr, False, INTEGER, 0, True, "Reajustement vit fin profil");
            }
#else /* PIC32_BUILD */
            ASSER_TRAJ_LogAsserMsgPC("Reajustement vitesse fin de profil! Vitesse fin impossible sur la distance", VEnd);
#endif /* PIC32_BUILD */
        }
            
        /* Calcul du Jerk Max */
        JAmax = (Amax * Amax) / (Vmax - VStart);
        JDmax = (Dmax * Dmax) / (Vmax - VEnd);
        
        if (JAmax > (VminMouv / TE))
        {
            JAmax = (VminMouv / TE);
        }
        if (JDmax > (VminMouv / TE))
        {
            JDmax = (VminMouv / TE);
        }
                
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

        if (VStart > 0.0)
        {
            Vconsigne0 = VStart;
        }
        else
        {
            Vconsigne0 = 0.0;
        }
        
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
                    if ((Vr >= (V0 - EcartVitesseAcc)) && (fSatPI == False))
                    {
                       ASRrunning = True;
                    }
                }
                               
                if (ASRrunning == True)
                {                
                    /* Vitesse Reelle atteinte => delcanchement de l'ASR pour plus de performance */
                    if ((Vr >= (V0 - EcartVitesseAcc)) && (fSatPI == False))
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
                            if (fSatPI == True)
                            {                              
                                /* Modification de la vitesse suivant la vitesse precedente */
                                *Vconsigne = Vconsigne0 - VgASR;  
                                
                                VgASR = 0;                                                                                                                           
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
                    if ((Vr > (V0 + EcartVitesseDecc)) && (fSatPI == False))
                    {                       
                       ASRrunning = True;
                    }
                }
                
                if (ASRrunning == True)
                {
                    /* Vitesse Reelle superieure => delcanchement de l'ASR pour mieux deccelerer */
                    if ((Vr > (V0 + EcartVitesseDecc)) && (fSatPI == False))
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
                    if ((Vr > (V0 + EcartVitesseDecc)) && (fSatPI == False))
                    {
                        ASRrunning = True;
                    }
                }
                
                if (ASRrunning == True)
                {
                    /* Vitesse Reelle superieure => delcanchement de l'ASR pour mieux deccelerer */
                    if ((Vr > (V0 + EcartVitesseDecc)) && (Vr > VminMouv) && (fSatPI == False))
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
                    if ((Vr >= (V0 - EcartVitesseAcc)) && (fSatPI == False))
                    {
                        ASRrunning = True;
                    }
                }
                
                if (ASRrunning == True)
                {
                    /* Vitesse Reelle atteinte => delcanchement de l'ASR pour plus de performance */
                    if ((Vr >= (V0 - EcartVitesseAcc)) && (fSatPI == False))
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
#ifdef PIC32_BUILD 
                if (Test_mode == (unsigned long)1)
#endif /* PIC32_BUILD */
                {
                    if (Vr < (Vmax - (VminMouvRef/ 2)))
                    {                
#ifdef PIC32_BUILD 
                        TOOLS_LogFault(AsserPosErr, True, FLOAT, (float *)&Vr , True, "Amax>aux capacitees,Vmax non atteinte");
#else /* PIC32_BUILD */
                        ASSER_TRAJ_LogAsserMsgPC("Asser: Amax > aux capacitees du robot Vmax non atteinte Vr", Vr);
                        ASSER_TRAJ_LogAsserMsgPC("Asser: Amax > aux capacitees du robot Vmax non atteinte Vmax", Vmax);
#endif /* PIC32_BUILD */                            
                    }
                    else
                    {
                        VmaxArrivedInAcc = True;
                    }
                }
                
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
                    
                    if (ASSER_TRAJ_isDeplacement(&chemin) == True)
                    {    
                        if (Vmax >= MIN(DonneeVmaxGauche, DonneeVmaxDroite))
                        {   
                            if (Vr > (VminMouv + EcartVitesseDecc))
                            {
#ifdef PIC32_BUILD      
                                if (Test_mode == (unsigned long)1)
                                {
                                    TOOLS_LogFault(AsserPosErr, True, FLOAT, (float *)&Vr, True, "Vr>VminMouv a l'arrivee");
                                }
#else /* PIC32_BUILD */
                                ASSER_TRAJ_LogAsserMsgPC("Asser: Vr>VminMouv a l'arrivee", Vr);
#endif /* PIC32_BUILD */  
                            }
                        }
                    }
                    else
                    {
                        Vr = fabs((((float)m_sensDeplacement) * POS_GetVitesseRotation() * (ECART_ROUE_MOTRICE / 2.0)));
    
                        if (Vr > (VminMouv + EcartVitesseDecc))
                        {
#ifdef PIC32_BUILD      
                            if (Test_mode == (unsigned long)1)
                            {
                                TOOLS_LogFault(AsserPosErr, True, FLOAT, (float *)&Vr, True, "Vr>VminMouv a l'arrivee");
                            }
#else /* PIC32_BUILD */
                            ASSER_TRAJ_LogAsserMsgPC("Asser: Vr>VminMouv a l'arrivee", Vr);
#endif /* PIC32_BUILD */  
                        }
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
            TOOLS_LogFault(AsserPosErr, True, INTEGER, (int *)&Phase, True, "Phase profil non gere");
#else /* PIC32_BUILD */
            ASSER_TRAJ_LogAsserMsgPC("Asser: Phase de profil non gere", (float)Phase);
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

