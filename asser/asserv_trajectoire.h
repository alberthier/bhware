/************************************************************************************************/
/*! \brief asser_trajectoire.h
 *
 *  \author Jerome Poncin/Francois Ouvradou/Nicolas Chevillon
 *  \version 1.0.0
 *  \date    16/11/2007
 */
/************************************************************************************************/

#ifndef _ASSERV_TRAJECTOIRE_H_
#define _ASSERV_TRAJECTOIRE_H_

#include "position.h"

/*! \addtogroup Task_Asser
 *  @{
 */

/*! \addtogroup Asser_Position
 *  @{
 */

/*! \addtogroup Asser_Position_Interface
 *  @{
 */
 
/* MACROS */
#define SQUARE(a) ((a)*(a))
#define CUBE(a) ((a)*(a)*(a))
#define POWER4(a) ((a)*(a)*(a)*(a))
#define POWER5(a) ((a)*(a)*(a)*(a)*(a))

/* Nombre max de points pouvant definir la trajectoire */
#define NBRE_MAX_PTS_TRAJ               62

#ifndef PIC32_BUILD
/** Parametres du profil de vitesse des trajectoires */
#define NBR_ASSER_LOG_VALUE             60
#else /* PIC32_BUILD */
#define NBR_ASSER_LOG_VALUE             ((256 - (3 * sizeof(float)) - (2 * sizeof(unsigned char)) - sizeof(int)) / sizeof(float))
#endif /* PIC32_BUILD */

#define CONVERT_DISTANCE(d)             (((float)d)/10000.0)
#define CONVERT_FLOAT2SHORT_DISTANCE(d) ((unsigned short)((d) * 10000.0))

/** Structure de parametres du profil de vitesse des trajectoires */
typedef struct
{
    float           vmax;
    unsigned int    p;
    float           diffThetaCourant;
    int             etat;
    float           distNormaliseeRestante;
    float           distance_parcourue;
    float           pas_echantillon_distance;
    float           vitesse_courante;

    /* parametres profil */
    float           Amax;
    float           Dmax;
    float           AmaxRot;
    float           DmaxRot;
} ParametresProfilVitesse;

typedef struct
{
    Pose                    poseDepartRobot;
    float                   angle;
    float                   angle_final;
} ParametresRotation;

typedef struct
{
    float                   distance;
    Vecteur                 posDepart;
    Vecteur                 vectDepart;
    Vecteur                 posArrivee;
    Vecteur                 vectArrivee;
} segmentTrajectoire;

typedef struct
{
    float                   distance;
    float                   qx[2];
    float                   qy[2];
    float                   ax;
    float                   ay;
    float                   bx;
    float                   by;
    float                   aix;
    float                   aiy;
    float                   bix;
    float                   biy;
} segmentTrajectoireBS;

typedef struct
{
    unsigned char           mouvement;
    unsigned char           use_angle;
    ParametresProfilVitesse profilVitesse;
    segmentTrajectoireBS    segmentTrajBS[(NBRE_MAX_PTS_TRAJ + 1)];
    unsigned int            nbreSegments;
    ParametresRotation      rotation;
    float                   distance;
    Vecteur                 posArrivee;
} Trajectoire;

/* Variables globales de reglage de l'asser */

extern float                gainCentreRot;
extern float                gainDeplacement1;
extern float                gainDeplacement2;
extern float                gainDeplacement3;

extern float                Ratio_Acc;            
extern float                Ratio_Decc;
extern float                Ratio_Acc_Rot;            
extern float                Ratio_Decc_Rot;

extern float                A_MAX;
extern float                D_MAX;

extern float                k0_init; 
extern float                C_init;

extern float                VminMouvRef;

extern unsigned int         ASSER_compteurPeriode;
extern unsigned int         ASSER_segmentCourant;

extern Trajectoire          chemin;

extern float                VminMouv;

extern const float          EcartVitesseDecc;
/* Prototypes de function globales asserv_trajectoire */

extern void                 ASSER_TRAJ_InitialisationGenerale(void);
extern void                 ASSER_TRAJ_InitialisationTrajectoire(Pose poseRobot, unsigned char Mouvement, Data_Goto * Data);
extern void                 ASSER_TRAJ_AsservissementMouvementRobot(Pose poseRobot, VitessesRobotUnicycle * vitessesConsignes);
extern Pose                 ASSER_TRAJ_Trajectoire(segmentTrajectoireBS * segmentTraj, unsigned int iSegment, float t);
extern void                 ASSER_TRAJ_ParcoursTrajectoire(Trajectoire *traj, float delta_distance, unsigned int *segmentCourant, float *paramPoseSegTraj, unsigned char * pReturn);
extern unsigned char        ASSER_TRAJ_isDeplacement(Trajectoire *traj);
extern float                ASSER_TRAJ_DiffThetaBSplinePerLenghtUnit(segmentTrajectoireBS * segmentTraj, unsigned int iSegment, float t);
extern float                ASSER_TRAJ_VitesseLimiteEnVirage(Trajectoire *traj, float diffThetaTrajectoire);

#ifdef PIC32_BUILD
extern void                 ASSER_TRAJ_LogAsserPIC(char * keyWord, float Val1, float * pVal2, float * pVal3, float * pVal4, float * pVal5);
#define                     ASSER_TRAJ_LogAsserValPC(a, b)
#define                     ASSER_TRAJ_LogAsserMsgPC(a, b)
#else /* PIC32_BUILD */
#define                     ASSER_TRAJ_LogAsserPIC(a, b, c, d, e, f)
extern void                 ASSER_TRAJ_LogAsserValPC(char * keyWord, float Val);
void                        ASSER_TRAJ_LogAsserMsgPC(char * message, float Val);
#endif /* PIC32_BUILD */

/*! @} */

/*! @} */

/*! @} */

#endif /* _ASSERV_TRAJECTOIRE_H_ */

/* End of file : asserv_trahectoire.h */

