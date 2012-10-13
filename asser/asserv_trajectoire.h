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
#define NBR_ASSER_LOG_VALUE             60
#else
#define NBR_ASSER_LOG_VALUE             ((256 - (3 * sizeof(float)) - (2 * sizeof(unsigned char)) - sizeof(int)) / sizeof(float))
#endif

/** Parametres du profil de vitesse des trajectoires */
#define TAILLE_TAB_GABARIT_VITESSE      2001

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
} ParametresProfilVitesse;

typedef struct
{
    Pose                    poseDepartRobot;
    float                   angle;
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
    ParametresProfilVitesse profilVitesse;
    segmentTrajectoireBS    segmentTrajBS[(NBRE_MAX_PTS_TRAJ + 1)];
    unsigned int            nbreSegments;
    ParametresRotation      rotation;
    float                   distance;
    Vecteur                 posArrivee;
} Trajectoire;

/* Variables globales de reglage de l'asser */

extern float                gainRotation1;          /* Gain de l'asservissement de la vitesse longitudinale */
extern float                gainRotation2;          /* Gain de l'asservissement de la vitesse de rotation */
extern float                gainDeplacement1;
extern float                gainDeplacement2;
extern float                gainDeplacement3;

extern float                Ratio_Acc;            
extern float                Ratio_Decc;
extern float                FacteurVitesseAngulaireMax;

extern float                A_MAX;
extern float                D_MAX;

extern float                k0_init; 
extern float                C_init;

extern float                VminMouv;

/* Prototypes de function globales asserv_trajectoire */

extern void                 ASSER_TRAJ_InitialisationGenerale(void);
extern void                 ASSER_TRAJ_InitialisationTrajectoire(Pose poseRobot, PtTraj * point, unsigned int nbrePts, unsigned int mouvement, float angle_rad);
extern void                 ASSER_TRAJ_AsservissementMouvementRobot(Pose poseRobot, VitessesRobotUnicycle * vitessesConsignes);
extern Pose                 ASSER_TRAJ_Trajectoire(segmentTrajectoireBS * segmentTraj, unsigned int iSegment, float t);
extern unsigned int         ASSER_TRAJ_GetSegmentCourant(void);
extern void                 ASSER_TRAJ_ResetLogAsserTable(void);
extern unsigned int         ASSER_TRAJ_GetCompteur(void);
extern float                ASSER_TRAJ_GabaritVitesse_getVitesse_vs_Distance(float distance);

#ifndef PIC32_BUILD
extern void                 ASSER_TRAJ_LogAsserValPC(char * keyWord, float Val);
#endif

/*! @} */

/*! @} */

/*! @} */

#endif /* _ASSERV_TRAJECTOIRE_H_ */

/* End of file : asserv_trahectoire.h */

