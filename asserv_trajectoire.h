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
#ifdef PIC32_BUILD
#include "net.h"
#endif

/*! \addtogroup Task_Asser
 *  @{
 */

/*! \addtogroup Asser_Position
 *  @{
 */

/*! \addtogroup Asser_Position_Interface
 *  @{
 */
 
/* Nombre max de points pouvant definir la trajectoire */
#ifndef PIC32_BUILD
#define NBRE_MAX_PTS_TRAJ    70
#else
#define NBRE_MAX_PTS_TRAJ    5
#endif

#ifndef PIC32_BUILD
#define NBR_ASSER_LOG_VALUE     60
#else
#define NBR_ASSER_LOG_VALUE  ((NET_BUF_CFG_DATA_SIZE_SMALL - (3 * sizeof(float)) - (2 * sizeof(unsigned char)) - sizeof(int)) / sizeof(float))
#endif

#define TAILLE_TAB_GABARIT_VITESSE  101
#define PAS_TEMPS 0
#define PAS_DISTANCE 1

/** Structure de parametres du profil de vitesse des trajectoires */
typedef struct
{
    //unsigned int n;
    float           vmax;
    float           acc;
    float           delta_tmax;
    unsigned int    pmax;
    float           delta_t;
    float           delta2t;
    unsigned int    p;
    float           diffThetaCourant;
    int             etat;
    float           distNormaliseeRestante;

    float           distance_parcourue;
    signed int      phase;
    float           pas_echantillon_distance;
    float           acc_alpha;
    float           acc_a1;
    float           acc_a2;
    float           decc_alpha;
    float           decc_a1;
    float           decc_a2;
    float           vitesse_courante;
    float           decc_vmax;
    float           decc_tempsAcc;
    float           acc_D1;
    float           acc_D2;
    float           acc_Dtot;
    float           decc_D3;
    float           decc_D4;
    float           decc_Dtot;
    //unsigned int m;
} ParametresProfilVitesse;

typedef struct
{
    Pose    poseDepartRobot;
    float   angle;
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
    float                   mx[2];
    float                   my[2];
    float                   qx[2];                  /* Defini pour l'ordre 5 */
    float                   qy[2];
    float                   ax;
    float                   ay;
    float                   bx;
    float                   by;
    float                   t[2];
    unsigned int            i;
    unsigned int            ordre;
} segmentTrajectoireBS;

typedef struct
{
    unsigned int            mouvement;
    ParametresProfilVitesse profilVitesse;
    segmentTrajectoireBS    segmentTrajBS[NBRE_MAX_PTS_TRAJ];
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

extern float                tempsAcc;               /* Acceleration du deplacement sur la trajectoire, -> temps en secondes pour passer de 0 a la vitesse max retournee par POS_GetConsVitesseMax() */
extern float                ALPHA_ACC;
extern float                ALPHA_DECC;
extern float                facteurVitesseAngulaireMax;

/** Table pour le log asser */
extern float                tabLogAsser[NBR_ASSER_LOG_VALUE];

/** Echnatillon de mesure pour le log asser */
extern unsigned char        Sample;

/** Pour prendre les mesures 1/2 */
extern unsigned char        TakeMesure;

/** Prototypes de function globales asserv_trajectoire */

extern void                 ASSER_TRAJ_InitialisationGenerale(void);
extern void                 ASSER_TRAJ_InitialisationTrajectoire(Pose poseRobot, PtTraj *point, unsigned int nbrePts, unsigned int mouvement);
extern void                 ASSER_TRAJ_AsservissementMouvementRobot(Pose poseRobot, VitessesRobotUnicycle *vitessesConsignes);
extern Pose                 ASSER_TRAJ_Trajectoire(float t);
extern unsigned int         ASSER_TRAJ_GetSegmentCourant(void);
extern void                 ASSER_TRAJ_LogAsser(char *keyWord, unsigned char index, float val);
extern void                 ASSER_TRAJ_ResetLogAsserTable(void);
extern unsigned int         ASSER_TRAJ_GetCompteur(void);

#ifndef PIC32_BUILD
extern void                 ASSER_TRAJ_InitialisationLogAsser(void);
#endif

/*! @} */

/*! @} */

/*! @} */

#endif /* _ASSERV_TRAJECTOIRE_H_ */

/* End of file : asserv_trahectoire.h */

