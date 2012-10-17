/************************************************************************************************/
/*! \brief position.h
 *
 *  \author Jerome Poncin/Francois Ouvradou/Nicolas Chevillon
 *  \version 1.0.0
 *  \date    21/12/2008
 */
/************************************************************************************************/

#ifndef _POSITION_H_
#define _POSITION_H_

/*! \addtogroup Task_Asser
 *  @{
 */

/*! \addtogroup Position
 *  @{
 */

/*! \addtogroup Position_Interface
 *  @{
 */

extern unsigned short                   DonneeNbrPasCdrDroite;

/* Define asser de position */

/** Flag de mouvement */
#define     ROTATION                    ((unsigned char)     0u)
#define     DEPLACEMENT                 ((unsigned char)     1u)
#define     DEPLACEMENT_LIGNE_DROITE    ((unsigned char)     2u)

#define     MARCHE_AVANT                ((signed char)       1)
#define     MARCHE_ARRIERE              ((signed char)       -1)

#define     ANGLE_INVALID               ((float)             -1e6)

/** Parametres materiels */
#define     NBRE_PAS                    ((unsigned int)(DonneeNbrPasCdrDroite * 4))

/** Parametres pour le calcul des consignes des regulateurs PI */
#define     BORNE_PWM_AV                2047u
#define     BORNE_PWM_AR                0u
#define     OffsetPWM                   1023u

/** Structure de coordonnees et de vitesses */
typedef struct __attribute__ ((packed))
{
    float x;
    float y;
    float angle;
} Pose;

typedef struct __attribute__ ((packed))
{
    float x;
    float y;
} Vecteur;

typedef struct __attribute__ ((packed))
{
    unsigned short  x;
    unsigned short  y;
} PtTraj;

typedef struct __attribute__ ((packed))
{
    float longitudinale;
    float rotation;
} VitessesRobotUnicycle;

/* Variables globales */

/** Gains statiques des moteurs de deplacement */
extern float            GAIN_STATIQUE_MOTEUR_G;
extern float            GAIN_STATIQUE_MOTEUR_D;

/** Periode de la commande d'asservissement */
extern const float      TE;

/** Ecart entre les roues libres des codeurs incrementaux */
extern float            ECART_ROUE_LIBRE;   

/** Entraxe des roues motrices */
extern float            ECART_ROUE_MOTRICE;

/** Tolerances de la condition d'arret des asservissements */
extern float            DIST_MIN;
extern float            ANGLE_MIN;

/** Coefficient de glissement lateral */
extern float            COEFFICIENT_DE_GLISSEMENT_LATERAL;

/** UMAX tension PWM max + offset (0x03FF) */
extern const float      Umax;

/** Coordonnees de la pose actuelle du robot */
extern Pose             m_poseRobot;

/** Sens de deplacement du robot */
extern signed char      m_sensMarcheMouvement;

/** Prototypes de function globales asserv_position */

extern void             POS_InitialisationConfigRobot(void);
extern void             POS_InitialisationPoseRobot(Pose poseRobot);
extern void             POS_InitialisationSensMarche(signed char marche);
extern Pose             POS_GetPoseRobot(void);
extern Pose             POS_GetPoseAsserRobot(void);
extern void             POS_Positionnement(signed int delta_impDroite, signed int delta_impGauche);
extern float            POS_ModuloAngle(float angle);
extern float            POS_ErreurDistance(Pose poseRobot, Vecteur posArrivee);
extern float            POS_ErreurOrientation(Pose posRobot, Vecteur posArrivee);
extern void             POS_ConversionVitessesLongRotToConsignesPWMRouesRobotUnicycle(float vitesseLongitudinale, float vitesseRotation, unsigned short * consPWMRoueGauche, unsigned short * consPWMRoueDroite);
extern float            POS_GetConsVitesseMax(void);
extern float            POS_GetConsVitesseAngulaireMax(void);
extern void             POS_SetGainStatiqueMoteur(float gain_G, float gain_D);
extern float            POS_GetVitesseRelle(void);
extern float            POS_GetVitesseRotation(void);
 
/*! @} */

/*! @} */

/*! @} */

#endif /* _POSITION_H_ */

/* End of file : position.h */

