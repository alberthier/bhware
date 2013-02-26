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
#define     ROTATE                      ((unsigned char)     0u)
#define     MOVE_CURVE                  ((unsigned char)     1u)
#define     MOVE_LINE                   ((unsigned char)     2u)
#define     MOVE_ARC                    ((unsigned char)     3u)

#define     MARCHE_AVANT                ((signed char)       1)
#define     MARCHE_ARRIERE              ((signed char)       -1)

#define     ANGLE_INVALID               ((float)             -1e6)

/** Parametres materiels */
#define     NBRE_PAS                    ((unsigned int)(DonneeNbrPasCdrDroite * 4))

/** Parametres pour le calcul des consignes des regulateurs PI */
#define     BORNE_PWM_AV                2047u
#define     BORNE_PWM_AR                0u
#define     OffsetPWM                   1023u
/* Nombre maximum de veteurs dans une trame */
#define     MaxNbrPtTrajInTrame1        ((int)((256 - sizeof(unsigned char) - sizeof(unsigned char) - sizeof(signed char) - sizeof(unsigned char) - sizeof(float) - sizeof(unsigned char)) / sizeof(PtTraj)))
#define     MaxNbrPtTrajInTrame2        ((int)((256 - sizeof(unsigned char) - sizeof(unsigned char) - sizeof(signed char) - sizeof(PtTraj)) / sizeof(float)))

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
typedef enum
{
    NO_ANGLE         = (unsigned char)0,
    ANGLE        = (unsigned char)1
} Type_Deplacement;

typedef union
{
    /* params de rotation */
    struct __attribute__ ((packed))
    {        
        signed char     Marche;    
        float           Angle;
    } rotation;

    /* params courbe */
    struct __attribute__ ((packed))
    {   
        signed char     Marche; 
        unsigned char   Use_Angle;
        float           Angle;
        unsigned char   NbrePtsChemin;
        PtTraj          Chemin[MaxNbrPtTrajInTrame1];
    } courbe;

    /* params courbe */
    struct __attribute__ ((packed))
    {
        signed char     Marche;
        unsigned char   NbrePtsChemin;
        PtTraj          Chemin[MaxNbrPtTrajInTrame1];
    } line;

    /* params arc */
    struct __attribute__ ((packed))
    {
        signed char     Marche;
        PtTraj          Centre_rotation;
        float           Rayon;
        unsigned char   NbrePtsChemin;
        float           Chemin[MaxNbrPtTrajInTrame2];
    } arc;
} Data_Goto;

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

