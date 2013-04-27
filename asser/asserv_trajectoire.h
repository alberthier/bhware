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

/*! \addtogroup Asser
 *  @{
 */

/*! \addtogroup Asser_Trajectoire
 *  @{
 */

/*! \addtogroup Asser_Trajectoire_Interface
 *  @{
 */

/* Nombre max de points pouvant definir la trajectoire */
#define NBRE_MAX_PTS_TRAJ               63

#define CONVERT_DISTANCE(d)             (((float)d)/10000.0)
#define CONVERT_FLOAT2SHORT_DISTANCE(d) ((unsigned short)((d) * 10000.0))

#ifndef PIC32_BUILD
#define PLOTS_SIMU
#endif /* PIC32_BUILD */

typedef enum
{
    SPLINE31     = (unsigned char)0,
    ARC1         = (unsigned char)1,
    SPLINE341    = (unsigned char)2,
    LINE         = (unsigned char)3,
    SPLINE342    = (unsigned char)4,
    ARC2         = (unsigned char)5,
    SPLINE32     = (unsigned char)6,
    SPLINE341_EXT = (unsigned char)7,
    SPLINE342_EXT = (unsigned char)8
} Type_SubSeg;

typedef enum
{
    SPLINE3      = (unsigned char)0,
    SPLINE4      = (unsigned char)1,
    SPLINE4_n    = (unsigned char)2
} Type_Spline;

typedef enum
{
    C_LINE         = (unsigned char)0,
    C_SPLINE3      = (unsigned char)1,
    C_SPLINE34     = (unsigned char)2,
    C_ARC          = (unsigned char)3
} Type_SegClass;

/** Structure de parametres du profil de vitesse des trajectoires */
typedef struct __attribute__ ((aligned (4)))
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

typedef struct __attribute__ ((aligned (4)))
{
    float           angle;
    float           angle_final;
    Pose            poseDepartRobot;
} ParametresRotation;

typedef struct __attribute__ ((aligned (4)))
{
    float           ax;
    float           ay;
    float           bx;
    float           by;
} ConfigSpline3;

typedef struct __attribute__ ((aligned (4)))
{
    float           x;
    float           y;
    float           theta;
    unsigned char   split;

} ConfigSpline3R;

typedef struct __attribute__ ((aligned (4)))
{
    float           ax;
    float           ay;
    float           bx;
    float           by;
    float           cx;
    float           cy;
    float           qx;
    float           qy;
    float           theta_seg;    
    unsigned char   n;

} ConfigSpline34;

typedef struct __attribute__ ((aligned (4)))
{
    float           xc;
    float           yc;
    float           rayon_inverse;
    float           theta0;
    float           angle;
} ConfigArc;

typedef struct __attribute__ ((aligned (4)))
{
    float           distance;
    float           theta_seg;

    struct
    {
        float       ax;
        float       ay;
        float       bx;
        float       by;
    } line;

    ConfigSpline3R  spline31;
    ConfigSpline3R  spline32;
    ConfigSpline34  spline41;
    ConfigSpline34  spline42;
    ConfigArc       arc1;
    ConfigArc       arc2;

    union
    {
        struct
        {
            unsigned char SPLINE31_USED     : 1;
            unsigned char ARC1_USED         : 1;
            unsigned char SPLINE341_USED    : 1;
            unsigned char LINE_USED         : 1;
            unsigned char SPLINE342_USED    : 1;
            unsigned char ARC2_USED         : 1;
            unsigned char SPLINE32_USED     : 1;
            unsigned char Dummy             : 1;
        } Flags;

        unsigned char     Info;
    } subSeg_used;

    unsigned char    subSeg_firstUsed;
    unsigned char    subSeg_lastUsed;
    unsigned char    line_used;
} segmentTrajectoire;

typedef union __attribute__ ((aligned (4)))
{
    struct
    {        
        unsigned int            nbreSegments;
        unsigned int            segmentCourant;
        float                   paramPoseSubSegCourant;
        segmentTrajectoire      segmentTraj[NBRE_MAX_PTS_TRAJ];
        unsigned char           subSegmentCourant;
        unsigned char           use_angle;
    } subTrajs;

    ParametresRotation          rotation;
} Trajectoire;

typedef struct __attribute__ ((aligned (4)))
{    
    float                   distance;
    Vecteur                 posArrivee;
    ParametresProfilVitesse profilVitesse;
    Trajectoire             trajectoire;    
    unsigned char           mouvement;
} Deplacement;

typedef struct __attribute__ ((aligned (4)))
{
    float           v_Rinv_ref;
    float           Rinv_ref;
    float           x1;
    float           y1;
    float           theta1;
    float           x2;
    float           y2;
    float           theta2;
    float           x1_n;
    float           y1_n;
    float           theta1_n;
    float           qx1;
    float           qy1;
    float           x2_n;
    float           y2_n;
    float           theta2_n;
    float           qx2;
    float           qy2;
    float           Rb_prec;
    float           qx0;
    float           qy0;    
    float           angle_step;
    float           angle_r1;
    float           angle_r2;    
    float           theta_seg;
    float           s_Rinv_1;
    float           s_Rinv_2;
    unsigned char   sp1_type;
    unsigned char   sp2_type;
    unsigned char   curve1;
    unsigned char   curve2;
    unsigned char   curvature_forced;
    unsigned char   curvature_forced_1;
    unsigned char   curvature_forced_2;
    unsigned char   spline4rot;
    unsigned char   inflexion_point;
} ConfigSegment;

/* Variables globales de reglage de l'asser */

extern float                gainCentreRot;
extern float                gainDeplacement1;
extern float                gainDeplacement2;
extern float                gainDeplacement3;

extern float                Ratio_Acc;            
extern float                Ratio_Decc;
extern float                Ratio_Acc_Rot;            
extern float                Ratio_Decc_Rot;
extern float                Vmax_limit;

extern float                A_MAX;
extern float                D_MAX;

extern float                VminMouvRef;

extern unsigned int         ASSER_compteurPeriode;
extern unsigned int         ASSER_segmentCourant;

extern Deplacement          chemin;

extern float                VminMouv;

extern const float          EcartVitesseDecc;
extern float                EcartVitesseAcc;

extern unsigned char        VmaxArrivedInAcc;

/* Prototypes de function globales asserv_trajectoire */

extern void                 ASSER_TRAJ_InitialisationGenerale(void);
extern void                 ASSER_TRAJ_InitialisationTrajectoire(Pose poseRobot, unsigned char Mouvement, Data_Goto * Data);
extern void                 ASSER_TRAJ_AsservissementMouvementRobot(Pose poseRobot, VitessesRobotUnicycle * vitessesConsignes);
extern void                 ASSER_TRAJ_Trajectoire(Trajectoire * traj, unsigned int iSegment, unsigned char subSeg, float t, Pose * poseTraj, Vecteur * diff1Traj, Vecteur * diff2Traj);
extern void                 ASSER_TRAJ_ParcoursTrajectoire(Deplacement *traj, float delta_distance, unsigned int *segmentCourant, unsigned char *subSegmentCourant, float *paramPoseSegTraj, unsigned char * pReturn);
extern unsigned char        ASSER_TRAJ_isDeplacement(Deplacement *traj);
extern float                ASSER_TRAJ_DiffThetaBSplinePerLenghtUnit(Trajectoire * traj, unsigned int iSegment, unsigned char iSubSegment, float t);
extern float                ASSER_TRAJ_VitesseLimiteEnVirage(Deplacement *traj, float diffThetaTrajectoire);

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

/* End of file : asserv_trajectoire.h */

