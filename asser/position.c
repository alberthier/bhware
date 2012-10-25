/************************************************************************************************/
/*! \brief position.c
 *
 *  \author Jerome Poncin/Francois Ouvradou/Nicolas Chevillon
 *  \version 1.0.0
 *  \date    21/12/2008
 */
/************************************************************************************************/

/************************************************************************************************
 * position managment : Gestion de la pose du robot
 *                                  -> boite a outils du traitement de la pose du robot.
 ************************************************************************************************/

/* Fichier *.h qui sont utilises par le module Position */

#include "define.h"
#ifdef PIC32_BUILD
#include "includes.h"
#include "tools.h"
#include "HTTPpages.h"
#include "task_uart.h"
#ifndef S_SPLINT_S
#include "math.h"
#endif /* S_SPLINT_S */
#else /* PIC32_BUILD */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "simuAsser.h"
#endif /* PIC32_BUILD */
#include "pic18.h"
#include "asserv_trajectoire.h"

/*! \addtogroup Task_Asser
 *  @{
 */

/*! \addtogroup Position
 *  @{
 */

/*! \addtogroup Position_Core
 *  @{
 */

/**********************************************************************/

/* Constantes */

#define                 PERIMETRE_DROIT         (PI * (DonneeDRoueDroite / 1000))   /* Perimetre en m de la roue libre du codeur droit */
#define                 PERIMETRE_GAUCHE        (PI * (DonneeDRoueGauche / 1000))   /* Perimetre en m de la roue libre du codeur gauche */
    
const   float           TE                      = 0.010;                            /* Periode de l'asservissement en seconde */

/** Vitesse maximale a demander aux moteurs */
const   float           Umax                    = 1023.0;                           /* Vitesse maximale a demander aux moteurs, image de la tension (en unite PWM) etant donne le gain statique des moteurs */

/**********************************************************************/

/* Variables globales */

float                   GAIN_STATIQUE_MOTEUR_D;                                     /* Gain statique du moteur CC droit en (m/s) / unitePWM */
float                   GAIN_STATIQUE_MOTEUR_G;                                     /* Gain statique du moteur CC gauche en (m/s) / unitePWM */
float                   GAIN_STATIQUE_MOTEUR;

float                   ECART_ROUE_LIBRE        = 0.235;                            /* Ecart entre les roues libres des codeurs incrementaux */
float                   ECART_ROUE_MOTRICE      = 0.1735;                           /* Entraxe des roues motrices */
float                   COEFFICIENT_DE_GLISSEMENT_LATERAL = 0.0;

/** Tolerances de la condition d'arret des asservissements */
float                   DIST_MIN                = 0.001;
float                   ANGLE_MIN               = 0.001;

/** Coordonnees de la pose actuelle du robot */
Pose                    m_poseRobot             = {0.0, 0.0, 0.0};

/** Sens de deplacement du robot */
signed char             m_sensMarcheMouvement   = MARCHE_AVANT;

/**********************************************************************/

/* Variables locales */

/** Vitesse reelle mesure */
static float            VitesseReelleMesure     =   0.0;
static float            VitesseRotationMesure   =   0.0;

/**********************************************************************
 * Definition dedicated to the local functions.
 **********************************************************************/

/**********************************************************************/
/*! \brief POS_InitialisationConfigRobot
 *
 *  \note   Configuration des parametres lies au materiel du robot
 *
 *  \return None
 */
/**********************************************************************/
extern void POS_InitialisationConfigRobot(void)
{
#ifdef PIC32_BUILD

    POS_SetGainStatiqueMoteur((DonneeVmaxGauche / 1023.0), (DonneeVmaxDroite / 1023.0));

#else /* PIC32_BUILD */

    //POS_SetGainStatiqueMoteur(SIMU_gain(), SIMU_gain());
    POS_SetGainStatiqueMoteur((DonneeVmaxGauche / 1023.0), (DonneeVmaxDroite / 1023.0));

#endif  /* PIC32_BUILD */
}

/**********************************************************************/
/*! \brief POS_InitialisationPoseRobot
 * 
 *  \note   Configuration de la pose du robot selon une orientation MARCHE_AVANT
 *
 *  \param [in] poseRobot   pose du robot
 *
 *  \return None
 */
/**********************************************************************/
extern void POS_InitialisationPoseRobot(Pose poseRobot)
{   
    m_poseRobot.x = poseRobot.x;
    m_poseRobot.y = poseRobot.y;
    m_poseRobot.angle = poseRobot.angle;
}

/**********************************************************************/
/*! \brief POS_InitialisationSensMarche
 * 
 *  \note   Configuration de l'orientation du robot avant la demande d'execution d'un mouvement
 *
 *  \param [in] marche   MARCHE_AVANT ou MARCHE_ARRIERE
 *
 *  \return None
 */
/**********************************************************************/
extern void POS_InitialisationSensMarche(signed char marche)
{
    m_sensMarcheMouvement = marche;
}

/**********************************************************************/
/*! \brief POS_GetPoseRobot
 * 
 *  \note   Lecture de la pose du robot du module POSITION
 *
 *  \return pose du robot selon une orientation MARCHE_AVANT 
 *
 *  \return None
 */
/**********************************************************************/
extern Pose POS_GetPoseRobot(void)
{
    return m_poseRobot;
}

/**********************************************************************/
/*! \brief POS_GetPoseAsserRobot
 * 
 *  \note   calcul et retourne la pose du robot
 *
 *  \return pose du robot dont l'orientation depend de l'orientation (AVANT ou ARRIERE) configuree dans le module POSITION
 */
/**********************************************************************/
extern Pose POS_GetPoseAsserRobot(void)
{
    Pose poseRobot;

    poseRobot.x = m_poseRobot.x;
    poseRobot.y = m_poseRobot.y;
    poseRobot.angle = m_poseRobot.angle;
    
    if (m_sensMarcheMouvement == MARCHE_ARRIERE)
    {
        poseRobot.angle = POS_ModuloAngle(m_poseRobot.angle + PI);
    }

    return poseRobot;
}

/**********************************************************************/
/*! \brief POS_SetGainStatiqueMoteur
 *
 *  \note   Calcul et retourne la vitesse maximale de consigne des moteurs de deplacement
 *
 *  \param [in] gain_G     Gain statique du moteur de deplacement gauche 
 *  \param [in] gain_D     Gain statique du moteur de deplacement droit
 */
/**********************************************************************/
extern void POS_SetGainStatiqueMoteur(float gain_G, float gain_D)
{
     GAIN_STATIQUE_MOTEUR_G = gain_G;
     GAIN_STATIQUE_MOTEUR_D = gain_D;
     GAIN_STATIQUE_MOTEUR = MIN(GAIN_STATIQUE_MOTEUR_G, GAIN_STATIQUE_MOTEUR_D);
}

/**********************************************************************/
/*! \brief POS_GetConsVitesseMax
 * 
 *  \note   Calcul et retourne la vitesse maximale de consigne des moteurs de deplacement
 *
 *  \return vitesse en m/s
 */
/**********************************************************************/
extern float POS_GetConsVitesseMax(void)
{
    return ((float)(Umax * GAIN_STATIQUE_MOTEUR));
}

/**********************************************************************/
/*! \brief POS_GetConsVitesseAngulaireMax
 * 
 *  \note   Calcul et retourne la vitesse angulaire maximale de rotation de consigne pour le deplacement du robot
 *
 *  \return vitesse en rd/s
 */
/**********************************************************************/
extern float POS_GetConsVitesseAngulaireMax(void)
{
    return ((float)((Umax * GAIN_STATIQUE_MOTEUR * 2.0) / ECART_ROUE_LIBRE));
}

/**********************************************************************/
/*! \brief POS_Positionnement
 * 
 *  \note  Calcul de la nouvelle position du robot et des vitesses de ses roues motrices
 *  -> vitesse en unite entiere par seconde
 *
 *  \param [in]   delta_impDroite      Delta du nombre d'impulsions roue droite
 *  \param [in]   delta_impGauche    Delta du nombre d'impulsions roue gauche 
 *
 *  \return None
 */
/**********************************************************************/ 
extern void POS_Positionnement(signed int delta_impDroite, signed int delta_impGauche)
{
    float Delta_moy, Delta_diff;
    float Dx, Dy;
    float Dtheta;                   /* Angle de rotation du robot entre deux lecture des capteurs de position */
    float Deviation_x, Deviation_y;

    Delta_moy = (((float)delta_impDroite * PERIMETRE_DROIT) + ((float)delta_impGauche * PERIMETRE_GAUCHE)) / (2.0 * (float)NBRE_PAS);
    Delta_diff = ((((float)delta_impDroite * PERIMETRE_DROIT) - ((float)delta_impGauche * PERIMETRE_GAUCHE)) / (float)NBRE_PAS);

    VitesseReelleMesure = Delta_moy / TE;
    
#ifndef PIC32_BUILD 
    ASSER_TRAJ_LogAsserValPC("VitesseReelleMesure", VitesseReelleMesure);
#endif /* PIC32_BUILD */ 

    Dx = (Delta_moy * cosf(m_poseRobot.angle));
    Dy = (Delta_moy * sinf(m_poseRobot.angle));

    Dtheta = (Delta_diff / ECART_ROUE_LIBRE);
    VitesseRotationMesure = Dtheta / TE;

    Deviation_x = - COEFFICIENT_DE_GLISSEMENT_LATERAL * Dtheta * Dy;
    Deviation_y = COEFFICIENT_DE_GLISSEMENT_LATERAL * Dtheta * Dx;

    m_poseRobot.x = m_poseRobot.x + Dx + Deviation_x;
    m_poseRobot.y = m_poseRobot.y + Dy + Deviation_y;
    m_poseRobot.angle = POS_ModuloAngle(m_poseRobot.angle + Dtheta);
}

/**********************************************************************/
/*! \brief POS_GetVitesseRelle
 * 
 *  \note  Retourne la vitesse reelle longitudinale du robot
 *
 *  \return Vitesse reelle mesure
 */
/**********************************************************************/
extern float POS_GetVitesseRelle(void)
{
    return VitesseReelleMesure;
}

/**********************************************************************/
/*! \brief POS_GetVitesseRotation
 *
 *  \note  Retourne la vitesse reelle de rotation du robot
 *
 *  \return Vitesse de rotation mesuree (en rd/s)
 */
/**********************************************************************/
extern float POS_GetVitesseRotation(void)
{
    return VitesseRotationMesure;
}

/**********************************************************************/
/*!  \brief POS_ModuloAngle
 * 
 *  \note Fonction de modulo d'un angle, conservation de l'angle entre [-pi; pi] 
 *
 *  \param [in] angle       angle en radian
 *
 *  \return angle module
 */
/**********************************************************************/
extern float POS_ModuloAngle(float angle)
{
    angle = fmodf(angle, (2.0 * PI));

    if (angle >= PI)
    {
        angle -= (2.0 * PI);
    }
    if (angle < (- PI))
    {
        angle += (2.0 * PI);
    }

    return(angle);
}

/**********************************************************************/
/*! \brief POS_ErreurDistance
 * 
 *  \note Distance en ligne droite entre le robot et le point d'arrivee
 *
 *  \param [in] poseRobot   pose du robot
 *  \param [in] posArrivee  position du point d'arrivee
 *
 *  \return Erreur Distance
 */
/**********************************************************************/
extern float POS_ErreurDistance(Pose poseRobot, Vecteur posArrivee)
{
    float temp1;
    float temp2;
    
    temp1 = ((poseRobot.x - posArrivee.x) * (poseRobot.x - posArrivee.x));
    temp2 = ((poseRobot.y - posArrivee.y ) * (poseRobot.y - posArrivee.y ));

    return sqrtf(temp1 + temp2);
}

/**********************************************************************/
/*! \brief POS_ErreurOrientation
 * 
 *  \note ecart d'angle entre l'angle absolu de la droite passant par le robot et le point d'arrivee, et l'orientation du robot
 *
 *  \param [in] poseRobot    pose du robot
 *  \param [in] posArrivee   position d'arrivee
 *
 *  \return Erreur Orientation
 */
/**********************************************************************/
extern float POS_ErreurOrientation(Pose poseRobot, Vecteur posArrivee)
{
    float thetaRobotArrivee;    /* Orientation de la trajectoire directe robot -pt d'arrivee */
    
    thetaRobotArrivee = atan2f((posArrivee.y - poseRobot.y), (posArrivee.x - poseRobot.x));

    /* Workarround library math */
    if ((posArrivee.y - poseRobot.y) < 0.0)
    {
        if (thetaRobotArrivee > 0.0)
        {
            thetaRobotArrivee = - thetaRobotArrivee;
        }
    }
    else
    {
        if (thetaRobotArrivee < 0.0)
        {
            thetaRobotArrivee = - thetaRobotArrivee;
        }
    }
    
    return (POS_ModuloAngle(thetaRobotArrivee - poseRobot.angle));
}

/**********************************************************************/
/*! \brief POS_ConversionVitessesLongRotToConsignesPWMRouesRobotUnicycle
 * 
 *  \note conversion des vitesses longitudinale et de rotation, en tensions de consignes des moteurs gauche et droit
 *
 *  \param [in] vitesseLongitudinale            vitesse longitudinale de consigne en m/s
 *  \param [in] vitesseAngulaireRotation     vitesse de rotation de consigne en rd/s
 *  \param [in] consPWMRoueGauche         tension de consigne du moteur gauche en unite PWM, transmise par pointeur
 *  \param [in] consPWMRoueDroite           tension de consigne du moteur droit en unite PWM, transmise par pointeur
 */
/**********************************************************************/
extern void POS_ConversionVitessesLongRotToConsignesPWMRouesRobotUnicycle(float vitesseLongitudinale, float vitesseAngulaireRotation, unsigned short * consPWMRoueGauche, unsigned short * consPWMRoueDroite)
{
    float			vitRoueGauche, vitRoueDroite;
	unsigned short  vitRoueGpwm, vitRoueDpwm;
		
    vitRoueGauche = (m_sensMarcheMouvement * vitesseLongitudinale) - ((vitesseAngulaireRotation * ECART_ROUE_LIBRE) / 2.0);
    vitRoueDroite = (m_sensMarcheMouvement * vitesseLongitudinale) + ((vitesseAngulaireRotation * ECART_ROUE_LIBRE) / 2.0);

	ASSER_TRAJ_LogAsserValPC("VposG", vitRoueGauche);
    ASSER_TRAJ_LogAsserValPC("VposD", vitRoueDroite);
         		
    vitRoueGpwm = ((unsigned short)(vitRoueGauche / GAIN_STATIQUE_MOTEUR_G)) + OffsetPWM;
    vitRoueDpwm = ((unsigned short)(vitRoueDroite / GAIN_STATIQUE_MOTEUR_D)) + OffsetPWM;

    /* Saturation des consignes PWM de vitesse */
    /* Consigne roue gauche */
    if (vitRoueGpwm < (unsigned short)BORNE_PWM_AR)
    {
        *consPWMRoueGauche = (unsigned short)BORNE_PWM_AR;
    }
    else
    {
        if (vitRoueGpwm > (unsigned short)BORNE_PWM_AV)
        {
            *consPWMRoueGauche = (unsigned short)BORNE_PWM_AV;
        }
        else
        {
            *consPWMRoueGauche = vitRoueGpwm;
        }
    }
    /* Consigne roue droite */
    if (vitRoueDpwm < (unsigned short)BORNE_PWM_AR)
    {
        *consPWMRoueDroite = (unsigned short)BORNE_PWM_AR;
    }
    else
    {
        if (vitRoueDpwm > (unsigned short)BORNE_PWM_AV)
        {
            *consPWMRoueDroite = (unsigned short)BORNE_PWM_AV;
        }
        else
        {
            *consPWMRoueDroite = vitRoueDpwm;
        }
    }
}

/*! @} */

/*! @} */

/*! @} */

/* End of File : position.c */

