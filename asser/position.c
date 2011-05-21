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

#define                 GAIN_STATIQUE_MOTEUR    (((DonneeVmaxDroite + DonneeVmaxGauche) / 2.0) / 1023.0)   /* Gain statique des moteurs CC du deplacement 1 m.s-1 / 1023 */
#define                 PERIMETRE_DROIT         (PI * (DonneeDRoueDroite / 1000))                          /* Perimetre en m de la roue libre du codeur droit */
#define                 PERIMETRE_GAUCHE        (PI * (DonneeDRoueGauche / 1000))                          /* Perimetre en m de la roue libre du codeur gauche */
    
const   float           TE                      = 0.020;                                                    /* Periode de l'asservissement en seconde */

/**********************************************************************/

/* Variables globales */
    
float                   ECART_ROUE_LIBRE        = 0.1704;                                                   /* Ecart entre les roues libres des codeurs incrementaux */
float                   ECART_ROUE_MOTRICE      = 0.339;                                                    /* Entraxe des roues motrices */
float                   COEFFICIENT_DE_GLISSEMENT_LATERAL = 0.0;

/** Tolerances de la condition d'arret des asservissements */
float                   DIST_MIN                = 0.005;
float                   ANGLE_MIN               = 0.080;

/** Coordonnees de la pose actuelle du robot */
Pose                    m_poseRobot             = {0.0, 0.0, 0.0};

/** Sens de deplacement du robot */
signed char             m_sensMarcheMouvement   = MARCHE_AVANT;

/** Vitesse maximale a demander aux moteurs */
float                   Umax                    = 900.0;                /* Vitesse maximale a demander aux moteurs, image de la tension (en unite PWM) etant donne le gain statique des moteurs */

/**********************************************************************
 * Definition dedicated to the local functions.
 **********************************************************************/

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
void POS_InitialisationPoseRobot(Pose poseRobot)
{   
#ifdef PIC32_BUILD

#if OS_CRITICAL_METHOD == 3                      /* Allocate storage for CPU status register           */
    OS_CPU_SR  cpu_sr = 0;
#endif 

    OS_ENTER_CRITICAL();


#endif /* PIC32_BUILD */

    ASSER_TRAJ_LogAsser("poseRobotInitX", NBR_ASSER_LOG_VALUE, poseRobot.x);
    ASSER_TRAJ_LogAsser("poseRobotInitY", NBR_ASSER_LOG_VALUE, poseRobot.y);


    m_poseRobot.x = poseRobot.x;
    m_poseRobot.y = poseRobot.y;
    m_poseRobot.angle = poseRobot.angle;

#ifdef PIC32_BUILD

    OS_EXIT_CRITICAL();

#endif /* PIC32_BUILD */
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
void POS_InitialisationSensMarche(signed char marche)
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
Pose POS_GetPoseRobot(void)
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
Pose POS_GetPoseAsserRobot(void)
{
    Pose poseRobot;

    poseRobot.x = m_poseRobot.x;
    poseRobot.y = m_poseRobot.y;
    poseRobot.angle = m_poseRobot.angle;
    
    if (m_sensMarcheMouvement == MARCHE_ARRIERE)
    {
        poseRobot.angle = m_poseRobot.angle + PI;
    }

    return poseRobot;
}

/**********************************************************************/
/*! \brief POS_GetConsVitesseMax
 * 
 *  \note   Calcul et retourne la vitesse maximale de consigne des moteurs de deplacement
 *
 *  \return vitesse en m/s
 */
/**********************************************************************/
float POS_GetConsVitesseMax(void)
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
float POS_GetConsVitesseAngulaireMax(void)
{
    return ((float)((Umax * GAIN_STATIQUE_MOTEUR * 2.0) / ECART_ROUE_MOTRICE));
}

/**********************************************************************/
/*! \brief ASSER_POS_Positionnement
 * 
 *  \note  Calcul de la nouvelle position du robot et des vitesses de ses roues motrices
 *  -> vitesse en unite entiere par seconde
 *
 *  \param [in] delta_impDroite     Delta du nombre d'impulsions roue droite
 *  \param [in] delta_impGauche   Delta du nombre d'impulsions roue gauche 
 *
 *  \return None
 */
/**********************************************************************/
  
void POS_Positionnement(signed int delta_impDroite, signed int delta_impGauche)
{
    float Delta_moy, Delta_diff;
    float Dx, Dy;
    float Dtheta;                   /* Angle de rotation du robot entre deux lecture des capteurs de position */
    float Deviation_x, Deviation_y;

    Delta_moy  = ((((float)delta_impDroite * PERIMETRE_DROIT) + ((float)delta_impGauche * PERIMETRE_GAUCHE)) / (2.0 * (float)NBRE_PAS));
    Delta_diff = ((((float)delta_impDroite * PERIMETRE_DROIT) - ((float)delta_impGauche * PERIMETRE_GAUCHE)) / (float)NBRE_PAS);

    Dx = (Delta_moy * cos(m_poseRobot.angle));
    Dy = (Delta_moy * sin(m_poseRobot.angle));

    Dtheta = (Delta_diff / ECART_ROUE_LIBRE);

    Deviation_x = - COEFFICIENT_DE_GLISSEMENT_LATERAL * Dtheta * Dy;
    Deviation_y = COEFFICIENT_DE_GLISSEMENT_LATERAL * Dtheta * Dx;

    m_poseRobot.x = m_poseRobot.x + Dx + Deviation_x;
    m_poseRobot.y = m_poseRobot.y + Dy + Deviation_y;
    m_poseRobot.angle = POS_ModuloAngle(m_poseRobot.angle + Dtheta);

    if (Test_mode == (unsigned long)1)
    {
        if (TakeMesure == True)
        {
            if (Sample < (unsigned char)5)
            {
                ASSER_TRAJ_LogAsser("Tension PWM MG", (unsigned char)(Sample * 7), (float) TensionPWM_MG);
                ASSER_TRAJ_LogAsser("Tension PWM MD", (unsigned char)((Sample * 7) + 1), (float) TensionPWM_MD);
                ASSER_TRAJ_LogAsser("Vitesse MG", (unsigned char)((Sample * 7) + 2),Vitesse_MG_PIC_PI);            
                ASSER_TRAJ_LogAsser("Vitesse MS", (unsigned char)((Sample * 7) + 3), Vitesse_MG_PIC_PI);
                ASSER_TRAJ_LogAsser("Position X", (unsigned char)((Sample * 7) + 4), m_poseRobot.x);    
                ASSER_TRAJ_LogAsser("Position Y", (unsigned char)((Sample * 7) + 5), m_poseRobot.y);
                ASSER_TRAJ_LogAsser("Position Angle", (unsigned char)((Sample * 7) + 6), m_poseRobot.angle);

                Sample++;
            }

            TakeMesure = False;
        }
        else
        {
            TakeMesure = True;
        }
    }
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
float POS_ModuloAngle(float angle)
{
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
float POS_ErreurDistance(Pose poseRobot, Vecteur posArrivee)
{
    float temp1;
    float temp2;
    
    temp1 = ((poseRobot.x - posArrivee.x) * (poseRobot.x - posArrivee.x));
    temp2 = ((poseRobot.y - posArrivee.y ) * (poseRobot.y - posArrivee.y ));

    return (sqrt(temp1 + temp2));
}

/**********************************************************************/
/*! \brief POS_ErreurOrientation
 * 
 *  \note ecart d'angle entre l'angle absolu de la droite passant par le robot et le point d'arrivee, et l'orientation du robot
 *
 *  \param [in] poseRobot   pose du robot
 *  \param [in] posArrivee  position d'arrivee
 *
 *  \return Erreur Orientation
 */
/**********************************************************************/

float POS_ErreurOrientation(Pose poseRobot, Vecteur posArrivee)
{
    float thetaRobotArrivee;    /* Orientation de la trajectoire directe robot -pt d'arrivee */
    
    thetaRobotArrivee = atan2((posArrivee.y - poseRobot.y), (posArrivee.x - poseRobot.x));
 
    return (POS_ModuloAngle(thetaRobotArrivee - poseRobot.angle));
}

/**********************************************************************/
/*! \brief POS_ConversionVitessesLongRotToConsignesPWMRouesRobotUnicycle
 * 
 *  \note conversion des vitesses longitudinale et de rotation, en tensions de consignes des moteurs gauche et droit
 *
 *  \param [in] vitesseLongitudinale    vitesse longitudinale de consigne en m/s
 *  \param [in] vitesseRotation         vitesse de rotation de consigne en rd/s
 *  \param [in] consPWMRoueGauche       tension de consigne du moteur gauche en unite PWM, transmise par pointeur
 *  \param [in] consPWMRoueDroite       tension de consigne du moteur droit en unite PWM, transmise par pointeur
 */
/**********************************************************************/
void POS_ConversionVitessesLongRotToConsignesPWMRouesRobotUnicycle(float vitesseLongitudinale, float vitesseRotation, unsigned short *consPWMRoueGauche, unsigned short *consPWMRoueDroite)
{
    float vitRoueGauche, vitRoueDroite;

    vitRoueGauche = (m_sensMarcheMouvement * vitesseLongitudinale) - ((vitesseRotation * ECART_ROUE_MOTRICE) / 2.0);
    vitRoueDroite = (m_sensMarcheMouvement * vitesseLongitudinale) + ((vitesseRotation * ECART_ROUE_MOTRICE) / 2.0);
    
    vitRoueGauche = (vitRoueGauche / GAIN_STATIQUE_MOTEUR) + (float)OffsetPWM;
    vitRoueDroite = (vitRoueDroite / GAIN_STATIQUE_MOTEUR) + (float)OffsetPWM;

    /* Saturation des consignes PWM de vitesse */
    /* Consigne roue gauche */
    if (vitRoueGauche < (float)BORNE_PWM_AR)
    {
        *consPWMRoueGauche = (unsigned short)BORNE_PWM_AR;
    }
    else
    {
        if (vitRoueGauche > (float)BORNE_PWM_AV)
        {
            *consPWMRoueGauche = (unsigned short)BORNE_PWM_AV;
        }
        else
        {
            *consPWMRoueGauche = (unsigned short)vitRoueGauche;
        }
    }
    /* Consigne roue droite */
    if (vitRoueDroite < (float)BORNE_PWM_AR)
    {
        *consPWMRoueDroite = (unsigned short)BORNE_PWM_AR;
    }
    else
    {
        if (vitRoueDroite > (float)BORNE_PWM_AV)
        {
            *consPWMRoueDroite = (unsigned short)BORNE_PWM_AV;
        }
        else
        {
            *consPWMRoueDroite = (unsigned short)vitRoueDroite;
        }
    }
}

/*! @} */

/*! @} */

/*! @} */

/* End of File : position.c */
