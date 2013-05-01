/************************************************************************************************/
/*! \brief simuCasser.h
 *
 *  \author Jerome Poncin/Francois Ouvradou/Nicolas Chevillon
 *  \version 1.0.0
 *  \date    26/04/2009
 */
/************************************************************************************************/

#ifndef _SIMUCASSER_H_
#define _SIMUCASSER_H_

/*! \addtogroup Simulation
 *  @{
 */

/*! \addtogroup Asservissement
 *  @{
 */

/*! \addtogroup MoteurEtRedefinitionCommande
 *  @{
 */

#include "asserv_trajectoire.h"

#define TAILLE_TAB_GABARIT_VITESSE      2001

int     SIMU_GetCompteur(void);
void    SIMU_REDEF_ASSER_RecoverNbrPas(float vitesseRoueGauche, float vitesseRoueDroite, signed int* deltaPasGauche, signed int* deltaPasDroite);
float   SIMU_ErreurVitesseConsPWMVToVitMS(unsigned int consPMW, float vitMS, float K);
extern float   SIMU_gain(void);
void    SIMU_SimulationMoteurCC(signed int tensionPWM, float* vitesseMoteur, float constanteTemps, float gainStatique, float coupleResistant);
void    SIMU_InitialisationLogRobot(void);
void    SIMU_LogRobot(void);
void    SIMU_TabGabarit_AddVitesse(float vitesse);
void    SIMU_TabGabarit_AddAcceleration(float acceleration);
void    SIMU_AfficheInfoFinAsser(void);
void    SIMU_CalculPeriodique(void);
int     SIMU_Mouvement(void);
void    SIMU_AsserVitessePI(int moteur_testPI, int profil_testPI, int nb_pts_mesure_testPI);

#ifndef PIC32_BUILD
extern void 			SIMU_InitAsser(void);
extern void             SIMU_SetGainsPI(float KpG, float KiG, float KpD, float KiD);
extern void             SIMU_SetParamMoteur(float m, float R, float f, float Fr, float r, float L, float kc, float kv, float Rred);
extern void             SIMU_SetParamProfilVitesse(float Amax, float Dmax);
extern void             SIMU_SetConfigProfilVitesse(float ratioAcc, float ratioDecc, float ratioAccRot, float ratioDeccRot);
#endif

/*! @} */

/*! @} */

/*! @} */

#endif /* _SIMUCASSER_H_ */

/* End of file : simuCasser.h */
