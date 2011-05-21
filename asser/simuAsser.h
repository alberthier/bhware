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

int     SIMU_GetCompteur(void);
void    SIMU_REDEF_ASSER_RecoverNbrPas(float vitesseRoueGauche, float vitesseRoueDroite, signed int* deltaPasGauche, signed int* deltaPasDroite);
float   SIMU_ErreurVitesseConsPWMVToVitMS(unsigned int consPMW, float vitMS);
void    SIMU_SimulationMoteurCC(signed int tensionPWM, float* vitesseMoteur, float constanteTemps, float gainStatique, float coupleResistant);
void    SIMU_InitialisationLogRobot(void);
void    SIMU_LogRobot(void);
void    SIMU_AfficheInfoFinAsser(void);
void    SIMU_CalculPeriodique(void);
int     SIMU_Mouvement(void);
int     SIMU_AsserVitessePI(void);

#ifndef PIC32_BUILD
extern void             SIMU_SetGainsPI(float kp, float ki);
#endif

/*! @} */

/*! @} */

/*! @} */

#endif /* _SIMUCASSER_H_ */

/* End of file : simuCasser.h */
