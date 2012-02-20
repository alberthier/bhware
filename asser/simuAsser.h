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
float   SIMU_ErreurVitesseConsPWMVToVitMS(unsigned int consPMW, float vitMS, float K);
extern float   SIMU_gain(void);
void    SIMU_SimulationMoteurCC(signed int tensionPWM, float* vitesseMoteur, float constanteTemps, float gainStatique, float coupleResistant);
void    SIMU_InitialisationLogRobot(void);
void    SIMU_LogRobot(void);
void    SIMU_AfficheInfoFinAsser(void);
void    SIMU_CalculPeriodique(void);
int     SIMU_Mouvement(void);
int     SIMU_AsserVitessePI(void);

#ifndef PIC32_BUILD
extern void             SIMU_SetGainsPI(float kp, float ki);
extern void             SIMU_SetParamMoteur(float m, float R, float f, float Fr, float r, float L, float kc, float kv, float Rred);
extern void             SIMU_SetParamProfilVitesse(float Amax, float Dmax, float coeff_vi1, float vit_seuil_decc, float coeff_decc_finale, float decc_min, float umax, float fvam);
extern void             SIMU_SetConfigGeneraleProfilVitesse(float Ratio_Acc, float Ratio_Decc);
#endif

/*! @} */

/*! @} */

/*! @} */

#endif /* _SIMUCASSER_H_ */

/* End of file : simuCasser.h */
