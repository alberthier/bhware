/************************************************************************************************/
/*! \brief configPIC18.h
 *
 *  \author Jerome Poncin/Francois Ouvradou/Nicolas Chevillon
 *  \version 1.0.0
 *  \date    01/11/2010
 */
/************************************************************************************************/

#ifndef _configPIC18_H_
#define _configPIC18_H_

/*! \addtogroup Asser
 *  @{
 */

/*! \addtogroup configPIC18
 *  @{
 */

/*! \addtogroup configPIC18_Interface
 *  @{
 */

/* #define */

#define                         NbrMesuresMax           150u   

/* Macro */

#define                         CONVERT_V(x)            (((float)x * 20.0) / 128.0)

/************************************************************************************************/

/* Variables globales */

/** Flag Test mode */ 
extern unsigned long            Test_mode;

/** Tableaux de memorisation des tensions mesurees */
extern unsigned short           TensionPWM_MD_Mesures[NbrMesuresMax];
extern unsigned short           TensionPWM_MG_Mesures[NbrMesuresMax];

/** Tableaux de memorisation des vitesses mesurees */
extern float                    Vitesse_MD_Mesures[NbrMesuresMax];
extern float                    Vitesse_MG_Mesures[NbrMesuresMax];

/** Variables pour la lecture en EEPROM des configuration PIC18 */
extern float                    DonneeKpDroite;
extern float                    DonneeKiDroite;
extern float                    DonneeDRoueDroite;
extern float                    DonneeVmaxDroite;
extern unsigned short           DonneeNbrPasCdrDroite;
extern float                    DonneeKpGauche;
extern float                    DonneeKiGauche;
extern float                    DonneeDRoueGauche;
extern float                    DonneeVmaxGauche;
extern unsigned short           DonneeNbrPasCdrGauche;

/** Tensions des moteurs (consignes reelles PWM) */
extern  unsigned short          TensionPWM_MD;
extern  unsigned short          TensionPWM_MG;

/** Vitesses des moteurs */
extern  float                   Vitesse_MD_PIC_PI;
extern  float                   Vitesse_MG_PIC_PI;

/** Flag de saturation du PI*/
extern unsigned char            SaturationPIDflag;
extern unsigned char            SaturationPIGflag;

/** Tension batteries D */
extern unsigned char            VBatD;
/** Tension batteries G */
extern unsigned char            VBatG;

/*! @} */

/*! @} */

/*! @} */

#endif /* _configPIC18_H_ */

/* End of file : configPIC18.h */

