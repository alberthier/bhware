
/************************************************************************************************/
/*! \brief configPIC18.c
 *
 *  \author Jerome Poncin/Francois Ouvradou/Nicolas Chevillon
 *  \version 1.0.0
 *  \date    01/11/2010
 */
/************************************************************************************************/

/************************************************************************************************
 * donnees de config du PIC18
 ************************************************************************************************/

#include "define.h"
#ifdef PIC32_BUILD
#include "includes.h"
#include "task_uart.h"
#include "position.h"
#include "HTTPpages.h"
#endif /* PIC32_BUILD */
#include "pic18.h"

/*! \addtogroup Asser
 *  @{
 */

/*! \addtogroup configPIC18
 *  @{
 */

/*! \addtogroup configPIC18_core
 *  @{
 */

/** Mode de fonctionnement */
unsigned long                   Test_mode                                   =   (unsigned long)0;

/** Tableaux de memorisation des tensions mesurees */
unsigned short                  TensionPWM_MD_Mesures[NbrMesuresMax]        =   {0};
unsigned short                  TensionPWM_MG_Mesures[NbrMesuresMax]        =   {0};

/** Tableaux de memorisation des vitesses mesurees */
float                           Vitesse_MD_Mesures[NbrMesuresMax]           =   {0};
float                           Vitesse_MG_Mesures[NbrMesuresMax]           =   {0};

/** Variables des configuration PIC18 */
/* Right side */
float                           DonneeKpDroite                              =   3.0;
float                           DonneeKiDroite                              =   11.0;
float                           DonneeDRoueDroite                           =   40.0;   /* en mm */
float                           DonneeVmaxDroite                            =   0.911;  /* en m/s */
unsigned short                  DonneeNbrPasCdrDroite                       =   5000*4;

/* Left side */
float                           DonneeKpGauche                              =   3.0;
float                           DonneeKiGauche                              =   8.0;
float                           DonneeDRoueGauche                           =   40.0;   /* en mm */
float                           DonneeVmaxGauche                            =   0.900;  /* en m/s */
unsigned short                  DonneeNbrPasCdrGauche                       =   5000*4;

/** Tensions des moteurs (consignes reelles PWM) */
unsigned short                  TensionPWM_MD                               =   0x0000;
unsigned short                  TensionPWM_MG                               =   0x0000;

/** Vitesses des moteurs */
float                           Vitesse_MD_PIC_PI                           =   0.0;
float                           Vitesse_MG_PIC_PI                           =   0.0;

/** Flag de saturation du PI D */
unsigned char                   SaturationPIDflag                           =   False;
/** Flag de saturation du PI G */
unsigned char                   SaturationPIGflag                           =   False;

/** Tension batteries D */
unsigned char                   VBatD                                       =   0;
/** Tension batteries G */
unsigned char                   VBatG                                       =   0;

/*! @} */

/*! @} */

/*! @} */

/* End of File : configPIC18.c */

