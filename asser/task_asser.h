/************************************************************************************************/
/*! \brief task_asser.h
 *
 *  \author Jerome Poncin/Francois Ouvradou/Nicolas Chevillon
 *  \version 1.0.0
 *  \date    03/06/2007
 */
/************************************************************************************************/

#ifndef _TASK_ASSER_H_
#define _TASK_ASSER_H_

#include "position.h"

/*! \addtogroup Task_Asser
 *  @{
 */

/*! \addtogroup Task_Asser_Interface
 *  @{
 */

/* Define Asser */

/** Taille de la Stack Asser */
#define                 STACK_ASSER_SIZE            1024L

/** Variables globales asservissement */

/** Flag de boucle d'asservissement */
extern unsigned char    ASSER_Running;
extern unsigned char    ASSER_FragmentTraj;

#ifdef PIC32_BUILD

/** Stack Asser */
extern OS_STK           ASSER_TaskStk[STACK_ASSER_SIZE];

/** Groupe d'evenement recupere et traite dans la tache Asser */
extern OS_FLAG_GRP *    ASSER_event_gpr;

/** Flags d'evenement recupere et traite dans la tache Asser */
extern OS_FLAGS         ASSER_event; 

/** Variables du nombre de pas codeur mesures */
extern unsigned long    NbrPasMesureRoueD;
extern unsigned long    NbrPasMesureRoueG;

#else /* PIC32_BUILD */

extern unsigned short   simu_consigneMoteurG;
extern unsigned short   simu_consigneMoteurD;

#endif /* PIC32_BUILD */

/* Prototypes de fonctions asservissement */

#ifdef PIC32_BUILD

extern void             TASK_ASSER(void *pdata);
extern void             ASSER_GoTo(PtTraj *p_chemin, unsigned int nbrePtsChemin, unsigned char Mouvement, signed char Marche, float Angle);
extern void             ASSER_SendConsigne(unsigned short ConsigneMoteurD, unsigned short ConsigneMoteurG);

#else /* PIC32_BUILD */

extern void             SIMU_REDEF_ASSER_GoTo(PtTraj *p_chemin, unsigned int nbrePtsChemin, unsigned int Mouvement, signed int Marche, float angle_rad);
extern void             SIMU_REDEF_ASSER_SendConsigne(unsigned short ConsigneMoteurD, unsigned short ConsigneMoteurG);
extern float            SIMU_RegulateurPI_BHT(float erreurVitesse, float* integ, float Ki, float Kp, float vmax, float Te);
extern void             logAsser0(char* nomFichier, char* mode, float valeur, char* chaine); /* le prototype a utiliser reste logAsser(...) (logAsser0() <- astuce pour la simu) */

#endif /* PIC32_BUILD */

/*! @} */

/*! @} */

#endif /* _TASK_ASSER_H_ */

/* End of file : task_asser.h */

