/************************************************************************************************/
/*! \brief define.h
 *
 *  \author Jerome Poncin/Francois Ouvradou/Nicolas Chevillon
 *  \version 1.0.0
 *  \date    03/06/2007
 */
/************************************************************************************************/

#ifndef _DEFINE_H
#define _DEFINE_H

#ifdef PIC32_BUILD
#include "tools.h"
#endif /* PIC32_BUILD */

/*! \addtogroup OS
 *  @{
 */

/*! \addtogroup OS_Interface
 *  @{
 */

/***********************************************************************
 * ----------------------------- Events --------------------------------
 **********************************************************************/

/** Debug */
#define DEBUG_EVENT                 (OS_FLAGS) 0x01

/** Main */
#define MAIN_EVENT                  (OS_FLAGS) 0x01
#define MAIN_ASSER_FINISHED         (OS_FLAGS) 0x02
#define MAIN_EVITEMENT              (OS_FLAGS) 0x04
#define MAIN_SEND_KEEP_ALIVE        (OS_FLAGS) 0x08
#define MAIN_START                  (OS_FLAGS) 0x10
#define MAIN_ASSER_STOP             (OS_FLAGS) 0x20

/** Evitement */
#define EVIT_EVENT                  (OS_FLAGS) 0x01
#define EVIT_SENDSTART              (OS_FLAGS) 0x02

/** Asservissement */
#define ASSER_EVENT                 (OS_FLAGS) 0x01
#define ASSER_NBR_PAS               (OS_FLAGS) 0x02
#define ASSER_RUNNING               (OS_FLAGS) 0x04
#define ASSER_CONFIG                (OS_FLAGS) 0x08

/** I/O */
#define IO_EVENT                    (OS_FLAGS) 0x01
#define IO_STOP_ROBOT               (OS_FLAGS) 0x02

/** UART RS232 */
#define UART_EVENT                  (OS_FLAGS) 0x01
#define UART_TX_MSG                 (OS_FLAGS) 0x02
#define UART_RX_MSG1                (OS_FLAGS) 0x04
#define UART_RX_MSG2                (OS_FLAGS) 0x08
#define UART_CONFIG                 (OS_FLAGS) 0x10   

/***********************************************************************
 * ----------------------------- Task ID -------------------------------
 **********************************************************************/

/** Task ID */

#define TASK_HTTP_ID                (INT16U) 9
#define TASK_DEBUG_ID               (INT16U) 8
#define TASK_MAIN_ID                (INT16U) 7
#define TASK_EVIT_ID                (INT16U) 6
#define TASK_IO_ID                  (INT16U) 5
#define TASK_ASSER_ID               (INT16U) 2
#define TASK_UART_ID                (INT16U) 1

/** Task Priority */

#define TASK_HTTP_PRIO               (INT8U) 9
#define TASK_DEBUG_PRIO              (INT8U) 8
#define TASK_MAIN_PRIO               (INT8U) 7
#define TASK_EVIT_PRIO               (INT8U) 6
#define TASK_IO_PRIO                 (INT8U) 5
#define TASK_ASSER_PRIO              (INT8U) 4
#define TASK_UART_PRIO               (INT8U) 1

/***********************************************************************
 * ----------------------------- Defines globaux -----------------------
 **********************************************************************/

/** Defines globaux */

#define True                (unsigned char) 1u
#define False               (unsigned char) 0u

#ifndef NULL
#define NULL                                0
#endif

/* Constantes */

#ifndef PI
#define PI                                  3.14159265358979323846
#endif

/**********************************************************************/

/** Define pour la compilation pour PIC32 */
//#define PIC32_BUILD

/** Define pour la stack TCP IP BSD socket 4.0 */
#define API_00833_SUPPORT

/** Utilisation des DMA pour l'emission UART */
#define UART_DMA_TX

/**********************************************************************/

/** Active le fonctionnement du capteur de couleur */ 
//#define Capteur_Couleur_connected

/** Flag de test des IO */
//#define Test_IO

/**********************************************************************/

/*! @} */

/*! @} */

#endif /* _DEFINE_H */

/* End of File : define.h */
