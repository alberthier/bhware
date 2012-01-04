#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//#define TEMPS_SIMULATION 60 /* secondes */

#include "position.h"
#include "asserv_trajectoire.h"
#include "define.h"
//#include "position.h"
//#include "asserv_trajectoire.h"
#include "task_asser.h"
#include "simuAsser.h"
//#include "task_main.h"


typedef struct
{
    char name[30];
    float value;
} Parameter;

void ASSER_GoTo(PtTraj *p_chemin, unsigned int nbrePtsChemin, unsigned int Mouvement, signed int Marche)
{
    SIMU_REDEF_ASSER_GoTo(p_chemin, nbrePtsChemin, Mouvement, Marche);
}

int commandMsgTreatment(char *buffer, unsigned char *p_mouvement, char *p_marche, unsigned int *p_nbPts, PtTraj *p_chemin)
{
    char cmd[25], strMvt[30], strMarche[30], strNbPts[30], strValTemp[30];
    int iVal, shiftChar = 0, msgSize = 0;
    float data[150], valTemp;

//    printf("logStr_msgR: %s\n", buffer);

    /* decryptage du message de l'entree standard */
    sscanf(buffer, "%s %hhu %s %u", cmd, p_mouvement, strMarche, p_nbPts);
    if (strMarche[0] == '-')
        *p_marche = -1;
    else
        *p_marche = 1;

    sscanf(buffer, "%s %s %s %s", cmd, strMvt, strMarche, strNbPts);
    msgSize = strlen(cmd) + strlen(strMvt) + strlen(strMarche) + strlen(strNbPts) + 4;
    for(iVal = 0; iVal < (*p_nbPts*4); iVal++)
    {
        sscanf(&(buffer[msgSize]), "%s", strValTemp);
        //printf("logStr_data: %s\n", strValTemp);
        sscanf(strValTemp, "%f", &valTemp);
        data[iVal] = valTemp;
//        ASSER_TRAJ_LogAsser("data", valTemp);
        msgSize = msgSize + strlen(strValTemp) + 1;
    }
    /* msgSize = strlen(cmd) + strlen(strMvt) + strlen(strMarche) + strlen(strNbPts) + 4 + shiftChar; */
    /* log de la commande recue en ecrivant la commande de deplacement */
    //printf("msg(%s): %s, %u, %d, %u:\n", buffer, cmd, *p_mouvement, *p_marche, *p_nbPts);
    for(iVal = 0; iVal < *p_nbPts; iVal++)
    {
        p_chemin[iVal].pose.x = data[iVal*4];
//        ASSER_TRAJ_LogAsser("x_traj", p_chemin[iVal].pose.x);
        p_chemin[iVal].pose.y = data[iVal*4 + 1];
//        ASSER_TRAJ_LogAsser("y_traj", p_chemin[iVal].pose.y);
        p_chemin[iVal].pose.angle = data[iVal*4 + 2];
//        ASSER_TRAJ_LogAsser("angle_traj", p_chemin[iVal].pose.angle);
        p_chemin[iVal].mask = (unsigned char)data[iVal*4 + 3];
//        ASSER_TRAJ_LogAsser("mask_traj", p_chemin[iVal].mask);
        //printf("%d: x:%f, y:%f, angle:%f\n", iVal, p_chemin[iVal].x, p_chemin[iVal].y, p_chemin[iVal].angle);
    }

    return msgSize;
}

int parameterMsgTreatment(char *buffer, unsigned int *p_nbParam, Parameter *p_param, unsigned int nbMaxParam)
{
    char cmd[25], strNbParam[30], paramName[30], strValTemp[30];
    int iVal, shiftChar = 0, msgSize;
    float data[100], valTemp;

    /* decryptage du message de l'entree standard */
    sscanf(buffer, "%s %u", cmd, p_nbParam);
    sscanf(buffer, "%s %s", cmd, strNbParam);
    shiftChar = strlen(cmd) + strlen(strNbParam) + 2;

    for(iVal = 0; (iVal < *p_nbParam) && (iVal < nbMaxParam); iVal++)
    {
        //msgSize = strlen(cmd) + strlen(strMvt) + strlen(strMarche) + strlen(strNbPts) + 4 + shiftChar;
        sscanf(&(buffer[shiftChar]), "%s %s", p_param[iVal].name, strValTemp);
        sscanf(strValTemp, "%f", &p_param[iVal].value);
        shiftChar = shiftChar + strlen(p_param[iVal].name) + strlen(strValTemp) + 2;
    }

    return shiftChar;
}

int main(void)
{
    char buffer[255], command[25]="";
    PtTraj ptRobotInitial;
    PtTraj chemin[30];
    unsigned char mouvement, mvtNull;
    char marche, marcheNull;
    unsigned int nbrPtsChemin, nbrPtsNull, nbrParameters, iParam;
    float angleArrivee;
    Parameter paramPI[2], paramK[3], paramR[2], paramT[3], paramMotor[9];

    printf("asserSimulator: Demarrage simulateur\n");
    fflush(stdout);

    while (strcmp(command, "QUIT") != 0)
    {
        /* lecture de l'entree standard */
        fgets(buffer, 255, stdin);

        /* extraction du type de commande */
        sscanf(buffer, "%s", command);
        printf("asserSimulator: Command %s\n", command);
        fflush(stdout);

        if (strcmp(command, "INIT_POSE_ROBOT") == 0)
        {
            commandMsgTreatment(buffer, &mouvement, &marche, &nbrPtsChemin, &ptRobotInitial);
            /* Initialisation de la pose du robot */
            POS_InitialisationPoseRobot(ptRobotInitial.pose);
            ASSER_TRAJ_InitialisationGenerale();
            SIMU_InitialisationLogRobot();
            printf("asserSimulator: Initialisation effectuee\n");
            fflush(stdout);
        }
        else if (strcmp(command, "MSG_MAIN_GOTO") == 0)
        {
            commandMsgTreatment(buffer, &mouvement, &marche, &nbrPtsChemin, chemin);

            SIMU_InitialisationLogRobot();
            /* initialisation des données pour l'ordre de déplacement */
            //angleArrivee = chemin[nbrPtsChemin-1].pose.angle;
            /* lancement du deplacement */
            ASSER_GoTo(chemin, nbrPtsChemin, mouvement, marche);

            // execution du deplacement
            SIMU_Mouvement();

            //printf("SimuC: Deplacement termine.\n");
        }
        else if (strcmp(command, "MSG_TEST_PI") == 0)
        {
            SIMU_InitialisationLogRobot();
            SIMU_AsserVitessePI();
        }
        else if (strcmp(command, "PARAMETERS_PI") == 0)
        {
            parameterMsgTreatment(buffer, &nbrParameters, paramPI, 2);
            if (nbrParameters == 2)
            {
                SIMU_SetGainsPI(paramPI[0].value, paramPI[1].value);
            }

        }
        else if (strcmp(command, "PARAMETERS_GAIN_K") == 0)
        {
            parameterMsgTreatment(buffer, &nbrParameters, paramK, 3);
            if (nbrParameters == 3)
            {
                gainDeplacement1 = paramK[0].value;
                gainDeplacement2 = paramK[1].value;
                gainDeplacement3 = paramK[2].value;
            }

        }
        else if (strcmp(command, "PARAMETERS_GAIN_ROT") == 0)
        {
            parameterMsgTreatment(buffer, &nbrParameters, paramR, 2);
            if (nbrParameters == 2)
            {
                gainRotation1 = paramR[0].value;
                gainRotation2 = paramR[1].value;
            }
        }
        else if (strcmp(command, "PARAMETERS_TIME") == 0)
        {
            parameterMsgTreatment(buffer, &nbrParameters, paramT, 3);
            if (nbrParameters == 3)
            {
                tempsAcc = paramT[0].value;
                facteurVitesseAngulaireMax = paramT[1].value;
                Umax = (unsigned int)paramT[2].value;
            }
        }
        else if (strcmp(command, "PARAMETERS_MOTOR") == 0)
        {
            parameterMsgTreatment(buffer, &nbrParameters, paramMotor, 9);
            if (nbrParameters == 9)
            {
                SIMU_SetParamMoteur(paramMotor[0].value
                                    , paramMotor[1].value
                                    , paramMotor[2].value
                                    , paramMotor[3].value
                                    , paramMotor[4].value
                                    , paramMotor[5].value
                                    , paramMotor[6].value
                                    , paramMotor[7].value
                                    , paramMotor[8].value);
            }

        }
        else
        {
            /* reception de la commande 'QUIT' ou d'une commande inconnue */
            strcpy(command, "QUIT");
        }
    }

    printf("asserSimulator: Simulateur arrete.\n");
    fflush(stdout);
    return(0);
}
