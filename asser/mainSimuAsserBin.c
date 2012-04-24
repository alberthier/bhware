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

void ASSER_GoTo(PtTraj *p_chemin, unsigned int nbrePtsChemin, unsigned int Mouvement, signed int Marche, float angle_rad)
{
    SIMU_REDEF_ASSER_GoTo(p_chemin, nbrePtsChemin, Mouvement, Marche, angle_rad);
}

int commandMsgTreatment(char *buffer, unsigned char *p_mouvement, char *p_marche, float *p_angle_rad, unsigned int *p_nbPts, PtTraj *p_chemin)
{
    char cmd[25], strMvt[30], strMarche[30], strAngleRad[30], strNbPts[30], strValTemp[30];
    int iVal, shiftChar = 0, msgSize = 0;
    float data[150], valTemp;

//    printf("logStr_msgR: %s\n", buffer);

    /* decryptage du message de l'entree standard */
    sscanf(buffer, "%s %hhu %s %f %u", cmd, p_mouvement, strMarche, p_angle_rad, p_nbPts);
    if (strMarche[0] == '-')
        *p_marche = -1;
    else
        *p_marche = 1;

    sscanf(buffer, "%s %s %s %s %s", cmd, strMvt, strMarche, strAngleRad, strNbPts);
    msgSize = strlen(cmd) + strlen(strMvt) + strlen(strMarche) + strlen(strAngleRad) + strlen(strNbPts) + 5;
    for(iVal = 0; iVal < ((*p_nbPts)*2); iVal++)
    {
        sscanf(&(buffer[msgSize]), "%s", strValTemp);
        //printf("logStr_data: %s\n", strValTemp);
        sscanf(strValTemp, "%f", &valTemp);
        data[iVal] = valTemp;
        //ASSER_TRAJ_LogAsser("init", NBR_ASSER_LOG_VALUE, data[iVal]);
        msgSize = msgSize + strlen(strValTemp) + 1;
    }
    /* msgSize = strlen(cmd) + strlen(strMvt) + strlen(strMarche) + strlen(strNbPts) + 4 + shiftChar; */
    /* log de la commande recue en ecrivant la commande de deplacement */
    //printf("msg(%s): %s, %u, %d, %u:\n", buffer, cmd, *p_mouvement, *p_marche, *p_nbPts);
    for(iVal = 0; iVal < *p_nbPts; iVal++)
    {
        p_chemin[iVal].x = CONVERT_FLOAT2SHORT_DISTANCE(data[iVal*2]);
        //ASSER_TRAJ_LogAsser("x_traj", NBR_ASSER_LOG_VALUE, CONVERT_DISTANCE(p_chemin[iVal].x));
        p_chemin[iVal].y = CONVERT_FLOAT2SHORT_DISTANCE(data[iVal*2 + 1]);
        //ASSER_TRAJ_LogAsser("y_traj", NBR_ASSER_LOG_VALUE, CONVERT_DISTANCE(p_chemin[iVal].y));
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
    char buffer[10000], command[25]="";
    PtTraj ptTraj_RobotInitial;
    Pose poseInitiale;
    PtTraj chemin[62];
    unsigned char mouvement, mvtNull;
    char marche, marcheNull;
    float angle_rad;
    unsigned int nbrPtsChemin, nbrPtsNull, nbrParameters, iParam;
    float angleArrivee;
    Parameter paramPI[2], paramK[3], paramR[2], paramT[8], paramConfAsser[2], paramMotor[9];

    printf("asserSimulator: Demarrage simulateur\n");

    /* Config des parametres par défaut  */
    // param des PI # 1:Kp, 2:Ki
    SIMU_SetGainsPI(2.0, 12.0);
    // param des gains asser haut niveau
    gainDeplacement1 = 20.0;
    gainDeplacement2 = 50.0;
    gainDeplacement3 = 20.0;
    // param des gains asser rotation
    gainRotation1 = -6.0;
    gainRotation2 = -6.0;
    // param du profil de vitesse # 1:A_MAX, 2:D_MAX, 3:COEFF_VI1, 4:VITESSE_SEUIL_DECC, 5:COEFF_DECC_FINALE, 6:DECC_MIN, 7:Umax, 8:facteurVitesseAngulaireMax
    SIMU_SetParamProfilVitesse(3.0, -1.5, 0.95, 0.15, 0.08, -0.3, 900.0, 0.5);
    // param du moteur de deplacement # 1:MASSE, 2:RAYON_ROUE, 3:FROTTEMENT_FLUIDE, 4:FORCE_RESISTANTE, 5:RESISTANCE_INDUIT, 6:INDUCTANCE_INDUIT, 7:CONSTANTE_COUPLE, 8:CONSTANTE_VITESSE, 9:RAPPORT_REDUCTION
    SIMU_SetParamMoteur(1.2, 0.03, 0.0000504, 0.4, 2.18, 0.00024, 0.0234, 0.02346, 20.0);


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
            commandMsgTreatment(buffer, &mouvement, &marche, &angle_rad, &nbrPtsChemin, &ptTraj_RobotInitial);
            /* Initialisation de la pose du robot */
            poseInitiale.x = CONVERT_DISTANCE(ptTraj_RobotInitial.x);
            poseInitiale.y = CONVERT_DISTANCE(ptTraj_RobotInitial.y);
            poseInitiale.angle = angle_rad;
            POS_InitialisationPoseRobot(poseInitiale);
            ASSER_TRAJ_InitialisationGenerale();
            SIMU_InitialisationLogRobot();
            printf("asserSimulator: Initialisation effectuee\n");
            fflush(stdout);
        }
        else if (strcmp(command, "MSG_MAIN_GOTO") == 0)
        {
            commandMsgTreatment(buffer, &mouvement, &marche, &angle_rad, &nbrPtsChemin, chemin);

            SIMU_InitialisationLogRobot();
            /* initialisation des données pour l'ordre de déplacement */
            /* lancement du deplacement */
            ASSER_GoTo(chemin, nbrPtsChemin, mouvement, marche, angle_rad);

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
            parameterMsgTreatment(buffer, &nbrParameters, paramT, 8);
            if (nbrParameters == 8)
            {
                A_MAX = paramT[0].value;
                D_MAX = paramT[1].value;
                COEFF_VI1 = paramT[2].value;
                VITESSE_SEUIL_DECC = paramT[3].value;
                COEFF_DECC_FINALE = paramT[4].value;
                DECC_MIN = paramT[5].value;
                Umax = (unsigned int)paramT[6].value;
                facteurVitesseAngulaireMax = paramT[7].value;
            }
        }
        else if (strcmp(command, "CONFIG_ASSER") == 0)
        {
            parameterMsgTreatment(buffer, &nbrParameters, paramConfAsser, 2);
            if (nbrParameters == 2)
            {
                SIMU_SetConfigGeneraleProfilVitesse(paramConfAsser[0].value, paramConfAsser[1].value);
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
