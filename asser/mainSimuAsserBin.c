#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

//#define TEMPS_SIMULATION 60 /* secondes */

#include "position.h"
#include "asserv_trajectoire.h"
#include "define.h"
//#include "asserv_trajectoire.h"
#include "task_asser.h"
#include "simuAsser.h"
//#include "task_main.h"


typedef struct
{
    char name[30];
    float value;
} Parameter;

void ASSER_GoTo(unsigned char Mouvement, Data_Goto * Data)
{
    SIMU_REDEF_ASSER_GoTo(Mouvement, Data);
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
        //ASSER_TRAJ_LogAsserValPC("init", data[iVal]);
        msgSize = msgSize + strlen(strValTemp) + 1;
    }
    /* msgSize = strlen(cmd) + strlen(strMvt) + strlen(strMarche) + strlen(strNbPts) + 4 + shiftChar; */
    /* log de la commande recue en ecrivant la commande de deplacement */
    //printf("msg(%s): %s, %u, %d, %u:\n", buffer, cmd, *p_mouvement, *p_marche, *p_nbPts);
    for(iVal = 0; iVal < *p_nbPts; iVal++)
    {
        p_chemin[iVal].x = CONVERT_FLOAT2SHORT_DISTANCE(data[iVal*2]);
        //ASSER_TRAJ_LogAsserValPC("x_traj", CONVERT_DISTANCE(p_chemin[iVal].x));
        p_chemin[iVal].y = CONVERT_FLOAT2SHORT_DISTANCE(data[iVal*2 + 1]);
        //ASSER_TRAJ_LogAsserValPC("y_traj", CONVERT_DISTANCE(p_chemin[iVal].y));
    }

    return msgSize;
}

int depCommandMsgTreatment(char *buffer, unsigned char * p_mouvement, Data_Goto * Data)
{
    char msg_mvt[25], strMvt[30], strUseAngle[30], strMarche[30], strXCentre[30], strYCentre[30], strRayon[30], strAngleRad[30], strNbPts[30], strValTemp[30];
    unsigned char use_angle;
    char marche;
    float angle_rad;
    unsigned int nbPts;
    int iVal, iAngle, shiftChar = 0, msgSize = 0;
    float data[150], valTemp;
    float xCentre, yCentre, rayon;
    PtTraj Centre;

    /* decryptage du message de l'entree standard */
    sscanf(buffer, "%s", msg_mvt);

    if (strcmp(msg_mvt, "MSG_ROTATE") == 0)
    {
        *p_mouvement = ROTATE;
    }
    else if (strcmp(msg_mvt, "MSG_MOVE_CURVE") == 0)
    {
        *p_mouvement = MOVE_CURVE;
    }
    else if (strcmp(msg_mvt, "MSG_MOVE_LINE") == 0)
    {
        *p_mouvement = MOVE_LINE;
    }
    else if (strcmp(msg_mvt, "MSG_MOVE_ARC") == 0)
    {
        *p_mouvement = MOVE_ARC;
    }

    switch(*p_mouvement)
    {
        case ROTATE :
            /* decryptage du message de l'entree standard */
            sscanf(buffer, "%s %s %f", msg_mvt, strMarche, &angle_rad);
            if (strMarche[0] == '-')
                marche = -1;
            else
                marche = 1;

            Data->rotate.Marche = marche;
            Data->rotate.Angle = angle_rad;

            sscanf(buffer, "%s %s %s", msg_mvt, strMarche, strAngleRad);
            msgSize = strlen(msg_mvt) + strlen(strMarche) + strlen(strAngleRad) + 3;

            break;

        case MOVE_CURVE :
            /* decryptage du message de l'entree standard */
            sscanf(buffer, "%s %s %hhu %f %u", msg_mvt, strMarche, &use_angle, &angle_rad, &nbPts);
            if (strMarche[0] == '-')
                marche = -1;
            else
                marche = 1;

            Data->curve.Marche = marche;
            Data->curve.Use_Angle = use_angle;
            Data->curve.Angle = angle_rad;
            Data->curve.NbrePtsChemin = nbPts;

            sscanf(buffer, "%s %s %s %s %s", msg_mvt, strMarche, strUseAngle, strAngleRad, strNbPts);
            msgSize = strlen(msg_mvt) + strlen(strMarche) + strlen(strUseAngle) + strlen(strAngleRad) + strlen(strNbPts) + 5;
            for(iVal = 0; iVal < ((nbPts)*2); iVal++)
            {
                sscanf(&(buffer[msgSize]), "%s", strValTemp);
                sscanf(strValTemp, "%f", &valTemp);
                data[iVal] = valTemp;
                msgSize = msgSize + strlen(strValTemp) + 1;
            }
            for(iVal = 0; iVal < nbPts; iVal++)
            {
                Data->curve.Chemin[iVal].x = CONVERT_FLOAT2SHORT_DISTANCE(data[iVal*2]);
                Data->curve.Chemin[iVal].y = CONVERT_FLOAT2SHORT_DISTANCE(data[iVal*2 + 1]);
            }

            break;

        case MOVE_LINE :
            /* decryptage du message de l'entree standard */
            sscanf(buffer, "%s %s %u", msg_mvt, strMarche, &nbPts);
            if (strMarche[0] == '-')
                marche = -1;
            else
                marche = 1;

            Data->line.Marche = marche;
            Data->line.NbrePtsChemin = nbPts;

            sscanf(buffer, "%s %s %s", msg_mvt, strMarche, strNbPts);
            msgSize = strlen(msg_mvt) + strlen(strMarche) + strlen(strNbPts) + 3;
            for(iVal = 0; iVal < ((nbPts)*2); iVal++)
            {
                sscanf(&(buffer[msgSize]), "%s", strValTemp);
                sscanf(strValTemp, "%f", &valTemp);
                data[iVal] = valTemp;
                msgSize = msgSize + strlen(strValTemp) + 1;
            }
            for(iVal = 0; iVal < nbPts; iVal++)
            {
                Data->line.Chemin[iVal].x = CONVERT_FLOAT2SHORT_DISTANCE(data[iVal*2]);
                Data->line.Chemin[iVal].y = CONVERT_FLOAT2SHORT_DISTANCE(data[iVal*2 + 1]);
            }

            break;

        case MOVE_ARC :
            /* decryptage du message de l'entree standard */
            sscanf(buffer, "%s %s %f %f %f %u", msg_mvt, strMarche, &xCentre, &yCentre, &rayon, &nbPts);
            if (strMarche[0] == '-')
                marche = -1;
            else
                marche = 1;

            Data->arc.Marche = marche;
            Centre.x = CONVERT_FLOAT2SHORT_DISTANCE(xCentre);
            Centre.y = CONVERT_FLOAT2SHORT_DISTANCE(yCentre);
            Data->arc.Centre_rotation = Centre;
            Data->arc.Rayon = rayon;
            Data->arc.NbrePtsChemin = nbPts;

            sscanf(buffer, "%s %s %s %s %s %s", msg_mvt, strMarche, strXCentre, strYCentre, strRayon, strNbPts);
            msgSize = strlen(msg_mvt) + strlen(strMarche) + strlen(strXCentre) + strlen(strYCentre) + strlen(strRayon) + strlen(strNbPts) + 6;
            for(iAngle = 0; iAngle < nbPts; iAngle++)
            {
                sscanf(&(buffer[msgSize]), "%s", strValTemp);
                sscanf(strValTemp, "%f", &valTemp);
                Data->arc.Chemin[iAngle] = (float)valTemp;
                msgSize = msgSize + strlen(strValTemp) + 1;

                ASSER_TRAJ_LogAsserValPC("chemin_arc", (float)valTemp);
            }

            ASSER_TRAJ_LogAsserMsgPC(buffer, 0.0);

            break;
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
    Data_Goto Data_deplacement;
    short iPt, iAngle;
    clock_t temps_i, temps_f, temps_f2;

    printf("asserSimulator: Demarrage simulateur\n");

    /* Config des parametres par défaut  */
    // param des PI # 1:Kp, 2:Ki
    SIMU_SetGainsPI(2.0, 6.0);
    // param des gains asser haut niveau
    gainDeplacement1 = 20.0;
    gainDeplacement2 = 50.0;
    gainDeplacement3 = 50.0;
    // param des gains asser rotation
    gainCentreRot = 20.0;
    // param du profil de vitesse # 1:A_MAX, 2:D_MAX
    SIMU_SetParamProfilVitesse(1.0, 1.0);
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
        else if ((strcmp(command, "MSG_ROTATE") == 0)
                 | (strcmp(command, "MSG_MOVE_CURVE") == 0)
                 | (strcmp(command, "MSG_MOVE_LINE") == 0)
                 | (strcmp(command, "MSG_MOVE_ARC") == 0) )
        {
            depCommandMsgTreatment(buffer, &mouvement, &Data_deplacement);

#ifdef PLOTS_SIMU
            SIMU_InitialisationLogRobot();
#endif /* PLOTS_SIMU */

            /* initialisation des données pour l'ordre de déplacement */
            /* lancement du deplacement */
            temps_i = clock();
            ASSER_GoTo(mouvement, &Data_deplacement);
            temps_f = clock();
            ASSER_TRAJ_LogAsserValPC("temps_init", ((float)(temps_f - temps_i)) / CLOCKS_PER_SEC);
            // execution du deplacement
            SIMU_Mouvement();  /* TOCHANGE */
            temps_f2 = clock();
            ASSER_TRAJ_LogAsserValPC("temps_init", ((float)(temps_f2 - temps_f)) / CLOCKS_PER_SEC);
        }
        else if (strcmp(command, "MSG_TEST_PI") == 0)
        {
            SIMU_InitialisationLogRobot();
            //SIMU_AsserVitessePI();
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
            if (nbrParameters == 1)
            {
                gainCentreRot = paramR[0].value;
            }
        }
        else if (strcmp(command, "PARAMETERS_TIME") == 0)
        {
            parameterMsgTreatment(buffer, &nbrParameters, paramT, 2);
            if (nbrParameters == 2)
            {
                SIMU_SetParamProfilVitesse(paramT[0].value, D_MAX = paramT[1].value);   
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
