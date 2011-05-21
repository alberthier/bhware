#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define TEMPS_SIMULATION 60 /* secondes */

#include "position.h"

#ifdef ASSER_POS
    #include "asserv_position-2.h"
#endif
#ifdef ASSER_TRAJ
    #include "asserv_trajectoire.h"
#endif
#include "task_asser.h"
#include "define.h"
#include "simuAsser.h"
//#include "task_main.h"

float   ASSER_FragmentTraj = False;

void ASSER_GoTo(Pose *p_chemin, unsigned int nbrePtsChemin, float angleArrivee, unsigned int Mouvement, signed int Marche)
{
    SIMU_REDEF_ASSER_GoTo(p_chemin, nbrePtsChemin, angleArrivee, Mouvement, Marche);
    SIMU_Mouvement();
}

int main(void)
{
    Pose poseRobot;
    Pose chemin[10];
    unsigned int nbrPtsChemin;
    float angleArrivee;

    /* Initialisation de la pose du robot */
    poseRobot.x = 0.25;
    poseRobot.y = 0.25;
    poseRobot.angle = 0.0*PI/1.0;
    POS_InitialisationPoseRobot(poseRobot);

    ASSER_TRAJ_InitialisationGenerale();

    chemin[0].x = 0.5;
    chemin[0].y = 0.25;
    chemin[1].x = 0.75;
    chemin[1].y = 0.25;
    chemin[2].x = 1.0;
    chemin[2].y = 0.25;
    chemin[3].x = 1.25;
    chemin[3].y = 0.3;
    chemin[4].x = 1.5;
    chemin[4].y = 0.75;
    chemin[5].x = 1.25;
    chemin[5].y = 1.1;
    chemin[6].x = 1.0;
    chemin[6].y = 1.2;
    chemin[7].x = 0.75;
    chemin[7].y = 1.2;
    chemin[8].x = 0.6;
    chemin[8].y = 1.2;
    chemin[9].x = 0.4;
    chemin[9].y = 1.2;
    angleArrivee = (-1.0*PI)/1.0;
    nbrPtsChemin = 10;

    ASSER_GoTo(chemin, nbrPtsChemin, angleArrivee , DEPLACEMENT, MARCHE_AVANT);

    SIMU_InitialisationLogRobot();
    ASSER_TRAJ_InitialisationLogAsser();

    // 1 deplacement
    SIMU_Mouvement();

    printf("Print Ok");

    return(0);
}
