#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "position.h"

int main(void)
{
    char buffer[255], cmd[15], strMvt[4], strMarche[5], strNbPts[6], strValTemp[10];
    unsigned char mouvement;
    char marche;
    unsigned int nbPts;
    float data[100], valTemp;
    int iVal, shiftChar = 0;
    //tableau de la trajectoire
    Pose chemin[10];
    
    /* lecture de l'entree standard */
    gets(buffer);
    /* decryptage du message de l'entree standard */
    sscanf(buffer, "%s %hhu %s %u", cmd, &mouvement, strMarche, &nbPts);
    if (strMarche[0] == '-')
        marche = -1;
    else
        marche = 1;

    sscanf(buffer, "%s %s %s %s", cmd, strMvt, strMarche, strNbPts);
    for(iVal = 0; iVal < (nbPts*3); iVal++)
    {
        sscanf(&(buffer[strlen(cmd) + strlen(strMvt) + strlen(strMarche) + strlen(strNbPts) + 4 + shiftChar]), "%s", strValTemp);
        sscanf(strValTemp, "%f", &valTemp);
        data[iVal] = valTemp;
        shiftChar = shiftChar + strlen(strValTemp) + 1;
    }
    /* log de la commande recue en ecrivant la commande de deplacement */
    printf("msg: %s, %u, %d, %u:\n", cmd, mouvement, marche, nbPts);
    for(iVal = 0; iVal < nbPts; iVal++)
    {
        chemin[iVal].x = data[iVal*3];
        chemin[iVal].y = data[iVal*3 + 1];
        chemin[iVal].angle = data[iVal*3 + 2];
        printf("%d: x:%f, y:%f, angle:%f\n", iVal, chemin[iVal].x, chemin[iVal].y, chemin[iVal].angle);
    }

    

    return 0;
}