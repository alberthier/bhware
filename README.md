# BH Ware

Ce repository contient une partie du code utilisé dans le robot de la (BH Team)[http://bhteam.org] participant à la (Coupe de France de robotique)[http://www.planete-sciences.org/robot/index.php?section=pages&pageid=79]. Cette compétition a pour but de concevoir un robot totalement autonome qui devra concourir face à un adversaire afin de marquer un maximum de points en moins de 90 secondes. Le thème de la compétition change chaque année, (en 2012 il s'agira d'une chasse au trésor)[http://www.planete-sciences.org/robot/index.php?section=pages&pageid=108].

Nous mettons ici à disposition le code de ce que nous appelons l'informatique de haut niveau, c'est à dire le pilotage et la stratégie du robot. Ce code est bien évidement spécifique à notre architecture matérielle, cependant certains concepts peuvent être réutilisés ou servir d'exemple à d'autres réalisations robotiques ou non.

## Matériel

Notre robot est principalement composé de deux cartes électroniques: la première à base de (PIC32)[http://www.microchip.com/en_US/family/32bit/] est chargée de piloter les actionneurs (moteurs) et de lire les informations fournies par les capteurs. La seconde carte est un (Seagate Dockstar)[http://forum.hardware.fr/hfr/OSAlternatifs/Hardware-2/seagate-dockstar-computer-sujet_71314_1.htm] c'est un produit qui est à l'origine destiné à servir de dock réseau pour disque dur. Il s'agit d'un Marvell Kirkwood à 1,2 GHz (ARM9) avec 128 Mo de RAM et 256 Mo de flash.

Ces deux cartes communiquent à l'aide d'une connexion ethernet. L'ARM est responsable de la stratégie et envoie ses requêtes au PIC32 qui est chargé de piloter le robot en conséquence.
Ceci nous permet de développer la stratégie à Strasbourg et l'informatique bas niveau à Lyon en n'ayant qu'un protocole simple entre les deux entités.

## Logiciel

Le code présenté ici concerne la stratégie embarquée sur la carte ARM.
Nous avons choisi de développer un maximum de code en (Python)[http://www.python.org] principalement parce que nous sommes très à l'aise avec ce langage et qu'il permet de développer très rapidement.
Pour les outils graphiques, nous utilisons le (framework Qt)[http://http://qt-project.org//] à travers les (bindings PyQt)[http://www.riverbankcomputing.co.uk/software/pyqt/intro].
Nous faisons l'hypothèse que ces programmes seront exécutés sur un système Unix (Linux ou Mac OS X). Ils fonctionnent mal ou ne fonctionnent pas sous Windows bien qu'il soit tout à fait possible de les adapter pour ce système.

### (Brewery)[https://github.com/alberthier/bhware/tree/master/bhbot/brewery]

C'est le code qui s'execute sur le robot. Il s'articule autour des composants suivants:
- une boucle d'évènements utilisant le module (asyncore)[http://docs.python.org/library/asyncore.html] de Python
- une machine à états
- une interface web reposant sur un serveur web (WSGI)[http://www.wsgi.org/] basé sur (asyncore)[http://docs.python.org/library/asyncore.html] réalisé par nos soins

![https://bitbucket.org/bhteam/bhware-open/downloads/wiki_webinterface.png]

### (Visualiseur de logs)[https://github.com/alberthier/bhware/tree/master/bhbot/tools/logviewer]

Tous les évènements qui transitent entre les deux cartes sont loggées dans un fichier.
Ce module nous permet d'inspecter un log et d'en filtrer le contenu afin de savoir ce qu'il s'est passé en cas de dysfonctionnement. Il nous permet également de voir la trajectoire demandée et réelle du robot sur le terrain.

!(Log Viewer)[https://bitbucket.org/bhteam/bhware-open/downloads/wiki_logviewer1.png] !(Log Viewer)[https://bitbucket.org/bhteam/bhware-open/downloads/wiki_logviewer2.png]

### (Simulateur)[https://github.com/alberthier/bhware/tree/master/bhbot/tools/simulator]

C'est l'un des composants les plus importants de notre infrastructure. Il nous permet de tester le code de la stratégie de sans avoir le matériel (avant même qu'il ne soit construit !). Ce simulateur se comporte comme une carte PIC32 et nous permet d'exécuter la stratégie. La visualisation graphique nous permet de vérifier que le robot se comporte comme voulu.

![https://bitbucket.org/bhteam/bhware-open/downloads/wiki_simulator.png]

### (Linux embarqué)[https://bitbucket.org/bhteam/bhware-open/src/tip/drunkstar/]

Nous construisons notre propre linux embarqué à l'aide de (Buildroot)[http://buildroot.uclibc.org/]. Ceci nous permet d'avoir un système minimaliste taillé à nos besoins sans superflu. Le système utilisé en 2011 consommait 5 Mo de RAM une fois chargé et disposait entre autre des outils suivants:
- Le shell (busybox)[http://busybox.net/]
- Python 2.7
- Une connexion Wifi qui nous permet de nous connecter au robot sans fil. La clé Wifi étant  retirée pour les matchs.
- Du port forwarding (netfilter) qui nous permet d'accéder au PIC32 depuis l'extérieur du robot
- Un éditeur de texte ((vim)[http://www.vim.org/])
- (Mercurial)[http://mercurial.selenic.com/] pour la synchronisation des sources
- etc...

(Buildroot)[http://buildroot.uclibc.org/] est un outil d'aide à la cross-compilation. Il propose une liste de paquets et d'options que l'on peut installer sur la cible. Une fois les composants choisis il télécharge les sources, les patche éventuellement et les compile pour l'architecture cible. La chaîne de cross-compilation est elle-même compilée par Buildroot, il n'est donc pas nécessaire d'en avoir une. Une fois le processus terminé on obtient une image disque à flasher sur la cible ou une archive à décompresser sur une clé USB, il ne reste plus ensuite qu'à démarrer sur ce système.
