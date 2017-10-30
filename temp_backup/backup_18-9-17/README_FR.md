# UCL_DRONE_2016

Ce dépot git est le lieu de partage du code entre les deux groupes de mémorants 2016 dans le but de créer une base commune.

Lire `CONTRIBUTING.md` pour contribuer

Mise en place du workspace
--------------------------

1.	(Installer ROS)
2.	Se placer dans le dossier où l'on veut copier le workspace de développement (n'importe où)
3.	Cloner ce dépôt dans ce dossier (`git clone`).
4.	Procédure d'initialisation d'nu workspace ROS
5.	Ajouter à la fin de `.bashrc` le chemin vers ce dossier:

```
$ nano ~/.bashrc
source <chemin vers le workspace>/devel/setup.bash
```

1.	si `ardrone_autonomy` n'est pas installé, cloner le dans `<chemin vers le workspace>/src`
2.	Dans un terminal, se placer dans ce dossier et taper

```
catkin_make
```

Arborescence
============


- `drone_preparation/Appareillage` Contient le code adapté de Nicolas Rowier (`autoconfarparrot`) ainsi que les fichiers de configuration associés

- `doc/` Documentation `rosdoc` (à générer)

-	`include/ucl_drone/` Contient les headers de nos fichiers C++.

- `launch/` contient tous les fichiers `.launch`

- `msg/` contient les définitions des messages ROS propres à notre code

-	`src/` Contient le code des noeuds ROS (un noeud par fichier C++):

	-	`controller/`

	    noeud pour envoyer les instructions de décollage/aterrissage et controller en position dans le repère inertiel

	-	`computer_vision/`

	    noeud pour traiter les images et extraire les descripteurs

	-	`map/`

	    noeud pour répertorier les descripteurs dans le repère inertiel, ainsi que pour estimer la pose du drone

	-	`path_planning/`

	    noeud pour envoyer, au controlleur, une suite de positions à atteindre

	-	`pose_estimation/`

	    noeud pour estimer la pose (position et orientation) du drone sur base des capteurs et de la carte

	-   `strategy/`

	    noeud qui détermine la suite des opérations à effectuer pour remplir son role attribué en début de mission

	-   `multi_strategy/`

	    noeud pour attribuer un role à chaque drone *avant le début de la mission*

- `srv/` contient les définitions des services ROS

- `target/` Contient les images des targets qui peuvent être sélectionnées

-	`CmakeLists.txt` Contient les chemins de tout ce qui doit être pris en compte dans la compilation catkin.

-	`package.xml` [TO BE UPDATED] Informations relatives au projet, notamment les dépendances et versions.

Procédure de lancement
======================

1.	Placer une batterie chargée dans le drone (au préalable, configurer l'essid du drone et lui attribuer une adresse ip)
2.	Activer l'applet réseau, attendre la découverte du drone et s'y connecter
3.	Dans un terminal, se placer dans le dossier workspace et lancer le drone:

    ```
    $ cd src/ucl_drone/drone_preparation/Appareillage
    $ bash autoconfarparrot
    ```

    Attendre de recevoir un message positif (en vert) pour chaque drone.
    Si une erreur apparaît, vérifier que le routeur est accessible (`192.168.1.254`).

4. Dans un autre terminal, lancer le démon ROS:
    ```
    $ roscore
    ```
    attendre le message
    ```
    started core service [/rosout]
    ```

5. Dans un autre terminal,
    ```
    $ roslaunch ucl_drone <NOM DE FICHIER DU DRONE>.launch
    ```

6. `Ctrl-C` pour tuer

    Attention! si on tue les noeuds avant d'avoir envoyer l'ordre d'atterir, le drone reste en vol !

    Utiliser `ucl_drone_gui` en lançant `rqt` dans un terminal (le plugin se trouve dans un menu contenant une liste de tout ceux disponibles). Attention! `rqt` doit être lancé après `roscore` et doit être redémarré si `roscore` est arrêté.
    

## Documentation
This folder `src/ucl_drone/doc` contains the documentation generated in html with `rosdoc-lite` which uses doxygen comments in the codes.

The main page of the generated html documentation is here:
`src/ucl_drone/doc/html/index-msg.html`

### Generate documentation
Go into the `src/ucl_drone` folder and use:
```
$ rosdoc_lite .
```
(do not forget the dot `.` which means "the current folder" in bash)
