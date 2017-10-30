# Objectifs
NEW
1. Contrôler trajectoire drone
2. Détecter porte: contour + filtre couleur rouge

OLD
1. détecter un mur
2. longer un mur
3. détecter une ouverture
4. franchir l'ouverture

# Idées
- utiliser l'emergency d'ardrone_autonomy
- utiliser l'interface graphique
- Comparer 2016 perf avec ArUco markers pose estimation
- ArUco Markers with ArUco library (pas assez flexible pour l'objectif à long terme)
- Check source: PIG Guidelines
  - http://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlPID
  - http://ieeexplore.ieee.org/document/7402144/?reload=true
  - https://en.wikipedia.org/wiki/PID_controller#Manual_tuning

- Odometers wrappers
  - www.ros.org/wiki/viso2_ros 
  - www.ros.org/wiki/fovis_ros

- GetMap service from nav_msgs
- Noeud spécifique aux infos du drone 

# A faire
- Checker refs.md
- Checker utilisation de ar_nav package
- Checker tum_autonomy

- Envoyer mail à Tobias

- Recherche détection contour et filtre couleur

- Utiliser ardrone_velocity_ekf (TU Darmstadt)
- Utiliser contrôleur UCL 2016

- Adapter ardrone_velocity pour contrôle en position 
- Adapter ardrone_velocity pour contrôle en vitesse altitude

# Structure Rapport TFE
- Abstract: 1p
- Introduction: 5p
- State of the art: 15p
- Computer vision: 15p
- Control: 10p
- Results: 10p
- Conclusion: 5p
--> Sum = 61p

# A lire
- Nouvelles ressources

http://www.edwardrosten.com/work/fast.html
http://www.ipb.uni-bonn.de/sfop/?L=1
https://sites.google.com/a/compgeom.com/stann/Home
http://pointclouds.org/
http://robwhess.github.io/opensift/
http://koen.me/research/colordescriptors/
http://laurentkneip.github.io/opengv/
http://www2.warwick.ac.uk/fac/sci/dcs/research/combi/research/bic/software/sntoolbox
http://users.ics.forth.gr/~lourakis/posest/

- Illinois
https://scholar.google.com/citations?view_op=view_citation&hl=en&user=-ClLU3EAAAAJ&citation_for_view=-ClLU3EAAAAJ:wbdj-CoPYUoC
https://scholar.google.com/citations?view_op=view_citation&hl=en&user=-ClLU3EAAAAJ&citation_for_view=-ClLU3EAAAAJ:qjMakFHDy7sC
https://scholar.google.com/citations?view_op=view_citation&hl=en&user=-ClLU3EAAAAJ&cstart=20&pagesize=80&citation_for_view=-ClLU3EAAAAJ:ufrVoPGSRksC
https://scholar.google.com/citations?view_op=view_citation&hl=en&user=-ClLU3EAAAAJ&cstart=20&pagesize=80&citation_for_view=-ClLU3EAAAAJ:ns9cj8rnVeAC
https://scholar.google.com/citations?view_op=view_citation&hl=en&user=-ClLU3EAAAAJ&cstart=20&pagesize=80&citation_for_view=-ClLU3EAAAAJ:b0M2c_1WBrUC
https://scholar.google.com/citations?view_op=view_citation&hl=en&user=-ClLU3EAAAAJ&cstart=100&pagesize=100&citation_for_view=-ClLU3EAAAAJ:qUcmZB5y_30C

# Astuces
1. /ARDrone_SDK_2_0_1/ARDroneLib/Soft/Common/vision_common.h
contient les n° de modèles des caméras utilisée + autres composants

/ARDrone_SDK_2_0_1/ARDroneLib/Soft/Lib/ardrone_tool/

2. ## toggle_cam package
petit package pour alterner de manière automatique les deux caméras

3. tentative de lancement du PTAM de Zurich pour palier au problème de recherche de nouveaux points chez TUM

Installation du package ROS pour PTAM --> ok
Calibration de la caméra --> ok
Besoin de convertir l'image raw du drone en image BW. Pistes:
- cv-bridge (comme TUM)
- image_pipeline (--> image_proc)
- récupérer tout le mécanisme de conversion de TUM
Essais avec image_proc --> bug
Résolu: il faut le lancer depuis

$ source ~/rosbuild_ws/setup.bash
$ ROS_NAMESPACE=ardrone rosrun image_proc image_proc

4. Nous avons suivi le livre 'A Gentle Introduction to ROS' de "O'Kane", Chapitre 3 => `hello.cpp` notre premier noeud ROS

5. Choix d'une librairie C++ pour le calcul matriciel

Plusieurs choix:
 * TooN (utilisé par tum_ardrone)
 * Eigen (utilisé par PCL)
 * GMTL

Eigen semble être recommandé par la communauté ROS

6. Calibration des caméras
Rubrique "Cameras" du wiki de ardrone_autonomy:
    http://ardrone-autonomy.readthedocs.org/en/latest/reading.html?highlight=calibration
Rubrique FAQ pour plus de précisions:
    http://ardrone-autonomy.readthedocs.org/en/latest/FAQ.html?highlight=calibration
Demande l'utilisation du calibrateur tout fait en ROS
    http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration
