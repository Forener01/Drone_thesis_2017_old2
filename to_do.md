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

- TFE 2014: Filtre de Canny sur algo de Harris
- https://answers.ros.org/question/272291/drone-feedback-object-detection-color-detection/
- Harris Corner detector, Shi-Tomasi, Personalized corner detector, Homography

# A faire
- Lire toutes les références
- Checker refs.md
- Checker utilisation de ar_nav package

- Envoyer mail à Tobias

- Adapter pour contrôle en position
- Adapter pour contrôle en vitesse altitude
- Adapter pour contrôle en vitesse rotation (yaw)

- Tester d'autres espaces de couleur: RGB, LAB, HSV

- Immédiat
  - Computer vision
    - programmer object matching

  - Controller
    - hardcode position avec FSM
    - mesurer et paramétrer le profil trapéz.
    - faire mesures vitesse trapéz.
    - contrôle position trapéz.
    - faire mesures position

  - Divers
    - Affiner structure pages mémoire

-----------
15h40-18h34
18h50-19h50

# Planning 
4/12: transfert image + calibration   !!! OK !!! 
5/12: filtre rouge                    !!! OK !!! 
6/12: algo Sobel                      !!! OK !!!
7/12: matching 
8/12: matching
9/12: fusion
10/12: fusion 

# Update meet 21/11
- change name for "no control" wave (controller)
- préciser que les courbes sont moyennées, et ajouter la variance des 5 essais.
- rajouter min/max de chaque type de contrôleur sur le même graphe (jouer avec couleur de la courbe)
- faire recherches pour expliquer la fréquence des 200 Hz
- lookup table pour matching step

# Commentaires
- TUM_ardrone font leur propre intégration visuelle sur base des raw images de
la caméra frontale/ventrale. Ensuite, ils contrôlent ça, et le tout est lié à
leur PTAM => trop compliqué de reprendre quoique ce soit.

- Adapter prend du temps: il faut créer une architecture de communication ros 
et intégrer les noeuds appropriés aux bons endroits.

- Quelques jours de perdu pendant campagne de mesures: controleur de vitesse 
fonctionnait avec le marquage de base du DroneLab, mais pas à la DroneZone à
cause du revêtement trop uniforme de celle-ci. Ordre de débug: erreurs de code,
batteries défectueuses, pc portable trop peu puissant, et enfin, revêtement
inadéquat.

- Camera calibration: http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration
- cv_bridge: permet de switcher entre ROS et OpenCV images
- image_proc: permet d'appliquer le fichier de calibration aux raw images.
- transform data: use tf package
- color conversion: https://docs.opencv.org/master/de/d25/imgproc_color_conversions.html
- sobel conversion:
https://docs.opencv.org/master/d4/d86/group__imgproc__filter.html#gacea54f142e81b6758cb6f375ce782c8d
https://docs.opencv.org/master/d4/d86/group__imgproc__filter.html#gacea54f142e81b6758cb6f375ce782c8d
https://docs.opencv.org/master/d2/d2c/tutorial_sobel_derivatives.html

    src_gray: In our example, the input image. Here it is CV_8U
    grad_x/*grad_y*: The output image.
    ddepth: The depth of the output image. We set it to CV_16S to avoid overflow.
    x_order: The order of the derivative in x direction.
    y_order: The order of the derivative in y direction.
    scale, delta and BORDER_DEFAULT: We use default values.

- cv::findContours
- cv::threshold: GRAY to BIN 	

- matching: surf descriptor ?
  - floodfill algorithm?
  - hough transform
  - binary dilation
  - rectangle detection: OpenCV (look in samples/squares.c)
    0. rectangles <- {}
    1. image <- load image
    2. for every channel:
      2.1  image_canny <- apply canny edge detector to this channel
      2.2  for threshold in bunch_of_increasing_thresholds:
        2.2.1   image_thresholds[threshold] <- apply threshold to this channel
      2.3  for each contour found in {image_canny} U image_thresholds:
        2.3.1   Approximate contour with polygons
        2.3.2   if the approximation has four corners and the angles are close to 90 degrees.
          2.3.2.1    rectangles <- rectangles U {contour}
  - Harris + center of gravity + moments (cv::cornerHarris + cv::threshold)
  - PCL to build door detection?
- parler des color spaces et du choix fait https://www.learnopencv.com/color-spaces-in-opencv-cpp-python/


- OpenCV stores internally a RGB image as a BGR one.
- Quid Blurring on the color image ?

- défaillance du décollage après trop de temps d'utilisation continue (3h+) ? 
Confimé !!

- discuter choix RGB vs HSV

- expliquer qu'au début on trouvait Harris intéressant, puis finalement inutile.

- Parler du Scharr operator

- Comparer HoughLines avec HoughLinesP

- Présenter les pires cas et les cas idéaux pour ouverture porte

# Matching step sources
- Hough lines https://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/hough_lines/hough_lines.html
-> OpenCV: HoughLines and HoughLinesP

- Tuned Harris algo (p.248)
https://books.google.be/books?id=J1QoDwAAQBAJ&pg=PA239&lpg=PA239&dq=door+detection+opencv&source=bl&ots=HATCdKbllA&sig=fvGqEZBrXsFLIzEu4jWbMVUaL4Y&hl=fr&sa=X&ved=0ahUKEwi7rLq67_rXAhXoCMAKHZKdBMc4KBDoAQgtMAA#v=onepage&q=door%20detection%20opencv&f=false

- https://stackoverflow.com/questions/1817442/how-to-recognize-rectangles-in-this-image

- https://stackoverflow.com/questions/1817442/how-to-recognize-rectangles-in-this-image

# Structure Rapport TFE
- Abstract: 1p
- Introduction: 5p
- State of the art: 20p
- Computer vision: 15p
- Control: 5p
- Results: 10p
- Améliorations locales: 5p
- Améliorations générales: 10p (changer hardware, rajouter sonar, contrôle yaw)
- Conclusion: 5p
--> Sum = 76p

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
