---
title: Démarage multi-drone
date: 25-09-2015
author: Alexandre Leclère, Arnaud Jacques, Julien Gérardy
taxonomy:
  category: [docs]
  tag: [ardrone,multi-drone]
---

# Démarage multi-drone 

## `autoconfarparrot` version 1.0


* Dans un premier terminal:
	1. `cd Bureau/backup/TFE_Drone`
	2. Pour chaque drone
		1. Se connecter au drone via wifi
		2. `bash autoconfarparrot`
	3. Répéter le point 2. pour chaque drone
	4. désactiver l'interface réseau
	5. `sudo ifconfig eth0 192.168.1.23`
	6. `sudo dhclient eth1`
	


## `autoconfarparrot` version 2.0

* vérifier que `nm-applet` est activé
* attendre que tous les drones soient découverts par la recherche wifi
* ourvir un terminal et utiliser `bash autoconfarparrot`
* le script s'occupe d'appareiller tous les drones et désactive `nm-applet`
* la configuration réseau est également effectuée

## ROS

* Dans un deuxième terminal
	* roscore
* Dans un troisième terminal
	* `roslaunch ardrone_tutorials keyboard_controller.launch`
* Accès aux fichiers de ardrone_tutorials
	* roscd ardrone_tutorials
		- `launch` : fichiers de paramétrisation du démarrage des noeud utilisés
(on déclare ici un groupe pour chaque drone)
		- `src` : fichiers de controle (on instantie les controleurs pour chaque drone)

Pour tuer ROS : `ctrl+c`

Pour tuer un driver en arrière plan, le mieux c'est de tuer ROS

Pour checker les topics ouverts :

Pour tuer un topic :

Pour trouver les IP des drones aux alentours : `nmap -pn 192.168.1.1-10`

Pour se connecter au linux du drone : `telnet ip_du_drone`

Pour "annuler" le ifconfig en cas de problème : `ifdown eth0`

Commandes ROS:
```
rostopic
rosnode
rosnode list
rosnode kill nom_du_node
```
