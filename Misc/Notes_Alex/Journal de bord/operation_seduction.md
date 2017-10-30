1) nouvelle batterie pour le drone 192.168.1.4

2) Placer le drone sur le post-it

2) Activer le réseau et attendre découverte du drone

3) terminaux:

================== terminal

```
$ cd Bureau/TFE_Alex_Arnaud/Appareillage/
$ bash autoconfarparrot
```

Attendre de recevoir
	Connection closed by foreign host.
Sinon, si échec, par exemple
	Interface doesn't support scanning : Device or resource busy
simplement recommencer au point 2 (autoconfparrot)

================= autre terminal

```
$ roscore
```

attendre le message
```
	started core service [/rosout]
```

================= autre terminal

```
$ roslaunch tum_ardrone ardrone_driver.launch
```

Attendre de recevoir
```
	Drone Navdata Send Speed: 200Hz
```

================= autre terminal

```
$ roslaunch tum_ardrone tum_ardrone.launch
```

Attendre de recevoir

```
	Object::connect:  (receiver name: 'tum_ardrone_guiClass')
```

================= autre terminal

```
$ source ~/rosbuild_ws/setup.bash
$ rosrun lsd_slam_viewer viewer
```

================= autre terminal

```
$ source ~/rosbuild_ws/setup.bash
$ rosrun lsd_slam_core live_slam /image:=/ardrone/front/image_raw _calib:=/home/laboinmastudent/ardrone_front.cfg
```


======== dans l'interface de tum_ardrone

sélectionner le fichier Demo_operation_seduction.txt dans "load file"

Cliquer sur `[Reset]`
Cliquer sur `[Clear an Send]` pour commencer la démo
si ne se lance pas, et que leds sont rouges, Cliquer sur `[Emergency]` ensuite répéter les deux points précédents (sans oublier reset!)

Pour terminer la démo: `[Land]`


================ Problèmes connus:

 * si la carte est illisible car trop de points => sélectionner sa fenêtre et taper `r`
