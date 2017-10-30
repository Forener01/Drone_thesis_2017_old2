# Etapes pour lancer la démo "Opération Séduction"
**1** Insérer une nouvelle batterie dans le drone 192.168.1.4

**2** Placer le drone près du bureau (loin des armoires)

**3** Activer le réseau et attendre découverte du drone

**4** Lancer dans un terminal:
$ cd ~/Bureau/Drone_thesis_2017/Operation_Seduction
$ bash launch.sh

**5** Insérer rapidement le mot de passe dans le premier terminal qui s'ouvre

**6** Attendre 20 sec que les autres terminaux s'ouvrent automatiquement

**7** Dans l'interface graphique de tum_ardrone:

- Sélectionner le fichier Demo_operation_seduction.txt dans "load file"
- Cliquer sur `[Reset]`
- Cliquer sur `[Clear an Send]` pour commencer la démo
si ne se lance pas, et que leds sont rouges, Cliquer sur `[Emergency]` ensuite répéter les deux points précédents (sans oublier reset!)
- Pour terminer la démo: `[Land]`

# Problèmes connus:
Si la carte est illisible car trop de points => sélectionner sa fenêtre et taper `r`
