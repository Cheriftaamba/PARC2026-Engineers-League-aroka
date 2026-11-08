# PARC2026-Engineers-League
Pan-African Robotics Competition (PARC) Engineers League 2026 project development.

## Package Overview

- [`parc_robot_bringup`](./parc_robot_bringup/) : Contains config, world, scripts and launch files to bringup the CAYTU SITO-E robot.
- [`parc_robot_description`](./parc_robot_description/) : Contains the URDF description files for the CAYTU SITO-E and launch files for the robot state publisher and description.

## References
- [Cafe world](https://app.gazebosim.org/OpenRobotics/fuel/models/Cafe)
- [Other Gazebo models](https://app.gazebosim.org/models)
- [realsense_gazebo_plugin](https://github.com/pal-robotics/realsense_gazebo_plugin)
- [realsense-ros](https://github.com/realsenseai/realsense-ros)
- [RobotCAD](https://github.com/drfenixion/freecad.robotcad)

## Introduction

Cette tâche consiste à développer une solution de navigation autonome pour un robot de service évoluant dans un stade simulé à l’aide de Gazebo et RViz. L’objectif principal est de permettre au robot d’atteindre une position cible en utilisant le framework de navigation Nav2 sous ROS 2. Le robot doit percevoir son environnement grâce à ses capteurs, détecter les obstacles et planifier une trajectoire adaptée pour les éviter. La navigation autonome s’appuie sur la localisation, la planification du chemin et le contrôle des mouvements du robot. Cette technologie présente des avantages tels que l’autonomie, la précision et la réduction des interventions humaines, mais elle comporte également des défis liés aux obstacles dynamiques, à la fiabilité des capteurs et à la complexité de la configuration logicielle. Une solution efficace doit donc assurer une navigation sûre, fluide et robuste dans un environnement en constante évolution.

## Pays de l'équipe
Sénégal
## Noms des membres de l'équipe
Cherif TAMBA
Keba FAYE
Awa DIOUF
Khady THIAM
## Dépendances
Les principaux packages et outils nécessaires à la réalisation de cette tâche sont les suivants :

-   ROS 2 Jazzy : environnement de développement permettant la communication entre les différents composants du robot.
sudo apt-get install ros-jazzy-desktop

- Nav2 : framework de navigation autonome pour la planification des trajectoires, la gestion des obstacles et le contrôle du déplacement.
sudo apt-get install ros-jazzy-navigation2 ros-jazzy-nav2-bringup

- Gazebo : simulateur permettant de tester la navigation du robot dans un environnement virtuel. 

- RViz2 : outil de visualisation permettant d’observer la carte, la position du robot, les capteurs et la trajectoire planifiée.
sudo apt-get install ros-jazzy-rviz2

- TF2 : système de gestion des transformations entre les repères du robot et ceux de l’environnement.

# Tâche
Notre approche consiste à développer un programme Python compatible avec ROS 2 afin de guider le robot vers la position cible définie dans l’environnement de simulation. Nous utiliserons Nav2 pour gérer la planification du chemin, le suivi de trajectoire et l’évitement des obstacles. Le programme devra communiquer avec les composants de navigation en utilisant les interfaces ROS 2 appropriées, notamment les actions de navigation de Nav2. Nous vérifierons que la localisation du robot et les transformations entre les différents repères sont correctement configurées. Le robot devra adapter son déplacement en fonction des obstacles détectés et des informations fournies par ses capteurs. Enfin, nous testerons la solution dans Gazebo et RViz afin d’évaluer sa capacité à atteindre la cible de manière sûre et efficace.

## Défis rencontés
Les principaux défis susceptibles d’être rencontrés pendant le développement de la solution sont les suivants :
- Détection des obstacles : exploiter correctement les données des capteurs pour identifier les obstacles présents dans l’environnement en temps réelle.
- Atteinte de la cible : vérifier que le robot atteint la position et l’orientation souhaitées dans les tolérances et temps prévues.
