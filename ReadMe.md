# Mission WindWatch 2024 - Aqua.Bot

Bienvenue dans le projet **Mission WindWatch 2024**. Ce projet vise à développer un drone maritime autonome pour une mission de **surveillance**, **d'inspection**, et de **navigation autonome** dans un environnement maritime complexe. Le drone doit être capable de naviguer parmi des obstacles, d'inspecter des cibles (éoliennes), et d'accomplir des objectifs de mission tels que l'identification de défaillances et l'évitement des obstacles. Ce projet se concentre sur l'intégration de plusieurs capteurs, le contrôle de la propulsion, et la coordination de la mission dans un cadre dynamique.

Ce **README** présente une vue d'ensemble de la structure du projet, les différents **packages ROS2**, leurs nœuds associés, ainsi que les **topics** personnalisés utilisés pour la communication entre les nœuds.

## Comment Ajouter ce Package au Projet

Pour commencer, vous pouvez ajouter le package de base à votre projet en utilisant les commandes suivantes :

```bash
# Créez un nouveau package avec ROS2
ros2 pkg create --build-type ament_cmake --node-name simple_node simple_package

# Allez dans le répertoire vrx_ws
cd ~/vrx_ws

# Installation
colcon build --merge-install
. install/setup.bash

# Lancer le nœud
ros2 launch simple_package simple_node_launch.py
```

Ces étapes permettent de créer et lancer un package de base. Le package `simple_package` est un point de départ, mais il devra être modifié et étendu pour inclure les fonctionnalités spécifiques de la mission WindWatch 2024.

## Structure du Projet

Le projet est organisé en plusieurs **packages ROS2** pour garantir une approche modulaire, claire, et extensible. Chaque package contient un ensemble de nœuds responsables de certaines tâches spécifiques liées à la mission.

### Packages et Nœuds

#### 1. Package `navigation`

Ce package contient les nœuds responsables de la **navigation** et du **contrôle des moteurs**.

- **Nœud `navigation_node`**
  - **Rôle** : Responsable de la **navigation autonome**. Planifie les trajets pour atteindre les objectifs de mission tout en évitant les obstacles.
  - **Topics Souscrits** :
    - `/mission/odometry` : Odométrie fusionnée (position et orientation).
    - `/mission/objective_positions` : Positions des cibles à atteindre.
    - `/mission/avoidance_course` : Trajectoire d’évitement à suivre pour éviter les obstacles.
    - `/mission/target_orientations` : Orientations des éoliennes pour ajuster la trajectoire.
    - `/aqua_bot/ais_sensor/vessel_positions` : Positions des navires environnants pour éviter les collisions.
  - **Topics Publiés** :
    - `/propulsion/command` : Commandes de poussée et d’orientation des moteurs.

- **Nœud `propulsion_control_node`**
  - **Rôle** : Gérer les **commandes des propulseurs** (poussée et orientation).
  - **Topics Souscrits** :
    - `/propulsion/command` : Commandes provenant du `navigation_node`.
  - **Topics Publiés** :
    - `/aqua_bot/thrusters/left/pos`, `/aqua_bot/thrusters/right/pos` : Commandes d'orientation des moteurs.
    - `/aqua_bot/thrusters/left/thrust`, `/aqua_bot/thrusters/right/thrust` : Commandes de poussée des moteurs.

#### 2. Package `sensors`

Ce package contient les nœuds responsables de la collecte des **données des capteurs** et de leur **fusion** pour fournir une vue globale fiable.

- **Nœud `sensor_fusion_node`**
  - **Rôle** : Fusionner les données du GPS et de l'IMU via un **EKF** pour obtenir une estimation fiable de la position et de l’orientation du bateau.
  - **Topics Souscrits** :
    - `/aqua_bot/sensors/gps/fix` : Données GPS.
    - `/aqua_bot/sensors/imu/data` : Données IMU.
  - **Topics Publiés** :
    - `/mission/odometry` : Odométrie fusionnée pour les autres nœuds.

- **Nœud `camera_processing_node`**
  - **Rôle** : Traiter les images de la **caméra** pour identifier les cibles, lire les QR codes, et déterminer l’orientation des éoliennes.
  - **Topics Souscrits** :
    - `/aqua_bot/sensors/cameras/main_camera/image_raw` : Images de la caméra 360°.
  - **Topics Publiés** :
    - `/mission/target_status` : Statut des éoliennes détectées (fonctionnelle ou défectueuse).
    - `/mission/target_orientations` : Orientation des cibles (éoliennes).

- **Nœud `tgt_pos_update_node`**
  - **Rôle** : Traiter les positions des cibles reçues par **GPS** et les transmettre.
  - **Topics Souscrits** :
    - `/aqua_bot/ais_sensor/windturbines_positions` : Liste des positions GPS des éoliennes.
  - **Topics Publiés** :
    - `/aqua_bot/ais_sensor/target_positions` : Positions des éoliennes.

#### 3. Package `mission_manager`

Ce package gère la **planification des missions**, les **objectifs à atteindre**, et la **coordination de la mission**.

- **Nœud `target_manager_node`**
  - **Rôle** : Centraliser et mettre à jour les informations sur les éoliennes (position, orientation, statut).
  - **Topics Souscrits** :
    - `/aqua_bot/ais_sensor/target_positions` : Positions des éoliennes.
    - `/mission/target_orientations` : Orientations mises à jour des éoliennes.
  - **Topics Publiés** :
    - `/mission/objective_positions` : Positions des éoliennes à atteindre.

- **Nœud `obstacle_avoidance_node`**
  - **Rôle** : Identifier les **obstacles** et proposer des modifications de trajectoire pour les éviter.
  - **Sources de données** :
    - **Obstacles fixes** : Les coordonnées des obstacles fixes (rochers, îles, phare) sont disponibles dès le départ à partir d'un fichier de configuration.
  - **Topics Souscrits** :
    - `/mission/odometry` : Odométrie fusionnée.
    - `/aqua_bot/ais_sensor/vessel_positions` : Positions des autres navires pour éviter les collisions.
  - **Topics Publiés** :
    - `/mission/avoidance_course` : Trajectoire d’évitement proposée.

- **Nœud `mission_coordinator_node`**
  - **Rôle** : Coordonner les **phases de la mission**, émettre les ordres de mission.
  - **Topics Souscrits** :
    - `/mission/target_status` : Statut des cibles (éoliennes).
  - **Topics Publiés** :
    - `/mission/mission_goal` : Objectifs de mission pour le `navigation_node`.

#### 4. Package `visualization`

Ce package est responsable de la **supervision** et de la **visualisation** en temps réel de la mission.

- **Nœud `visualization_node`**
  - **Rôle** : Fournir une visualisation tactique via **RViz2**.
  - **Topics Souscrits** :
    - `/mission/odometry` : Position et orientation du drone.
    - `/aqua_bot/ais_sensor/target_positions` : Positions des cibles et des obstacles.
    - `/mission/target_status` : Statut des éoliennes.
    - `/mission/target_orientations` : Orientation des cibles.
    - `/aqua_bot/ais_sensor/vessel_positions` : Positions des autres navires.

### Topics Personnalisés

Pour une communication efficace entre les nœuds, plusieurs **topics personnalisés** sont utilisés :

1. **Topic `/mission/target_orientations`**
   - **Description** : Contient les orientations des cibles (éoliennes).
   - **Publié Par** : `camera_processing_node`.
   - **Souscrit Par** : `target_manager_node`, `navigation_node`.

2. **Topic `/mission/objective_positions`**
   - **Description** : Positions des cibles à atteindre.
   - **Publié Par** : `target_manager_node`.
   - **Souscrit Par** : `navigation_node`.

3. **Topic `/propulsion/command`**
   - **Description** : Commandes pour la position et la poussée des moteurs.
   - **Publié Par** : `navigation_node`.
   - **Souscrit Par** : `propulsion_control_node`.

4. **Topic `/mission/target_status`**
   - **Description** : Statut des éoliennes (état fonctionnel ou défectueux).
   - **Publié Par** : `camera_processing_node
