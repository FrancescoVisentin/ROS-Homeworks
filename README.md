# Instruzioni per clonare repo:
Da terminale:
1) ```mkdir -p my_workspace/src/```
2) ```cd my_workspace/src/```
3) ```git clone https://github.com/FrancescoVisentin/ROS-Homeworks.git .```
4) ```catkin_init_workspace```
5) ```cd .. && catkin build```

# Istruzioni per eseguire:
L'eseguibile si chiama ```laserScan``` quindi: ```rosrun laser_scan laserScan```

# Istruzioni Rviz:
Per visualizzare su Rviz l'output aggiungere ```LaserScan``` ed ```Marker``` con parametri settati come nello screen sotto
![screen](https://github.com/FrancescoVisentin/ROS-Homeworks/assets/74708171/c2a9175d-bcf1-40bf-a762-93597edc8bb7)
