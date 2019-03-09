#!/usr/bin/env python

from CarMaker import *
from world import *
from Trajectory import *


## Vytvoreni sveta, prekazek a auta	
World_p = World()
Clio = World_p.SpawnCar(6,3.7, 3*pi/2, 0) # definovani pocatecni polohy auta	
Obs1 = obstacles(1.6,3.8,[4.5,.5], pi/2)
Obs2 = obstacles(1.6,3.8,[10,.5], pi/2)
Obs3 = obstacles(1.5,3.8,[20.2,.55], pi/2) 
Obs4 = obstacles(1.6,4.,[25,.5], 0)
Obs5 = obstacles(1.7,3.8,[29.8,.5], 0) 

## Prevod hranice sveta a prekazek na krivky
l1 = Obs1.getLines()
l2 = Obs2.getLines()
l3 = World_p.getLines()
l4 = Obs3.getLines()
l5 = Obs4.getLines()
l6 = Obs5.getLines()
lines = np.vstack((l1, l2, l3, l4, l5, l6)) # v pripade vytvoreni dalsi prekazky je nutno krivky z ni zahrnout do lines

## Vykresleni

#adjust plot
plt.subplots_adjust(left=0.05, bottom=0.05, right=0.95, top=0.95,
                wspace=0, hspace=0)
rectangles = [World_p.plot(),Obs1.plot(),Obs2.plot(),Obs2.plot(),Obs3.plot(),Obs4.plot(),Obs4.plot(),Obs5.plot()] # i zde se musi doplnit pripadne dalsi vytvorena prekazka
fig = plt.gcf()


## Vytvoreni parkovaciho asistenta pro dany svet
ClioPlanner = Planner(rectangles)

## Parkovani - sekvence najdi paralelni parkovaci misto, presun se, najdi pricne misto
ClioPlanner.ParkParallel(Clio,lines)
plt.pause(4)
plt.text(4, 10, 'Posun na kolmé parkování', style='oblique',fontsize = 30,
bbox={'facecolor':'green', 'alpha':0.5, 'pad':10})
plt.pause(1)
Clio = World_p.SpawnCar(21,7.5, 3*pi/2, 0)
ClioPlanner.ParkTransverse(Clio, lines)





