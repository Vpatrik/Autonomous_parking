#!/usr/bin/env python

import numpy as np
from numpy import pi
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from linecross import *
from CarMaker import *
from world import *

class Planner:

	def __init__(self, rectangles, sirka = 1.65, delka = 3.82, minR = 5.5):
		self.rectangles = rectangles
		self.sirka = sirka
		self.delka = delka
		self.minR = minR
		self.min_parallel_place = math.sqrt(2*self.minR*self.sirka+(self.delka-0.5)**2)
		self.parkingPlaceSpotted = 0
		self.firstPoint = [0,0]
		self.lastPoint = [0,0]
		self.place = 0

	def decideParallelParking(self, lines, wheelAngularRotation, parkInProgress, Car):
		if parkInProgress is 0:
			SpottedPlace = Car.spotParkingPlace(lines, self.parkingPlaceSpotted)

			if SpottedPlace is None:
				self.place += Car.timeStep*wheelAngularRotation*Car.D_kola/2
				if self.place  > 2*self.min_parallel_place - 3 and self.firstPoint[0] is not 0:
					parkPoint = [(Car.Sx+self.firstPoint[0])/2,self.firstPoint[1]]
					self.lastPoint = [Car.Sx,Car.Sy]
					self.parkingPlaceSpotted = 1
					return parkPoint
				return

			elif self.firstPoint[0] is 0:
				self.firstPoint = [SpottedPlace[0],SpottedPlace[1]]
				return

			elif self.lastPoint[0] is 0:
				self.lastPoint = [SpottedPlace[0],SpottedPlace[1]]
				self.place = abs(self.lastPoint[0]-self.firstPoint[0])
				if self.place > self.min_parallel_place:
					self.parkingPlaceSpotted = 1
					parkPoint = [(self.lastPoint[0]+self.firstPoint[0])/2,self.lastPoint[1]]
					return parkPoint
				else:
					self.firstPoint = [SpottedPlace[0],SpottedPlace[1]]
					self.lastPoint = [0,0]
			return


	def decideTransverseParking(self, lines, wheelAngularRotation, parkInProgress, Car):
		if parkInProgress is 0:
			SpottedPlace = Car.spotParkingPlace(lines, self.parkingPlaceSpotted)

			if SpottedPlace is None:
				self.place += Car.timeStep*wheelAngularRotation*Car.D_kola/2
				if self.place  > self.sirka + 2 and self.firstPoint[0] is not 0:
					self.parkingPlaceSpotted = 1
					self.lastPoint = [Car.Sx,self.firstPoint[1]]
					parkPoint = [(Car.Sx+self.firstPoint[0])/2, self.lastPoint[1]]
					return parkPoint
				return

			elif self.firstPoint[0] is 0:
				self.firstPoint = [SpottedPlace[0],SpottedPlace[1]]
				return

			elif self.lastPoint[0] is 0:
				self.lastPoint = [SpottedPlace[0],SpottedPlace[1]]
				self.place = abs(self.lastPoint[0]-self.firstPoint[0])
				if self.place > self.sirka + 0.2:
					self.parkingPlaceSpotted = 1
					parkPoint = [(self.lastPoint[0]+self.firstPoint[0])/2,self.lastPoint[1]]
					return parkPoint
				else:
					self.firstPoint = [SpottedPlace[0],SpottedPlace[1]]
					self.lastPoint = [0,0]
			return



	def ParkParallel(self, Car_o, lines):
		wheelAngularRotation = 4;
		delta_s_dot = 0
		parkInProgress = 0
		self.firstPoint = [0,0]
		self.lastPoint = [0,0]
		ParkPoint = self.decideParallelParking(lines, wheelAngularRotation, parkInProgress, Car_o)
		if ParkPoint is None: 
			ProceedParking = 0
		while ProceedParking is 0:
			plt.cla()
			Car_o = Car(*Car_o.drive(wheelAngularRotation, delta_s_dot))
			colision = Car_o.controlColision(lines)

			if colision == 'Inside':
				print('Auto narazilo do prekazky')
				return
			Car_o.plotCar()
			self.plotWorld(self.rectangles)
			plt.pause(0.1)
			ParkPoint = self.decideParallelParking(lines, wheelAngularRotation, parkInProgress, Car_o)
			if ParkPoint is None: 
				ProceedParking = 0
			else:
				ProceedParking = 1
				print('Parking place was found')
		plt.text(4, 10, 'Parkovací místo pro podélné parkování nalezeno', style='oblique',fontsize = 30,
        	bbox={'facecolor':'green', 'alpha':0.5, 'pad':10})
		plt.pause(4)

		while abs(abs(self.lastPoint[0] - Car_o.Sx)  - 1) > 0.07:
			plt.cla()
			wheelAngularRotation = 2;
			delta_s_dot = 0
			Car_o = Car(*Car_o.drive(wheelAngularRotation, delta_s_dot))
			colision = Car_o.controlColision(lines)
			if colision == 'Inside':
				print('Auto narazilo do prekazky')
				return
			Car_o.plotCar()
			self.plotWorld(self.rectangles)
			plt.pause(0.01)
		R = Car_o.rozvor/(np.tan(Car_o.delta_s_max))
		P = self.place
		p = self.min_parallel_place
		W = self.sirka
		d = abs(Car_o.Sy+np.sin(Car_o.fi)*self.sirka/2-ParkPoint[1])
		s = P-p
		M = [Car_o.Sx-P/2, Car_o.Sy-(W+d)/2]
		F = [Car_o.Sx-P, Car_o.Sy-(W+d)]
		Oi = [Car_o.Sx, Car_o.Sy -((W+d)**2+P**2)/(4*(W+d))]
		Of = [Car_o.Sx - P, Car_o.Sy - ((3*(W+d)**2 - P**2)/(4*(W+d)))]
		Rf = ((W+d)**2 + P**2)/(4*(W+d))
		if Rf < R:
			print('Najed si s vetsim odstupem')
			print('Potrebny minimalni polomer je:',Rf)
			print('Minimalni mozny polomer je:', R)
			plt.text(20, 10, 'Nelze zaparkovat - je nutno najet jiným způsobem', style='oblique',fontsize = 30,
        		bbox={'facecolor':'green', 'alpha':0.5, 'pad':10})
			plt.pause(4)
			return
		delta_s_optimal = np.arctan(Car_o.rozvor/Rf)
		self.plotParallel(F, Oi, Of, M, Rf)
		plt.text(20, 10, 'Ideální trajektorie', style='oblique',fontsize = 30,
        	bbox={'facecolor':'green', 'alpha':0.5, 'pad':10})
		plt.pause(4)
		while abs(delta_s_optimal - abs(Car_o.delta_s)) > 0.005:
			plt.cla()
			wheelAngularRotation = 0;
			delta_s_dot = -0.03
			Car_o = Car(*Car_o.drive(wheelAngularRotation, delta_s_dot))
			colision = Car_o.controlColision(lines)
			if colision == 'Inside':
				print('Auto narazilo do prekazky')
				return
			Car_o.plotCar()
			self.plotParallel(F, Oi, Of, M, Rf)
			self.plotWorld(self.rectangles)
			plt.pause(0.001)

		while abs(math.sqrt(abs(M[0]-Car_o.Sx)**2) + abs(Car_o.Sy-M[1])**2) > 0.03:
			plt.cla()
			wheelAngularRotation = -1.2;
			delta_s_dot = 0
			Car_o = Car(*Car_o.drive(wheelAngularRotation, delta_s_dot))
			colision = Car_o.controlColision(lines)
			if colision == 'Inside':
				print('Auto narazilo do prekazky')
				return
			Car_o.plotCar()
			self.plotParallel(F, Oi, Of, M, Rf)
			self.plotWorld(self.rectangles)
			plt.pause(0.001)

		while abs(delta_s_optimal - Car_o.delta_s) > 0.005:
			plt.cla()
			wheelAngularRotation = 0;
			delta_s_dot = 0.04
			Car_o = Car(*Car_o.drive(wheelAngularRotation, delta_s_dot))
			colision = Car_o.controlColision(lines)
			if colision == 'Inside':
				print('Auto narazilo do prekazky')
				return
			Car_o.plotCar()
			self.plotParallel(F, Oi, Of, M, Rf)
			self.plotWorld(self.rectangles)
			plt.pause(0.001)

		while abs(math.sqrt(abs(F[0]-Car_o.Sx)**2) + abs(Car_o.Sy-F[1])**2) > 0.05:
			plt.cla()
			wheelAngularRotation = -1.2;
			delta_s_dot = 0
			Car_o = Car(*Car_o.drive(wheelAngularRotation, delta_s_dot))
			colision = Car_o.controlColision(lines)
			if colision == 'Inside':
				print('Auto narazilo do prekazky')
				return
			Car_o.plotCar()
			self.plotParallel(F, Oi, Of, M, Rf)
			self.plotWorld(self.rectangles)
			plt.pause(0.001)

		while abs(Car_o.delta_s) > 0.008:
			plt.cla()
			wheelAngularRotation = 0;
			delta_s_dot = -0.07
			Car_o = Car(*Car_o.drive(wheelAngularRotation, delta_s_dot))
			colision = Car_o.controlColision(lines)
			if colision == 'Inside':
				print('Auto narazilo do prekazky')
				return
			Car_o.plotCar()
			self.plotParallel(F, Oi, Of, M, Rf)
			self.plotWorld(self.rectangles)
			plt.pause(0.005)

		while abs(Car_o.stred[0] - ParkPoint[0] + 0.2) > 0.07:
			plt.cla()
			wheelAngularRotation = 3;
			delta_s_dot = 0
			Car_o = Car(*Car_o.drive(wheelAngularRotation, delta_s_dot))
			colision = Car_o.controlColision(lines)
			if colision == 'Inside':
				print('Auto narazilo do prekazky')
				return
			Car_o.plotCar()
			self.plotParallel(F, Oi, Of, M, Rf)
			self.plotWorld(self.rectangles)
			plt.pause(0.01)
		plt.text(20, 10, 'Zaparkováno', style='oblique',fontsize = 30,
        	bbox={'facecolor':'green', 'alpha':0.5, 'pad':10})
		plt.pause(4)
		print('Parked')
		self.parkingPlaceSpotted = 0
		self.place = 0 

	def ParkTransverse(self, Car_o, lines):
		wheelAngularRotation = 5;
		delta_s_dot = 0
		parkInProgress = 0
		self.firstPoint = [0,0]
		self.lastPoint = [0,0]
		ParkPoint = self.decideTransverseParking(lines, wheelAngularRotation, parkInProgress, Car_o)
		if ParkPoint is None: 
			ProceedParking = 0

		while ProceedParking is 0:
			plt.cla()
			Car_o = Car(*Car_o.drive(wheelAngularRotation, delta_s_dot))
			colision = Car_o.controlColision(lines)

			if colision == 'Inside':
				print('Auto narazilo do prekazky')
				break
			Car_o.plotCar()
			self.plotWorld(self.rectangles)
			plt.pause(0.05)
			ParkPoint = self.decideTransverseParking(lines, wheelAngularRotation, parkInProgress, Car_o)
			if ParkPoint is None: 
				ProceedParking = 0
			else:
				ProceedParking = 1
				print('Parking place was found')
		plt.text(4, 10, 'Parkovací místo pro kolmé parkování nalezeno', style='oblique',fontsize = 30,
        	bbox={'facecolor':'green', 'alpha':0.5, 'pad':10})
		plt.pause(4)
		for i in range(15): # ok nejuzsi
#		while abs(abs(self.lastPoint[0] - Car_o.Sx)  - Car_o.rozvor/2) > 0.15:
			plt.cla()
			wheelAngularRotation = 5;
			delta_s_dot = 0
			Car_o = Car(*Car_o.drive(wheelAngularRotation, delta_s_dot))
			colision = Car_o.controlColision(lines)
			if colision == 'Inside':
				print('Auto narazilo do prekazky')
				return
			Car_o.plotCar()
			self.plotWorld(self.rectangles)
			plt.pause(0.05)
		R = Car_o.rozvor/(np.tan(Car_o.delta_s_max))
		X0 = ParkPoint[0]
		X = Car_o.Sx - X0
		Dp = self.delka + 0.2
#		Yd = Car_o.Sy - ParkPoint[1] + Car_o.rozvor/2
		Yd = X
#		Rf = (X**2+Yd**2)/(2*X)
		Rf = X
		G = [X0, Car_o.Sy - Yd]
		F = [X0, ParkPoint[1]-Dp + 0.7] # posledni clen - vdalenost zadnich poloos od konce auta + rezerva na zaparkovani
		O = [X0 + Rf, G[1]]
		delta_s_optimal = np.arctan(Car_o.rozvor/Rf)

		if Rf < R:
			print('Najed si s vetsim odstupem')
			print('Potrebny minimalni polomer je:',Rf)
			print('Minimalni mozny polomer je:', R)
			plt.text(4, 10, 'Trajektorie nenalezana - najeď jinak', style='oblique',fontsize = 30,
        		bbox={'facecolor':'green', 'alpha':0.5, 'pad':10})
			plt.pause(4)
			return			
		self.plotTransverse(G, O, F, Rf)
		plt.text(4, 10, 'Ideální trajektorie', style='oblique',fontsize = 30,
        	bbox={'facecolor':'green', 'alpha':0.5, 'pad':10})
		plt.pause(4)
		while abs(delta_s_optimal - abs(Car_o.delta_s)) > 0.003:
			plt.cla()
			wheelAngularRotation = 0;
			delta_s_dot = -0.028
			Car_o = Car(*Car_o.drive(wheelAngularRotation, delta_s_dot))
			colision = Car_o.controlColision(lines)
			if colision == 'Inside':
				print('Auto narazilo do prekazky')
				return
			Car_o.plotCar()
			self.plotTransverse(G, O, F, Rf)
			self.plotWorld(self.rectangles)
			plt.pause(0.05)

		while abs(math.sqrt(abs(G[0]-Car_o.Sx)**2) + abs(Car_o.Sy-G[1])**2) > 0.025:
			plt.cla()
			wheelAngularRotation = -1.25;
			delta_s_dot = 0
			Car_o = Car(*Car_o.drive(wheelAngularRotation, delta_s_dot))
			colision = Car_o.controlColision(lines)
			if colision == 'Inside':
				print('Auto narazilo do prekazky')
				return
			Car_o.plotCar()
			self.plotTransverse(G, O, F, Rf)
			self.plotWorld(self.rectangles)
			plt.pause(0.05)

		while abs(Car_o.delta_s) > 0.007:
			plt.cla()
			wheelAngularRotation = 0;
			delta_s_dot = 0.06
			Car_o = Car(*Car_o.drive(wheelAngularRotation, delta_s_dot))
			colision = Car_o.controlColision(lines)
			if colision == 'Inside':
				print('Auto narazilo do prekazky')
				return
			Car_o.plotCar()
			self.plotTransverse(G, O, F, Rf)
			self.plotWorld(self.rectangles)
			plt.pause(0.05)

		while abs(math.sqrt(abs(F[0]-Car_o.Sx)**2) + abs(Car_o.Sy-F[1])**2) > 0.05:
			plt.cla()
			wheelAngularRotation = -1.3;
			delta_s_dot = 0
			Car_o = Car(*Car_o.drive(wheelAngularRotation, delta_s_dot))
			colision = Car_o.controlColision(lines)
			if colision == 'Inside':
				print('Auto narazilo do prekazky')
				return
			Car_o.plotCar()
			self.plotTransverse(G, O, F, Rf)
			self.plotWorld(self.rectangles)
			plt.pause(0.05)

		self.parkingPlaceSpotted = 0
		self.place = 0
		print('Parked')
		plt.text(4, 10, 'Zaparkováno', style='oblique',fontsize = 30,
        	bbox={'facecolor':'green', 'alpha':0.5, 'pad':10})
		plt.pause(4)
		
	def plotParallel(self,F, Oi, Of, M, Rf):
		plt.plot(F[0],F[1],'ko',markersize=12)
		plt.plot(Oi[0], Oi[1],'ko',markersize=12)
		plt.plot(Of[0], Of[1],'ko',markersize=12)
		plt.plot(M[0], M[1],'ko',markersize=12)
		plt.gca().add_patch(plt.Circle(Oi, radius = Rf, color = 'brown', fill = False, linewidth = 3))
		plt.gca().add_patch(plt.Circle(Of, radius = Rf, color = 'brown', fill = False, linewidth = 3))

	def plotTransverse(self, G, O, F, Rf):
		plt.plot(G[0],G[1],'ko',markersize=12)
		plt.plot(O[0], O[1],'ko',markersize=12)
		plt.plot(F[0], F[1], 'ko',markersize=12)
		plt.gca().add_patch(plt.Circle(O, radius = Rf, color = 'brown', fill = False, linewidth = 3))

	def plotWorld(self, rectangles):
		for rec in rectangles:
			plt.gca().add_patch(rec)

