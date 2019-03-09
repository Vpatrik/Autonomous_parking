#!/usr/bin/env python

import numpy as np
from numpy import pi
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from linecross import *

## Definice parametru pro Clio II
#rozvor = 2480# mm
#rozchod = 1406 # mm
#sirka = 1650 # mm
#delka = 3820 # mm
#D_kola = 130*2.54 # mm
#minimal_D_otaceni = 11 # m
#sirka_kol = 155 #mm

# timeStep = 0.1 s


class Car:
	
	def __init__(self,Sx = 10.0, Sy = 5.0, fi = pi/2, delta_s = 0.0, rozvor = 2.480, rozchod = 1.406, sirka = 1.650, delka = 3.820, D_kola = 130*2.54/1000,
		minimal_R_otaceni = 11/2, sirka_kol = 0.155, timeStep = 0.2):
		self.rozvor = rozvor
		self.rozchod = rozchod
		self.sirka = sirka
		self.delka = delka
		self.D_kola = D_kola
		self.minR= minimal_R_otaceni
		self.sirka_kol = sirka_kol
		self.timeStep = timeStep
		self.delta1_max = np.arcsin(self.rozvor/(self.minR)) # rad
		self.delta2_max = np.arctan(self.rozvor/(math.sqrt((self.minR)**2 - self.rozvor**2)-self.rozchod))
		self.delta_s_max = np.arctan((2*np.tan(self.delta1_max)*np.tan(self.delta2_max))/(np.tan(self.delta1_max)+np.tan(self.delta2_max)))
		self.Sx,self.Sy = Sx, Sy
		self.fi = fi
		self.delta_s = delta_s
		Wheel1 = self.Sx - np.cos(self.fi)*self.rozchod/2, self.Sy - np.sin(self.fi)*self.rozchod/2
		Wheel2 = self.Sx + np.cos(self.fi)*self.rozchod/2, self.Sy + np.sin(self.fi)*self.rozchod/2
		Wheel3 = self.Sx + np.cos(self.fi)*self.rozchod/2 - np.sin(self.fi)*self.rozvor, self.Sy + np.sin(self.fi)*self.rozchod/2 + np.cos(self.fi)*self.rozvor
		Wheel4 = self.Sx - np.cos(self.fi)*self.rozchod/2 - np.sin(self.fi)*self.rozvor, self.Sy - np.sin(self.fi)*self.rozchod/2 + np.cos(self.fi)*self.rozvor
		self.Wheels = [Wheel1, Wheel2, Wheel3, Wheel4]
		self.Ax, self.Ay = self.Sx - self.sirka/2*np.cos(self.fi) + 0.5*np.sin(self.fi), self.Sy - self.sirka/2*np.sin(self.fi) - 0.5*np.cos(self.fi)
		self.rectangle = plt.Rectangle((self.Ax, self.Ay), self.sirka, self.delka, (self.fi)*180/pi, ec='blue',facecolor= 'silver', linewidth=2.5,  gid = 'car')
		obrys = np.array(self.rectangle.get_bbox().corners())
		b = np.vstack([obrys[:,0]-self.Ax, obrys[:,1] - self.Ay])
		self.obrys = np.array(b.transpose().dot(np.array([[np.cos(self.fi), np.sin(self.fi)], [-np.sin(self.fi), np.cos(self.fi)]]))) + \
		[self.Ax,self.Ay]
		self.stred = np.array([self.Sx-self.rozvor/2*np.sin(self.fi), self.Sy + self.rozvor/2*np.cos(self.fi)])
		self.roh = math.sqrt((self.sirka/2)**2 + (self.delka/2)**2)


	def plotCar(self):
		i = 0
		if self.delta_s > self.delta_s_max:
			self.delta_s = self.delta_s_max
		elif self.delta_s < -self.delta_s_max:
			self.delta_s = -self.delta_s_max
		

		fig = plt.gcf()
		plt.axis('equal')
		plt.axis((-0.5,38.5,0,13))

		plt.gca().add_patch(self.rectangle)


			
		for wheel in self.Wheels:
			if (i == 2 and self.delta_s > 0.0001) or (i == 2 and self.delta_s < -0.0001):
				delta = np.arctan(1/(1/np.tan(self.delta_s)+self.rozchod/(2*self.rozvor)))
			elif (i == 3 and self.delta_s < -0.001) or (i == 3 and self.delta_s > 0.0001):
				delta = np.arctan(1/(1/np.tan(self.delta_s)-self.rozchod/(2*self.rozvor)))
			else:
				delta = 0
			if delta == 0:
				delta = 0.000001
			R = abs(self.rozvor/np.sin(delta))

			if delta == 0.000001 and (self.delta_s > 0.0001 or self.delta_s < - 0.0001):
				R = abs(self.rozvor/np.tan(self.delta_s)) - self.rozchod/2
			if R > 10:
				R = 10
				
			if self.delta_s > 0.0001:	 
				Rx = wheel[0] - R*np.cos(self.fi+delta)
				Ry = wheel[1] -  R*np.sin(self.fi+delta) 
				plt.plot([Rx,wheel[0]],[Ry,wheel[1]],'-g')
			
			elif self.delta_s < -0.0001:
				Rx = wheel[0] + R*np.cos(self.fi+delta)
				Ry = wheel[1]  + R*np.sin(self.fi+delta) 
				plt.plot([Rx,wheel[0]],[Ry,wheel[1]],'-g')


			wheel = wheel[0] - self.sirka_kol/2*np.cos(self.fi+delta) + self.D_kola/2*np.sin(self.fi+delta), wheel[1] -\
			self.sirka_kol/2*np.sin(self.fi+delta) - self.D_kola/2*np.cos(self.fi+delta)
			plt.axes()
			rectangleW = plt.Rectangle(wheel, self.sirka_kol, self.D_kola, (self.fi+delta)*180/pi, facecolor= 'black')
			plt.gca().add_patch(rectangleW)

			i+=1

	def moveForward(self, wheelAngularRotation):
		self.Sx -= wheelAngularRotation*self.D_kola/2*np.sin(self.fi)*self.timeStep
		self.Sy += wheelAngularRotation*self.D_kola/2*np.cos(self.fi)*self.timeStep
		return self.Sx, self.Sy, self.fi, self.delta_s, self.rozvor, self.rozchod, self.sirka, self.delka, self.D_kola, self.minR, self.sirka_kol, \
		self.timeStep
	

	def moveBackward(self, wheelAngularRotation):
		self.Sx += wheelAngularRotation*self.D_kola/2*np.sin(self.fi)*self.timeStep
		self.Sy -= wheelAngularRotation*self.D_kola/2*np.cos(self.fi)*self.timeStep
		return self.Sx, self.Sy, self.fi, self.delta_s, self.rozvor, self.rozchod, self.sirka, self.delka, self.D_kola, self.minR, self.sirka_kol, \
		self.timeStep

	def turnWheels(self,delta_s_dot, direction):
		if direction == 'clockwise':
			self.delta_s -= delta_s_dot*self.timeStep
		elif direction == 'anticlockwise':
			self.delta_s += delta_s_dot*self.timeStep
		else:
			print('Wrong direction was chosen')
		if self.delta_s > self.delta_s_max:
			self.delta_s = self.delta_s_max
		elif self.delta_s < -self.delta_s_max:
			self.delta_s = -self.delta_s_max
		return self.Sx, self.Sy, self.fi, self.delta_s, self.rozvor, self.rozchod, self.sirka, self.delka, self.D_kola, self.minR, self.sirka_kol, \
		self.timeStep

	def drive(self,wheelAngularRotation, delta_s_dot):
		self.delta_s += delta_s_dot*self.timeStep
		if self.delta_s > self.delta_s_max:
			self.delta_s = self.delta_s_max
		elif self.delta_s < -self.delta_s_max:
			self.delta_s = -self.delta_s_max
		self.fi += np.tan(self.delta_s)*wheelAngularRotation*self.D_kola*self.timeStep/(2*self.rozvor)
		self.Sx -= (wheelAngularRotation*self.D_kola/2)*np.sin(self.fi)*self.timeStep
		self.Sy += (wheelAngularRotation*self.D_kola/2)*np.cos(self.fi)*self.timeStep
		return self.Sx, self.Sy, self.fi, self.delta_s, self.rozvor, self.rozchod, self.sirka, self.delka, self.D_kola, self.minR, self.sirka_kol, \
		self.timeStep
	
	def scanObstacles(self, lines):

		points = np.zeros((50,2))
		i = 0
		for alfa in np.linspace(0,2*pi,50):
			for line in lines:
				if max(self.obrys[:,0]) + 0.5 < min(line[0], line[2]) or min(self.obrys[:,0]) > max(line[0], line[2]) + 0.5:
					continue
				if abs(min(self.obrys[:,1]) - max(line[1], line[3])) > 2.6 and min(self.obrys[:,1]) > max(line[1], line[3]):

					continue
				beam = SensorBeam(self.stred, self.fi+alfa, line, 3.6) # posledni argument - dosah scanneru
				if intersect(beam) == True:
					vz, xp, yp = GetDistance(beam)
					plt.plot([beam.Ax,xp],[beam.Ay,yp],'r-')
					if vz < self.roh:
						points[i,:] = xp, yp
						i +=1
							

		return points	

	def controlColision(self, lines):
		args = self.scanObstacles(lines)
	# test for colision
		if args is not None:
			cross = 'Outside'
			for arg in args:
				if (arg[0] + arg [1]) > 0.1:
					bax = self.obrys[2,0] - self.obrys[0,0]
					bay = self.obrys[2,1] - self.obrys[0,1]
					dax = self.obrys[3,0] - self.obrys[0,0]
					day = self.obrys[3,1] - self.obrys[0,1]
					if ((arg[0] - self.obrys[0,0]) * bax + (arg[1] - self.obrys[0,1]) * bay < 0.0): 
						continue
					if ((arg[0] - self.obrys[2,0]) * bax + (arg[1] - self.obrys[2,1]) * bay > 0.0):
						continue
					if ((arg[0] - self.obrys[0,0]) * dax + (arg[1] - self.obrys[0,1]) * day < 0.0):
						continue
					if ((arg[0] - self.obrys[3,0]) * dax + (arg[1] - self.obrys[3,1]) * day > 0.0):
						continue

					cross = 'Inside'
				
			return cross

	def spotParkingPlace(self, lines, ParkingPlace):
		x,y, vz0 = 0,0,10
		if ParkingPlace == 0:
			for line in lines:
				if max(self.obrys[:,0]) < min(line[0], line[2]) or min(self.obrys[:,0]) > max(line[0], line[2]):
					continue
				
				if abs(min(self.obrys[:,1]) - max(line[1], line[3])) > 2.6 and min(self.obrys[:,1]) > max(line[1], line[3]):
					continue
#				print(line)
				beam = SensorBeam([self.Sx, self.Sy], self.fi, line, 3.6) # posledni argument - dosah scanneru
				if intersect(beam) == True:
					vz, xp, yp = GetDistance(beam)
					if vz < vz0:
						x,y,vz0 = xp, yp, vz
			if x is not 0:			
				return x, y


					
			
			

