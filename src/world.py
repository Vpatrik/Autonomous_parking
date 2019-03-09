#!/usr/bin/env python

import numpy as np
from numpy import pi
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from CarMaker import *
import matplotlib as mp
from Trajectory import *

class World:

	def __init__(self, sirka = 38 , delka = 13): # m
		
		self.sirka, self.delka = sirka, delka
		self.rectangle = plt.Rectangle((0,0), self.sirka, self.delka, 0, fill = False)

	def SpawnCar(self, Sx, Sy, uhel, uhel_kol):
		Clio_p = Car(Sx, Sy, uhel, uhel_kol)
		Clio_p.plotCar()
		return Clio_p

	def plot(self):

		plt.gca().add_patch(self.rectangle)
		return self.rectangle

	def getLines(self):
		obrys = self.rectangle.get_bbox().corners()
		l = np.array([[obrys[0,0],obrys[0,1], obrys[1,0], obrys[1,1]], \
		[obrys[0,0],obrys[0,1], obrys[2,0], obrys[2,1]], \
		[obrys[1,0],obrys[1,1], obrys[3,0], obrys[3,1]], \
		[obrys[2,0],obrys[2,1], obrys[3,0], obrys[3,1]]])
		return l
		

class obstacles:

	def __init__(self, sirka = 1.7, delka = 4, pozice = [0,0], uhel = 0):
		self.sirka, self.delka, self.pozice, self.uhel = sirka, delka, pozice, uhel
		self.rectangle = plt.Rectangle((self.pozice), self.sirka, self.delka, self.uhel*180/pi, ec='black',facecolor= 'red', linewidth=2.5)

	def plot(self):

		plt.gca().add_patch(self.rectangle)
		return self.rectangle

	def getLines(self):
		corners = self.rectangle.get_bbox().corners()
		b = np.vstack([corners[:,0]-self.pozice[0], corners[:,1] - self.pozice[1]])
		obrys = np.array(b.transpose().dot(np.array([[np.cos(self.uhel), np.sin(self.uhel)], [-np.sin(self.uhel), np.cos(self.uhel)]]))) + \
		[self.pozice[0],self.pozice[1]]
		l = np.array([[obrys[0,0],obrys[0,1], obrys[1,0], obrys[1,1]], \
		[obrys[0,0],obrys[0,1], obrys[2,0], obrys[2,1]], \
		[obrys[1,0],obrys[1,1], obrys[3,0], obrys[3,1]], \
		[obrys[2,0],obrys[2,1], obrys[3,0], obrys[3,1]]])
		return l




