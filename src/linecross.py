import math
import numpy as np


class SensorBeam:
	def __init__(self, Position, fi, line, SensorDistance = 2.5 ):
		self.Ax, self.Ay = Position[0], Position[1],
		self.Bx, self.By = self.Ax + SensorDistance*np.cos(fi), self.Ay + SensorDistance*np.sin(fi)
		self.Cx, self.Cy, self.Dx, self.Dy = line[0], line[1], line[2], line[3]
		

def GetDistance(Beam):
# vraci vzdalenost beamu AB s pocatkem v A od usecky CD

	dist = -1; #no crossection
	safetyZone = 0.01;

    #prvni primka
	t = Beam.Bx-Beam.Ax;
	if t==0:
		k1 = 100000;
	else:
		k1 = (Beam.By-Beam.Ay)/t;
	q1 = Beam.Ay - (k1 * Beam.Ax);

    # druha primka
	t = Beam.Dx-Beam.Cx;
	if t==0:
		k2 = 100000;
	else:
		k2 = (Beam.Dy-Beam.Cy)/t;
	q2 = Beam.Cy - (k2 * Beam.Cx);

	t2 = k2 - k1;
	if (abs(t2) < 0.0001):
		yp = 100000;
		xp = 100000;
	else:
		yp = (q1*k2-q2*k1) / t2;
		xp = (q1-q2) / t2;

    #specialni pripady

	if (Beam.Bx - Beam.Ax) == 0:
		xp = Beam.Ax;
		yp = k2*xp+q2;
	if (Beam.Dx-Beam.Cx) == 0:
		xp = Beam.Cx;
		yp = k1*xp+q1;

	minx1 = min(Beam.Ax,Beam.Bx);
	maxx1 = max(Beam.Ax, Beam.Bx);
	miny1 = min(Beam.Ay, Beam.By);
	maxy1 = max(Beam.Ay, Beam.By);
	minx2 = min(Beam.Cx, Beam.Dx);
	maxx2 = max(Beam.Cx, Beam.Dx);
	miny2 = min(Beam.Cy, Beam.Dy);
	maxy2 = max(Beam.Cy, Beam.Dy);

	if ((xp + safetyZone < minx1) or 
        (xp - safetyZone > maxx1) or 
        (yp + safetyZone < miny1) or 
        (yp - safetyZone > maxy1) or 
        (xp + safetyZone < minx2) or 
        (xp - safetyZone > maxx2) or 
        (yp + safetyZone < miny2) or 
        (yp - safetyZone > maxy2)):
		dist = -1;
	else:
		dist = math.sqrt((xp-Beam.Ax)**2 + (yp-Beam.Ay)**2);
    
	return dist, xp, yp


def cross(Ax,Ay,Bx,By,Cx,Cy):
    return (Cy-Ay) * (Bx-Ax) > (By-Ay) * (Cx-Ax)

# Return true if line segments AB and CD intersect
def intersect(bm):
    return cross(bm.Ax,bm.Ay,bm.Cx,bm.Cy,bm.Dx,bm.Dy) != cross(bm.Bx,bm.By,bm.Cx,bm.Cy,bm.Dx,bm.Dy) and cross(bm.Ax,bm.Ay,bm.Bx,bm.By,bm.Cx,bm.Cy) != cross(bm.Ax,bm.Ay,bm.Bx,bm.By,bm.Dx,bm.Dy)



