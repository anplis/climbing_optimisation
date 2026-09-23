import matplotlib.pyplot as plt
import math as ma

T = 10  # Durée totale de la simulation en secondes
dt = 0.1
chronologie = [i*dt for i in range(int(round(T//dt)))]  # Liste des instants de temps

def poid(m):
    g = 9.81
    return [0, -m*g]
        
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
size = 30
# Création de la figure
fig, ax = plt.subplots()
ax.set_xlim(-size, size)
ax.set_ylim(-size, size)
ax.set_aspect("equal")

# Active le mode interactif
plt.ion()
plt.show()

class PointViewer:
    def __init__(self, point: Point):
        self.point = point

        # Scatter pour afficher le point
        self.scatter = ax.scatter([point.x], [point.y], s=80, color="blue")

    def update(self):
        self.scatter.set_offsets([[self.point.x, self.point.y]])

class solid:
    def __init__(self, teta, l, m, x, y):
        self.x = x
        self.y = y
        self.teta = teta
        self.vx = 0
        self.vy = 0
        self.omega = 0
        
        self.m = m
        self.l = l
        
        self.p1 = Point(x + (self.l/2)*ma.cos(teta), y + (self.l/2)*ma.sin(teta))
        self.p2 = Point(x - (self.l/2)*ma.cos(teta), y - (self.l/2)*ma.sin(teta))
        
        self.viewer_1 = PointViewer(self.p1)
        self.viewer_2 = PointViewer(self.p2)
        
        self.line, = ax.plot(
            [self.p1.x, self.p2.x],
            [self.p1.y, self.p2.y],
            color="blue", linewidth=2)
        
    def move(self, dx, dy):
        self.x += dx
        self.y += dy
        self.vx = dx/dt
        self.vy = dy/dt
    
    def rotate(self, dteta):
        self.teta += dteta
        self.omega = dteta/dt
    
    def force(self, F): # F = [Fx, Fy]  
        a = [F[0]/self.m, F[1]/self.m]
        self.move(0.5*a[0]*dt**2 + self.vx*dt, 0.5*a[1]*dt**2 + self.vy*dt)
    
    def moment(self, M):
        a_angl = [M/((self.m*(self.l)**2)/12)]   # (self.m*(self.l)**2)/12 étant l'intertie d'une tige uniforme
        self.rotate(0.5*a_angl[0]*dt**2 + self.omega*dt)
    
    def update(self):
        self.p1.x = self.x + (self.l/2)*ma.cos(self.teta)
        self.p1.y = self.y + (self.l/2)*ma.sin(self.teta)
        self.p2.x = self.x - (self.l/2)*ma.cos(self.teta)
        self.p2.y = self.y - (self.l/2)*ma.sin(self.teta)
        
        self.line.set_data(
                    [self.p1.x, self.p2.x],
                    [self.p1.y, self.p2.y])
        
        self.viewer_1.update()
        self.viewer_2.update()

        fig.canvas.draw()
        fig.canvas.flush_events() 
        
        
teta = 0
l = 5
m = 0.5

s = solid(teta, l, m, 0, 0)

set_force = [[{'F' : [0,10], 'ini_t' : 2, 'durée' : 2}], []]    # Forces appliquées sur p1 puis p2 (au point p1 et p2)
set_global_force = [{'F' : poid, 'ini_t' : 0, 'durée' : T}]    # Forces appliquées sur le solide s (au centre du solide)

if __name__ == "__main__":
    
    # Déplacements successifs
    for t in chronologie:
        plt.pause(dt)
        
        force_p = [[0,0],[0,0]] # Sommes des force des deux points p1 et p2
        force_s = [0,0]         # Force résultante du solide s
        moment_s = 0            # Moment du solide s
        for i in range(2):
            for force in set_force[i]:
                if t <= force['ini_t']+force['durée'] and t >= force['ini_t']:
                    Fx,Fy = force['F']
                    force_p[i][0] += Fx
                    force_p[i][1] += Fy
        
            force_s[0] += force_p[i][0] # Somme des forces selon x du point p
            force_s[1] += force_p[i][1] # Somme des forces selon y du point p
            
            moment_s += ((-1)**i)*(s.l/2)*(ma.cos(s.teta)*force_p[i][1] - ma.sin(s.teta)*force_p[i][0])   # Produit vectoriel entre vecteur position et la force appliquée au point p

        
        for force in set_global_force:  # Les forces globals s'appliquent sur le solide donc pas de moment induit
            Fx,Fy = force['F'](s.m)
            force_s[0] += Fx
            force_s[1] += Fy
            
        s.force(force_s)
        
        s.moment(moment_s)
        
        s.update()
