import matplotlib.pyplot as plt
import keyboard as key

T = 10  # Durée totale de la simulation en secondes
dt = 0.1

chronologie = [i*dt for i in range(int(T//dt))]  # Liste des instants de temps
set_forces = [{'F' : [3,0], 'ini_t' : 0, 'durée' : 2},
              {'F' : [0,-1], 'ini_t' : 0, 'durée' : T},
              {'F' : [2,2], 'ini_t' : 2, 'durée' : 2}]
        
        
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.m = 10
        self.vx = 0
        self.vy = 0

    def move(self, dx, dy):
        self.x += dx
        self.y += dy
        self.vx = dx/dt
        self.vy = dy/dt
    
    def force(self, F): # F = [Fx, Fy]  
        a = [F[0]/self.m, F[1]/self.m]
        self.move(a[0]*dt**2 + self.vx*dt, a[1]*dt**2 + self.vy*dt)


class PointViewer:
    def __init__(self, point: Point):
        self.point = point

        # Création de la figure
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_aspect("equal")

        # Scatter pour afficher le point
        self.scatter = self.ax.scatter([point.x], [point.y], s=80, color="blue")

        # Active le mode interactif
        plt.ion()
        plt.show()

    def update(self):
        self.scatter.set_offsets([[self.point.x, self.point.y]])
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

p = Point(0, 0)
if __name__ == "__main__":
    viewer = PointViewer(p)

    # Déplacements successifs
    for t in chronologie:
        plt.pause(dt)
        for force in set_forces:
            if t <= force['ini_t']+force['durée'] and t >= force['ini_t']:
                p.force(force['F'])
        viewer.update()

