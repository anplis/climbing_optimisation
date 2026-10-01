"""
En escalade, on peut distinguer deux types de mouvements : 
 - les mouvements de replacement où on se repositionne pour améliorer le placement de son centre de gravité ou se préparer pour le prochain mouvement
 - les movement de déplacment où le(s) membre(s) quitte(nt) une prise pour en attraper une autre afin d'avancer ou pour optimiser son énérgie en lachant une prise inutile à tenir

On modélisera le corps du grimpeur par un ensemble de 5 points et 4 liaisons :
 - le point centrale, noté G car aussi le centre de gravité (par approximation), qui représente le bassin/le dos
 - 4 points pour les mains et les pieds
 - 4 liaisons pour lier ces mains et pieds avec le point centrale, chaque liaisons est équivalent cinématiquement à une liaison pivot et une liaison glissiére placés en séries sur le même axe centrale qui est le vecteur entre la/le main/pied et le point G

Ainsi, un grimpeur pourra faire le mouvement (que l'on effectura seulement si ils sont possible pour le corps) :
 - Déplacer le point centrale du corps G 
 - Déplacer un pied ou une main qui n'est accrochée à une prise

Autres caractéristiques du modèle :
 - Le grimpeur doit toujours avoir au moins 2 points d'accroche au mur (ce qui est réaliste pour la plupart des mouvements en escalade)
"""

import matplotlib.pyplot as plt
import math as ma

def distance(XY, AB):
    x = XY[0] - AB[0]
    y = XY[1] - AB[1]
    return ma.sqrt(x**2 + y**2)

def angle_in(teta, a, b):   # si teta est dans a,b modulo 2pi
    return (teta - a) % (2*ma.pi) <= (b - a) % (2*ma.pi)

# Interval_l = [0, l_max]
# Interval_teta = [teta_min, teta_max]

class point :
    def __init__(self, x, y, l, teta, Interval_l, Interval_teta):
        self.hold = False
        self.x = x
        self.y = y
        self.l = l
        self.teta = teta
        
        self.Interval_l = Interval_l
        self.Interval_teta = Interval_teta
        
        self.scatter = ax.scatter([self.x], [self.y], s=40, color="blue")
    
    def update_graph(self):
        self.scatter.set_offsets([self.x, self.y])
    
    def move(self, F):
        self.hold = False
        self.x += F[0]
        self.y += F[1]
    
    def hold_hold(self, eps = 0.1):
        if any(distance(h, (self.x, self.y)) <= eps for h in Holds):
            self.hold = True
        else:
            print('trop loin')
    
    def drop_hold(self):
        self.hold = False
    
        
class climber :
    def __init__(self, x = 0, y = 0 ):
        self.G = [x, y]
        self.list_point = []
        
        # create a graph point
        self.scatter = ax.scatter([self.G[0]], [self.G[1]], s=40, color="red")  
    
    def add_point(self, l, teta, Interval_l, Interval_teta):
        x, y = self.G[0] + l*ma.cos(teta), self.G[1] + l*ma.sin(teta)
        self.list_point.append(point(x,  y, l , teta, Interval_l, Interval_teta))  
        
        # create a graph line between G and the new point
        i = len(self.list_point) - 1    # index of the point added
        self.list_point[i].line, = ax.plot([self.G[0], self.list_point[i].x], [self.G[1], self.list_point[i].y], color="blue", linewidth=2)
    
    
    def move_G(self, F):
        if self.is_possible_move_G(F):
            self.G[0] += F[0]
            self.G[1] += F[1]
            
            for point in self.list_point:
                x = point.x - self.G[0]
                y = point.y - self.G[1]
                point.l = ma.sqrt(x**2 + y**2)
                point.teta = ma.atan2(y,x + 10**(-10)) 
        else:
            print('not possible')
    

    def move_p(self, index_point, F):
        if self.if_possible_move_p(self.list_point[index_point], F):
            self.list_point[index_point].move(F)    # met à jour coord du point
            
            x = self.list_point[index_point].x - self.G[0]  # coord du point par rapport à G
            y = self.list_point[index_point].y - self.G[1]
            
            self.list_point[index_point].l = ma.sqrt(x**2 + y**2)   # met à jour l et teta du point
            self.list_point[index_point].teta = ma.atan2(y,x) 
        else:
            print('not possible')
    
    def is_possible_move_G(self, F):
        G_new = [self.G[0] + F[0], self.G[1] + F[1]]
        return all(self.is_valid(p, G_new, (p.x, p.y)) for p in self.list_point)
    
    def if_possible_move_p(self, p, F):
        nb_hold = sum(1 for q in self.list_point if q.hold)
        if p.hold and nb_hold <= 2:
            return False
        return self.is_valid(p, self.G, (p.x + F[0], p.y + F[1]))
    
    def is_valid(self, p, G, pos):
        x, y = pos[0] - G[0], pos[1] - G[1]
        l = ma.hypot(x, y)
        teta = ma.atan2(y, x)
        return (p.Interval_l[0] <= l <= p.Interval_l[1] and angle_in(teta, p.Interval_teta[0], p.Interval_teta[1]))
    
    def update_graph(self):
        self.scatter.set_offsets([self.G[0], self.G[1]])                    # update G
        
        for point in self.list_point :
            point.update_graph()                                            # update points
            point.line.set_data([self.G[0], point.x], [self.G[1], point.y])  # update lines
        
        fig.canvas.draw()
        fig.canvas.flush_events()

def add_F(Seq, T, p, F):
    if T < time[-1]:
        if T not in Seq:
            Seq[T] = [{"p" : p, "F" : F}]
        else:
            Seq[T].append({"p" : p, "F" : F})

# ---------------MAIN--------------- #

# Création de la figure :
size = 3
fig, ax = plt.subplots()
ax.set_xlim(-size, size)
ax.set_ylim(-size, size)
ax.set_aspect("equal")

plt.ion()   # mode interactif 
plt.show()

# test :

C = climber()
Holds = []

Interval_l = [0, 1]
l = 0.5
teta = ma.pi/4

for i in range(4):
    Interval_teta = [teta - ma.pi/2, teta + ma.pi/2]
    C.add_point(l,teta, Interval_l, Interval_teta)
    
    x, y = C.list_point[i].x, C.list_point[i].y
    Holds.append((x,y))
    teta += ma.pi/2

T = 5  # Durée totale de la simulation en secondes
dt = 0.1
time = [i*dt for i in range(int(round(T/dt)))]  # Liste des instants de temps

Seq = {}    # Seq = {t : [{"p": index_p or None, "F" : F}]}, None is for G move
add_F(Seq,1,0,(0.2,0.4))
add_F(Seq,3,None,(0.2,-0.1))

for hold in Holds:
    plt.scatter(hold[0],hold[1], color = "green")
for p in C.list_point:
    p.hold_hold()
    
for t in time:
    plt.pause(dt)
    for T in Seq.keys():
        if abs(T-t) < dt/2:
            for move in Seq[T]:
                if move['p'] is None:
                    C.move_G(move["F"])
                else:
                    C.move_p(move['p'], move["F"])
    C.update_graph()
