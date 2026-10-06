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
 - Déplacer un pied ou une main 

Autres caractéristiques du modèle :
 - Le grimpeur doit toujours avoir au moins 2 points d'accroche au mur (ce qui est réaliste pour la plupart des mouvements en escalade)
 - On considère une grimpe dite statique, sans mouvement dynamique
"""

# ---------------IMPORT--------------- #


import matplotlib.pyplot as plt
import math as ma
import random as rd
import copy as copy


# ---------------CLASS--------------- #

# Interval_l = [0, l_max]
# Interval_teta = [teta_min, teta_max]

class point :
    def __init__(self, x, y, l, teta, Interval_l, Interval_teta, do_graph=False):
        self.hold = False
        self.x = x
        self.y = y
        self.l = l
        self.teta = teta
        
        self.Interval_l = Interval_l
        self.Interval_teta = Interval_teta
        
        if do_graph:
            self.scatter = ax.scatter([self.x], [self.y], s=40, color="blue")
    
    def update_graph(self):
        self.scatter.set_offsets([self.x, self.y])
    
    def move(self, F):
        self.hold = False
        self.x += F[0]
        self.y += F[1]
    
    def hold_hold(self):
        if any(distance(h, (self.x, self.y)) <= eps for h in Holds):
            self.hold = True
    
    def drop_hold(self):
        self.hold = False
    
        
class climber :
    def __init__(self, x = 0., y = 0. , do_graph = False):
        self.fit = float('inf')
        self.Seq = {}   # Seq = {t : {"p": index_p or None, "F" : F}}, None is for G move
        
        self.G = [x, y]
        self.list_point = []
        
        self.do_graph = do_graph
        if do_graph : # create a graph point
            self.scatter = ax.scatter([self.G[0]], [self.G[1]], s=40, color="red")  
    
    def reset(self):
        reset_all_points(self)
    
    def add_F(self, T, p, F):
        if T <= time[-1]:
            self.Seq[T] = {"p" : p, "F" : F}
    
    def add_F_rd(self):
        T = rd.choice(range(len(time)))
        if rd.random() > 0.5:   # move G
            F = [rd.uniform(-max_F, max_F) for _ in range(2)]
            self.add_F(T, None, F)
        else:   # move point to hold
            p_index = rd.choice(range(len(self.list_point)))
            if self.list_point[p_index].hold and sum(q.hold for q in self.list_point) <= 2: # si pas assez de points accrochés
                pass
            possible_hold_F = self.possible_move_to_hold(T)
            if possible_hold_F:
                F = rd.choice(possible_hold_F)
                self.add_F(T, p_index, F)
    
    def possible_move_to_hold(self, T_new):
        # On simule la grimpe jusqu'au temps T_new
        self.reset()
        try:
            for T in sorted(self.Seq):
                if T >= T_new:
                    break
            
                self.apply_move(self.Seq[T])
        except NameError:       # arrêt au premier mouvement impossible
            pass
        
        p = rd.choice(self.list_point)
        return [[h[0] - p.x, h[1] - p.y] for h in Holds if distance(h, (p.x, p.y)) > 1e-9 and self.is_valid(p, self.G, h)]  # 1e-9 pour pas que ce soit la même prise
    
    def del_F_rd(self):    # supprime un mouvement au hasard
        Ts = [T for T in self.Seq if self.Seq[T]]      # instants qui ont encore des mouvements
        if Ts:
            T = rd.choice(Ts)
            self.Seq.pop(T)  # retire un seul mouvement
    
    def change_move_rd(self):
        if self.Seq:
            T = rd.choice(list(self.Seq.keys()))

            if rd.random() < 0.5:   # change le temps du mouvement
                r = max(1, len(time)//10)  # arbitraire, correspond à 10% de la durée totale de la simulation
                new_T = rd.choice(range(max(0, T-r), min(len(time), T+r)))
                if new_T != T:
                    self.Seq[new_T] = self.Seq[T]
                    self.Seq.pop(T)
            else:   # change le vecteur de déplacement du mouvement
                move = self.Seq[T]
                self.Seq[T]["F"] = [f + rd.uniform(-0.1, 0.1)*max_F for f in move["F"]]
        
    
    def add_point(self, l, teta, Interval_l, Interval_teta):
        x, y = self.G[0] + l*ma.cos(teta), self.G[1] + l*ma.sin(teta)
        self.list_point.append(point(x,  y, l , teta, Interval_l, Interval_teta, self.do_graph))  
        
        if self.do_graph:
            # create a graph line between G and the new point
            i = len(self.list_point) - 1    # index of the point added
            self.list_point[i].line, = ax.plot([self.G[0], self.list_point[i].x], [self.G[1], self.list_point[i].y], color="blue", linewidth=2)
    
    def apply_move(self, move):
        if move['p'] is None:
            self.move_G(move['F'])
        else:
            self.move_p(move['p'],move['F'])
    
    def move_G(self, F):
        if self.is_possible_move_G(F):
            self.G[0] += F[0]
            self.G[1] += F[1]
            
            if self.do_graph:
                for point in self.list_point:
                    x = point.x - self.G[0]
                    y = point.y - self.G[1]
                    point.l = ma.hypot(x, y)
                    point.teta = ma.atan2(y,x) 
        else:
            raise NameError('not possible')
    

    def move_p(self, index_point, F):
        if self.if_possible_move_p(self.list_point[index_point], F):
            self.list_point[index_point].move(F)    # met à jour coord du point
            
            x = self.list_point[index_point].x - self.G[0]  # coord du point par rapport à G
            y = self.list_point[index_point].y - self.G[1]
            
            self.list_point[index_point].hold_hold()  # essaie de s'accrocher à une prise si possible
            
            if self.do_graph:
                self.list_point[index_point].l = ma.sqrt(x**2 + y**2)   # met à jour l et teta du point
                self.list_point[index_point].teta = ma.atan2(y,x) 
        else:
            raise NameError('not possible')
    
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
        return (p.Interval_l[0] <= l <= p.Interval_l[1] and angle_in(teta, p.Interval_teta))
    
    def update_graph(self):
        self.scatter.set_offsets([self.G[0], self.G[1]])                    # update G
        
        for point in self.list_point :
            point.update_graph()                                            # update points
            point.line.set_data([self.G[0], point.x], [self.G[1], point.y])  # update lines
        
        fig.canvas.draw()
        fig.canvas.flush_events()


# ---------------GRAPH--------------- #

# Création de la figure :

fig, ax = plt.subplots()
def create_fig():
    ax.set_title("Simulation d'escalade")
    size = max(abs(z) for XY in Holds for z in XY)
    size += size*0.1  # ajout 10% de marge
    ax.set_xlim(-size, size)
    ax.set_ylim(-size, size)
    ax.set_aspect("equal")
    plt.ion()   # mode interactif 
    plt.show()

def show_holds():
    for hold in Holds:
        plt.scatter(hold[0],hold[1], color = "green")

def simulation(Seq):
    show_holds()
    create_fig()
    C = init_climber(do_graph=True)      # créé une seule fois
    while True:
        C.reset()
        C.update_graph()
        plt.pause(1)
        stop = False
        for T in sorted(Seq):
            try:
                C.apply_move(Seq[T])
            except NameError:
                stop = True
                break
            C.update_graph()
            plt.pause(dt)
            if stop:
                break
        plt.pause(1)

# ---------------FONCTIONS--------------- #

def distance(XY, AB):
    x = XY[0] - AB[0]
    y = XY[1] - AB[1]
    return ma.sqrt(x**2 + y**2)

def distance_in(XY, AB, Interval):
    return Interval[0] <= distance(XY,AB) <= Interval[1]

def angle_in(teta, Interval):   # si teta est dans a,b modulo 2pi
    a,b = Interval
    return (teta - a)%(2*ma.pi) <= (b - a)%(2*ma.pi)

def line_intersection(line1, line2):    # line1 = [(x1,y1),(x2,y2)], line2 = [(x3,y3),(x4,y4)]
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       raise NameError('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y

# ---------------GENETIC--------------- #
Interval_teta = {0: [-ma.pi, 0], 1: [-ma.pi, 0], 2: [-ma.pi/2, ma.pi/2], 3: [ma.pi/2, 3*ma.pi/2]}

def init_climber(do_graph = False):
    G, POS_point = pos_init()
    C = climber(G[0], G[1], do_graph)
    for i in range(4):
        l,teta = POS_point[i]
        l_max = l_j if i < 2 else l_b
        C.add_point(l, teta, [0,l_max], Interval_teta[i] )
        C.list_point[i].hold_hold()  # on accroche les points de départ aux prises
    return C

def reset_all_points(C):
    G, POS_point = pos_init()
    C.G = G
    for i in range(4):
        C.list_point[i].x = Holds[i][0]
        C.list_point[i].y = Holds[i][1]
        C.list_point[i].l = POS_point[i][0]
        C.list_point[i].teta = POS_point[i][1]
        C.list_point[i].hold_hold()  # on accroche les points de départ aux prises

def pos_init():
    G = line_intersection([(Holds[0][0], Holds[0][1]), (Holds[3][0], Holds[3][1])], [(Holds[1][0], Holds[1][1]), (Holds[2][0], Holds[2][1])])
    G_x = G[0]
    G_y = G[1]

    POS_point = []
    # jambes :
    x_jd, y_jd = Holds[0][0]-G_x, Holds[0][1]-G_y
    POS_point.append([ma.hypot(x_jd, y_jd),ma.atan2(y_jd, x_jd)])
    x_jg, y_jg = Holds[1][0]-G_x, Holds[1][1]-G_y
    POS_point.append([ma.hypot(x_jg, y_jg),ma.atan2(y_jg, x_jg)])
    
    # bras :
    x_bd, y_bd = Holds[2][0]-G_x, Holds[2][1]-G_y
    POS_point.append([ma.hypot(x_bd, y_bd),ma.atan2(y_bd, x_bd)])
    x_bg, y_bg = Holds[3][0]-G_x, Holds[3][1]-G_y
    POS_point.append([ma.hypot(x_bg, y_bg),ma.atan2(y_bg, x_bg)])

    return [G_x, G_y],POS_point

def init_pop(P = []) -> list :     # créé une population initiale de taille n
    for _ in range(n):
        C = init_climber()
        for _ in range(rd.randint(1,5)):
            C.add_F_rd()
        P.append(C)
    return insert([], P)

def mutation(selected : list) -> list :     # créé m variantes de chaque individu par mutations génétiques et des crossover
    new_Cs = []
    for C in selected:
        for _ in range(m):
            new_C = copy.deepcopy(C)
            rd.choices([new_C.add_F_rd, new_C.del_F_rd, new_C.change_move_rd],weights=[1,1,3])[0]()
            if new_C.Seq != C.Seq:
                new_Cs.append(new_C)
                
    for _ in range(int(nb_crv)):
        C1, C2 = rd.sample(selected, k = 2) # tirage sans remise de 2 individus
        new_C = crossover(C1,C2)
        if new_C.Seq != C1.Seq and new_C.Seq != C2.Seq:
            new_Cs.append(new_C)
    
    return new_Cs

def crossover(C1,C2):    # Opérateur génétique qui mélange deux individus
    C_new = init_climber()
    crp_1 = rd.randrange(0,time[-2])
    crp_2 = rd.randrange(crp_1,time[-1])
    if C1.Seq.keys():
        for T in C1.Seq.keys() :
            if crp_1 <= T >= crp_2:
                C_new.Seq[T] = copy.deepcopy(C1.Seq[T])
    if C2.Seq.keys():
        for T in C2.Seq.keys():
            if crp_1 > T < crp_2:
                C_new.Seq[T] = copy.deepcopy(C2.Seq[T])
    return C_new


def insert(P, new_ind): # insert les individues dans la pop par croissance de fistness en conservant la taille de Pop (n)
    for C in new_ind:
        C.fit = fitness(C)
        k = 0
        while k < len(P) and C.fit > P[k].fit:
            k += 1
        P.insert(k, C)
    return P[:n]    # garde une taille de la pop de n

def fitness(C):
    C.reset()
    try:
        for T in sorted(C.Seq):
            C.apply_move(C.Seq[T])
    except NameError:       # arrêt au premier mouvement impossible
        pass
    held_y = max(p.y for p in C.list_point if p.hold)
    return (Holds[-1][1] - held_y)**2 + c_G*distance(C.G, Holds[-1])  + len(list(C.Seq.keys()))*c_size
 
# pour chaque gén on créé n_selc*m nouveaux individus par mutations génétiqeus et on les insert dans la pop qui est triée par fitness(croissant)
def evolution():
    P = init_pop()
    for g in range(1,gen+1):  
        P_selc = rd.choices(P,k = n_selc, weights=[1/(x**a+1) for x in range(n)])
        P = insert(P,mutation(P_selc))
        
        if g % 100 == 0:
            print(g,'ième génération', P[0].fit,'meilleur fitness')
    print(P[0].Seq)
    # simulation graphique du meilleur individu
    simulation(P[0].Seq)


# ---------------PARAMETRE--------------- #

"Paramètres génétiques :"
n = 300         # nombre d'individus dans la population
n_selc = 20     # nombre d'individus séléctionnés dans la population pour être muté, n_selc < n
a = 0.5         # paramètre dans [0,1] qui gére la probabilité qu'un individu soit séléctionné en fonction de son rang dans la pop(0 : equiproba)
m = 40          # nombre d'individues créés par mutation génétique pour 1 individu
nb_crv = 40     # nombre de crossover fait par gen
gen = 1000      # nombre de générations
max_F = 2       # maximum d'un mouvement 
c_G = 0.01       # bonus for G height
c_size = 0      # malus for size of Seq

"Paramètres climber :"
l_b = 2   # longueur bras
l_j = 3  # longueur jambe

"Paramètre de la voie"
Holds = [(1,-1),(-1,-1),(0,0.8),(-1,0.5),(2.5,2.75),(3,5),(2,5),(0,4),(0,2),(1.5,1),(1,3),(1.5,7),(3,7),(2,10)]  # liste des coordonnées des prises, tel que Holds[:4] prises de départ (*) et Holds[-1] la prise d'arrivée
#(*): 0 pied d/ 1 pied g/ 2 main d/ 3 main g
eps = 0.3   # distance max entre un point et une prise pour que le point soit accroché à la prise

"Paramètre temporels"
T_tot = 5  # Durée totale de la simulation en secondes
dt = 0.1    # une puissance de 10 négative
time = [i for i in range(round(1 + T_tot/dt)) ]  # Liste des instants de temps (int)


# ---------------MAIN--------------- #

evolution()

# ---------------DATA--------------- #
