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
 - On considère une grimpe dite statique, sans mouvement dynamique avec des mouvement instananés pour l'instant
 - les pieds sont toujours en dessous de G
"""

# ---------------IMPORT--------------- #


import matplotlib.pyplot as plt
import math as ma
import random as rd
import copy as copy


# ---------------CLASS--------------- #

# Interval_l = [0, l_max]

class point :
    def __init__(self, x, y, z, l, Interval_l, do_graph=False):
        self.hold = False
        self.x = x
        self.y = y
        self.z = z
        self.l = l
        
        self.Interval_l = Interval_l
        
        if do_graph:
            self.scatter, = ax.plot([self.x], [self.y], [self.z], 'o', color="blue", markersize=6)
    
    def update_graph(self):
        self.scatter.set_data_3d([self.x], [self.y], [self.z])
    
    def move(self, F):
        self.hold = False
        self.x += F[0]
        self.y += F[1]
        self.z += F[2]
    
    def hold_hold(self):
        if any(distance(h, (self.x, self.y, self.z)) <= eps for h in Holds):
            self.hold = True
    
    def drop_hold(self):
        self.hold = False
    
        
class climber :
    def __init__(self, x = 0., y = 0., z = 0., do_graph = False):
        self.fit = float('inf')
        self.Seq = {}   # Seq = {t : {"p": index_p or None, "F" : F}}, None is for G move
        
        self.G = [x, y, z]
        self.list_point = []
        
        self.do_graph = do_graph
        if do_graph : # create a graph point
            self.scatter, = ax.plot([self.G[0]], [self.G[1]], [self.G[2]], 'o', color="red", markersize=6)
    
    def reset(self):
        reset_all_points(self)
    
    
    def add_F(self, T, p, F):
        if T <= time[-1]:
            self.Seq[T] = {"p" : p, "F" : F}
    
    def add_F_rd(self):
        T = rd.choice(range(len(time)))
        if rd.random() > 0.5:   # move G
            F = [rd.uniform(-max_F, max_F) for _ in range(3)]
            self.add_F(T, None, F)
        else:   # move point to hold
            p_index = rd.choice(range(len(self.list_point)))
            if self.list_point[p_index].hold and sum(q.hold for q in self.list_point) <= 2: # si pas assez de points accrochés
                pass
            possible_hold_F = self.possible_move_to_hold(T, p_index)
            if possible_hold_F:
                F = rd.choice(possible_hold_F)
                self.add_F(T, p_index, F)
    
    def possible_move_to_hold(self, T_new, p_index):
        # On simule la grimpe jusqu'au temps T_new
        self.reset()
        try:
            for T in sorted(self.Seq):
                if T >= T_new:
                    break
            
                self.apply_move(self.Seq[T])
        except NameError:       # arrêt au premier mouvement impossible
            pass
        
        p = self.list_point[p_index]
        F_possible = [[h[0] - p.x, h[1] - p.y, h[2] - p.z] for h in Holds if distance(h, (p.x, p.y, p.z)) > 1e-9 and self.is_valid(p, self.G, h)]  # 1e-9 pour pas que ce soit la même prise
        return [F for F in F_possible if self.if_possible_move_p(p,F)]
    
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
        
    
    def add_point(self, x, y, z, l, Interval_l):
        self.list_point.append(point(x,  y, z, l , Interval_l, self.do_graph))  
        
        if self.do_graph:
            # create a graph line between G and the new point
            i = len(self.list_point) - 1    # index of the point added
            self.list_point[i].line, = ax.plot([self.G[0], self.list_point[i].x], [self.G[1], self.list_point[i].y], [self.G[2], self.list_point[i].z], color="blue", linewidth=2)
    
    def apply_move(self, move):
        if move['p'] is None:
            self.move_G(move['F'])
        else:
            self.move_p(move['p'],move['F'])
    
    def move_G(self, F):
        if self.is_possible_move_G(F):
            self.G[0] += F[0]
            self.G[1] += F[1]
            self.G[2] += F[2]
            for p in self.list_point:
                x = p.x - self.G[0]
                y = p.y - self.G[1]
                z = p.z - self.G[2]
                p.l = ma.hypot(x, y, z)   # met à jour l du point
        else:
            raise NameError('not possible')
    

    def move_p(self, index_point, F):
        if self.if_possible_move_p(self.list_point[index_point], F):
            self.list_point[index_point].move(F)    # met à jour coord du point
            
            x = self.list_point[index_point].x - self.G[0]  # coord du point par rapport à G
            y = self.list_point[index_point].y - self.G[1]
            z = self.list_point[index_point].z - self.G[2]

            self.list_point[index_point].hold_hold()  # essaie de s'accrocher à une prise si possible
            
            self.list_point[index_point].l = ma.sqrt(x**2 + y**2 + z**2)   # met à jour l et teta du point
        else:
            raise NameError('not possible')
    
    def is_possible_move_G(self, F):
        G_new = [self.G[0] + F[0], self.G[1] + F[1], self.G[2] + F[2]]
        return all(self.is_valid(p, G_new, (p.x, p.y, p.z)) for p in self.list_point)
    
    def if_possible_move_p(self, p, F):
        nb_hold = sum(1 for q in self.list_point if q.hold)
        if p.hold and nb_hold <= 2:
            return False
        return self.is_valid(p, self.G, (p.x + F[0], p.y + F[1], p.z + F[2]))
    
    def is_valid(self, p, G, pos):
        x, y, z = pos[0] - G[0], pos[1] - G[1], pos[2] - G[2] # position relative à G
        l = ma.hypot(x, y, z)
        if not (p.Interval_l[0] <= l <= p.Interval_l[1]):
            return False
        if p in self.list_point[:2] and z > 0:  # les jambes ne doivent pas être au dessu de centre G
            return False
        return True
    
    def update_graph(self):
        self.scatter.set_data_3d([self.G[0]], [self.G[1]], [self.G[2]])
        
        for p in self.list_point :
            p.update_graph()                                            # update points
            p.line.set_data_3d([self.G[0], p.x], [self.G[1], p.y], [self.G[2], p.z])  # update lines
        
        fig.canvas.draw()
        fig.canvas.flush_events()


# ---------------GRAPH--------------- #

# Création de la figure :
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
def create_fig():
    ax.set_title("Simulation d'escalade")
    size = max(abs(z) for XYZ in Holds for z in XYZ)
    marge = 1
    for lim,i in zip([ax.set_xlim,ax.set_ylim,ax.set_zlim],[0,1,2]):
        lim(min(h[i] for h in Holds)-marge,max(h[i] for h in Holds)+marge)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_aspect('equal')
    ax.grid(False)
    ax.xaxis.pane.set_visible(False)
    ax.yaxis.pane.set_visible(False)
    ax.zaxis.pane.set_visible(False)
    plt.ion()   # mode interactif 
    plt.show()

def show_holds():
    for h in Holds:
        ax.plot([h[0]], [h[1]], [h[2]], 'o', color="green")

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

def distance(XYZ, ABC):
    x = XYZ[0] - ABC[0]
    y = XYZ[1] - ABC[1]
    z = XYZ[2] - ABC[2]
    return ma.sqrt(x**2 + y**2 + z**2)

def distance_in(XYZ, ABC, Interval):
    return Interval[0] <= distance(XYZ, ABC) <= Interval[1]

def angle_in(teta, Interval):   # si teta est dans a,b modulo 2pi
    a,b = Interval
    return (teta - a)%(2*ma.pi) <= (b - a)%(2*ma.pi)

def line_intersection(line1, line2):  # line1 = [(x1,y1,z1),(x2,y2,z2)], line2 = [(x3,y3,z3),(x4,y4,z4)]
    P1, P2 = line1[0], line2[0]
    d1 = [line1[1][i] - line1[0][i] for i in range(3)]
    d2 = [line2[1][i] - line2[0][i] for i in range(3)]
    w = [P2[i] - P1[i] for i in range(3)]

    def cross(a, b):
        return [a[1]*b[2] - a[2]*b[1],
                a[2]*b[0] - a[0]*b[2],
                a[0]*b[1] - a[1]*b[0]]

    def dot(a, b):
        return sum(a[i] * b[i] for i in range(3))

    n = cross(d1, d2)
    n2 = dot(n, n)

    # paramètres des points les plus proches sur chaque droite
    t = dot(cross(w, d2), n) / n2
    s = dot(cross(w, d1), n) / n2
    A = [P1[i] + t * d1[i] for i in range(3)]
    B = [P2[i] + s * d2[i] for i in range(3)]

    # si les droites sont gauches, on prend le milieu du segment le plus court
    return tuple((A[i] + B[i]) / 2 for i in range(3))

# ---------------GENETIC--------------- #

def init_climber(do_graph = False):
    G, l_points = pos_init()
    C = climber(G[0], G[1], G[2], do_graph)
    for i in range(4):
        l = l_points[i]
        l_max = l_j if i < 2 else l_b
        C.add_point(Holds[i][0], Holds[i][1], Holds[i][2], l, [0,l_max])
        C.list_point[i].hold_hold()  # on accroche les points de départ aux prises
    return C

def reset_all_points(C):
    G, l_points = pos_init()
    C.G = G
    for i in range(4):
        C.list_point[i].x = Holds[i][0]
        C.list_point[i].y = Holds[i][1]
        C.list_point[i].z = Holds[i][2]
        C.list_point[i].l = l_points[i]
        C.list_point[i].hold_hold()  # on accroche les points de départ aux prises

def pos_init():
    G = line_intersection([(Holds[0][0], Holds[0][1], Holds[0][2]), (Holds[3][0], Holds[3][1], Holds[3][2])], [(Holds[1][0], Holds[1][1], Holds[1][2]), (Holds[2][0], Holds[2][1], Holds[2][2])])
    G_x = G[0]
    G_y = G[1]
    G_z = G[2]

    l_points = []
    # jambes :
    x_jd, y_jd, z_jd = Holds[0][0]-G_x, Holds[0][1]-G_y, Holds[0][2]-G_z
    l_points.append(ma.hypot(x_jd, y_jd, z_jd))
    x_jg, y_jg, z_jg = Holds[1][0]-G_x, Holds[1][1]-G_y, Holds[1][2]-G_z
    l_points.append(ma.hypot(x_jg, y_jg, z_jg))

    # bras :
    x_bd, y_bd, z_bd = Holds[2][0]-G_x, Holds[2][1]-G_y, Holds[2][2]-G_z
    l_points.append(ma.hypot(x_bd, y_bd, z_bd))
    x_bg, y_bg, z_bg = Holds[3][0]-G_x, Holds[3][1]-G_y, Holds[3][2]-G_z
    l_points.append(ma.hypot(x_bg, y_bg, z_bg))

    return [G_x, G_y, G_z], l_points

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
            if crp_1 >= T or T >= crp_2:
                C_new.Seq[T] = copy.deepcopy(C1.Seq[T])
    if C2.Seq.keys():
        for T in C2.Seq.keys():
            if crp_1 < T < crp_2:
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
    held_z = max(p.z for p in C.list_point if p.hold)
    height_still = (Holds[-1][2] - held_z)**2
    if height_still <= 1e-5:
        c_G = 0
        c_size = fitness_w['c_size']
    else:
        c_G = fitness_w['c_G']
        c_size = 0
    return height_still + c_G*distance(C.G, Holds[-1])  + len(list(C.Seq.keys()))*c_size
 
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
 

c_G = 0.01      # bonus for G height
c_size = 0.01   # malus for size of Seq
fitness_w = {'c_G' : c_G,'c_size'  :c_size}

"Paramètres climber :"
l_b = 2         # longueur bras
l_j = 3         # longueur jambe
max_F = 2       # longueur maximum d'un mouvement

"Paramètre de la voie"
Holds = [(1, 0, -1), (-1, 0, -1), (0, 0, 0.8), (-1, 0, 0.5), (2.5, 0, 2.75), (3, 0, 5), (2, -1, 5), (0, -1, 4), (0, -1, 2), (1.5, -1, 1), (1, -1, 3), (1.5, -1.5, 7), (3, -1.5, 7), (2, -1.5, 10)]#(*): 0 pied d/ 1 pied g/ 2 main d/ 3 main g
eps = 0.3   # distance max entre un point et une prise pour que le point soit accroché à la prise

"Paramètre temporels"
T_tot = 5  # Durée totale de la simulation en secondes
dt = 0.1    # une puissance de 10 négative
time = [i for i in range(round(1 + T_tot/dt)) ]  # Liste des instants de temps (int)


# ---------------MAIN--------------- #

evolution()

# ---------------DATA--------------- #


