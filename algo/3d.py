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
 - les pieds sont toujours en dessous de J
"""

# ---------------IMPORT--------------- #


import matplotlib.pyplot as plt
import math as ma
import random as rd
import copy as copy


# ---------------CLASS--------------- #

# Interval_l = [0, l_max]
class InvalidMove(Exception):
    pass
    
class point :
    def __init__(self, x, y, z, Interval_l : float, do_graph=False):
        self.hold = None
        self.x = x
        self.y = y
        self.z = z
        
        self.Interval_l = Interval_l
        
        if do_graph:
            self.scatter, = ax.plot([self.x], [self.y], [self.z], 'o', color="black", markersize=6)
    
    def update_graph_p(self):
        self.scatter.set_data_3d([self.x], [self.y], [self.z])

    def move(self, F):
        self.x += F[0]
        self.y += F[1]
        self.z += F[2]
          
class climber :
    def __init__(self, XYZ_J, XYZ_B, do_graph = False):
        self.J = XYZ_J
        self.B = XYZ_B

        self.list_point = []
        
        self.do_graph = do_graph
        
        if do_graph :
            self.scatter_J = ax.plot([self.J[0]], [self.J[1]], [self.J[2]], 'o', color="red", markersize=6)[0]
            self.scatter_B = ax.plot([self.B[0]], [self.B[1]], [self.B[2]], 'o', color="red", markersize=6)[0]
            self.line_JB = ax.plot([self.J[0], self.B[0]], [self.J[1], self.B[1]], [self.J[2], self.B[2]], color="blue", linewidth=2)[0]

        
    def add_point(self, x, y, z, Interval_l):
        self.list_point.append(point(x,  y, z, Interval_l, self.do_graph))  
        
        if self.do_graph:
            # create a graph line between G and the new point
            i = len(self.list_point) - 1    # index of the point added
            if i < 2:   # les jambes
                self.list_point[i].line = ax.plot([self.J[0], self.list_point[i].x], [self.J[1], self.list_point[i].y], [self.J[2], self.list_point[i].z], color="blue", linewidth=2)[0]
            else:   # les bras
                self.list_point[i].line = ax.plot([self.B[0], self.list_point[i].x], [self.B[1], self.list_point[i].y], [self.B[2], self.list_point[i].z], color="blue", linewidth=2)[0]
    
    def apply_move(self, move):
        if move["type"] == 'rot':
            self.move_rot(move["i_bdy"], move['r'])
        elif move["type"] == 'body':
            self.move_body(move["F"])
        elif move["type"] == 'p':
            self.move_p(move["i_p"], move["F"])
        elif move["type"] == 'to_hold':
            self.move_to_hold(move["i_p"], move["i_h"], move["hold"])
        
        if not self.is_valid():
            raise InvalidMove
    
    def move_body(self, F):
        for i in range(3):
            self.J[i] += F[i]
            self.B[i] += F[i]

    def move_p(self, i_p, F):
        self.list_point[i_p].move(F)    # met à jour coord du point
        self.list_point[i_p].hold = None
         
    
    def move_to_hold(self, i_p, i_h, hold):
        self.list_point[i_p].x = hold[0]  
        self.list_point[i_p].y = hold[1]
        self.list_point[i_p].z = hold[2]
        self.list_point[i_p].hold = i_h
    
    def move_rot(self, i_bdy, r):
        theta, phi = r
        moving, fixed = (self.J, self.B) if i_bdy == 0 else (self.B, self.J)
        
        v = [moving[k] - fixed[k] for k in range(3)]
        v_rot = rotate_vector(v, theta, phi)
        
        new = [fixed[k] + v_rot[k] for k in range(3)]
        
        if i_bdy == 0:
            self.J = new
        else:
            self.B = new
    
    
    def is_valid(self): # renvoie si la position actuel de C est valide sans la modifier
        l_bdy = ma.hypot(*(self.B[j] - self.J[j] for j in range(3)))
        
        hold_held = [p.hold for p in self.list_point if p.hold is not None]
        if len(set(hold_held)) != len(hold_held):          # deux points sur la même prise
            return False
        
        for p in self.list_point:
            G = self.J if self.list_point.index(p) < 2 else self.B
            x,y,z = (p.x - G[0], p.y - G[1], p.z - G[2])
            
            l_p = ma.hypot(x, y, z)
            x_b,y_b,z_b = (self.B[i] - self.J[i] for i in range(3))
            denom = ((ma.sqrt(x**2 + y**2 + z**2))*(ma.sqrt(x_b**2 + y_b**2 + z_b**2)))
            nom = (x*x_b + y*y_b + z*z_b)
            if denom>1e-5 and -1<= nom/denom <= 1:
                theta = ma.acos(nom/denom)
            else:
                theta = 0
            
            if not (p.Interval_l[0] <= l_p <= p.Interval_l[1]): # si la taille du bras est raisonnable
                return False
            if abs(l_bdy - l_dos)>1e-4:    # la longeur du dos doit rester la même
                return False
            if self.B[2] < self.J[2]:   # le bassin doit être plus bas que les épaules
                return False
            if self.list_point.index(p) < 2 and not angle_in(theta, [ma.pi/2,3*ma.pi/2]):    # angle cohérent des jambes(inférieure au plan normal du dos en J)
                return False
        if not self.has_min_hold():    # si moins de deux membres accrochés dont au moins une main
            return False
        return True
    
    def has_min_hold(self):
        if sum(1 if p.hold is not None else 0 for p in self.list_point[2:]) == 0:    # si aucune main est accrochée
            return False
        if sum(1 if p.hold is not None else 0 for p in self.list_point) < 2:    # si moins de 2 accroches au total
            return False
        return True
                
            
    
    def update_graph(self):
        self.scatter_J.set_data_3d([self.J[0]], [self.J[1]], [self.J[2]])
        self.scatter_B.set_data_3d([self.B[0]], [self.B[1]], [self.B[2]])

        self.line_JB.set_data_3d([self.J[0], self.B[0]],[self.J[1], self.B[1]], [self.J[2], self.B[2]])
        
        for i in range(len(self.list_point)):
            self.list_point[i].update_graph_p()
            
            parent = self.J if i<2 else self.B
            p = self.list_point[i]
            p.line.set_data_3d([parent[0], p.x],[parent[1], p.y], [parent[2], p.z])
        
        fig.canvas.draw()
        fig.canvas.flush_events()

class Seq:
    def __init__(self, ini_seq = None):
        self.Seq = copy.deepcopy(ini_seq) if ini_seq is not None else {}  # Seq = {t : {"p": index_p or None, "F" : F}}, None is for G move
        self.fit = 1e+5
        
    def add_F(self, T : int,  rd_mut : dict):
            if T <= time[-1]:
                self.Seq[T] = rd_mut    # F = {"type": ..., "move" : {"i_bdy": ,"r" : [theta,phi]} or {"i_p": , "F": } or {"i_p": , "i_h", "hold":} or {"F": }}
        
    def rd_mut(self):
        T_new = rd.choice(range(len(time)))
        
        if rd.random()>0.3:
            C = init_climber()
            try :
                self.simu_to_T(C,T_new-1)   # on se place à l'intant d'avant l'ajout du mouvement
            except InvalidMove:
                return
            
            rd_mut = rd.choice([self.rot, self.to_hold, self.p, self.body])
            Dict = rd_mut(C)
            if Dict is not None:
                self.add_F(T_new, Dict)
        else:
            self.del_rd()
        
    def simu_to_T(self, C , T_new):
        # On simule la grimpe jusqu'au temps T_new
        for T in sorted(self.Seq.keys()):
            if T > T_new:
                break
        
            C.apply_move(self.Seq[T])
    
    def rot(self, C):
        i_bdy = rd.choice([0,1])
        r = [rd.uniform(-max_theta,max_theta), rd.uniform(-max_phi,max_phi)]
        C.move_rot(i_bdy,r)
        if C.is_valid():
            return {"type":"rot", "i_bdy" : i_bdy, "r" : r}
        
    def to_hold(self, C):
        i_p = rd.randrange(len(C.list_point))
        h_possible = []
        for i_h in range(len(Holds)):
            h = Holds[i_h]
            C.move_to_hold(i_p, i_h, h)
            if C.is_valid():
                h_possible.append((h, i_h))
        
        if h_possible:
            h, i_h =  rd.choice(h_possible)
            return {"type": "to_hold", "i_p" : i_p, "i_h" : i_h, "hold" :h}
    
    def p(self, C):
        i_p = rd.randrange(len(C.list_point))
        F = [rd.uniform(-max_F, max_F) for _ in range(3)]
        C.move_p(i_p,F)
        if C.is_valid():
            return {"type" : "p", "i_p" : i_p, "F" : F}
    
    def body(self,C):
        F = [rd.uniform(-max_F, max_F) for _ in range(3)]
        C.move_body(F)
        if C.is_valid():
            return {"type" : "body", "F" : F}
    
    def del_rd(self):    # supprime un mouvement au hasard
        if self.Seq:
            self.Seq.pop(rd.choice(list(self.Seq.keys())))

    
# ---------------GRAPH--------------- #

# Création de la figure :
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
def set_fig():
    ax.set_title("Simulation d'escalade")
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
    x,y,z = [[h[i] for h in Holds] for i in range(3)]
    ax.scatter(x,y,z, c='green', marker='X', s=50, depthshade=False)

def simulation(Seq, n_loop = -1):
    while plt.fignum_exists(fig.number) and (n_loop < 0 or n_loop > 0):
        ax.clear()
        set_fig()
        show_holds()
        
        C = init_climber(do_graph=True)
        C.update_graph()
        plt.pause(0.5)
        
        for T in sorted(Seq):
            try:
                C.apply_move(Seq[T])
            except InvalidMove:
                break
            C.update_graph()
            plt.pause(dt)
        plt.pause(1)
        n_loop -= 1
    plt.ioff()

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

def rotate_vector(v, theta, phi):
    """Rotation d'angle theta autour de Z, puis d'angle phi autour de Y (conserve la norme)."""
    x, y, z = v
    ct, st = ma.cos(theta), ma.sin(theta)
    cp, sp = ma.cos(phi), ma.sin(phi)

    # Rz(theta)
    x1 = ct * x - st * y
    y1 = st * x + ct * y
    z1 = z

    # Ry(phi)
    return [cp * x1 + sp * z1, y1,-sp * x1 + cp * z1]

def middle_seg(A,B):
    AB = [B[i]-A[i] for i in range(len(A))]
    return [A[i] + AB[i]/2 for i in range(len(A))]

def atan_c(x):
    return ma.atan(x)*2/ma.pi

# ---------------GENETIC--------------- #

def init_climber(do_graph = False):
    XYZ_J, XYZ_B = body_pos_init()
    C = climber(XYZ_J, XYZ_B, do_graph)
    for i in range(4):
        l_max = l_j if i < 2 else l_b
        C.add_point(Holds[i][0], Holds[i][1], Holds[i][2], [0,l_max])
        C.list_point[i].hold = i  # on accroche les points de départ aux prises
    if C.is_valid():
        return C
    else:
        raise InvalidMove

def body_pos_init():
    J_M = middle_seg(Holds[0], Holds[1])
    B_M = middle_seg(Holds[2], Holds[3])

    M = middle_seg(J_M, B_M)

    direction = [B_M[i] - J_M[i] for i in range(3)]
    norm = ma.sqrt(sum(d**2 for d in direction))

    if norm > 0:
        direction = [d / norm for d in direction]
    else:
        direction = [0, 0, 1]  # Direction verticale par défaut

    J = [M[i] - direction[i]*l_dos/2 for i in range(3)]
    B = [M[i] + direction[i]*l_dos/2 for i in range(3)]

    return J, B

def init_pop() :     # créé une population initiale de taille n
    new_P = []
    for _ in range(n):
        ind = Seq()
        for _ in range(rd.randint(1,5)):
            ind.rd_mut()
        if ind.Seq:
            new_P.append(ind)
    P = []
    insert(P,new_P)
    return P
    

def mutation(selected : list) -> list :     # créé m variantes de chaque individu par mutations génétiques et des crossover
    Inds = []
    for ind in selected:
        for _ in range(m):
            essai = 0
            new_ind = copy.deepcopy(ind)
            while essai<20 and (new_ind is None or new_ind.Seq == ind.Seq):
                essai += 1
                new_ind = copy.deepcopy(ind)
                for _ in range(rd.randint(1,5)):
                    new_ind.rd_mut()
            
            if essai!=20:
                Inds.append(new_ind)
            
    for _ in range(nb_crv):
        len_ini = len(Inds)
        essai = 0
        while len_ini == len(Inds) and essai < 10:
            ind_1, ind_2 = rd.sample(selected, k = 2) # tirage sans remise de 2 individus
            if ind_1.Seq != ind_2.Seq:
                new_ind = crossover(ind_1, ind_2)
                if new_ind is not None:
                    Inds.append(new_ind)
            essai += 1

    return Inds

def crossover(ind_1, ind_2):    # Opérateur génétique qui mélange deux individus
    new_ind = Seq()
    crp_1 = rd.randrange(0,time[-2])
    crp_2 = rd.randrange(crp_1,time[-1])
    if ind_1.Seq.keys():
        for T in ind_1.Seq.keys() :
            if crp_1 >= T or T >= crp_2:
                new_ind.Seq[T] = copy.deepcopy(ind_1.Seq[T])
    if ind_2.Seq.keys():
        for T in ind_2.Seq.keys():
            if crp_1 < T < crp_2:
                new_ind.Seq[T] = copy.deepcopy(ind_2.Seq[T])
    if new_ind.Seq == ind_1.Seq or new_ind.Seq == ind_2.Seq:
        return None
    return new_ind


def insert(P, new_inds:list): # insert les individues dans la pop par croissance de fistness 
    for C in new_inds:
        C.fit = fitness(C)
        if all(C.Seq != D.Seq for D in P):  # tout les individus sont différents
            k = 0
            while k < len(P) and C.fit > P[k].fit:
                k += 1
            P.insert(k, C)

def fitness(ind):   # calcul du fitness d'un individu
    C = init_climber()
    try :
        ind.simu_to_T(C, time[-1])
    except InvalidMove:
        return 1e5  # +inf
    
    max_height_z = max(p.z for p in C.list_point if p.hold is not None)
    h = (Holds[-1][2] - max_height_z)**2
    
    #tot_F = sum(f**2 for move in ind.Seq.values() for f in move['F']) if ind.Seq.values() else 0# somme quadratique des déplacements
    #nb_mvt = len(list(ind.Seq.keys()))
    G_stil = distance(C.B, Holds[-1])
    
    if h<=1e-2:
        G_stil = 0
    return h + c_G*G_stil

# pour chaque gén on créé n_selc*m nouveaux individus par mutations génétiqeus et on les insert dans la pop qui est triée par fitness(croissant)
def evolution(ini_seq = None):
    if ini_seq is None:
        P = init_pop()
    else:
        P = [Seq(copy.deepcopy(ini_seq)) for _ in range(n)]

    for g in range(1,gen+1):  
        P_selc = rd.choices(P, k = n_selc, weights = [len(P)-x for x in range(len(P))])
        insert(P, mutation(P_selc))
        P = P[: n + m + nb_crv]
        if g % 100 == 0:
            print(g,'ième génération : \n- meilleur fitness : ',P[0].fit)

    print(P[0].Seq)
    # simulation graphique du meilleur individu
    simulation(P[0].Seq)


# ---------------PARAMETRE--------------- #

"Paramètres génétiques :"
n = 100                 # nombre d'individus dans la population
n_selc = int(n/2)       # nombre d'individus séléctionnés dans la population pour être muté, n_selc <= n
m = 5                   # nombre d'individues créés par mutation génétique pour 1 individu
nb_crv = int(n/2)       # nombre de crossover fait par gen
gen = 500              # nombre de générations

c_G = 0.01          # bonus for G height
c_size = 0      # malus for size of Seq
c_totF = 0      # malus pour la quantité de mouvements effectués

"Paramètres climber :"
l_b = 2             # longueur bras
l_j = 3             # longueur jambe
l_dos = 1           # longeur du dos
max_F = 2.5         # norme maximal d'un mouvement
max_theta = ma.pi/2 # angle maximal d'un movement de rotation polaire
max_phi = ma.pi     # angle maximal d'un movement de rotation azimut

"Paramètre de la voie"
Holds = [(1, 0, -1), (-1, 0, -1), (0, 0, 0.8), (-1, 0, 0.5), (2.5, 0, 2.75), (3, 0, 5), (2, -1, 5), (0, -1, 4), (0, -1, 2), (1.5, -1, 1), (1, -1, 3), (1.5, -1.5, 7), (3, -1.5, 7), (2, -1.5, 9)]#(*): 0 pied d/ 1 pied g/ 2 main d/ 3 main g
eps = 0.3   # distance max entre un point et une prise pour que le point soit accroché à la prise

"Paramètre temporels"
T_tot = 5  # Durée totale de la simulation en secondes
dt = 0.1    # une puissance de 10 négative
time = [i for i in range(round(1 + T_tot/dt)) ]  # Liste des instants de temps (int)


# ---------------MAIN--------------- #

evolution()

# ---------------DATA--------------- #

Seq_2 = {49: {'type': 'body', 'F': [0.3579372596004786, 0.26407179449313967, 1.6743077552180203]}, 50: {'type': 'to_hold', 'i_p': 2, 'i_h': 11, 'hold': (1.5, -1.5, 7)}, 1: {'type': 'to_hold', 'i_p': 3, 'i_h': 3, 'hold': (-1, 0, 0.5)}, 48: {'type': 'rot', 'i_bdy': 1, 'r': [-0.5215602891193407, 0.8533282800841331]}, 47: {'type': 'rot', 'i_bdy': 0, 'r': [0.9106912014235253, -0.4560164132363851]}, 5: {'type': 'rot', 'i_bdy': 1, 'r': [0.06355448106235762, -0.025968516717251422]}, 6: {'type': 'body', 'F': [-0.5441796380316748, -0.494370580287832, 2.094737915151863]}, 7: {'type': 'to_hold', 'i_p': 3, 'i_h': 7, 'hold': (0, -1, 4)}, 8: {'type': 'rot', 'i_bdy': 1, 'r': [-0.038879470674358485, -0.039414135258367455]}, 11: {'type': 'to_hold', 'i_p': 0, 'i_h': 3, 'hold': (-1, 0, 0.5)}, 12: {'type': 'rot', 'i_bdy': 1, 'r': [-0.1005785166490123, -0.02148551317731906]}, 13: {'type': 'rot', 'i_bdy': 0, 'r': [-0.6236833738275243, -0.07586493000986883]}, 14: {'type': 'to_hold', 'i_p': 2, 'i_h': 8, 'hold': (0, -1, 2)}, 16: {'type': 'to_hold', 'i_p': 0, 'i_h': 2, 'hold': (0, 0, 0.8)}, 15: {'type': 'to_hold', 'i_p': 0, 'i_h': 2, 'hold': (0, 0, 0.8)}, 22: {'type': 'rot', 'i_bdy': 0, 'r': [-0.0644655043776956, 0.004895468182327445]}, 21: {'type': 'rot', 'i_bdy': 1, 'r': [0.07239210852892763, 0.37532356079758955]}, 18: {'type': 'rot', 'i_bdy': 0, 'r': [0.6923698984306226, -0.42309603736434687]}, 20: {'type': 'rot', 'i_bdy': 0, 'r': [-0.11042694963184085, -0.11615160335237329]}, 23: {'type': 'to_hold', 'i_p': 1, 'i_h': 1, 'hold': (-1, 0, -1)}, 25: {'type': 'to_hold', 'i_p': 1, 'i_h': 4, 'hold': (2.5, 0, 2.75)}, 46: {'type': 'rot', 'i_bdy': 0, 'r': [1.5334112000552573, -0.7299853715461251]}, 45: {'type': 'rot', 'i_bdy': 1, 'r': [0.2679355555366356, 0.10536514334920977]}, 44: {'type': 'p', 'i_p': 3, 'F': [2.003933603477896, 0.7136815711317999, 1.4258701491178871]}, 43: {'type': 'to_hold', 'i_p': 2, 'i_h': 6, 'hold': (2, -1, 5)}, 41: {'type': 'rot', 'i_bdy': 1, 'r': [0.018308412172736377, 0.9326813543749575]}, 40: {'type': 'body', 'F': [0.21428717951857257, -0.5218193950532835, 0.1909789879366901]}, 39: {'type': 'body', 'F': [0.5758860944539057, -0.42704040008229516, 0.11462037031852113]}, 38: {'type': 'rot', 'i_bdy': 0, 'r': [-0.03862982421301453, -0.042025673159843624]}, 31: {'type': 'body', 'F': [-0.2638804555019254, 0.23323357139645395, 0.08311008646959461]}, 30: {'type': 'body', 'F': [0.6107417288988075, 0.3086003114336715, 0.2861110617132039]}, 29: {'type': 'body', 'F': [-0.013930330878935582, -0.05928432970720987, -0.009336670046804496]}, 28: {'type': 'body', 'F': [0.21738994651010968, -0.14750932121196314, 0.33128142734557153]}, 32: {'type': 'rot', 'i_bdy': 0, 'r': [1.1848588570413998, -0.5675517945116675]}, 35: {'type': 'rot', 'i_bdy': 0, 'r': [-1.205659133589791, -0.5311558894247472]}, 34: {'type': 'p', 'i_p': 0, 'F': [1.5704113609817414, 1.3375856914457875, 1.4101078452325204]}, 33: {'type': 'to_hold', 'i_p': 2, 'i_h': 10, 'hold': (1, -1, 3)}, 36: {'type': 'rot', 'i_bdy': 1, 'r': [0.394745652192797, 0.28389526438732604]}, 27: {'type': 'p', 'i_p': 0, 'F': [0.7449721431024137, -1.164076097468754, 0.6106294907992584]}, 26: {'type': 'to_hold', 'i_p': 2, 'i_h': 10, 'hold': (1, -1, 3)}}
