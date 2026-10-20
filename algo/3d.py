"""
En escalade, on peut distinguer deux types de mouvements : 
 - les mouvements de replacement où on se repositionne pour améliorer le placement de son centre de gravité ou se préparer pour le prochain mouvement
 - les movement de déplacment où le(s) membre(s) quitte(nt) une prise pour en attraper une autre afin d'avancer ou pour optimiser son énérgie en lachant une prise inutile à tenir

On modélisera le corps du grimpeur par un ensemble de 5 points et 4 liaisons :
 - le point centrale, noté G car aussi le centre de gravité (par approximation), qui représente le bassin/le dos
 - 4 points pour les mains et les pieds
 - 4 liaisons pour lier ces mains et pieds avec le point centrale, chaque liaisons est équivalent cinématiquement à une liaison pivot et une liaison glissiére placés en séries sur le même axe centrale qui est le vecteur entre la/le main/pied et le point G

Ainsi, un grimpeur pourra faire les mouvement (que l'on effectura seulement si ils sont possible garder l'intégrité du corps et la réalité physique) :
 - Translater ou rotationner le corps (J et B)
 - Translater ou aller à une prise donnée pour un pied ou une main 

Autres caractéristiques du modèle :
 - On considère une grimpe dite statique, sans mouvement dynamique avec des mouvement instananés pour l'instant
 - Lorsque on déplace le corps(J ou B) tout les points qui ne sont pas sur une prise suiveront ce mouvement 

Un position valide signifie :
 - respecter l'intégrité du corps : 
    - longeur des membres, pieds en dessous de J
    - les pieds en dessous du bassin(J) par rapport à la position(dépend de l'angle du corps)
 - respecter la physique(du moins sa simulation) : 
    - la force verticale(maximale théoriqement possible) résultante doit vaincre la gravité 
    - l'angle de tenu de prise doit respecter le cône d'adhérence
 - être dans une position raisonnable :
    - un membre par prise au maximum (pas de ramener)
    - 2 points d'accroche au mur au minimum
"""

# ---------------IMPORT--------------- #


import matplotlib.pyplot as plt
import math as ma
import random as rd
import copy as copy
import multiprocessing as mltp
import numpy as np


# ---------------CLASS--------------- #

# Interval_l = [0, l_max]
class InvalidMove(Exception):
    pass
    
class point :
    def __init__(self, x, y, z, Interval_l : float, do_graph=False):
        self.hold = None
        self.XYZ = np.array([x,y,z])
        self.Interval_l = Interval_l
        
        if do_graph:
            self.scatter, = ax.plot([self.XYZ[0]], [self.XYZ[1]], [self.XYZ[2]], 'o', color="black", markersize=6)
    
    def update_graph_p(self):
        self.scatter.set_data_3d([self.XYZ[0]], [self.XYZ[1]], [self.XYZ[2]])

    def move(self, F):
        self.XYZ = np.add(self.XYZ, F)
    
    def go_to(self, M):
        self.XYZ = M
          
class climber :
    def __init__(self, XYZ_J, XYZ_B, do_graph = False):
        self.J = np.array(XYZ_J)
        self.B = np.array(XYZ_B)

        self.list_point = []
        
        self.do_graph = do_graph
        
        if do_graph :
            self.scatter_J = ax.plot([self.J[0]], [self.J[1]], [self.J[2]], 'o', color="red", markersize=6)[0]
            self.scatter_B = ax.plot([self.B[0]], [self.B[1]], [self.B[2]], 'o', color="red", markersize=6)[0]
            self.line_JB = ax.plot([self.J[0], self.B[0]], [self.J[1], self.B[1]], [self.J[2], self.B[2]], color="blue", linewidth=2)[0]

        
    def add_point(self, x, y, z, Interval_l):
        p = point(x, y, z, Interval_l, self.do_graph)
        self.list_point.append(p)

        if self.do_graph:
            G = self.J if len(self.list_point) <= 2 else self.B   # jambes → J, bras → B
            p.line = ax.plot(*zip(G, p.XYZ), color="blue", linewidth=2)[0]
            
    def apply_move(self, move):
        type = move["type"]
        if type == 'rot':
            self.move_rot(move["i_bdy"], move['r'])
        elif type == 'body':
            self.move_body(move["F"])
        elif type == 'p':
            self.move_p(move["i_p"], move["F"])
        elif type == 'to_hold':
            self.move_to_hold(move["i_p"], move["i_h"], move["hold"])
        
        if not self.is_valid():
            raise InvalidMove
    
    def move_body(self, F):
        self.J = np.add(self.J, F)
        self.B = np.add(self.B, F)
        for p in self.list_point:    # on déplace les points non acrochés aussi
            if p.hold is None:
                p.move(F)

    def move_p(self, i_p, F):
        self.list_point[i_p].move(F)    # met à jour coord du point
        self.list_point[i_p].hold = None
         
    
    def move_to_hold(self, i_p, i_h, hold):
        self.list_point[i_p].go_to(hold)
        self.list_point[i_p].hold = i_h
    
    def move_rot(self, i_bdy, r):
        theta, phi = r
        moving, fixed = (self.J, self.B) if i_bdy == 0 else (self.B, self.J)

        v = moving - fixed
        new = fixed + np.array(rotate_vector(v, theta, phi))
        shift = new - moving                       # déplacement du point qui tourne

        if i_bdy == 0:
            self.J = new
            libres = self.list_point[:2]           # jambes, attachées à J
        else:
            self.B = new
            libres = self.list_point[2:]           # bras, attachés à B

        for p in libres:
            if p.hold is None:
                p.move(shift)
        
    def is_valid(self): # renvoie si la position actuel de C est valide sans la modifier
        global Fz, min_h, cone, angle, J_up_B, not_l_dos, not_l_p, same_h
        
        JB = np.subtract(self.B, self.J)
        l_bdy = d(self.B, self.J)
        
        hold_held = [p.hold for p in self.list_point if p.hold is not None]
        if len(set(hold_held)) != len(hold_held):          # deux points sur la même prise
            same_h += 1
            return False
        
        for i_p,p in enumerate(self.list_point):
            G = self.J if i_p < 2 else self.B
            Gp = np.subtract(p.XYZ,G)
            l_p = d(G, p.XYZ)
            
            if not (p.Interval_l[0] <= l_p <= p.Interval_l[1]): # si la taille du bras est raisonnable
                not_l_p += 1
                return False
            if abs(l_bdy - l_dos)>1e-4:    # la longeur du dos doit rester la même
                not_l_dos += 1
                return False
            if self.B[2] < self.J[2]:   # le bassin doit être plus bas que les épaules
                J_up_B += 1
                return False
            if i_p < 2:    # angle cohérent des jambes(inférieure au plan normal du dos en J)
                denom = l_p*l_bdy
                nom = np.dot(Gp, JB)
                if denom>1e-5 and -1<= nom/denom <= 1:
                    theta = ma.acos(nom/denom)  # l'angle formé par la jambe par rapport au bassin(point J)
                else:
                    theta = 0
                if not angle_in(theta, [ma.pi/2,3*ma.pi/2]):    
                    angle += 1
                    return False
            if i_p >= 2 and p.hold is not None and l_p > 1e-5:# pour les mains, l'angle formé par le bras doit être dans le cône d'adhérence de la prise qu'il tient
                c = np.dot(vect_h[p.hold],Gp)/l_p
                if ma.acos(max(-1.0, min(1.0, c))) > coef_h[p.hold]:   # hors du cône
                    cone += 1
                    return False
                
        if not self.has_min_hold():    # si moins de deux membres accrochés dont au moins une main
            min_h += 1
            return False
        if not self.Fz_max()>=0:  # si la résultante, selon z est dirigée vers le bas ou selon y est dirigée dans sens opposé au mur 
            Fz += 1
            return False
        return True
    
    def Fz_max(self):    # fait bilan des forces, pour des forces maximales appliquées sur les prises
        P = - masse*9.81 # poids
        return sum(vect_Fmax_h[p.hold][2] for p in self.list_point if p.hold is not None) + P
    
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
            p.line.set_data_3d([parent[0], p.XYZ[0]],[parent[1], p.XYZ[1]], [parent[2], p.XYZ[2]])
        
        fig.canvas.draw()
        fig.canvas.flush_events()

class Seq:
    def __init__(self, ini_seq = None):
        self.Seq = copy.deepcopy(ini_seq) if ini_seq is not None else {}  # Seq = {t : {"p": index_p or None, "F" : F}}, None is for G move
        self.fit = 1e+5
        
    def add_F(self, T : int,  rd_mut_dict : dict):
            if T <= time[-1]:
                self.Seq[T] = rd_mut_dict    # F = {"type": ..., / "i_bdy": ,"r" : [theta,phi] / "i_p": , "F": / "i_p": , "i_h", "hold": / "F": }
        
    def rd_mut(self):
        if rd.random()>0.3: # dans 70% des cas on ajoute un mvt
            if rd.random()<0.5 and self.Seq.keys(): # dans 5O% des cas on prend le dernier instant du mvt
                T_new = min(time[-1], max(self.Seq.keys()) + 1) # dernier instanté
            else:   # sinon un instant ne contenant pas déjà un mvt 
                # instanté random contenant pas de mvt
                free_T = [t for t in range(len(time)) if t not in self.Seq]
                if not free_T:  # si tout les instants sont occupés
                    return
                T_new = rd.choice(free_T)
            
            C = init_climber()
            try :
                self.simu_to_T(C,T_new-1)   # on se place à l'intant d'avant l'ajout du mouvement
            except InvalidMove:
                return
            
            rd_mut = rd.choice([self.rot, self.to_hold, self.p, self.body])
            for _ in range(10): # 10 essaies pour faire une mutation valide
                Cc = copy.deepcopy(C)
                rd_mvt = rd_mut(Cc)
                if rd_mvt is not None:
                    try:
                        Cc.apply_move(rd_mvt)
                    except InvalidMove:
                        continue
                    self.add_F(T_new, rd_mvt)
                    break

        else:   # dans 30% des cas on en enléve un
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
        return {"type" : "p", "i_p" : i_p, "F" : F}

    def body(self,C):
        F = [rd.uniform(-max_F, max_F) for _ in range(3)]
        return {"type" : "body", "F" : F}
    
    def del_rd(self):    # supprime un mouvement au hasard
        if self.Seq:
            self.Seq.pop(rd.choice(list(self.Seq.keys())))

    
# ---------------GRAPH--------------- #

def create_fig():
    global fig, ax
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

# Création de la figure :
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
    x,y,z = zip(*Holds)
    ax.scatter(x,y,z, c='green', marker='X', s=50, depthshade=False)
    
    u,v,w = zip(*vect_h)
    ax.quiver(x,y,z,u,v,w)

def simulation(Seq, n_loop = -1):
    create_fig()
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

def d(A, B):
    AB = np.subtract(B,A)
    return ma.hypot(*AB)

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
    AB = np.subtract(B,A)
    return np.add(A,AB/2)

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

def init_pop(n) :     # créé une population initiale de taille n
    try:
        init_climber()
    except InvalidMove:
        raise InvalidMove("position initiale non valide")
    new_P = []
    for _ in range(n):
        ind = Seq()
        for _ in range(rd.randint(1,5)):
            ind.rd_mut()
        if ind.Seq:
            new_P.append(ind)
    return new_P
    

def mutation(selected : list) -> list :     # créé m variantes de chaque individu par mutations génétiques et des crossover
    Inds = []
    for ind in selected:
        for _ in range(n_m):
            
            for _ in range(10): # 10 essaies
                new_ind = copy.deepcopy(ind)
                
                for _ in range(rd.randint(1, max_mut)):
                    new_ind.rd_mut()
                    if new_ind is None:
                        break
                    
                if new_ind.Seq == ind.Seq or new_ind is None:
                    continue
                else:
                    Inds.append(new_ind)
                    break
            
            
    for _ in range(nb_crv):
        len_ini = len(Inds)
        essai = 0
        while len(selected)>=2 and len_ini == len(Inds) and essai < 10:
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


def insert(P, P_set, new_inds : list, type : str, height_ref): # insert les individues dans la pop par croissance de fistness 
    for ind in new_inds:
        if type=="height":
            ind.fit = fitness(ind)
        elif type=="opti":
            ind.fit = fitness_opti(ind, height_ref)
        
        k = seq_key(ind)
        if k not in P_set:# tout les individus dans la population sont différents
            P_set.add(k)
            P.append(ind)
    
    P.sort(key=lambda ind: ind.fit)


def fitness(ind : Seq, only_height = False) -> float:   # calcul du fitness d'un individu
    C = init_climber()
    try :
        ind.simu_to_T(C, time[-1])
    except InvalidMove:
        return float('inf')
    
    max_height_z = max(p.XYZ[2] for p in C.list_point if p.hold is not None)
    h_still = (Holds[-1][2] - max_height_z)**2
    if only_height:
        return h_still
    B_stil = d(C.B, Holds[-1])
    
    if h_still<=1e-5:    # si touche la dernière prise
        B_stil = 0
    return h_still + c_B*B_stil 

def move_cost(move): #  pour tout les mvt sauf to_hold(dépend de la position actuelle)
    type = move["type"]
    if type in ("p", "body"):
        return sum(f**2 for f in move["F"])
    if type == "rot":
        return move["r"][0]**2 + move["r"][1]**2
    return 0 

def fitness_opti(ind, high_ref): # évalue si l'individu à réussi à optimiser la grimpe tout en gardant au minimu la même hauteur final h
    C = init_climber()
    tot_F = 0
    try :
        for T in sorted(ind.Seq.keys()):
                    if T > time[-1]:
                        break
                    
                    if ind.Seq[T]["type"] != "to_hold":
                        tot_F += move_cost(ind.Seq[T])
                    else : # move "to_hold"
                        p = C.list_point[ind.Seq[T]["i_p"]]
                        h = ind.Seq[T]["hold"]
                        tot_F += d(p.XYZ, h)
                    C.apply_move(ind.Seq[T])
    except InvalidMove:
        return float('inf')  # +inf
    
    max_height_z = max(p.XYZ[2] for p in C.list_point if p.hold is not None)
    h_still = (Holds[-1][2] - max_height_z)**2
    
    nb_mvt = len(list(ind.Seq.keys())) if ind.Seq.keys() else 0
    
    if h_still > high_ref + 1e-6:
        return float('inf')
    return c_nb_mvt*nb_mvt + c_totF*tot_F

def seq_key(ind):
    return repr(sorted(ind.Seq.items()))    # chaine de caractère

# pour chaque gén on créé n_selc*m nouveaux individus par mutations génétiqeus et on les insert dans la pop qui est triée par fitness(croissant)
def evolution(type, ini_P):
    # initialisation
    height_ref = fitness(ini_P[0], True)     # avant de réinsérer

    P_set = set()
    P = []
    insert(P, P_set, ini_P, type, height_ref) # we need to reinsert to evaluate the fit with thid type

    # loop
    for g in range(1,gen+1):  
        P_selc = rd.choices(P, k = min(n_selc,len(P)), weights = [1/x for x in range(1,len(P)+1)])
        insert(P, P_set, mutation(P_selc), type, height_ref)   # permet de trier les individues selon leur fit, dépend donc de type
        P = P[: n + n_m + nb_crv]
        
        if g % int(n/3) == 0:
            print("\t\t",g,'ième génération : best fitness ',type,' : ',P[0].fit)
            #print(P[0].Seq)

    #print(P[0].Seq)
    return P


# ---------------PARAMETRE--------------- #

"Paramètres génétiques :"
n = 200                 # nombre d'individus dans la population
n_selc = int(n/6)       # nombre d'individus séléctionnés dans la population pour être muté, n_selc <= n
n_m = 3                   # nombre d'individues créés par mutation génétique pour 1 individu
max_mut = 5               # nombre maximal de mutations faites sur un individu
nb_crv = int(n/6)       # nombre de crossover fait par gen
gen = 500              # nombre de générations

c_height = 1
c_B = 0.01              # bonus for G height
c_nb_mvt = 1          # malus for size of Seq
c_totF = 0.1              # malus pour la quantité de mouvements effectués

"Paramètres climber :"
masse = 60          # masse(kg)
l_b = 1             # longueur bras
l_j = 1.5           # longueur jambe
l_dos = 0.7         # longeur du dos
max_F = 1         # norme maximal d'un mouvement
max_theta = ma.pi # angle maximal d'un movement de rotation polaire
max_phi = ma.pi     # angle maximal d'un movement de rotation azimut

"Paramètre de la voie"
Holds = [[0.7, 0.0, -0.7], [-0.7, 0.0, -0.7], [0.0, 0.0, 0.5599999999999999], [-0.7, 0.0, 0.35], [1.75, -0.7, 1.9249999999999998], [2.0999999999999996, -0.7, 3.5], [1.4, -0.7, 3.5], [0.0, -0.7, 2.8], [0.0, -0.7, 1.4], [1.0499999999999998, -0.7, 0.7], [0.7, -0.7, 2.0999999999999996], [1.0499999999999998, -1.0499999999999998, 4.8999999999999995], [2.0999999999999996, -1.0499999999999998, 4.8999999999999995], [1.4, -1.75, 6.3]]
coef_h = [3*ma.pi/4]*len(Holds)   # coefficient d'adhérence

Fmax_h = [400]*len(Holds)   # force maximal(N) que l'on peut appliquer sur une prise

# direction dans laquelle chaque prise peut tirer la main (à adapter prise par prise)
dir_h  = [(0, 0.5, 1)] * len(Holds)
vect_h = [tuple(c / ma.hypot(*d) for c in d) for d in dir_h]
vect_Fmax_h = [[Fmax*vect[i] for i in range(3)] for Fmax,vect in zip(Fmax_h,vect_h)]

"Paramètre temporels"
T_tot = 10  # Durée totale de la simulation en secondes
dt = 0.1    # une puissance de 10 négative
time = [i for i in range(round(1 + T_tot/dt)) ]  # Liste des instants de temps (int)


# ---------------MAIN--------------- #

def main():
    P = init_pop(10)
    for i in range(5):
        print("MODULE :",i)
        P = evolution("height", P) # garde les 10 meilleurs ind
        P = evolution("opti", P)   

    print(P[0].Seq)
    simulation(P[0].Seq)


import os
cpu = os.cpu_count() - 1

def module(P):
    global Fz, min_h, cone ,angle ,J_up_B ,not_l_dos ,not_l_p ,same_h
    Fz = min_h = cone = angle = J_up_B = not_l_dos = not_l_p = same_h = 0
    P = evolution("height", P)
    P = evolution("opti", P)
    P.sort(key=lambda ind: fitness(ind))    # tri par "vrai" fitness (pas celui de opti)
    counters = [Fz, min_h, cone ,angle ,J_up_B ,not_l_dos ,not_l_p ,same_h]
    return P,counters


spread = 10 # nb of ind spreading to neighbour islands
def main_mltp():
    P = init_pop(100)
    tot = [0]*8
    Ps = [copy.deepcopy(P) for _ in range(10)]
    for i in range(5):
        print("MODULE :",i)

        for i in range(0,len(Ps)-1):
            Ps[i] = Ps[i][:n-2*spread]
            Ps[i] += Ps[i-1][:spread] + Ps[i+1][:spread]
        Ps[-1] = Ps[-1][:n-2*spread]
        Ps[-1] += Ps[-2][:spread] + Ps[0][:spread]
        
        with mltp.Pool() as pool:
            results = pool.map(module, Ps)
        Ps = []
        for P_i, counters in results:
            Ps.append(P_i)
            for k in range(8):
                tot[k] += counters[k]
    best = max(sum(Ps, []), key=lambda ind: fitness(ind, True))
    print(best.Seq)
    print('fitness :',fitness(best),'height still :',fitness(best,True))
    print(tot)
    simulation(best.Seq)

Fz, min_h, cone ,angle ,J_up_B ,not_l_dos ,not_l_p ,same_h = [0]*8

if __name__ == '__main__':
    main_mltp()
