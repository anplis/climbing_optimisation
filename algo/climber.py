import numpy as np
import random as rd
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from copy import copy
 
#-----------------------------------------PARAM-----------------------------------------#
HEIGHT = 1.80   # m, taille du grimpeur
masse = 60      # kg
links_ref = {((0,0),(0,1)) : 0.39, ((0,0),(1,0)) : 0.195, ((1,0),(1,1)) : 0.285, ((0,0),(2,0)) : 0.195, ((2,0),(2,1)) : 0.285, ((0,1),(3,0)) : 0.265, ((3,0),(3,1)) : 0.254, ((0,1),(4,0)) : 0.265, ((4,0),(4,1)) : 0.254}
links = {k : HEIGHT*v for k,v in links_ref.items()}
 
def link_len(i):
    """Longueurs : origine -> genou/coude, genou/coude -> pied/main"""
    return bdy_param["links"][(ORIGIN[i],(i,0))], bdy_param["links"][((i,0),(i,1))]
 
points = {
    (0,0): "bassin",
    (0,1): "epaule",
    (1,0): "genou",
    (1,1): "pied",
    (2,0): "genou",
    (2,1): "pied",
    (3,0): "coude",
    (3,1): "main",
    (4,0): "coude",
    (4,1): "main",
    }
 
masse_perc_membre = {
    0 : [0.619],             # dos + bassin + tête
    1 : [0.10, 0.0465],
    2 : [0.10, 0.0465],
    3 : [0.028, 0.016],
    4 : [0.028, 0.016]
}
 
cm_solides = {  # position du CdM en fraction de la longueur du segment
    0 : [0.50],            # tronc
    1 : [0.433, 0.433],    # cuisse, tibia
    2 : [0.433, 0.433],
    3 : [0.436, 0.430],    # bras, avant-bras
    4 : [0.436, 0.430],
}
 
# masses des segments : (segment 1, segment 2)
masses_membre = {k : tuple(v*masse for v in vs) for k,vs in masse_perc_membre.items()}
 
bdy_param = {"links" : links, "points" : points, "cm_solides" : cm_solides, "masses_membre" : masses_membre}
 
ORIGIN = {1: (0,0), 2: (0,0), 3: (0,1), 4: (0,1)}
 
eps = 1e-4      # tolérance de déformation du corps (longueur des liens)
r_hold = 0.1    # m, distance pour rester accroché à une prise
 
v_max = 15      # m/s  (tronc)
w_max = 3       # rad/s (tronc et articulations)
C_max = masse*9.81
F_max = 300     # N, force max de translation du tronc
F_HOLD_MAX = 1000   # N, force verticale max qu'une prise peut encaisser (générateur)
 
Kp = 100
Kd = 10
min_q = 0.1
min_trs = 0.01
 
# Paramètres temporels
T_tot = 5   # Durée totale de la simulation en secondes
f = 100     # Hz
dt = 1/f
 
# Taille du mur (largeur, hauteur) en m
WALL = (2.5, 5)
 
# q = angles relatifs, tronc-cuisse/bras, cuisse/bras-pied/main
# w = dangle/dt
# convention : angle absolu mesuré depuis l'axe x, sens trigonométrique.
# tronc : q_trc = angle du vecteur bassin -> épaule (si debout = pi/2)
 
#-----------------------------------------CLASS-----------------------------------------#
 
 
class InvalidPosition(Exception):
    pass
 
class point:
    def __init__(self, coord, q, typ):
        self.type = typ   # in ["main","pied","bassin","epaule","coude","genou"]
        self.coord = np.asarray(coord, float)
        self.hold = None
        self.q = q
 
    def set_to(self, M):
        self.coord = np.asarray(M, float)
 
 
class line_viz:
    def __init__(self, p1, p2):
        self.p = [p1, p2]
 
FORM = {(1,1):"o",(2,1):"o",(3,1):".",(4,1):"."}
 
class climber:
    def __init__(self, bdy_param, Holds, f0, direc, Fv_max, do_graph=False):
        self.Holds = Holds
        self.f0 = f0
        self.direc = direc
        self.Fv_max = Fv_max
        self.points = {ix : point(np.zeros(2), 0, typ) for ix,typ in bdy_param["points"].items()}
        self.cm_solides = bdy_param["cm_solides"]
        self.masses_membre = bdy_param["masses_membre"]
        # masse ponctuelle équivalente du tronc (répartie sur bassin + épaule)
        self.m_tronc = self.masses_membre[0][0]
 
        self.links = bdy_param["links"] # dict
 
        self.trc_v = np.zeros(2)    # vitesse du tronc
        self.trc_w = 0.0            # vitesse angulaire du tronc
        self.q_limb = {i : np.zeros(2) for i in range(1,5)}   # angles relatifs des membres
 
        if do_graph:
            self.lines = []
            for ix,jx in self.links.keys():
                p1 = self.points[ix]
                p2 = self.points[jx]
                l = line_viz(p1, p2)
                self.lines.append(l)
                l.line = ax.plot(*zip(p1.coord, p2.coord), color="blue", linewidth=2)[0]
 
            self.scatters = {}
            for ix, p in self.points.items():
                if ix[0] != 0 and ix[1]==1:
                    self.scatters[ix] = ax.plot(*[[x] for x in p.coord], FORM[ix], color="red", markersize=6)[0]
 
    @property
    def q_trc(self):
        """Angle du tronc"""
        return angle(self.points[(0,1)].coord - self.points[(0,0)].coord)
 
    @property
    def holds_held(self):
        """Toutes les prises actuellement tenues """
        return {p.hold for p in self.points.values() if p.hold is not None}
 
    def init(self, bassin, q_trc):
        """bassin : coordonnées du bassin ; q_trc : angle du tronc"""
        self.trc_v = np.zeros(2)
        self.trc_w = 0.0
 
        B, E = self.points[(0,0)], self.points[(0,1)]
        B.coord = np.array(bassin, float)
        E.coord = B.coord + self.links[((0,0),(0,1))]*np.array([np.cos(q_trc), np.sin(q_trc)])
 
        for i in range(1,5): # 4 prises de départ
            if not self.grab(i, i-1, init=True):
                raise InvalidPosition(f"le membre {i} n'atteint pas la prise:{self.Holds[i-1]}")
 
    def grab(self, i, i_h, init=False): # téléporte le membre i sur la prise si c'est physiquement possible
        """Accroche le membre i à la prise i_h si elle est atteignable, retourne True si réussi"""
        o = self.points[ORIGIN[i]].coord
        h = np.array(self.Holds[i_h], float)
        l0, l1 = link_len(i)
        dist = d(h, o)
        # atteignable seulement si |l0-l1| <= dist <= l0+l1
        if dist > (l0 + l1) or dist < abs(l0 - l1):
            return False
 
        p1, p0 = self.points[(i,1)], self.points[(i,0)]
        p1.coord = h
        p1.hold = i_h
        p0.coord = self.coord_knee(i, init)
        self.q_limb[i] = chain_angles(o, p0.coord, h, self.q_trc + np.pi)
        return True
 
    def coord_knee(self, i, init=False):
        """Coordonnées du genou/coude connaissant l'origine et la main/pied (intersection de cercles)"""
        p1 = self.points[(i,1)].coord
        p0 = self.points[(i,0)].coord
        l0, l1 = link_len(i)
        o = self.points[ORIGIN[i]].coord
 
        AB = p1 - o
        dist = np.linalg.norm(AB)
        if dist < 1e-12:
            return p0  # dégénéré : origine et extrémité confondues
 
        adj = (l0**2 - l1**2 + dist**2) / (2*dist)
        # clamp pour éviter sqrt d'un négatif à cause d'arrondis
        hyp = np.sqrt(max(0.0, l0**2 - adj**2))
 
        C = o + adj * AB / dist
        perp = np.array([-AB[1], AB[0]]) / dist
        P1 = C + hyp * perp
        P2 = C - hyp * perp
        if i<=2:#jambe
            if init:
                return max((P1,P2), key=lambda P : P[1])
            return min((P1, P2), key=lambda p: np.linalg.norm(p - p0))  # le plus proche de l'ancienne position
        else:
            return min((P1, P2), key=lambda p: p[1])    # coude vers la bas
 
    def nearest_hold(self, i):
        p1 = self.points[(i,1)].coord
        return min(([k for k in range(len(self.Holds)) if k not in self.holds_held]),
                        key=lambda k: d(p1, self.Holds[k]))
 
    def dist_nearest_k_hold(self, i, k):
        p1 = self.points[(i,1)].coord
        return sorted(d(h,p1) for h in self.Holds)[:k]
 
    def can_hold(self, i, i_h = None):
        p1 = self.points[(i,1)]
        if p1.hold is not None:
            return True
        if i_h is None:
            i_h = self.nearest_hold(i)
        o = self.points[ORIGIN[i]].coord
        l0,l1 = link_len(i)
        return d(p1.coord, self.Holds[i_h]) < r_hold and abs(l0 - l1) <= d(o, self.Holds[i_h]) <= l0 + l1
 
    def try_hold(self, i):
        """Accroche les mains/pieds en fct de leur coordonnées"""
        p1 = self.points[(i,1)]
        if p1.hold is None:
            i_h = self.nearest_hold(i)
            if self.can_hold(i,i_h):
                p1.hold = i_h
                p1.coord = np.array(self.Holds[i_h], float)
 
    def solve_limbs(self, q_target):
        """Calcule la pose des 4 membres
        - membre libre    : l'angle relatif rejoint la cible à w_max au plus
        - membre accroché : cinématique inverse vers la prise
        \nRetourne (coords, q_limb)"""
        coords, q_new = {}, {}
        for i in range(1,5):
            o = self.points[ORIGIN[i]].coord
            p0_ix, p1_ix = (i,0), (i,1)
            p1 = self.points[p1_ix]
            if p1.hold is not None:
                h = np.array(self.Holds[p1.hold], float)
                p0 = self.coord_knee(i)
                coords[p0_ix], coords[p1_ix] = p0, h
                q_new[i] = chain_angles(o, p0, h, self.q_trc + np.pi)
            else:
                q = np.array(self.q_limb[i], float)
                step = np.clip(wrap(q_target[i-1] - q), -w_max*dt, w_max*dt)
                q = wrap(q + step)
                p0, p1 = chain_coords(o, self.q_trc + np.pi, q, link_len(i))
                coords[p0_ix], coords[p1_ix] = p0, p1
                q_new[i] = q
        return coords, q_new
 
    def mvt(self, output, no_physic = False):
        """output = [q_trc, x_trc, y_trc,  q10,q11, q20,q21, q30,q31, q40,q41,  g1,g2,g3,g4]
        q_trc, x_trc, y_trc : angle et position (du milieu) cibles du tronc
        qi0, qi1            : angles relatifs cibles du membre i
        gi                  : 1 = le membre i s'accroche, 0 = il lâche"""
        output = np.asarray(output, float)
 
        grip = output[11:15]    # between 0 and 1
        for i in range(1,5):
            if grip[i-1] > 0.5:
                self.try_hold(i)
            else:
                self.points[(i,1)].hold = None
 
        q_bdy, x_bdy, y_bdy = output[:3]
        q_target = output[3:11].reshape(4,2)
 
        B, E = self.points[(0,0)], self.points[(0,1)]
 
        # déplace le tronc
            # rotation autour du milieu
        if abs(wrap(q_bdy - self.q_trc)) > min_q:
            C = PD(self.q_trc, self.trc_w, q_bdy, C_max, ang=True)
            self.rot_bdy(C)
        else:
            self.trc_w = 0.0
 
            # translation
        mid = (B.coord + E.coord)/2
        cible = np.array([x_bdy, y_bdy])
        if d(mid, cible) > min_trs:
            F = PD(mid, self.trc_v, cible, F_max)
            self.trs_bdy(F)
        else:
            self.trc_v = np.zeros(2)
 
        # déplace les membres (libres) et fige les membres accrochés sur leur prise
        coords, self.q_limb = self.solve_limbs(q_target)
        for ix, c in coords.items():
            self.points[ix].coord = c
 
        if no_physic:
            return np.zeros((4,2))
        C = self.compute_couple()
        return C
 
    def centre_masse(self):
        """CdM global = somme pondérée des CdM de chaque segment."""
        G = np.zeros(2)
        # tronc : 1 segment (bassin -> épaule), CdM à cm_solides[0][0] du segment
        A, B = self.points[(0,0)].coord, self.points[(0,1)].coord
        m = self.masses_membre[0][0]
        G += m * (A + self.cm_solides[0][0] * (B - A))
        # membres : 2 segments
        for i in range(1,5):
            o = self.points[ORIGIN[i]].coord
            p0 = self.points[(i,0)].coord
            p1 = self.points[(i,1)].coord
            m0, m1 = self.masses_membre[i]
            c0, c1 = self.cm_solides[i]
            G += m0 * (o + c0 * (p0 - o)) + m1 * (p0 + c1 * (p1 - p0))
        return G / masse
 
    def compute_couple(self):
        """tableau (4,2), couples statiques de chaque membre"""
        G = self.centre_masse()
        i_p_held = [i for i in range(1,5) if self.points[(i,1)].hold is not None]
        holds_held = [self.points[(i,1)].hold for i in i_p_held]
        f0_h = [self.f0[h] for h in holds_held]
        direc_h = [self.direc[h] for h in holds_held]
        Fv_h = [self.Fv_max[h] for h in holds_held]
        F_ctc = contact_forces([self.points[(i,1)].coord for i in i_p_held], G, f0_h, Fv_h, direc_h)
        if F_ctc is None:
            raise InvalidPosition("aucune solution d'équilibre respectant le frottement")
        F_ctc_membre = {i : F_ctc[k] for k,i in enumerate(i_p_held)}
 
        C = np.zeros((4,2))
        for i in range(1,5):
            chaine = [self.points[ORIGIN[i]].coord, self.points[(i,0)].coord, self.points[(i,1)].coord]
            masses = list(self.masses_membre[i])
            C[i-1] = couple_static(chaine, masses, self.cm_solides[i],F_ctc_membre.get(i, np.zeros(2)))
        return C
 
    def rot_bdy(self, C):
        """Rotationne le tronc autour de son milieu, avec un couple C"""
        B = copy(self.points[(0,0)].coord)
        E = copy(self.points[(0,1)].coord)
        c = (B + E)/2       # milieu
        h = (E - B)/2       # demi-vecteur milieu -> épaule
        m = self.m_tronc            # masse totale du tronc répartie sur les 2 points
        J = m * d(B, E)**2 / 4      # inertie : 2 masses ponctuelles à L/2 du milieu
        self.trc_w = float(np.clip(self.trc_w + C/J*dt, -w_max, w_max)) # limite la vitesse
        h = rot(h, self.trc_w*dt)
        self.points[(0,0)].coord = c - h
        self.points[(0,1)].coord = c + h
        if not self.are_reachable(): # on annule le mvt
            self.points[(0,0)].coord = B
            self.points[(0,1)].coord = E
            self.trc_w = 0.
 
    def trs_bdy(self, F):
        """Translation du tronc, avec une force F"""
        B = copy(self.points[(0,0)].coord)
        E = copy(self.points[(0,1)].coord)
        a = F / self.m_tronc
        self.trc_v = self.trc_v + a*dt
 
        norm = np.linalg.norm(self.trc_v)
        if norm > v_max:            # limite la vitesse
            self.trc_v = self.trc_v * v_max/norm
 
        self.points[(0,0)].coord = B + self.trc_v*dt
        self.points[(0,1)].coord = E + self.trc_v*dt
        if not self.are_reachable(): # on annule la translation
            self.points[(0,0)].coord = B
            self.points[(0,1)].coord = E
            self.trc_v = np.zeros(2)
 
    def are_reachable(self):
        for i in range(1,5):
            i_h = self.points[(i,1)].hold
            if i_h is not None :
                o = self.points[ORIGIN[i]].coord
                l0, l1 = link_len(i)
                if not (abs(l0 - l1) <= d(o, self.Holds[i_h]) <= l0 + l1):
                    return False
        return True
 
    def win(self):
        return any(p.hold == len(self.Holds)-1 for p in [self.points[3,1], self.points[4,1]])
 
    def update_graph(self):
        for l in self.lines:
            p1, p2 = l.p
            l.line.set_data(*zip(p1.coord, p2.coord))
        for ix, p in self.points.items():
            if ix[0] != 0 and ix[1] == 1:
                sc = self.scatters[ix]
                sc.set_data(*[[x] for x in p.coord])
                sc.set_color("green" if p.hold is not None else "red")
 
    def erease(self):
        for l in self.lines:
            l.line.remove()
        for ix in self.scatters:
            self.scatters[ix].remove()
 
#-----------------------------------------FUNC-----------------------------------------#
 
def vect(A, B):
    return np.subtract(B, A)
 
def d(A, B):
    return np.linalg.norm(vect(A, B))
 
def rot(A, q):
    return np.array([[np.cos(q), -np.sin(q)], [np.sin(q), np.cos(q)]]) @ A
 
def cross(a, b):
    return a[0]*b[1] - a[1]*b[0]
 
def angle(v):
    return np.arctan2(v[1], v[0])
 
def wrap(a):
    """Ramène un angle (ou un tableau d'angles) dans [-pi, pi)."""
    return (a + np.pi) % (2*np.pi) - np.pi
 
 
#-----------------------------------------CALCUL-----------------------------------------#
 
 
def PD(q0, w0, q_trgt, Cmax, ang=False):
    """Si ang=True : l'erreur d'angle est ramenée dans [-pi, pi), le couple/la force est limité à Cmax (en norme)"""
    e = np.asarray(q_trgt - q0, float)
    if ang:
        e = wrap(e)
    C = Kp*e - Kd*np.asarray(w0, float)
    norm = np.linalg.norm(C)
    return C if norm <= Cmax else C*Cmax/norm
 
 
def couple_static(pts, masses, cms, F_ctc=np.zeros(2)):
    """Couples que chaque moteur doit fournir pour que la chaîne soit à l'équilibre.
    F_ctc : force exercée par la prise sur l'extrémité ([0,0] si membre libre).
    CdM de chaque segment pris aux positions réelles (via cm_solides)."""
    C = np.zeros(2)
    # positions des CdM réels des 2 segments
    cms = [pts[0] + cms[0]*(pts[1]-pts[0]), pts[1] + cms[1]*(pts[2]-pts[1])]
    for i in range(2):
        # moment au point pivôt i : forces de contact + poids des segments en aval
        C[i] = -(cross(F_ctc, pts[i]-pts[-1])
                 + sum(cross(masses[j] * np.array([0,-9.81]), pts[i]-cms[j]) for j in range(i, 2)))
    return C
 
 
def contact_forces(ctc_pts, G, f0, Fv_max, direc):
    """
    Retourne tableau (n, 2) forces de contact (repère global),
    ou None si aucune solution.
    f0, Fv_max, direc : une valeur par point de contact (déjà filtrés).
    """
    # Conventions :
    # F_c[k] = [Fn, Ft] dans le repère de la prise (axe = direc[k])
    # Fn >= 0, |Ft| <= f0*Fn, composante verticale globale <= Fv_max
    n = len(ctc_pts)
    if n == 0:
        return None
 
    ctc_pts = np.asarray(ctc_pts, dtype=float)
    G = np.asarray(G, dtype=float)
 
    # Matrice de rotation dans repère : normale, tangent
    R = []
    for k in range(n):
        theta = angle(np.asarray(direc[k], dtype=float))
        R.append(np.array([ [np.cos(theta), -np.sin(theta)],
                            [np.sin(theta),  np.cos(theta)]]))
 
    A = np.zeros((3, 2*n))
 
    for k in range(n):
        Rk = R[k]
        x, y = ctc_pts[k] - G
 
        # Équilibre des forces
        A[0:2, 2*k:2*k+2] = Rk
 
        # Moment de la force au point de contact en G
        A[2, 2*k:2*k+2] = np.array([-y, x]) @ Rk
 
    b = np.array([0.0, masse*9.81, 0.0])
 
    # Fn >= 0, Ft pas borne
    bounds = []
    for k in range(n):
        bounds.append((0.0, None))  # Fn
        bounds.append((None, None)) # Ft
 
    # Minimisation de la norme des forces, sous contraintes
    # d'équilibre et de frottement.
    def norm(F):
        return F @ F
 
    constraints = [{"type": "eq",
                    "fun": lambda F: A @ F - b}]
    for k in range(n):
        f0k = f0[k]             # ||Ft|| <= f0||Fn||
        constraints.append({
            "type": "ineq",
            "fun" : lambda F, k=k, f0k=f0k: f0k*F[2*k] - F[2*k+1]})
        constraints.append({
            "type": "ineq",
            "fun" : lambda F, k=k, f0k=f0k: f0k*F[2*k] + F[2*k+1]})
        constraints.append({    # composante verticale (globale) <= force verticale max de la prise
            "type": "ineq",
            "fun" : lambda F, k=k : Fv_max[k] - (R[k] @ F[2*k:2*k+2])[1]})
    # Initialisation :
    x0 = np.zeros(2*n)
    x0[::2] = masse*9.81/n
 
    result = minimize(
        norm,
        x0,
        method="SLSQP",
        bounds=bounds,
        constraints=constraints,)
 
    if not result.success:
        return None
 
    # Conversion des forces locales vers le repère global
    F_global = np.zeros((n, 2))
    for k in range(n):
        F_local = result.x[2*k:2*k+2]
        F_global[k] = R[k] @ F_local
 
    return F_global
 
 
def chain_coords(origin, theta0, Q, L):
    """theta0 = angle trc + pi, Q = angles relatifs, L = longueurs.
    Retourne les coords des points"""
    theta = theta0
    p0 = np.asarray(origin, float)
    out = []
    for qi, Li in zip(Q, L):
        theta = theta + qi
        p0 = p0 + Li*np.array([np.cos(theta), np.sin(theta)])
        out.append(p0)
    return out
 
def chain_angles(o, p0, p1, theta0):
    """Angles relatifs d'un membre"""
    th1 = angle(p0 - o)
    th2 = angle(p1 - p0)
    return np.array([wrap(th1 - theta0), wrap(th2 - th1)])
 
#-----------------------------------------GRAPH-----------------------------------------#
 
def create_fig():
    global fig, ax
    fig, ax = plt.subplots()
 
def set_fig(Holds):
    ax.set_title("Climbing simulation")
    marge = 1
    ax.set_xlim(min(h[0] for h in Holds)-marge, max(h[0] for h in Holds)+marge)
    ax.set_ylim(min(h[1] for h in Holds)-marge, max(h[1] for h in Holds)+marge)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_aspect('equal')
    ax.grid(False)
 
def show_holds(Holds):
    ax.scatter(*zip(*Holds), c='black', marker='v', s=30)
 
def simulation(Seq, voie, no_physic = False, n_loop=-1):
    """voie = (Holds, Fv_max, f0, direc, bassin, q_trc)"""
    Holds = voie[0]
    create_fig()
    set_fig(Holds)
    show_holds(Holds)
    plt.ion()
    plt.show()
 
    while plt.fignum_exists(fig.number) and (n_loop < 0 or n_loop > 0):
        C = init_climber(voie, do_graph=True)
        C.update_graph()
        for _, output in zip(range(int(T_tot*f)), Seq):   # s'arrête si Seq plus court
            C.mvt(output,no_physic)
            C.update_graph()
            plt.pause(dt)
        plt.pause(5)
        C.erease()
        n_loop -= 1
    plt.ioff()
 
#-----------------------------------------INIT-----------------------------------------#
 
def init_climber(voie, do_graph=False):
    """voie = (Holds, Fv_max, f0, direc, bassin, q_trc)"""
    Holds, Fv_max, f0, direc, bassin, q_trc = voie
    C = climber(bdy_param, Holds, f0, direc, Fv_max, do_graph)
    # départ : pieds sur les prises 0 et 1, mains sur les prises 2 et 3
    C.init(bassin, q_trc)
    return C
