#-----------------------------------------GENERATOR-----------------------------------------#
import numpy as np
import random as rd
from climber import bdy_param,link_len,climber,InvalidPosition
"""générer des voies aléatoires"""

wall = (2.5, 5)   # 2,5 m largeur / 5 m hauteur
F_HOLD_MAX = 1000    # N

def holds_generator_rd(alpha): # densitée = nb prise/surface ou volumme de size, alpha = indice de difficulté in ]0,1[ : 1 dur, 0 facile
    l_dos, l_j_0, l_j_1, l_b_1, l_b_0 = [bdy_param["links"][((0,0),(0,1))]] + list(link_len(1)) + list(link_len(3))
    max_j = l_j_0 + l_j_1
    min_j = abs(l_j_0 - l_j_1)
    max_b = l_b_0 + l_b_1
    min_b = abs(l_b_0 - l_b_1)
    
    d = (1-alpha)*5
    v = np.prod(wall)
    n = int(d*v)
    f0, direc, Fv_max = [],[],[]
    # prises de départ(4)
    bassin = [rd.uniform(0,wall[0]),rd.uniform(0, 1)]  # coord du bassin pour commencer
    q_trc = rd.uniform(3*np.pi/4,np.pi/4)
    epaule = bassin + l_dos*np.array([np.cos(q_trc), np.sin(q_trc)])
    origin = [bassin]*2 + [epaule]*2
    
    thetas = sorted([rd.uniform(0,2*np.pi) for _ in range(4)],reverse=True)

    ls = [rd.uniform(min_j*1.2, max_j*0.8) for _ in range(2)] + [rd.uniform(min_b*1.2,max_b*0.8) for _ in range(2)]
    Holds = [[G[0] + np.cos(theta)*l, G[1] + np.sin(theta)*l] for l,theta,G in zip(ls,thetas,origin)]
    
    for _ in range(max(0,n-4)):
        for _ in range(5):
            new = [rd.uniform(0,s) for s in wall]
            if all(np.linalg.norm(np.subtract(new,h))>0.15 for h in Holds):
                Holds.append(new)
                break

    f0, direc, Fv_max = [], [], []
    for _ in Holds:
        f0.append(min(np.pi, max(0.1, rd.gauss((np.pi/2)*(1-alpha), 0.5))))
        theta = np.pi/2 + rd.gauss(0, alpha)*np.pi/2      # axe vers le haut
        direc.append([np.cos(theta), np.sin(theta)])
        Fv_max.append(max(200, F_HOLD_MAX*(1 - min(1, abs(rd.gauss(0, alpha))))))
    
    # permutation pour que la prise la plus haute soit en dernier
    top = max(Holds, key=lambda h: h[1])
    i = Holds.index(top)
    Holds[i],Holds[-1] = Holds[-1],Holds[i]
    Fv_max[i],Fv_max[-1] = Fv_max[-1],Fv_max[i]
    f0[i],f0[-1] = f0[-1],f0[i]
    direc[i],direc[-1] = direc[-1],direc[i]
    
    return Holds, Fv_max, f0, direc, bassin, q_trc

def holds_generator(alpha=None, stats=None):
    for i in range(100):
        a = rd.random() if alpha is None else alpha
        Holds, Fv_max, f0, direc, bassin, q_trc = holds_generator_rd(a)
        try:
            C = climber(bdy_param, Holds, f0, direc, Fv_max)
            C.init(bassin, q_trc)
            C.compute_couple()
        except InvalidPosition:
            continue
        if stats is not None:
            stats.append(i)
        return Holds, Fv_max, f0, direc, bassin, q_trc
    raise RuntimeError("Impossible de générer une voie valide après 100 essais")

def test(alpha=None):
    stats = []
    for i in range(1, 1001):
        holds_generator(alpha, stats)
        if i % 100 == 0:
            print(i, "sur 1000")
    print(f"moyenne essais : {sum(stats)/len(stats):.2f}, max : {max(stats)}")

