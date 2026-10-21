# ---------------GEN_ALGO--------------- #

#from gen_re_use import *
from gen_replace import *

# ---------------NO MLTP--------------- #

def main():
    P = init_pop(n)
    global Fz, min_h, cone ,angle ,J_up_B ,not_l_dos ,not_l_p ,same_h
    Fz = min_h = cone = angle = J_up_B = not_l_dos = not_l_p = same_h = 0
    
    for _ in range(n_mod):
        P = evo("height", P)
        P = evo("opti", P)
        P.sort(key=lambda ind: fitness(ind))    # tri par "vrai" fitness (pas celui de opti)
    
    counters = [Fz, min_h, cone ,angle ,J_up_B ,not_l_dos ,not_l_p ,same_h]
    print(counters)
    print(P[0].Seq)
    simulation(P[0].Seq)

# ---------------MLTP--------------- #

import multiprocessing as mltp
import os
cpu = os.cpu_count() - 1

def module(P):
    global Fz, min_h, cone ,angle ,J_up_B ,not_l_dos ,not_l_p ,same_h
    Fz = min_h = cone = angle = J_up_B = not_l_dos = not_l_p = same_h = 0
    P = evo("height", P)
    P = evo("opti", P)
    P.sort(key=lambda ind: fitness(ind))    # tri par "vrai" fitness (pas celui de opti)
    counters = [Fz, min_h, cone ,angle ,J_up_B ,not_l_dos ,not_l_p ,same_h]
    return P,counters


spread = 10 # nb of ind spreading to neighbour islands
def main_mltp():
    P = init_pop(n)
    tot = [0]*8
    Ps = [copy.deepcopy(P) for _ in range(cpu)]
    for j in range(n_mod):
        print("MODULE :",j)

        # spreading with height fitness
        old = [P[:spread] for P in Ps]
        for i in range(len(Ps)):
            Ps[i] = Ps[i][:n-2*spread] + old[i-1] + old[(i+1) % len(Ps)]
        
        # evolution (height and opti)
        with mltp.Pool() as pool:
            results = pool.map(module, Ps)
        
        # recup results
        Ps = []
        for P_i, counters in results:
            Ps.append(P_i)
            for k in range(8):
                tot[k] += counters[k]
                
        best = min(sum(Ps, []), key=lambda ind: fitness(ind, True)) # meilleur fit sur toutes les iles
        print(best.Seq)
    print('fitness :',fitness(best),'height still :',fitness(best,True))
    print(tot)
    simulation(best.Seq)

n_mod = 5

if __name__ == '__main__':
    main_mltp()
