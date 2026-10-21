from algo import *

# ---------------PARAM--------------- #

n = 100                 # nombre d'individus dans la population
n_selc = int(n/4)       # nombre d'individus séléctionnés dans la population pour être muté, n_selc <= n
n_m = 3                   # nombre d'individues créés par mutation génétique pour 1 individu
max_mut = 5               # nombre maximal de mutations faites sur un individu
nb_crv = int(n/6)       # nombre de crossover fait par gen
gen = 400              # nombre de générations

# ---------------GEN--------------- #
def mutation(selected : list) -> list :     # créé m variantes de chaque individu par mutations génétiques et des crossover
    Inds = []
    for ind in selected:
        for _ in range(n_m):
            
            for _ in range(10): # 10 essaies
                new_ind = copy.deepcopy(ind)
                
                for _ in range(rd.randint(1, max_mut)):
                    new_ind.rd_mut()
                    
                if new_ind.Seq == ind.Seq or new_ind is None:
                    continue
                else:
                    Inds.append(new_ind)
                    break
    return Inds

def cross_over(selected : list) -> list:
    Inds = []
    for _ in range(nb_crv):
        len_ini = len(Inds)
        essai = 0
        while len(selected)>=2 and len_ini == len(Inds) and essai < 10:
            ind_1, ind_2 = rd.sample(selected, k = 2) # tirage sans remise de 2 individus
            if ind_1.Seq != ind_2.Seq:
                new_ind = crv(ind_1, ind_2)
                if new_ind is not None:
                    Inds.append(new_ind)
            essai += 1

    return Inds
# ---------------EVO--------------- #

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

def seq_key(ind):
    return repr(sorted(ind.Seq.items()))    # chaine de caractère

# pour chaque gén on créé n_selc*m nouveaux individus par mutations génétiqeus et on les insert dans la pop qui est triée par fitness(croissant)
def evo(type, ini_P):
    # initialisation
    height_ref = fitness(ini_P[0], True)     # avant de réinsérer

    P_set = set()
    P = []
    insert(P, P_set, ini_P, type, height_ref) # we need to reinsert to evaluate the fit with thid type

    # loop
    for g in range(1,gen+1):  
        P_selc = rd.choices(P, k = min(n_selc,len(P)), weights = [1/x for x in range(1,len(P)+1)])
        insert(P, P_set, mutation(P_selc)+cross_over(P_selc), type, height_ref)   # permet de trier les individues selon leur fit, dépend donc de type
        P = P[: n + n_m + nb_crv]
        
        if g % int(gen/3) == 0:
            print("\t\t",g,'ième génération : best fitness ',type,' : ',P[0].fit)
            #print(P[0].Seq)

    #print(P[0].Seq)
    return P

