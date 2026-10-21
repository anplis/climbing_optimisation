from algo import *

# ---------------PARAM--------------- #

n = 200                 # nombre d'individus dans la population
n_mut = n//6        # nb d'individus muté
max_mut = 5               # nombre maximal de mutations faites sur un individu
n_crv = n//10       # nombre de crossover fait par gen
gen = 400              # nombre de générations
n_keep = 3          # nombre individues inchangés

# ---------------GEN--------------- #
def mutation(ind : list) -> list :     # créé m variantes de chaque individu par mutations génétiques et des crossover
    for _ in range(10): # 10 essaies
        new_ind = Seq()
        new_ind.Seq = copy.deepcopy(ind.Seq)
        
        for _ in range(rd.randint(1, max_mut)):
            new_ind.rd_mut()
            
        if new_ind.Seq == ind.Seq or new_ind is None:
            continue
        return new_ind

def cross_over(ind_1,ind_2):
    for _ in range(10):
        new_ind = crv(ind_1,ind_2)
        if new_ind is None or new_ind.Seq == ind_1.Seq or new_ind.Seq == ind_2.Seq:
            continue
        return new_ind

# ---------------EVO--------------- #

def evaluate(ind, type, height_ref):
    if type=="height":
        ind.fit = fitness(ind)
    elif type=="opti":
        ind.fit = fitness_opti(ind, height_ref)

# on prend n_selc ind que l'on remplace par leur mutation/cross_over
def evo(type, P):
    # initialisation
    height_ref = fitness(P[0], True)     # avant de réinsérer
    for ind in P:
        evaluate(ind,type,height_ref)
    P.sort(key=lambda ind: ind.fit)
    
    # loop
    for g in range(1,gen+1):
        # mutation
        for i_mut in rd.choices(range(n_keep,len(P)), k = n_mut, weights=[1/(1+x) for x in range(len(P)-n_keep)]):
            for _ in range(10):
                new_ind = mutation(P[i_mut])
                if new_ind is not None:
                    evaluate(new_ind,type,height_ref)
                    if new_ind.fit != float('inf'):
                        P[i_mut] = new_ind
                        break
        
        # cross over
        for _ in range(n_crv):
            i_1,i_2 = rd.choices(range(len(P)), k=2, weights=[1/(1+x) for x in range(len(P))])
            for _ in range(10): # 10 essaies
                new_ind = cross_over(P[i_1],P[i_2])
                if new_ind is not None:
                    evaluate(new_ind,type,height_ref)
                    if new_ind.fit != float('inf'):
                        P.append(new_ind)
                        break
        
        # sort, truncate 
        P.sort(key=lambda ind: ind.fit)
        P = P[: n + n_mut + n_crv]
        
        # evolution update
        if g % int(gen/3) == 0:
            print("\t\t",g,'ième génération : best fitness ',type,' : ',P[0].fit)

    return P



