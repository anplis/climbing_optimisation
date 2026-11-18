# vizualiser un NN qprès entrainement
print("importing imports")
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np
import torch
from torch import nn
from tensordict import TensorDict
from tensordict.nn import TensorDictModule
from tensordict.nn.distributions import NormalParamExtractor
from torchrl.envs.utils import ExplorationType, set_exploration_type
from torchrl.modules import ProbabilisticActor, TanhNormal

from climber import (init_climber, create_fig, set_fig, show_holds,
                     InvalidPosition, T_tot, f)
from rd_voie import holds_generator
from brain import ClimbEnv, mlp, ACTION_DIM, OBS_DIM   

print("import imported")

# --- chargement ---

print("loading brain")
brain = torch.load("data/brain_1.pt", weights_only=False)
print("brain loaded")

# --- même architecture qu'à l'entraînement ---
action_spec = ClimbEnv(0.5).action_spec
policy = ProbabilisticActor(
    module=TensorDictModule(
        nn.Sequential(mlp(2 * ACTION_DIM), NormalParamExtractor()),
        in_keys=["observation"], out_keys=["loc", "scale"]),
    spec=action_spec,
    in_keys=["loc", "scale"],
    distribution_class=TanhNormal,
    distribution_kwargs={"low": action_spec.space.low,
                         "high": action_spec.space.high},
    return_log_prob=True,
)
policy(TensorDict({"observation": torch.zeros(OBS_DIM)}, []))  # matérialise les LazyLinear
policy.load_state_dict(brain["policy"])
policy.eval()

# --- stats de normalisation sauvegardées ---
loc = brain["obs_norm"]["loc"]
scale = brain["obs_norm"]["scale"]

skip = {"value" : False}
def skip_fig(event):
    skip["value"] = True

def run(diff):
    outputs = []

    voie = holds_generator(diff)
    Holds = voie[0]
    type = voie[6]

    create_fig()
    set_fig(Holds)
    show_holds(Holds, type)
    axes = plt.axes([0.8, 0.01, 0.1, 0.05])
    button_skip = Button(axes,label = 'skip', color = 'gray')
    button_skip.on_clicked(skip_fig)
    
    # axe = 
    # button_stop = Button(axe,label = 'stop', color = 'red')
    # button_stop.on_clicked(lambda event: exit())
    C = init_climber(voie, do_graph=True)
    env = ClimbEnv(diff)
    env.holds, env.C = voie, C
    txt = plt.text(0,0,f"pas 0")
    T = int(T_tot * f)
    with set_exploration_type(ExplorationType.DETERMINISTIC), torch.no_grad():
        for t in range(T):
            if skip["value"] == True:
                break
            obs = (env._obs() - loc) / scale
            td = policy(TensorDict({"observation": obs}, []))
            output = td["action"].numpy().astype(float)
            outputs.append(output)
            try:
                C.mvt(output)
            except InvalidPosition:
                print(f"chute au pas {t}")
                break
            if C.win():
                print(f"voie réussie au pas {t}")
                break
            C.update_graph()
            plt.pause(1 / f)
            if t % 10 == 0:
                txt.remove()
                txt = plt.text(0,0, f"pas {t}/{T}")
    if not skip["value"]:
        plt.pause(3)
    plt.clf()
    skip["value"] = False
    return outputs

def test(diff=0.5, n=None):
    for _ in range(n if n is not None else 10000):
        outputs = run(diff)

        outputs = np.array(outputs)
        for i in range(outputs.shape[1]):
            plt.plot(outputs[:,i],label = i+1)
            
        plt.legend()
        axes = plt.axes([0.8, 0.01, 0.1, 0.05])
        button_skip = Button(axes,label = 'skip', color = 'gray')
        button_skip.on_clicked(skip_fig)

        while skip["value"] == False:
            plt.pause(0.1)
        skip["value"] = False
        plt.close("all")
            

#brain['logs'] = ['reward', 'step_count', 'eval reward (sum)', 'eval step_count']

def train_stats():
    reward,step_count,eval_reward,eval_step_count,couples = brain["logs"]["reward"],brain["logs"]["step_count"],brain["logs"]["eval reward (sum)"],brain["logs"]["eval step_count"],brain["logs"]["couple"]

    fig, axs = plt.subplots(2, 2, figsize=(8, 6))

    axs[0, 0].plot(range(len(reward)), reward)
    axs[0, 0].set_title("reward")
    axs[0, 1].plot(range(len(step_count)), step_count)
    axs[0, 1].set_title("step_count")
    axs[1, 0].plot(range(len(eval_reward)), eval_reward)
    axs[1, 0].set_title("eval reward (sum)")
    axs[1, 1].plot(range(len(eval_step_count)), eval_step_count)
    axs[1, 1].set_title("eval step_count")
    
    fig_c = plt.figure()
    plt.plot(range(len(couples)), couples)
    plt.title("couple")

    plt.show()

# --- MAIN ---

#train_stats()
test(0.1,10)
