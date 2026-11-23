# vizualiser un NN qprès entrainement
print("importing imports")
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np
import torch
from torch import diff, nn
from tensordict import TensorDict
from tensordict.nn import TensorDictModule
from tensordict.nn.distributions import NormalParamExtractor
from torchrl.envs.utils import ExplorationType, set_exploration_type
from torchrl.modules import ProbabilisticActor, TanhNormal

from climber import (init_climber, create_fig, set_fig, show_holds,
                     InvalidPosition, T_tot, f)
from rd_voie import holds_generator
from brain import ClimbEnv, mlp, ACTION_DIM, OBS_DIM, rewards_w

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
    actions, rewards = [], []

    voie = holds_generator(diff)
    Holds = voie[0]
    type = voie[6]

    create_fig()
    set_fig(Holds)
    show_holds(Holds, type)
    axes = plt.axes([0.8, 0.01, 0.1, 0.05])
    button_skip = Button(axes,label = 'skip', color = 'gray')
    button_skip.on_clicked(skip_fig)
    
    C = init_climber(voie, do_graph=True)
    C.update_graph()
    env = ClimbEnv(diff)
    env.holds, env.C = voie, C
    txt = plt.text(0,0,f"pas 0")
    T = int(T_tot * f)
    
    # init reward
    env.prev_y_G = C.centre_masse()[1]
    env.prev_y_pmax = max(p.coord[1] for p in C.points.values())
    env.prev_y_pmin = min(p.coord[1] for p in C.points.values())
    env.prev_p_coord = np.asarray([env.C.points[k].coord for k in sorted(env.C.points)])
    
    obs = (env._obs() - loc) / scale
    with set_exploration_type(ExplorationType.DETERMINISTIC), torch.no_grad():
        for t in range(T):
            if skip["value"] == True:
                break
            
            td = policy(TensorDict({"observation": obs}, []))
            action = td["action"]
            actions.append(action)
            
            out = env._step(TensorDict({"action": action}, [])) # retourn _step et fait un step
            rewards.append(float(out["reward"].item()))
            obs = (out["observation"] - loc) / scale

            if bool(out["terminated"].item()):
                if C.win():
                    print(f"voie réussie au pas {t}")
                    C.update_graph()
                else:
                    print(f"chute au pas {t}")
                break

            C.update_graph()
            plt.pause(1 / f)
            if t % 10 == 0:
                txt.remove()
                txt = plt.text(0,0, f"pas {t}/{T}")
    if not skip["value"]:
        plt.pause(3)
    plt.close()
    skip["value"] = False
    return actions, rewards

# actions = [q_trc, x, y, 8 angles, 4 grips]
Name_action = {1 : "q_trc", 2 : "x_trc", 3 : "t_trc", 4 : "q_10", 5 : "q_11", 6 : "q_20", 7 : "q_21", 8 : "q_30", 9 : "q_31", 10 : "q_40", 11 : "q_41", 12 : "g_1", 13 : "g_2", 14 : "g_3", 15 : "g_4"}
def test(diff=0.5, n=None):
    for _ in range(n if n is not None else 10000):
        
        def graph(L):
            L = np.array(L)

            if len(L.shape) > 1:  # actions
                fig, axs = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
                for i in range(3):
                    axs[0].plot(L[:, i], label=Name_action[i + 1]) # q_trc, x_trc, t_trc

                axs[0].set_title("body")# 8 angles
                axs[0].legend()

                for i in range(3, 11):
                    axs[1].plot(L[:, i], label=Name_action[i + 1])

                axs[1].set_title("Angles")  #4 grips
                axs[1].legend(ncol=4)

                for i in range(11, 15):
                    axs[2].plot(L[:, i], label=Name_action[i + 1])

                axs[2].set_title("Grips")
                axs[2].legend(ncol=4)

                axs[3].axis("off")  # vide

                fig.suptitle("NN outputs")

                # bouton skip
                axes_button = plt.axes([0.85, 0.01, 0.1, 0.05])
                button_skip = Button(axes_button, label='skip', color='gray')
                button_skip.on_clicked(skip_fig)

            else:  # reward
                plt.figure()
                plt.plot(L)
                plt.title("Reward")

                axes_button = plt.axes([0.85, 0.01, 0.1, 0.05])
                button_skip = Button(axes_button, label='skip', color='gray')
                button_skip.on_clicked(skip_fig)

            while not skip["value"]:
                plt.pause(0.1)

            skip["value"] = False
            plt.close()
        
        for L in run(diff):
            graph(L)
        
        plt.close("all")



#brain['logs'] = ['reward', 'step_count', 'eval reward (sum)', 'eval step_count', 'couple']

def train_stats():
    reward,step_count,eval_reward,eval_step_count,couples = brain["logs"]["reward"],brain["logs"]["step_count"],brain["logs"]["eval reward (sum)"],brain["logs"]["eval step_count"],brain["logs"]["couple"]

    fig, axs = plt.subplots(2, 2, figsize=(8, 6))

    axs[0, 0].plot(range(len(reward)), reward)
    axs[0, 0].set_title("reward")
    axs[0, 1].plot(range(len(step_count)), step_count)
    axs[0, 1].set_title("step_count")
    axs[1, 0].plot(range(len(eval_reward)), eval_reward)
    axs[1, 0].set_title("eval reward (sum)")
    axs[1, 1].plot(range(len(couples)), couples)
    axs[1, 1].set_title("couples")

    plt.show()

# --- MAIN ---

#train_stats()
test(0.3,10)
