#-----------------------------------------IMPORT-----------------------------------------#
import os
from collections import defaultdict

import numpy as np
import torch
from tensordict import TensorDict
from tensordict.nn import TensorDictModule
from tensordict.nn.distributions import NormalParamExtractor
from torch import load, nn
from torchrl.collectors import Collector
from torchrl.data import Bounded, Composite, Unbounded
from torchrl.data.replay_buffers import ReplayBuffer
from torchrl.data.replay_buffers.samplers import SamplerWithoutReplacement
from torchrl.data.replay_buffers.storages import LazyTensorStorage
from torchrl.envs import (Compose, DoubleToFloat, EnvBase, ObservationNorm,
                          ParallelEnv, StepCounter, TransformedEnv)
from torchrl.envs.utils import ExplorationType, set_exploration_type
from torchrl.modules import ProbabilisticActor, TanhNormal, ValueOperator
from torchrl.objectives import ClipPPOLoss
from torchrl.objectives.value import GAE
from tqdm import tqdm

from climber import init_climber, InvalidPosition, T_tot, f, d
from rd_voie import holds_generator


#-----------------------------------------PARAM-----------------------------------------#
num_workers = 15
max_diff = 0.5

num_cells = 256
lr = 3e-4
max_grad_norm = 1.0

frames_per_batch = 4500 # multiple de nuw_workers
total_frames = 450_000

sub_batch_size = 100 # frames_per_batch doit être un multiple de sub_batch_size
num_epochs = 10
clip_epsilon = 0.2
gamma = 0.99
lmbda = 0.95
entropy_eps = 1e-4

n_hold_visible = 5
OBS_DIM = 42 + 4 * n_hold_visible
ACTION_DIM = 15


#-----------------------------------------ENVIRONNEMENT-----------------------------------------#
class ClimbEnv(EnvBase):
    def __init__(self, diff):
        super().__init__(device="cpu", batch_size=[])
        self.diff = diff
        self.C = None
        self.holds = None

        # actions = [q_trc, x, y, 8 angles, 4 grips]
        low = [-np.pi, -5, -5] + [-np.pi] * 8 + [0] * 4
        high = [np.pi, 5, 5] + [np.pi] * 8 + [1] * 4
        self.action_spec = Bounded(
            low=torch.tensor(low, dtype=torch.float32),
            high=torch.tensor(high, dtype=torch.float32),
            shape=(ACTION_DIM,), dtype=torch.float32)
        self.observation_spec = Composite(
            observation=Unbounded(shape=(OBS_DIM,), dtype=torch.float32),
            couple = Unbounded(shape=(1,), dtype=torch.float32))
        self.reward_spec = Unbounded(shape=(1,), dtype=torch.float32)

    def _obs(self):
        C = self.C
        G = C.centre_masse()
        goal = d(C.points[(0, 0)].coord, self.holds[-1])
        q = [C.q_limb[i] for i in range(1, 5)]
        coord = [C.points[k].coord for k in sorted(C.points)]
        held = [float(C.points[(i, 1)].hold is not None) for i in range(1, 5)]
        can_hold = [float(C.can_hold(i)) for i in range(1, 5)]
        dist_holds = [C.dist_nearest_k_hold(i, n_hold_visible) for i in range(1, 5)]

        brut = coord + q + [G, goal, [C.trc_w], C.trc_v, held, can_hold] + dist_holds
        obs = np.concatenate([np.ravel(x) for x in brut])
        return torch.tensor(obs, dtype=torch.float32)

    def _reset(self, td = None):
        self.holds = holds_generator(self.diff)
        self.C = init_climber(self.holds, do_graph=False)
        self.prev_y = self.C.centre_masse()[1]
        self.prev_y_p = max(p.coord[1] for p in self.C.points.values())
        return TensorDict({"observation": self._obs(),
                           "done": torch.zeros(1, dtype=torch.bool),
                           "terminated": torch.zeros(1, dtype=torch.bool),
                           "couple" : torch.zeros(1, dtype=torch.float32)},
                           batch_size=[])

    def _step(self, td):
        output = td["action"].cpu().numpy().astype(float)
        terminated, reward, couple = False, 0.0, 0.0
        try:
            Couples = self.C.mvt(output)
            couple = np.abs(Couples).sum()
            y = self.C.centre_masse()[1]
            y_p = max(p.coord[1] for p in self.C.points.values())
            reward = 10.0 * (y - self.prev_y) + 10 * (y_p - self.prev_y_p) + 0.1 / (1 + couple)
            self.prev_y = y
            self.prev_y_p = y_p
            if self.C.win():
                reward += 100.0
                terminated = True
        except InvalidPosition:
            terminated, reward = True, -10.0  # chute ou position impossible
        return TensorDict({"observation": self._obs(),
                        "reward": torch.tensor([reward], dtype=torch.float32),
                        "done": torch.tensor([terminated]),
                        "terminated": torch.tensor([terminated]),
                        "couple" : torch.tensor([couple], dtype=torch.float32),},
                        batch_size=[])

    def _set_seed(self, seed):
        torch.manual_seed(seed)


def make_env(diff):
    """diff dans [0, 1] : difficulté de la voie générée."""
    def _create():
        return ClimbEnv(diff)
    return _create



#-----------------------------------------MODELES-----------------------------------------#
def mlp(out_dim):
    return nn.Sequential(
        nn.LazyLinear(num_cells), nn.Tanh(),
        nn.LazyLinear(num_cells), nn.Tanh(),
        nn.LazyLinear(num_cells), nn.Tanh(),
        nn.LazyLinear(out_dim),
    )


#-----------------------------------------MAIN-----------------------------------------#
def compute_obs_stats(n_steps=400):
    obs = []
    for diff in np.linspace(0, max_diff, num_workers):
        e = ClimbEnv(diff)
        r = e.rollout(n_steps, break_when_any_done=False)
        obs.append(r["observation"])
    obs = torch.cat(obs, 0)
    loc = obs.mean(0)
    scale = obs.std(0).clamp_min(1e-7)  # évite la division par 0 sur les features constantes
    return loc, scale


def main(name, load : bool):
    path = os.path.join(save_dir, name)
    if load:
        brain = torch.load(path, map_location="cpu")
        print(f"Modèle chargé depuis {path}")
        loc = brain["obs_norm"]["loc"]
        scale = brain["obs_norm"]["scale"]
    else:
        loc, scale = compute_obs_stats()

    penv = ParallelEnv(num_workers, [make_env(diff) for diff in np.linspace(0, max_diff, num_workers)])
    env = TransformedEnv(penv, Compose(
        ObservationNorm(loc=loc, scale=scale, in_keys=["observation"], standard_normal=True),
        DoubleToFloat(),
        StepCounter(int(T_tot * f)),
    ))

    action_spec = env.action_spec_unbatched

    policy_module = ProbabilisticActor(
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
    value_module = ValueOperator(module=mlp(1), in_keys=["observation"])

    collector = Collector(env, policy_module,
                          frames_per_batch=frames_per_batch,
                          total_frames=total_frames,
                          split_trajs=False)

    replay_buffer = ReplayBuffer(
        storage=LazyTensorStorage(max_size=frames_per_batch),
        sampler=SamplerWithoutReplacement(),
    )
    advantage_module = GAE(gamma=gamma, lmbda=lmbda, value_network=value_module,
                           average_gae=True)

    # passage d'initialisation : crée les poids des couches LazyLinear
    policy_module(env.reset())
    value_module(env.reset())

    loss_module = ClipPPOLoss(
        actor_network=policy_module,
        critic_network=value_module,
        clip_epsilon=clip_epsilon,
        entropy_bonus=True,
        entropy_coeff=entropy_eps,
        critic_coeff=1.0,
        loss_critic_type="smooth_l1",
    )
    optim = torch.optim.Adam(loss_module.parameters(), lr)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optim, total_frames // frames_per_batch, 0.0)

    pbar = tqdm(total=total_frames)
    
    logs = defaultdict(list)
    if load:
        policy_module.load_state_dict(brain["policy"])
        value_module.load_state_dict(brain["value"])
        optim.load_state_dict(brain["optim"])
        for g in optim.param_groups:    
            g["lr"] = lr          # ou une valeur plus faible pour un fine-tuning
        for k, v in brain["logs"].items():
            logs[k] = list(v)
    
    for i, tensordict_data in enumerate(collector):
        for _ in range(num_epochs):
            advantage_module(tensordict_data)
            replay_buffer.extend(tensordict_data.reshape(-1).cpu())
            for _ in range(frames_per_batch // sub_batch_size):
                subdata = replay_buffer.sample(sub_batch_size)
                loss_vals = loss_module(subdata)
                loss = (loss_vals["loss_objective"]
                        + loss_vals["loss_critic"]
                        + loss_vals["loss_entropy"])
                loss.backward()
                torch.nn.utils.clip_grad_norm_(loss_module.parameters(), max_grad_norm)
                optim.step()
                optim.zero_grad()

        logs["reward"].append(tensordict_data["next", "reward"].mean().item())
        logs["couple"].append(tensordict_data["next", "couple"].mean().item())
        logs["step_count"].append(tensordict_data["step_count"].max().item())
        pbar.update(tensordict_data.numel())

        parts = []
        if i % 10 == 0:
            with set_exploration_type(ExplorationType.DETERMINISTIC), torch.no_grad():
                eval_rollout = env.rollout(1000, policy_module, break_when_any_done=False)
            logs["eval reward (sum)"].append(eval_rollout["next", "reward"].sum().item())
            logs["eval step_count"].append(eval_rollout["step_count"].max().item())
            logs["eval couple"].append(eval_rollout["next", "couple"].mean().item())
            parts.append(f"eval couple={logs['eval couple'][-1]:.2f}")
            parts.append(f"eval reward (sum)={logs['eval reward (sum)'][-1]:.2f}")
            parts.append(f"eval step-count={logs['eval step_count'][-1]}")
        parts.append(f"reward moyen={logs['reward'][-1]:.4f}")
        parts.append(f"step count max={logs['step_count'][-1]}")
        parts.append(f"lr={optim.param_groups[0]['lr']:.2e}")
        pbar.set_description(", ".join(parts))

        scheduler.step()

    os.makedirs(save_dir, exist_ok=True)
    torch.save(
        {
            "policy": policy_module.state_dict(),
            "value": value_module.state_dict(),
            "optim": optim.state_dict(),
            "obs_norm": env.transform[0].state_dict(),
            "logs": dict(logs),
        },
        os.path.join(save_dir, name),
    )
    print("Modèle sauvegardé.")
    collector.shutdown()
    
name = "brain_1.pt"
save_dir = r"C:\Users\andre\Documents\école\Prepa\TIPE\Algo\V2_DL\data"
if __name__ == "__main__":
    main(name, load = False)
