# Provenance: aav_diffusion_planner

Written by the 2026-27 team (2 Nov 2026). **No code is copied** from another project.
Ideas it follows, each read from its paper and its own repository:

| Idea | From | Licence | Used how |
|---|---|---|---|
| A 1D U-Net over the path's points, the conditioning by FiLM in each residual block | Diffusion Policy, `ConditionalUnet1D` (Chi et al., RSS 2023), [real-stanford/diffusion_policy](https://github.com/real-stanford/diffusion_policy) | MIT | the shape of `training/model.py`'s `Denoiser`, written anew, smaller (3 levels, 96-192 channels) |
| Flow matching: the network learns the velocity from noise to the path; Euler steps to plan | Lipman et al., "Flow Matching for Generative Modeling" (ICLR 2023); Flow Planner (MIT) uses it for driving | paper | `training/train.py`'s loss, `model.Sampler` |
| Holding the path's ends in place while it is cleaned (inpainting) | Diffuser (Janner et al., ICML 2022), [jannerm/diffuser](https://github.com/jannerm/diffuser) | MIT | `model.pin` |
| Teaching from classic planners' paths, then ranking samples by a cost | Motion Planning Diffusion (Carvalho et al., IROS 2023), [joaoamcarvalho/mpd-public](https://github.com/joaoamcarvalho/mpd-public) | MIT | `tools/diffusion_data.py`, `guard.choose` |

Read but not used: Diffusion Planner (Zheng et al., ICLR 2025) and Diffusion-ES have no
licence in their repositories, so nothing of theirs is here.

The teachers are Nav2's own `SmacPlannerLattice` and `SmacPlannerHybrid` (Apache-2.0),
run as they are. The model's weights are ours, trained on their paths
(`pixi run diffusion train`); they are not in git.

## Changes since

None yet.
