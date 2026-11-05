"""
The diffusion planner's network: a small flow-matching model that draws paths. PyTorch.

    crop (1 x 128 x 128) ─► MapEncoder ─► 256 ┐
    cond (8)             ─► MLP        ─► 64  ├─► FiLM in every block
    t (0..1)             ─► sine embed ─► 64  ┘
    x (64 points x 5)    ─► a 1D U-Net over the points ─► velocity (64 x 5)

Its shape follows Diffusion Policy's ConditionalUnet1D (Chi et al., RSS 2023, MIT: a 1D
U-Net over the path, the conditioning by FiLM in each residual block); written here
anew, nothing copied (PROVENANCE.md). Flow matching (Lipman et al., 2023): x_t =
(1 - t) noise + t path, and the network learns the velocity path - noise. To plan,
start from noise and take `steps` Euler steps from t = 0 to 1.

Sampler is what goes to ONNX: the map encoded once, then the steps unrolled, with the
start and goal points held in place at each step (inpainting, as Diffuser does).
"""

import math

import torch
from torch import nn
import torch.nn.functional as F

SIZE, POINTS, CHANNELS, COND = 128, 64, 5, 8


def groups(c):
    return 8 if c % 8 == 0 else 1


class MapEncoder(nn.Module):
    """The costmap crop -> one vector: five strided convolutions, then a linear layer."""

    def __init__(self, out=256, widths=(32, 64, 96, 128, 128)):
        super().__init__()
        layers, c = [], 1
        for w in widths:
            layers += [nn.Conv2d(c, w, 3, stride=2, padding=1), nn.GroupNorm(groups(w), w),
                       nn.SiLU(), nn.Conv2d(w, w, 3, padding=1), nn.GroupNorm(groups(w), w),
                       nn.SiLU()]
            c = w
        self.conv = nn.Sequential(*layers)
        side = SIZE // 2 ** len(widths)
        self.fc = nn.Linear(c * side * side, out)

    def forward(self, crop):
        return self.fc(self.conv(crop).flatten(1))


def time_embed(t, dim=64):
    """Sine and cosine of t at dim / 2 frequencies."""
    half = dim // 2
    freq = torch.exp(-math.log(1000.0) * torch.arange(half, device=t.device) / half)
    a = t[:, None] * 1000.0 * freq[None, :]
    return torch.cat([torch.sin(a), torch.cos(a)], dim=1)


class Block(nn.Module):
    """Two 1D convolutions, scaled and shifted by the conditioning (FiLM), plus the input."""

    def __init__(self, cin, cout, cdim):
        super().__init__()
        self.a = nn.Sequential(nn.Conv1d(cin, cout, 5, padding=2),
                               nn.GroupNorm(groups(cout), cout), nn.SiLU())
        self.b = nn.Sequential(nn.Conv1d(cout, cout, 5, padding=2),
                               nn.GroupNorm(groups(cout), cout), nn.SiLU())
        self.film = nn.Linear(cdim, 2 * cout)
        self.skip = nn.Conv1d(cin, cout, 1) if cin != cout else nn.Identity()

    def forward(self, x, c):
        h = self.a(x)
        scale, shift = self.film(c).unsqueeze(-1).chunk(2, dim=1)
        h = h * (1.0 + scale) + shift
        return self.b(h) + self.skip(x)


class Denoiser(nn.Module):
    """The 1D U-Net over the path's points: 64 -> 32 -> 16 points, and back."""

    def __init__(self, cdim, widths=(96, 160, 192)):
        super().__init__()
        w0, w1, w2 = widths
        # One extra input channel: where along the path each point is (0..1).
        self.inp = nn.Conv1d(CHANNELS + 1, w0, 1)
        self.d0 = Block(w0, w0, cdim)
        self.d1 = Block(w0, w1, cdim)
        self.d2 = Block(w1, w2, cdim)
        self.mid = Block(w2, w2, cdim)
        self.u2 = Block(w2 + w2, w1, cdim)
        self.u1 = Block(w1 + w1, w0, cdim)
        self.u0 = Block(w0 + w0, w0, cdim)
        self.out = nn.Conv1d(w0, CHANNELS, 1)

    def forward(self, x, c):
        b = x.shape[0]
        where = torch.linspace(0.0, 1.0, POINTS, device=x.device).expand(b, 1, POINTS)
        h = self.inp(torch.cat([x.transpose(1, 2), where], dim=1))
        s0 = self.d0(h, c)                                   # 64
        s1 = self.d1(F.avg_pool1d(s0, 2), c)                 # 32
        s2 = self.d2(F.avg_pool1d(s1, 2), c)                 # 16
        m = self.mid(s2, c)
        h = self.u2(torch.cat([m, s2], 1), c)
        h = self.u1(torch.cat([F.interpolate(h, scale_factor=2.0), s1], 1), c)
        h = self.u0(torch.cat([F.interpolate(h, scale_factor=2.0), s0], 1), c)
        return self.out(h).transpose(1, 2)


class FlowPlanner(nn.Module):
    """The velocity field: v(x_t, t | crop, cond)."""

    def __init__(self, map_dim=256, cond_dim=64, t_dim=64):
        super().__init__()
        self.encoder = MapEncoder(map_dim)
        self.cond_mlp = nn.Sequential(nn.Linear(COND, cond_dim), nn.SiLU(),
                                      nn.Linear(cond_dim, cond_dim))
        cdim = 256
        self.mix = nn.Sequential(nn.Linear(map_dim + cond_dim + t_dim, cdim), nn.SiLU(),
                                 nn.Linear(cdim, cdim))
        self.t_dim = t_dim
        self.denoiser = Denoiser(cdim)

    def context(self, crop, cond):
        """Return what doesn't change between steps: the map's vector and the cond's."""
        return torch.cat([self.encoder(crop), self.cond_mlp(cond)], dim=1)

    def velocity(self, x, t, ctx):
        c = self.mix(torch.cat([ctx, time_embed(t, self.t_dim)], dim=1))
        return self.denoiser(x, c)

    def forward(self, x, t, crop, cond):
        return self.velocity(x, t, self.context(crop, cond))


def ends(cond):
    """Return the start and goal points (B, 4): u, v, cos, sin; the start faces 0."""
    one = torch.ones_like(cond[:, :1])
    zero = torch.zeros_like(cond[:, :1])
    start = torch.cat([cond[:, 0:2], one, zero], dim=1)
    goal = torch.cat([cond[:, 2:4], cond[:, 4:6]], dim=1)
    return start, goal


def pin(x, x0, t, start, goal):
    """Hold the first and last points where they belong at time t (inpainting)."""
    x = x.clone()
    t = t.view(-1, 1)
    x[:, 0, :4] = (1.0 - t) * x0[:, 0, :4] + t * start
    x[:, -1, :4] = (1.0 - t) * x0[:, -1, :4] + t * goal
    return x


class Sampler(nn.Module):
    """Noise -> paths: `steps` Euler steps, the ends pinned. One map, K samples."""

    def __init__(self, model, steps=10):
        super().__init__()
        self.model, self.steps = model, steps

    def forward(self, crop, cond, noise):
        k = noise.shape[0]
        crop, cond = crop.expand(k, -1, -1, -1), cond.expand(k, -1)
        ctx = self.model.context(crop, cond)
        start, goal = ends(cond)
        x = pin(noise, noise, torch.zeros(k, device=noise.device), start, goal)
        dt = 1.0 / self.steps
        for i in range(self.steps):
            t = torch.full((k,), i * dt, device=noise.device)
            x = x + dt * self.model.velocity(x, t, ctx)
            x = pin(x, noise, t + dt, start, goal)
        return x
