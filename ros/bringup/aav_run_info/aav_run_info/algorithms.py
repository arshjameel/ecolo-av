"""
Which algorithms a run uses, in words for a screen: planner, follower, localizers.

No ROS. The launch file knows its choices (its launch arguments, already checked); this
turns them into rows of (what, which) for the cockpit's table, and one line for a
title. Unknown names are refused, so a screen never shows a planner that doesn't exist.
"""

from dataclasses import dataclass

PLANNERS = {
    'lattice': 'lattice (SmacPlannerLattice)',
    'hybrid': 'Hybrid-A* (SmacPlannerHybrid)',
    'smac2d': 'Smac 2D (trial: ignores the turning radius)',
    'navfn': 'NavFn (trial: ignores the turning radius)',
    'theta_star': 'Theta* (trial: ignores the turning radius)',
    'diffusion': 'diffusion model, the lattice as fallback (trial)',
}
CONTROLLERS = {
    'mppi': 'MPPI (Nav2 controller_server)',
    'rpp': 'Regulated Pure Pursuit (Nav2 controller_server)',
}
TRACKERS = ('none', 'pure_pursuit', 'stanley', 'lqr', 'mpc', 'mpc_full')
# How a trial localizer is used: not at all, scored beside the EKFs, or fused into one.
USES = ('off', 'scored', 'fused')


@dataclass(frozen=True)
class Choices:
    """One run's choices, as the launch resolved them."""

    profile: str = 'sim'
    nav: bool = True
    planner: str = 'lattice'
    controller: str = 'mppi'
    tracker: str = 'none'
    lidar: bool = False
    radar: bool = False
    voxels: bool = False
    amcl: bool = False
    ndt: str = 'off'
    kiss_icp: str = 'off'

    def __post_init__(self):
        checks = (('planner', self.planner, PLANNERS),
                  ('controller', self.controller, CONTROLLERS),
                  ('tracker', self.tracker, TRACKERS), ('ndt', self.ndt, USES),
                  ('kiss_icp', self.kiss_icp, USES))
        for name, value, known in checks:
            if value not in known:
                raise ValueError(f'{name}: {value!r} is none of {", ".join(known)}')
        if self.voxels and not self.lidar:
            raise ValueError('voxels: the cubes come from the lidar, and there is none')


def use(on, fused):
    """Return how a trial localizer is used, from its two launch switches."""
    return 'fused' if fused else 'scored' if on else 'off'


def rows(c):
    """Return the run's algorithms as (what, which) pairs, in the order a person reads them."""
    if c.nav:
        planner = PLANNERS[c.planner]
        follower = (CONTROLLERS[c.controller] if c.tracker == 'none'
                    else f'{c.tracker} (aav_track, trial)')
    else:
        planner = follower = 'off (no Nav2: nav:=false)'
    sensors = (['voxels: the lidar as 0.25 m cubes (trial)'] if c.voxels
               else ['lidar scan'] if c.lidar else [])
    sensors += ['radar'] if c.radar else []
    local = 'wheels, IMU' + (', KISS-ICP speed (trial)' if c.kiss_icp == 'fused' else '')
    globl = ('wheels, IMU, GPS, GPS heading'
             + (', NDT pose (trial)' if c.ndt == 'fused' else ''))
    trial = {'off': 'off', 'scored': 'on, scored only (drives nothing)'}
    return [
        ('run', c.profile),
        ('planner', planner),
        ('path follower', follower),
        ('obstacles', ' + '.join(sensors) if sensors else 'none'),
        ('local EKF', local),
        ('global EKF', globl),
        ('NDT', trial.get(c.ndt, 'on, into the global EKF')),
        ('KISS-ICP', trial.get(c.kiss_icp, 'on, into the local EKF')),
        ('AMCL', 'on, scored only (drives nothing)' if c.amcl else 'off'),
    ]


def summary(c):
    """Return one short line: the planner, the follower and the trials that are on."""
    if c.nav:
        parts = [c.planner, c.controller if c.tracker == 'none' else c.tracker]
    else:
        parts = ['no Nav2']
    parts += ['voxels'] if c.voxels else []
    parts += [f'{name} {how}' for name, how in (('NDT', c.ndt), ('KISS-ICP', c.kiss_icp))
              if how != 'off']
    parts += ['AMCL scored'] if c.amcl else []
    return ', '.join(parts)
