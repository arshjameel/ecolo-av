"""
The simulated car's own actuator numbers, over vehicle.yaml's: a known car to measure. No ROS.

    sim.launch.py actuators:="throttle_lag=0.3,throttle_dead_time=0.1,drag=0.2"

Only the actuator model in sim_adapter (the car itself) takes them; every driver, ego and
the planners still read vehicle.yaml. So a test can set a car whose numbers it knows and
check that the hallway fits (car-test --hallway --sim, TODO 248) find them.
"""

from dataclasses import fields, replace

from aav_description.car_model import ActuatorConfig

# The car model's own numbers: not its size or limits (wheelbase, max_steering, mass, payload
# come from vehicle.yaml and the run).
KEYS = tuple(f.name for f in fields(ActuatorConfig)
             if f.name not in ('wheelbase', 'max_steering', 'mass', 'payload'))


def parse(text):
    """Return {name: value} from 'name=value' pairs, by commas or spaces ('' gives {})."""
    out = {}
    for part in (text or '').replace(',', ' ').split():
        name, eq, value = part.partition('=')
        if not eq:
            raise ValueError(f'actuators: not name=value: {part!r}')
        if name not in KEYS:
            raise ValueError(f'actuators: no actuator number {name!r} (pick from '
                             f'{", ".join(KEYS)})')
        try:
            number = float(value)
        except ValueError:
            raise ValueError(f'actuators: {name} is not a number: {value!r}') from None
        if number < 0 and name != 'steer_offset':
            raise ValueError(f'actuators: {name} must not be negative ({number})')
        out[name] = number
    return out


def overridden(config, text):
    """Return the config with the overrides in `text` (the same config for '')."""
    values = parse(text)
    return replace(config, **values) if values else config
