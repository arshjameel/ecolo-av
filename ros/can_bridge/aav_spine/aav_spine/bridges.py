"""
Which spine_bridge the launch files start, and what it does on the bus.

No ROS imports. Both are the same node, `spine_bridge`, on the same topics, with this
package's config/spine.yaml: aav_spine_bridge (Rust, T1.8) and aav_spine (Python, kept
until the Rust one has run on the car). The launch argument is `bridge:=rust|python`, in
spine.launch.py, bench.launch.py, sim.launch.py and ravensim.launch.py.

Its `role`: `spine` drives the car over CAN; `sensors` only reads the wheel nodes, and
sends nothing. car.launch.py picks it from `actuators:=`, the car's wiring
(docs/can-plan.md): `uart`, today's, where the ESP32 is driven through the Motor Pi's
UART and the Orin's CAN bus has only sensors on it; `can`, the KEFC plan's CAN spine.
"""

PACKAGES = {'rust': 'aav_spine_bridge', 'python': 'aav_spine'}
DEFAULT = 'rust'
EXECUTABLE = 'spine_bridge'

# spine_bridge's role parameter; the first is the node's own default.
ROLES = ('spine', 'sensors')
# car.launch.py's actuators:=, and the role it gives the CAN bus's spine_bridge.
ACTUATORS = {'uart': 'sensors', 'can': 'spine'}
DEFAULT_ACTUATORS = 'uart'   # the car as it is wired today (Ben, 23 Oct)


def package(language: str) -> str:
    """Return the ROS package whose spine_bridge to start, for bridge:=<language>."""
    try:
        return PACKAGES[language.strip().lower()]
    except KeyError:
        raise ValueError(f'bridge:={language}: use one of {sorted(PACKAGES)}') from None


def check_role(role: str) -> str:
    """Return role:= if it is spine or sensors; else raise ValueError."""
    if role not in ROLES:
        raise ValueError(f'role:={role}: use one of {list(ROLES)}')
    return role


def role_for(actuators: str) -> str:
    """Return the CAN bridge's role for actuators:=uart|can: sensors, or spine."""
    try:
        return ACTUATORS[actuators.strip().lower()]
    except KeyError:
        raise ValueError(f'actuators:={actuators}: use one of {sorted(ACTUATORS)}') from None
