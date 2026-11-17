# Where the data comes from: published on /system/source, latched (reliable, transient
# local, depth 1), once at start-up, by each node that makes data up or passes it on
# (the backends, the bench's stand-ins, a replay). A late subscriber still gets it.
#
# Several nodes may announce: a listener keeps the newest list from each announcer and
# merges them. Anything not REAL means the HUD shows a SIMULATED / BENCH / REPLAY banner.

std_msgs/Header header
string announcer          # the node that says so, e.g. sim_adapter
SensorSource[] sensors    # what it publishes, and where that comes from
