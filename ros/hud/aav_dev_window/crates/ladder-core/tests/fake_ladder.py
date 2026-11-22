"""A stand-in for tools/ladder.py, for ladder-core's tests: no yaml, no rules.

It answers the three calls the window makes, and writes each call's arguments to
calls.log beside tools/, so a test can see exactly what the window asked for.
"""
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
args = sys.argv[1:]
with open(os.path.join(ROOT, 'calls.log'), 'a') as log:
    log.write('|'.join(args) + '\n')

if args == ['--json']:
    with open(os.path.join(ROOT, 'ladder.json')) as f:
        print(f.read())
elif args[0] == 'run':
    print(f'Running {args[1]} in the simulator', flush=True)
    if args[1] == 'stack':
        time.sleep(60)
    print(f'Recorded: {args[1]} pass (sim) at fake')
elif args[0] == 'record':
    if args[1] == 'stands' and not any(a.startswith('--override=') for a in args):
        print('Shut: bench has not passed (sim). A pass here needs --override "why".',
              file=sys.stderr)
        sys.exit(2)
    print(f'Recorded: {args[1]} {args[2]} at fake')
else:
    sys.exit(f'fake_ladder: {args}?')
