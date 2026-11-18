# Getting started

This guide takes a new laptop from nothing to driving the TurtleSim turtle.
Target: under 30 minutes (T1.1). If it takes longer, tell the integration team where you got stuck.

Works on Linux (x86-64 and ARM) and macOS (Apple Silicon or Intel). No admin rights needed.
You need about 10 GB of free disk space. The longer version, with Ravensim, our simulator:
[Your first day](first-day.md).

## 1. Install Pixi

Pixi installs ROS 2 Jazzy and every other tool into this project's folder,
pinned to the same versions for everyone.

    curl -fsSL https://pixi.sh/install.sh | sh

Close and reopen your terminal, then check:

    pixi --version

## 2. Get the code

The repository is public: https://github.com/AAV-2025-2026/ecolo-av. You work in your
own fork of it. Put an SSH key on your GitHub account, click **Fork** on that page, then:

    git clone git@github.com:<your-username>/ecolo-av.git ~/ecolo-av
    cd ~/ecolo-av
    git remote add upstream https://github.com/AAV-2025-2026/ecolo-av.git

Only want to try the demo? `git clone https://github.com/AAV-2025-2026/ecolo-av.git ~/ecolo-av`
works too. How changes get in: [Contributing](../CONTRIBUTING.md).

## 3. Install the environment

    pixi install

The first run downloads ROS 2 Jazzy and the toolchains (about 7 GB). Later runs
are instant. Then build our packages once:

    pixi run build

## 4. Run the demo

Terminal 1:

    pixi run turtle

A window with a turtle opens. It stands in for the car.

Terminal 2 (in the same `ecolo-av` folder):

    pixi run keyboard --local

Keep terminal 2 focused and press `i` (forward), `u` / `o` (turn), `,` (reverse) to drive
the turtle. `k` stops it. Let go of a key and it stops by itself. `Ctrl+C` quits.

## 5. Run the tests

    pixi run test

This runs the same checks CI runs on every pull request. All of them should pass.

## Troubleshooting

| Problem | Fix |
|---|---|
| The turtle does not move | Terminal 2 (the keyboard) needs keyboard focus, not the TurtleSim window |
| `pixi: command not found` | Reopen the terminal after installing, or add `~/.pixi/bin` to your `PATH` |
| Another network's car or turtles show up | `pixi run turtle` and `keyboard --local` stay on this computer. For anything else on shared WiFi: `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` |
| Something else | `pixi run doctor` checks the network and the setup ([On a Mac](mac.md)) |
