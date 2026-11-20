# Git: worktrees, conflicts, handy commands

The help that [CONTRIBUTING.md](../CONTRIBUTING.md) doesn't give.

> **The flow is in [CONTRIBUTING.md](../CONTRIBUTING.md):** fork, branch from
> `upstream/main`, pull request, one approval, then the merge queue (a merge commit,
> so every commit keeps its message).
> Keep your branch current with `git rebase upstream/main`, never a merge.

## The picture

```text
   AAV-2025-2026/ecolo-av          the team's repo: "upstream"
      main  ◄──────────────── merge queue (approved, CI green, merge commit)
        │                                ▲ pull request
        │ git fetch upstream             │
        ▼                                │
   your laptop  ── git push origin <branch> ──►  <you>/ecolo-av
   (branches, worktrees)                         your fork: "origin"
```

Already cloned the team's repo, not your fork? Turn it into this setup:

    git remote rename origin upstream
    git remote add origin git@github.com:<you>/ecolo-av.git

## Conflicts during a rebase

`git rebase upstream/main` stops when you and someone else changed the same lines.

| # | Do this |
|---|---|
| 1 | `git status` lists the files "both modified" |
| 2 | In each, keep the right lines between `<<<<<<<`, `=======` and `>>>>>>>`, and delete the markers |
| 3 | `git add <file>` for each, then `git rebase --continue` |
| 4 | `tools/test.sh` again: a clean rebase can still break a test |
| 5 | Push with `git push --force-with-lease`: your fork's branch only, never `main` |

!!! warning "In a rebase, ours and theirs swap"
  During a rebase, `--ours` is `upstream/main` and `--theirs` is **your** commit. The
  opposite of a merge.

- **`pixi.lock` in conflict:** take the team's, then let Pixi redo it:
  `git checkout --ours pixi.lock && pixi install`, then `git add pixi.lock`.
- **Lost?** `git rebase --abort` puts everything back as it was before the rebase.

## Branches

| Do | Command |
|---|---|
| See your branches, and which you're on | `git branch` |
| Make one from the team's latest `main` | `git switch -c <name> upstream/main` (names: CONTRIBUTING.md, section 3) |
| Switch | `git switch <branch>` (commit or `git stash` your changes first) |
| Delete one after its pull request says **Merged** | `git branch -d <branch>` (it refuses if the branch has commits `main` doesn't) |
| See what's on a branch and not on `main` | `git log --oneline upstream/main..<branch>` |

## Worktrees: two branches at once

A worktree is a second folder on the same repo, with another branch checked out. Use one
to test a teammate's pull request, or to keep a long job running while you work on
something else.

    git worktree add ../ecolo-av-review -b review upstream/main    # a new folder and branch
    git worktree list                                              # every worktree
    git worktree remove ../ecolo-av-review                         # done: remove the folder

| Know this | Why |
|---|---|
| Each worktree has its own `build/`, `install/` and `.pixi/` | run `pixi install` and `pixi run build` in it once; Pixi reuses its download cache, so it's quick |
| A branch can be checked out in only one worktree | git refuses a second one |
| Commits made in any worktree are in the same repo | push them from wherever you are |
| Two runs at once on one computer | give each its own `ROS_DOMAIN_ID`, 0–101 (safe on Linux; Fast DDS breaks above 232), or their ROS nodes hear each other |

Claude Code's helpers make worktrees under `.claude/worktrees/`: git ignores that folder.

## Handy commands

| Do | Command |
|---|---|
| What changed, not yet committed | `git status`, `git diff` |
| The last commits, short | `git log --oneline -10` |
| Undo changes to one file (not yet committed) | `git restore <file>` |
| Put work aside, and bring it back | `git stash`, then `git stash pop` |
| Fetch everything, change nothing | `git fetch upstream` |
| Bring your fork's `main` up to date | `git switch main && git merge --ff-only upstream/main && git push origin main` |
