# Contributing to ecolo-av

How a change gets from your laptop into `main`. Everyone follows the same steps,
maintainers included.

New here? Set up your laptop first: [Getting started](docs/getting-started.md) (30 minutes).

## The flow

| Step | What you do | Where |
|---|---|---|
| 1 | Fork the repo and clone your fork | once |
| 2 | Open or pick an issue | GitHub |
| 3 | Make a branch from the team's `main` | your laptop |
| 4 | Commit in small steps | your laptop |
| 5 | Push the branch to your fork | your fork |
| 6 | Open a pull request to `AAV-2025-2026/ecolo-av` | GitHub |
| 7 | Get one approval and green CI | GitHub |
| 8 | Merge it | GitHub |
| 9 | Delete the branch, sync your fork | both |

Nobody pushes to the team's `main`. An approved, green pull request is the only way in.

## 1. Fork and clone (once)

A fork is your own copy of the repo on GitHub. You push there; the team's repo only
changes through pull requests.

1. Put an SSH key on your GitHub account ([GitHub's guide](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)).
2. On https://github.com/AAV-2025-2026/ecolo-av, click **Fork**, then **Create fork**.
3. Clone your fork and point `upstream` at the team's repo:

```bash
git clone git@github.com:<your-username>/ecolo-av.git ~/ecolo-av
cd ~/ecolo-av
git remote add upstream https://github.com/AAV-2025-2026/ecolo-av.git
git remote -v
```

| Remote | Is | You |
|---|---|---|
| `origin` | your fork | push here |
| `upstream` | the team's repo | fetch from here, never push |

4. Turn CI on in your fork: its **Actions** tab, then **I understand my workflows, go
   ahead and enable them**. Each push to a branch there then runs the Linux and ARM
   tests, so you know it's green before you open the pull request.

## 2. Issues

Every piece of work starts as an issue, so the team can see who is doing what.

**Open one:** Issues, then **New issue**.

| Part | Write |
|---|---|
| Title | the problem or the goal, in a few words: "Spine ignores a stale heartbeat" |
| Body | what is wrong or wanted, why it matters, and how we know it's done |
| Task | the proposal's task number, if there is one: `T3.6` |
| Assignee | yourself, when you start on it |

**Close one:**

| How | When |
|---|---|
| Write `Closes #12` in the pull request | the work is done: GitHub closes the issue when the PR merges |
| **Close issue**, then **Close as not planned**, with a comment saying why | the work won't happen, or it's a duplicate (link the other issue) |

Don't close someone else's issue without asking them.

## 3. Branches

Always start from the team's latest `main`, never from your fork's old one:

```bash
git fetch upstream
git switch -c t3.6-spine-heartbeat upstream/main
```

| Rule | Example |
|---|---|
| Name: task number, then a few words, lowercase, hyphens | `t3.6-spine-heartbeat`, `t4.4-rs485-reader` |
| No task number: say the kind of change | `fix-gps-heading-sign`, `docs-car-install` |
| One branch, one change, one pull request | a fix and an unrelated refactor are two branches |
| Never commit to `main`, even in your fork | `main` only follows `upstream/main` |

Branch still open while `main` moves on? Bring it up to date:

```bash
git fetch upstream
git rebase upstream/main
```

On a conflict, git stops and names the files. Fix them, `git add` them, then
`git rebase --continue`. Stuck: `git rebase --abort` puts everything back. More help:
[Git](docs/git.md), "Conflicts during a rebase".

A rebase rewrites your branch, so a plain `git push` of an already-pushed branch is
refused. Push it with `git push --force-with-lease`: to your fork's branch only, never
to `main`.

## 4. Commits

Each commit is one logical change that still builds and passes the tests.

### The message

```
Stop the spine when the Orin's heartbeat is stale

The spine kept the last command for up to 2 s after the Orin went
quiet. It now brakes after 100 ms without a heartbeat (proposal 3.4),
and a test covers the timeout.

Closes #12
```

| Part | Rule |
|---|---|
| Subject | what the commit does, as a command: "Add", "Fix", "Remove", not "Added" or "Fixes" |
| Subject length | 72 characters at most; no full stop at the end |
| Blank line | between the subject and the body |
| Body | **why**: the problem, the reason for this fix, anything a reviewer can't see in the diff |
| Body width | wrap at 72 characters |
| Footer | `Closes #12` when the commit finishes an issue |

| Good subject | Bad subject |
|---|---|
| `Fix the GPS heading sign for west-facing starts` | `fix` |
| `Add the RS-485 reader for the Junctek monitor` | `stuff`, `WIP`, `asdf` |
| `Move vehicle limits into vehicle.yaml` | `Updated files` |

### What goes in a commit

| Do | Don't |
|---|---|
| run `tools/test.sh` before every commit | commit something that fails the tests |
| keep each commit to one change | mix a fix, a rename and a format pass |
| update `PROVENANCE.md` when you change inherited code | copy old code without saying where from |
| change `docs/interfaces.yaml` first when a topic changes | change a topic without the contract |
| give a new or changed feature its row in `docs/features.md`, in the same commit | leave the features table behind |
| give a new `pixi run` task its row in README.md's "Every command" | add a task nobody can find |
| changed `docs/*.md`? run `pixi run docs` and commit the HTML | let the web pages drift from the markdown |
| | commit `build/`, `install/`, `log/`, `target/` or `.pixi/` |
| | commit keys, passwords, tokens or `.env` files: the repo is public |
| | commit large data files: add a download step instead |
| | add attribution lines (`Co-Authored-By`, "Generated with"): the README's Credits say who did what |

Made a mess before pushing? `git commit --amend` fixes the last commit.

## 5. Push to your fork

First run the full `pixi run test`: it adds the build, the style check and `colcon test`,
which `tools/test.sh` skips. Then:

```bash
git push -u origin t3.6-spine-heartbeat
```

After the first push, `git push` is enough (after a rebase: `git push --force-with-lease`,
section 3). Once someone is reviewing, add new commits
instead of rewriting old ones, so they can see what changed.

## 6. Pull requests

**Open one:** after a push, GitHub shows **Compare & pull request** on your fork. Check
that it says `AAV-2025-2026/ecolo-av` `main` ← `<you>/ecolo-av` `<your-branch>`. Or:

```bash
gh pr create --repo AAV-2025-2026/ecolo-av --base main
```

Not finished but want feedback? Open it as a **draft**, then **Ready for review** later.

The description box fills in with our template. Replace every comment in it.

### Writing a good pull request

| Section | Write | Leave out |
|---|---|---|
| Title | what `main` gains, as a command, 72 characters at most. It becomes the commit subject on `main` | "Update", "Changes", a branch name |
| What and why | the problem, and why this is the fix, in 2 to 4 sentences | a list of every file |
| Issue / task | `Closes #12`, `Task: T3.6` | |
| How I tested | the exact commands you ran and what happened; say what you did **not** test | "tested, works" |
| Risk | what could break, and on which machine: laptop, sim or car | |
| Screenshots | for the HUD, RViz or anything visual | |
| Checklist | tick only what you actually did | ticking everything by habit |

Keep a pull request small enough to review in 15 minutes: one change, a few hundred
lines at most. Split anything bigger.

## 7. Review

A pull request merges only when all of these are true:

| Rule | Why |
|---|---|
| 1 approval from someone other than the author | a second pair of eyes on every change |
| CI green: `linux`, `arm` and `car-kit` | it builds and passes `pixi run test` on a laptop's and the car's computers |
| every review comment resolved | questions get answered, not skipped |
| approval given after the last push | a new push needs a new approval |

Your first pull request from a fork: a maintainer may need to click **Approve and run
workflows** before CI starts.

**Reviewing someone else's:** read the description first, then the diff. Check that the
tests cover the change, that names and topics follow [Conventions](docs/conventions.md),
and that nothing secret or generated is committed. Ask questions as comments; approve
only what you understand.

## 8. Merge it

When the pull request is approved and green, click **Merge pull request**. It merges at
once, with a merge commit: every commit of yours keeps its own message on `main`.

| | |
|---|---|
| Before you click | if `main` has moved a lot since your CI ran, rebase on `upstream/main` and push: CI runs again on the newer `main` |
| After the merge | CI runs the full tests on `main` itself. Watch it: if it goes red, fix it or revert, first thing |

There was a merge queue until 21 Nov. It ran the whole CI a second time, about an hour,
on every pull request (Ben, 21 Nov: "sitting in a queue"). So now two changes that each
pass alone, but break together, can reach `main`: the run on `main` is what catches them.

## 9. After the merge

Delete the branch and bring your fork's `main` up to date:

```bash
git switch main
git fetch upstream
git merge --ff-only upstream/main
git push origin main
git branch -d t3.6-spine-heartbeat
git push origin --delete t3.6-spine-heartbeat
```

Your commits are now part of `main`, so `-d` deletes the branch. If it refuses, the
branch has commits `main` doesn't: check the pull request says **Merged** first.

## Rules on `main`

| Rule | |
|---|---|
| No direct pushes, admins included | changes arrive through a pull request: one approval, and `linux`, `arm` and `car-kit` green |
| No force pushes, no deletion | `main`'s history is never rewritten |
| Tags can't be moved or deleted | `pre-aav-merge` stays a reliable backup |
