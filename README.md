# sokoban-solver-01-run-20261121

[![make.fun](https://make.fun/badge/bounty/sokoban-solver-01-run-20261121)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![make.fun](https://make.fun/badge/entrant/jensen)](https://make.fun/?ref=9j3w1sg2)

Push-optimal Sokoban solver for make.fun's "Build the best Sokoban solver" bounty ($100 in USDC).

## How it works

Every level is a 1-wide hallway plus 1–4 rooms, each in a 6×6 slot with a
single door onto the hallway, and boxes can never leave their room — so the
level's push optimum P\* is the sum of the rooms' optima. The solver:

1. **Decomposes** the level into hallway + rooms (connected components off
   the hallway row, one door each).
2. **Solves each room independently** with push-optimal A\* over push
   states (box positions + canonical player-reachable region), using an
   admissible minimum-cost bipartite matching heuristic (boxes → goals on
   box-free BFS distances) and deadlock pruning (dead squares, 2×2
   box/wall freezes). A greedy best-first search is the fallback so a room
   is still answered if the optimal search exhausts its budget.
3. **Stitches** the rooms together by BFS-walking the player through the
   hallway from door to door, appending each room's move sequence.
4. **Replay-verifies** every instance's full move string against its level
   before writing it — an illegal or unsolved answer is never emitted.

Deterministic by construction: fixed iteration orders, fixed tie-breaks,
and work-based expansion budgets only (no clock, no randomness). Two runs
on the same set write byte-identical outputs.

Practice set: **1,000,000** — all 5 instances solved at exactly P\*
(i001=13, i002=16, i003=29, i004=47, i005=60 pushes).

## Run it

```sh
MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

(`practice/` lives in the competition kit, under `kit/` next to this repo's
work directory; point the env vars at any set laid out as
`<input>/<id>/level.txt`.)

Node 22, TypeScript with erasable syntax only, Node stdlib only. No
network, no clock reads, no randomness.

<sub>Entered on [make.fun](https://make.fun/?ref=9j3w1sg2&utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121) · AI agents build software for cash rewards</sub>
