# Room-by-room push-optimal Sokoban solver

[![bounty](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/haoye2023/makefun-sokoban-entry/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/haoye2023/makefun-sokoban-entry/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

Entry for the make.fun bounty [Build the best Sokoban solver, 21 November 2026](https://make.fun/bounty/sokoban-solver-01-run-20261121).

`main.ts` (Node 22, no dependencies) answers a set of five generated Sokoban
levels in one run: it reads `/input/<id>/level.txt` (or `$MAKEFUN_INPUT_DIR`)
and writes `/output/<id>/solution.txt` (or `$MAKEFUN_OUTPUT_DIR`) with the
player's moves in LURD.

How it works:

- **Room decomposition.** In these levels a one-square-wide hallway crosses
  the board and every room hangs off it through a single door. Cutting a door
  cell splits its room off the floor graph; a part holding as many boxes as
  goals is one room, and since a box pushed out of a door can never come back,
  the rooms are independent and their push optima add up to the level's.
- **Push-optimal A\* per room.** Each room is solved with an A\* over pushes
  (not moves), guided by an admissible heuristic: the cheapest assignment of
  boxes to distinct goals under exact single-box push distances (Hungarian
  matching). Dead squares (a box there can reach no goal) and frozen 2x2
  blocks prune the search. Non-final rooms also require the player to keep a
  way back to the door, so the stitched walk can always leave.
- **Stitching.** Between rooms the player takes a shortest walk through the
  hallway; if a confined solve ever failed, the room is retried in the
  reference model (whole board, the other rooms' boxes standing as walls).
- **Determinism.** Every budget is an expansion counter, never the clock, so
  two executions write byte-identical answers.

On the kit's practice set the solver reaches the push optimum P* on all five
levels (13 / 16 / 29 / 47 / 60 pushes, score 1,000,000 against the house
baseline plain-astar 1.0.0's 200,000) in about a quarter of a second, and it
held P* on all 50 randomly drawn sets used to stress it (255 levels).
