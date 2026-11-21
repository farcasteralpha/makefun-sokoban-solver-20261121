# Push-optimal Sokoban solver

[![make.fun](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/competitions/sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/farcasteralpha/makefun-sokoban-solver-20261121/rank.svg)](https://make.fun/competitions/sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/farcasteralpha/makefun-sokoban-solver-20261121/reward.svg)](https://make.fun/competitions/sokoban-solver-01-run-20261121)

A make.fun entry for `sokoban-solver-01-run-20261121`: one program answers
all 5 instances of a set in a single run, in as few pushes as possible.

## How it works

Each level is a 1-wide hallway plus 1–4 rooms (6×6 slots, one door each).
A box pushed through a door into the hallway can never come back, so every
box stays in its room and a level's push-optimal count P\* is the sum of its
rooms' optima. The solver therefore:

1. decomposes the level into rooms (connected components off the hallway
   row, each holding equal boxes and goals, with its door onto the hallway);
2. solves each room push-optimally with A\* over pushes — admissible
   min-cost-assignment heuristic (BFS distances to goals), dead-square
   pruning by reverse-pull reachability, and 2×2 freeze-deadlock pruning —
   starting from the hallway cell in front of the door (so a box that starts
   on a door cell can be pushed back inside);
3. concatenates the per-room move sequences with hallway walks between
   doors (rooms whose goal sits on a door cell are solved last, since the
   player may be trapped inside such a room afterwards).

If a level does not decompose as specified, it falls back to solving the
whole level with the same search (Hungarian assignment for the heuristic).
The search budget is counted in node expansions, never wall-clock time, so
two runs write byte-identical outputs.

## Run

```sh
node main.ts   # reads /input/<id>/level.txt, writes /output/<id>/solution.txt
```

Node 22, standard library only, no network, no dependencies.

## Practice-set score

Push-optimal on all 5 practice instances (13, 16, 29, 47 and 60 pushes
against P\* 13, 16, 29, 47, 60) — 1,000 points per instance, total
**1,000,000**, in ~0.4 s for the whole set. Cross-checked against an
independent Dijkstra push-optimal solver on randomly generated rooms
(19 random levels plus door-goal edge cases, all matching).

## License

MIT — see [LICENSE](LICENSE).
