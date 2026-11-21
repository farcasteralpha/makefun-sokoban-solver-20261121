// Push-optimal Sokoban solver for make.fun sokoban-solver-01-run-20261121.
//
// Strategy (from the prose spec, written from scratch):
//   Each level is a 1-wide hallway plus 1-4 rooms (6x6 slots, one door each).
//   A box pushed through a door into the hallway can never come back, so every
//   box stays in its room and the level optimum P* is the sum of the rooms'
//   optima. We therefore decompose the level into rooms, solve each room
//   push-optimally with A* over pushes (admissible assignment heuristic +
//   dead-square / freeze-deadlock pruning), and concatenate the per-room move
//   sequences with hallway walks between doors.
//
// Determinism: the search budget is counted in node expansions, never wall
// clock; all iteration orders are fixed. No clock, environment, or files
// outside /bot, /input, /output, /tmp are read.
//
// SPDX-License-Identifier: MIT

import fs from "node:fs";

const DIRS = [
  [-1, 0, "u"],
  [1, 0, "d"],
  [0, -1, "l"],
  [0, 1, "r"],
];

// ---------------------------------------------------------------------------
// Small binary heap with deterministic tie-breaking (insertion sequence).
// ---------------------------------------------------------------------------
function makeHeap() {
  const a = [];
  function less(x, y) {
    if (x.f !== y.f) return x.f < y.f;
    if (x.h !== y.h) return x.h < y.h;
    return x.seq < y.seq;
  }
  return {
    get size() {
      return a.length;
    },
    push(node) {
      a.push(node);
      let i = a.length - 1;
      while (i > 0) {
        const p = (i - 1) >> 1;
        if (!less(a[i], a[p])) break;
        const t = a[i];
        a[i] = a[p];
        a[p] = t;
        i = p;
      }
    },
    pop() {
      const top = a[0];
      const last = a.pop();
      if (a.length > 0) {
        a[0] = last;
        let i = 0;
        for (;;) {
          const l = i * 2 + 1;
          const r = l + 1;
          let m = i;
          if (l < a.length && less(a[l], a[m])) m = l;
          if (r < a.length && less(a[r], a[m])) m = r;
          if (m === i) break;
          const t = a[i];
          a[i] = a[m];
          a[m] = t;
          i = m;
        }
      }
      return top;
    },
  };
}

// ---------------------------------------------------------------------------
// Room solver: A* (or greedy best-first) over pushes.
//   W: board width. region: Set of flat cell indices the player may stand on
//      (room component cells + the hallway cell(s) in front of the door).
//   entry: flat index where the player starts (a hallway cell by the door).
//   boxes0: sorted flat indices of boxes; goals: flat indices of goals.
//   hallwayRow: row index of the hallway; boxes may never be pushed onto it.
//   needExit: if true, a solved state also requires the player to be able to
//      reach a door cell (so it can leave the room afterwards).
//   doors: flat indices of door cells (room cells adjacent to the hallway).
//   budget: { astar, greedy } node-expansion caps.
// Returns { status: "ok", moves } or { status: "cap"|"fail" }.
// ---------------------------------------------------------------------------
function solveRoom(W, region, entry, boxes0, goals, hallwayRow, needExit, doors, budget, work) {
  const n = goals.length;
  const goalSet = new Set(goals);

  // --- live cells: reverse-pull reachability from goals over region cells.
  // A pull is the reverse of a push: a box at b that was pushed there along
  // d (player ending at b-d) can be pulled back to b-d while the player
  // steps to b-2d. So from a live cell b, the cell b-d is live when b-2d
  // is also floor (a place for the player to step to). Cells outside this
  // set can never hold a box that reaches a goal, so they are dead
  // (a sound prune: the player is assumed able to stand anywhere).
  const live = new Set(goals);
  {
    const q = [...goals];
    let qi = 0;
    while (qi < q.length) {
      const b = q[qi++];
      const br = Math.floor(b / W);
      const bc = b % W;
      for (let d = 0; d < 4; d++) {
        const dr = DIRS[d][0];
        const dc = DIRS[d][1];
        const from = (br - dr) * W + (bc - dc);
        const step = (br - 2 * dr) * W + (bc - 2 * dc);
        if (region.has(from) && region.has(step) && !live.has(from)) {
          live.add(from);
          q.push(from);
        }
      }
    }
  }

  // --- BFS distances from each goal over region cells (ignoring boxes).
  const distToGoal = [];
  for (let gi = 0; gi < n; gi++) {
    const dist = new Map();
    dist.set(goals[gi], 0);
    const q = [goals[gi]];
    let qi = 0;
    while (qi < q.length) {
      const cell = q[qi++];
      const r = Math.floor(cell / W);
      const c = cell % W;
      const nd = dist.get(cell) + 1;
      for (let d = 0; d < 4; d++) {
        const nr = r + DIRS[d][0];
        const nc = c + DIRS[d][1];
        const nb = nr * W + nc;
        if (region.has(nb) && !dist.has(nb)) {
          dist.set(nb, nd);
          q.push(nb);
        }
      }
    }
    distToGoal.push(dist);
  }

  // --- admissible heuristic: min-cost assignment boxes -> goals.
  // Small rooms (n <= 5): exact by permutations. Larger (only the
  // whole-level fallback can have many boxes): Hungarian O(n^3). A
  // factorial permutation scan here would hang on 10+ boxes.
  const perms = (function () {
    const out = [];
    if (n > 5) return out;
    const used = new Array(n).fill(false);
    const cur = new Array(n);
    (function gen(pos) {
      if (pos === n) {
        out.push(cur.slice());
        return;
      }
      for (let i = 0; i < n; i++) {
        if (used[i]) continue;
        used[i] = true;
        cur[pos] = i;
        gen(pos + 1);
        used[i] = false;
      }
    })(0);
    return out;
  })();
  function hungarian(cost) {
    const n = cost.length;
    const BIG = 1000000;
    const m = cost.map((row) => row.map((x) => (isFinite(x) ? x : BIG)));
    const u = new Array(n + 1).fill(0);
    const v = new Array(n + 1).fill(0);
    const p = new Array(n + 1).fill(0);
    const way = new Array(n + 1).fill(0);
    for (let i = 1; i <= n; i++) {
      p[0] = i;
      let j0 = 0;
      const minv = new Array(n + 1).fill(Infinity);
      const used = new Array(n + 1).fill(false);
      do {
        used[j0] = true;
        const i0 = p[j0];
        let delta = Infinity;
        let j1 = 0;
        for (let j = 1; j <= n; j++) {
          if (used[j]) continue;
          const cur = m[i0 - 1][j - 1] - u[i0] - v[j];
          if (cur < minv[j]) {
            minv[j] = cur;
            way[j] = j0;
          }
          if (minv[j] < delta) {
            delta = minv[j];
            j1 = j;
          }
        }
        for (let j = 0; j <= n; j++) {
          if (used[j]) {
            u[p[j]] += delta;
            v[j] -= delta;
          } else {
            minv[j] -= delta;
          }
        }
        j0 = j1;
      } while (p[j0] !== 0);
      do {
        const j1 = way[j0];
        p[j0] = p[j1];
        j0 = j1;
      } while (j0 !== 0);
    }
    let total = 0;
    for (let j = 1; j <= n; j++) {
      const c = m[p[j] - 1][j - 1];
      if (c >= BIG) return Infinity;
      total += c;
    }
    return total;
  }
  function heuristic(boxes) {
    if (n <= 5) {
      let best = Infinity;
      for (const p of perms) {
        let s = 0;
        let ok = true;
        for (let i = 0; i < n; i++) {
          const d = distToGoal[p[i]].get(boxes[i]);
          if (d === undefined) {
            ok = false;
            break;
          }
          s += d;
        }
        if (ok && s < best) best = s;
      }
      return best;
    }
    const cost = [];
    for (let i = 0; i < n; i++) {
      const row = [];
      for (let j = 0; j < n; j++) {
        const d = distToGoal[j].get(boxes[i]);
        row.push(d === undefined ? Infinity : d);
      }
      cost.push(row);
    }
    return hungarian(cost);
  }

  // --- player walk BFS over region cells avoiding boxes; returns move string.
  function walkPath(from, to, boxSet) {
    if (from === to) return "";
    const prev = new Map();
    prev.set(from, -1);
    const q = [from];
    let qi = 0;
    while (qi < q.length) {
      const cell = q[qi++];
      const r = Math.floor(cell / W);
      const c = cell % W;
      for (let d = 0; d < 4; d++) {
        const nr = r + DIRS[d][0];
        const nc = c + DIRS[d][1];
        const nb = nr * W + nc;
        if (!region.has(nb) || boxSet.has(nb) || prev.has(nb)) continue;
        prev.set(nb, cell * 4 + d);
        if (nb === to) {
          let s = "";
          let cur = nb;
          while (cur !== from) {
            const code = prev.get(cur);
            s = DIRS[code % 4][2] + s;
            cur = Math.floor(code / 4);
          }
          return s;
        }
        q.push(nb);
      }
    }
    return null;
  }

  // --- canonical player cell: smallest cell index in the player's reachable
  // region (boxes fixed). Only the reachable region matters, not the exact
  // cell, which collapses the state space.
  function canonicalPlayer(pos, boxSet) {
    const seen = new Set([pos]);
    const q = [pos];
    let qi = 0;
    let min = pos;
    while (qi < q.length) {
      const cell = q[qi++];
      if (cell < min) min = cell;
      const r = Math.floor(cell / W);
      const c = cell % W;
      for (let d = 0; d < 4; d++) {
        const nr = r + DIRS[d][0];
        const nc = c + DIRS[d][1];
        const nb = nr * W + nc;
        if (!region.has(nb) || boxSet.has(nb) || seen.has(nb)) continue;
        seen.add(nb);
        q.push(nb);
      }
    }
    return min;
  }

  function playerCanReach(pos, target, boxSet) {
    if (pos === target) return true;
    const seen = new Set([pos]);
    const q = [pos];
    let qi = 0;
    while (qi < q.length) {
      const cell = q[qi++];
      const r = Math.floor(cell / W);
      const c = cell % W;
      for (let d = 0; d < 4; d++) {
        const nr = r + DIRS[d][0];
        const nc = c + DIRS[d][1];
        const nb = nr * W + nc;
        if (!region.has(nb) || boxSet.has(nb) || seen.has(nb)) continue;
        if (nb === target) return true;
        seen.add(nb);
        q.push(nb);
      }
    }
    return false;
  }

  // --- deadlock test for a box configuration.
  function deadlocked(boxes) {
    for (const b of boxes) {
      if (goalSet.has(b)) continue;
      if (!live.has(b)) return true;
      // 2x2 freeze: any 2x2 block of walls/boxes holding a box off-goal.
      const r = Math.floor(b / W);
      const c = b % W;
      for (let dr = -1; dr <= 0; dr++) {
        for (let dc = -1; dc <= 0; dc++) {
          let allBlocked = true;
          let hasBox = false;
          for (let i = 0; i < 2; i++) {
            for (let j = 0; j < 2; j++) {
              const cell = (r + dr + i) * W + (c + dc + j);
              if (region.has(cell)) {
                if (boxes.includes(cell)) hasBox = true;
                else allBlocked = false;
              }
              // cells outside the region are walls: blocked.
            }
          }
          if (allBlocked && hasBox) return true;
        }
      }
    }
    return false;
  }

  function solved(boxes) {
    for (const b of boxes) if (!goalSet.has(b)) return false;
    return true;
  }

  // --- generic search: weight 1 = A* (push-optimal), weight 1000 ~= greedy.
  // Returns { status, nodes, idx, expansions }.
  function search(weight, cap) {
    const startBoxes = boxes0.slice().sort((a, b) => a - b);
    const startSet = new Set(startBoxes);
    const h0 = heuristic(startBoxes);
    if (h0 === Infinity || deadlocked(startBoxes)) return { status: "fail", expansions: 0 };
    const startCanon = canonicalPlayer(entry, startSet);
    const heap = makeHeap();
    const best = new Map();
    const nodes = [];
    let seq = 0;
    const startKey = startBoxes.join(",") + "|" + startCanon;
    best.set(startKey, 0);
    nodes.push({ parent: -1, seg: "", boxes: startBoxes, player: entry });
    heap.push({ f: weight * h0, h: h0, seq: seq++, idx: 0, g: 0 });
    let expansions = 0;
    while (heap.size > 0) {
      const cur = heap.pop();
      const node = nodes[cur.idx];
      const canon = canonicalPlayer(node.player, new Set(node.boxes));
      const key = node.boxes.join(",") + "|" + canon;
      const recorded = best.get(key);
      if (recorded !== undefined && recorded < cur.g) continue;
      if (solved(node.boxes)) {
        if (!needExit) return { status: "ok", nodes, idx: cur.idx, expansions };
        const boxSet = new Set(node.boxes);
        for (const door of doors) {
          if (playerCanReach(node.player, door, boxSet)) {
            return { status: "ok", nodes, idx: cur.idx, expansions };
          }
        }
        // solved but the player is trapped inside: keep searching for an
        // exitable solution instead of accepting this one.
      }
      if (expansions >= cap || work.used >= work.limit) return { status: "cap", nodes, idx: -1, expansions };
      expansions++;
      work.used++;
      const boxSet = new Set(node.boxes);
      for (let i = 0; i < node.boxes.length; i++) {
        const b = node.boxes[i];
        const br = Math.floor(b / W);
        const bc = b % W;
        for (let d = 0; d < 4; d++) {
          const dr = DIRS[d][0];
          const dc = DIRS[d][1];
          const dest = (br + dr) * W + (bc + dc);
          // never push a box onto the hallway row: it can never come back.
          if (Math.floor(dest / W) === hallwayRow) continue;
          if (!region.has(dest) || boxSet.has(dest)) continue;
          const behind = (br - dr) * W + (bc - dc);
          if (!region.has(behind) || boxSet.has(behind)) continue;
          const walk = walkPath(node.player, behind, boxSet);
          if (walk === null) continue;
          const nb = node.boxes.slice();
          nb[i] = dest;
          nb.sort((x, y) => x - y);
          if (deadlocked(nb)) continue;
          const h = heuristic(nb);
          if (h === Infinity) continue;
          const nSet = new Set(nb);
          const nCanon = canonicalPlayer(b, nSet);
          const nKey = nb.join(",") + "|" + nCanon;
          const ng = cur.g + 1;
          const prevBest = best.get(nKey);
          if (prevBest !== undefined && prevBest <= ng) continue;
          best.set(nKey, ng);
          const nIdx = nodes.length;
          nodes.push({ parent: cur.idx, seg: walk + DIRS[d][2], boxes: nb, player: b });
          heap.push({ f: ng + weight * h, h, seq: seq++, idx: nIdx, g: ng });
        }
      }
    }
    return { status: "fail", nodes, idx: -1, expansions };
  }

  function reconstruct(res) {
    const parts = [];
    let idx = res.idx;
    while (idx > 0) {
      parts.push(res.nodes[idx].seg);
      idx = res.nodes[idx].parent;
    }
    parts.reverse();
    return parts.join("");
  }

  // Primary: A* (push-optimal). Fallback: greedy best-first.
  const a = search(1, budget.astar);
  if (a.status === "ok") return { status: "ok", moves: reconstruct(a), expansions: a.expansions };
  const g = search(1000, budget.greedy);
  if (g.status === "ok") return { status: "ok", moves: reconstruct(g), expansions: g.expansions };
  return { status: g.status, expansions: (a.expansions || 0) + (g.expansions || 0) };
}

// ---------------------------------------------------------------------------
// Whole-level walk BFS (any walkable cell, avoiding boxes). Returns moves.
// ---------------------------------------------------------------------------
function levelWalk(W, H, walkable, from, to, boxSet) {
  if (from === to) return "";
  const prev = new Map();
  prev.set(from, -1);
  const q = [from];
  let qi = 0;
  while (qi < q.length) {
    const cell = q[qi++];
    const r = Math.floor(cell / W);
    const c = cell % W;
    for (let d = 0; d < 4; d++) {
      const nr = r + DIRS[d][0];
      const nc = c + DIRS[d][1];
      if (nr < 0 || nr >= H || nc < 0 || nc >= W) continue;
      const nb = nr * W + nc;
      if (!walkable[nb] || boxSet.has(nb) || prev.has(nb)) continue;
      prev.set(nb, cell * 4 + d);
      if (nb === to) {
        let s = "";
        let cur = nb;
        while (cur !== from) {
          const code = prev.get(cur);
          s = DIRS[code % 4][2] + s;
          cur = Math.floor(code / 4);
        }
        return s;
      }
      q.push(nb);
    }
  }
  return null;
}

// ---------------------------------------------------------------------------
// Solve one level. Returns the move string ("" if already solved / failed).
// ---------------------------------------------------------------------------
function solveLevel(text) {
  const rawLines = text.replace(/\r/g, "").split("\n");
  while (rawLines.length > 0 && rawLines[rawLines.length - 1] === "") rawLines.pop();
  const H = rawLines.length;
  const W = Math.max.apply(null, rawLines.map((l) => l.length));
  const grid = rawLines.map((l) => (l + "#".repeat(W - l.length)).split(""));
  const walkable = new Array(W * H).fill(false);
  let player = -1;
  const boxes = [];
  const goals = [];
  for (let r = 0; r < H; r++) {
    for (let c = 0; c < W; c++) {
      const ch = grid[r][c];
      if (ch !== "#") walkable[r * W + c] = true;
      if (ch === "@" || ch === "+") player = r * W + c;
      if (ch === "$" || ch === "*") boxes.push(r * W + c);
      if (ch === "." || ch === "*") goals.push(r * W + c);
      if (ch === "+") goals.push(r * W + c);
    }
  }
  if (player < 0 || boxes.length === 0) return "";
  if (boxes.length === goals.length && boxes.every((b) => goals.includes(b))) return "";

  const hallwayRow = Math.floor(player / W);
  const D4 = [
    [-1, 0],
    [1, 0],
    [0, -1],
    [0, 1],
  ];

  // Connected components of walkable cells excluding the hallway row.
  const seen = new Array(W * H).fill(false);
  const comps = [];
  for (let r = 0; r < H; r++) {
    for (let c = 0; c < W; c++) {
      const cell = r * W + c;
      if (r === hallwayRow || !walkable[cell] || seen[cell]) continue;
      const cells = [];
      const q = [cell];
      seen[cell] = true;
      let qi = 0;
      while (qi < q.length) {
        const cur = q[qi++];
        cells.push(cur);
        const cr = Math.floor(cur / W);
        const cc = cur % W;
        for (const [dr, dc] of D4) {
          const nr = cr + dr;
          const nc = cc + dc;
          if (nr < 0 || nr >= H || nc < 0 || nc >= W) continue;
          const nb = nr * W + nc;
          if (nr === hallwayRow || !walkable[nb] || seen[nb]) continue;
          seen[nb] = true;
          q.push(nb);
        }
      }
      comps.push(cells);
    }
  }

  // Rooms = components holding boxes. Each must have boxes == goals and
  // exactly the doors onto the hallway.
  const rooms = [];
  const skippedBoxes = [];
  let decompositionOk = true;
  for (const cells of comps) {
    const set = new Set(cells);
    const rb = boxes.filter((b) => set.has(b));
    if (rb.length === 0) continue;
    const rg = goals.filter((g) => set.has(g));
    const doors = cells.filter((cell) => {
      const r = Math.floor(cell / W);
      const c = cell % W;
      return Math.abs(r - hallwayRow) === 1 && walkable[hallwayRow * W + c];
    });
    // A component whose boxes already sit on their goals needs no solving
    // and no door (its door may sit in a separate tiny component); skip it
    // instead of failing the whole decomposition over it.
    if (doors.length === 0 && rb.length === rg.length && rb.every((b) => rg.includes(b))) {
      for (const b of rb) skippedBoxes.push(b);
      continue;
    }
    if (rb.length !== rg.length || doors.length === 0) {
      decompositionOk = false;
      break;
    }
    const entryCells = [];
    for (const d of doors) {
      const c = d % W;
      const e = hallwayRow * W + c;
      if (!entryCells.includes(e)) entryCells.push(e);
    }
    rooms.push({ cells: set, boxes: rb, goals: rg, doors, entry: entryCells[0] });
  }
  if (rooms.length === 0) decompositionOk = false;
  if (decompositionOk) {
    const inRoom = new Set();
    for (const room of rooms) for (const b of room.boxes) inRoom.add(b);
    for (const b of skippedBoxes) inRoom.add(b);
    if (inRoom.size !== boxes.length) decompositionOk = false;
  }

  // Work budget (node expansions, never wall clock): bounds the whole run
  // far below the 60s wall limit even if several rooms hit their caps.
  const budget = { astar: 40000, greedy: 40000 };
  const work = { used: 0, limit: 150000 };
  const curBoxes = boxes.slice();

  if (decompositionOk) {
    // Rooms with a goal on a door cell go last: the player may be trapped
    // inside such a room, so it must be the final one.
    rooms.sort((a, b) => {
      const ag = a.doors.some((d) => a.goals.includes(d)) ? 1 : 0;
      const bg = b.doors.some((d) => b.goals.includes(d)) ? 1 : 0;
      if (ag !== bg) return ag - bg;
      if (a.doors[0] !== b.doors[0]) return a.doors[0] - b.doors[0];
      return 0;
    });
    let moves = "";
    let curPlayer = player;
    for (let ri = 0; ri < rooms.length; ri++) {
      const room = rooms[ri];
      const last = ri === rooms.length - 1;
      const region = new Set(room.cells);
      // The player may stand on the hallway cell in front of each door
      // (needed to push a box that starts on a door cell back into the room).
      for (const d of room.doors) {
        const c = d % W;
        region.add(hallwayRow * W + c);
      }
      // Walk from the current position to this room's entry cell.
      const boxSet = new Set(curBoxes);
      const walk = levelWalk(W, H, walkable, curPlayer, room.entry, boxSet);
      if (walk === null) return "";
      moves += walk;
      const res = solveRoom(W, region, room.entry, room.boxes, room.goals, hallwayRow, !last, room.doors, budget, work);
      if (res.status !== "ok") return "";
      moves += res.moves;
      // Replay the room solution to update box/player positions.
      const replay = replayMoves(W, H, grid, curBoxes, room.entry, res.moves);
      if (!replay) return "";
      for (let i = 0; i < curBoxes.length; i++) curBoxes[i] = replay.boxes[i];
      curPlayer = replay.player;
    }
    return moves;
  }

  // Fallback: the level does not decompose as specified; solve it whole.
  const region = new Set();
  for (let i = 0; i < W * H; i++) if (walkable[i]) region.add(i);
  const res = solveRoom(W, region, player, boxes, goals, -1, false, [], budget, work);
  if (res.status !== "ok") return "";
  return res.moves;
}

// Replay a move string on the level; returns { boxes, player } or null if
// any step is illegal. Used to track positions between rooms.
function replayMoves(W, H, grid, boxes0, player0, moves) {
  const boxes = boxes0.slice();
  let player = player0;
  function boxAt(cell) {
    return boxes.indexOf(cell);
  }
  for (const ch of moves) {
    let dr = 0;
    let dc = 0;
    if (ch === "u") dr = -1;
    else if (ch === "d") dr = 1;
    else if (ch === "l") dc = -1;
    else if (ch === "r") dc = 1;
    else return null;
    const r = Math.floor(player / W);
    const c = player % W;
    const nr = r + dr;
    const nc = c + dc;
    if (nr < 0 || nr >= H || nc < 0 || nc >= W) return null;
    if (grid[nr][nc] === "#") return null;
    const next = nr * W + nc;
    const bi = boxAt(next);
    if (bi >= 0) {
      const br = nr + dr;
      const bc = nc + dc;
      if (br < 0 || br >= H || bc < 0 || bc >= W) return null;
      if (grid[br][bc] === "#") return null;
      const beyond = br * W + bc;
      if (boxAt(beyond) >= 0) return null;
      boxes[bi] = beyond;
    }
    player = next;
  }
  return { boxes, player };
}

// ---------------------------------------------------------------------------
// Main: solve every instance under /input, write /output/<id>/solution.txt.
// ---------------------------------------------------------------------------
function main() {
  const inputDir = "/input";
  const outputDir = "/output";
  let ids;
  try {
    ids = fs.readdirSync(inputDir).filter((d) => {
      try {
        return fs.statSync(inputDir + "/" + d).isDirectory();
      } catch (e) {
        return false;
      }
    });
  } catch (e) {
    ids = [];
  }
  ids.sort();
  for (const id of ids) {
    let text = "";
    try {
      text = fs.readFileSync(inputDir + "/" + id + "/level.txt", "utf8");
    } catch (e) {
      text = "";
    }
    let moves = "";
    try {
      moves = solveLevel(text);
    } catch (e) {
      moves = "";
    }
    const outDir = outputDir + "/" + id;
    fs.mkdirSync(outDir, { recursive: true });
    fs.writeFileSync(outDir + "/solution.txt", moves);
  }
}

main();
