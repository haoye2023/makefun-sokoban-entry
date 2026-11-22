// Sokoban solver for make.fun's generated levels (hallway + rooms of 3-4 boxes).
// Strategy: split the level at its door cells into independent rooms, solve each room
// push-optimally with an A* over pushes (Hungarian-matching heuristic on push distances,
// dead-square and frozen-2x2 deadlock pruning), then stitch the rooms together with
// shortest player walks through the hallway. All budgets are expansion counters, never
// the clock, so two runs of the program write byte-identical answers.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const INPUT_DIR = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const OUTPUT_DIR = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';
const DX = [0, 0, -1, 1];
const DY = [-1, 1, 0, 0];
const DC = ['u', 'd', 'l', 'r'];
const INF = 1e9;

interface Level {
  w: number;
  h: number;
  wall: Uint8Array;
  goal: Uint8Array;
  goals: number[];
  boxes: number[];
  player: number;
}

function parseLevel(text: string): Level {
  const rows = text
    .split('\n')
    .filter((r) => r.length > 0)
    .map((r) => r.replace(/\r$/, ''));
  const w = Math.max(...rows.map((r) => r.length));
  const h = rows.length;
  const wall = new Uint8Array(w * h).fill(1);
  const goal = new Uint8Array(w * h);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = 0;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x]!;
      const i = y * w + x;
      if (ch !== '#') wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') {
        goal[i] = 1;
        goals.push(i);
      }
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goal, goals, boxes, player };
}

// Breadth-first flood over the cells of `allowed` that are not box-occupied, from `from`.
// Returns the reached mask and a parent array (parent[cell] = previous cell, -1 at root).
function flood(
  w: number,
  allowed: Uint8Array,
  boxSet: Set<number>,
  from: number,
): { reach: Uint8Array; parent: Int32Array } {
  const n = allowed.length;
  const reach = new Uint8Array(n);
  const parent = new Int32Array(n).fill(-1);
  const queue = new Int32Array(n);
  let qh = 0;
  let qt = 0;
  queue[qt++] = from;
  reach[from] = 1;
  while (qh < qt) {
    const c = queue[qh++]!;
    for (let d = 0; d < 4; d++) {
      const nx = (c % w) + DX[d]!;
      const ny = ((c / w) | 0) + DY[d]!;
      if (nx < 0 || ny < 0 || nx >= w) continue;
      const t = c + DY[d]! * w + DX[d]!;
      if (t < 0 || t >= n || reach[t] || !allowed[t] || boxSet.has(t)) continue;
      reach[t] = 1;
      parent[t] = c;
      queue[qt++] = t;
    }
  }
  return { reach, parent };
}

// The canonical representative of a player region: its smallest cell index.
function regionRep(reach: Uint8Array): number {
  for (let i = 0; i < reach.length; i++) if (reach[i]) return i;
  return -1;
}

// Move letters of the flood-tree path from the root to `to` (both in the same flood).
function walkString(w: number, parent: Int32Array, from: number, to: number): string {
  const chars: string[] = [];
  let c = to;
  while (c !== from) {
    const p = parent[c]!;
    const diff = c - p;
    chars.push(diff === -w ? 'u' : diff === w ? 'd' : diff === -1 ? 'l' : 'r');
    c = p;
  }
  return chars.reverse().join('');
}

// Shortest player walk on open floor avoiding boxes, as move letters; null if none.
function walkPath(
  lv: Level,
  boxSet: Set<number>,
  from: number,
  to: number,
): string | null {
  if (from === to) return '';
  const open = new Uint8Array(lv.wall.length);
  for (let i = 0; i < open.length; i++) open[i] = lv.wall[i] ? 0 : 1;
  const { reach, parent } = flood(lv.w, open, boxSet, from);
  if (!reach[to]) return null;
  return walkString(lv.w, parent, from, to);
}

// ---------------------------------------------------------------------------
// Push-distance tables for one region: for every goal, the fewest pushes that
// bring a lone box from any cell to that goal (other boxes ignored, the player
// must have a legal stand cell for each push). A cell that reaches no goal is a
// dead square: a box resting there (off a goal) can never be solved.
// ---------------------------------------------------------------------------
interface PushTables {
  distTo: Int32Array[]; // distTo[goalIndex][cell]
  dead: Uint8Array;
}

function buildPushTables(
  w: number,
  wall: Uint8Array,
  boxAllowed: Uint8Array,
  standAllowed: Uint8Array,
  goals: number[],
): PushTables {
  const n = wall.length;
  const distTo: Int32Array[] = [];
  const dead = new Uint8Array(n).fill(1);
  for (const g of goals) {
    const dist = new Int32Array(n).fill(INF);
    dist[g] = 0;
    const queue = new Int32Array(n);
    let qh = 0;
    let qt = 0;
    queue[qt++] = g;
    while (qh < qt) {
      const y = queue[qh++]!;
      // Reverse one push: the box came from x = y - d, the player stood at x - d.
      for (let d = 0; d < 4; d++) {
        const delta = DY[d]! * w + DX[d]!;
        const x = y - delta;
        const stand = y - 2 * delta;
        if (x < 0 || x >= n || stand < 0 || stand >= n) continue;
        // Guard against row wrap-around for the horizontal directions.
        if (DX[d] !== 0 && ((x / w) | 0) !== ((y / w) | 0)) continue;
        if (DX[d] !== 0 && ((stand / w) | 0) !== ((x / w) | 0)) continue;
        if (!boxAllowed[x] || wall[x] || !standAllowed[stand] || wall[stand]) continue;
        if (dist[x] !== INF) continue;
        dist[x] = dist[y]! + 1;
        queue[qt++] = x;
      }
    }
    distTo.push(dist);
    for (let i = 0; i < n; i++) if (dist[i] !== INF) dead[i] = 0;
  }
  return { distTo, dead };
}

// Admissible push lower bound: the cheapest assignment of boxes to distinct
// goals under the push distances (exact matching for up to 6 boxes, a greedy
// per-box minimum beyond that, which stays admissible).
function heuristic(tables: PushTables, boxes: number[]): number {
  const { distTo } = tables;
  const n = boxes.length;
  if (n === 0) return 0;
  if (n > 6) {
    let sum = 0;
    for (const b of boxes) {
      let best = INF;
      for (const dist of distTo) if (dist[b]! < best) best = dist[b]!;
      if (best >= INF) return INF;
      sum += best;
    }
    return sum;
  }
  const size = 1 << n;
  const dp = new Float64Array(size).fill(INF);
  dp[0] = 0;
  const popcount = new Uint8Array(size);
  for (let m = 1; m < size; m++) popcount[m] = popcount[m >> 1]! + (m & 1);
  for (let mask = 0; mask < size; mask++) {
    const i = popcount[mask]!; // next box to assign
    if (i >= n || dp[mask]! >= INF) continue;
    const b = boxes[i]!;
    for (let j = 0; j < n; j++) {
      if (mask & (1 << j)) continue;
      const cost = distTo[j]![b]!;
      if (cost >= INF) continue;
      const nm = mask | (1 << j);
      if (dp[mask]! + cost < dp[nm]!) dp[nm] = dp[mask]! + cost;
    }
  }
  return dp[size - 1]!;
}

// ---------------------------------------------------------------------------
// A* over pushes for one region. A state is the sorted box cells plus the
// player's reachable region; the cost is pushes. Returns the full move string
// (walks + pushes, all lowercase), or null when the budget runs out or the
// position is hopeless.
// ---------------------------------------------------------------------------
interface RegionProblem {
  w: number;
  wall: Uint8Array;
  goal: Uint8Array;
  goals: number[];
  boxAllowed: Uint8Array;
  playerAllowed: Uint8Array;
  startBoxes: number[];
  startPlayer: number;
  exitCell: number; // if >= 0, a goal state must let the player reach this cell
}

interface Budget {
  left: number;
}

function solvePush(prob: RegionProblem, budget: Budget): { moves: string; endPlayer: number } | null {
  const { w, wall, goal, goals, boxAllowed, playerAllowed } = prob;
  const tables = buildPushTables(w, wall, boxAllowed, playerAllowed, goals);
  const startBoxes = [...prob.startBoxes].sort((a, b) => a - b);
  for (const b of startBoxes) if (tables.dead[b] && !goal[b]) return null;
  const h0 = heuristic(tables, startBoxes);
  if (h0 >= INF) return null;

  const boxKey = (boxes: number[], rep: number): string =>
    String.fromCharCode(rep, ...boxes);

  // Frozen 2x2 deadlock: four squares of walls and boxes with a box among them
  // off its goal can never be undone. (A recursive freeze test was tried here
  // and removed: memoising "frozen" across recursion roots is unsound — a box
  // can look frozen only because a neighbour was assumed frozen mid-chain,
  // while that neighbour is in fact free to move away.)
  const deadlock = (boxes: number[], boxSet: Set<number>, movedTo: number): boolean => {
    void boxes;
    for (const t of [movedTo, movedTo - 1, movedTo - w, movedTo - w - 1]) {
      if (t < 0 || t + w + 1 >= wall.length || (t % w) === w - 1) continue;
      const cells = [t, t + 1, t + w, t + w + 1];
      let full = true;
      let badBox = false;
      for (const c of cells) {
        if (wall[c]) continue;
        if (boxSet.has(c)) {
          if (!goal[c]) badBox = true;
          continue;
        }
        full = false;
        break;
      }
      if (full && badBox) return true;
    }
    return false;
  };

  interface State {
    boxes: number[];
    player: number;
    g: number;
    h: number;
    parent: number;
    seg: string;
  }
  const states: State[] = [];
  const best = new Map<string, number>();
  const heap: number[] = [];
  const less = (a: number, b: number): boolean => {
    const sa = states[a]!;
    const sb = states[b]!;
    const fa = sa.g + sa.h;
    const fb = sb.g + sb.h;
    if (fa !== fb) return fa < fb;
    if (sa.h !== sb.h) return sa.h < sb.h;
    if (sa.g !== sb.g) return sa.g > sb.g;
    return a < b;
  };
  const heapPush = (idx: number): void => {
    heap.push(idx);
    let i = heap.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i]!, heap[p]!)) break;
      [heap[i], heap[p]] = [heap[p]!, heap[i]!];
      i = p;
    }
  };
  const heapPop = (): number => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1]!, heap[c]!)) c++;
        if (!less(heap[c]!, heap[i]!)) break;
        [heap[i], heap[c]] = [heap[c]!, heap[i]!];
        i = c;
      }
    }
    return top;
  };

  const startSet = new Set(startBoxes);
  const startFlood = flood(w, playerAllowed, startSet, prob.startPlayer);
  const start: State = {
    boxes: startBoxes,
    player: prob.startPlayer,
    g: 0,
    h: h0,
    parent: -1,
    seg: '',
  };
  states.push(start);
  best.set(boxKey(startBoxes, regionRep(startFlood.reach)), 0);
  heapPush(0);

  const isGoal = (s: State): boolean => {
    for (const b of s.boxes) if (!goal[b]) return false;
    if (prob.exitCell < 0) return true;
    const set = new Set(s.boxes);
    const { reach } = flood(w, playerAllowed, set, s.player);
    return reach[prob.exitCell] === 1;
  };

  while (heap.length > 0) {
    const idx = heapPop();
    const s = states[idx]!;
    const boxSet = new Set(s.boxes);
    const sFlood = flood(w, playerAllowed, boxSet, s.player);
    const key = boxKey(s.boxes, regionRep(sFlood.reach));
    if (best.get(key) !== idx) continue;
    if (isGoal(s)) {
      const parts: string[] = [];
      for (let t = idx; t > 0; t = states[t]!.parent) parts.push(states[t]!.seg);
      return { moves: parts.reverse().join(''), endPlayer: s.player };
    }
    if (budget.left <= 0) return null;
    budget.left--;
    const { reach, parent } = sFlood;
    for (let bi = 0; bi < s.boxes.length; bi++) {
      const b = s.boxes[bi]!;
      for (let d = 0; d < 4; d++) {
        const delta = DY[d]! * w + DX[d]!;
        const stand = b - delta;
        const dest = b + delta;
        if (stand < 0 || stand >= wall.length || !reach[stand]) continue;
        if (dest < 0 || dest >= wall.length || wall[dest] || boxSet.has(dest)) continue;
        if (!boxAllowed[dest] || tables.dead[dest]) continue;
        const nb = s.boxes.slice();
        nb[bi] = dest;
        nb.sort((a, b2) => a - b2);
        const nSet = new Set(nb);
        if (deadlock(nb, nSet, dest)) continue;
        const nh = heuristic(tables, nb);
        if (nh >= INF) continue;
        const nFlood = flood(w, playerAllowed, nSet, b);
        const nKey = boxKey(nb, regionRep(nFlood.reach));
        const seenIdx = best.get(nKey);
        if (seenIdx !== undefined && states[seenIdx]!.g <= s.g + 1) continue;
        const seg = walkString(w, parent, s.player, stand) + DC[d]!;
        const nState: State = {
          boxes: nb,
          player: b,
          g: s.g + 1,
          h: nh,
          parent: idx,
          seg,
        };
        const nIdx = states.length;
        states.push(nState);
        best.set(nKey, nIdx);
        heapPush(nIdx);
      }
    }
  }
  return null;
}

// ---------------------------------------------------------------------------
// Decomposition. The hallway is the player's row. A door is a floor cell one
// row above or below it whose removal cuts off a region holding as many boxes
// as goals; that region plus the door cell is one room, solved on its own.
// The player enters a room from the hallway cell in front of its door.
// ---------------------------------------------------------------------------
interface Room {
  door: number;
  hall: number;
  boxAllowed: Uint8Array;
  playerAllowed: Uint8Array;
  goals: number[];
  boxes: number[];
}

function decompose(lv: Level): Room[] | null {
  const { w, wall } = lv;
  const n = wall.length;
  const yH = (lv.player / w) | 0;
  const floorMask = new Uint8Array(n);
  for (let i = 0; i < n; i++) floorMask[i] = wall[i] ? 0 : 1;
  const rooms: Room[] = [];
  const claimed = new Set<number>();
  for (let d = 0; d < n; d++) {
    if (wall[d]) continue;
    const dx = d % w;
    const dy = (d / w) | 0;
    if (Math.abs(dy - yH) !== 1) continue;
    const hall = dx + yH * w;
    if (wall[hall]) continue;
    const side = dy < yH ? -1 : 1;
    const seeds: number[] = [];
    for (let dir = 0; dir < 4; dir++) {
      const t = d + DY[dir]! * w + DX[dir]!;
      if (t < 0 || t >= n || wall[t] || t === hall) continue;
      if (Math.sign(((t / w) | 0) - yH) !== side) continue;
      seeds.push(t);
    }
    if (seeds.length === 0) continue;
    const blocked = new Set<number>([d]);
    const { reach } = flood(w, floorMask, blocked, seeds[0]!);
    if (reach[lv.player]) continue;
    let consistent = true;
    for (const s of seeds) if (!reach[s]) consistent = false;
    if (!consistent) continue;
    const region: number[] = [];
    for (let i = 0; i < n; i++) if (reach[i]) region.push(i);
    const boxes: number[] = [];
    const goals: number[] = [];
    for (const b of lv.boxes) if (reach[b] || b === d) boxes.push(b);
    for (const g of lv.goals) if (reach[g] || g === d) goals.push(g);
    if (boxes.length === 0 || boxes.length !== goals.length) continue;
    let overlap = false;
    for (const b of boxes) if (claimed.has(b)) overlap = true;
    if (overlap) return null;
    for (const b of boxes) claimed.add(b);
    const boxAllowed = new Uint8Array(n);
    const playerAllowed = new Uint8Array(n);
    for (const c of region) {
      boxAllowed[c] = 1;
      playerAllowed[c] = 1;
    }
    boxAllowed[d] = 1;
    playerAllowed[d] = 1;
    playerAllowed[hall] = 1;
    rooms.push({ door: d, hall, boxAllowed, playerAllowed, goals, boxes });
  }
  if (rooms.length === 0) return null;
  if (claimed.size !== lv.boxes.length) return null;
  return rooms;
}

// Replay moves on a position; returns the new position or null when illegal.
function simulate(
  lv: Level,
  boxSet: Set<number>,
  player: number,
  moves: string,
): { boxes: Set<number>; player: number } | null {
  const boxes = new Set(boxSet);
  let p = player;
  for (const ch of moves) {
    const d = DC.indexOf(ch);
    if (d < 0) return null;
    const delta = DY[d]! * lv.w + DX[d]!;
    const q = p + delta;
    if (q < 0 || q >= lv.wall.length || lv.wall[q]) return null;
    if (boxes.has(q)) {
      const dest = q + delta;
      if (dest < 0 || dest >= lv.wall.length || lv.wall[dest] || boxes.has(dest)) {
        return null;
      }
      boxes.delete(q);
      boxes.add(dest);
    }
    p = q;
  }
  return { boxes, player: p };
}

function solveByRooms(lv: Level, rooms: Room[], budget: Budget): string | null {
  const order = [...rooms].sort(
    (a, b) => (a.door % lv.w) - (b.door % lv.w) || a.door - b.door,
  );
  let boxes = new Set(lv.boxes);
  let player = lv.player;
  const parts: string[] = [];
  for (let i = 0; i < order.length; i++) {
    const room = order[i]!;
    const walk = walkPath(lv, boxes, player, room.hall);
    if (walk === null) return null;
    const res = solvePush(
      {
        w: lv.w,
        wall: lv.wall,
        goal: lv.goal,
        goals: room.goals,
        boxAllowed: room.boxAllowed,
        playerAllowed: room.playerAllowed,
        startBoxes: room.boxes,
        startPlayer: room.hall,
        exitCell: i === order.length - 1 ? -1 : room.door,
      },
      budget,
    );
    if (res) {
      const sim = simulate(lv, boxes, room.hall, res.moves);
      if (!sim) return null;
      parts.push(walk, res.moves);
      boxes = sim.boxes;
      player = sim.player;
      continue;
    }
    // Retry in the reference solver's model: the whole board, the other
    // rooms' boxes standing as walls, the player roaming from where they are.
    const wall2 = lv.wall.slice();
    for (const b of boxes) if (!room.boxes.includes(b)) wall2[b] = 1;
    const open = new Uint8Array(lv.wall.length);
    for (let c = 0; c < open.length; c++) open[c] = wall2[c] ? 0 : 1;
    const res2 = solvePush(
      {
        w: lv.w,
        wall: wall2,
        goal: lv.goal,
        goals: room.goals,
        boxAllowed: open,
        playerAllowed: open,
        startBoxes: room.boxes,
        startPlayer: player,
        exitCell: -1,
      },
      budget,
    );
    if (!res2) return null;
    const sim2 = simulate(lv, boxes, player, res2.moves);
    if (!sim2) return null;
    parts.push(res2.moves);
    boxes = sim2.boxes;
    player = sim2.player;
  }
  for (const b of boxes) if (!lv.goal[b]) return null;
  return parts.join('');
}

function solveLevel(lv: Level): string {
  const budget: Budget = { left: 800_000 };
  const rooms = decompose(lv);
  if (rooms) {
    const solved = solveByRooms(lv, rooms, budget);
    return solved !== null ? solved : '';
  }
  // No room structure found: one bounded whole-board search, then give up
  // with a valid empty answer rather than risking the run's time or memory.
  const all = new Uint8Array(lv.wall.length);
  for (let i = 0; i < all.length; i++) all[i] = lv.wall[i] ? 0 : 1;
  const res = solvePush(
    {
      w: lv.w,
      wall: lv.wall,
      goal: lv.goal,
      goals: lv.goals,
      boxAllowed: all,
      playerAllowed: all,
      startBoxes: lv.boxes,
      startPlayer: lv.player,
      exitCell: -1,
    },
    { left: 250_000 },
  );
  return res ? res.moves : '';
}

function main(): void {
  let ids: string[] = [];
  try {
    ids = readdirSync(INPUT_DIR).sort();
  } catch {
    return;
  }
  for (const id of ids) {
    let moves = '';
    try {
      const text = readFileSync(join(INPUT_DIR, id, 'level.txt'), 'utf8');
      moves = solveLevel(parseLevel(text));
    } catch {
      moves = '';
    }
    try {
      mkdirSync(join(OUTPUT_DIR, id), { recursive: true });
      writeFileSync(join(OUTPUT_DIR, id, 'solution.txt'), `${moves}\n`);
    } catch {
      // Nothing else can be done for this instance.
    }
  }
}

main();
