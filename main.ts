// Push-optimal Sokoban solver: room decomposition + push-optimal A* per room.
//
// Structure (from the kit's published rules): every level is a 1-wide hallway
// plus 1-4 rooms, each in a 6x6 slot with a single door onto the hallway.
// Boxes never leave their room, so the level's push optimum P* is the sum of
// the rooms' optima. We therefore solve each room independently with a
// push-optimal A* (state = box positions + canonical player-reachable
// region; admissible heuristic = min-cost bipartite matching of boxes to
// goals on box-free BFS distances), then stitch the rooms together by
// BFS-walking the player through the hallway from door to door.
//
// Deterministic by construction: fixed iteration orders, fixed tie-breaks,
// work-based expansion budgets only (no clock, no randomness).
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

const DIRS = [
  { dx: 0, dy: -1, ch: 'u' },
  { dx: 0, dy: 1, ch: 'd' },
  { dx: -1, dy: 0, ch: 'l' },
  { dx: 1, dy: 0, ch: 'r' },
] as const;

// Work-based budgets only (never the clock): total push-state expansions
// across the whole set, so two runs always do identical work.
let expansionsLeft = 2500000;

interface Level {
  W: number;
  H: number;
  wall: boolean[];
  goals: Set<number>;
  boxes: number[];
  player: number;
}

function parseLevel(text: string): Level {
  const rows = text.replace(/\r/g, '').split('\n').filter((r) => r.length > 0);
  const H = rows.length;
  const W = Math.max(0, ...rows.map((r) => r.length));
  const wall = new Array<boolean>(W * H).fill(true);
  const goals = new Set<number>();
  const boxes: number[] = [];
  let player = 0;
  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x++) {
      const c = rows[y][x] ?? ' ';
      const i = y * W + x;
      if (c === '#') continue;
      wall[i] = false;
      if (c === '.' || c === '*') goals.add(i);
      if (c === '$' || c === '*') boxes.push(i);
      if (c === '@' || c === '+') player = i;
    }
  }
  return { W, H, wall, goals, boxes, player };
}

const xy = (lv: Level, i: number): [number, number] => [i % lv.W, (i / lv.W) | 0];

// Flood fill over floor cells satisfying `ok`, from `from`. Sorted result.
function flood(lv: Level, from: number, ok: (i: number) => boolean): number[] {
  const seen = new Set<number>([from]);
  const q: number[] = [from];
  for (let qi = 0; qi < q.length; qi++) {
    const cur = q[qi];
    const [cx, cy] = xy(lv, cur);
    for (const d of DIRS) {
      const nx = cx + d.dx;
      const ny = cy + d.dy;
      if (nx < 0 || ny < 0 || nx >= lv.W || ny >= lv.H) continue;
      const ni = ny * lv.W + nx;
      if (lv.wall[ni] || !ok(ni) || seen.has(ni)) continue;
      seen.add(ni);
      q.push(ni);
    }
  }
  return [...seen].sort((a, b) => a - b);
}

// Shortest walk path (move string) from `from` to `to`, avoiding walls and
// `blocked` cells. Deterministic neighbour order. Null when unreachable.
function bfsPath(lv: Level, from: number, to: number, blocked: Set<number>): string | null {
  if (from === to) return '';
  const prev = new Map<number, { p: number; ch: string }>();
  const q: number[] = [from];
  prev.set(from, { p: -1, ch: '' });
  for (let qi = 0; qi < q.length; qi++) {
    const cur = q[qi];
    const [cx, cy] = xy(lv, cur);
    for (const d of DIRS) {
      const nx = cx + d.dx;
      const ny = cy + d.dy;
      if (nx < 0 || ny < 0 || nx >= lv.W || ny >= lv.H) continue;
      const ni = ny * lv.W + nx;
      if (lv.wall[ni] || blocked.has(ni) || prev.has(ni)) continue;
      prev.set(ni, { p: cur, ch: d.ch });
      if (ni === to) {
        let s = '';
        let c = ni;
        while (c !== from) {
          const e = prev.get(c)!;
          s = e.ch + s;
          c = e.p;
        }
        return s;
      }
      q.push(ni);
    }
  }
  return null;
}

// BFS distances from `src` over floor cells satisfying `ok` (boxes ignored).
function bfsDist(lv: Level, src: number, ok: (i: number) => boolean): Map<number, number> {
  const dist = new Map<number, number>([[src, 0]]);
  const q: number[] = [src];
  for (let qi = 0; qi < q.length; qi++) {
    const cur = q[qi];
    const [cx, cy] = xy(lv, cur);
    for (const d of DIRS) {
      const nx = cx + d.dx;
      const ny = cy + d.dy;
      if (nx < 0 || ny < 0 || nx >= lv.W || ny >= lv.H) continue;
      const ni = ny * lv.W + nx;
      if (lv.wall[ni] || !ok(ni) || dist.has(ni)) continue;
      dist.set(ni, dist.get(cur)! + 1);
      q.push(ni);
    }
  }
  return dist;
}

interface HeapItem {
  f: number;
  h: number;
  g: number;
  seq: number;
  key: string;
}

class Heap {
  private a: HeapItem[] = [];
  get size(): number {
    return this.a.length;
  }
  push(v: HeapItem): void {
    const a = this.a;
    a.push(v);
    let i = a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (cmpItem(a[p], a[i]) <= 0) break;
      [a[p], a[i]] = [a[i], a[p]];
      i = p;
    }
  }
  pop(): HeapItem | undefined {
    const a = this.a;
    if (a.length === 0) return undefined;
    const top = a[0];
    const last = a.pop()!;
    if (a.length > 0) {
      a[0] = last;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1;
        const r = l + 1;
        let m = i;
        if (l < a.length && cmpItem(a[l], a[m]) < 0) m = l;
        if (r < a.length && cmpItem(a[r], a[m]) < 0) m = r;
        if (m === i) break;
        [a[m], a[i]] = [a[i], a[m]];
        i = m;
      }
    }
    return top;
  }
}

function cmpItem(x: HeapItem, y: HeapItem): number {
  if (x.f !== y.f) return x.f - y.f;
  if (x.h !== y.h) return x.h - y.h;
  return x.seq - y.seq;
}

interface RoomCtx {
  lv: Level;
  cells: Set<number>; // room floor cells (door included)
  boxes0: number[]; // initial room boxes, sorted
  goals: number[]; // room goals, sorted
  goalsSet: Set<number>;
  entry: number; // hallway cell adjacent to the door
  dead: Set<number>; // dead-square cells (non-goal corners)
  dist: Map<number, Map<number, number>>; // box-free distances within the room
  otherBoxes: Set<number>; // boxes outside this room (fixed walking obstacles)
}

function buildRoomCtx(
  lv: Level,
  cells: Set<number>,
  boxes0: number[],
  goals: number[],
  entry: number,
  otherBoxes: Set<number>,
): RoomCtx {
  const goalsSet = new Set(goals);
  const inRoom = (i: number): boolean => cells.has(i);
  // Dead squares: non-goal floor cells with walls on two perpendicular sides.
  const dead = new Set<number>();
  for (const c of cells) {
    if (goalsSet.has(c)) continue;
    const [x, y] = xy(lv, c);
    const wallAt = (nx: number, ny: number): boolean =>
      nx < 0 || ny < 0 || nx >= lv.W || ny >= lv.H || lv.wall[ny * lv.W + nx];
    const u = wallAt(x, y - 1);
    const d = wallAt(x, y + 1);
    const l = wallAt(x - 1, y);
    const r = wallAt(x + 1, y);
    if ((u || d) && (l || r)) dead.add(c);
  }
  const dist = new Map<number, Map<number, number>>();
  for (const c of cells) dist.set(c, bfsDist(lv, c, inRoom));
  return { lv, cells, boxes0: boxes0.slice().sort((a, b) => a - b), goals: goals.slice().sort((a, b) => a - b), goalsSet, entry, dead, dist, otherBoxes };
}

// Minimum-cost bipartite matching of boxes to goals (n <= 4: brute force
// with pruning). Infinity when some box can never reach some assigned goal.
function matchingHeuristic(ctx: RoomCtx, boxes: number[]): number {
  const n = boxes.length;
  const used = new Array<boolean>(n).fill(false);
  let best = Infinity;
  const rec = (i: number, acc: number): void => {
    if (acc >= best) return;
    if (i === n) {
      best = acc;
      return;
    }
    for (let j = 0; j < n; j++) {
      if (used[j]) continue;
      const d = ctx.dist.get(boxes[i])!.get(ctx.goals[j]);
      if (d === undefined) continue;
      used[j] = true;
      rec(i + 1, acc + d);
      used[j] = false;
    }
  };
  rec(0, 0);
  return best;
}

// A box off-goal on a dead square, or inside a 2x2 block fully occupied by
// walls/boxes, can never move again.
function isDeadState(ctx: RoomCtx, boxes: number[]): boolean {
  const { lv } = ctx;
  const boxSet = new Set(boxes);
  for (const b of boxes) {
    if (ctx.goalsSet.has(b)) continue;
    if (ctx.dead.has(b)) return true;
    const [x, y] = xy(lv, b);
    for (const [ox, oy] of [[-1, -1], [0, -1], [-1, 0], [0, 0]] as const) {
      let full = true;
      for (let dy = 0; dy < 2; dy++) {
        for (let dx = 0; dx < 2; dx++) {
          const nx = x + ox + dx;
          const ny = y + oy + dy;
          const ni = ny * lv.W + nx;
          if (nx < 0 || ny < 0 || nx >= lv.W || ny >= lv.H) continue; // outside board counts as wall
          if (!lv.wall[ni] && !boxSet.has(ni)) {
            full = false;
          }
        }
      }
      if (full) return true;
    }
  }
  return false;
}

interface NodeInfo {
  boxes: number[];
  player: number;
  region: number[];
  g: number;
  h: number;
  parent: string | null;
  seg: string;
}

// Push-optimal (A*) or greedy best-first search for one room.
// Returns the move string from `entry`, the push count and the final player cell.
function searchRoom(
  ctx: RoomCtx,
  greedy: boolean,
  budget: number,
): { moves: string; pushes: number; endPlayer: number } | null {
  const { lv } = ctx;
  const startBoxes = ctx.boxes0.slice();
  const startBlocked = new Set<number>([...ctx.otherBoxes, ...startBoxes]);
  const startRegion = flood(lv, ctx.entry, (i) => !startBlocked.has(i));
  const startKey = startBoxes.join(',') + '|' + startRegion.join(',');
  const h0 = matchingHeuristic(ctx, startBoxes);
  if (h0 === Infinity || isDeadState(ctx, startBoxes)) return null;

  const nodes = new Map<string, NodeInfo>();
  const heap = new Heap();
  let seq = 0;
  const startNode: NodeInfo = { boxes: startBoxes, player: ctx.entry, region: startRegion, g: 0, h: h0, parent: null, seg: '' };
  nodes.set(startKey, startNode);
  heap.push({ f: greedy ? h0 : h0, h: h0, g: 0, seq: seq++, key: startKey });

  let spent = 0;
  while (heap.size > 0) {
    const item = heap.pop()!;
    const node = nodes.get(item.key);
    if (!node || node.g !== item.g) continue; // stale heap entry
    if (spent >= budget || expansionsLeft <= 0) return null;
    // Node-count guard: the sandbox has 512 MB, and a pathological room
    // could otherwise grow the closed set without bound. Aborting here lets
    // the caller fall back (greedy search, then an empty answer) instead of
    // dying on memory. Practice rooms stay under ~2,000 nodes.
    if (nodes.size > 1000000) return null;
    spent++;
    expansionsLeft--;
    if (node.boxes.every((b) => ctx.goalsSet.has(b))) {
      // Reconstruct the move string.
      const segs: string[] = [];
      let cur: NodeInfo | undefined = node;
      let k: string | null = item.key;
      while (cur && cur.parent !== null) {
        segs.push(cur.seg);
        k = cur.parent;
        cur = k === null ? undefined : nodes.get(k);
      }
      segs.reverse();
      return { moves: segs.join(''), pushes: node.g, endPlayer: node.player };
    }
    const boxSet = new Set(node.boxes);
    const allBlocked = new Set<number>([...ctx.otherBoxes, ...node.boxes]);
    for (const b of node.boxes) {
      const [bx, by] = xy(lv, b);
      for (const d of DIRS) {
        const dx = bx + d.dx;
        const dy = by + d.dy;
        if (dx < 0 || dy < 0 || dx >= lv.W || dy >= lv.H) continue;
        const dest = dy * lv.W + dx;
        if (lv.wall[dest] || boxSet.has(dest)) continue;
        if (!ctx.cells.has(dest)) continue; // boxes never leave their room
        const px = bx - d.dx;
        const py = by - d.dy;
        if (px < 0 || py < 0 || px >= lv.W || py >= lv.H) continue;
        const behind = py * lv.W + px;
        if (!node.region.includes(behind)) continue; // player cannot reach the pushing spot
        const walk = bfsPath(lv, node.player, behind, allBlocked);
        if (walk === null) continue;
        const nb = node.boxes.slice();
        nb[nb.indexOf(b)] = dest;
        nb.sort((x, y) => x - y);
        if (isDeadState(ctx, nb)) continue;
        const h = matchingHeuristic(ctx, nb);
        if (h === Infinity) continue;
        const nBlocked = new Set<number>([...ctx.otherBoxes, ...nb]);
        const region = flood(lv, b, (i) => !nBlocked.has(i));
        const key = nb.join(',') + '|' + region.join(',');
        const g = node.g + 1;
        const prev = nodes.get(key);
        if (prev && prev.g <= g) continue;
        const info: NodeInfo = { boxes: nb, player: b, region, g, h, parent: item.key, seg: walk + d.ch };
        nodes.set(key, info);
        heap.push({ f: greedy ? h : g + h, h, g, seq: seq++, key });
      }
    }
  }
  return null;
}

// Solve one room: A* first, greedy best-first as a fallback so the instance
// is still answered (a non-optimal solved answer beats 0).
function solveRoom(ctx: RoomCtx): { moves: string; pushes: number; endPlayer: number } | null {
  const astarBudget = Math.min(600000, expansionsLeft);
  const r = searchRoom(ctx, false, astarBudget);
  if (r) return r;
  const greedyBudget = Math.min(200000, expansionsLeft);
  return searchRoom(ctx, true, greedyBudget);
}

interface Room {
  cells: Set<number>;
  boxes: number[];
  goals: number[];
  door: number;
  entry: number;
}

// Decompose a level into hallway + rooms. Null when the structure is not as
// expected (caller falls back to a whole-level search).
function decompose(lv: Level): Room[] | null {
  const hallRow = (lv.player / lv.W) | 0;
  const seen = new Set<number>();
  const rooms: Room[] = [];
  for (let i = 0; i < lv.W * lv.H; i++) {
    if (lv.wall[i] || (i / lv.W | 0) === hallRow || seen.has(i)) continue;
    const comp = flood(lv, i, (j) => (j / lv.W | 0) !== hallRow);
    for (const c of comp) seen.add(c);
    const cells = new Set(comp);
    const boxes = lv.boxes.filter((b) => cells.has(b));
    const goals = [...lv.goals].filter((g) => cells.has(g));
    if (boxes.length === 0) continue; // stray floor with no boxes
    if (boxes.length !== goals.length) return null;
    // The door: a room cell adjacent to a hallway floor cell.
    let door = -1;
    let entry = -1;
    for (const c of [...cells].sort((a, b) => a - b)) {
      const [x, y] = xy(lv, c);
      for (const d of DIRS) {
        const nx = x + d.dx;
        const ny = y + d.dy;
        if (ny !== hallRow || nx < 0 || nx >= lv.W) continue;
        const ni = ny * lv.W + nx;
        if (!lv.wall[ni]) {
          door = c;
          entry = ni;
          break;
        }
      }
      if (door !== -1) break;
    }
    if (door === -1) return null;
    rooms.push({ cells, boxes, goals, door, entry });
  }
  if (rooms.length === 0) return null;
  // Every box must belong to exactly one room.
  const roomBoxes = rooms.reduce((n, r) => n + r.boxes.length, 0);
  if (roomBoxes !== lv.boxes.length) return null;
  rooms.sort((a, b) => a.door - b.door);
  return rooms;
}

// Replay-verify a move string on the level; true only when every box ends
// on a goal and no step was illegal.
function replayOk(lv: Level, moves: string): boolean {
  const boxes = new Set(lv.boxes);
  let p = lv.player;
  for (const ch of moves) {
    const d = DIRS.find((dd) => dd.ch === ch);
    if (!d) return false;
    const [px, py] = xy(lv, p);
    const nx = px + d.dx;
    const ny = py + d.dy;
    if (nx < 0 || ny < 0 || nx >= lv.W || ny >= lv.H) return false;
    const ni = ny * lv.W + nx;
    if (lv.wall[ni]) return false;
    if (boxes.has(ni)) {
      const bx = nx + d.dx;
      const by = ny + d.dy;
      if (bx < 0 || by < 0 || bx >= lv.W || by >= lv.H) return false;
      const bi = by * lv.W + bx;
      if (lv.wall[bi] || boxes.has(bi)) return false;
      boxes.delete(ni);
      boxes.add(bi);
    }
    p = ni;
  }
  return [...boxes].every((b) => lv.goals.has(b));
}

function solveLevel(text: string): string {
  const lv = parseLevel(text);
  if (lv.boxes.length === 0) return '';
  const rooms = decompose(lv);
  let curBoxes = lv.boxes.slice();
  let curPlayer = lv.player;
  let moves = '';

  if (rooms) {
    for (const room of rooms) {
      const allBlocked = new Set(curBoxes);
      const walk = bfsPath(lv, curPlayer, room.entry, allBlocked);
      if (walk === null) return '';
      moves += walk;
      curPlayer = room.entry;
      const otherBoxes = new Set(curBoxes.filter((b) => !room.cells.has(b)));
      const roomBoxes = curBoxes.filter((b) => room.cells.has(b));
      const ctx = buildRoomCtx(lv, room.cells, roomBoxes, room.goals, room.entry, otherBoxes);
      const solved = solveRoom(ctx);
      if (!solved) return '';
      moves += solved.moves;
      curPlayer = solved.endPlayer;
      // The room's boxes are now on its goals.
      curBoxes = [...otherBoxes, ...room.goals];
    }
  } else {
    // Fallback: whole-level push search (small levels only).
    const allCells = new Set<number>();
    for (let i = 0; i < lv.W * lv.H; i++) if (!lv.wall[i]) allCells.add(i);
    const ctx = buildRoomCtx(lv, allCells, lv.boxes, [...lv.goals], lv.player, new Set());
    const solved = solveRoom(ctx);
    if (!solved) return '';
    moves = solved.moves;
  }

  return replayOk(lv, moves) ? moves : '';
}

for (const id of readdirSync(I).sort()) {
  const moves = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
