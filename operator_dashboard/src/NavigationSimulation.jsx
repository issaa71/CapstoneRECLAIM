import React, { useState, useRef, useCallback, useMemo, useEffect, useReducer, memo, createContext, useContext } from 'react'
import { Canvas, useFrame, useThree, createPortal } from '@react-three/fiber'
import { OrbitControls, Line, Text, Html, PerspectiveCamera } from '@react-three/drei'
import * as THREE from 'three'

// ═══════════════════════════════════════════════════════════════
// SECTION 0: CONSTANTS
// ═══════════════════════════════════════════════════════════════

const ROBOT_WIDTH = 0.5
const ROBOT_DEPTH = 0.4
const ROBOT_HEIGHT = 0.15
const MAX_LINEAR_VEL = 0.5
const MAX_ANGULAR_VEL = 1.0
const LINEAR_ACC = 0.5
const ANGULAR_ACC = 1.5
const ARM_REACH = 0.41
const CAMERA_FOV_DEG = 73
const CAMERA_MIN_RANGE = 0.4
const CAMERA_MAX_RANGE = 3.0
const LIDAR_MAX_RANGE = 6.0
const INFLATION_RADIUS = 0.5
const GRID_RESOLUTION = 0.1
const BOUSTROPHEDON_SPACING = 2.5
const BIN_CAPACITY = 15
const DUMP_THRESHOLD = 0.8
const PICKUP_TIME = 3.0
const DUMP_TIME = 1.5
const BATTERY_DRAIN_PER_METER = 0.1
const BATTERY_DRAIN_PER_PICK = 0.5
const BATTERY_DRAIN_PER_DUMP = 2.0
const FIXED_DT = 1 / 60
const GOLDEN_ANGLE = 2.39996323 // radians, ~137.508 degrees

const BIN_COLORS = { recyclable: '#3b82f6', compost: '#22c55e', landfill: '#6b7280' }

const ALGORITHMS = [
  { id: 'random_walk', label: 'Random Walk', color: '#ef4444', description: 'Random direction changes on collision' },
  { id: 'full_boustrophedon', label: 'Full Boustrophedon', color: '#eab308', description: 'Systematic lawnmower sweep coverage' },
  { id: 'nearest_neighbor', label: 'Nearest-Visible', color: '#f97316', description: 'Greedy nearest detected item + frontier exploration' },
  { id: 'info_gain', label: 'Info-Gain Explorer', color: '#06b6d4', description: 'FOV-aware exploration, maximizes unseen cells per meter' },
  { id: 'reclaim', label: 'RECLAIM', color: '#22c55e', description: '3-mode hybrid: collect + scan + sweep + bin intelligence' },
]
const RANDOM_WALK_TIMEOUT = 600 // 10 min simulated

// Smart Bin Balancing thresholds
const BIN_SOFT_PREF = 0.80            // 80% = 12/15 — soft preference for other bin types
const BIN_HARD_PREF = 0.93            // 93% = 14/15 — hard skip unless within 8m
const BIN_CONSOLIDATE_THRESHOLD = 0.53 // 53% = 8/15 — dump when ALL bins above this

// Info-Gain / RECLAIM mode constants
const INFO_GAIN_SCAN_ARC = (120 * Math.PI) / 180  // 120° scan arc
const RECLAIM_UNSCANNED_THRESHOLD = 0.20           // >20% unscanned → SCAN mode
const RECLAIM_SWEEP_THRESHOLD = 0.92               // >92% scanned → SWEEP mode
const VISIT_GRID_RESOLUTION = 0.5                  // visit frequency tracking

const WASTE_TYPES = [
  { typeId: 'aluminum_can', label: 'Aluminum Can', bin: 'recyclable', color: '#C0C0C0', shape: 'cylinder' },
  { typeId: 'plastic_bottle', label: 'Plastic Bottle', bin: 'recyclable', color: '#4FC3F7', shape: 'tallcylinder' },
  { typeId: 'glass_bottle', label: 'Glass Bottle', bin: 'recyclable', color: '#2E7D32', shape: 'tallcylinder' },
  { typeId: 'paper_cup', label: 'Paper Cup', bin: 'landfill', color: '#F5F5DC', shape: 'cone' },
  { typeId: 'napkin', label: 'Napkin', bin: 'landfill', color: '#FAFAFA', shape: 'flat' },
  { typeId: 'banana_peel', label: 'Banana Peel', bin: 'compost', color: '#FFD54F', shape: 'curved' },
  { typeId: 'apple_core', label: 'Apple Core', bin: 'compost', color: '#C62828', shape: 'sphere' },
  { typeId: 'food_container', label: 'Food Container', bin: 'compost', color: '#8D6E63', shape: 'box' },
]

let _nextId = 1
function nextId() { return _nextId++ }

// Shared ref context so 3D components read mutable sim state without React re-renders
const SimRefContext = createContext(null)

// ═══════════════════════════════════════════════════════════════
// SECTION 1: UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════════

function clamp(v, min, max) { return Math.max(min, Math.min(max, v)) }
function distXZ(a, b) { return Math.sqrt((a.x - b.x) ** 2 + (a.z - b.z) ** 2) }
function angleDiff(a, b) {
  let d = ((b - a) % (2 * Math.PI) + 3 * Math.PI) % (2 * Math.PI) - Math.PI
  return d
}
function lerpAngle(a, b, t) { return a + angleDiff(a, b) * t }
function moveToward(current, target, maxDelta) {
  if (Math.abs(target - current) <= maxDelta) return target
  return current + Math.sign(target - current) * maxDelta
}

function pointInFOVCone(rx, rz, rAngle, tx, tz, fovDeg, minR, maxR) {
  const dx = tx - rx, dz = tz - rz
  const dist = Math.sqrt(dx * dx + dz * dz)
  if (dist < minR || dist > maxR) return false
  const angleToTarget = Math.atan2(dx, dz)
  const diff = Math.abs(angleDiff(rAngle, angleToTarget))
  return diff <= (fovDeg * Math.PI / 180) / 2
}

// --- Min Heap for A* ---
class MinHeap {
  constructor() { this.data = [] }
  get size() { return this.data.length }
  push(item, priority) {
    this.data.push({ item, priority })
    this._bubbleUp(this.data.length - 1)
  }
  pop() {
    if (!this.data.length) return null
    const top = this.data[0]
    const last = this.data.pop()
    if (this.data.length > 0) { this.data[0] = last; this._sinkDown(0) }
    return top.item
  }
  _bubbleUp(i) {
    while (i > 0) {
      const p = (i - 1) >> 1
      if (this.data[p].priority <= this.data[i].priority) break
      ;[this.data[p], this.data[i]] = [this.data[i], this.data[p]]
      i = p
    }
  }
  _sinkDown(i) {
    const n = this.data.length
    while (true) {
      let smallest = i, l = 2 * i + 1, r = 2 * i + 2
      if (l < n && this.data[l].priority < this.data[smallest].priority) smallest = l
      if (r < n && this.data[r].priority < this.data[smallest].priority) smallest = r
      if (smallest === i) break
      ;[this.data[smallest], this.data[i]] = [this.data[i], this.data[smallest]]
      i = smallest
    }
  }
}

// --- Occupancy Grid ---
function createOccupancyGrid(venueW, venueH, obstacles, resolution, walls) {
  const gw = Math.ceil(venueW / resolution)
  const gh = Math.ceil(venueH / resolution)
  const grid = new Uint8Array(gw * gh) // 0=free, 1=obstacle, 2=inflated

  // Helper to mark a rectangular region with inflation
  function markRect(cx, cz, hw, hd) {
    const minGx = Math.floor((cx - hw - INFLATION_RADIUS) / resolution)
    const maxGx = Math.ceil((cx + hw + INFLATION_RADIUS) / resolution)
    const minGz = Math.floor((cz - hd - INFLATION_RADIUS) / resolution)
    const maxGz = Math.ceil((cz + hd + INFLATION_RADIUS) / resolution)
    for (let gz = Math.max(0, minGz); gz < Math.min(gh, maxGz); gz++) {
      for (let gx = Math.max(0, minGx); gx < Math.min(gw, maxGx); gx++) {
        const wx = gx * resolution, wz = gz * resolution
        const insideObs = wx >= cx - hw && wx <= cx + hw && wz >= cz - hd && wz <= cz + hd
        if (insideObs) grid[gz * gw + gx] = 1
        else if (grid[gz * gw + gx] === 0) grid[gz * gw + gx] = 2
      }
    }
  }

  for (const obs of obstacles) {
    const hw = (obs.width || 0.45) / 2
    const hd = (obs.depth || 0.45) / 2
    markRect(obs.x, obs.z, hw, hd)
  }

  // Interior wall segments
  if (walls) {
    for (const w of walls) {
      const cx = (w.x1 + w.x2) / 2, cz = (w.z1 + w.z2) / 2
      const hw = Math.abs(w.x2 - w.x1) / 2 + (w.thickness || 0.15) / 2
      const hd = Math.abs(w.z2 - w.z1) / 2 + (w.thickness || 0.15) / 2
      markRect(cx, cz, Math.max(hw, 0.05), Math.max(hd, 0.05))
    }
  }

  // Mark perimeter walls as inflated
  for (let gz = 0; gz < gh; gz++) {
    for (let gx = 0; gx < gw; gx++) {
      const wx = gx * resolution, wz = gz * resolution
      if (wx < INFLATION_RADIUS || wx > venueW - INFLATION_RADIUS ||
          wz < INFLATION_RADIUS || wz > venueH - INFLATION_RADIUS) {
        if (grid[gz * gw + gx] === 0) grid[gz * gw + gx] = 2
      }
    }
  }
  return { grid, gw, gh, resolution }
}

function worldToGrid(x, z, resolution) {
  return { gx: Math.round(x / resolution), gz: Math.round(z / resolution) }
}
function gridToWorld(gx, gz, resolution) {
  return { x: gx * resolution, z: gz * resolution }
}

// Check if a world position is reachable (not inside obstacle or inflation zone)
function isPositionReachable(x, z, obstacles, venueW, venueH, walls) {
  // Quick bounds check
  if (x < 0.5 || x > venueW - 0.5 || z < 0.5 || z > venueH - 0.5) return false
  // Build a temporary grid and check
  const grid = createOccupancyGrid(venueW, venueH, obstacles, GRID_RESOLUTION, walls || [])
  const g = worldToGrid(x, z, GRID_RESOLUTION)
  if (g.gx < 0 || g.gx >= grid.gw || g.gz < 0 || g.gz >= grid.gh) return false
  return grid.grid[g.gz * grid.gw + g.gx] === 0
}

// Place waste at (x,z) only if reachable; retry up to maxRetries times with random offsets
function placeWasteItem(wt, x, z, obstacles, venueW, venueH, walls, maxRetries = 10) {
  for (let attempt = 0; attempt <= maxRetries; attempt++) {
    const px = attempt === 0 ? x : x + (Math.random() - 0.5) * 2
    const pz = attempt === 0 ? z : z + (Math.random() - 0.5) * 2
    const cx = clamp(px, 0.8, venueW - 0.8)
    const cz = clamp(pz, 0.8, venueH - 0.8)
    // Quick check: not inside any obstacle's raw bounds + inflation margin
    const blocked = obstacles.some(o => {
      const hw = ((o.width || 0.5) / 2) + INFLATION_RADIUS + 0.1
      const hd = ((o.depth || 0.5) / 2) + INFLATION_RADIUS + 0.1
      return Math.abs(cx - o.x) < hw && Math.abs(cz - o.z) < hd
    })
    if (!blocked) {
      // Also check walls
      const nearWall = (walls || []).some(w => {
        const isVert = Math.abs(w.x1 - w.x2) < 0.1
        if (isVert) return Math.abs(cx - w.x1) < (w.thickness || 0.15) + INFLATION_RADIUS + 0.1 && cz >= Math.min(w.z1, w.z2) - 0.1 && cz <= Math.max(w.z1, w.z2) + 0.1
        return Math.abs(cz - w.z1) < (w.thickness || 0.15) + INFLATION_RADIUS + 0.1 && cx >= Math.min(w.x1, w.x2) - 0.1 && cx <= Math.max(w.x1, w.x2) + 0.1
      })
      if (!nearWall) return { id: nextId(), ...wt, x: cx, z: cz, collected: false, detected: false, confidence: 0, picking: false }
    }
  }
  // Last resort: try wider random positions to find a free spot
  for (let i = 0; i < 20; i++) {
    const rx = 1.5 + Math.random() * (venueW - 3)
    const rz = 1.5 + Math.random() * (venueH - 3)
    const blocked = obstacles.some(o => {
      const hw = ((o.width || 0.5) / 2) + INFLATION_RADIUS + 0.1
      const hd = ((o.depth || 0.5) / 2) + INFLATION_RADIUS + 0.1
      return Math.abs(rx - o.x) < hw && Math.abs(rz - o.z) < hd
    })
    if (!blocked) return { id: nextId(), ...wt, x: rx, z: rz, collected: false, detected: false, confidence: 0, picking: false }
  }
  return { id: nextId(), ...wt, x: clamp(x, 1.5, venueW - 1.5), z: clamp(z, 1.5, venueH - 1.5), collected: false, detected: false, confidence: 0, picking: false }
}

// --- A* Search ---
function aStarSearch(occGrid, startX, startZ, goalX, goalZ) {
  const { grid, gw, gh, resolution } = occGrid
  const s = worldToGrid(startX, startZ, resolution)
  const g = worldToGrid(goalX, goalZ, resolution)
  const sx = clamp(s.gx, 0, gw - 1), sz = clamp(s.gz, 0, gh - 1)
  const gx = clamp(g.gx, 0, gw - 1), gz = clamp(g.gz, 0, gh - 1)
  if (grid[gz * gw + gx] === 1) return null // goal is in obstacle

  const SQRT2 = 1.414
  const dirs = [[-1,0,1],[1,0,1],[0,-1,1],[0,1,1],[-1,-1,SQRT2],[1,-1,SQRT2],[-1,1,SQRT2],[1,1,SQRT2]]
  const gScore = new Float32Array(gw * gh).fill(Infinity)
  const cameFrom = new Int32Array(gw * gh).fill(-1)
  const closed = new Uint8Array(gw * gh)

  const startIdx = sz * gw + sx
  gScore[startIdx] = 0
  const heap = new MinHeap()
  heap.push(startIdx, 0)
  const maxIter = gw * gh
  let iter = 0

  while (heap.size > 0 && iter++ < maxIter) {
    const cur = heap.pop()
    if (closed[cur]) continue
    closed[cur] = 1
    const cx = cur % gw, cz = (cur / gw) | 0
    if (cx === gx && cz === gz) {
      // Reconstruct path
      const path = []
      let idx = cur
      while (idx !== -1) {
        const px = idx % gw, pz = (idx / gw) | 0
        const w = gridToWorld(px, pz, resolution)
        path.unshift(w)
        idx = cameFrom[idx]
      }
      return smoothPath(path, occGrid)
    }
    for (const [dx, dz, cost] of dirs) {
      const nx = cx + dx, nz = cz + dz
      if (nx < 0 || nx >= gw || nz < 0 || nz >= gh) continue
      const nIdx = nz * gw + nx
      if (closed[nIdx] || grid[nIdx] === 1) continue
      const cellCost = grid[nIdx] === 2 ? cost * 3 : cost // inflated cells have higher cost (reduced from 5x for narrower gaps)
      const ng = gScore[cur] + cellCost
      if (ng < gScore[nIdx]) {
        gScore[nIdx] = ng
        cameFrom[nIdx] = cur
        const h = Math.max(Math.abs(nx - gx), Math.abs(nz - gz)) * 1 +
                  (Math.abs(nx - gx) + Math.abs(nz - gz)) * 0.414
        heap.push(nIdx, ng + h)
      }
    }
  }
  return null // no path
}

function smoothPath(path, occGrid) {
  if (!path || path.length <= 2) return path
  const result = [path[0]]
  let i = 0
  while (i < path.length - 1) {
    let farthest = i + 1
    for (let j = path.length - 1; j > i + 1; j--) {
      if (lineOfSight(path[i], path[j], occGrid)) { farthest = j; break }
    }
    result.push(path[farthest])
    i = farthest
  }
  return result
}

function lineOfSight(a, b, occGrid) {
  const { grid, gw, gh, resolution } = occGrid
  const dx = b.x - a.x, dz = b.z - a.z
  const dist = Math.sqrt(dx * dx + dz * dz)
  const steps = Math.ceil(dist / (resolution * 0.5))
  for (let s = 0; s <= steps; s++) {
    const t = s / steps
    const wx = a.x + dx * t, wz = a.z + dz * t
    const g = worldToGrid(wx, wz, resolution)
    if (g.gx < 0 || g.gx >= gw || g.gz < 0 || g.gz >= gh) return false
    const cell = grid[g.gz * gw + g.gx]
    if (cell === 1) return false
  }
  return true
}

// --- Sector Division (legacy, kept for reference) ---
function divideSectors(venueW, venueH) {
  const sectorSize = Math.max(4, Math.min(8, Math.sqrt(venueW * venueH / 6)))
  const cols = Math.max(1, Math.ceil(venueW / sectorSize))
  const rows = Math.max(1, Math.ceil(venueH / sectorSize))
  const sw = venueW / cols, sh = venueH / rows
  const sectors = []
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      sectors.push({
        id: r * cols + c,
        x: c * sw, z: r * sh,
        width: sw, height: sh,
        cx: c * sw + sw / 2, cz: r * sh + sh / 2,
        completed: false,
      })
    }
  }
  return sectors
}

// Find the best unseen target position by scanning the fog grid for the largest dark cluster
function findBestUnseenTarget(sim, venue) {
  if (!sim.fogGrid || !sim.fogW || !sim.fogH) return null
  const res = sim.fogResolution || 0.5
  const fw = sim.fogW, fh = sim.fogH
  const fogCov = getFogCoverage(sim)

  // Strategy: directly scan the fog grid for unseen cells, find nearest cluster center
  // At high coverage (>95%), look at individual fog cells, not coarse grid
  if (fogCov > 0.95) {
    // Find the nearest unseen fog cell that's reachable (not inside an obstacle)
    let bestX = null, bestZ = null, bestDist = Infinity
    for (let fz = 0; fz < fh; fz++) {
      for (let fx = 0; fx < fw; fx++) {
        if (sim.fogGrid[fz * fw + fx]) continue // already seen
        const wx = fx * res + res / 2
        const wz = fz * res + res / 2
        // Skip cells inside obstacles
        const gx = Math.floor(wx / GRID_RESOLUTION)
        const gz = Math.floor(wz / GRID_RESOLUTION)
        if (sim.occGrid && gx >= 0 && gx < sim.occGrid.gw && gz >= 0 && gz < sim.occGrid.gh && sim.occGrid.grid[gz * sim.occGrid.gw + gx] === 1) continue
        const dist = Math.sqrt((wx - sim.robot.x) ** 2 + (wz - sim.robot.z) ** 2)
        if (dist < 0.3) continue // Don't target where we already are
        if (dist < bestDist) { bestDist = dist; bestX = wx; bestZ = wz }
      }
    }
    if (bestX !== null) return { x: bestX, z: bestZ }
    return null
  }

  // Normal mode: coarse grid scan for large unseen patches
  const coarseRes = 2.0 // Smaller cells for better precision
  const cw = Math.ceil(venue.width / coarseRes)
  const ch = Math.ceil(venue.height / coarseRes)
  let bestX = null, bestZ = null, bestScore = -1
  for (let cz = 0; cz < ch; cz++) {
    for (let cx = 0; cx < cw; cx++) {
      const wx = cx * coarseRes + coarseRes / 2
      const wz = cz * coarseRes + coarseRes / 2
      let unseen = 0, total = 0
      const fxMin = Math.max(0, Math.floor(cx * coarseRes / res))
      const fxMax = Math.min(fw - 1, Math.ceil((cx + 1) * coarseRes / res))
      const fzMin = Math.max(0, Math.floor(cz * coarseRes / res))
      const fzMax = Math.min(fh - 1, Math.ceil((cz + 1) * coarseRes / res))
      for (let fz = fzMin; fz <= fzMax; fz++) {
        for (let fx = fxMin; fx <= fxMax; fx++) {
          total++
          if (!sim.fogGrid[fz * fw + fx]) unseen++
        }
      }
      if (unseen === 0) continue
      const dist = Math.sqrt((wx - sim.robot.x) ** 2 + (wz - sim.robot.z) ** 2)
      if (dist < 0.5) continue
      // Score: prefer nearby unseen clusters
      const score = unseen * 10 - dist * 3
      if (score > bestScore) { bestScore = score; bestX = wx; bestZ = wz }
    }
  }
  if (bestX === null) return null
  return { x: bestX, z: bestZ }
}

// Calculate fog coverage percentage
function getFogCoverage(sim) {
  if (!sim.fogGrid) return 0
  let seen = 0
  for (let i = 0; i < sim.fogGrid.length; i++) {
    if (sim.fogGrid[i]) seen++
  }
  return seen / sim.fogGrid.length
}

// Get direction toward largest unseen area (for Random Walk bias)
function getUnseenDirection(sim, venue) {
  // Sample 8 directions, pick the one with most unseen cells along that ray
  const r = sim.robot
  let bestAngle = Math.random() * Math.PI * 2, bestUnseen = 0
  const res = sim.fogResolution || 0.5
  const fw = sim.fogW || 1, fh = sim.fogH || 1
  if (!sim.fogGrid) return bestAngle
  for (let i = 0; i < 8; i++) {
    const angle = (i / 8) * Math.PI * 2
    let unseen = 0
    // Cast ray in this direction, count unseen cells
    for (let d = 1; d <= 6; d++) {
      const sx = r.x + Math.sin(angle) * d
      const sz = r.z + Math.cos(angle) * d
      const gx = Math.floor(sx / res), gz = Math.floor(sz / res)
      if (gx < 0 || gx >= fw || gz < 0 || gz >= fh) break
      if (!sim.fogGrid[gz * fw + gx]) unseen++
    }
    if (unseen > bestUnseen) { bestUnseen = unseen; bestAngle = angle }
  }
  return bestAngle
}

// (getNextSector removed — replaced by findInfoGainViewpoint)

// --- Boustrophedon ---
function generateBoustrophedonPath(sector, margin = 0.5, spacing = BOUSTROPHEDON_SPACING) {
  const path = []
  const x0 = sector.x + margin, x1 = sector.x + sector.width - margin
  const z0 = sector.z + margin, z1 = sector.z + sector.height - margin
  if (x1 <= x0 || z1 <= z0) return [{ x: sector.cx, z: sector.cz }]
  const lanes = []
  let z = z0
  while (z <= z1) {
    lanes.push(z)
    z += spacing
  }
  if (lanes.length === 0) lanes.push((z0 + z1) / 2)
  // Add intermediate waypoints every 2m along each lane for smoother tracking
  const wpSpacing = 2.0
  for (let i = 0; i < lanes.length; i++) {
    const startX = i % 2 === 0 ? x0 : x1
    const endX = i % 2 === 0 ? x1 : x0
    const dir = endX > startX ? 1 : -1
    let cx = startX
    while (dir > 0 ? cx < endX : cx > endX) {
      path.push({ x: cx, z: lanes[i] })
      cx += dir * wpSpacing
    }
    path.push({ x: endX, z: lanes[i] })
  }
  return path
}

// (Clarke-Wright CVRP removed — replaced by NN+2-opt for single vehicle)

// --- Held-Karp Exact TSP with A*-precomputed distances ---
// Optimal for n<=15 items. O(n^2 * 2^n) time, ~5-20ms for n=15.
function heldKarpTSP(distMatrix, n) {
  // distMatrix[i][j] = distance from node i to node j
  // Node 0 = robot (start), nodes 1..n = items
  // Returns: optimal tour order (indices 1..n) starting from 0
  if (n === 0) return { tour: [], totalDist: 0 }
  if (n === 1) return { tour: [1], totalDist: distMatrix[0][1] }

  const FULL = (1 << n) - 1
  // dp[mask][i] = min cost to visit all items in mask, ending at item i
  // Items are numbered 0..n-1 in the mask, but 1..n in distMatrix
  const dp = new Array(1 << n)
  const parent = new Array(1 << n)
  for (let m = 0; m < (1 << n); m++) {
    dp[m] = new Float64Array(n).fill(Infinity)
    parent[m] = new Int8Array(n).fill(-1)
  }

  // Base: starting from robot (node 0) to each item
  for (let i = 0; i < n; i++) {
    dp[1 << i][i] = distMatrix[0][i + 1]
  }

  // Fill DP
  for (let mask = 1; mask <= FULL; mask++) {
    for (let last = 0; last < n; last++) {
      if (!(mask & (1 << last))) continue
      if (dp[mask][last] === Infinity) continue
      for (let next = 0; next < n; next++) {
        if (mask & (1 << next)) continue
        const newMask = mask | (1 << next)
        const newCost = dp[mask][last] + distMatrix[last + 1][next + 1]
        if (newCost < dp[newMask][next]) {
          dp[newMask][next] = newCost
          parent[newMask][next] = last
        }
      }
    }
  }

  // Find best ending node
  let bestEnd = 0, bestCost = Infinity
  for (let i = 0; i < n; i++) {
    if (dp[FULL][i] < bestCost) {
      bestCost = dp[FULL][i]
      bestEnd = i
    }
  }

  // If no valid tour exists (all items unreachable), return greedy Euclidean order
  if (bestCost === Infinity) {
    const tour = Array.from({ length: n }, (_, i) => i + 1)
    return { tour, totalDist: Infinity }
  }

  // Reconstruct tour
  const tour = []
  let mask = FULL, cur = bestEnd
  while (mask > 0 && cur >= 0) {
    tour.unshift(cur + 1) // convert to distMatrix index
    const prev = parent[mask][cur]
    mask ^= (1 << cur)
    cur = prev
  }

  return { tour, totalDist: bestCost }
}

// Pre-compute A* distance matrix for Held-Karp
function computeAStarDistMatrix(robotPos, items, occGrid) {
  const nodes = [robotPos, ...items] // node 0 = robot, 1..n = items
  const n = nodes.length
  const matrix = Array.from({ length: n }, () => new Float64Array(n).fill(Infinity))

  for (let i = 0; i < n; i++) {
    matrix[i][i] = 0
    for (let j = i + 1; j < n; j++) {
      const path = aStarSearch(occGrid, nodes[i].x, nodes[i].z, nodes[j].x, nodes[j].z)
      let d = Infinity
      if (path && path.length > 1) {
        d = 0
        for (let k = 1; k < path.length; k++) d += distXZ(path[k - 1], path[k])
      }
      matrix[i][j] = d
      matrix[j][i] = d
    }
  }
  return matrix
}

// --- NN + 2-opt Route Optimizer (replaces Held-Karp + Clarke-Wright) ---
function nnTwoOptRoute(robotPos, items) {
  if (items.length === 0) return { orderedItems: [], totalDist: 0 }
  if (items.length === 1) return { orderedItems: [...items], totalDist: distXZ(robotPos, items[0]) }

  // Step 1: NN greedy tour
  const visited = new Set()
  const order = []
  let pos = robotPos
  for (let i = 0; i < items.length; i++) {
    let bestIdx = -1, bestDist = Infinity
    for (let j = 0; j < items.length; j++) {
      if (visited.has(j)) continue
      const d = distXZ(pos, items[j])
      if (d < bestDist) { bestDist = d; bestIdx = j }
    }
    if (bestIdx < 0) break
    visited.add(bestIdx)
    order.push(bestIdx)
    pos = items[bestIdx]
  }

  // Step 2: 2-opt improvement
  if (order.length >= 4) {
    let improved = true, iters = 0
    while (improved && iters < 50) {
      improved = false; iters++
      for (let i = 0; i < order.length - 1; i++) {
        for (let j = i + 2; j < order.length; j++) {
          const a = i === 0 ? robotPos : items[order[i - 1]]
          const b = items[order[i]], c = items[order[j]]
          const d = j + 1 < order.length ? items[order[j + 1]] : null
          const oldDist = distXZ(a, b) + (d ? distXZ(c, d) : 0)
          const newDist = distXZ(a, c) + (d ? distXZ(b, d) : 0)
          if (newDist < oldDist - 0.01) {
            const seg = order.slice(i, j + 1).reverse()
            order.splice(i, j - i + 1, ...seg)
            improved = true
          }
        }
      }
    }
  }

  const orderedItems = order.map(idx => items[idx])
  let totalDist = distXZ(robotPos, orderedItems[0])
  for (let i = 1; i < orderedItems.length; i++) totalDist += distXZ(orderedItems[i - 1], orderedItems[i])
  const naiveDist = computeNaiveDistance(robotPos, items)
  return { orderedItems, totalDist, naiveDist }
}

// --- Info-Gain Viewpoint Scoring ---
function findInfoGainViewpoint(sim, venue, minDist = 1.5) {
  if (!sim.fogGrid) return null
  const r = sim.robot
  const res = sim.fogResolution
  const fw = sim.fogW, fh = sim.fogH
  const coarseRes = 1.5
  const cw = Math.ceil(venue.width / coarseRes)
  const ch = Math.ceil(venue.height / coarseRes)
  let bestPos = null, bestScore = -Infinity, bestAngle = 0

  for (let cz = 0; cz < ch; cz++) {
    for (let cx = 0; cx < cw; cx++) {
      const wx = cx * coarseRes + coarseRes / 2
      const wz = cz * coarseRes + coarseRes / 2
      // Skip if inside obstacle
      if (sim.occGrid) {
        const gx = Math.floor(wx / sim.occGrid.resolution)
        const gz = Math.floor(wz / sim.occGrid.resolution)
        if (gx >= 0 && gx < sim.occGrid.gw && gz >= 0 && gz < sim.occGrid.gh
            && sim.occGrid.grid[gz * sim.occGrid.gw + gx] >= 1) continue
      }
      const travelDist = distXZ(r, { x: wx, z: wz })
      if (travelDist < minDist) continue

      // Count unseen cells within camera range (full circle — robot will scan on arrival)
      let unseenVisible = 0, unseenCx = 0, unseenCz = 0
      const scanR = CAMERA_MAX_RANGE
      const minFx = Math.max(0, Math.floor((wx - scanR) / res))
      const maxFx = Math.min(fw - 1, Math.ceil((wx + scanR) / res))
      const minFz = Math.max(0, Math.floor((wz - scanR) / res))
      const maxFz = Math.min(fh - 1, Math.ceil((wz + scanR) / res))
      for (let fz = minFz; fz <= maxFz; fz++) {
        for (let fx = minFx; fx <= maxFx; fx++) {
          if (sim.fogGrid[fz * fw + fx]) continue
          const cellX = fx * res + res / 2, cellZ = fz * res + res / 2
          const d = Math.sqrt((cellX - wx) ** 2 + (cellZ - wz) ** 2)
          if (d >= CAMERA_MIN_RANGE && d <= CAMERA_MAX_RANGE) {
            unseenVisible++
            unseenCx += cellX
            unseenCz += cellZ
          }
        }
      }
      if (unseenVisible === 0) continue

      // Face angle: toward the centroid of unseen cells near this viewpoint
      unseenCx /= unseenVisible
      unseenCz /= unseenVisible
      const faceAngle = Math.atan2(unseenCx - wx, unseenCz - wz)

      // Penalize revisits
      let visitPenalty = 1.0
      if (sim.visitGrid) {
        let maxVisits = 0
        const vgx = Math.floor(wx / sim.visitGrid.res)
        const vgz = Math.floor(wz / sim.visitGrid.res)
        for (let dz = -1; dz <= 1; dz++) {
          for (let dx = -1; dx <= 1; dx++) {
            const nx = vgx + dx, nz = vgz + dz
            if (nx >= 0 && nx < sim.visitGrid.vw && nz >= 0 && nz < sim.visitGrid.vh) {
              maxVisits = Math.max(maxVisits, sim.visitGrid.grid[nz * sim.visitGrid.vw + nx])
            }
          }
        }
        visitPenalty = 1.0 / (1 + maxVisits * 3)
      }
      const score = unseenVisible * visitPenalty / (travelDist + 1.0)
      if (score > bestScore) { bestScore = score; bestPos = { x: wx, z: wz }; bestAngle = faceAngle }
    }
  }
  if (bestPos) sim._infoGainTargetAngle = bestAngle
  return bestPos
}

// --- Unscanned Patch Finder (for RECLAIM SWEEP mode) ---
function findUnscannedPatches(sim, minPatchSize) {
  if (!sim.fogGrid) return []
  const res = sim.fogResolution
  const fw = sim.fogW, fh = sim.fogH
  const visited = new Uint8Array(fw * fh)
  const patches = []

  for (let fz = 0; fz < fh; fz++) {
    for (let fx = 0; fx < fw; fx++) {
      if (sim.fogGrid[fz * fw + fx] || visited[fz * fw + fx]) continue
      const queue = [{ fx, fz }]
      visited[fz * fw + fx] = 1
      let minX = fx, maxX = fx, minZ = fz, maxZ = fz, count = 0
      while (queue.length > 0) {
        const { fx: cx, fz: cz } = queue.shift()
        count++
        for (const [dx, dz] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
          const nx = cx + dx, nz = cz + dz
          if (nx < 0 || nx >= fw || nz < 0 || nz >= fh) continue
          if (sim.fogGrid[nz * fw + nx] || visited[nz * fw + nx]) continue
          visited[nz * fw + nx] = 1
          queue.push({ fx: nx, fz: nz })
          minX = Math.min(minX, nx); maxX = Math.max(maxX, nx)
          minZ = Math.min(minZ, nz); maxZ = Math.max(maxZ, nz)
        }
      }
      const pw = (maxX - minX + 1) * res
      const ph = (maxZ - minZ + 1) * res
      if (pw * ph >= (minPatchSize || 1)) {
        patches.push({
          x: minX * res, z: minZ * res,
          width: pw, height: ph,
          cx: (minX + maxX) / 2 * res + res / 2,
          cz: (minZ + maxZ) / 2 * res + res / 2,
          area: count * res * res,
        })
      }
    }
  }
  return patches.sort((a, b) => b.area - a.area)
}

// --- Visit-Frequency Grid (for path overlap metric) ---
function initVisitGrid(venueW, venueH) {
  const vw = Math.ceil(venueW / VISIT_GRID_RESOLUTION)
  const vh = Math.ceil(venueH / VISIT_GRID_RESOLUTION)
  return { grid: new Uint16Array(vw * vh), vw, vh, res: VISIT_GRID_RESOLUTION }
}

function updateVisitGrid(visitGrid, x, z) {
  if (!visitGrid) return
  const gx = Math.floor(x / visitGrid.res)
  const gz = Math.floor(z / visitGrid.res)
  if (gx >= 0 && gx < visitGrid.vw && gz >= 0 && gz < visitGrid.vh) {
    visitGrid.grid[gz * visitGrid.vw + gx]++
  }
}

function computePathOverlap(visitGrid) {
  if (!visitGrid) return 0
  let visited = 0, revisited = 0
  for (let i = 0; i < visitGrid.grid.length; i++) {
    if (visitGrid.grid[i] > 0) visited++
    if (visitGrid.grid[i] > 1) revisited++
  }
  return visited > 0 ? revisited / visited : 0
}

// --- Advanced Metrics Computation ---
function computeAdvancedMetrics(sim) {
  const itemsPerMeter = sim.stats.distanceTraveled > 0
    ? sim.stats.itemsCollected / sim.stats.distanceTraveled : 0
  const pathOverlap = computePathOverlap(sim.visitGrid)
  const avgDumpFullness = sim.dumpTripFullness && sim.dumpTripFullness.length > 0
    ? sim.dumpTripFullness.reduce((a, b) => a + b, 0) / sim.dumpTripFullness.length / (BIN_CAPACITY * 3) : 0
  return {
    itemsPerMeter: itemsPerMeter.toFixed(3),
    pathOverlapPct: (pathOverlap * 100).toFixed(1),
    deadheadDist: (sim.deadheadDist || 0).toFixed(1),
    avgDumpFullness: (avgDumpFullness * 100).toFixed(0),
    timeTo90: sim.timeTo90 != null ? sim.timeTo90.toFixed(0) : null,
    timeToLast: sim.timeToLast != null ? sim.timeToLast.toFixed(0) : null,
  }
}

// --- Collection Queue: rotate failed item to back instead of dropping ---
function rotateOrDropItem(sim) {
  if (sim.collectionQueue.length === 0) return
  const item = sim.collectionQueue.shift()
  item._failCount = (item._failCount || 0) + 1
  if (item._failCount < 8) {
    sim.collectionQueue.push(item)
  } else {
    // Mark as unreachable so it won't be re-queued
    const wasteItem = sim.wasteItems?.find(w => w.id === item.id)
    if (wasteItem) wasteItem._unreachable = true
  }
  sim._consecutiveQueueFails = (sim._consecutiveQueueFails || 0) + 1
  if (sim._consecutiveQueueFails > Math.max(30, (sim.collectionQueue.length + 1) * 5)) {
    // Drop items with many failures and mark unreachable
    const dropped = sim.collectionQueue.filter(q => (q._failCount || 0) >= 4)
    for (const d of dropped) {
      const w = sim.wasteItems?.find(wi => wi.id === d.id)
      if (w) w._unreachable = true
    }
    sim.collectionQueue = sim.collectionQueue.filter(q => (q._failCount || 0) < 4)
    sim._consecutiveQueueFails = 0
  }
}

// --- Debug Logging ---
function logDebug(sim, type, msg) {
  if (!sim.debugLog) return
  const t = sim.stats.elapsedTime.toFixed(1)
  sim.debugLog.push({ time: t, type, message: msg })
  if (sim.debugLog.length > 200) sim.debugLog.shift()
  if (type === 'ANOMALY') console.warn(`[SIM ${t}s] ${msg}`)
}

function computeNaiveDistance(depot, items) {
  if (items.length === 0) return 0
  const dist = (a, b) => Math.sqrt((a.x - b.x) ** 2 + (a.z - b.z) ** 2)
  const visited = new Set()
  let pos = depot, totalDist = 0
  const bins = { recyclable: 0, compost: 0, landfill: 0 }
  for (let count = 0; count < items.length; count++) {
    let bestIdx = -1, bestDist = Infinity
    for (let i = 0; i < items.length; i++) {
      if (visited.has(i)) continue
      const d = dist(pos, items[i])
      if (d < bestDist) { bestDist = d; bestIdx = i }
    }
    if (bestIdx === -1) break
    visited.add(bestIdx)
    totalDist += bestDist
    pos = items[bestIdx]
    bins[items[bestIdx].bin]++
    if (bins.recyclable >= BIN_CAPACITY * DUMP_THRESHOLD ||
        bins.compost >= BIN_CAPACITY * DUMP_THRESHOLD ||
        bins.landfill >= BIN_CAPACITY * DUMP_THRESHOLD) {
      totalDist += dist(pos, depot)
      pos = depot
      bins.recyclable = 0; bins.compost = 0; bins.landfill = 0
    }
  }
  totalDist += dist(pos, depot)
  return totalDist
}

// ═══════════════════════════════════════════════════════════════
// SECTION 2: STATE MANAGEMENT
// ═══════════════════════════════════════════════════════════════

const initialState = {
  phase: 'SETUP', // SETUP | RUNNING | PAUSED | COMPLETE
  venue: { width: 20, height: 15 },
  walls: [], // Interior wall segments: { x1, z1, x2, z2, thickness }
  obstacles: [],
  wasteItems: [],
  robotStart: { x: 1.5, z: 1.5, angle: 0 },
  dumpStation: { x: 10, z: 0.8 },
  // Simulation
  robot: { x: 1.5, z: 1.5, angle: 0, linVel: 0, angVel: 0, state: 'IDLE', armState: 'stowed', pickupTimer: 0, dumpTimer: 0, targetWaste: null },
  bins: { recyclable: 0, compost: 0, landfill: 0 },
  sectors: [],
  activeSectorIdx: -1,
  coveragePath: [],
  coverageIdx: 0,
  currentPath: [],
  pathIdx: 0,
  detectedWaste: [], // ids
  collectionQueue: [], // waste items to collect
  cvrpResult: null,
  cvrpCumulative: { totalNaive: 0, totalOptimized: 0, batchCount: 0 },
  stats: { distanceTraveled: 0, itemsCollected: 0, dumpTrips: 0, elapsedTime: 0, battery: 100 },
  speedMultiplier: 1,
  paused: false,
  showHeatmap: false,
  showFog: false,
  showCostmap: false,
  showLidar: true,
  showFOV: true,
  showPath: true,
  showSectors: true,
  showDebug: false,
  particles: [],
  placementMode: null, // null | wasteTypeId | 'table' | 'chair'
  returnPoint: null,
  algorithm: 'reclaim',
  smartBinBalancing: true,
  runHistory: [],
}

function reducer(state, action) {
  switch (action.type) {
    case 'SET_VENUE': return { ...state, venue: action.venue, dumpStation: { x: action.venue.width / 2, z: 0.8 } }
    case 'SET_PLACEMENT_MODE': return { ...state, placementMode: action.mode }
    case 'ADD_WASTE': return { ...state, wasteItems: [...state.wasteItems, action.item] }
    case 'ADD_OBSTACLE': return { ...state, obstacles: [...state.obstacles, action.obstacle] }
    case 'REMOVE_ITEM': return {
      ...state,
      wasteItems: state.wasteItems.filter(w => w.id !== action.id),
      obstacles: state.obstacles.filter(o => o.id !== action.id),
    }
    case 'MOVE_ITEM': {
      const waste = state.wasteItems.map(w => w.id === action.id ? { ...w, x: action.x, z: action.z } : w)
      const obs = state.obstacles.map(o => o.id === action.id ? { ...o, x: action.x, z: action.z } : o)
      return { ...state, wasteItems: waste, obstacles: obs }
    }
    case 'SET_ROBOT_START': return { ...state, robotStart: { ...state.robotStart, x: action.x, z: action.z } }
    case 'SET_DUMP_STATION': return { ...state, dumpStation: { x: action.x, z: action.z } }
    case 'LOAD_PRESET': return { ...state, ...action.payload }
    case 'CLEAR_ALL': return { ...state, wasteItems: [], obstacles: [], walls: [] }
    case 'SCATTER_RANDOM': return { ...state, wasteItems: [...state.wasteItems, ...action.items] }
    case 'START_SIM': {
      const sectors = divideSectors(state.venue.width, state.venue.height)
      return {
        ...state,
        phase: 'RUNNING',
        robot: { ...state.robot, x: state.robotStart.x, z: state.robotStart.z, angle: state.robotStart.angle, state: 'IDLE', linVel: 0, angVel: 0, armState: 'stowed', pickupTimer: 0, dumpTimer: 0, targetWaste: null },
        bins: { recyclable: 0, compost: 0, landfill: 0 },
        sectors,
        activeSectorIdx: -1,
        coveragePath: [],
        coverageIdx: 0,
        currentPath: [],
        pathIdx: 0,
        detectedWaste: [],
        collectionQueue: [],
        cvrpResult: null,
        stats: { distanceTraveled: 0, itemsCollected: 0, dumpTrips: 0, elapsedTime: 0, battery: 100 },
        particles: [],
        wasteItems: state.wasteItems.map(w => ({ ...w, collected: false, detected: false, confidence: 0, picking: false })),
        returnPoint: null,
        smartBinBalancing: state.algorithm === 'reclaim' && state.smartBinBalancing,
      }
    }
    case 'PAUSE': return { ...state, paused: !state.paused }
    case 'SET_SPEED': return { ...state, speedMultiplier: action.speed }
    case 'TOGGLE': return { ...state, [action.key]: !state[action.key] }
    case 'SYNC_SIM': {
      const p = action.payload
      const newState = { ...state, ...p }
      // Apply collectedIds to wasteItems
      if (p.collectedIds && !p.wasteItems) {
        const idSet = new Set(p.collectedIds)
        const changed = state.wasteItems.some(w => !w.collected && idSet.has(w.id))
        if (changed) {
          newState.wasteItems = state.wasteItems.map(w => idSet.has(w.id) ? { ...w, collected: true } : w)
        } else {
          newState.wasteItems = state.wasteItems
        }
      }
      return newState
    }
    case 'SET_ALGORITHM': return { ...state, algorithm: action.algorithm }
    case 'SET_BIN_BALANCING': return { ...state, smartBinBalancing: action.value }
    case 'ADD_RUN_HISTORY': return { ...state, runHistory: [...state.runHistory, action.run] }
    case 'DISMISS_RESULTS': return { ...state, resultsDismissed: true }
    case 'RESET': return { ...initialState, venue: state.venue, walls: state.walls, obstacles: state.obstacles, wasteItems: state.wasteItems.map(w => ({ ...w, collected: false, detected: false, confidence: 0, picking: false })), robotStart: state.robotStart, dumpStation: state.dumpStation, runHistory: state.runHistory, algorithm: state.algorithm, smartBinBalancing: state.smartBinBalancing }
    case 'RESET_FULL': return { ...initialState }
    default: return state
  }
}

// ═══════════════════════════════════════════════════════════════
// SECTION 3: SIMULATION LOOP
// ═══════════════════════════════════════════════════════════════

function SimulationEngine({ state, dispatch, simEngineRef }) {
  const internalRef = useRef(null)
  const simRef = simEngineRef || internalRef
  const accumRef = useRef(0)

  // Initialize sim state synchronously when phase transitions to RUNNING
  if (state.phase === 'RUNNING' && !simRef.current) {
    simRef.current = {
      robot: { ...state.robot },
      bins: { ...state.bins },
      sectors: state.sectors.map(s => ({ ...s })),
      activeSectorIdx: -1,
      coveragePath: [],
      coverageIdx: 0,
      currentPath: [],
      pathIdx: 0,
      wasteItems: state.wasteItems.map(w => ({ ...w })),
      detectedWaste: [],
      collectionQueue: [],
      stats: { ...state.stats },
      particles: [],
      cvrpResult: null,
      allDetectedItems: [], // cumulative list of all detected items for CVRP comparison
      occGrid: createOccupancyGrid(state.venue.width, state.venue.height, state.obstacles, GRID_RESOLUTION, state.walls),
      returnPoint: null,
      dumpStation: { ...state.dumpStation },
      stuckCounter: 0,
      lastPos: { x: 0, z: 0 },
      progressCheckpoint: null,
      algorithm: state.algorithm,
      smartBinBalancing: (state.algorithm === 'reclaim') && state.smartBinBalancing,
      deferredItems: [],           // items deferred due to bin balancing
      deferredRecoveries: 0,       // count of deferred recovery passes
      totalDeferred: 0,            // total items ever deferred
      deferredLabels: [],          // transient labels: { id, x, z, timer }
      pickupMarkers: [],            // flash markers at collection points
      fogGrid: null,                // fog of war grid — painted by FOV each step
      fogResolution: 0.5,           // 0.5m per cell
      randomWalkDir: Math.random() * Math.PI * 2,
      randomWalkTimer: 0,
      scanTimer: 0,
      scanAngle: 0,
      quadrantVisits: [0, 0, 0, 0],
      // Debug tools
      debugLog: [],
      debugAnomalies: { spins: 0, oscillations: 0, pathFailures: 0, stateStalls: 0 },
      spinTimer: 0,
      _lastStateChange: null,
      // New metrics + RECLAIM mode
      reclaimMode: 'SCAN',
      visitGrid: initVisitGrid(state.venue.width, state.venue.height),
      pathTrail: [],
      plannedRoute: [],
      deadheadDist: 0,
      dumpTripFullness: [],
      timeTo90: null,
      timeToLast: null,
      // Sweep state for RECLAIM
      _sweepPatches: [],
      _sweepPath: [],
      sweepIdx: 0,
      scanAngleAccum: 0,
    }
  }
  if (state.phase === 'SETUP') simRef.current = null

  // Read state props directly from closure — R3F re-renders update the closure each frame.
  // NEVER dispatch React state from inside useFrame; HUD syncing is done by the polling
  // interval in App that reads simRef.current every 500ms.
  useFrame((_, delta) => {
    if (!simRef.current || state.phase !== 'RUNNING' || state.paused) return
    const sim = simRef.current
    window.__sim = sim
    window.__simConstants = { CAMERA_FOV_DEG, CAMERA_MIN_RANGE, CAMERA_MAX_RANGE, ARM_REACH, MAX_LINEAR_VEL, MAX_ANGULAR_VEL, BIN_CAPACITY, DUMP_THRESHOLD }
    window.__stepSim = () => stepSimulationDispatch(sim, FIXED_DT, state.venue, state.obstacles)
    if (sim.robot.state === 'DONE') return // Stop simulation when complete
    const speed = state.speedMultiplier
    accumRef.current += delta * speed
    const maxSteps = 10
    let steps = 0
    while (accumRef.current >= FIXED_DT && steps++ < maxSteps) {
      accumRef.current -= FIXED_DT
      try {
        stepSimulationDispatch(sim, FIXED_DT, state.venue, state.obstacles)
      } catch (e) {
        console.error('Sim step error:', e)
        sim.robot.state = 'DONE'
        return
      }
    }
    if (accumRef.current > FIXED_DT * maxSteps) accumRef.current = 0
  })

  return null
}

// ═══════════════════════════════════════════════════════════════
// SECTION 3a: SHARED SUB-STATE HELPERS (used by alternative algorithms)
// ═══════════════════════════════════════════════════════════════

function stepPickingState(sim, dt) {
  const r = sim.robot
  r.pickupTimer -= dt
  if (r.pickupTimer > PICKUP_TIME * 0.65) r.armState = 'reaching'
  else if (r.pickupTimer > PICKUP_TIME * 0.4) r.armState = 'gripping'
  else if (r.pickupTimer > 0) r.armState = 'lifting'
  else {
    r.armState = 'stowed'
    const item = sim.wasteItems.find(w => w.id === r.targetWaste)
    if (item && !item.collected) {
      item.collected = true
      item.picking = false
      sim.bins[item.bin]++
      sim.stats.itemsCollected++
      sim._consecutiveQueueFails = 0 // reset on successful pickup
      sim.stats.battery -= BATTERY_DRAIN_PER_PICK
      const binColor = BIN_COLORS[item.bin]
      for (let i = 0; i < 20; i++) {
        const angle = Math.random() * Math.PI * 2
        const speed = 1 + Math.random() * 2
        sim.particles.push({
          id: nextId(), x: item.x, y: 0.1, z: item.z,
          vx: Math.cos(angle) * speed, vy: 2 + Math.random() * 2, vz: Math.sin(angle) * speed,
          life: 0.5, color: binColor,
        })
      }
      sim.collectionQueue = sim.collectionQueue.filter(q => q.id !== item.id)
      sim.detectedWaste = sim.detectedWaste.filter(id => id !== item.id)
      // Pickup flash marker
      if (!sim.pickupMarkers) sim.pickupMarkers = []
      sim.pickupMarkers.push({ x: item.x, z: item.z, timer: 1.5, color: binColor })
      logDebug(sim, 'PICKUP', `Collected ${item.label || item.bin} (#${sim.stats.itemsCollected}) bins: R=${sim.bins.recyclable} C=${sim.bins.compost} L=${sim.bins.landfill}`)
    }
    r.targetWaste = null
    r.state = 'POST_PICK' // Caller handles transition
  }
}

function stepNavigatingDumpShared(sim, dt) {
  const r = sim.robot
  if (sim.pathIdx >= sim.currentPath.length) {
    r.state = 'DUMPING'
    r.dumpTimer = DUMP_TIME
    return
  }
  const wp = sim.currentPath[sim.pathIdx]
  driveToward(r, wp.x, wp.z, dt, sim.occGrid)
  sim.stats.distanceTraveled += Math.abs(r.linVel) * dt
  sim.stats.battery -= Math.abs(r.linVel) * dt * BATTERY_DRAIN_PER_METER
  if (distXZ(r, wp) < 0.2) sim.pathIdx++
}

function stepDumpingState(sim, dt) {
  const r = sim.robot
  r.dumpTimer -= dt
  if (r.dumpTimer <= 0) {
    logDebug(sim, 'DUMP', `Dump trip #${sim.stats.dumpTrips + 1}: R=${sim.bins.recyclable} C=${sim.bins.compost} L=${sim.bins.landfill}`)
    sim.bins.recyclable = 0
    sim.bins.compost = 0
    sim.bins.landfill = 0
    sim.stats.dumpTrips++
    sim.stats.battery -= BATTERY_DRAIN_PER_DUMP
    r.state = 'POST_DUMP' // Caller handles transition
  }
}

// ═══════════════════════════════════════════════════════════════
// SECTION 3b: ALTERNATIVE ALGORITHM STEP FUNCTIONS
// ═══════════════════════════════════════════════════════════════

function planGreedyNavigation(sim) {
  if (sim.collectionQueue.length === 0) return
  sim.collectionQueue.sort((a, b) => distXZ(sim.robot, a) - distXZ(sim.robot, b))
  const target = sim.collectionQueue[0]
  const path = aStarSearch(sim.occGrid, sim.robot.x, sim.robot.z, target.x, target.z)
  if (path && path.length > 1) {
    sim.currentPath = path
    sim.pathIdx = 1
    sim.robot.state = 'NAVIGATING'
  } else {
    rotateOrDropItem(sim)
    if (sim.collectionQueue.length > 0) planGreedyNavigation(sim)
    else { sim.scanTimer = 0; sim.robot.state = 'SCANNING' }
  }
}

function stepRandomWalk(sim, dt, venue) {
  const r = sim.robot
  // Timeout after 10 min simulated
  if (sim.stats.elapsedTime >= RANDOM_WALK_TIMEOUT) {
    r.state = 'DONE'
    r.linVel = 0
    return
  }

  switch (r.state) {
    case 'IDLE':
      sim.randomWalkDir = r.angle
      r.state = 'COVERAGE' // repurpose as "walking"
      break
    case 'COVERAGE': {
      const margin = 0.8
      const nearWall = r.x < margin || r.x > venue.width - margin || r.z < margin || r.z > venue.height - margin
      const stuck = r.linVel < 0.02

      if (nearWall || stuck) {
        sim.randomWalkTimer += dt
        if (sim.randomWalkTimer > 0.15) {
          sim.randomWalkStuckCount = (sim.randomWalkStuckCount || 0) + 1
          // After 3 failed redirects, use A* to escape to a random open point
          if (sim.randomWalkStuckCount > 3) {
            const escX = clamp(venue.width * 0.2 + Math.random() * venue.width * 0.6, 1, venue.width - 1)
            const escZ = clamp(venue.height * 0.2 + Math.random() * venue.height * 0.6, 1, venue.height - 1)
            const path = aStarSearch(sim.occGrid, r.x, r.z, escX, escZ)
            if (path && path.length > 1) {
              sim.currentPath = path
              sim.pathIdx = 1
              r.state = 'NAVIGATING_SECTOR' // Reuse for escape navigation
              sim.randomWalkStuckCount = 0
              sim.randomWalkTimer = 0
              break
            }
          }
          // Try a random direction biased away from walls/obstacles
          const cx = venue.width / 2 + (Math.random() - 0.5) * venue.width * 0.6
          const cz = venue.height / 2 + (Math.random() - 0.5) * venue.height * 0.6
          sim.randomWalkDir = Math.atan2(cx - r.x, cz - r.z)
          // Add large random perturbation to escape obstacle corners
          sim.randomWalkDir += (Math.random() - 0.5) * Math.PI
          sim.randomWalkTimer = 0
          // Nudge away from venue walls
          if (r.x < 0.3) r.x = 0.5
          if (r.x > venue.width - 0.3) r.x = venue.width - 0.5
          if (r.z < 0.3) r.z = 0.5
          if (r.z > venue.height - 0.3) r.z = venue.height - 0.5
        }
      } else {
        sim.randomWalkTimer = 0
        sim.randomWalkStuckCount = 0
        // Lévy Flight: heavy-tailed step lengths — mostly short moves, occasionally long jumps
        // This is theoretically optimal for sparse random targets (Viswanathan et al, Nature 1999)
        // Bias toward unseen areas when redirecting
        if (Math.random() < 0.015) {
          // Lévy step: draw from power-law distribution (µ=1.5 for optimal foraging)
          const u = Math.random()
          const levyStep = Math.pow(u, -1 / 1.5) // Inverse CDF of Pareto distribution
          const stepDist = Math.min(levyStep * 2, Math.max(venue.width, venue.height) * 0.8)
          // Direction: bias toward unseen areas
          const baseDir = getUnseenDirection(sim, venue)
          sim.randomWalkDir = baseDir + (Math.random() - 0.5) * 0.8
          // Navigate to the Lévy target point via A* if it's a long step
          if (stepDist > 3) {
            const tx = clamp(r.x + Math.sin(sim.randomWalkDir) * stepDist, 1, venue.width - 1)
            const tz = clamp(r.z + Math.cos(sim.randomWalkDir) * stepDist, 1, venue.height - 1)
            const path = aStarSearch(sim.occGrid, r.x, r.z, tx, tz)
            if (path && path.length > 1) {
              sim.currentPath = path
              sim.pathIdx = 1
              r.state = 'NAVIGATING_SECTOR'
              break
            }
          }
        }
      }

      // Drive forward in current direction
      const tx = r.x + Math.sin(sim.randomWalkDir) * 5
      const tz = r.z + Math.cos(sim.randomWalkDir) * 5
      driveToward(r, tx, tz, dt, sim.occGrid)
      sim.stats.distanceTraveled += Math.abs(r.linVel) * dt
      sim.stats.battery -= Math.abs(r.linVel) * dt * BATTERY_DRAIN_PER_METER

      detectWaste(sim)

      // Opportunistic: only pick up if within arm reach (NO diversion)
      const nearby = findNearbyWaste(sim, ARM_REACH)
      if (nearby) {
        r.targetWaste = nearby.id
        r.state = 'PICKING'
        r.pickupTimer = PICKUP_TIME
        r.armState = 'reaching'
        break
      }

      // Dump check
      if (sim.bins.recyclable >= BIN_CAPACITY * DUMP_THRESHOLD ||
          sim.bins.compost >= BIN_CAPACITY * DUMP_THRESHOLD ||
          sim.bins.landfill >= BIN_CAPACITY * DUMP_THRESHOLD) {
        navigateToDump(sim)
      }
      break
    }
    case 'PICKING':
      stepPickingState(sim, dt)
      // After picking, return to random walk (not planCollectionNavigation)
      if (r.state === 'POST_PICK') {
        if (sim.bins.recyclable >= BIN_CAPACITY * DUMP_THRESHOLD ||
            sim.bins.compost >= BIN_CAPACITY * DUMP_THRESHOLD ||
            sim.bins.landfill >= BIN_CAPACITY * DUMP_THRESHOLD) {
          navigateToDump(sim)
        } else {
          r.state = 'COVERAGE'
        }
      }
      break
    case 'NAVIGATING_SECTOR': {
      // A* escape path — follow it, then resume random walking
      if (sim.pathIdx >= sim.currentPath.length) {
        r.state = 'COVERAGE'
        sim.randomWalkDir = r.angle + (Math.random() - 0.5) * Math.PI
        break
      }
      const wp = sim.currentPath[sim.pathIdx]
      driveToward(r, wp.x, wp.z, dt, sim.occGrid)
      sim.stats.distanceTraveled += Math.abs(r.linVel) * dt
      sim.stats.battery -= Math.abs(r.linVel) * dt * BATTERY_DRAIN_PER_METER
      if (distXZ(r, wp) < 0.3) sim.pathIdx++
      detectWaste(sim)
      const esc_nearby = findNearbyWaste(sim, ARM_REACH)
      if (esc_nearby) {
        r.targetWaste = esc_nearby.id
        r.state = 'PICKING'
        r.pickupTimer = PICKUP_TIME
        r.armState = 'reaching'
      }
      break
    }
    case 'NAVIGATING_DUMP':
      stepNavigatingDumpShared(sim, dt)
      break
    case 'DUMPING':
      stepDumpingState(sim, dt)
      // After dump, resume random walk
      if (r.state === 'POST_DUMP') r.state = 'COVERAGE'
      break
    case 'DONE': break
  }
}

function nnExploreUnseen(sim, venue) {
  const r = sim.robot
  const unseenTarget = findBestUnseenTarget(sim, venue)
  if (unseenTarget) {
    const path = aStarSearch(sim.occGrid, r.x, r.z, unseenTarget.x, unseenTarget.z)
    if (path && path.length > 1) {
      sim.currentPath = path
      sim.pathIdx = 1
      r.state = 'NAVIGATING_SECTOR'
      return
    }
  }
  // No unseen targets — check completion
  const fogCov = getFogCoverage(sim)
  if (fogCov > 0.99 && sim.collectionQueue.length === 0) {
    r.state = 'DONE'
  } else {
    // Fallback: quick scan then retry
    sim.scanTimer = 0
    r.state = 'SCANNING'
  }
}

function stepNearestNeighbor(sim, dt, venue) {
  const r = sim.robot
  // Early exit: all items collected
  const allCollected = sim.wasteItems && sim.wasteItems.length > 0 && sim.wasteItems.every(w => w.collected)
  if (allCollected && sim.collectionQueue.length === 0 && r.state !== 'DONE') {
    r.state = 'DONE'
    logDebug(sim, 'STATE', `NN: DONE — all ${sim.stats.itemsCollected} items collected`)
    return
  }

  switch (r.state) {
    case 'IDLE':
      // Start with a 360 scan
      sim.scanTimer = 0
      sim.scanAngle = r.angle
      r.state = 'SCANNING'
      break
    case 'SCANNING': {
      // Slow rotation to scan
      sim.scanTimer += dt
      r.angle += 1.0 * dt // ~1 rad/s rotation
      r.angle = ((r.angle % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)
      detectWaste(sim)

      // Opportunistic pickup during scan
      const nearby = findNearbyWaste(sim, ARM_REACH)
      if (nearby) {
        r.targetWaste = nearby.id
        r.state = 'PICKING'
        r.pickupTimer = PICKUP_TIME
        r.armState = 'reaching'
        sim._scanAttempts = 0
        break
      }

      if (sim.scanTimer > 2 * Math.PI / 1.0) { // Full rotation done
        sim._scanAttempts = (sim._scanAttempts || 0) + 1

        // Re-queue detected but uncollected items that were dropped from the queue
        if (sim.collectionQueue.length === 0) {
          const requeueable = sim.wasteItems.filter(w => w.detected && !w.collected && !w._unreachable)
          const alreadyQueued = new Set(sim.collectionQueue.map(q => q.id))
          const toRequeue = requeueable.filter(w => !alreadyQueued.has(w.id))
          if (toRequeue.length > 0 && (sim._requeueCount || 0) < 3) {
            sim._requeueCount = (sim._requeueCount || 0) + 1
            for (const w of toRequeue) {
              sim.collectionQueue.push({ id: w.id, x: w.x, z: w.z, bin: w.bin })
            }
            logDebug(sim, 'STATE', `Re-queued ${toRequeue.length} dropped items (attempt ${sim._requeueCount})`)
          }
        }

        if (sim.collectionQueue.length > 0) {
          sim._scanAttempts = 0
          planGreedyNavigation(sim)
        } else {
          // Navigate toward largest unseen area on the fog map
          const unseenTarget = findBestUnseenTarget(sim, venue)
          if (unseenTarget) {
            const path = aStarSearch(sim.occGrid, r.x, r.z, unseenTarget.x, unseenTarget.z)
            if (path && path.length > 1) {
              sim.currentPath = path
              sim.pathIdx = 1
              r.state = 'NAVIGATING_SECTOR'
              sim._scanAttempts = 0
            } else if (sim._scanAttempts >= 3) {
              // Can't reach unseen area after 3 tries — try random exploration
              const uncollected = sim.wasteItems.filter(w => !w.collected).length
              const giveUpThreshold = uncollected > 0 ? Math.min(3 + uncollected, 15) : 3
              let escaped = false
              for (let a = 0; a < 8; a++) {
                const angle = ((a + sim._scanAttempts) % 8 / 8) * Math.PI * 2
                const dist = 3 + (sim._scanAttempts % 4)
                const escX = clamp(r.x + Math.cos(angle) * dist, 1, venue.width - 1)
                const escZ = clamp(r.z + Math.sin(angle) * dist, 1, venue.height - 1)
                const escapePath = aStarSearch(sim.occGrid, r.x, r.z, escX, escZ)
                if (escapePath && escapePath.length > 1) {
                  sim.currentPath = escapePath
                  sim.pathIdx = 1
                  r.state = 'NAVIGATING_SECTOR'
                  escaped = true
                  break
                }
              }
              if (!escaped && sim._scanAttempts >= giveUpThreshold) {
                logDebug(sim, 'DONE', `Scan stuck: no reachable path after ${sim._scanAttempts} attempts. items=${sim.stats.itemsCollected}/${sim.wasteItems.length}`)
                r.state = 'DONE'
              } else if (!escaped) {
                sim.scanTimer = 0
              }
            } else {
              sim.scanTimer = 0
            }
          } else {
            // No unseen target from fog — but items may still be uncollected
            const fogCov = getFogCoverage(sim)
            const allCollected = sim.wasteItems.every(w => w.collected)
            const uncollectedCount = sim.wasteItems.filter(w => !w.collected).length
            // Scale patience: more uncollected items = more attempts before giving up
            const maxAttempts = allCollected ? 3 : Math.min(5 + uncollectedCount * 2, 20)
            if ((fogCov > 0.99 && sim.collectionQueue.length === 0 && allCollected) || sim._scanAttempts >= maxAttempts) {
              if (!allCollected && sim._scanAttempts >= maxAttempts) {
                logDebug(sim, 'DONE', `Scan exhausted after ${sim._scanAttempts} rotations. fog=${(fogCov*100).toFixed(0)}% items=${sim.stats.itemsCollected}/${sim.wasteItems.length}`)
              }
              r.state = 'DONE'
            } else if (sim._scanAttempts >= 2 && !allCollected) {
              // Try random exploration to find undetected items
              let escaped = false
              for (let a = 0; a < 8; a++) {
                const angle = ((a + sim._scanAttempts) % 8 / 8) * Math.PI * 2
                const dist = 3 + (sim._scanAttempts % 3) * 2
                const escX = clamp(r.x + Math.cos(angle) * dist, 1, venue.width - 1)
                const escZ = clamp(r.z + Math.sin(angle) * dist, 1, venue.height - 1)
                const escapePath = aStarSearch(sim.occGrid, r.x, r.z, escX, escZ)
                if (escapePath && escapePath.length > 1) {
                  sim.currentPath = escapePath
                  sim.pathIdx = 1
                  r.state = 'NAVIGATING_SECTOR'
                  escaped = true
                  break
                }
              }
              if (!escaped) sim.scanTimer = 0
            } else {
              sim.scanTimer = 0
            }
          }
        }
      }
      break
    }
    case 'NAVIGATING_SECTOR': {
      // Navigate to unseen area, scanning along the way
      if (sim.pathIdx >= sim.currentPath.length) {
        // Don't immediately scan — check if there are more unseen areas to chain
        if (sim.collectionQueue.length > 0) {
          planGreedyNavigation(sim)
          break
        }
        const nextUnseen = findBestUnseenTarget(sim, venue)
        if (nextUnseen) {
          const path = aStarSearch(sim.occGrid, r.x, r.z, nextUnseen.x, nextUnseen.z)
          if (path && path.length > 1) {
            sim.currentPath = path
            sim.pathIdx = 1
            // Stay in NAVIGATING_SECTOR — chain unseen targets without stopping to scan
            break
          }
        }
        // No more unseen targets reachable — do a scan
        sim.scanTimer = 0
        r.state = 'SCANNING'
        break
      }
      const wp = sim.currentPath[sim.pathIdx]
      driveToward(r, wp.x, wp.z, dt, sim.occGrid)
      sim.stats.distanceTraveled += Math.abs(r.linVel) * dt
      sim.stats.battery -= Math.abs(r.linVel) * dt * BATTERY_DRAIN_PER_METER
      if (distXZ(r, wp) < 0.2) sim.pathIdx++
      detectWaste(sim)

      // If items detected, go collect
      if (sim.collectionQueue.length > 0) {
        planGreedyNavigation(sim)
      }

      const nearbyS = findNearbyWaste(sim, ARM_REACH)
      if (nearbyS) {
        r.targetWaste = nearbyS.id
        r.state = 'PICKING'
        r.pickupTimer = PICKUP_TIME
        r.armState = 'reaching'
      }
      break
    }
    case 'NAVIGATING': {
      if (sim.pathIdx >= sim.currentPath.length) {
        if (sim.collectionQueue.length > 0) {
          const target = sim.collectionQueue[0]
          if (distXZ(r, target) < ARM_REACH + 0.1) {
            r.targetWaste = target.id
            r.state = 'PICKING'
            r.pickupTimer = PICKUP_TIME
            r.armState = 'reaching'
          } else {
            planGreedyNavigation(sim)
          }
        } else {
          // Skip scan — go directly to nearest unseen area
          const unseenTarget = findBestUnseenTarget(sim, venue)
          if (unseenTarget) {
            const path = aStarSearch(sim.occGrid, r.x, r.z, unseenTarget.x, unseenTarget.z)
            if (path && path.length > 1) {
              sim.currentPath = path
              sim.pathIdx = 1
              r.state = 'NAVIGATING_SECTOR'
              break
            }
          }
          sim.scanTimer = 0
          r.state = 'SCANNING'
        }
        break
      }
      const wp = sim.currentPath[sim.pathIdx]
      driveToward(r, wp.x, wp.z, dt, sim.occGrid)
      sim.stats.distanceTraveled += Math.abs(r.linVel) * dt
      sim.stats.battery -= Math.abs(r.linVel) * dt * BATTERY_DRAIN_PER_METER
      if (distXZ(r, wp) < 0.2) sim.pathIdx++
      detectWaste(sim)

      if (sim.collectionQueue.length > 0 && distXZ(r, sim.collectionQueue[0]) < ARM_REACH) {
        r.targetWaste = sim.collectionQueue[0].id
        r.state = 'PICKING'
        r.pickupTimer = PICKUP_TIME
        r.armState = 'reaching'
      }
      break
    }
    case 'PICKING':
      stepPickingState(sim, dt)
      if (r.state === 'POST_PICK') {
        if (sim.bins.recyclable >= BIN_CAPACITY * DUMP_THRESHOLD ||
            sim.bins.compost >= BIN_CAPACITY * DUMP_THRESHOLD ||
            sim.bins.landfill >= BIN_CAPACITY * DUMP_THRESHOLD) {
          navigateToDump(sim)
        } else if (sim.collectionQueue.length > 0) {
          planGreedyNavigation(sim)
        } else {
          // Go directly to unseen area instead of scanning in place
          nnExploreUnseen(sim, venue)
        }
      }
      break
    case 'NAVIGATING_DUMP':
      stepNavigatingDumpShared(sim, dt)
      break
    case 'DUMPING':
      stepDumpingState(sim, dt)
      if (r.state === 'POST_DUMP') {
        if (sim.collectionQueue.length > 0) planGreedyNavigation(sim)
        else nnExploreUnseen(sim, venue)
      }
      break
    case 'DONE': break
  }
}

function stepFullBoustrophedon(sim, dt, venue) {
  const r = sim.robot
  // Early exit: all items collected
  const allCollected = sim.wasteItems && sim.wasteItems.length > 0 && sim.wasteItems.every(w => w.collected)
  if (allCollected && sim.collectionQueue.length === 0 && r.state !== 'DONE') {
    r.state = 'DONE'
    logDebug(sim, 'STATE', `Boustrophedon: DONE — all ${sim.stats.itemsCollected} items collected`)
    return
  }

  // After sweep completes, collect detected items, then optionally do a second pass
  if (sim.sweepComplete) {
    // Check if we should start a second offset pass
    if (!sim._secondPassDone && r.state === 'SCANNING' && sim.collectionQueue.length === 0) {
      const uncollected = sim.wasteItems.filter(w => !w.collected && !w._unreachable).length
      if (uncollected > 0 && !sim._secondPassStarted) {
        // Start second pass with offset lanes to catch items missed by first sweep
        sim._secondPassStarted = true
        sim.sweepComplete = false
        sim.sweepDir = 1
        sim.sweepZ = 0.5 + BOUSTROPHEDON_SPACING / 2 // offset by half spacing
        sim.boustroStuck = 0
        sim.currentPath = []
        sim.pathIdx = 0
        r.state = 'COVERAGE'
        logDebug(sim, 'STATE', `Starting second offset sweep pass (${uncollected} items remaining)`)
        return
      }
      sim._secondPassDone = true
    }
    stepNearestNeighbor(sim, dt, venue)
    return
  }

  switch (r.state) {
    case 'IDLE': {
      // Initialize sweep: go to top-left corner first, then sweep right
      sim.sweepDir = 1  // 1 = going right (+X), -1 = going left (-X)
      sim.sweepZ = 0.5  // current lane Z position
      sim.boustroStuck = 0  // stuck timer for obstacle escape
      sim.coveragePath = []
      sim.coverageIdx = 0
      sim.currentPath = []
      sim.pathIdx = 0
      const margin = 0.5
      // Navigate to top-left start position via A*
      const startPath = aStarSearch(sim.occGrid, r.x, r.z, margin, margin)
      if (startPath && startPath.length > 1) {
        sim.currentPath = startPath
        sim.pathIdx = 1
        r.state = 'NAVIGATING_SECTOR' // Follow A* to start, then begin COVERAGE
      } else {
        r.state = 'COVERAGE' // Already near top-left
      }
      break
    }
    case 'COVERAGE': {
      // Straight horizontal sweeps (lawn-mower pattern).
      // Drive STRAIGHT along the current lane (fixed Z = sim.sweepZ).
      // When hitting the far wall, drop down by BOUSTROPHEDON_SPACING, reverse.
      // Only use A* when actually blocked by an obstacle mid-lane.
      if (!sim.sweepDir) { sim.sweepDir = 1; sim.sweepZ = 0.5 }

      const margin = 0.5
      const targetX = sim.sweepDir > 0 ? venue.width - margin : margin
      const atEdge = sim.sweepDir > 0 ? r.x >= venue.width - margin - 0.3 : r.x <= margin + 0.3

      if (atEdge) {
        // Reached wall edge — advance to next lane
        sim.sweepZ += BOUSTROPHEDON_SPACING
        sim.sweepDir *= -1
        sim.currentPath = []
        sim.pathIdx = 0
        sim.boustroStuck = 0

        if (sim.sweepZ >= venue.height - margin) {
          // All lanes done — enter post-sweep collection phase
          sim.sweepComplete = true
          planNextBoustCollection(sim, venue)
          break
        }
        // Use A* to navigate to the start of the next lane (handles obstacle transitions)
        const nextStartX = sim.sweepDir > 0 ? margin : venue.width - margin
        const path = aStarSearch(sim.occGrid, r.x, r.z, nextStartX, sim.sweepZ)
        if (path && path.length > 1) {
          sim.currentPath = path
          sim.pathIdx = 1
          // Stay in COVERAGE — we'll follow the A* path via the path-following block below
        }
        break
      }

      // If we have an A* escape/transition path, follow it first
      if (sim.currentPath && sim.currentPath.length > 0 && sim.pathIdx < sim.currentPath.length) {
        const pathWp = sim.currentPath[sim.pathIdx]
        driveToward(r, pathWp.x, pathWp.z, dt, sim.occGrid)
        sim.stats.distanceTraveled += Math.abs(r.linVel) * dt
        sim.stats.battery -= Math.abs(r.linVel) * dt * BATTERY_DRAIN_PER_METER
        if (distXZ(r, pathWp) < 0.25) {
          sim.pathIdx++
          if (sim.pathIdx >= sim.currentPath.length) {
            sim.currentPath = []
            sim.pathIdx = 0
            sim.boustroStuck = 0
          }
        }
      } else {
        // Drive STRAIGHT along the lane: target is (targetX, sweepZ)
        driveToward(r, targetX, sim.sweepZ, dt, sim.occGrid)
        sim.stats.distanceTraveled += Math.abs(r.linVel) * dt
        sim.stats.battery -= Math.abs(r.linVel) * dt * BATTERY_DRAIN_PER_METER

        // Obstacle detection: if robot isn't making progress, use A* to route around
        if (r.linVel < 0.05) {
          sim.boustroStuck = (sim.boustroStuck || 0) + dt
        } else {
          sim.boustroStuck = 0
        }

        if (sim.boustroStuck > 0.5) {
          // Robot stuck — use A* to get around the obstacle back to the same lane
          sim.boustroStuck = 0
          // Try to find a path to a point further along the same lane
          const skipDist = 2.0
          const escapeX = clamp(r.x + sim.sweepDir * skipDist, margin, venue.width - margin)
          let escapePath = aStarSearch(sim.occGrid, r.x, r.z, escapeX, sim.sweepZ)
          if (!escapePath || escapePath.length <= 1) {
            // Try further ahead
            const escapeX2 = clamp(r.x + sim.sweepDir * skipDist * 2, margin, venue.width - margin)
            escapePath = aStarSearch(sim.occGrid, r.x, r.z, escapeX2, sim.sweepZ)
          }
          if (!escapePath || escapePath.length <= 1) {
            // Can't get around obstacle — skip to next lane
            sim.sweepZ += BOUSTROPHEDON_SPACING
            sim.sweepDir *= -1
          } else {
            sim.currentPath = escapePath
            sim.pathIdx = 1
            // Stay in COVERAGE — will follow A* path in the block above
          }
        }
      }

      detectWaste(sim)

      // Also detect items within proximity (1.5m) regardless of FOV angle.
      // Simulates downward-facing or wide-angle sensor for nearby items.
      for (const w of sim.wasteItems) {
        if (w.collected || w.detected) continue
        const d = distXZ(r, w)
        if (d < 1.5) {
          w.detected = true
          w.confidence = (0.85 + Math.random() * 0.14).toFixed(2)
          sim.detectedWaste.push(w.id)
          sim.collectionQueue.push({ id: w.id, x: w.x, z: w.z, bin: w.bin })
          logDebug(sim, 'DETECT', `Detected ${w.label || w.bin} at (${w.x.toFixed(1)}, ${w.z.toFixed(1)}) conf=${w.confidence}`)
        }
      }

      // Opportunistic pickup during sweep — grab items within arm reach
      const boustNearby = findNearbyWaste(sim, ARM_REACH)
      if (boustNearby) {
        // Save return point so we resume sweep after pickup
        sim.returnPoint = { x: r.x, z: sim.sweepZ, sweepDir: sim.sweepDir, sweepZ: sim.sweepZ, coverageIdx: sim.coverageIdx }
        r.targetWaste = boustNearby.id
        r.state = 'PICKING'
        r.pickupTimer = PICKUP_TIME
        r.armState = 'reaching'
        break
      }

      // Check dump threshold
      if (sim.bins.recyclable >= BIN_CAPACITY * DUMP_THRESHOLD ||
          sim.bins.compost >= BIN_CAPACITY * DUMP_THRESHOLD ||
          sim.bins.landfill >= BIN_CAPACITY * DUMP_THRESHOLD) {
        sim.returnPoint = { x: r.x, z: sim.sweepZ, sweepDir: sim.sweepDir, sweepZ: sim.sweepZ, coverageIdx: sim.coverageIdx }
        navigateToDump(sim)
        break
      }
      break
    }
    case 'NAVIGATING': {
      if (sim.pathIdx >= sim.currentPath.length) {
        if (sim.collectionQueue.length > 0) {
          const target = sim.collectionQueue[0]
          if (distXZ(r, target) < ARM_REACH + 0.1) {
            r.targetWaste = target.id
            r.state = 'PICKING'
            r.pickupTimer = PICKUP_TIME
            r.armState = 'reaching'
          } else {
            const path = aStarSearch(sim.occGrid, r.x, r.z, target.x, target.z)
            if (path && path.length > 1) { sim.currentPath = path; sim.pathIdx = 1 }
            else {
              rotateOrDropItem(sim)
              if (sim.sweepComplete) planNextBoustCollection(sim, venue)
              else returnToCoverage(sim)
            }
          }
        } else {
          if (sim.sweepComplete) planNextBoustCollection(sim, venue)
          else returnToCoverage(sim)
        }
        break
      }
      const navWp = sim.currentPath[sim.pathIdx]
      driveToward(r, navWp.x, navWp.z, dt, sim.occGrid)
      sim.stats.distanceTraveled += Math.abs(r.linVel) * dt
      sim.stats.battery -= Math.abs(r.linVel) * dt * BATTERY_DRAIN_PER_METER
      if (distXZ(r, navWp) < 0.2) sim.pathIdx++
      detectWaste(sim)
      if (sim.collectionQueue.length > 0 && distXZ(r, sim.collectionQueue[0]) < ARM_REACH) {
        r.targetWaste = sim.collectionQueue[0].id
        r.state = 'PICKING'
        r.pickupTimer = PICKUP_TIME
        r.armState = 'reaching'
      }
      break
    }
    case 'PICKING':
      stepPickingState(sim, dt)
      if (r.state === 'POST_PICK') {
        // Dump check first
        if (sim.bins.recyclable >= BIN_CAPACITY * DUMP_THRESHOLD ||
            sim.bins.compost >= BIN_CAPACITY * DUMP_THRESHOLD ||
            sim.bins.landfill >= BIN_CAPACITY * DUMP_THRESHOLD) {
          navigateToDump(sim)
        }
        // Full Boustrophedon post-sweep: chain directly to next item
        else if (sim.sweepComplete) {
          planNextBoustCollection(sim, venue)
        }
        // During sweep: ONLY pick up items within arm reach. Don't leave the sweep path.
        else if (sim.collectionQueue.length > 0) {
          const nearest = sim.collectionQueue.reduce((best, q) => !best || distXZ(r, q) < distXZ(r, best) ? q : best, null)
          if (nearest && distXZ(r, nearest) < ARM_REACH + 0.1) {
            r.targetWaste = nearest.id
            r.state = 'PICKING'
            r.pickupTimer = PICKUP_TIME
            r.armState = 'reaching'
          } else {
            // Items exist but too far — return to sweep, collect after sweep completes
            returnToCoverage(sim)
          }
        } else {
          returnToCoverage(sim)
        }
      }
      break
    case 'NAVIGATING_DUMP':
      stepNavigatingDumpShared(sim, dt)
      break
    case 'DUMPING':
      stepDumpingState(sim, dt)
      if (r.state === 'POST_DUMP') {
        if (sim.sweepComplete) planNextBoustCollection(sim, venue)
        else returnToCoverage(sim)
      }
      break
    case 'NAVIGATING_SECTOR': {
      // Used for initial navigation to start, return-to-coverage, stuck escape, and unseen exploration
      if (sim.pathIdx >= sim.currentPath.length) {
        sim.returnPoint = null
        sim.currentPath = []
        sim.pathIdx = 0
        sim.diversionCooldown = 2.0
        // After sweep: go to next collection target directly
        if (sim.sweepComplete) {
          detectWaste(sim)
          planNextBoustCollection(sim, venue)
          break
        }
        r.state = 'COVERAGE'
        // For Full Boustrophedon: snap sweepZ to robot's actual Z so horizontal sweep resumes straight
        if (sim.algorithm === 'full_boustrophedon' && sim.sweepZ !== undefined) {
          // Snap to nearest lane boundary (not raw Z) to prevent drift
          if (Math.abs(r.z - sim.sweepZ) < BOUSTROPHEDON_SPACING) {
            const laneIdx = Math.round(r.z / BOUSTROPHEDON_SPACING)
            sim.sweepZ = Math.max(0.5, laneIdx * BOUSTROPHEDON_SPACING)
          }
        }
        break
      }
      const sectorWp = sim.currentPath[sim.pathIdx]
      driveToward(r, sectorWp.x, sectorWp.z, dt, sim.occGrid)
      sim.stats.distanceTraveled += Math.abs(r.linVel) * dt
      sim.stats.battery -= Math.abs(r.linVel) * dt * BATTERY_DRAIN_PER_METER
      if (distXZ(r, sectorWp) < 0.2) sim.pathIdx++
      detectWaste(sim)
      break
    }
    case 'DONE': break
  }
}

// ═══════════════════════════════════════════════════════════════
// SECTION 3c: MASTER STEP DISPATCHER
// ═══════════════════════════════════════════════════════════════

function stepSimulationDispatch(sim, dt, venue, obstacles) {
  const r = sim.robot
  sim.stats.elapsedTime += dt

  // Paint FOV cone onto fog-of-war grid
  if (!sim.fogGrid) {
    const fw = Math.ceil(venue.width / sim.fogResolution)
    const fh = Math.ceil(venue.height / sim.fogResolution)
    sim.fogGrid = new Uint8Array(fw * fh) // 0=unseen, 1=seen
    sim.fogW = fw
    sim.fogH = fh
    // Pre-reveal 5x5m area around robot starting position (initial scan)
    const startRevealRadius = 2.5 // 5x5m = 2.5m radius
    const res = sim.fogResolution
    const sMinGx = Math.max(0, Math.floor((r.x - startRevealRadius) / res))
    const sMaxGx = Math.min(fw - 1, Math.ceil((r.x + startRevealRadius) / res))
    const sMinGz = Math.max(0, Math.floor((r.z - startRevealRadius) / res))
    const sMaxGz = Math.min(fh - 1, Math.ceil((r.z + startRevealRadius) / res))
    for (let gz = sMinGz; gz <= sMaxGz; gz++) {
      for (let gx = sMinGx; gx <= sMaxGx; gx++) {
        sim.fogGrid[gz * fw + gx] = 1
      }
    }
  }
  // Paint current FOV cone cells as seen (every 6th step to save performance)
  sim._fogCounter = (sim._fogCounter || 0) + 1
  if (sim._fogCounter % 6 === 0) {
    const fovHalfRad = (CAMERA_FOV_DEG / 2) * Math.PI / 180
    const res = sim.fogResolution
    // Only check cells within camera range box
    const scanR = CAMERA_MAX_RANGE
    const minGx = Math.max(0, Math.floor((r.x - scanR) / res))
    const maxGx = Math.min(sim.fogW - 1, Math.ceil((r.x + scanR) / res))
    const minGz = Math.max(0, Math.floor((r.z - scanR) / res))
    const maxGz = Math.min(sim.fogH - 1, Math.ceil((r.z + scanR) / res))
    for (let gz = minGz; gz <= maxGz; gz++) {
      for (let gx = minGx; gx <= maxGx; gx++) {
        if (sim.fogGrid[gz * sim.fogW + gx]) continue // already seen
        const wx = gx * res + res / 2, wz = gz * res + res / 2
        const dx = wx - r.x, dz = wz - r.z
        const dist = Math.sqrt(dx * dx + dz * dz)
        if (dist < CAMERA_MIN_RANGE || dist > CAMERA_MAX_RANGE) continue
        const angleToCell = Math.atan2(dx, dz)
        let aDiff = angleToCell - r.angle
        while (aDiff > Math.PI) aDiff -= 2 * Math.PI
        while (aDiff < -Math.PI) aDiff += 2 * Math.PI
        if (Math.abs(aDiff) < fovHalfRad) {
          sim.fogGrid[gz * sim.fogW + gx] = 1
        }
      }
    }
  }

  // Update particles
  sim.particles = sim.particles.filter(p => {
    p.life -= dt
    p.x += p.vx * dt; p.y += p.vy * dt; p.z += p.vz * dt
    p.vy -= 4 * dt
    return p.life > 0
  })

  // Decay deferred labels
  if (sim.deferredLabels) {
    sim.deferredLabels = sim.deferredLabels.filter(l => { l.timer -= dt; return l.timer > 0 })
  }
  // Pickup markers persist — no decay (they stay for the whole run)

  // Early completion — all items collected AND no deferred items AND venue fully scanned
  const allCollected = sim.wasteItems.every(w => w.collected)
  const noDeferred = !sim.deferredItems || sim.deferredItems.length === 0
  const fogCov = getFogCoverage(sim)
  const venueFullyScanned = fogCov >= 0.99
  if (sim.wasteItems.length > 0 && allCollected && noDeferred && venueFullyScanned && r.state !== 'DONE' && r.state !== 'DUMPING' && r.state !== 'NAVIGATING_DUMP' && r.state !== 'RECOVERING_DEFERRED') {
    r.state = 'DONE'; r.linVel = 0; r.angVel = 0; return
  }

  // Stuck detection — two tiers:
  // Tier 1: hasn't moved 0.1m in 2s (completely stuck)
  // Tier 2: hasn't moved 1.0m from checkpoint in 5s (oscillating near obstacle)
  const movableStates = ['COVERAGE', 'NAVIGATING', 'NAVIGATING_SECTOR', 'RECOVERING_DEFERRED', 'NAVIGATING_DUMP',
    'NAVIGATING_VIEWPOINT', 'SCAN_NAVIGATE', 'SWEEP_NAVIGATE', 'COLLECTING', 'SWEEPING']
  const movedDist = Math.sqrt((r.x - sim.lastPos.x) ** 2 + (r.z - sim.lastPos.z) ** 2)
  if (movedDist > 0.1) {
    sim.stuckCounter = 0; sim.lastPos = { x: r.x, z: r.z }
  } else if (movableStates.includes(r.state)) {
    // Only accumulate stuck counter in movable states (not during PICKING, SCANNING, etc.)
    sim.stuckCounter += dt
  } else {
    // Reset counter during non-movable states so it doesn't fire right after pickup
    sim.stuckCounter = 0
  }
  // Tier 2: long-range progress check
  if (!sim.progressCheckpoint) sim.progressCheckpoint = { x: r.x, z: r.z, timer: 0 }
  sim.progressCheckpoint.timer += dt
  const netProgress = Math.sqrt((r.x - sim.progressCheckpoint.x) ** 2 + (r.z - sim.progressCheckpoint.z) ** 2)
  if (netProgress > 1.0) {
    sim.progressCheckpoint = { x: r.x, z: r.z, timer: 0 }
  }
  const isStuck = (sim.stuckCounter > 2.0 || sim.progressCheckpoint.timer > 5.0) && movableStates.includes(r.state)
  if (isStuck) {
    logDebug(sim, 'STUCK', `Stuck escape triggered in ${r.state} at (${r.x.toFixed(1)}, ${r.z.toFixed(1)})`)
    sim.stuckCounter = 0
    sim.progressCheckpoint = { x: r.x, z: r.z, timer: 0 }

    // If stuck during dump navigation, re-plan path to dump station
    if (r.state === 'NAVIGATING_DUMP') {
      const dumpPath = aStarSearch(sim.occGrid, r.x, r.z, sim.dumpStation.x, sim.dumpStation.z)
      if (dumpPath && dumpPath.length > 1) {
        sim.currentPath = dumpPath
        sim.pathIdx = 1
        return // stay in NAVIGATING_DUMP with new path
      }
    }

    // Full Boustrophedon stuck escape: advance sweep to skip obstacle
    if (r.state === 'COVERAGE' && sim.algorithm === 'full_boustrophedon' && sim.sweepDir !== undefined) {
      const margin = 0.5
      // Try A* to a point further along the same lane
      const skipDist = 3.0
      const escapeX = clamp(r.x + sim.sweepDir * skipDist, margin, venue.width - margin)
      const escapePath = aStarSearch(sim.occGrid, r.x, r.z, escapeX, sim.sweepZ)
      if (escapePath && escapePath.length > 1) {
        sim.currentPath = escapePath
        sim.pathIdx = 1
        // Stay in COVERAGE — will follow A* path
        return
      }
      // Can't escape on same lane — advance to next lane
      sim.sweepZ += BOUSTROPHEDON_SPACING
      sim.sweepDir *= -1
      sim.currentPath = []
      sim.pathIdx = 0
      return
    }

    // Try to A* to a future waypoint (skip 1-5 ahead to get past blocked area)
    if (r.state === 'COVERAGE' && sim.coveragePath && sim.coverageIdx < sim.coveragePath.length) {
      for (let skip = 1; skip <= 5; skip++) {
        const tryIdx = sim.coverageIdx + skip
        if (tryIdx >= sim.coveragePath.length) break
        const nextWp = sim.coveragePath[tryIdx]
        const escapePath = aStarSearch(sim.occGrid, r.x, r.z, nextWp.x, nextWp.z)
        if (escapePath && escapePath.length > 1) {
          sim.coverageIdx = tryIdx
          sim.currentPath = escapePath
          sim.pathIdx = 1
          // Stay in COVERAGE for Boustrophedon (it handles A* paths in COVERAGE state)
          if (sim.algorithm !== 'full_boustrophedon') {
            sim.returnPoint = { x: nextWp.x, z: nextWp.z, coverageIdx: tryIdx }
            r.state = 'NAVIGATING_SECTOR'
          }
          return // handled
        }
      }
    }

    // If stuck during post-sweep collection NAVIGATING, rotate the current target to back
    if (sim.sweepComplete && r.state === 'NAVIGATING' && sim.collectionQueue.length > 0) {
      rotateOrDropItem(sim)
      planNextBoustCollection(sim, venue)
      return
    }

    // Fallback for non-coverage states: A* to a point 2m away in a clear direction
    let escaped = false
    for (let a = 0; a < 8; a++) {
      const angle = (a / 8) * Math.PI * 2
      const escX = clamp(r.x + Math.cos(angle) * 2, 1, venue.width - 1)
      const escZ = clamp(r.z + Math.sin(angle) * 2, 1, venue.height - 1)
      const escapePath = aStarSearch(sim.occGrid, r.x, r.z, escX, escZ)
      if (escapePath && escapePath.length > 1) {
        if (r.state === 'COVERAGE' && sim.coveragePath && sim.coverageIdx < sim.coveragePath.length) {
          sim.returnPoint = { x: r.x, z: r.z, coverageIdx: sim.coverageIdx }
        }
        sim.currentPath = escapePath
        sim.pathIdx = 1
        // Algorithm-specific escape state
        if (sim.algorithm === 'info_gain') {
          r.state = 'NAVIGATING_VIEWPOINT'
        } else if (sim.algorithm === 'reclaim') {
          r.state = 'SCAN_NAVIGATE'
        } else if (sim.algorithm === 'full_boustrophedon') {
          // Stay in COVERAGE — the path-following block handles A* paths
          r.state = 'COVERAGE'
        } else {
          r.state = 'NAVIGATING_SECTOR'
        }
        escaped = true
        break
      }
    }
    if (!escaped && r.state === 'COVERAGE' && sim.coveragePath) {
      sim.coverageIdx = Math.min(sim.coverageIdx + 3, sim.coveragePath.length - 1)
      r.state = 'COVERAGE'
    } else if (!escaped) {
      if (r.state === 'COVERAGE' && sim.coverageIdx < (sim.coveragePath?.length || 0)) sim.coverageIdx++
      else if (r.state === 'SCANNING') { sim.scanAngleAccum = (sim.scanAngleAccum || 0) + Math.PI }
      else { if (sim.collectionQueue.length > 0) rotateOrDropItem(sim) }
    }
  }

  // --- Anomaly detection ---
  // Spin detection: high angular velocity + near-zero linear velocity
  if (Math.abs(r.angVel) > 0.5 && Math.abs(r.linVel) < 0.05) {
    sim.spinTimer = (sim.spinTimer || 0) + dt
    if (sim.spinTimer > 3.0) {
      logDebug(sim, 'ANOMALY', `Spinning in place: angVel=${r.angVel.toFixed(2)}, linVel=${r.linVel.toFixed(3)}, state=${r.state}`)
      sim.debugAnomalies.spins++
      sim.spinTimer = 0
    }
  } else {
    sim.spinTimer = 0
  }
  // State stall detection: same FSM state for >60s without changing
  if (!sim._lastStateChange) sim._lastStateChange = { state: r.state, time: sim.stats.elapsedTime }
  if (r.state !== sim._lastStateChange.state) {
    logDebug(sim, 'STATE', `${sim._lastStateChange.state} → ${r.state}`)
    sim._lastStateChange = { state: r.state, time: sim.stats.elapsedTime }
  } else if (sim.stats.elapsedTime - sim._lastStateChange.time > 60) {
    logDebug(sim, 'ANOMALY', `State stall: stuck in ${r.state} for 60s+`)
    sim.debugAnomalies.stateStalls++
    sim._lastStateChange.time = sim.stats.elapsedTime
  }

  // Track new metrics
  if (sim.visitGrid) updateVisitGrid(sim.visitGrid, r.x, r.z)

  // Track path trail (green = has target, red = deadheading)
  const hasTarget = sim.collectionQueue.length > 0
  if (sim.pathTrail) {
    sim.pathTrail.push({ x: r.x, z: r.z, hasTarget })
    if (sim.pathTrail.length > 5000) sim.pathTrail = sim.pathTrail.slice(-3000)
  }

  // Track deadheading distance
  if (sim.collectionQueue.length === 0 && Math.abs(r.linVel) > 0.01) {
    sim.deadheadDist = (sim.deadheadDist || 0) + Math.abs(r.linVel) * dt
  }

  // Track time to 90% and time to last
  if (sim.wasteItems && sim.wasteItems.length > 0) {
    const pctCollected = sim.stats.itemsCollected / sim.wasteItems.length
    if (pctCollected >= 0.9 && sim.timeTo90 == null) sim.timeTo90 = sim.stats.elapsedTime
    if (sim.stats.itemsCollected === sim.wasteItems.length && sim.timeToLast == null) sim.timeToLast = sim.stats.elapsedTime
  }

  // Dispatch to algorithm
  switch (sim.algorithm) {
    case 'random_walk': stepRandomWalk(sim, dt, venue); break
    case 'nearest_neighbor': stepNearestNeighbor(sim, dt, venue); break
    case 'full_boustrophedon': stepFullBoustrophedon(sim, dt, venue); break
    case 'info_gain': stepInfoGainExplorer(sim, dt, venue); break
    case 'reclaim': stepReclaim(sim, dt, venue); break
    default: stepNearestNeighbor(sim, dt, venue)
  }

  // Universal distance + battery tracking (for new algorithms that don't track internally)
  if (sim.algorithm === 'info_gain' || sim.algorithm === 'reclaim') {
    const moved = Math.abs(r.linVel) * dt
    sim.stats.distanceTraveled += moved
    sim.stats.battery -= moved * BATTERY_DRAIN_PER_METER
  }
}

// ═══════════════════════════════════════════════════════════════
// ALGORITHM 4: Information-Gain Explorer
// States: IDLE, SCANNING, NAVIGATING, NAVIGATING_VIEWPOINT, PICKING,
//         POST_PICK, NAVIGATING_DUMP, DUMPING, POST_DUMP, DONE
// ═══════════════════════════════════════════════════════════════
function stepInfoGainExplorer(sim, dt, venue) {
  const r = sim.robot
  detectWaste(sim)

  switch (r.state) {
    case 'IDLE': {
      // Initial 360° scan
      r.state = 'SCANNING'
      sim.scanAngleStart = r.angle
      sim.scanAngleAccum = 0
      break
    }
    case 'SCANNING': {
      // Rotate to scan
      const scanSpeed = 1.5
      r.angVel = scanSpeed
      r.linVel = 0
      r.angle += scanSpeed * dt
      r.angle = ((r.angle % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)
      sim.scanAngleAccum = (sim.scanAngleAccum || 0) + scanSpeed * dt
      detectWaste(sim)

      // If items found mid-scan (after initial scan), stop and collect immediately
      if (sim.collectionQueue.length > 0 && sim._initialScanDone) {
        r.angVel = 0
        planGreedyNavigation(sim)
        r.state = 'NAVIGATING'
        break
      }

      // 120° directed scan at viewpoints (face toward fog, scan 120° arc)
      const scanThreshold = sim._initialScanDone ? INFO_GAIN_SCAN_ARC : (Math.PI * 2)
      if (sim.scanAngleAccum >= scanThreshold) {
        r.angVel = 0
        sim._initialScanDone = true
        const allCollected = sim.wasteItems && sim.wasteItems.length > 0 && sim.wasteItems.every(w => w.collected)
        if (allCollected && sim.collectionQueue.length === 0) {
          r.state = 'DONE'
          logDebug(sim, 'STATE', `Info-Gain: DONE — all ${sim.stats.itemsCollected} items collected`)
          break
        }
        // Re-queue any detected-but-uncollected items back into the queue
        if (sim.collectionQueue.length === 0 && sim.wasteItems) {
          const missed = sim.wasteItems.filter(w => w.detected && !w.collected && !w._unreachable)
          for (const w of missed) {
            w._failCount = 0
            sim.collectionQueue.push(w)
          }
        }
        if (sim.collectionQueue.length > 0) {
          planGreedyNavigation(sim)
          r.state = 'NAVIGATING'
        } else {
          const fogCov = getFogCoverage(sim)
          // Done if: 95%+ fog covered, OR fog stalled 90s with no new pickups
          if (fogCov >= 0.95) {
            r.state = 'DONE'
            logDebug(sim, 'STATE', `Info-Gain: DONE — ${(fogCov*100).toFixed(0)}% coverage`)
            break
          }
          // Anti-oscillation: if position hasn't changed >1m in 30s, force move to opposite side
          if (!sim._igPosCheck) sim._igPosCheck = { time: sim.stats.elapsedTime, x: r.x, z: r.z, fog: fogCov }
          const posElapsed = sim.stats.elapsedTime - sim._igPosCheck.time
          const posMoved = Math.sqrt((r.x - sim._igPosCheck.x)**2 + (r.z - sim._igPosCheck.z)**2)
          const fogProgress = fogCov - sim._igPosCheck.fog
          if (posElapsed > 20 && posMoved < 3.0 && fogProgress < 0.03) {
            // Force navigate to a different area — cycle through targets
            sim._igEscapeIdx = ((sim._igEscapeIdx || 0) + 1) % 6
            const targets = [
              { x: venue.width * 0.15, z: venue.height * 0.15 },
              { x: venue.width * 0.85, z: venue.height * 0.15 },
              { x: venue.width * 0.85, z: venue.height * 0.85 },
              { x: venue.width * 0.15, z: venue.height * 0.85 },
              { x: venue.width * 0.5, z: venue.height * 0.15 },
              { x: venue.width * 0.5, z: venue.height * 0.85 },
            ]
            const target = targets[sim._igEscapeIdx]
            const escapePath = aStarSearch(sim.occGrid, r.x, r.z, target.x, target.z)
            if (escapePath && escapePath.length > 1) {
              sim.currentPath = escapePath
              sim.pathIdx = 1
              r.state = 'NAVIGATING_VIEWPOINT'
              sim._igPosCheck = { time: sim.stats.elapsedTime, x: r.x, z: r.z, fog: fogCov }
              break
            }
          }
          if (posElapsed > 20) sim._igPosCheck = { time: sim.stats.elapsedTime, x: r.x, z: r.z, fog: fogCov }

          // Hard stall: if no items and no fog progress for 150s, DONE
          if (!sim._igStallCheck) sim._igStallCheck = { time: sim.stats.elapsedTime, items: sim.stats.itemsCollected, fog: fogCov }
          if (sim.stats.elapsedTime - sim._igStallCheck.time > 150) {
            if (sim.stats.itemsCollected === sim._igStallCheck.items && fogCov - sim._igStallCheck.fog < 0.05) {
              r.state = 'DONE'
              logDebug(sim, 'STATE', `Info-Gain: DONE — stalled 150s, fog ${(fogCov*100).toFixed(0)}%`)
              break
            }
            sim._igStallCheck = { time: sim.stats.elapsedTime, items: sim.stats.itemsCollected, fog: fogCov }
          }
          // Explore: unseen blob → info-gain viewpoint → random escape
          let vp = findBestUnseenTarget(sim, venue)
          if (!vp) vp = findInfoGainViewpoint(sim, venue, 1.0)
          if (vp) {
            const path = aStarSearch(sim.occGrid, r.x, r.z, vp.x, vp.z)
            if (path && path.length > 1) {
              sim.currentPath = path
              sim.pathIdx = 1
              r.state = 'NAVIGATING_VIEWPOINT'
              break
            }
          }
          // No viewpoint — try random escape
          sim._igEscapeCount = (sim._igEscapeCount || 0) + 1
          let escaped = false
          for (let a = 0; a < 8; a++) {
            const angle = ((a + sim._igEscapeCount) % 8 / 8) * Math.PI * 2
            const dist = 3 + (sim._igEscapeCount % 4)
            const escX = clamp(r.x + Math.cos(angle) * dist, 1, venue.width - 1)
            const escZ = clamp(r.z + Math.sin(angle) * dist, 1, venue.height - 1)
            const escapePath = aStarSearch(sim.occGrid, r.x, r.z, escX, escZ)
            if (escapePath && escapePath.length > 1) {
              sim.currentPath = escapePath
              sim.pathIdx = 1
              r.state = 'NAVIGATING_VIEWPOINT'
              escaped = true
              break
            }
          }
          if (!escaped) sim.scanAngleAccum = 0
        }
      }
      break
    }
    case 'NAVIGATING_VIEWPOINT': {
      // Navigate to info-gain viewpoint
      if (sim.pathIdx >= sim.currentPath.length) {
        // Arrived at viewpoint — face toward fog, then do a short scan
        if (sim._infoGainTargetAngle !== undefined) {
          // Turn toward the computed optimal facing angle before scanning
          let angleDiff = sim._infoGainTargetAngle - r.angle
          while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI
          while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI
          if (Math.abs(angleDiff) > 0.1) {
            r.angVel = Math.sign(angleDiff) * Math.min(Math.abs(angleDiff) * 3, MAX_ANGULAR_VEL)
            r.linVel = 0
            r.angle += r.angVel * dt
            r.angle = ((r.angle % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)
            detectWaste(sim)
            break
          }
          r.angVel = 0
          delete sim._infoGainTargetAngle
        }
        r.state = 'SCANNING'
        sim.scanAngleAccum = 0
        break
      }
      const wp = sim.currentPath[sim.pathIdx]
      driveToward(r, wp.x, wp.z, dt, sim.occGrid)
      if (distXZ(r, wp) < 0.2) sim.pathIdx++
      detectWaste(sim)

      // If items found during travel, switch to collecting
      if (sim.collectionQueue.length > 0) {
        const nearest = sim.collectionQueue[0]
        if (distXZ(r, nearest) < ARM_REACH + 0.2) {
          r.targetWaste = nearest.id
          r.state = 'PICKING'
          r.pickupTimer = PICKUP_TIME
          r.armState = 'reaching'
        } else {
          planGreedyNavigation(sim)
          r.state = 'NAVIGATING'
        }
      }
      break
    }
    case 'NAVIGATING': {
      // Navigate to item in collection queue
      if (sim.collectionQueue.length === 0) {
        r.state = 'SCANNING'
        sim.scanAngleAccum = 0
        break
      }
      if (sim.pathIdx >= sim.currentPath.length) {
        const target = sim.collectionQueue[0]
        if (target && distXZ(r, target) < ARM_REACH + 0.2) {
          r.targetWaste = target.id
          r.state = 'PICKING'
          r.pickupTimer = PICKUP_TIME
          r.armState = 'reaching'
        } else if (target) {
          const path = aStarSearch(sim.occGrid, r.x, r.z, target.x, target.z)
          if (path && path.length > 1) { sim.currentPath = path; sim.pathIdx = 1 }
          else { rotateOrDropItem(sim); if (sim.collectionQueue.length > 0) planGreedyNavigation(sim); else { r.state = 'SCANNING'; sim.scanAngleAccum = 0 } }
        } else {
          r.state = 'SCANNING'
          sim.scanAngleAccum = 0
        }
        break
      }
      const wp = sim.currentPath[sim.pathIdx]
      driveToward(r, wp.x, wp.z, dt, sim.occGrid)
      if (distXZ(r, wp) < 0.2) sim.pathIdx++
      detectWaste(sim)

      // Opportunistic pickup
      const nearest = sim.collectionQueue[0]
      if (nearest && distXZ(r, nearest) < ARM_REACH + 0.2) {
        r.targetWaste = nearest.id
        r.state = 'PICKING'
        r.pickupTimer = PICKUP_TIME
        r.armState = 'reaching'
      }
      break
    }
    case 'PICKING': {
      stepPickingState(sim, dt)
      if (r.state === 'POST_PICK') {
        // Check if dump needed
        const needsDump = Object.values(sim.bins).some(v => v >= BIN_CAPACITY)
        if (needsDump) {
          navigateToDump(sim)
          r.state = 'NAVIGATING_DUMP'
        } else if (sim.collectionQueue.length > 0) {
          planGreedyNavigation(sim)
          r.state = 'NAVIGATING'
        } else {
          const vp = findInfoGainViewpoint(sim, venue, 0.5) // RECLAIM: pick nearby viewpoints too
          if (vp) {
            const path = aStarSearch(sim.occGrid, r.x, r.z, vp.x, vp.z)
            if (path && path.length > 1) { sim.currentPath = path; sim.pathIdx = 1; r.state = 'NAVIGATING_VIEWPOINT' }
            else { r.state = 'SCANNING'; sim.scanAngleAccum = 0 }
          } else {
            r.state = 'SCANNING'
            sim.scanAngleAccum = 0
          }
        }
      }
      break
    }
    case 'NAVIGATING_DUMP': {
      stepNavigatingDumpShared(sim, dt)
      break
    }
    case 'DUMPING': {
      stepDumpingState(sim, dt)
      if (r.state === 'POST_DUMP') {
        if (!sim.dumpTripFullness) sim.dumpTripFullness = []
        sim.dumpTripFullness.push(sim.bins.recyclable + sim.bins.compost + sim.bins.landfill)
        sim.bins = { recyclable: 0, compost: 0, landfill: 0 }
        sim.stats.dumpTrips++
        logDebug(sim, 'DUMP', `Dump trip #${sim.stats.dumpTrips} complete`)
        if (sim.collectionQueue.length > 0) {
          planGreedyNavigation(sim)
          r.state = 'NAVIGATING'
        } else {
          r.state = 'SCANNING'
          sim.scanAngleAccum = 0
        }
      }
      break
    }
    case 'DONE': break
  }
}

// ═══════════════════════════════════════════════════════════════
// ALGORITHM 5: RECLAIM — 3-Mode Hybrid
// Modes: COLLECT, SCAN, SWEEP
// States: IDLE, SCANNING, SCAN_NAVIGATE, COLLECTING, PICKING,
//         POST_PICK, NAVIGATING_DUMP, DUMPING, POST_DUMP,
//         SWEEP_NAVIGATE, SWEEPING, DONE
// ═══════════════════════════════════════════════════════════════
function stepReclaim(sim, dt, venue) {
  const r = sim.robot
  detectWaste(sim)

  // --- Mode controller (executive layer) ---
  if (r.state !== 'PICKING' && r.state !== 'NAVIGATING_DUMP' && r.state !== 'DUMPING'
      && r.state !== 'POST_PICK' && r.state !== 'POST_DUMP' && r.state !== 'DONE') {
    const fogCov = getFogCoverage(sim)
    const unscanned = 1 - fogCov
    const hasItems = sim.collectionQueue.length > 0
    const oldMode = sim.reclaimMode

    // Check if all items collected — done regardless of fog
    const allCollected = sim.wasteItems && sim.wasteItems.length > 0 && sim.wasteItems.every(w => w.collected)
    if (allCollected && !hasItems) {
      sim.reclaimMode = 'DONE'
    }
    // SWEEP time cap: if sweeping >200s with no new pickups, stop
    else if (sim.reclaimMode === 'SWEEP') {
      if (!sim._sweepStartTime) sim._sweepStartTime = sim.stats.elapsedTime
      if (!sim._sweepStartItems) sim._sweepStartItems = sim.stats.itemsCollected
      const sweepElapsed = sim.stats.elapsedTime - sim._sweepStartTime
      const sweepNewItems = sim.stats.itemsCollected - sim._sweepStartItems
      if (sweepElapsed > 400 && sweepNewItems === 0) {
        sim.reclaimMode = 'DONE'
        logDebug(sim, 'STATE', `RECLAIM: DONE — sweep stalled after ${sweepElapsed.toFixed(0)}s`)
      }
    }
    if (sim.reclaimMode === 'DONE') { /* already set */ }
    else if (hasItems) {
      sim.reclaimMode = 'COLLECT'
    } else if (unscanned > RECLAIM_UNSCANNED_THRESHOLD) {
      // Use SWEEP instead of SCAN when >40% is covered (switch earlier to systematic coverage)
      if (fogCov > 0.40) {
        const patches = findUnscannedPatches(sim, 2.0)
        if (patches.length > 0) {
          sim.reclaimMode = 'SWEEP'
          if (!sim._sweepPatches || sim._sweepPatches.length === 0) sim._sweepPatches = patches
        } else {
          sim.reclaimMode = 'SCAN'
        }
      } else {
        sim.reclaimMode = 'SCAN'
      }
    } else {
      const patches = findUnscannedPatches(sim, 2.0)
      if (patches.length > 0) {
        sim.reclaimMode = 'SWEEP'
        if (!sim._sweepPatches || sim._sweepPatches.length === 0) sim._sweepPatches = patches
      } else if (fogCov >= 0.95) {
        sim.reclaimMode = 'DONE'
      } else {
        sim.reclaimMode = 'SCAN'
      }
    }
    if (oldMode !== sim.reclaimMode) {
      if (sim.reclaimMode === 'SWEEP') {
        sim._sweepStartTime = sim.stats.elapsedTime
        sim._sweepStartItems = sim.stats.itemsCollected
      }
      logDebug(sim, 'MODE', `RECLAIM mode: ${oldMode} → ${sim.reclaimMode} (fog=${(fogCov * 100).toFixed(0)}%, queue=${sim.collectionQueue.length})`)
    }
  }

  switch (r.state) {
    case 'IDLE': {
      r.state = 'SCANNING'
      sim.scanAngleAccum = 0
      sim.reclaimMode = 'SCAN'
      break
    }

    // --- SCAN mode states ---
    case 'SCANNING': {
      const scanSpeed = 1.5
      r.angVel = scanSpeed
      r.linVel = 0
      r.angle += scanSpeed * dt
      r.angle = ((r.angle % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)
      sim.scanAngleAccum = (sim.scanAngleAccum || 0) + scanSpeed * dt
      detectWaste(sim)

      // If items found during scan, switch to collect
      if (sim.collectionQueue.length > 0) {
        r.angVel = 0
        reclaimStartCollecting(sim)
        break
      }

      if (sim.scanAngleAccum >= INFO_GAIN_SCAN_ARC) {
        r.angVel = 0
        if (sim.reclaimMode === 'DONE') {
          r.state = 'DONE'
          logDebug(sim, 'STATE', 'RECLAIM: DONE — full coverage')
          break
        }
        // If mode is SWEEP, go directly to sweep instead of more viewpoints
        if (sim.reclaimMode === 'SWEEP') {
          reclaimStartSweep(sim, venue)
          break
        }
        // Navigate to next info-gain viewpoint
        const vp = findInfoGainViewpoint(sim, venue, 0.5)
        if (vp) {
          const path = aStarSearch(sim.occGrid, r.x, r.z, vp.x, vp.z)
          if (path && path.length > 1) {
            sim.currentPath = path; sim.pathIdx = 1
            r.state = 'SCAN_NAVIGATE'
          } else {
            sim.scanAngleAccum = 0
          }
        } else if (sim.reclaimMode === 'SWEEP') {
          reclaimStartSweep(sim, venue)
        } else {
          // No viewpoint, no patches — might be done
          if (getFogCoverage(sim) >= 0.95 && sim.collectionQueue.length === 0) {
            r.state = 'DONE'
            logDebug(sim, 'STATE', 'RECLAIM: DONE — no viewpoints, high coverage')
          } else {
            sim.scanAngleAccum = 0
          }
        }
      }
      break
    }
    case 'SCAN_NAVIGATE': {
      // If mode changed to SWEEP mid-navigate, switch immediately
      if (sim.reclaimMode === 'SWEEP') {
        reclaimStartSweep(sim, venue)
        break
      }
      if (sim.pathIdx >= sim.currentPath.length) {
        r.state = 'SCANNING'
        sim.scanAngleAccum = 0
        break
      }
      const wp = sim.currentPath[sim.pathIdx]
      driveToward(r, wp.x, wp.z, dt, sim.occGrid)
      if (distXZ(r, wp) < 0.2) sim.pathIdx++
      detectWaste(sim)

      // If items found, switch to collect
      if (sim.collectionQueue.length > 0) {
        reclaimStartCollecting(sim)
      }
      break
    }

    // --- COLLECT mode states ---
    case 'COLLECTING': {
      if (sim.collectionQueue.length === 0) {
        // Queue empty — switch mode
        if (sim.reclaimMode === 'SWEEP') reclaimStartSweep(sim, venue)
        else { r.state = 'SCANNING'; sim.scanAngleAccum = 0 }
        break
      }
      if (sim.pathIdx >= sim.currentPath.length) {
        const target = sim.collectionQueue[0]
        if (target && distXZ(r, target) < ARM_REACH + 0.2) {
          r.targetWaste = target.id
          r.state = 'PICKING'
          r.pickupTimer = PICKUP_TIME
          r.armState = 'reaching'
        } else if (target) {
          const path = aStarSearch(sim.occGrid, r.x, r.z, target.x, target.z)
          if (path && path.length > 1) { sim.currentPath = path; sim.pathIdx = 1 }
          else { rotateOrDropItem(sim); reclaimStartCollecting(sim) }
        } else {
          r.state = 'SCANNING'; sim.scanAngleAccum = 0
        }
        break
      }
      const wp = sim.currentPath[sim.pathIdx]
      driveToward(r, wp.x, wp.z, dt, sim.occGrid)
      if (distXZ(r, wp) < 0.2) sim.pathIdx++
      detectWaste(sim)

      // Opportunistic pickup of nearby items
      const nearest = sim.collectionQueue[0]
      if (nearest && distXZ(r, nearest) < ARM_REACH + 0.2) {
        r.targetWaste = nearest.id
        r.state = 'PICKING'
        r.pickupTimer = PICKUP_TIME
        r.armState = 'reaching'
      }
      break
    }

    // --- SWEEP mode states ---
    case 'SWEEP_NAVIGATE': {
      if (sim.pathIdx >= sim.currentPath.length) {
        r.state = 'SWEEPING'
        sim.sweepIdx = 0
        break
      }
      const wp = sim.currentPath[sim.pathIdx]
      driveToward(r, wp.x, wp.z, dt, sim.occGrid)
      if (distXZ(r, wp) < 0.2) sim.pathIdx++
      detectWaste(sim)
      if (sim.collectionQueue.length > 0) reclaimStartCollecting(sim)
      break
    }
    case 'SWEEPING': {
      if (!sim._sweepPath || sim.sweepIdx >= sim._sweepPath.length) {
        // Patch done — find next or switch mode
        sim._sweepPatches = findUnscannedPatches(sim, 2.0)
        if (sim._sweepPatches.length > 0) {
          reclaimStartSweep(sim, venue)
        } else if (sim.collectionQueue.length > 0) {
          reclaimStartCollecting(sim)
        } else {
          r.state = 'SCANNING'; sim.scanAngleAccum = 0
        }
        break
      }
      const wp = sim._sweepPath[sim.sweepIdx]
      driveToward(r, wp.x, wp.z, dt, sim.occGrid)
      if (distXZ(r, wp) < 0.2) sim.sweepIdx++
      detectWaste(sim)
      if (sim.collectionQueue.length > 0) reclaimStartCollecting(sim)
      break
    }

    // --- Shared states ---
    case 'PICKING': {
      stepPickingState(sim, dt)
      if (r.state === 'POST_PICK') {
        // Bin-aware dump check
        const needsDump = Object.values(sim.bins).some(v => v >= BIN_CAPACITY)
        const shouldConsolidate = shouldConsolidateDump(sim)
        if (needsDump || shouldConsolidate) {
          if (!sim.dumpTripFullness) sim.dumpTripFullness = []
          sim.dumpTripFullness.push(sim.bins.recyclable + sim.bins.compost + sim.bins.landfill)
          navigateToDump(sim)
          r.state = 'NAVIGATING_DUMP'
        } else if (sim.collectionQueue.length > 0) {
          // Bin-aware filtering: defer abundant-type items
          reclaimFilterQueue(sim)
          reclaimStartCollecting(sim)
        } else {
          if (sim.reclaimMode === 'SWEEP') reclaimStartSweep(sim, venue)
          else { r.state = 'SCANNING'; sim.scanAngleAccum = 0 }
        }
      }
      break
    }
    case 'NAVIGATING_DUMP': {
      stepNavigatingDumpShared(sim, dt)
      break
    }
    case 'DUMPING': {
      stepDumpingState(sim, dt)
      if (r.state === 'POST_DUMP') {
        if (!sim.dumpTripFullness) sim.dumpTripFullness = []
        sim.bins = { recyclable: 0, compost: 0, landfill: 0 }
        sim.stats.dumpTrips++
        logDebug(sim, 'DUMP', `RECLAIM dump trip #${sim.stats.dumpTrips}`)

        // Recover deferred items
        if (sim.deferredItems && sim.deferredItems.length > 0) {
          for (const d of sim.deferredItems) {
            if (!d.collected && !sim.collectionQueue.find(q => q.id === d.id)) {
              sim.collectionQueue.push(d)
            }
          }
          sim.deferredItems = []
          sim.deferredRecoveries = (sim.deferredRecoveries || 0) + 1
        }

        if (sim.collectionQueue.length > 0) {
          reclaimStartCollecting(sim)
        } else {
          r.state = 'SCANNING'; sim.scanAngleAccum = 0
        }
      }
      break
    }
    case 'DONE': break
  }
}

// --- RECLAIM helper: start collecting items in queue ---
function reclaimStartCollecting(sim) {
  if (sim.collectionQueue.length === 0) {
    sim.robot.state = 'SCANNING'
    sim.scanAngleAccum = 0
    return
  }
  // NN+2-opt route optimization
  if (sim.collectionQueue.length > 1) {
    const result = nnTwoOptRoute(sim.robot, sim.collectionQueue)
    sim.collectionQueue = result.orderedItems
    sim.cvrpResult = { totalDist: result.totalDist, naiveDist: result.naiveDist, solveTimeMs: 0 }
    if (!sim.cvrpCumulative) sim.cvrpCumulative = { totalNaive: 0, totalOptimized: 0, batchCount: 0 }
    sim.cvrpCumulative.totalNaive += result.naiveDist
    sim.cvrpCumulative.totalOptimized += result.totalDist
    sim.cvrpCumulative.batchCount++
    // Store planned route for visualization
    sim.plannedRoute = sim.collectionQueue.slice(0, 5).map(q => ({ x: q.x, z: q.z }))
  }
  const target = sim.collectionQueue[0]
  const path = aStarSearch(sim.occGrid, sim.robot.x, sim.robot.z, target.x, target.z)
  if (path && path.length > 1) {
    sim.currentPath = path; sim.pathIdx = 1
    sim.robot.state = 'COLLECTING'
  } else {
    rotateOrDropItem(sim)
    if (sim.collectionQueue.length > 0) reclaimStartCollecting(sim)
    else { sim.robot.state = 'SCANNING'; sim.scanAngleAccum = 0 }
  }
}

// --- RECLAIM helper: start sweeping an unscanned patch ---
function reclaimStartSweep(sim, venue) {
  if (!sim._sweepPatches || sim._sweepPatches.length === 0) {
    sim._sweepPatches = findUnscannedPatches(sim, 2.0)
  }
  if (sim._sweepPatches.length === 0) {
    sim.robot.state = 'SCANNING'; sim.scanAngleAccum = 0
    return
  }
  const patch = sim._sweepPatches.shift()
  const sweepPath = generateBoustrophedonPath(patch, 0.3, BOUSTROPHEDON_SPACING)
  sim._sweepPath = sweepPath
  sim.sweepIdx = 0
  // Reset sweep stall timer when starting a new patch
  sim._sweepStartTime = sim.stats.elapsedTime
  sim._sweepStartItems = sim.stats.itemsCollected

  // Navigate to start of sweep
  if (sweepPath.length > 0) {
    const path = aStarSearch(sim.occGrid, sim.robot.x, sim.robot.z, sweepPath[0].x, sweepPath[0].z)
    if (path && path.length > 1) {
      sim.currentPath = path; sim.pathIdx = 1
      sim.robot.state = 'SWEEP_NAVIGATE'
    } else {
      sim.robot.state = 'SWEEPING'
    }
  } else {
    sim.robot.state = 'SCANNING'; sim.scanAngleAccum = 0
  }
}

// --- RECLAIM helper: bin-aware queue filtering ---
function reclaimFilterQueue(sim) {
  if (!sim.smartBinBalancing || sim.collectionQueue.length === 0) return
  const maxBin = Math.max(sim.bins.recyclable, sim.bins.compost, sim.bins.landfill)
  const maxBinPct = maxBin / BIN_CAPACITY

  if (maxBinPct >= BIN_HARD_PREF) {
    // Find which bin type(s) are at hard threshold
    const fullTypes = []
    if (sim.bins.recyclable / BIN_CAPACITY >= BIN_HARD_PREF) fullTypes.push('recyclable')
    if (sim.bins.compost / BIN_CAPACITY >= BIN_HARD_PREF) fullTypes.push('compost')
    if (sim.bins.landfill / BIN_CAPACITY >= BIN_HARD_PREF) fullTypes.push('landfill')

    const deferred = []
    const kept = []
    for (const item of sim.collectionQueue) {
      if (fullTypes.includes(item.bin) && distXZ(sim.robot, item) > 8.0) {
        deferred.push(item)
      } else {
        kept.push(item)
      }
    }
    if (deferred.length > 0) {
      if (!sim.deferredItems) sim.deferredItems = []
      for (const d of deferred) {
        if (!sim.deferredItems.find(e => e.id === d.id)) {
          sim.deferredItems.push(d)
          sim.totalDeferred = (sim.totalDeferred || 0) + 1
        }
      }
      sim.collectionQueue = kept
      logDebug(sim, 'BIN', `Deferred ${deferred.length} items (bin at ${(maxBinPct * 100).toFixed(0)}%)`)
    }
  }
}

// ═══════════════════════════════════════════════════════════════
// (Old stepSimulation removed — replaced by stepInfoGainExplorer + stepReclaim)
// ═══════════════════════════════════════════════════════════════

// Old stepSimulation FSM deleted — all code below until driveToward is dead
function driveToward(r, tx, tz, dt, occGrid) {
  const dx = tx - r.x, dz = tz - r.z
  const dist = Math.sqrt(dx * dx + dz * dz)
  if (dist < 0.05) { r.linVel = 0; return dist }
  const desiredAngle = Math.atan2(dx, dz)
  const aDiff = angleDiff(r.angle, desiredAngle)
  const angTarget = clamp(aDiff * 3.0, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
  r.angVel = moveToward(r.angVel, angTarget, ANGULAR_ACC * dt)
  r.angle += r.angVel * dt
  r.angle = ((r.angle % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)
  // Smooth alignment: linear velocity scales smoothly with alignment (no hard threshold)
  const alignFactor = 1 - Math.min(1, Math.abs(aDiff) / (Math.PI * 0.5))
  const linTarget = MAX_LINEAR_VEL * alignFactor * Math.min(1, dist / 0.5)
  r.linVel = moveToward(r.linVel, linTarget, LINEAR_ACC * dt)
  let moveX = Math.sin(r.angle) * r.linVel * dt
  let moveZ = Math.cos(r.angle) * r.linVel * dt

  // Repulsion force: push away from nearby obstacles (potential field)
  if (occGrid && r.linVel > 0.01) {
    let repX = 0, repZ = 0
    const checkR = 4 // check 4 cells around robot
    const cg = worldToGrid(r.x, r.z, occGrid.resolution)
    for (let dz = -checkR; dz <= checkR; dz++) {
      for (let dx = -checkR; dx <= checkR; dx++) {
        const nx = cg.gx + dx, nz = cg.gz + dz
        if (nx < 0 || nx >= occGrid.gw || nz < 0 || nz >= occGrid.gh) continue
        const cell = occGrid.grid[nz * occGrid.gw + nx]
        if (cell === 1) {
          const wx = nx * occGrid.resolution, wz = nz * occGrid.resolution
          const ddx = r.x - wx, ddz = r.z - wz
          const d = Math.sqrt(ddx * ddx + ddz * ddz) + 0.01
          if (d < 0.6) {
            const force = 0.3 * (0.6 - d) / d
            repX += ddx * force
            repZ += ddz * force
          }
        }
      }
    }
    moveX += repX * dt
    moveZ += repZ * dt
  }

  const newX = r.x + moveX
  const newZ = r.z + moveZ
  // Multi-point collision check — test center + 4 corners of robot footprint
  if (occGrid) {
    const halfW = ROBOT_WIDTH / 2, halfD = ROBOT_DEPTH / 2
    const cosA = Math.cos(r.angle), sinA = Math.sin(r.angle)
    const checkPoints = [
      { x: newX, z: newZ }, // center
      { x: newX + sinA * halfD + cosA * halfW, z: newZ + cosA * halfD - sinA * halfW }, // front-right
      { x: newX + sinA * halfD - cosA * halfW, z: newZ + cosA * halfD + sinA * halfW }, // front-left
      { x: newX - sinA * halfD + cosA * halfW, z: newZ - cosA * halfD - sinA * halfW }, // rear-right
      { x: newX - sinA * halfD - cosA * halfW, z: newZ - cosA * halfD + sinA * halfW }, // rear-left
    ]
    let blocked = false
    for (const pt of checkPoints) {
      const g = worldToGrid(pt.x, pt.z, occGrid.resolution)
      if (g.gx >= 0 && g.gx < occGrid.gw && g.gz >= 0 && g.gz < occGrid.gh) {
        if (occGrid.grid[g.gz * occGrid.gw + g.gx] === 1) { blocked = true; break }
      }
    }
    if (blocked) {
      // Try sliding: move only in X or only in Z
      const slideX = r.x + moveX
      const gSlideX = worldToGrid(slideX, r.z, occGrid.resolution)
      if (gSlideX.gx >= 0 && gSlideX.gx < occGrid.gw && gSlideX.gz >= 0 && gSlideX.gz < occGrid.gh
          && occGrid.grid[gSlideX.gz * occGrid.gw + gSlideX.gx] !== 1) {
        r.x = slideX
        r.linVel *= 0.5
        return dist
      }
      const slideZ2 = r.z + moveZ
      const gSlideZ = worldToGrid(r.x, slideZ2, occGrid.resolution)
      if (gSlideZ.gx >= 0 && gSlideZ.gx < occGrid.gw && gSlideZ.gz >= 0 && gSlideZ.gz < occGrid.gh
          && occGrid.grid[gSlideZ.gz * occGrid.gw + gSlideZ.gx] !== 1) {
        r.z = slideZ2
        r.linVel *= 0.5
        return dist
      }
      r.linVel = 0
      return dist
    }
  }
  r.x = newX
  r.z = newZ
  return dist
}

function detectWaste(sim) {
  const r = sim.robot
  let newDetection = false
  for (const w of sim.wasteItems) {
    if (w.collected || w.detected) continue
    if (pointInFOVCone(r.x, r.z, r.angle, w.x, w.z, CAMERA_FOV_DEG, CAMERA_MIN_RANGE, CAMERA_MAX_RANGE)) {
      w.detected = true
      w.confidence = (0.85 + Math.random() * 0.14).toFixed(2)
      sim.detectedWaste.push(w.id)
      sim.collectionQueue.push({ id: w.id, x: w.x, z: w.z, bin: w.bin })
      sim.allDetectedItems.push({ id: w.id, x: w.x, z: w.z, bin: w.bin })
      logDebug(sim, 'DETECT', `Detected ${w.label || w.bin} at (${w.x.toFixed(1)}, ${w.z.toFixed(1)}) conf=${w.confidence}`)
      newDetection = true
    }
  }
}


function findNearbyWaste(sim, radius) {
  const r = sim.robot
  const deferredIds = sim.deferredItems ? new Set(sim.deferredItems.map(d => d.id)) : new Set()
  let best = null, bestDist = Infinity
  for (const w of sim.wasteItems) {
    if (w.collected || deferredIds.has(w.id)) continue
    const d = distXZ(r, w)
    if (d < radius && d < bestDist && w.detected) {
      bestDist = d
      best = w
    }
  }
  return best
}

// Smart Bin Balancing: should we defer picking up this item?
function shouldDeferItem(sim, itemBin) {
  if (!sim.smartBinBalancing) return false
  const bins = sim.bins
  const targetRatio = bins[itemBin] / BIN_CAPACITY
  // If any bin is at 100%, force dump regardless
  if (bins.recyclable >= BIN_CAPACITY || bins.compost >= BIN_CAPACITY || bins.landfill >= BIN_CAPACITY) return false
  // If target bin is below skip threshold, no deferral
  if (targetRatio < BIN_HARD_PREF) return false
  // Get other bins' average ratio
  const binNames = ['recyclable', 'compost', 'landfill']
  const otherBins = binNames.filter(b => b !== itemBin)
  const otherAvg = otherBins.reduce((s, b) => s + bins[b] / BIN_CAPACITY, 0) / otherBins.length
  // If all bins above consolidate threshold, pick up anyway (will trigger consolidated dump)
  if (otherAvg >= BIN_CONSOLIDATE_THRESHOLD) return false
  // Defer: target bin near full, others still have room
  return true
}

// Check if we should trigger a consolidated dump (all bins above consolidate threshold)
function shouldConsolidateDump(sim) {
  if (!sim.smartBinBalancing) return false
  const bins = sim.bins
  return (bins.recyclable / BIN_CAPACITY >= BIN_CONSOLIDATE_THRESHOLD &&
          bins.compost / BIN_CAPACITY >= BIN_CONSOLIDATE_THRESHOLD &&
          bins.landfill / BIN_CAPACITY >= BIN_CONSOLIDATE_THRESHOLD)
}

// After dump: plan deferred recovery if there are deferred items
function planDeferredRecovery(sim) {
  if (sim.deferredItems.length === 0) return false
  sim.deferredRecoveries++
  // Re-add deferred items to collection queue
  for (const d of sim.deferredItems) {
    sim.collectionQueue.push({ id: d.id, x: d.x, z: d.z, bin: d.bin })
  }
  sim.deferredItems = []
  // NN+2-opt optimize the recovery route
  if (sim.collectionQueue.length > 1) {
    const result = nnTwoOptRoute(sim.robot, sim.collectionQueue)
    sim.collectionQueue = result.orderedItems
    sim.cvrpResult = { totalDist: result.totalDist, naiveDist: result.naiveDist, solveTimeMs: 0 }
    if (!sim.cvrpCumulative) sim.cvrpCumulative = { totalNaive: 0, totalOptimized: 0, batchCount: 0 }
    sim.cvrpCumulative.totalNaive += result.naiveDist
    sim.cvrpCumulative.totalOptimized += result.totalDist
    sim.cvrpCumulative.batchCount++
  }
  planCollectionNavigation(sim)
  return true
}

function planCollectionNavigation(sim, _depth = 0) {
  // Guard: remove any stale/undefined entries from queue
  sim.collectionQueue = sim.collectionQueue.filter(q => q && q.bin && q.x !== undefined)
  if (sim.collectionQueue.length === 0) {
    returnToCoverage(sim)
    return
  }
  // Prevent infinite recursion
  if (_depth > sim.collectionQueue.length + 2) {
    returnToCoverage(sim)
    return
  }

  const robotPos = { x: sim.robot.x, z: sim.robot.z }

  // RECLAIM: NN + 2-opt optimized routing with bin-aware filtering
  if (sim.algorithm === 'reclaim' && sim.collectionQueue.length > 1 && _depth === 0) {
    const result = nnTwoOptRoute(robotPos, sim.collectionQueue)
    sim.collectionQueue = result.orderedItems
    sim.cvrpResult = { totalDist: result.totalDist, naiveDist: result.naiveDist, solveTimeMs: 0 }
    if (!sim.cvrpCumulative) sim.cvrpCumulative = { totalNaive: 0, totalOptimized: 0, batchCount: 0 }
    sim.cvrpCumulative.totalNaive += result.naiveDist
    sim.cvrpCumulative.totalOptimized += result.totalDist
    sim.cvrpCumulative.batchCount++
    sim.plannedRoute = sim.collectionQueue.slice(0, 5).map(q => ({ x: q.x, z: q.z }))
  } else if (_depth === 0) {
    sim.collectionQueue.sort((a, b) => distXZ(sim.robot, a) - distXZ(sim.robot, b))
  }

  if (sim.collectionQueue.length === 0) {
    returnToCoverage(sim)
    return
  }
  const target = sim.collectionQueue[0]
  if (!target || target.x === undefined) {
    sim.collectionQueue.shift()
    if (sim.collectionQueue.length > 0) planCollectionNavigation(sim, _depth + 1)
    else returnToCoverage(sim)
    return
  }
  const path = aStarSearch(sim.occGrid, sim.robot.x, sim.robot.z, target.x, target.z)
  if (path && path.length > 1) {
    sim.currentPath = path
    sim.pathIdx = 1
    sim.robot.state = 'NAVIGATING'
  } else {
    rotateOrDropItem(sim)
    if (sim.collectionQueue.length > 0) planCollectionNavigation(sim, _depth + 1)
    else returnToCoverage(sim)
  }
}

function planNextBoustCollection(sim, venue, _depth = 0) {
  const r = sim.robot
  // Prevent infinite recursion — if we've tried all items in one tick, stop and scan
  if (_depth > sim.collectionQueue.length + 2) {
    r.state = 'SCANNING'
    sim.scanTimer = 0
    sim.scanAngle = r.angle
    return
  }
  // Nudge robot away from walls if stuck at edge
  const wallMargin = 0.6
  if (r.x < wallMargin) r.x = wallMargin
  if (r.x > venue.width - wallMargin) r.x = venue.width - wallMargin
  if (r.z < wallMargin) r.z = wallMargin
  if (r.z > venue.height - wallMargin) r.z = venue.height - wallMargin
  // Hand off to Nearest-Neighbor behavior for collection phase
  if (sim.collectionQueue.length > 0) {
    sim.collectionQueue.sort((a, b) => distXZ(r, a) - distXZ(r, b))
    const target = sim.collectionQueue[0]
    const path = aStarSearch(sim.occGrid, r.x, r.z, target.x, target.z)
    if (path && path.length > 1) {
      sim.currentPath = path; sim.pathIdx = 1; r.state = 'NAVIGATING'
      return
    } else {
      rotateOrDropItem(sim)
      return planNextBoustCollection(sim, venue, _depth + 1)
    }
  }
  // No items in queue — scan for more or explore unseen
  r.state = 'SCANNING'
  sim.scanTimer = 0
  sim.scanAngle = r.angle
}

function navigateToDump(sim) {
  const dumpX = sim.dumpStation.x, dumpZ = sim.dumpStation.z
  const path = aStarSearch(sim.occGrid, sim.robot.x, sim.robot.z, dumpX, dumpZ)
  if (path && path.length > 1) {
    sim.currentPath = path
    sim.pathIdx = 1
    sim.robot.state = 'NAVIGATING_DUMP'
  } else {
    // Fallback: direct
    sim.currentPath = [{ x: sim.robot.x, z: sim.robot.z }, { x: dumpX, z: dumpZ }]
    sim.pathIdx = 1
    sim.robot.state = 'NAVIGATING_DUMP'
  }
}

function returnToCoverage(sim) {
  const r = sim.robot
  if (sim.returnPoint) {
    const rp = sim.returnPoint
    if (rp.sweepDir !== undefined) { sim.sweepDir = rp.sweepDir; sim.sweepZ = rp.sweepZ }
    // Find the nearest waypoint on the coverage path from current position
    // Search from saved index forward — pick closest, but never go backwards
    if (sim.coveragePath && sim.coveragePath.length > 0) {
      const startIdx = rp.coverageIdx !== undefined ? rp.coverageIdx : (sim.coverageIdx || 0)
      let bestIdx = startIdx, bestDist = Infinity
      for (let i = startIdx; i < sim.coveragePath.length; i++) {
        const d = distXZ(r, sim.coveragePath[i])
        if (d < bestDist) { bestDist = d; bestIdx = i }
        if (d > bestDist + 2) break
      }
      // If the best waypoint is behind us (farther than 3m), skip ahead to the next one past it
      if (bestDist > 3 && bestIdx + 1 < sim.coveragePath.length) bestIdx++
      sim.coverageIdx = bestIdx
    }
    sim.returnPoint = null
  }
  r.state = 'COVERAGE'
  sim.currentPath = []
  sim.pathIdx = 0
}


// ═══════════════════════════════════════════════════════════════
// SECTION 4: 3D SCENE COMPONENTS
// ═══════════════════════════════════════════════════════════════

function Floor({ width, height }) {
  return (
    <group>
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[width / 2, -0.01, height / 2]}>
        <planeGeometry args={[width, height]} />
        <meshStandardMaterial color="#e2e8f0" />
      </mesh>
      <gridHelper args={[Math.max(width, height), Math.max(width, height), '#cbd5e1', '#cbd5e1']}
        position={[width / 2, 0, height / 2]} />
    </group>
  )
}

function Walls({ width, height }) {
  const wallH = 0.3, wallT = 0.1
  return (
    <group>
      <mesh position={[width / 2, wallH / 2, 0]}><boxGeometry args={[width, wallH, wallT]} /><meshStandardMaterial color="#64748b" transparent opacity={0.6} /></mesh>
      <mesh position={[width / 2, wallH / 2, height]}><boxGeometry args={[width, wallH, wallT]} /><meshStandardMaterial color="#64748b" transparent opacity={0.6} /></mesh>
      <mesh position={[0, wallH / 2, height / 2]}><boxGeometry args={[wallT, wallH, height]} /><meshStandardMaterial color="#64748b" transparent opacity={0.6} /></mesh>
      <mesh position={[width, wallH / 2, height / 2]}><boxGeometry args={[wallT, wallH, height]} /><meshStandardMaterial color="#64748b" transparent opacity={0.6} /></mesh>
    </group>
  )
}

function InteriorWalls({ walls }) {
  if (!walls || walls.length === 0) return null
  const wallH = 0.35
  return (
    <group>
      {walls.map((w, i) => {
        const dx = w.x2 - w.x1, dz = w.z2 - w.z1
        const len = Math.sqrt(dx * dx + dz * dz)
        const cx = (w.x1 + w.x2) / 2, cz = (w.z1 + w.z2) / 2
        const angle = Math.atan2(dx, dz)
        const thickness = w.thickness || 0.15
        return (
          <mesh key={i} position={[cx, wallH / 2, cz]} rotation={[0, angle, 0]}>
            <boxGeometry args={[thickness, wallH, len]} />
            <meshStandardMaterial color="#475569" />
          </mesh>
        )
      })}
    </group>
  )
}

function WasteItemMesh({ item, isSetup, onPlace, onRemove }) {
  const meshRef = useRef()
  const [hovered, setHovered] = useState(false)

  // Detailed 3D waste item models — detected items are brighter and larger
  const yOff = item.shape === 'flat' ? 0.003 : 0.0
  const isDetectedUncollected = item.detected && !item.collected
  const emissive = isDetectedUncollected ? '#ffffff' : '#000000'
  const emissiveIntensity = isDetectedUncollected ? 0.6 : 0
  const scale = isDetectedUncollected ? 1.8 : 1.0
  const matProps = { color: item.color, emissive, emissiveIntensity }

  const model = useMemo(() => {
    switch (item.shape) {
      case 'cylinder': // Aluminum Can — cylinder with rim rings
        return (
          <group>
            <mesh><cylinderGeometry args={[0.028, 0.028, 0.1, 12]} /><meshStandardMaterial {...matProps} metalness={0.7} roughness={0.3} /></mesh>
            <mesh position={[0, 0.05, 0]}><torusGeometry args={[0.028, 0.004, 6, 12]} /><meshStandardMaterial {...matProps} metalness={0.8} roughness={0.2} /></mesh>
            <mesh position={[0, -0.05, 0]}><torusGeometry args={[0.028, 0.004, 6, 12]} /><meshStandardMaterial {...matProps} metalness={0.8} roughness={0.2} /></mesh>
          </group>
        )
      case 'tallcylinder': // Bottle — body + narrow neck + cap
        return (
          <group>
            <mesh position={[0, 0, 0]}><cylinderGeometry args={[0.025, 0.028, 0.12, 10]} /><meshStandardMaterial {...matProps} transparent opacity={0.85} /></mesh>
            <mesh position={[0, 0.08, 0]}><cylinderGeometry args={[0.012, 0.02, 0.04, 8]} /><meshStandardMaterial {...matProps} transparent opacity={0.85} /></mesh>
            <mesh position={[0, 0.105, 0]}><cylinderGeometry args={[0.013, 0.013, 0.015, 8]} /><meshStandardMaterial color="#ffffff" /></mesh>
          </group>
        )
      case 'cone': // Paper Cup — truncated cone
        return (
          <group>
            <mesh><cylinderGeometry args={[0.035, 0.025, 0.1, 10]} /><meshStandardMaterial {...matProps} /></mesh>
            <mesh position={[0, 0.05, 0]}><torusGeometry args={[0.035, 0.003, 4, 10]} /><meshStandardMaterial {...matProps} /></mesh>
          </group>
        )
      case 'sphere': // Apple Core / fruit — sphere with small stem
        return (
          <group>
            <mesh><sphereGeometry args={[0.035, 10, 10]} /><meshStandardMaterial {...matProps} /></mesh>
            <mesh position={[0, 0.035, 0]}><cylinderGeometry args={[0.003, 0.003, 0.02, 4]} /><meshStandardMaterial color="#5d4037" /></mesh>
          </group>
        )
      case 'box': // Food Container — box with lid line
        return (
          <group>
            <mesh><boxGeometry args={[0.09, 0.04, 0.07]} /><meshStandardMaterial {...matProps} /></mesh>
            <mesh position={[0, 0.015, 0]}><boxGeometry args={[0.092, 0.003, 0.072]} /><meshStandardMaterial color="#4a3728" /></mesh>
          </group>
        )
      case 'flat': // Napkin — crumpled flat shape
        return (
          <group rotation={[0, Math.random() * Math.PI, 0]}>
            <mesh rotation={[0.1, 0, 0.15]}><boxGeometry args={[0.08, 0.006, 0.07]} /><meshStandardMaterial {...matProps} /></mesh>
            <mesh position={[0.01, 0.005, 0.01]} rotation={[-0.2, 0.3, 0]}><boxGeometry args={[0.04, 0.005, 0.04]} /><meshStandardMaterial {...matProps} /></mesh>
          </group>
        )
      case 'curved': // Banana Peel — curved shape
        return (
          <group rotation={[Math.PI / 2, 0, 0]}>
            <mesh><torusGeometry args={[0.04, 0.012, 6, 12, Math.PI * 1.2]} /><meshStandardMaterial {...matProps} /></mesh>
            <mesh position={[0.03, 0, 0]}><sphereGeometry args={[0.015, 6, 6]} /><meshStandardMaterial {...matProps} /></mesh>
          </group>
        )
      default:
        return <mesh><boxGeometry args={[0.05, 0.05, 0.05]} /><meshStandardMaterial {...matProps} /></mesh>
    }
  }, [item.shape, item.detected])

  if (item.collected) return null

  return (
    <group position={[item.x, yOff + 0.04, item.z]}>
      <group
        ref={meshRef}
        scale={[scale, scale, scale]}
        onPointerOver={() => setHovered(true)}
        onPointerOut={() => setHovered(false)}
        onContextMenu={(e) => { e.stopPropagation(); if (isSetup && onRemove) onRemove(item.id) }}
      >
        {model}
      </group>
      {/* Pulsing ring for detected uncollected items */}
      {isDetectedUncollected && (
        <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.01, 0]}>
          <ringGeometry args={[0.08, 0.12, 16]} />
          <meshBasicMaterial color={BIN_COLORS[item.bin] || '#ffffff'} transparent opacity={0.5} depthWrite={false} />
        </mesh>
      )}
      {(hovered && isSetup) && (
        <Html center position={[0, 0.15, 0]}>
          <div className="bg-slate-800/90 text-white text-xs px-2 py-1 rounded whitespace-nowrap pointer-events-none">
            {item.label}
          </div>
        </Html>
      )}
    </group>
  )
}

function ObstacleMesh({ obs, isSetup, onRemove, onDrag }) {
  const groupRef = useRef()
  const dragState = useRef(null)

  const handlePointerDown = useCallback((e) => {
    if (!isSetup || !onDrag) return
    e.stopPropagation()
    dragState.current = { startX: e.point.x, startZ: e.point.z, origX: obs.x, origZ: obs.z }
    e.target.setPointerCapture(e.pointerId)
  }, [isSetup, onDrag, obs.x, obs.z])

  const handlePointerMove = useCallback((e) => {
    if (!dragState.current) return
    e.stopPropagation()
    const dx = e.point.x - dragState.current.startX
    const dz = e.point.z - dragState.current.startZ
    onDrag(obs.id, dragState.current.origX + dx, dragState.current.origZ + dz)
  }, [onDrag, obs.id])

  const handlePointerUp = useCallback(() => { dragState.current = null }, [])

  if (obs.type === 'table') {
    // Table: tabletop (flat box) + 4 cylindrical legs
    const tw = 1.2, td = 0.6, topH = 0.04, legR = 0.025, legH = 0.72
    return (
      <group ref={groupRef} position={[obs.x, 0, obs.z]}
        onPointerDown={handlePointerDown} onPointerMove={handlePointerMove} onPointerUp={handlePointerUp}
        onContextMenu={(e) => { e.stopPropagation(); if (isSetup && onRemove) onRemove(obs.id) }}>
        {/* Tabletop */}
        <mesh position={[0, legH + topH / 2, 0]}>
          <boxGeometry args={[tw, topH, td]} />
          <meshStandardMaterial color="#a16207" />
        </mesh>
        {/* 4 Legs */}
        {[[-tw/2+0.06, -td/2+0.06], [tw/2-0.06, -td/2+0.06], [-tw/2+0.06, td/2-0.06], [tw/2-0.06, td/2-0.06]].map(([lx, lz], i) => (
          <mesh key={i} position={[lx, legH / 2, lz]}>
            <cylinderGeometry args={[legR, legR, legH, 8]} />
            <meshStandardMaterial color="#78350f" />
          </mesh>
        ))}
      </group>
    )
  }
  if (obs.type === 'trashbin') {
    // Trash bin — cylinder with dark top rim
    return (
      <group ref={groupRef} position={[obs.x, 0, obs.z]}
        onPointerDown={handlePointerDown} onPointerMove={handlePointerMove} onPointerUp={handlePointerUp}
        onContextMenu={(e) => { e.stopPropagation(); if (isSetup && onRemove) onRemove(obs.id) }}>
        <mesh position={[0, 0.35, 0]}>
          <cylinderGeometry args={[0.22, 0.2, 0.7, 12]} />
          <meshStandardMaterial color="#374151" />
        </mesh>
        <mesh position={[0, 0.71, 0]}>
          <torusGeometry args={[0.22, 0.02, 6, 12]} />
          <meshStandardMaterial color="#1f2937" />
        </mesh>
      </group>
    )
  }
  if (obs.type === 'podium') {
    // Podium / lectern — tapered box with angled top
    return (
      <group ref={groupRef} position={[obs.x, 0, obs.z]}
        onPointerDown={handlePointerDown} onPointerMove={handlePointerMove} onPointerUp={handlePointerUp}
        onContextMenu={(e) => { e.stopPropagation(); if (isSetup && onRemove) onRemove(obs.id) }}>
        <mesh position={[0, 0.5, 0]}>
          <boxGeometry args={[0.6, 1.0, 0.5]} />
          <meshStandardMaterial color="#4a3728" />
        </mesh>
        <mesh position={[0, 1.02, 0.05]} rotation={[-0.3, 0, 0]}>
          <boxGeometry args={[0.65, 0.03, 0.45]} />
          <meshStandardMaterial color="#5d4037" />
        </mesh>
      </group>
    )
  }
  if (obs.type === 'planter') {
    // Planter — low wide box with green sphere on top
    return (
      <group ref={groupRef} position={[obs.x, 0, obs.z]}
        onPointerDown={handlePointerDown} onPointerMove={handlePointerMove} onPointerUp={handlePointerUp}
        onContextMenu={(e) => { e.stopPropagation(); if (isSetup && onRemove) onRemove(obs.id) }}>
        <mesh position={[0, 0.2, 0]}>
          <boxGeometry args={[0.5, 0.4, 0.5]} />
          <meshStandardMaterial color="#6b7280" />
        </mesh>
        <mesh position={[0, 0.55, 0]}>
          <sphereGeometry args={[0.3, 10, 10]} />
          <meshStandardMaterial color="#166534" />
        </mesh>
      </group>
    )
  }
  // Chair: seat + backrest + 4 legs
  const sw = 0.4, sd = 0.4, seatH = 0.03, legR = 0.02, legH = 0.44, backH = 0.35
  return (
    <group ref={groupRef} position={[obs.x, 0, obs.z]}
      onPointerDown={handlePointerDown} onPointerMove={handlePointerMove} onPointerUp={handlePointerUp}
      onContextMenu={(e) => { e.stopPropagation(); if (isSetup && onRemove) onRemove(obs.id) }}>
      {/* Seat */}
      <mesh position={[0, legH + seatH / 2, 0]}>
        <boxGeometry args={[sw, seatH, sd]} />
        <meshStandardMaterial color="#92400e" />
      </mesh>
      {/* Backrest */}
      <mesh position={[0, legH + seatH + backH / 2, -sd / 2 + 0.02]}>
        <boxGeometry args={[sw, backH, 0.03]} />
        <meshStandardMaterial color="#78350f" />
      </mesh>
      {/* 4 Legs */}
      {[[-sw/2+0.04, -sd/2+0.04], [sw/2-0.04, -sd/2+0.04], [-sw/2+0.04, sd/2-0.04], [sw/2-0.04, sd/2-0.04]].map(([lx, lz], i) => (
        <mesh key={i} position={[lx, legH / 2, lz]}>
          <cylinderGeometry args={[legR, legR, legH, 6]} />
          <meshStandardMaterial color="#78350f" />
        </mesh>
      ))}
    </group>
  )
}

function RobotModel({ robot }) {
  const groupRef = useRef()
  const armGroupRef = useRef()
  const simRef = useContext(SimRefContext)

  useFrame(() => {
    if (!groupRef.current) return
    const r = simRef?.current?.robot || robot
    groupRef.current.position.set(r.x, 0, r.z)
    groupRef.current.rotation.y = r.angle
    // Update arm position imperatively
    if (armGroupRef.current) {
      const armState = r.armState || 'stowed'
      const ay = armState === 'reaching' ? 0.12 : armState === 'gripping' ? 0.05 : armState === 'lifting' ? 0.15 : 0.18
      const az = armState === 'reaching' ? 0.2 : armState === 'gripping' ? 0.25 : armState === 'lifting' ? 0.15 : 0
      armGroupRef.current.position.y = ay
      armGroupRef.current.position.z = az
    }
  })

  const armY = 0.18
  const armZ = 0

  return (
    <group ref={groupRef}>
      {/* Body */}
      <mesh position={[0, ROBOT_HEIGHT / 2 + 0.02, 0]} castShadow>
        <boxGeometry args={[ROBOT_WIDTH, ROBOT_HEIGHT, ROBOT_DEPTH]} />
        <meshStandardMaterial color="#1e293b" />
      </mesh>
      {/* Wheels */}
      {[-1, 1].map(side => (
        <mesh key={side} position={[side * (ROBOT_WIDTH / 2 + 0.02), 0.04, 0]} rotation={[0, 0, Math.PI / 2]}>
          <cylinderGeometry args={[0.04, 0.04, 0.03, 12]} />
          <meshStandardMaterial color="#374151" />
        </mesh>
      ))}
      {/* Direction indicator */}
      <mesh position={[0, ROBOT_HEIGHT + 0.04, ROBOT_DEPTH / 2 - 0.05]} rotation={[Math.PI / 2, 0, 0]}>
        <coneGeometry args={[0.04, 0.08, 4]} />
        <meshStandardMaterial color="#22c55e" emissive="#22c55e" emissiveIntensity={0.3} />
      </mesh>
      {/* Arm */}
      <group ref={armGroupRef} position={[0, armY, armZ]}>
        <mesh>
          <cylinderGeometry args={[0.015, 0.015, 0.12, 6]} />
          <meshStandardMaterial color="#94a3b8" />
        </mesh>
        <mesh position={[0, -0.06, 0.04]} rotation={[Math.PI / 4, 0, 0]}>
          <boxGeometry args={[0.03, 0.08, 0.015]} />
          <meshStandardMaterial color="#94a3b8" />
        </mesh>
      </group>
      {/* Bins on back */}
      {[
        { color: BIN_COLORS.recyclable, x: -0.1 },
        { color: BIN_COLORS.compost, x: 0 },
        { color: BIN_COLORS.landfill, x: 0.1 },
      ].map((b, i) => (
        <mesh key={i} position={[b.x, ROBOT_HEIGHT + 0.04, -ROBOT_DEPTH / 2 + 0.05]}>
          <boxGeometry args={[0.06, 0.05, 0.06]} />
          <meshStandardMaterial color={b.color} />
        </mesh>
      ))}
    </group>
  )
}

function CameraFOVCone({ robot, visible }) {
  const groupRef = useRef()
  const simRef = useContext(SimRefContext)

  const geometry = useMemo(() => {
    const halfAngle = (CAMERA_FOV_DEG / 2) * Math.PI / 180
    const shape = new THREE.Shape()
    const segments = 20
    // Build cone in XZ plane directly (X = left/right, Z = forward)
    shape.moveTo(Math.sin(-halfAngle) * CAMERA_MIN_RANGE, Math.cos(-halfAngle) * CAMERA_MIN_RANGE)
    for (let i = 0; i <= segments; i++) {
      const a = -halfAngle + (i / segments) * (halfAngle * 2)
      shape.lineTo(Math.sin(a) * CAMERA_MAX_RANGE, Math.cos(a) * CAMERA_MAX_RANGE)
    }
    for (let i = segments; i >= 0; i--) {
      const a = -halfAngle + (i / segments) * (halfAngle * 2)
      shape.lineTo(Math.sin(a) * CAMERA_MIN_RANGE, Math.cos(a) * CAMERA_MIN_RANGE)
    }
    shape.closePath()
    return new THREE.ShapeGeometry(shape)
  }, [])

  useFrame(() => {
    if (!groupRef.current) return
    const r = simRef?.current?.robot || robot
    // Parent group: position + Y rotation
    groupRef.current.position.set(r.x, 0, r.z)
    groupRef.current.rotation.y = r.angle
  })

  if (!visible) return null
  return (
    <group ref={groupRef}>
      {/* Flatten the shape onto the XZ plane */}
      <mesh geometry={geometry} rotation={[Math.PI / 2, 0, 0]} position={[0, 0.02, 0]}>
        <meshBasicMaterial color="#fde047" transparent opacity={0.2} side={THREE.DoubleSide} depthWrite={false} />
      </mesh>
    </group>
  )
}

function LidarVisualization({ robot, visible }) {
  const groupRef = useRef()
  const timeRef = useRef(0)
  const simRef = useContext(SimRefContext)
  const [angles, setAngles] = useState(() => Array.from({ length: 8 }, (_, i) => i * GOLDEN_ANGLE))

  useFrame((_, delta) => {
    if (!groupRef.current || !visible) return
    timeRef.current += delta
    const r = simRef?.current?.robot || robot
    groupRef.current.position.set(r.x, 0.05, r.z)
    // Update angles every few frames for visual animation
    if (Math.floor(timeRef.current * 10) % 2 === 0) {
      setAngles(Array.from({ length: 8 }, (_, i) => i * GOLDEN_ANGLE + timeRef.current * (0.5 + i * 0.1)))
    }
  })

  if (!visible) return null
  return (
    <group ref={groupRef}>
      {angles.map((a, i) => (
        <Line key={i}
          points={[[0, 0, 0], [Math.sin(a) * LIDAR_MAX_RANGE, 0, Math.cos(a) * LIDAR_MAX_RANGE]]}
          color="#06b6d4" lineWidth={1.5} transparent opacity={0.3} />
      ))}
    </group>
  )
}

function SectorGrid({ sectors, activeSectorIdx, visible }) {
  if (!visible || !sectors.length) return null
  return (
    <group>
      {sectors.map((s, i) => (
        <group key={s.id}>
          {/* Sector boundary */}
          <Line
            points={[
              [s.x, 0.015, s.z], [s.x + s.width, 0.015, s.z],
              [s.x + s.width, 0.015, s.z + s.height], [s.x, 0.015, s.z + s.height],
              [s.x, 0.015, s.z],
            ]}
            color={s.completed ? '#4ade80' : '#cbd5e1'}
            lineWidth={2}
            dashed
            dashSize={0.3}
            gapSize={0.15}
          />
          {/* Sector label */}
          <Text position={[s.x + s.width / 2, 0.02, s.z + s.height / 2]} fontSize={0.3} color={s.completed ? '#4ade80' : '#64748b'} anchorX="center" anchorY="middle" rotation={[-Math.PI / 2, 0, 0]}>
            {s.id + 1}
          </Text>
          {/* Active sector highlight */}
          {i === activeSectorIdx && (
            <mesh position={[s.x + s.width / 2, 0.005, s.z + s.height / 2]} rotation={[-Math.PI / 2, 0, 0]}>
              <planeGeometry args={[s.width, s.height]} />
              <meshBasicMaterial color="#3b82f6" transparent opacity={0.1} depthWrite={false} />
            </mesh>
          )}
          {/* Completed sector */}
          {s.completed && (
            <mesh position={[s.x + s.width / 2, 0.004, s.z + s.height / 2]} rotation={[-Math.PI / 2, 0, 0]}>
              <planeGeometry args={[s.width, s.height]} />
              <meshBasicMaterial color="#22c55e" transparent opacity={0.07} depthWrite={false} />
            </mesh>
          )}
        </group>
      ))}
    </group>
  )
}

function PathVisualization({ coveragePath, currentPath, visible }) {
  if (!visible) return null
  return (
    <group>
      {coveragePath.length > 1 && (
        <Line
          points={coveragePath.map(p => [p.x, 0.03, p.z])}
          color="#93c5fd"
          lineWidth={1}
          dashed
          dashSize={0.15}
          gapSize={0.1}
        />
      )}
      {currentPath.length > 1 && (
        <Line
          points={currentPath.map(p => [p.x, 0.04, p.z])}
          color="#f59e0b"
          lineWidth={2}
        />
      )}
    </group>
  )
}

// POV Camera controller — updates the default camera to follow the robot
function POVCameraController({ simRef }) {
  const { camera } = useThree()
  useFrame(() => {
    const sim = simRef?.current
    if (!sim) return
    const r = sim.robot
    camera.position.set(r.x, 0.25, r.z)
    camera.rotation.set(0, 0, 0)
    camera.lookAt(r.x + Math.sin(r.angle) * 5, 0.1, r.z + Math.cos(r.angle) * 5)
  })
  return null
}

// Live path visualization that reads from simRef each frame
function LivePathVisualization({ simRef, visible }) {
  const [paths, setPaths] = useState({ coverage: [], current: [] })

  const frameCount = useRef(0)
  useFrame(() => {
    if (!simRef?.current || !visible) return
    frameCount.current++
    // Update paths every ~15 frames (~4Hz) to keep in sync without excessive re-renders
    if (frameCount.current % 15 !== 0) return
    const sim = simRef.current
    const newCoverage = sim.coveragePath || []
    const newCurrent = sim.currentPath || []
    setPaths({ coverage: [...newCoverage], current: [...newCurrent] })
  })

  if (!visible) return null
  const algoColor = (ALGORITHMS.find(a => a.id === simRef?.current?.algorithm) || ALGORITHMS[4]).color
  return (
    <group>
      {paths.coverage.length > 1 && (
        <Line points={paths.coverage.map(p => [p.x, 0.03, p.z])} color="#60a5fa" lineWidth={2} dashed dashSize={0.2} gapSize={0.1} />
      )}
      {paths.current.length > 1 && (
        <Line points={paths.current.map(p => [p.x, 0.04, p.z])} color={algoColor} lineWidth={3} />
      )}
    </group>
  )
}

function CostmapOverlay({ obstacles, visible, venue }) {
  if (!visible) return null
  return (
    <group>
      {obstacles.map(obs => {
        const w = (obs.type === 'table' ? 1.2 : 0.45) + INFLATION_RADIUS * 2
        const d = (obs.type === 'table' ? 0.6 : 0.45) + INFLATION_RADIUS * 2
        return (
          <mesh key={obs.id} position={[obs.x, 0.015, obs.z]} rotation={[-Math.PI / 2, 0, 0]}>
            <planeGeometry args={[w, d]} />
            <meshBasicMaterial color="#ef4444" transparent opacity={0.15} depthWrite={false} />
          </mesh>
        )
      })}
    </group>
  )
}

function DumpStationMesh({ position, bins }) {
  return (
    <group position={[position.x, 0, position.z]}>
      {/* Base */}
      <mesh position={[0, 0.15, 0]}>
        <boxGeometry args={[1.5, 0.3, 0.6]} />
        <meshStandardMaterial color="#475569" />
      </mesh>
      {/* Three bin slots */}
      {[
        { color: BIN_COLORS.recyclable, label: 'R', x: -0.45, count: bins.recyclable },
        { color: BIN_COLORS.compost, label: 'C', x: 0, count: bins.compost },
        { color: BIN_COLORS.landfill, label: 'L', x: 0.45, count: bins.landfill },
      ].map((b, i) => (
        <group key={i} position={[b.x, 0.3, 0]}>
          <mesh position={[0, 0.1, 0]}>
            <boxGeometry args={[0.4, 0.2, 0.4]} />
            <meshStandardMaterial color={b.color} transparent opacity={0.7} />
          </mesh>
          <Text position={[0, 0.25, 0.21]} fontSize={0.12} color="white" anchorX="center" anchorY="middle">
            {b.label}
          </Text>
        </group>
      ))}
    </group>
  )
}

function Particles({ particles }) {
  if (particles.length === 0) return null
  return (
    <group>
      {particles.map(p => (
        <mesh key={p.id} position={[p.x, p.y, p.z]}>
          <sphereGeometry args={[0.015, 4, 4]} />
          <meshBasicMaterial color={p.color} transparent opacity={Math.max(0, p.life * 2)} />
        </mesh>
      ))}
    </group>
  )
}

function ClickPlane({ venue, onClick, placementMode }) {
  if (!placementMode) return null
  return (
    <mesh
      position={[venue.width / 2, 0, venue.height / 2]}
      rotation={[-Math.PI / 2, 0, 0]}
      onClick={(e) => {
        e.stopPropagation()
        onClick(e.point.x, e.point.z)
      }}
    >
      <planeGeometry args={[venue.width, venue.height]} />
      <meshBasicMaterial visible={false} />
    </mesh>
  )
}

// Robot start marker for setup — draggable
function RobotStartMarker({ position, onDrag }) {
  const dragState = useRef(null)
  return (
    <group position={[position.x, 0.01, position.z]}
      onPointerDown={(e) => { e.stopPropagation(); dragState.current = { sx: e.point.x, sz: e.point.z, ox: position.x, oz: position.z }; e.target.setPointerCapture(e.pointerId) }}
      onPointerMove={(e) => { if (!dragState.current || !onDrag) return; e.stopPropagation(); onDrag(dragState.current.ox + e.point.x - dragState.current.sx, dragState.current.oz + e.point.z - dragState.current.sz) }}
      onPointerUp={() => { dragState.current = null }}>
      <mesh rotation={[-Math.PI / 2, 0, 0]}>
        <circleGeometry args={[0.3, 3]} />
        <meshBasicMaterial color="#22c55e" transparent opacity={0.6} side={THREE.DoubleSide} />
      </mesh>
      <Text position={[0, 0.3, 0]} fontSize={0.15} color="#22c55e" anchorX="center">
        ROBOT
      </Text>
    </group>
  )
}

// Heatmap overlay showing waste density
function HeatmapOverlay({ wasteItems, venueW, venueH, visible }) {
  const meshRef = useRef()
  const texture = useMemo(() => {
    const res = 64
    const data = new Uint8Array(res * res * 4)
    const cellW = venueW / res, cellH = venueH / res
    // Compute density
    const density = new Float32Array(res * res)
    let maxD = 0
    for (const w of wasteItems) {
      if (w.collected) continue
      const gx = clamp(Math.floor(w.x / cellW), 0, res - 1)
      const gz = clamp(Math.floor(w.z / cellH), 0, res - 1)
      // Gaussian spread
      for (let dz = -3; dz <= 3; dz++) {
        for (let dx = -3; dx <= 3; dx++) {
          const nx = gx + dx, nz = gz + dz
          if (nx < 0 || nx >= res || nz < 0 || nz >= res) continue
          const dist = Math.sqrt(dx * dx + dz * dz)
          density[nz * res + nx] += Math.exp(-dist * dist / 3)
        }
      }
    }
    for (let i = 0; i < density.length; i++) if (density[i] > maxD) maxD = density[i]
    if (maxD === 0) maxD = 1
    for (let i = 0; i < res * res; i++) {
      const v = density[i] / maxD
      const idx = i * 4
      // Green -> Yellow -> Red gradient
      data[idx] = Math.floor(v > 0.5 ? 255 : v * 2 * 255)     // R
      data[idx + 1] = Math.floor(v > 0.5 ? (1 - (v - 0.5) * 2) * 255 : 255) // G
      data[idx + 2] = 0  // B
      data[idx + 3] = Math.floor(v * 180)  // A
    }
    const tex = new THREE.DataTexture(data, res, res, THREE.RGBAFormat)
    tex.needsUpdate = true
    return tex
  }, [wasteItems, venueW, venueH])

  if (!visible) return null
  return (
    <mesh ref={meshRef} position={[venueW / 2, 0.02, venueH / 2]} rotation={[-Math.PI / 2, 0, 0]}>
      <planeGeometry args={[venueW, venueH]} />
      <meshBasicMaterial map={texture} transparent depthWrite={false} />
    </mesh>
  )
}

// ═══════════════════════════════════════════════════════════════
// SECTION 5: UI OVERLAY COMPONENTS
// ═══════════════════════════════════════════════════════════════

function SetupPanel({ state, dispatch }) {
  const [scatterCount, setScatterCount] = useState(50)

  const handleScatter = () => {
    const items = []
    for (let i = 0; i < scatterCount; i++) {
      const wt = WASTE_TYPES[Math.floor(Math.random() * WASTE_TYPES.length)]
      const x = 1 + Math.random() * (state.venue.width - 2)
      const z = 2 + Math.random() * (state.venue.height - 3) // Avoid dump station area
      // Check not on obstacle
      const onObs = state.obstacles.some(o => {
        const hw = (o.type === 'table' ? 0.7 : 0.3)
        return Math.abs(x - o.x) < hw && Math.abs(z - o.z) < hw
      })
      if (!onObs) {
        items.push({ id: nextId(), ...wt, x, z, collected: false, detected: false, confidence: 0, picking: false })
      }
    }
    dispatch({ type: 'SCATTER_RANDOM', items })
  }

  const loadPreset = (name) => {
    const vw = state.venue.width, vh = state.venue.height
    let obs = [], waste = [], interiorWalls = []
    if (name === 'conference') {
      // 3x3 grid of tables with chairs, centered in venue
      const rows = 3, cols = 3
      const xStart = vw * 0.2, xEnd = vw * 0.8
      const zStart = vh * 0.2, zEnd = vh * 0.8
      const xSpacing = (xEnd - xStart) / (cols - 1)
      const zSpacing = (zEnd - zStart) / (rows - 1)
      for (let r = 0; r < rows; r++) {
        for (let c = 0; c < cols; c++) {
          const tx = xStart + c * xSpacing
          const tz = zStart + r * zSpacing
          obs.push({ id: nextId(), type: 'table', x: tx, z: tz, width: 1.2, depth: 0.6 })
          obs.push({ id: nextId(), type: 'chair', x: tx - 0.8, z: tz, width: 0.45, depth: 0.45 })
          obs.push({ id: nextId(), type: 'chair', x: tx + 0.8, z: tz, width: 0.45, depth: 0.45 })
        }
      }
      for (let i = 0; i < 30; i++) {
        const wt = WASTE_TYPES[Math.floor(Math.random() * WASTE_TYPES.length)]
        waste.push(placeWasteItem(wt, 2 + Math.random() * (vw - 4), 2 + Math.random() * (vh - 4), obs, vw, vh, interiorWalls))
      }
    } else if (name === 'arena') {
      for (let a = 0; a < 8; a++) {
        const angle = (a / 8) * Math.PI * 2
        const cx = vw / 2 + Math.cos(angle) * Math.min(vw, vh) * 0.35
        const cz = vh / 2 + Math.sin(angle) * Math.min(vw, vh) * 0.35
        obs.push({ id: nextId(), type: a % 2 === 0 ? 'table' : 'chair', x: cx, z: cz, width: a % 2 === 0 ? 1.2 : 0.45, depth: a % 2 === 0 ? 0.6 : 0.45 })
      }
      for (let i = 0; i < 25; i++) {
        const wt = WASTE_TYPES[Math.floor(Math.random() * WASTE_TYPES.length)]
        waste.push(placeWasteItem(wt, vw * 0.3 + Math.random() * vw * 0.4, vh * 0.3 + Math.random() * vh * 0.4, obs, vw, vh, interiorWalls))
      }
    } else if (name === 'dense') {
      for (let i = 0; i < 15; i++) {
        const types = ['table', 'chair', 'trashbin', 'planter']
        const t = types[Math.floor(Math.random() * types.length)]
        const sizes = { table: [1.2, 0.6], chair: [0.45, 0.45], trashbin: [0.5, 0.5], planter: [0.5, 0.5] }
        obs.push({ id: nextId(), type: t, x: 2 + Math.random() * (vw - 4), z: 2 + Math.random() * (vh - 4), width: sizes[t][0], depth: sizes[t][1] })
      }
      for (let i = 0; i < 200; i++) {
        const wt = WASTE_TYPES[Math.floor(Math.random() * WASTE_TYPES.length)]
        waste.push(placeWasteItem(wt, 1 + Math.random() * (vw - 2), 2 + Math.random() * (vh - 3), obs, vw, vh, interiorWalls))
      }
    } else if (name === 'hallway') {
      // L-shaped venue with interior walls forming two connected rooms
      // Room A: left half, full height. Room B: right half, bottom half only.
      // Interior wall across the middle vertically with a door gap
      const wallX = vw * 0.45
      const doorGap = 2.5
      const doorCenter = vh * 0.35
      interiorWalls = [
        // Vertical wall with door gap
        { x1: wallX, z1: 0, x2: wallX, z2: doorCenter - doorGap / 2, thickness: 0.15 },
        { x1: wallX, z1: doorCenter + doorGap / 2, x2: wallX, z2: vh * 0.55, thickness: 0.15 },
        // Horizontal wall closing off upper-right quadrant
        { x1: wallX, z1: vh * 0.55, x2: vw, z2: vh * 0.55, thickness: 0.15 },
      ]
      // Room A furniture
      obs.push({ id: nextId(), type: 'table', x: vw * 0.15, z: vh * 0.25, width: 1.2, depth: 0.6 })
      obs.push({ id: nextId(), type: 'chair', x: vw * 0.15 - 0.8, z: vh * 0.25, width: 0.45, depth: 0.45 })
      obs.push({ id: nextId(), type: 'chair', x: vw * 0.15 + 0.8, z: vh * 0.25, width: 0.45, depth: 0.45 })
      obs.push({ id: nextId(), type: 'table', x: vw * 0.2, z: vh * 0.7, width: 1.2, depth: 0.6 })
      obs.push({ id: nextId(), type: 'trashbin', x: 1, z: vh - 1.5, width: 0.5, depth: 0.5 })
      obs.push({ id: nextId(), type: 'planter', x: wallX - 1, z: 1, width: 0.5, depth: 0.5 })
      // Room B furniture (lower-right)
      obs.push({ id: nextId(), type: 'table', x: vw * 0.7, z: vh * 0.25, width: 1.2, depth: 0.6 })
      obs.push({ id: nextId(), type: 'podium', x: vw * 0.8, z: vh * 0.1, width: 0.6, depth: 0.5 })
      obs.push({ id: nextId(), type: 'trashbin', x: vw - 1, z: 1, width: 0.5, depth: 0.5 })
      for (let i = 0; i < 50; i++) {
        const wt = WASTE_TYPES[Math.floor(Math.random() * WASTE_TYPES.length)]
        let x, z
        if (Math.random() > 0.4) {
          x = 1 + Math.random() * (wallX - 2); z = 1 + Math.random() * (vh - 2)
        } else {
          x = wallX + 1 + Math.random() * (vw - wallX - 2); z = 1 + Math.random() * (vh * 0.55 - 2)
        }
        waste.push(placeWasteItem(wt, x, z, obs, vw, vh, interiorWalls))
      }
    } else if (name === 'banquet') {
      // Multi-room banquet: main hall + side room connected by doorway
      const wallZ = vh * 0.65
      const doorGap = 2.5
      interiorWalls = [
        { x1: 0, z1: wallZ, x2: vw * 0.4, z2: wallZ, thickness: 0.15 },
        { x1: vw * 0.4 + doorGap, z1: wallZ, x2: vw, z2: wallZ, thickness: 0.15 },
      ]
      // Main hall (bottom) — tables in grid
      const tablePositions = []
      for (let r = 0; r < 2; r++) {
        for (let c = 0; c < 4; c++) {
          const cx = 2.5 + c * (vw - 5) / 3, cz = 2.5 + r * (wallZ - 5) / 1.5
          tablePositions.push([cx, cz])
          obs.push({ id: nextId(), type: 'table', x: cx, z: cz, width: 1.2, depth: 0.6 })
          for (let a = 0; a < 4; a++) {
            const angle = (a / 4) * Math.PI * 2
            obs.push({ id: nextId(), type: 'chair', x: cx + Math.cos(angle) * 0.9, z: cz + Math.sin(angle) * 0.9, width: 0.45, depth: 0.45 })
          }
        }
      }
      // Side room (top) — lounge area
      obs.push({ id: nextId(), type: 'table', x: vw * 0.3, z: wallZ + (vh - wallZ) * 0.5, width: 1.2, depth: 0.6 })
      obs.push({ id: nextId(), type: 'table', x: vw * 0.7, z: wallZ + (vh - wallZ) * 0.5, width: 1.2, depth: 0.6 })
      obs.push({ id: nextId(), type: 'planter', x: 1, z: wallZ + 1, width: 0.5, depth: 0.5 })
      obs.push({ id: nextId(), type: 'planter', x: vw - 1, z: wallZ + 1, width: 0.5, depth: 0.5 })
      obs.push({ id: nextId(), type: 'trashbin', x: vw * 0.5, z: wallZ + 0.8, width: 0.5, depth: 0.5 })
      obs.push({ id: nextId(), type: 'podium', x: vw / 2, z: 1.5, width: 0.6, depth: 0.5 })
      for (let i = 0; i < 80; i++) {
        const wt = WASTE_TYPES[Math.floor(Math.random() * WASTE_TYPES.length)]
        let x, z
        if (Math.random() > 0.3) {
          const [tx, tz] = tablePositions[Math.floor(Math.random() * tablePositions.length)]
          x = tx + (Math.random() - 0.5) * 3; z = tz + (Math.random() - 0.5) * 3
        } else {
          x = 1 + Math.random() * (vw - 2); z = wallZ + 1 + Math.random() * (vh - wallZ - 2)
        }
        waste.push(placeWasteItem(wt, x, z, obs, vw, vh, interiorWalls))
      }
    } else if (name === 'demo') {
      // Demo: 40x30m, 40 items in 4 clusters around tables
      dispatch({ type: 'SET_VENUE', venue: { width: 40, height: 30 } })
      const clusterCenters = [[10, 8], [30, 8], [10, 22], [30, 22]]
      for (const [cx, cz] of clusterCenters) {
        for (let i = 0; i < 3; i++) {
          obs.push({ id: nextId(), type: 'table', x: cx + (i - 1) * 2.5, z: cz, width: 1.2, depth: 0.6 })
          obs.push({ id: nextId(), type: 'chair', x: cx + (i - 1) * 2.5 - 0.8, z: cz + 0.8, width: 0.45, depth: 0.45 })
          obs.push({ id: nextId(), type: 'chair', x: cx + (i - 1) * 2.5 + 0.8, z: cz + 0.8, width: 0.45, depth: 0.45 })
        }
      }
      for (let i = 0; i < 40; i++) {
        const wt = WASTE_TYPES[Math.floor(Math.random() * WASTE_TYPES.length)]
        const cluster = clusterCenters[Math.floor(Math.random() * clusterCenters.length)]
        waste.push(placeWasteItem(wt, cluster[0] + (Math.random() - 0.5) * 6, cluster[1] + (Math.random() - 0.5) * 6, obs, 40, 30, interiorWalls))
      }
    } else if (name === 'challenge') {
      // Challenge: 40x30m, corner dump station, 4 tight clusters
      dispatch({ type: 'SET_VENUE', venue: { width: 40, height: 30 } })
      dispatch({ type: 'SET_DUMP_STATION', x: 2, z: 2 })
      const clusters = [[10, 10], [30, 10], [10, 20], [30, 20]]
      for (const [cx, cz] of clusters) {
        obs.push({ id: nextId(), type: 'table', x: cx, z: cz, width: 1.2, depth: 0.6 })
        obs.push({ id: nextId(), type: 'table', x: cx + 2, z: cz, width: 1.2, depth: 0.6 })
        obs.push({ id: nextId(), type: 'chair', x: cx - 1, z: cz, width: 0.45, depth: 0.45 })
        obs.push({ id: nextId(), type: 'chair', x: cx + 3, z: cz, width: 0.45, depth: 0.45 })
      }
      for (let i = 0; i < 40; i++) {
        const wt = WASTE_TYPES[Math.floor(Math.random() * WASTE_TYPES.length)]
        const cluster = clusters[Math.floor(Math.random() * clusters.length)]
        waste.push(placeWasteItem(wt, cluster[0] + (Math.random() - 0.5) * 4, cluster[1] + (Math.random() - 0.5) * 4, obs, 40, 30, interiorWalls))
      }
    } else if (name === 'stress') {
      // Stress: 40x30 L-shaped with narrow corridor, 50 items non-uniform
      dispatch({ type: 'SET_VENUE', venue: { width: 40, height: 30 } })
      interiorWalls = [
        { x1: 20, z1: 0, x2: 20, z2: 12, thickness: 0.15 },
        { x1: 20, z1: 15, x2: 20, z2: 30, thickness: 0.15 },
      ]
      for (let i = 0; i < 6; i++) {
        obs.push({ id: nextId(), type: 'table', x: 5 + i * 2.5, z: 7, width: 1.2, depth: 0.6 })
        obs.push({ id: nextId(), type: 'table', x: 25 + i * 2.5, z: 20, width: 1.2, depth: 0.6 })
      }
      // 70% items in left wing, 30% in right wing
      for (let i = 0; i < 35; i++) {
        const wt = WASTE_TYPES[Math.floor(Math.random() * WASTE_TYPES.length)]
        waste.push(placeWasteItem(wt, 2 + Math.random() * 16, 2 + Math.random() * 26, obs, 40, 30, interiorWalls))
      }
      for (let i = 0; i < 15; i++) {
        const wt = WASTE_TYPES[Math.floor(Math.random() * WASTE_TYPES.length)]
        waste.push(placeWasteItem(wt, 22 + Math.random() * 16, 2 + Math.random() * 26, obs, 40, 30, interiorWalls))
      }
    } else if (name === 'skewed') {
      // Skewed (Realistic) — 80 items, recyclables dominant: 50% recyclable, 30% compost, 20% landfill
      const recyclableTypes = WASTE_TYPES.filter(w => w.bin === 'recyclable')
      const compostTypes = WASTE_TYPES.filter(w => w.bin === 'compost')
      const landfillTypes = WASTE_TYPES.filter(w => w.bin === 'landfill')
      // Tables and bins first so waste placement can avoid them
      for (let i = 0; i < 4; i++) {
        obs.push({ id: nextId(), type: 'table', x: 3 + i * 4, z: vh / 2, width: 1.2, depth: 0.6 })
      }
      obs.push({ id: nextId(), type: 'trashbin', x: 1, z: vh - 1.5, width: 0.5, depth: 0.5 })
      obs.push({ id: nextId(), type: 'trashbin', x: vw - 1, z: vh - 1.5, width: 0.5, depth: 0.5 })
      const makeItem = (types) => {
        const wt = types[Math.floor(Math.random() * types.length)]
        return placeWasteItem(wt, 1 + Math.random() * (vw - 2), 2 + Math.random() * (vh - 3), obs, vw, vh, interiorWalls)
      }
      for (let i = 0; i < 40; i++) waste.push(makeItem(recyclableTypes)) // 50% recyclable
      for (let i = 0; i < 24; i++) waste.push(makeItem(compostTypes))    // 30% compost
      for (let i = 0; i < 16; i++) waste.push(makeItem(landfillTypes))   // 20% landfill
    }
    dispatch({ type: 'CLEAR_ALL' })
    dispatch({ type: 'LOAD_PRESET', payload: { obstacles: obs, wasteItems: waste, walls: interiorWalls } })
  }
  useEffect(() => { window.__loadPreset = loadPreset }, [loadPreset])

  return (
    <div className="absolute top-0 left-0 w-72 h-full bg-slate-900/90 text-white p-4 overflow-y-auto backdrop-blur-sm z-10">
      <h2 className="text-lg font-bold mb-4 text-blue-400">Setup</h2>

      {/* Venue Size */}
      <div className="mb-4">
        <label className="text-xs text-slate-400 block">Width: {state.venue.width}m</label>
        <input type="range" min={10} max={50} value={state.venue.width}
          onChange={e => dispatch({ type: 'SET_VENUE', venue: { ...state.venue, width: +e.target.value } })}
          className="w-full" />
        <label className="text-xs text-slate-400 block mt-1">Height: {state.venue.height}m</label>
        <input type="range" min={10} max={50} value={state.venue.height}
          onChange={e => dispatch({ type: 'SET_VENUE', venue: { ...state.venue, height: +e.target.value } })}
          className="w-full" />
      </div>

      {/* Algorithm Selection */}
      <div className="mb-4">
        <h3 className="text-sm font-semibold mb-2 text-slate-300">Algorithm</h3>
        <div className="space-y-1">
          {ALGORITHMS.map(algo => (
            <button key={algo.id}
              className={`w-full text-left text-xs px-3 py-1.5 rounded border flex items-center gap-2 ${state.algorithm === algo.id ? 'border-blue-400 bg-blue-900/50' : 'border-slate-700 hover:border-slate-500'}`}
              onClick={() => dispatch({ type: 'SET_ALGORITHM', algorithm: algo.id })}>
              <span className="w-2 h-2 rounded-full flex-shrink-0" style={{ backgroundColor: algo.color }} />
              <span className="font-medium">{algo.label}</span>
            </button>
          ))}
        </div>
      </div>

      {/* Smart Bin Balancing Toggle (RECLAIM only) */}
      {(state.algorithm === 'reclaim') && (
        <div className="mb-4">
          <label className="flex items-center gap-2 cursor-pointer">
            <div className={`w-8 h-4 rounded-full flex items-center px-0.5 transition-colors ${state.smartBinBalancing ? 'bg-green-600' : 'bg-slate-700'}`}
              onClick={() => dispatch({ type: 'SET_BIN_BALANCING', value: !state.smartBinBalancing })}>
              <div className={`w-3 h-3 rounded-full bg-white transition-transform ${state.smartBinBalancing ? 'translate-x-4' : 'translate-x-0'}`} />
            </div>
            <span className="text-xs text-slate-300">Smart Bin Balancing</span>
          </label>
          <p className="text-[10px] text-slate-500 mt-1">Defers pickup of items for near-full bins to reduce dump trips</p>
        </div>
      )}

      {/* Waste Palette */}
      <div className="mb-4">
        <h3 className="text-sm font-semibold mb-2 text-slate-300">Waste Items</h3>
        <div className="grid grid-cols-2 gap-1">
          {WASTE_TYPES.map(wt => (
            <button key={wt.typeId}
              className={`text-xs px-2 py-1.5 rounded border transition-all ${state.placementMode === wt.typeId ? 'border-blue-400 bg-blue-900/50' : 'border-slate-700 hover:border-slate-500'}`}
              onClick={() => dispatch({ type: 'SET_PLACEMENT_MODE', mode: state.placementMode === wt.typeId ? null : wt.typeId })}>
              <span className="inline-block w-2 h-2 rounded-full mr-1" style={{ backgroundColor: wt.color }} />
              {wt.label}
            </button>
          ))}
        </div>
      </div>

      {/* Obstacles */}
      <div className="mb-4">
        <h3 className="text-sm font-semibold mb-2 text-slate-300">Obstacles</h3>
        <div className="grid grid-cols-3 gap-1">
          {[['table', 'Table'], ['chair', 'Chair'], ['trashbin', 'Trash Bin'], ['podium', 'Podium'], ['planter', 'Planter']].map(([type, label]) => (
            <button key={type} className={`text-xs px-2 py-1.5 rounded border ${state.placementMode === type ? 'border-blue-400 bg-blue-900/50' : 'border-slate-700 hover:border-slate-500'}`}
              onClick={() => dispatch({ type: 'SET_PLACEMENT_MODE', mode: state.placementMode === type ? null : type })}>
              {label}
            </button>
          ))}
        </div>
      </div>

      {/* Presets */}
      <div className="mb-4">
        <h3 className="text-sm font-semibold mb-2 text-slate-300">Presets</h3>
        <div className="grid grid-cols-2 gap-1">
          {[['empty', 'Empty'], ['conference', 'Conference'], ['arena', 'Arena'], ['dense', 'Dense Mess'], ['hallway', 'L-Hallway'], ['banquet', 'Banquet Hall'], ['skewed', 'Skewed'], ['demo', 'Demo 40x30'], ['challenge', 'Challenge'], ['stress', 'Stress L']].map(([k, label]) => (
            <button key={k} className="text-xs px-2 py-1.5 rounded border border-slate-700 hover:border-slate-500"
              onClick={() => k === 'empty' ? dispatch({ type: 'CLEAR_ALL' }) : loadPreset(k)}>
              {label}
            </button>
          ))}
        </div>
      </div>

      {/* Scatter */}
      <div className="mb-4">
        <h3 className="text-sm font-semibold mb-2 text-slate-300">Scatter Random</h3>
        <label className="text-xs text-slate-400">Count: {scatterCount}</label>
        <input type="range" min={10} max={200} value={scatterCount} onChange={e => setScatterCount(+e.target.value)} className="w-full" />
        <button className="w-full mt-1 text-xs px-3 py-1.5 rounded bg-blue-600 hover:bg-blue-500" onClick={handleScatter}>
          Scatter {scatterCount} Items
        </button>
      </div>

      {/* Clear */}
      <button className="w-full text-xs px-3 py-1.5 rounded border border-red-700 text-red-400 hover:bg-red-900/30 mb-4" onClick={() => dispatch({ type: 'CLEAR_ALL' })}>
        Clear All
      </button>

      {/* Summary */}
      <div className="text-xs text-slate-400">
        <p>{state.wasteItems.length} waste items</p>
        <p>{state.obstacles.length} obstacles</p>
        <p className="mt-1 text-slate-500">Click an item type, then click on the floor to place. Right-click to delete.</p>
      </div>
    </div>
  )
}

function getRobotStateLabel(state, sim) {
  // Full Boustrophedon post-sweep shows COLLECTING instead of COVERING
  if (sim?.sweepComplete && (state === 'COVERAGE' || state === 'NAVIGATING_SECTOR')) return 'COLLECTING'
  // RECLAIM mode label
  if (sim?.reclaimMode && sim?.algorithm === 'reclaim' && state !== 'DONE') {
    const modeLabels = { COLLECT: 'COLLECTING', SCAN: 'SCANNING', SWEEP: 'SWEEPING', DONE: 'COMPLETE' }
    return modeLabels[sim.reclaimMode] || state
  }
  switch (state) {
    case 'IDLE': return 'INITIALIZING'
    case 'COVERAGE': return 'COVERING'
    case 'NAVIGATING': return 'PICKING'
    case 'NAVIGATING_SECTOR': return 'COVERING'
    case 'NAVIGATING_DUMP': return 'NAVIGATING TO DUMP'
    case 'NAVIGATING_VIEWPOINT': return 'EXPLORING'
    case 'SCAN_NAVIGATE': return 'EXPLORING'
    case 'SWEEP_NAVIGATE': return 'SWEEPING'
    case 'SWEEPING': return 'SWEEPING'
    case 'COLLECTING': return 'COLLECTING'
    case 'APPROACHING': return 'PICKING'
    case 'PICKING': return 'PICKING'
    case 'DUMPING': return 'DUMPING'
    case 'SCANNING': return 'SCANNING'
    case 'RECOVERING_DEFERRED': return 'RECOVERING DEFERRED'
    case 'DONE': return 'COMPLETE'
    default: return state
  }
}

function getRobotStateBadgeColor(state) {
  switch (state) {
    case 'COVERAGE': case 'NAVIGATING_SECTOR': case 'SCANNING': case 'NAVIGATING_VIEWPOINT': case 'SCAN_NAVIGATE': return 'bg-blue-600'
    case 'NAVIGATING': case 'APPROACHING': case 'PICKING': case 'RECOVERING_DEFERRED': case 'COLLECTING': return 'bg-amber-600'
    case 'SWEEPING': case 'SWEEP_NAVIGATE': return 'bg-purple-600'
    case 'NAVIGATING_DUMP': case 'DUMPING': return 'bg-red-600'
    case 'DONE': return 'bg-green-600'
    default: return 'bg-slate-600'
  }
}

function SimulationHUD({ state, dispatch, sim }) {
  // Use sim snapshot from polling if available, fall back to state
  const robot = sim?.robot || state.robot
  const bins = sim?.bins || state.bins
  const stats = sim?.stats || state.stats
  const sectors = sim?.sectors || state.sectors
  const activeSectorIdx = sim?.activeSectorIdx ?? state.activeSectorIdx
  const cvrpResult = sim?.cvrpResult || state.cvrpResult
  const totalWaste = state.wasteItems.length
  const collected = stats.itemsCollected
  const elapsed = stats.elapsedTime
  const ipm = elapsed > 0 ? (collected / (elapsed / 60)).toFixed(1) : '0.0'
  const sectorsTotal = sectors.length
  const sectorsDone = sectors.filter(s => s.completed).length
  const algoInfo = ALGORITHMS.find(a => a.id === state.algorithm) || ALGORITHMS[4]

  return (
    <>
      {/* Top Bar */}
      <div className="absolute top-0 left-0 right-0 h-12 bg-slate-900/85 backdrop-blur-sm flex items-center px-4 z-10">
        <h1 className="text-white font-bold text-sm tracking-wide">RECLAIM</h1>
        <span className="text-slate-500 text-xs ml-2">Autonomous Waste Sorting</span>
        <div className="flex-1" />
        <span className="text-xs px-2 py-0.5 rounded font-medium mr-3" style={{ backgroundColor: algoInfo.color + '33', color: algoInfo.color, border: `1px solid ${algoInfo.color}66` }}>
          {algoInfo.label}
        </span>
        <span className={`${getRobotStateBadgeColor(robot.state)} text-white text-xs px-3 py-1 rounded-full font-medium`}>
          {getRobotStateLabel(robot.state, sim)}
        </span>
        <span className="text-white text-sm font-mono ml-4 bg-slate-800 px-2 py-0.5 rounded">{Math.floor(elapsed / 60)}:{String(Math.floor(elapsed % 60)).padStart(2, '0')}</span>
      </div>

      {/* Bin Bars - Top Right */}
      <div className="absolute top-14 right-4 bg-slate-900/85 backdrop-blur-sm rounded-lg p-3 z-10">
        <h3 className="text-xs text-slate-400 mb-2 font-medium">Bins</h3>
        <div className="flex gap-3">
          {[
            { label: 'Recyclable', key: 'recyclable', color: BIN_COLORS.recyclable },
            { label: 'Compost', key: 'compost', color: BIN_COLORS.compost },
            { label: 'Landfill', key: 'landfill', color: BIN_COLORS.landfill },
          ].map(b => {
            const count = bins[b.key]
            const pct = count / BIN_CAPACITY
            const warning = pct >= DUMP_THRESHOLD
            return (
              <div key={b.key} className="flex flex-col items-center">
                <div className="w-6 h-20 bg-slate-700 rounded overflow-hidden flex flex-col-reverse relative">
                  <div className={`transition-all duration-300 ${warning ? 'animate-pulse' : ''}`}
                    style={{ height: `${pct * 100}%`, backgroundColor: warning ? '#ef4444' : b.color }} />
                </div>
                <span className="text-xs text-slate-400 mt-1">{count}/{BIN_CAPACITY}</span>
                <span className="text-[10px] text-slate-500">{b.label[0]}</span>
              </div>
            )
          })}
        </div>
      </div>

      {/* Battery */}
      <div className="absolute top-44 right-4 bg-slate-900/85 backdrop-blur-sm rounded-lg p-3 z-10 w-32">
        <div className="flex items-center gap-2">
          <span className="text-xs text-slate-400">Battery</span>
          <span className="text-xs text-slate-300">{Math.max(0, stats.battery).toFixed(0)}%</span>
        </div>
        <div className="w-full h-2 bg-slate-700 rounded mt-1 overflow-hidden">
          <div className="h-full transition-all" style={{
            width: `${Math.max(0, stats.battery)}%`,
            backgroundColor: stats.battery > 50 ? '#22c55e' : stats.battery > 20 ? '#eab308' : '#ef4444',
          }} />
        </div>
      </div>

      {/* Stats Panel - Left */}
      <div className="absolute top-14 left-4 bg-slate-900/85 backdrop-blur-sm rounded-lg p-3 z-10 w-52">
        <h3 className="text-xs text-slate-400 mb-2 font-medium">Statistics</h3>
        <div className="space-y-1 text-xs">
          <div className="flex justify-between"><span className="text-slate-400">Items</span><span className="text-white">{collected} / {totalWaste}</span></div>
          <div className="flex justify-between"><span className="text-slate-400">Distance</span><span className="text-white">{stats.distanceTraveled.toFixed(1)} m</span></div>
          <div className="flex justify-between"><span className="text-slate-400">Dump trips</span><span className="text-white">{stats.dumpTrips}</span></div>
          <div className="flex justify-between"><span className="text-slate-400">Items/min</span><span className="text-white">{ipm}</span></div>
          <div className="flex justify-between"><span className="text-slate-400">Items/m</span><span className="text-white">{stats.distanceTraveled > 0 ? (stats.itemsCollected / stats.distanceTraveled).toFixed(3) : '---'}</span></div>
          <div className="flex justify-between"><span className="text-slate-400">Speed</span><span className="text-white">{(robot.linVel || 0).toFixed(2)} m/s</span></div>
          <div className="flex justify-between"><span className="text-slate-400">Vision</span><span className="text-white">{((sim?.fogCoverage || 0) * 100).toFixed(0)}%</span></div>
          {sim?.reclaimMode && state.algorithm === 'reclaim' && (
            <div className="flex justify-between"><span className="text-cyan-400">Mode</span><span className="text-cyan-300">{sim.reclaimMode}</span></div>
          )}
          {sim?.deferredItems && sim.deferredItems.length > 0 && (
            <div className="flex justify-between"><span className="text-orange-400">Deferred</span><span className="text-orange-300">{sim.deferredItems.length}</span></div>
          )}
        </div>
      </div>

      {/* Route Optimization Panel - below stats panel on left */}
      {cvrpResult && (
        <div className="absolute top-72 left-4 bg-slate-900/85 backdrop-blur-sm rounded-lg p-3 z-10 w-52">
          <h3 className="text-xs text-slate-400 mb-2 font-medium">Route Optimization (2-opt)</h3>
          <div className="space-y-1 text-xs">
            <div className="flex justify-between"><span className="text-slate-400">Last batch greedy</span><span className="text-white">{cvrpResult.naiveDist.toFixed(1)} m</span></div>
            <div className="flex justify-between"><span className="text-slate-400">Last batch optimized</span><span className="text-white">{cvrpResult.totalDist.toFixed(1)} m</span></div>
            {sim?.cvrpCumulative?.totalNaive > 0 && (<>
            <div className="border-t border-slate-700 my-1" />
            <div className="flex justify-between"><span className="text-slate-400">Total greedy</span><span className="text-white">{sim.cvrpCumulative.totalNaive.toFixed(1)} m</span></div>
            <div className="flex justify-between"><span className="text-slate-400">Total optimized</span><span className="text-white">{sim.cvrpCumulative.totalOptimized.toFixed(1)} m</span></div>
            <div className="flex justify-between"><span className="text-slate-400">Cumulative savings</span>
              <span className="text-green-400">{((1 - sim.cvrpCumulative.totalOptimized / sim.cvrpCumulative.totalNaive) * 100).toFixed(1)}%</span>
            </div>
            <div className="flex justify-between"><span className="text-slate-400">Batches planned</span><span className="text-white">{sim.cvrpCumulative.batchCount}</span></div>
            </>)}
          </div>
        </div>
      )}

      {/* Speed + View Controls - Bottom Center */}
      <div className="absolute bottom-4 left-1/2 -translate-x-1/2 flex flex-col items-center gap-2 z-10">
        {/* View Toggles */}
        <div className="bg-slate-900/85 backdrop-blur-sm rounded-lg p-2 flex flex-wrap gap-1 justify-center">
        {[
          ['showFOV', 'FOV'],
          ['showLidar', 'LiDAR'],
          ['showPath', 'Path'],
          ['showSectors', 'Sectors'],
          ['showCostmap', 'Costmap'],
          ['showHeatmap', 'Heatmap'],
          ['showFog', 'Vision'],
          ['showDebug', 'Debug'],
        ].map(([key, label]) => (
          <button key={key}
            className={`text-[10px] px-2 py-1 rounded ${state[key] ? 'bg-slate-600 text-white' : 'text-slate-500 bg-slate-800'}`}
            onClick={() => dispatch({ type: 'TOGGLE', key })}>
            {label}
          </button>
        ))}
        </div>
        {/* Speed Controls */}
        <div className="bg-slate-900/85 backdrop-blur-sm rounded-lg p-2 flex items-center gap-2">
          <button className="text-white text-xs px-2 py-1 rounded hover:bg-slate-700"
            onClick={() => dispatch({ type: 'PAUSE' })}>
            {state.paused ? '▶' : '⏸'}
          </button>
          {[0.5, 1, 2, 5, 10].map(s => (
            <button key={s}
              className={`text-xs px-2 py-1 rounded ${state.speedMultiplier === s ? 'bg-blue-600 text-white' : 'text-slate-400 hover:bg-slate-700'}`}
              onClick={() => dispatch({ type: 'SET_SPEED', speed: s })}>
              {s}x
            </button>
          ))}
          <div className="w-px h-5 bg-slate-600 mx-1" />
          <button className="text-xs px-2 py-1 rounded text-red-400 hover:bg-red-900/30 border border-red-800/50"
            onClick={() => dispatch({ type: 'RESET' })}>
            Stop
          </button>
        </div>
      </div>

      {/* Debug Log Panel */}
      {state.showDebug && sim?.debugLog && (
        <div className="absolute bottom-28 right-4 bg-black/90 backdrop-blur-sm rounded-lg p-2 z-10 w-96 max-h-64 overflow-y-auto font-mono">
          <div className="flex items-center justify-between mb-1">
            <span className="text-[10px] text-slate-400 font-medium">Debug Log</span>
            {sim.debugAnomalies && (
              <span className="text-[9px] text-slate-500">
                spins:{sim.debugAnomalies.spins} stalls:{sim.debugAnomalies.stateStalls} stuck:{sim.debugAnomalies.pathFailures}
              </span>
            )}
          </div>
          {sim.debugLog.slice(-50).reverse().map((entry, i) => {
            const color = entry.type === 'ANOMALY' ? 'text-red-400'
              : entry.type === 'STATE' ? 'text-blue-400'
              : entry.type === 'PICKUP' ? 'text-green-400'
              : entry.type === 'DUMP' ? 'text-orange-400'
              : entry.type === 'STUCK' ? 'text-yellow-400'
              : entry.type === 'DETECT' ? 'text-cyan-400'
              : 'text-slate-400'
            return (
              <div key={i} className={`text-[9px] ${color} leading-tight`}>
                <span className="text-slate-600">{entry.time}s</span> [{entry.type}] {entry.message}
              </div>
            )
          })}
          {sim.debugLog.length === 0 && <div className="text-[9px] text-slate-600">No events yet...</div>}
        </div>
      )}
    </>
  )
}

// ═══════════════════════════════════════════════════════════════
// SECTION 6: PRESET LAYOUTS (integrated in SetupPanel above)
// ═══════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════
// SECTION 7: END SUMMARY
// ═══════════════════════════════════════════════════════════════

function EndSummary({ state, dispatch }) {
  const totalWaste = (state.wasteItems || []).length
  const stats = state.stats || { itemsCollected: 0, elapsedTime: 0, dumpTrips: 0, distanceTraveled: 0, battery: 100 }
  const collected = stats.itemsCollected || 0
  const elapsed = stats.elapsedTime || 0
  const ipm = elapsed > 0 ? (collected / (elapsed / 60)).toFixed(1) : '0.0'
  const algoInfo = ALGORITHMS.find(a => a.id === state.algorithm) || ALGORITHMS[4]
  const isReclaim = state.algorithm === 'reclaim'
  const cum = state.cvrpCumulative || { totalNaive: 0, totalOptimized: 0 }
  const savings = isReclaim && cum.totalNaive > 0
    ? ((1 - cum.totalOptimized / cum.totalNaive) * 100).toFixed(1)
    : null
  const isTimeout = state.algorithm === 'random_walk' && elapsed >= RANDOM_WALK_TIMEOUT

  // Breakdown by type
  const breakdown = WASTE_TYPES.map(wt => ({
    ...wt,
    count: (state.wasteItems || []).filter(w => w.label === wt.label && w.collected).length,
  }))

  return (
    <div className="absolute inset-0 bg-black/80 backdrop-blur-sm flex items-center justify-center z-20" onClick={() => dispatch({ type: 'DISMISS_RESULTS' })}>
      <div className="bg-slate-900 border border-slate-700 rounded-2xl p-8 max-w-lg w-full mx-4 max-h-[90vh] overflow-y-auto relative" onClick={e => e.stopPropagation()}>
        <button onClick={() => dispatch({ type: 'DISMISS_RESULTS' })} className="absolute top-3 right-3 text-slate-400 hover:text-white text-xl leading-none" title="Close (click backdrop to close too)">✕</button>
        <div className="text-center mb-6">
          <div className="text-4xl mb-2">{isTimeout ? '⏱' : '✓'}</div>
          <h2 className="text-2xl font-bold text-white">{isTimeout ? 'TIMEOUT — 10 min limit reached' : 'Mission Complete'}</h2>
          <p className="text-slate-400 text-sm mt-1">
            <span className="inline-block px-2 py-0.5 rounded text-xs font-medium mr-1" style={{ backgroundColor: algoInfo.color + '33', color: algoInfo.color, border: `1px solid ${algoInfo.color}66` }}>
              {algoInfo.label}
            </span>
            {isTimeout ? 'Time limit reached' : 'All sectors covered'}
          </p>
        </div>

        {/* Report Card */}
        <div className="grid grid-cols-2 gap-3 mb-6">
          {[
            ['Items Collected', `${collected} / ${totalWaste}`],
            ['Sort Accuracy', '100%'],
            ['Distance Traveled', `${(stats.distanceTraveled || 0).toFixed(1)} m`],
            ['Time', `${Math.floor(elapsed / 60)}m ${Math.floor(elapsed % 60)}s`],
            ['Dump Trips', stats.dumpTrips || 0],
            ['Items/Minute', ipm],
            ['Items/Meter', stats.distanceTraveled > 0 ? (collected / stats.distanceTraveled).toFixed(3) : 'N/A'],
            ['Route Optimization', savings !== null ? `${savings}% shorter` : 'N/A'],
            ...(state.smartBinBalancing && (state.algorithm === 'reclaim') ? [
              ['Items Deferred', state.totalDeferred || 0],
              ['Deferred Recoveries', state.deferredRecoveries || 0],
            ] : []),
          ].map(([label, value], i) => (
            <div key={i} className="bg-slate-800 rounded-lg p-3">
              <div className="text-xs text-slate-400">{label}</div>
              <div className="text-lg font-bold text-white">{value}</div>
            </div>
          ))}
        </div>

        {/* Run Comparison Table */}
        {state.runHistory.length > 0 && (
          <div className="mb-6">
            <h3 className="text-sm font-semibold text-slate-300 mb-2">Run Comparison</h3>
            <div className="overflow-x-auto">
              <table className="w-full text-xs">
                <thead>
                  <tr className="text-slate-400 border-b border-slate-700">
                    <th className="text-left py-1 pr-2">Algorithm</th>
                    <th className="text-right py-1 px-1">Items</th>
                    <th className="text-right py-1 px-1">Distance</th>
                    <th className="text-right py-1 px-1">Time</th>
                    <th className="text-right py-1 px-1">Dumps</th>
                    <th className="text-right py-1 px-1">IPM</th>
                    <th className="text-right py-1 pl-1">Balancing</th>
                  </tr>
                </thead>
                <tbody>
                  {state.runHistory.map((run, i) => (
                    <tr key={i} className="border-b border-slate-800">
                      <td className="py-1 pr-2 flex items-center gap-1">
                        <span className="w-2 h-2 rounded-full" style={{ backgroundColor: run.color }} />
                        <span className="text-white">{run.algorithm}</span>
                      </td>
                      <td className="text-right py-1 px-1 text-white">{run.items}/{run.total}</td>
                      <td className="text-right py-1 px-1 text-white">{run.distance.toFixed(1)}m</td>
                      <td className="text-right py-1 px-1 text-white">{run.time}</td>
                      <td className="text-right py-1 px-1 text-white">{run.dumpTrips}</td>
                      <td className="text-right py-1 px-1 text-white">{run.ipm}</td>
                      <td className="text-right py-1 pl-1 text-white">{run.binBalancing || 'N/A'}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          </div>
        )}

        {/* Breakdown */}
        <div className="mb-6">
          <h3 className="text-sm font-semibold text-slate-300 mb-2">Breakdown</h3>
          <div className="space-y-1">
            {breakdown.filter(b => b.count > 0).map((b, i) => (
              <div key={i} className="flex items-center text-xs">
                <span className="inline-block w-2 h-2 rounded-full mr-2" style={{ backgroundColor: b.color }} />
                <span className="text-slate-300 flex-1">{b.label}</span>
                <span className="text-slate-400 mr-3">{b.count}</span>
                <span className="text-slate-500 capitalize">{b.bin}</span>
              </div>
            ))}
          </div>
        </div>

        {/* Buttons */}
        <div className="flex gap-3">
          <button className="flex-1 py-2 rounded-lg bg-blue-600 hover:bg-blue-500 text-white text-sm font-medium"
            onClick={() => dispatch({ type: 'RESET_FULL' })}>
            New Setup
          </button>
          <button className="flex-1 py-2 rounded-lg border border-slate-600 hover:bg-slate-800 text-white text-sm font-medium"
            onClick={() => dispatch({ type: 'RESET' })}>
            Run Same Layout
          </button>
        </div>
      </div>
    </div>
  )
}

// ═══════════════════════════════════════════════════════════════
// ═══════════════════════════════════════════════════════════════
// HEADLESS TEST RUNNER — runs a full sim without React/R3F
// ═══════════════════════════════════════════════════════════════
window.__headlessTest = function(algorithm, preset = 'conference', maxSteps = 200000, numItemsOverride = null) {
  const venue = { width: 20, height: 15 }
  const robotStart = { x: 1.5, z: 1.5, angle: 0 }
  const dumpStation = { x: 10, z: 0.8 }

  // Generate preset obstacles and waste (same logic as loadPreset)
  let obs = [], waste = [], walls = []
  const vw = venue.width, vh = venue.height
  if (preset === 'conference') {
    const rows = 3, cols = 3
    const xStart = vw * 0.2, xEnd = vw * 0.8
    const zStart = vh * 0.2, zEnd = vh * 0.8
    const xSpacing = (xEnd - xStart) / (cols - 1)
    const zSpacing = (zEnd - zStart) / (rows - 1)
    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < cols; c++) {
        const tx = xStart + c * xSpacing
        const tz = zStart + r * zSpacing
        obs.push({ id: nextId(), type: 'table', x: tx, z: tz, width: 1.2, depth: 0.6 })
        obs.push({ id: nextId(), type: 'chair', x: tx - 0.8, z: tz, width: 0.45, depth: 0.45 })
        obs.push({ id: nextId(), type: 'chair', x: tx + 0.8, z: tz, width: 0.45, depth: 0.45 })
      }
    }
    const nItems = numItemsOverride || 30
    for (let i = 0; i < nItems; i++) {
      const wt = WASTE_TYPES[Math.floor(Math.random() * WASTE_TYPES.length)]
      waste.push(placeWasteItem(wt, 2 + Math.random() * (vw - 4), 2 + Math.random() * (vh - 4), obs, vw, vh, walls))
    }
  } else if (preset === 'dense') {
    for (let i = 0; i < 15; i++) {
      const types = ['table', 'chair', 'trashbin', 'planter']
      const t = types[Math.floor(Math.random() * types.length)]
      const sizes = { table: [1.2, 0.6], chair: [0.45, 0.45], trashbin: [0.5, 0.5], planter: [0.5, 0.5] }
      obs.push({ id: nextId(), type: t, x: 2 + Math.random() * (vw - 4), z: 2 + Math.random() * (vh - 4), width: sizes[t][0], depth: sizes[t][1] })
    }
    const nItems = numItemsOverride || 100
    for (let i = 0; i < nItems; i++) {
      const wt = WASTE_TYPES[Math.floor(Math.random() * WASTE_TYPES.length)]
      waste.push(placeWasteItem(wt, 1 + Math.random() * (vw - 2), 2 + Math.random() * (vh - 3), obs, vw, vh, walls))
    }
  } else if (preset === 'arena') {
    for (let a = 0; a < 8; a++) {
      const angle = (a / 8) * Math.PI * 2
      const cx = vw / 2 + Math.cos(angle) * Math.min(vw, vh) * 0.35
      const cz = vh / 2 + Math.sin(angle) * Math.min(vw, vh) * 0.35
      obs.push({ id: nextId(), type: a % 2 === 0 ? 'table' : 'chair', x: cx, z: cz, width: a % 2 === 0 ? 1.2 : 0.45, depth: a % 2 === 0 ? 0.6 : 0.45 })
    }
    const nItems = numItemsOverride || 50
    for (let i = 0; i < nItems; i++) {
      const wt = WASTE_TYPES[Math.floor(Math.random() * WASTE_TYPES.length)]
      waste.push(placeWasteItem(wt, 2 + Math.random() * (vw - 4), 2 + Math.random() * (vh - 4), obs, vw, vh, walls))
    }
  }

  const sectors = divideSectors(vw, vh)
  const occGrid = createOccupancyGrid(vw, vh, obs, GRID_RESOLUTION, walls)

  const sim = {
    robot: { ...robotStart, linVel: 0, angVel: 0, state: 'IDLE', armState: 'stowed', pickupTimer: 0, dumpTimer: 0, targetWaste: null },
    bins: { recyclable: 0, compost: 0, landfill: 0 },
    sectors: sectors.map(s => ({ ...s })), activeSectorIdx: -1,
    coveragePath: [], coverageIdx: 0, currentPath: [], pathIdx: 0,
    wasteItems: waste.map(w => ({ ...w })),
    detectedWaste: [], collectionQueue: [],
    stats: { distanceTraveled: 0, itemsCollected: 0, dumpTrips: 0, elapsedTime: 0, battery: 100 },
    particles: [], cvrpResult: null, allDetectedItems: [],
    occGrid, returnPoint: null, dumpStation: { ...dumpStation },
    stuckCounter: 0, lastPos: { x: 0, z: 0 }, progressCheckpoint: null,
    algorithm,
    smartBinBalancing: (algorithm === 'reclaim'),
    deferredItems: [], deferredRecoveries: 0, totalDeferred: 0,
    deferredLabels: [], pickupMarkers: [],
    fogGrid: null, fogResolution: 0.5,
    randomWalkDir: Math.random() * Math.PI * 2,
    randomWalkTimer: 0, scanTimer: 0, scanAngle: 0,
    quadrantVisits: [0, 0, 0, 0],
    debugLog: [], debugAnomalies: { spins: 0, oscillations: 0, pathFailures: 0, stateStalls: 0 },
    spinTimer: 0, _lastStateChange: null,
    // New metrics + RECLAIM mode
    reclaimMode: 'SCAN',
    visitGrid: initVisitGrid(vw, vh),
    pathTrail: [],
    plannedRoute: [],
    deadheadDist: 0,
    dumpTripFullness: [],
    timeTo90: null,
    timeToLast: null,
    _sweepPatches: [],
    _sweepPath: [],
    sweepIdx: 0,
    scanAngleAccum: 0,
  }

  // Diagnostic: track state time, pickup milestones
  sim._diag = { stateTime: {}, pickupTimes: [], dumpTimes: [], deferEvents: [] }
  let lastState = 'IDLE', lastStateStart = 0

  let steps = 0
  const startTime = performance.now()
  while (sim.robot.state !== 'DONE' && steps < maxSteps) {
    try { stepSimulationDispatch(sim, FIXED_DT, venue, obs) }
    catch (e) { sim.robot.state = 'DONE'; sim._error = e.message; sim._stack = e.stack; break }
    // Track state time
    const curState = sim.robot.state
    if (curState !== lastState) {
      const elapsed = sim.stats.elapsedTime - lastStateStart
      sim._diag.stateTime[lastState] = (sim._diag.stateTime[lastState] || 0) + elapsed
      lastState = curState
      lastStateStart = sim.stats.elapsedTime
    }
    // Track RECLAIM mode changes
    if (sim.reclaimMode && sim._lastReclaimMode !== sim.reclaimMode) {
      sim._diag.modeChanges = sim._diag.modeChanges || []
      sim._diag.modeChanges.push({ mode: sim.reclaimMode, time: +sim.stats.elapsedTime.toFixed(0), items: sim.stats.itemsCollected })
      sim._lastReclaimMode = sim.reclaimMode
    }
    // Track pickups
    const prevPickups = sim._diag.pickupTimes.length
    if (sim.stats.itemsCollected > prevPickups) {
      sim._diag.pickupTimes.push(+sim.stats.elapsedTime.toFixed(0))
    }
    steps++
  }
  const wallTime = ((performance.now() - startTime) / 1000).toFixed(1)

  return {
    algorithm,
    preset,
    completed: sim.robot.state === 'DONE',
    steps,
    wallTimeSeconds: wallTime,
    itemsCollected: sim.stats.itemsCollected,
    totalItems: waste.length,
    collectionRate: waste.length > 0 ? ((sim.stats.itemsCollected / waste.length) * 100).toFixed(1) + '%' : 'N/A',
    distanceTraveled: sim.stats.distanceTraveled.toFixed(1),
    elapsedSimTime: sim.stats.elapsedTime.toFixed(1),
    dumpTrips: sim.stats.dumpTrips,
    batteryRemaining: sim.stats.battery.toFixed(1),
    anomalies: sim.debugAnomalies,
    lastState: sim.robot.state,
    lastLogs: sim.debugLog.slice(-5),
    error: sim._error || null,
    stack: sim._stack || null,
    // Detailed waste analysis
    detected: sim.wasteItems.filter(w => w.detected).length,
    collected: sim.wasteItems.filter(w => w.collected).length,
    detectedNotCollected: sim.wasteItems.filter(w => w.detected && !w.collected).map(w => ({ id: w.id, x: +w.x.toFixed(1), z: +w.z.toFixed(1), bin: w.bin })),
    neverDetected: sim.wasteItems.filter(w => !w.detected).map(w => ({ id: w.id, x: +w.x.toFixed(1), z: +w.z.toFixed(1), bin: w.bin })),
    queueRemaining: sim.collectionQueue.length,
    diag: sim._diag || null,
    // New metrics
    ...computeAdvancedMetrics(sim),
    reclaimMode: sim.reclaimMode,
  }
}

// Full position trace for debugging algorithm behavior
window.__traceTest = function(algorithm, preset = 'conference', maxSteps = 250000, intervalSteps = 300) {
  // Run headless but capture position/state every intervalSteps (~5s sim time)
  const origTest = window.__headlessTest
  // We need access to the sim object mid-loop. Reimplement the loop with tracing.
  const venue = { width: 20, height: 15 }
  const robotStart = { x: 1.5, z: 1.5, angle: 0 }
  const dumpStation = { x: 10, z: 0.8 }
  let obs = [], waste = [], walls = []
  const vw = venue.width, vh = venue.height
  const numItems = typeof arguments[4] === 'number' ? arguments[4] : null
  if (preset === 'conference') {
    const rows = 3, cols = 3
    const xStart = vw * 0.2, xEnd = vw * 0.8
    const zStart = vh * 0.2, zEnd = vh * 0.8
    const xSpacing = (xEnd - xStart) / (cols - 1)
    const zSpacing = (zEnd - zStart) / (rows - 1)
    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < cols; c++) {
        const tx = xStart + c * xSpacing
        const tz = zStart + r * zSpacing
        obs.push({ id: _nextId++, type: 'table', x: tx, z: tz, width: 1.2, depth: 0.6 })
        obs.push({ id: _nextId++, type: 'chair', x: tx - 0.8, z: tz, width: 0.45, depth: 0.45 })
        obs.push({ id: _nextId++, type: 'chair', x: tx + 0.8, z: tz, width: 0.45, depth: 0.45 })
      }
    }
    for (let i = 0; i < (numItems || 25); i++) {
      const wt = WASTE_TYPES[Math.floor(Math.random() * WASTE_TYPES.length)]
      waste.push(placeWasteItem(wt, 2 + Math.random() * (vw - 4), 2 + Math.random() * (vh - 4), obs, vw, vh, walls))
    }
  } else if (preset === 'arena') {
    for (let a = 0; a < 8; a++) {
      const angle = (a / 8) * Math.PI * 2
      const cx = vw / 2 + Math.cos(angle) * Math.min(vw, vh) * 0.35
      const cz = vh / 2 + Math.sin(angle) * Math.min(vw, vh) * 0.35
      obs.push({ id: _nextId++, type: a % 2 === 0 ? 'table' : 'chair', x: cx, z: cz, width: a % 2 === 0 ? 1.2 : 0.45, depth: a % 2 === 0 ? 0.6 : 0.45 })
    }
    for (let i = 0; i < (numItems || 50); i++) {
      const wt = WASTE_TYPES[Math.floor(Math.random() * WASTE_TYPES.length)]
      waste.push(placeWasteItem(wt, 2 + Math.random() * (vw - 4), 2 + Math.random() * (vh - 4), obs, vw, vh, walls))
    }
  } else if (preset === 'dense') {
    for (let i = 0; i < 15; i++) {
      const types = ['table', 'chair', 'trashbin', 'planter']
      const t = types[Math.floor(Math.random() * types.length)]
      const sizes = { table: [1.2, 0.6], chair: [0.45, 0.45], trashbin: [0.5, 0.5], planter: [0.5, 0.5] }
      obs.push({ id: _nextId++, type: t, x: 2 + Math.random() * (vw - 4), z: 2 + Math.random() * (vh - 4), width: sizes[t][0], depth: sizes[t][1] })
    }
    for (let i = 0; i < (numItems || 100); i++) {
      const wt = WASTE_TYPES[Math.floor(Math.random() * WASTE_TYPES.length)]
      waste.push(placeWasteItem(wt, 1 + Math.random() * (vw - 2), 2 + Math.random() * (vh - 3), obs, vw, vh, walls))
    }
  }
  const sectors = divideSectors(vw, vh)
  const occGrid = createOccupancyGrid(vw, vh, obs, GRID_RESOLUTION, walls)
  const sim = {
    robot: { ...robotStart, linVel: 0, angVel: 0, state: 'IDLE', armState: 'stowed', pickupTimer: 0, dumpTimer: 0, targetWaste: null },
    bins: { recyclable: 0, compost: 0, landfill: 0 },
    sectors: sectors.map(s => ({ ...s })), activeSectorIdx: -1,
    coveragePath: [], coverageIdx: 0, currentPath: [], pathIdx: 0,
    wasteItems: waste.map(w => ({ ...w })),
    detectedWaste: [], collectionQueue: [],
    stats: { distanceTraveled: 0, itemsCollected: 0, dumpTrips: 0, elapsedTime: 0, battery: 100 },
    particles: [], cvrpResult: null, allDetectedItems: [],
    occGrid, returnPoint: null, dumpStation: { ...dumpStation },
    stuckCounter: 0, lastPos: { x: 0, z: 0 }, progressCheckpoint: null,
    algorithm,
    smartBinBalancing: algorithm === 'reclaim',
    deferredItems: [], deferredRecoveries: 0, totalDeferred: 0,
    deferredLabels: [], pickupMarkers: [],
    fogGrid: null, fogResolution: 0.5,
    randomWalkDir: Math.random() * Math.PI * 2, randomWalkTimer: 0, scanTimer: 0, scanAngle: 0,
    quadrantVisits: [0, 0, 0, 0],
    debugLog: [], debugAnomalies: { spins: 0, oscillations: 0, pathFailures: 0, stateStalls: 0 },
    spinTimer: 0, _lastStateChange: null,
    reclaimMode: 'SCAN', visitGrid: initVisitGrid(vw, vh), pathTrail: [], plannedRoute: [],
    deadheadDist: 0, dumpTripFullness: [], timeTo90: null, timeToLast: null,
    _sweepPatches: [], _sweepPath: [], sweepIdx: 0, scanAngleAccum: 0,
  }
  const trace = []
  let steps = 0
  while (sim.robot.state !== 'DONE' && steps < maxSteps) {
    try { stepSimulationDispatch(sim, FIXED_DT, venue, obs) } catch(e) { sim.robot.state = 'DONE'; break }
    if (steps % intervalSteps === 0) {
      trace.push({
        t: +sim.stats.elapsedTime.toFixed(1),
        x: +sim.robot.x.toFixed(2), z: +sim.robot.z.toFixed(2),
        angle: +(sim.robot.angle * 180 / Math.PI).toFixed(0),
        state: sim.robot.state,
        sweepZ: sim.sweepZ != null ? +sim.sweepZ.toFixed(1) : null,
        sweepDir: sim.sweepDir || null,
        items: sim.stats.itemsCollected,
        queue: sim.collectionQueue.length,
        detected: sim.detectedWaste.length,
        fog: sim.fogGrid ? +(getFogCoverage(sim) * 100).toFixed(0) : 0,
      })
    }
    steps++
  }
  trace.push({ t: +sim.stats.elapsedTime.toFixed(1), x: +sim.robot.x.toFixed(2), z: +sim.robot.z.toFixed(2), state: sim.robot.state, items: sim.stats.itemsCollected, final: true })
  return { trace, summary: { items: sim.stats.itemsCollected + '/' + waste.length, time: sim.stats.elapsedTime.toFixed(1), dist: sim.stats.distanceTraveled.toFixed(1), completed: sim.robot.state === 'DONE' } }
}

// SECTION 8: MAIN APP
// ═══════════════════════════════════════════════════════════════

// 3D components that read from simRef to avoid React re-renders
const PARTICLE_POOL_SIZE = 100
function SimParticles() {
  const simRef = useContext(SimRefContext)
  const groupRef = useRef()
  const poolRef = useRef(false)

  useFrame(() => {
    if (!simRef?.current || !groupRef.current) return
    const group = groupRef.current
    // One-time pool init
    if (!poolRef.current) {
      poolRef.current = true
      const geo = new THREE.SphereGeometry(0.015, 4, 4)
      for (let i = 0; i < PARTICLE_POOL_SIZE; i++) {
        const mesh = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ transparent: true }))
        mesh.visible = false
        group.add(mesh)
      }
    }
    const particles = simRef.current.particles
    const children = group.children
    const count = Math.min(particles.length, PARTICLE_POOL_SIZE)
    for (let i = 0; i < PARTICLE_POOL_SIZE; i++) {
      if (i < count) {
        const p = particles[i]
        children[i].position.set(p.x, p.y, p.z)
        children[i].material.color.set(p.color)
        children[i].material.opacity = Math.max(0, p.life * 2)
        children[i].visible = true
      } else {
        children[i].visible = false
      }
    }
  })

  return <group ref={groupRef} />
}

// 3D glow rings on detected items — fixed pool, no allocation per frame
const GLOW_POOL_SIZE = 50
function SimWasteGlow() {
  const simRef = useContext(SimRefContext)
  const groupRef = useRef()
  const poolRef = useRef(false)

  useFrame(() => {
    if (!simRef?.current || !groupRef.current) return
    const group = groupRef.current
    // One-time pool init
    if (!poolRef.current) {
      poolRef.current = true
      const geo = new THREE.RingGeometry(0.12, 0.15, 12)
      const mat = new THREE.MeshBasicMaterial({ color: '#facc15', transparent: true, opacity: 0.6, side: THREE.DoubleSide })
      for (let i = 0; i < GLOW_POOL_SIZE; i++) {
        const ring = new THREE.Mesh(geo, mat.clone())
        ring.rotation.x = -Math.PI / 2
        ring.visible = false
        group.add(ring)
      }
    }
    const detected = simRef.current.wasteItems.filter(w => w.detected && !w.collected)
    const children = group.children
    const count = Math.min(detected.length, GLOW_POOL_SIZE)
    for (let i = 0; i < GLOW_POOL_SIZE; i++) {
      if (i < count) {
        const w = detected[i]
        children[i].position.set(w.x, 0.05, w.z)
        children[i].visible = true
        children[i].material.opacity = 0.4 + 0.3 * Math.sin(Date.now() * 0.005 + i)
      } else {
        children[i].visible = false
      }
    }
  })

  return <group ref={groupRef} />
}

// Pickup flash markers — green checkmarks that fade at collection points
const PICKUP_MARKER_POOL = 500
function SimPickupMarkers() {
  const simRef = useContext(SimRefContext)
  const groupRef = useRef()
  const poolRef = useRef(false)

  useFrame(() => {
    if (!simRef?.current || !groupRef.current) return
    const group = groupRef.current
    if (!poolRef.current) {
      poolRef.current = true
      const geo = new THREE.RingGeometry(0.15, 0.25, 16)
      for (let i = 0; i < PICKUP_MARKER_POOL; i++) {
        const mesh = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: '#22c55e', transparent: true, opacity: 0.8, side: THREE.DoubleSide }))
        mesh.rotation.x = -Math.PI / 2
        mesh.visible = false
        group.add(mesh)
      }
    }
    const markers = simRef.current.pickupMarkers || []
    const children = group.children
    const count = Math.min(markers.length, PICKUP_MARKER_POOL)
    for (let i = 0; i < PICKUP_MARKER_POOL; i++) {
      if (i < count) {
        const m = markers[i]
        children[i].position.set(m.x, 0.03, m.z)
        children[i].visible = true
        children[i].material.color.set(m.color || '#22c55e')
        children[i].material.opacity = 0.7
        children[i].scale.set(1, 1, 1)
      } else {
        children[i].visible = false
      }
    }
  })

  return <group ref={groupRef} />
}

// Fog of war overlay — strong dark overlay on unseen areas, fully transparent on seen areas
// Fog of war — dark tiles on unseen areas, removed when scanned
function FogOfWarOverlay({ venue, visible }) {
  const simRef = useContext(SimRefContext)
  const meshRef = useRef()
  const fcRef = useRef(0)
  const instancedRef = useRef(null)
  const prevCountRef = useRef(0)

  const tileSize = 0.5
  const fw = Math.ceil(venue.width / tileSize)
  const fh = Math.ceil(venue.height / tileSize)
  const maxTiles = fw * fh

  // Create InstancedMesh once
  const [geo, mat] = useMemo(() => {
    const g = new THREE.PlaneGeometry(tileSize * 1.01, tileSize * 1.01) // slight overlap to avoid gaps
    const m = new THREE.MeshBasicMaterial({ color: 0x0a0f1e, depthWrite: false })
    return [g, m]
  }, [])

  useFrame(() => {
    if (!simRef?.current || !visible || !meshRef.current) return
    const sim = simRef.current
    if (!sim.fogGrid) return
    fcRef.current++
    if (fcRef.current % 10 !== 0) return

    const inst = meshRef.current
    const dummy = new THREE.Object3D()
    let count = 0

    for (let gz = 0; gz < fh; gz++) {
      for (let gx = 0; gx < fw; gx++) {
        if (!sim.fogGrid[gz * fw + gx]) {
          dummy.position.set(gx * tileSize + tileSize / 2, 0.035, gz * tileSize + tileSize / 2)
          dummy.rotation.set(-Math.PI / 2, 0, 0)
          dummy.updateMatrix()
          inst.setMatrixAt(count, dummy.matrix)
          count++
        }
      }
    }
    inst.count = count
    inst.instanceMatrix.needsUpdate = true
  })

  if (!visible) return null
  return <instancedMesh ref={meshRef} args={[geo, mat, maxTiles]} />
}

// 2D overlay list of detected items (rendered outside Canvas)
function DetectionList({ simSnapshot }) {
  if (!simSnapshot || !simSnapshot.detectedItems) return null
  const items = simSnapshot.detectedItems
  if (items.length === 0) return null

  return (
    <div className="absolute left-1/2 -translate-x-1/2 top-12 z-10 pointer-events-none">
      <div className="bg-slate-900/80 backdrop-blur-sm rounded-lg px-3 py-1.5 flex gap-3 items-center flex-wrap justify-center max-w-[600px]">
        <span className="text-[10px] text-slate-400 font-semibold uppercase tracking-wider">Detected</span>
        {items.slice(0, 8).map((it, i) => (
          <span key={i} className="flex items-center gap-1">
            <span className="w-1.5 h-1.5 rounded-full inline-block" style={{ background: BIN_COLORS[it.bin] }} />
            <span className="text-[10px] text-white/80">{it.label} <span className="text-white/40">[{it.confidence}]</span></span>
          </span>
        ))}
        {items.length > 8 && <span className="text-[10px] text-slate-500">+{items.length - 8} more</span>}
      </div>
    </div>
  )
}

export default function NavigationSimulation() {
  const [state, dispatch] = useReducer(reducer, initialState)
  const simEngineRef = useRef(null)
  // Debug: expose simRef globally
  useEffect(() => { window.__simRef = simEngineRef }, [])
  useEffect(() => { window.__dispatch = dispatch }, [dispatch])

  const handleFloorClick = useCallback((x, z) => {
    const mode = state.placementMode
    if (!mode) return
    const obsSizes = { table: [1.2, 0.6], chair: [0.45, 0.45], trashbin: [0.5, 0.5], podium: [0.6, 0.5], planter: [0.5, 0.5] }
    if (obsSizes[mode]) {
      dispatch({
        type: 'ADD_OBSTACLE',
        obstacle: {
          id: nextId(), type: mode, x, z,
          width: obsSizes[mode][0],
          depth: obsSizes[mode][1],
        }
      })
    } else {
      const wt = WASTE_TYPES.find(w => w.typeId === mode)
      if (wt) {
        dispatch({
          type: 'ADD_WASTE',
          item: { id: nextId(), ...wt, x, z, collected: false, detected: false, confidence: 0, picking: false }
        })
      }
    }
  }, [state.placementMode])

  const handleRemove = useCallback((id) => {
    dispatch({ type: 'REMOVE_ITEM', id })
  }, [])

  const handleDragItem = useCallback((id, x, z) => {
    dispatch({ type: 'MOVE_ITEM', id, x, z })
  }, [])

  const handleDragRobot = useCallback((x, z) => {
    dispatch({ type: 'SET_ROBOT_START', x, z })
  }, [])

  const handleDragDump = useCallback((x, z) => {
    dispatch({ type: 'SET_DUMP_STATION', x, z })
  }, [])

  // Poll simRef for HUD updates (outside Canvas, avoids R3F re-render crashes)
  // Also steps the simulation in background when tab is hidden (rAF pauses)
  const [simSnapshot, setSimSnapshot] = useState(null)
  useEffect(() => {
    if (state.phase !== 'RUNNING' && state.phase !== 'COMPLETE') return
    const interval = setInterval(() => {
      let sim = simEngineRef.current
      // Fallback init: if R3F hasn't rendered yet (hidden tab), initialize sim here
      if (!sim && state.phase === 'RUNNING') {
        simEngineRef.current = {
          robot: { ...state.robot }, bins: { ...state.bins },
          sectors: state.sectors.map(s => ({ ...s })), activeSectorIdx: -1,
          coveragePath: [], coverageIdx: 0, currentPath: [], pathIdx: 0,
          wasteItems: state.wasteItems.map(w => ({ ...w })),
          detectedWaste: [], collectionQueue: [],
          stats: { ...state.stats }, particles: [], cvrpResult: null,
          allDetectedItems: [],
          occGrid: createOccupancyGrid(state.venue.width, state.venue.height, state.obstacles, GRID_RESOLUTION, state.walls),
          returnPoint: null, dumpStation: { ...state.dumpStation },
          stuckCounter: 0, lastPos: { x: 0, z: 0 }, progressCheckpoint: null,
          algorithm: state.algorithm,
          smartBinBalancing: (state.algorithm === 'reclaim') && state.smartBinBalancing,
          deferredItems: [], deferredRecoveries: 0, totalDeferred: 0,
          deferredLabels: [], pickupMarkers: [],
          fogGrid: null, fogResolution: 0.5,
          randomWalkDir: Math.random() * Math.PI * 2,
          randomWalkTimer: 0, scanTimer: 0, scanAngle: 0,
          quadrantVisits: [0, 0, 0, 0],
          debugLog: [], debugAnomalies: { spins: 0, oscillations: 0, pathFailures: 0, stateStalls: 0 },
          spinTimer: 0, _lastStateChange: null,
        }
        sim = simEngineRef.current
      }
      if (!sim) return
      // Always expose sim state for debug tools
      window.__sim = sim
      window.__simConstants = { CAMERA_FOV_DEG, CAMERA_MIN_RANGE, CAMERA_MAX_RANGE, ARM_REACH, MAX_LINEAR_VEL, MAX_ANGULAR_VEL, BIN_CAPACITY, DUMP_THRESHOLD }
      window.__stepSim = () => stepSimulationDispatch(sim, FIXED_DT, state.venue, state.obstacles)
      window.__venue = state.venue
      window.__obstacles = state.obstacles
      // Background stepping: when tab is hidden, useFrame doesn't fire, so step here
      if (document.visibilityState === 'hidden' && state.phase === 'RUNNING' && !state.paused && sim.robot.state !== 'DONE') {
        const speed = state.speedMultiplier
        const stepsNeeded = Math.min(Math.ceil(speed * 0.5 / FIXED_DT), 600) // 0.5s worth at speed, cap 600
        for (let i = 0; i < stepsNeeded; i++) {
          if (sim.robot.state === 'DONE') break
          try { stepSimulationDispatch(sim, FIXED_DT, state.venue, state.obstacles) }
          catch (e) { console.error('Sim bg step error:', e); sim.robot.state = 'DONE'; break }
        }
      }
      const isComplete = sim.robot.state === 'DONE'
      const detectedItems = sim.wasteItems
        .filter(w => w.detected && !w.collected)
        .slice(0, 12)
        .map(w => ({ label: w.label, confidence: w.confidence, bin: w.bin }))
      const snap = {
        robot: { ...sim.robot },
        bins: { ...sim.bins },
        sectors: sim.sectors.map(s => ({ ...s })),
        activeSectorIdx: sim.activeSectorIdx,
        stats: { ...sim.stats },
        cvrpResult: sim.cvrpResult ? { ...sim.cvrpResult } : null,
        cvrpCumulative: sim.cvrpCumulative ? { ...sim.cvrpCumulative } : null,
        collectedIds: sim.wasteItems.filter(w => w.collected).map(w => w.id),
        detectedItems,
        deferredItems: sim.deferredItems ? [...sim.deferredItems] : [],
        totalDeferred: sim.totalDeferred || 0,
        deferredRecoveries: sim.deferredRecoveries || 0,
        fogCoverage: getFogCoverage(sim),
      }
      setSimSnapshot(snap)
      if (isComplete && state.phase !== 'COMPLETE') {
        const runAlgoInfo = ALGORITHMS.find(a => a.id === state.algorithm) || ALGORITHMS[4]
        const runElapsed = sim.stats.elapsedTime
        dispatch({
          type: 'ADD_RUN_HISTORY',
          run: {
            algorithm: runAlgoInfo.label,
            color: runAlgoInfo.color,
            items: sim.stats.itemsCollected,
            total: sim.wasteItems.length,
            distance: sim.stats.distanceTraveled,
            time: `${Math.floor(runElapsed / 60)}m ${Math.floor(runElapsed % 60)}s`,
            dumpTrips: sim.stats.dumpTrips,
            ipm: runElapsed > 0 ? (sim.stats.itemsCollected / (runElapsed / 60)).toFixed(1) : '0.0',
            binBalancing: (state.algorithm === 'reclaim') ? (state.smartBinBalancing ? 'ON' : 'OFF') : 'N/A',
          }
        })
        dispatch({
          type: 'SYNC_SIM',
          payload: {
            ...snap,
            phase: 'COMPLETE',
            wasteItems: sim.wasteItems.map(w => ({ ...w })),
          }
        })
      }
    }, 500)
    return () => clearInterval(interval)
  }, [state.phase, state.paused, state.speedMultiplier])

  const isSetup = state.phase === 'SETUP'
  const cameraPos = useMemo(() => {
    const maxDim = Math.max(state.venue.width, state.venue.height)
    return [state.venue.width / 2, maxDim * 0.8, state.venue.height / 2 + maxDim * 0.4]
  }, [state.venue.width, state.venue.height])

  return (
    <div className="w-screen h-screen relative overflow-hidden bg-slate-950">
      <Canvas camera={{ position: cameraPos, fov: 50, near: 0.1, far: 200 }}>
        <ambientLight intensity={0.5} />
        <directionalLight position={[15, 25, 15]} intensity={0.8} />

        <Floor width={state.venue.width} height={state.venue.height} />
        <Walls width={state.venue.width} height={state.venue.height} />
        <InteriorWalls walls={state.walls} />

        {state.obstacles.map(obs => (
          <ObstacleMesh key={obs.id} obs={obs} isSetup={isSetup} onRemove={handleRemove} onDrag={handleDragItem} />
        ))}

        {state.wasteItems.map(item => (
          <WasteItemMesh key={item.id} item={item} isSetup={isSetup} onRemove={handleRemove} />
        ))}

        {isSetup && <RobotStartMarker position={state.robotStart} onDrag={handleDragRobot} />}
        <DumpStationMesh position={state.dumpStation} bins={simSnapshot?.bins || state.bins} />

        <SimulationEngine state={state} dispatch={dispatch} simEngineRef={simEngineRef} />
        {!isSetup && (
          <SimRefContext.Provider value={simEngineRef}>
            <RobotModel robot={simSnapshot?.robot || state.robot} />
            <CameraFOVCone robot={simSnapshot?.robot || state.robot} visible={state.showFOV} />
            <LidarVisualization robot={simSnapshot?.robot || state.robot} visible={state.showLidar} />
            <SectorGrid sectors={simSnapshot?.sectors || state.sectors} activeSectorIdx={simSnapshot?.activeSectorIdx ?? state.activeSectorIdx} visible={state.showSectors} />
            <LivePathVisualization simRef={simEngineRef} visible={state.showPath} />
            <CostmapOverlay obstacles={state.obstacles} visible={state.showCostmap} venue={state.venue} />
            <HeatmapOverlay wasteItems={state.wasteItems} venueW={state.venue.width} venueH={state.venue.height} visible={state.showHeatmap} />
            <FogOfWarOverlay venue={state.venue} visible={state.showFog} />
            <SimParticles />
            <SimWasteGlow />
            <SimPickupMarkers />
          </SimRefContext.Provider>
        )}

        <ClickPlane venue={state.venue} onClick={handleFloorClick} placementMode={isSetup ? state.placementMode : null} />
        <OrbitControls target={[state.venue.width / 2, 0, state.venue.height / 2]}
          maxPolarAngle={Math.PI / 2.1} minDistance={3} maxDistance={80} />
      </Canvas>

      {/* UI Overlays */}
      {isSetup && <SetupPanel state={state} dispatch={dispatch} />}
      {isSetup && state.wasteItems.length > 0 && (
        <button
          className="absolute bottom-8 right-8 bg-green-600 hover:bg-green-500 text-white font-bold py-3 px-8 rounded-xl text-lg shadow-lg shadow-green-900/50 z-10 transition-all"
          onClick={() => dispatch({ type: 'START_SIM' })}>
          Start Simulation
        </button>
      )}
      {!isSetup && (state.phase !== 'COMPLETE' || state.resultsDismissed) && <SimulationHUD state={state} dispatch={dispatch} sim={simSnapshot} />}
      {!isSetup && (state.phase !== 'COMPLETE' || state.resultsDismissed) && <DetectionList simSnapshot={simSnapshot} />}
      {state.phase === 'COMPLETE' && state.stats && !state.resultsDismissed && <EndSummary state={state} dispatch={dispatch} />}

      {/* Robot POV Inset Camera - bottom right, above speed controls */}
      {!isSetup && state.phase !== 'COMPLETE' && (
        <div className="absolute bottom-32 right-4 w-48 h-32 rounded-lg overflow-hidden border border-slate-600 z-10 bg-black">
          <div className="absolute top-1 left-2 text-[10px] text-white/60 z-20 pointer-events-none">Robot POV</div>
          <Canvas camera={{ fov: 73, near: 0.1, far: 30 }}>
            <ambientLight intensity={0.5} />
            <directionalLight position={[10, 15, 10]} intensity={0.6} />
            <POVCameraController simRef={simEngineRef} />
            <Floor width={state.venue.width} height={state.venue.height} />
            <Walls width={state.venue.width} height={state.venue.height} />
        <InteriorWalls walls={state.walls} />
            {state.obstacles.map(obs => (
              <ObstacleMesh key={obs.id} obs={obs} isSetup={false} />
            ))}
            {state.wasteItems.map(item => (
              <WasteItemMesh key={item.id} item={item} isSetup={false} />
            ))}
          </Canvas>
        </div>
      )}
    </div>
  )
}
