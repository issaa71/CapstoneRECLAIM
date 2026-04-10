import { useState, useEffect, useRef, useCallback } from 'react'

// --- Toast System ---
function ToastContainer({ toasts, onDismiss }) {
  return (
    <div className="fixed bottom-4 right-4 z-50 flex flex-col gap-2">
      {toasts.map(t => (
        <div key={t.id}
          className={`px-4 py-2.5 rounded-lg shadow-lg text-sm font-medium text-white flex items-center gap-2 animate-slide-in ${
            t.type === 'error' ? 'bg-red-600' : t.type === 'warning' ? 'bg-amber-600' : 'bg-slate-700'
          }`}>
          <span>{t.message}</span>
          <button onClick={() => onDismiss(t.id)} className="ml-2 text-white/60 hover:text-white">x</button>
        </div>
      ))}
    </div>
  )
}

function useToast() {
  const [toasts, setToasts] = useState([])
  const show = useCallback((message, type = 'info') => {
    const id = Date.now()
    setToasts(t => [...t, { id, message, type }])
    setTimeout(() => setToasts(t => t.filter(x => x.id !== id)), 3000)
  }, [])
  const dismiss = useCallback((id) => setToasts(t => t.filter(x => x.id !== id)), [])
  return { toasts, show, dismiss }
}

// --- Simulated Robot State ---
function useSimulatedRobot() {
  const [robotPose, setRobotPose] = useState({ x: 2, y: 2, theta: 0 })
  const [robotState, setRobotState] = useState('IDLE')
  const [binLevels, setBinLevels] = useState({ recyclable: 0, compost: 0, landfill: 0 })
  const [battery, setBattery] = useState(100)
  const [itemsCollected, setItemsCollected] = useState(0)
  const [running, setRunning] = useState(false)
  const intervalRef = useRef(null)
  const stateRef = useRef({ pathIdx: 0, progress: 0, modeIdx: 0, items: 0, batt: 100 })

  const demoPath = [
    { x: 2, y: 2 }, { x: 8, y: 2 }, { x: 8, y: 5 }, { x: 14, y: 5 },
    { x: 14, y: 2 }, { x: 18, y: 2 }, { x: 18, y: 8 }, { x: 14, y: 8 },
    { x: 14, y: 12 }, { x: 8, y: 12 }, { x: 8, y: 8 }, { x: 2, y: 8 },
    { x: 2, y: 12 }, { x: 6, y: 12 }, { x: 6, y: 14 }, { x: 2, y: 14 },
  ]
  const modes = ['SCAN', 'COLLECT', 'SCAN', 'COLLECT', 'SWEEP', 'COLLECT', 'SCAN', 'COLLECT']

  const start = useCallback(() => {
    stateRef.current = { pathIdx: 0, progress: 0, modeIdx: 0, items: 0, batt: 100 }
    setRobotState('SCAN')
    setBattery(100)
    setBinLevels({ recyclable: 0, compost: 0, landfill: 0 })
    setItemsCollected(0)
    setRunning(true)
  }, [])

  const stop = useCallback(() => {
    setRunning(false)
    if (intervalRef.current) clearInterval(intervalRef.current)
  }, [])

  const pause = useCallback(() => {
    setRunning(false)
    if (intervalRef.current) clearInterval(intervalRef.current)
  }, [])

  const resume = useCallback(() => {
    setRunning(true)
  }, [])

  useEffect(() => {
    if (!running) return
    intervalRef.current = setInterval(() => {
      const s = stateRef.current
      const from = demoPath[s.pathIdx % demoPath.length]
      const to = demoPath[(s.pathIdx + 1) % demoPath.length]

      s.progress += 0.03
      if (s.progress >= 1) {
        s.progress = 0
        s.pathIdx++
        s.modeIdx = (s.modeIdx + 1) % modes.length
        setRobotState(modes[s.modeIdx])

        if (Math.random() > 0.4) {
          s.items++
          setItemsCollected(s.items)
          const bins = ['recyclable', 'compost', 'landfill']
          const bin = bins[Math.floor(Math.random() * 3)]
          setBinLevels(prev => ({ ...prev, [bin]: Math.min(15, (prev[bin] || 0) + 1) }))
        }
      }

      const x = from.x + (to.x - from.x) * s.progress
      const y = from.y + (to.y - from.y) * s.progress
      const theta = Math.atan2(to.y - from.y, to.x - from.x)
      setRobotPose({ x, y, theta })

      s.batt = Math.max(5, s.batt - 0.05)
      setBattery(Math.round(s.batt))
    }, 200)

    return () => { if (intervalRef.current) clearInterval(intervalRef.current) }
  }, [running])

  return { robotPose, robotState, setRobotState, binLevels, battery, itemsCollected, running, start, stop, pause, resume }
}

// --- Map Canvas ---
function MapView({ robotPose, zones, onAddZone, isDrawing, setIsDrawing }) {
  const canvasRef = useRef(null)
  const containerRef = useRef(null)
  const [pan, setPan] = useState({ x: 0, y: 0 })
  const [zoom, setZoom] = useState(1)
  const [drawStart, setDrawStart] = useState(null)
  const [drawEnd, setDrawEnd] = useState(null)
  const dragRef = useRef(null)
  const trailRef = useRef([])

  // Static demo map
  const map = useRef(null)
  if (!map.current) {
    const w = 200, h = 150
    const data = new Array(w * h).fill(0)
    for (let i = 0; i < w; i++) { data[i] = 100; data[(h - 1) * w + i] = 100 }
    for (let j = 0; j < h; j++) { data[j * w] = 100; data[j * w + w - 1] = 100 }
    const tables = [[40, 30, 12, 6], [40, 80, 12, 6], [100, 30, 12, 6], [100, 80, 12, 6], [160, 50, 12, 6], [160, 100, 12, 6]]
    tables.forEach(([tx, ty, tw, th]) => {
      for (let dy = 0; dy < th; dy++)
        for (let dx = 0; dx < tw; dx++)
          if (ty + dy < h && tx + dx < w) data[(ty + dy) * w + (tx + dx)] = 100
    })
    map.current = { w, h, res: 0.1, data }
  }

  // Track robot trail
  useEffect(() => {
    const trail = trailRef.current
    const last = trail[trail.length - 1]
    if (!last || Math.hypot(robotPose.x - last.x, robotPose.y - last.y) > 0.3) {
      trail.push({ x: robotPose.x, y: robotPose.y })
      if (trail.length > 500) trail.shift()
    }
  }, [robotPose])

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    const m = map.current
    canvas.width = containerRef.current.clientWidth
    canvas.height = containerRef.current.clientHeight
    const cw = canvas.width, ch = canvas.height

    ctx.fillStyle = '#1e293b'
    ctx.fillRect(0, 0, cw, ch)
    ctx.save()
    ctx.translate(cw / 2 + pan.x, ch / 2 + pan.y)
    ctx.scale(zoom, zoom)
    ctx.translate(-m.w * m.res * 50 / 2, -m.h * m.res * 50 / 2)
    const scale = 50 * m.res

    // Occupancy grid
    for (let j = 0; j < m.h; j++) {
      for (let i = 0; i < m.w; i++) {
        const v = m.data[j * m.w + i]
        ctx.fillStyle = v > 50 ? '#1e293b' : '#cbd5e1'
        ctx.fillRect(i * scale, (m.h - 1 - j) * scale, scale + 0.5, scale + 0.5)
      }
    }

    // Grid lines
    ctx.strokeStyle = '#47556640'
    ctx.lineWidth = 0.5
    const gs = 1.0 / m.res * scale
    for (let x = 0; x < m.w * scale; x += gs) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, m.h * scale); ctx.stroke() }
    for (let y = 0; y < m.h * scale; y += gs) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(m.w * scale, y); ctx.stroke() }

    // Exclusion zones
    zones.forEach(zone => {
      ctx.fillStyle = 'rgba(239, 68, 68, 0.2)'
      ctx.strokeStyle = '#ef4444'
      ctx.lineWidth = 2
      const zx = zone.x / m.res * scale
      const zy = (m.h - (zone.y + zone.h) / m.res) * scale
      const zw = zone.w / m.res * scale
      const zh = zone.h / m.res * scale
      ctx.fillRect(zx, zy, zw, zh)
      ctx.strokeRect(zx, zy, zw, zh)
      ctx.fillStyle = '#ef4444'
      ctx.font = '11px system-ui'
      ctx.fillText('NO-GO', zx + 4, zy + 14)
    })

    // Robot trail
    const trail = trailRef.current
    if (trail.length > 1) {
      ctx.strokeStyle = 'rgba(16, 185, 129, 0.3)'
      ctx.lineWidth = 2
      ctx.beginPath()
      ctx.moveTo(trail[0].x / m.res * scale, (m.h - trail[0].y / m.res) * scale)
      for (let i = 1; i < trail.length; i++) {
        ctx.lineTo(trail[i].x / m.res * scale, (m.h - trail[i].y / m.res) * scale)
      }
      ctx.stroke()
    }

    // Robot
    const rx = robotPose.x / m.res * scale
    const ry = (m.h - robotPose.y / m.res) * scale
    ctx.save()
    ctx.translate(rx, ry)
    ctx.rotate(-robotPose.theta)

    // FOV cone
    ctx.fillStyle = 'rgba(250, 204, 21, 0.12)'
    ctx.beginPath()
    ctx.moveTo(0, 0)
    const fovRad = (73 / 2) * Math.PI / 180
    const fovRange = 3.0 / m.res * scale
    ctx.arc(0, 0, fovRange, -fovRad, fovRad)
    ctx.closePath()
    ctx.fill()

    // Body
    ctx.fillStyle = '#10b981'
    ctx.beginPath()
    ctx.arc(0, 0, 8, 0, Math.PI * 2)
    ctx.fill()
    ctx.strokeStyle = '#fff'
    ctx.lineWidth = 2
    ctx.stroke()

    // Direction arrow
    ctx.fillStyle = '#fff'
    ctx.beginPath()
    ctx.moveTo(12, 0)
    ctx.lineTo(4, -5)
    ctx.lineTo(4, 5)
    ctx.closePath()
    ctx.fill()
    ctx.restore()

    // Scale bar
    ctx.restore()
    ctx.fillStyle = '#94a3b8'
    ctx.font = '11px system-ui'
    const barLen = 50 * zoom
    ctx.fillRect(cw - barLen - 20, ch - 25, barLen, 3)
    ctx.fillText(`${(1 / (m.res * zoom)).toFixed(1)}m`, cw - barLen - 20, ch - 30)

    // Draw preview
    if (drawStart && drawEnd) {
      ctx.fillStyle = 'rgba(239, 68, 68, 0.3)'
      ctx.strokeStyle = '#ef4444'
      ctx.lineWidth = 2
      const x = Math.min(drawStart.x, drawEnd.x)
      const y = Math.min(drawStart.y, drawEnd.y)
      const w = Math.abs(drawEnd.x - drawStart.x)
      const h = Math.abs(drawEnd.y - drawStart.y)
      ctx.fillRect(x, y, w, h)
      ctx.strokeRect(x, y, w, h)
    }
  })

  const handleWheel = (e) => { e.preventDefault(); setZoom(z => Math.max(0.3, Math.min(5, z - e.deltaY * 0.001))) }
  const handleMouseDown = (e) => {
    if (isDrawing) { setDrawStart({ x: e.nativeEvent.offsetX, y: e.nativeEvent.offsetY }); setDrawEnd({ x: e.nativeEvent.offsetX, y: e.nativeEvent.offsetY }) }
    else { dragRef.current = { x: e.clientX - pan.x, y: e.clientY - pan.y } }
  }
  const handleMouseMove = (e) => {
    if (isDrawing && drawStart) setDrawEnd({ x: e.nativeEvent.offsetX, y: e.nativeEvent.offsetY })
    else if (dragRef.current) setPan({ x: e.clientX - dragRef.current.x, y: e.clientY - dragRef.current.y })
  }
  const handleMouseUp = () => {
    if (isDrawing && drawStart && drawEnd) {
      const m2 = map.current
      const canvas = canvasRef.current
      const cw = canvas.width, ch = canvas.height
      const toMap = (sx, sy) => ({
        x: ((sx - cw / 2 - pan.x) / zoom + m2.w * m2.res * 50 / 2) / 50,
        y: (m2.h * m2.res - ((sy - ch / 2 - pan.y) / zoom + m2.h * m2.res * 50 / 2) / 50)
      })
      const p1 = toMap(drawStart.x, drawStart.y)
      const p2 = toMap(drawEnd.x, drawEnd.y)
      const zone = { x: Math.min(p1.x, p2.x), y: Math.min(p1.y, p2.y), w: Math.abs(p2.x - p1.x), h: Math.abs(p2.y - p1.y), id: Date.now() }
      if (zone.w > 0.1 && zone.h > 0.1) onAddZone(zone)
      setDrawStart(null); setDrawEnd(null); setIsDrawing(false)
    }
    dragRef.current = null
  }

  return (
    <div ref={containerRef} className="flex-1 relative" onWheel={handleWheel}>
      <canvas ref={canvasRef} className="w-full h-full"
        onMouseDown={handleMouseDown} onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp} onMouseLeave={handleMouseUp}
        style={{ cursor: isDrawing ? 'crosshair' : 'grab' }} />
    </div>
  )
}

// --- Bin Level Bar ---
function BinBar({ label, level, max, color }) {
  const pct = Math.min(100, (level / max) * 100)
  return (
    <div className="flex items-center gap-2 text-sm">
      <span className="w-24 text-slate-400">{label}</span>
      <div className="flex-1 h-4 bg-slate-700 rounded-full overflow-hidden">
        <div className="h-full rounded-full transition-all duration-500" style={{ width: `${pct}%`, backgroundColor: color }} />
      </div>
      <span className="w-12 text-right text-slate-300 text-xs">{level}/{max}</span>
    </div>
  )
}

// --- Main Dashboard ---
export default function OperatorDashboard({ onSwitchToSimulation }) {
  const robot = useSimulatedRobot()
  const toast = useToast()
  const [zones, setZones] = useState(() => {
    try { return JSON.parse(localStorage.getItem('reclaim_zones') || '[]') } catch { return [] }
  })
  const [isDrawing, setIsDrawing] = useState(false)
  const [missionTime, setMissionTime] = useState(0)
  const [missionActive, setMissionActive] = useState(false)
  const [paused, setPaused] = useState(false)
  const [estopFlash, setEstopFlash] = useState(false)

  useEffect(() => { localStorage.setItem('reclaim_zones', JSON.stringify(zones)) }, [zones])

  // Robot starts when user clicks Start Collection

  // Mission timer
  useEffect(() => {
    if (!robot.running) return
    const interval = setInterval(() => setMissionTime(t => t + 1), 1000)
    return () => clearInterval(interval)
  }, [robot.running])

  const formatTime = (s) => `${Math.floor(s / 60)}:${(s % 60).toString().padStart(2, '0')}`

  const handleMapVenue = () => {
    robot.start()
    robot.setRobotState('MAPPING')
    setMissionActive(true); setPaused(false); setMissionTime(0)
    toast.show('Mapping venue...')
  }

  const handleStart = () => {
    setMissionActive(true); setPaused(false); setMissionTime(0)
    robot.start()
    toast.show('Collection started')
  }

  const handlePause = () => {
    if (paused) { setPaused(false); robot.resume(); toast.show('Resumed') }
    else { setPaused(true); robot.pause(); toast.show('Paused') }
  }

  const handleDock = () => {
    setMissionActive(false)
    robot.stop()
    robot.setRobotState('DOCKING')
    toast.show('Returning to dock')
  }

  const handleEStop = () => {
    setMissionActive(false); setPaused(false)
    setEstopFlash(true); setTimeout(() => setEstopFlash(false), 1000)
    robot.stop()
    robot.setRobotState('STOPPED')
    toast.show('EMERGENCY STOP activated', 'error')
  }

  const handleAddZone = (zone) => {
    setZones([...zones, zone])
    toast.show('No-go zone added')
  }

  const modeColors = {
    IDLE: 'bg-slate-600', MAPPING: 'bg-purple-600', SCAN: 'bg-cyan-600',
    COLLECT: 'bg-green-600', SWEEP: 'bg-amber-600', DONE: 'bg-emerald-700',
    DUMPING: 'bg-orange-600', PICKING: 'bg-blue-600', STOPPED: 'bg-red-600',
    DOCKING: 'bg-indigo-600'
  }

  return (
    <div className="flex h-full bg-slate-900 text-slate-200">
      {/* Left Sidebar */}
      <div className="w-80 flex flex-col border-r border-slate-700 bg-slate-800/50">
        {/* Status */}
        <div className="p-4 border-b border-slate-700 space-y-3">
          <div className="flex items-center justify-between">
            <span className="text-sm text-slate-400">Mode</span>
            <span className={`px-2 py-0.5 rounded text-xs font-medium text-white ${modeColors[robot.robotState] || 'bg-slate-600'}`}>
              {robot.robotState}
            </span>
          </div>
          <div className="flex items-center justify-between">
            <span className="text-sm text-slate-400">Battery</span>
            <div className="flex items-center gap-2">
              <div className="w-20 h-3 bg-slate-700 rounded-full overflow-hidden">
                <div className="h-full rounded-full transition-all duration-500"
                  style={{ width: `${robot.battery}%`, backgroundColor: robot.battery > 30 ? '#22c55e' : robot.battery > 15 ? '#f59e0b' : '#ef4444' }} />
              </div>
              <span className="text-sm text-white font-medium">{robot.battery}%</span>
            </div>
          </div>
          <div className="flex items-center justify-between">
            <span className="text-sm text-slate-400">Items Collected</span>
            <span className="text-sm text-white font-medium">{robot.itemsCollected}</span>
          </div>
          <div className="flex items-center justify-between">
            <span className="text-sm text-slate-400">Mission Time</span>
            <span className="text-sm text-white font-mono">{formatTime(missionTime)}</span>
          </div>
        </div>

        {/* Bin Levels */}
        <div className="p-4 border-b border-slate-700 space-y-2">
          <h3 className="text-xs font-medium text-slate-400 uppercase tracking-wider mb-2">Bin Levels</h3>
          <BinBar label="Recyclable" level={robot.binLevels.recyclable} max={15} color="#3b82f6" />
          <BinBar label="Compost" level={robot.binLevels.compost} max={15} color="#22c55e" />
          <BinBar label="Landfill" level={robot.binLevels.landfill} max={15} color="#f97316" />
        </div>

        {/* Controls */}
        <div className="flex-1 overflow-y-auto">
          <div className="p-4 space-y-2">
            <h3 className="text-xs font-medium text-slate-400 uppercase tracking-wider mb-3">Controls</h3>

            <button onClick={handleMapVenue}
              className="w-full py-2.5 rounded-lg bg-purple-600 hover:bg-purple-500 text-white font-medium text-sm transition-colors">
              Map Venue
            </button>

            <button onClick={handleStart}
              className="w-full py-2.5 rounded-lg bg-emerald-600 hover:bg-emerald-500 text-white font-medium text-sm transition-colors">
              Start Collection
            </button>

            <button onClick={handlePause}
              className="w-full py-2.5 rounded-lg bg-amber-600 hover:bg-amber-500 text-white font-medium text-sm transition-colors">
              {paused ? 'Resume' : 'Pause'}
            </button>

            <button onClick={handleDock}
              className="w-full py-2.5 rounded-lg bg-slate-600 hover:bg-slate-500 text-white font-medium text-sm transition-colors">
              Return to Dock
            </button>

            <div className="pt-2">
              <button onClick={handleEStop}
                className={`w-full py-3 rounded-lg text-white font-bold text-base transition-all border-2 ${
                  estopFlash ? 'bg-red-400 border-white scale-105' : 'bg-red-600 hover:bg-red-500 border-red-400'
                }`}>
                EMERGENCY STOP
              </button>
            </div>

            {/* Zone Exclusion */}
            <div className="pt-3 border-t border-slate-700 mt-3">
              <h3 className="text-xs font-medium text-slate-400 uppercase tracking-wider mb-2">Zone Exclusion</h3>
              <button onClick={() => setIsDrawing(!isDrawing)}
                className={`w-full py-2 rounded-lg text-sm font-medium transition-colors ${
                  isDrawing ? 'bg-red-600 text-white' : 'bg-slate-600 hover:bg-slate-500 text-white'
                }`}>
                {isDrawing ? 'Drawing... (click & drag on map)' : 'Draw No-Go Zone'}
              </button>
              {zones.length > 0 && (
                <div className="mt-2 space-y-1">
                  {zones.map(zone => (
                    <div key={zone.id} className="flex items-center justify-between text-xs bg-slate-700/50 rounded px-2 py-1">
                      <span className="text-red-400">Zone ({zone.w.toFixed(1)}x{zone.h.toFixed(1)}m)</span>
                      <button onClick={() => setZones(zones.filter(z => z.id !== zone.id))}
                        className="text-slate-400 hover:text-red-400">x</button>
                    </div>
                  ))}
                  <button onClick={() => { setZones([]); toast.show('All zones cleared') }}
                    className="text-xs text-slate-500 hover:text-red-400 mt-1">Clear all zones</button>
                </div>
              )}
            </div>
          </div>
        </div>

        {/* Footer */}
        <div className="p-3 border-t border-slate-700 text-center shrink-0">
          <span className="text-[10px] text-slate-500">RECLAIM v1.0 | Autonomous Waste Sorting</span>
        </div>
      </div>

      {/* Map Area */}
      <div className="flex-1 flex flex-col">
        <div className="h-10 bg-slate-800/30 border-b border-slate-700 flex items-center px-4 gap-4 shrink-0">
          <span className="text-xs text-slate-400">Venue Map</span>
          <span className="text-xs text-slate-500 ml-auto">Scroll to zoom | Drag to pan</span>
        </div>
        <MapView
          robotPose={robot.robotPose} zones={zones}
          onAddZone={handleAddZone} isDrawing={isDrawing} setIsDrawing={setIsDrawing}
        />
      </div>

      <ToastContainer toasts={toast.toasts} onDismiss={toast.dismiss} />
      <style>{`
        @keyframes slideIn { from { transform: translateX(100%); opacity: 0 } to { transform: translateX(0); opacity: 1 } }
        .animate-slide-in { animation: slideIn 0.3s ease-out }
      `}</style>
    </div>
  )
}
