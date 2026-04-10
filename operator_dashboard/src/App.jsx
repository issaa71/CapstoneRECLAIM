import { useState, lazy, Suspense } from 'react'

const OperatorDashboard = lazy(() => import('./OperatorDashboard'))
const NavigationSimulation = lazy(() => import('./NavigationSimulation'))

const TABS = [
  { id: 'dashboard', label: 'Operator Dashboard', icon: '🎛️' },
  { id: 'simulation', label: 'Navigation Simulation', icon: '🗺️' },
]

function LoadingSpinner() {
  return (
    <div className="flex items-center justify-center h-screen bg-slate-900">
      <div className="text-center">
        <div className="w-12 h-12 border-4 border-emerald-500 border-t-transparent rounded-full animate-spin mx-auto mb-4" />
        <p className="text-slate-400 text-sm">Loading...</p>
      </div>
    </div>
  )
}

export default function App() {
  const [activeTab, setActiveTab] = useState('dashboard')

  return (
    <div className="h-screen flex flex-col bg-slate-900">
      {/* Top Navigation Bar */}
      <nav className="h-14 bg-slate-800 border-b border-slate-700 flex items-center px-4 shrink-0 z-50">
        <div className="flex items-center gap-2 mr-8">
          <div className="w-8 h-8 rounded-lg bg-emerald-600 flex items-center justify-center font-bold text-sm text-white">R</div>
          <span className="text-white font-semibold text-lg tracking-tight">RECLAIM</span>
        </div>

        <div className="flex gap-1">
          {TABS.map(tab => (
            <button
              key={tab.id}
              onClick={() => setActiveTab(tab.id)}
              className={`px-4 py-2 rounded-lg text-sm font-medium transition-all flex items-center gap-2 ${
                activeTab === tab.id
                  ? 'bg-slate-700 text-white'
                  : 'text-slate-400 hover:text-slate-200 hover:bg-slate-700/50'
              }`}
            >
              <span>{tab.icon}</span>
              <span>{tab.label}</span>
            </button>
          ))}
        </div>

        <div className="ml-auto flex items-center gap-3">
          <span className="text-xs text-slate-500">Autonomous Waste Sorting Robot</span>
          <a href="https://github.com/issaa71/RECLAIM" target="_blank" rel="noopener"
            className="text-xs text-slate-400 hover:text-emerald-400 transition-colors">
            GitHub
          </a>
        </div>
      </nav>

      {/* Content */}
      <div className="flex-1 overflow-hidden">
        <Suspense fallback={<LoadingSpinner />}>
          {activeTab === 'dashboard' && (
            <OperatorDashboard onSwitchToSimulation={() => setActiveTab('simulation')} />
          )}
          {activeTab === 'simulation' && <NavigationSimulation />}
        </Suspense>
      </div>
    </div>
  )
}
