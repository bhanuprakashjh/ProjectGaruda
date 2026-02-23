import { ConnectionBar } from './components/ConnectionBar';
import { StatusPanel } from './components/StatusPanel';
import { GaugePanel } from './components/GaugePanel';
import { TimeChart } from './components/TimeChart';
import { ControlPanel } from './components/ControlPanel';
import { ThrottleSlider } from './components/ThrottleSlider';
import { ParamPanel } from './components/ParamPanel';

export default function App() {
  return (
    <div style={{ maxWidth: 960, margin: '0 auto', padding: 16 }}>
      <ConnectionBar />
      <div style={{ display: 'grid', gridTemplateColumns: '200px 1fr', gap: 16, marginTop: 16 }}>
        <StatusPanel />
        <GaugePanel />
      </div>
      <TimeChart />
      <ControlPanel />
      <ThrottleSlider />
      <ParamPanel />
    </div>
  );
}
