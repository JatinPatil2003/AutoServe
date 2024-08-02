import React, { useEffect, useState } from 'react';
import NextPage from './NextPage';

function RobotControl() {
  const [robotStatus, setRobotStatus] = useState('stopped');

  useEffect(() => {
    fetch('http://68e9-65-0-134-209.ngrok-free.app/robot/status')
      .then(response => response.json())
      .then(data => {
        if (data.status === 'started') {
          setRobotStatus('started');
        }
      });
  }, []);

  const handleStart = () => {
    fetch('http://68e9-65-0-134-209.ngrok-free.app/robot/start')
      .then(response => response.json())
      .then(() => setRobotStatus('started'));
  };

  const handleStop = () => {
    fetch('http://68e9-65-0-134-209.ngrok-free.app/robot/stop')
      .then(response => response.json())
      .then(() => setRobotStatus('stopped'));
  };

  if (robotStatus === 'started') {
    return <NextPage onBack={() => setRobotStatus('stopped')} />;
  }

  return (
    <div>
      <h1>Robot Control</h1>
      <button onClick={handleStart}>Start Robot</button>
      <button onClick={handleStop}>Stop Robot</button>
    </div>
  );
}

export default RobotControl;
