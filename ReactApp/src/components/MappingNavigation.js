import React from 'react';

function MappingNavigation({ operation, onBack }) {
  const handleStop = () => {
    fetch(`http://68e9-65-0-134-209.ngrok-free.app/${operation}/stop`)
      .then(response => response.json())
      .then(() => onBack());
  };

  return (
    <div>
      <h1>{operation === 'mapping' ? 'Mapping' : 'Navigation'} Mode</h1>
      <button onClick={handleStop}>Stop {operation.charAt(0).toUpperCase() + operation.slice(1)}</button>
      <button onClick={onBack}>Back</button>
    </div>
  );
}

export default MappingNavigation;
