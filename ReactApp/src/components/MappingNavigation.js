import React from 'react';

function MappingNavigation({ operation, onBack }) {
  const handleStop = () => {
    fetch(`http://172.27.232.5:8000/${operation}/stop`)
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
