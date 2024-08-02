import React, { useState } from 'react';

function MappingPage({ onBack }) {
  const [mapName, setMapName] = useState('');

  const handleStopMapping = () => {
    fetch('http://172.27.232.5:8000/mapping/stop')
      .then(response => response.json())
      .then(() => onBack());
  };

  const handleSaveMap = () => {
    fetch('http://172.27.232.5:8000/save_map', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ name: mapName }),
    }).then(response => response.json());
  };

  return (
    <div>
      <h1>Mapping Mode</h1>
      <input
        type="text"
        placeholder="Enter map name"
        value={mapName}
        onChange={e => setMapName(e.target.value)}
      />
      <button onClick={handleSaveMap} disabled={!mapName}>Save Map</button>
      <button onClick={handleStopMapping}>Stop Mapping</button>
      <button onClick={onBack}>Back</button>
    </div>
  );
}

export default MappingPage;
