import React, { useEffect, useState } from 'react';
import MapPosePage from './MapPosePage';

function NavigationPage({ onBack }) {
  const [maps, setMaps] = useState([]);
  const [selectedMap, setSelectedMap] = useState('');
  const [useMap, setUseMap] = useState(false);

  useEffect(() => {
    fetch('http://localhost:8000/list/maps')
      .then(response => response.json())
      .then(data => setMaps(data));
  }, []);

  const handleUseMap = () => {
    if (selectedMap) {
      fetch(`http://localhost:8000/use_map?name=${selectedMap}`)
        .then(response => response.json())
        .then(() => setUseMap(true));
    }
  };

  const handleStopNavigation = () => {
    fetch('http://localhost:8000/navigation/stop')
      .then(response => response.json())
      .then(() => onBack());
  };

  if (useMap) {
    return <MapPosePage mapName={selectedMap} onBack={() => setUseMap(false)} />;
  }

  return (
    <div>
      <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.3/dist/css/bootstrap.min.css" rel="stylesheet" integrity="sha384-QWTKZyjpPEjISv5WaRU9OFeRpok6YctnYmDr5pNlyT2bRjXh0JMhjY6hW+ALEwIH" crossorigin="anonymous"></link>
      <h1>Select a Map for Navigation</h1>
      <div>
        {maps.map(map => (
          <button key={map} onClick={() => setSelectedMap(map)}>{map}</button>
        ))}
      </div>
      <button class="btn btn-primary" onClick={handleUseMap} disabled={!selectedMap}>Use Map</button>
      <button class="btn btn-primary" onClick={handleStopNavigation}>Stop Navigation</button>
      <button class="btn btn-primary" onClick={onBack}>Back</button>
    </div>
  );
}

export default NavigationPage;
