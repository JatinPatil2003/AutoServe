import React, { useEffect, useState } from 'react';
import MapPosePage from './MapPosePage';

function NavigationPage({ onBack }) {
  const [maps, setMaps] = useState([]);
  const [selectedMap, setSelectedMap] = useState('');
  const [useMap, setUseMap] = useState(false);
  const [startNav, setStartNav] = useState(false);
  const [stopButton, setStopButton] = useState(true);

  useEffect(() => {
    fetch('http://172.27.232.2:8000/navigation/list/maps')
      .then(response => response.json())
      .then(data => setMaps(data));
  }, []);

  const handleUseMap = () => {
    if (selectedMap) {
      fetch('http://172.27.232.2:8000/navigation/use_map', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ name: selectedMap }),
      })
      .then(response => response.json())
      .then(() => setUseMap(true));
    }
  };

  const handleStopNavigation = () => {
    fetch('http://172.27.232.2:8000/navigation/stop')
      .then(response => response.json())
      .then(() => setSelectedMap(''))
      .then(() => setStopButton(true));
  };

  const handleStartNavigation = () => {
    fetch('http://172.27.232.2:8000/navigation/start', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(
        { 
          name: selectedMap
        }),
    })
    .then(response => response.json())
    .then(() => setStartNav(true))
    .then(() => setStopButton(false))
    .then(() => setUseMap(false));
  };

  if (startNav) {
    return <MapPosePage mapName={selectedMap} onBack={() => setStartNav(false)} />;
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
      <button class="btn btn-primary" onClick={handleUseMap} disabled={!selectedMap}>Use Map: {selectedMap}</button>
      <button class="btn btn-primary" onClick={handleStartNavigation} disabled={!useMap}>Start Navigation</button>
      <button class="btn btn-primary" onClick={handleStopNavigation} disabled={stopButton}>Stop Navigation</button>
      <button class="btn btn-primary" onClick={onBack}>Back</button>
    </div>
  );
}

export default NavigationPage;
