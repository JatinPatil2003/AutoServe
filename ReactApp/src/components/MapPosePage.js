import React, { useEffect, useState } from 'react';
import MapView from './MapView';

function MapPosePage({ mapName, onBack }) {
  const [poses, setPoses] = useState([]);
  const [newPose, setNewPose] = useState('');
  const [selectedPose, setSelectedPose] = useState(null);
  const [orientation, setOrientation] = useState(null);

  useEffect(() => {
    fetch(`http://localhost:8000/navigation/list/pose/${encodeURIComponent(mapName)}`)
      .then(response => response.json())
      .then(data => setPoses(data));
  }, []);

  const handleAddPose = () => {
    fetch('http://localhost:8000/navigation/new/pose', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(
        { 
          map_name: mapName,
          name: newPose,
          x: selectedPose.x,
          y: selectedPose.y,
          theta: orientation
        }),
    }).then(response => response.json());
    console.log(mapName, newPose, selectedPose, orientation)
  };

  const handleStartNavigation = () => {
    fetch('http://localhost:8000/navigation/start')
      .then(response => response.json())
      .then(() => onBack());
  };

  return (
    <div>
      <h1>Poses for Map: {mapName}</h1>
      <div>
        {poses.map(pose => (
          <button key={pose}>{pose}</button>
        ))}
      </div>
      <input
        type="text"
        placeholder="Enter new pose"
        value={newPose}
        onChange={e => setNewPose(e.target.value)}
      />
      <button onClick={handleStartNavigation}>Start Navigation</button>
      <button onClick={handleAddPose} disabled={!newPose}>Add Pose</button>
      <button onClick={onBack}>Back</button>
      <MapView 
        setSelectedPose={setSelectedPose}
        setOrientation={setOrientation}
        selectedPose={selectedPose}
        orientation={orientation}
      />
    </div>
  );
}

export default MapPosePage;
