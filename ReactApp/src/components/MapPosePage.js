import React, { useEffect, useState } from 'react';

function MapPosePage({ mapName, onBack }) {
  const [poses, setPoses] = useState([]);
  const [newPose, setNewPose] = useState('');

  useEffect(() => {
    fetch('http://localhost:8000/list/pose')
      .then(response => response.json())
      .then(data => setPoses(data));
  }, []);

  const handleAddPose = () => {
    fetch('http://localhost:8000/save/pose', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ pose: newPose }),
    }).then(response => response.json());
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
      <button onClick={handleAddPose} disabled={!newPose}>Add Pose</button>
      <button onClick={onBack}>Back</button>
    </div>
  );
}

export default MapPosePage;
