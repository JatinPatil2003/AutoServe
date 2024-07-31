import React, { useEffect, useState } from 'react';
import MapView from './MapView';

function MapPosePage({ mapName, onBack }) {
  const [poses, setPoses] = useState([]);
  const [newPose, setNewPose] = useState('');
  const [selectedPose, setSelectedPose] = useState(null);
  const [orientation, setOrientation] = useState(null);
  const [goalPose, setGoalPose] = useState(null);
  const [navStatus, setNavStatus] = useState('');
  const [changeNavStatus, setChangeNavStatus] = useState(null);
  const [remainDist, setRemainDist] = useState(0.0);

  useEffect(() => {
    fetch(`http://localhost:8000/navigation/list/pose/${encodeURIComponent(mapName)}`)
      .then(response => response.json())
      .then(data => setPoses(data));
  }, []);

  useEffect(() => {
    const fetchRobotLocation = async () => {
        try {
            const response = await fetch('http://localhost:8000/navigation/goal/feedback');
            if (!response.ok) {
                throw new Error('Network response was not ok');
            }
            const data = await response.json();
            const roundedData = parseFloat(data).toFixed(2);
            setRemainDist(roundedData);
            if(parseFloat(roundedData) === 0 && changeNavStatus){
              setNavStatus('Success');
            }
            else if(parseFloat(roundedData) !== 0){
              setChangeNavStatus(true);
                setNavStatus('Running');
            }
        } catch (error) {
            console.error('Error fetching robot location:', error);
        }
    };

    fetchRobotLocation();

    const intervalId = setInterval(fetchRobotLocation, 400);

    return () => clearInterval(intervalId);
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

  const handleNavigationGoalStart = async () => {
    const response_navigation = await fetch('http://localhost:8000/navigation/goal/start', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(
        goalPose
      ),
    });

    const data_navigation = await response_navigation.json();
    console.log(data_navigation);
  };

  const handleNavigationGoalStop = async () => {
    const response_cancel = await fetch('http://localhost:8000/navigation/goal/cancel', {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    if (!response_cancel.ok) {
      throw new Error('Network response was not ok');
    }

    const data_cancel = response_cancel.json();
    console.log(data_cancel);
    setGoalPose(null);
    setNavStatus('Cancelled');
    console.log(navStatus);
    setChangeNavStatus(false);
  };

  const handleGoalPoseDetails = async (pose) => {
    const response = await fetch('http://localhost:8000/navigation/pose', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        map_name: mapName,
        name: pose,
        x: 0.0,
        y: 0.0,
        theta: 0.0,
      }),
    });

    const data = await response.json();
    console.log(data);
    setGoalPose(data);
    console.log(goalPose);
  };

  return (
    <div>
      <h1>Poses for Map: {mapName}</h1>
      <div>
        {poses.map(pose => (
          <button onClick={() => handleGoalPoseDetails(pose)} key={pose}>{pose}</button>
        ))}
      </div>
      <input
        type="text"
        placeholder="Enter new pose"
        value={newPose}
        onChange={e => setNewPose(e.target.value)}
      />
      <button onClick={handleNavigationGoalStart}>Go to Goal</button>
      <button onClick={handleNavigationGoalStop}>Cancel Goal</button>
      <button onClick={handleAddPose} disabled={!newPose}>Add Pose</button>
      <button onClick={onBack}>Back</button>
      <p>Navigation Status: {navStatus}, Distance Remaining: {remainDist}</p>
      <MapView 
        setSelectedPose={setSelectedPose}
        setOrientation={setOrientation}
        selectedPose={selectedPose}
        orientation={orientation}
        goalPose={goalPose}
      />
    </div>
  );
}

export default MapPosePage;
