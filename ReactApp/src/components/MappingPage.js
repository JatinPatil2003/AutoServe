import React, { useState, useEffect } from "react";
import JoystickControl from "./JoystickView";
import MappingMap from "./MappingMap";

function MappingPage({ onBack }) {
  const [mapName, setMapName] = useState("");
  const [linear, setLinear] = useState(0.0);
  const [angular, setAngular] = useState(0.0);

  const handleStopMapping = () => {
    fetch("http://13.201.82.2:5747/mapping/stop")
      .then((response) => response.json())
      .then(() => onBack());
  };

  const handleSaveMap = () => {
    fetch("http://13.201.82.2:5747/save_map", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({ name: mapName }),
    }).then((response) => response.json());
  };

  const handleStartMapping = async () => {
    const response = await fetch(
      "http://13.201.82.2:5747/mapping/start"
    );
    console.log(response.json());
  };

  const handleJoystickControl = (linear, angular) => {
    setLinear(linear);
    setAngular(angular);
  };

  useEffect(() => {
    const setvelocity = async () => {
      try {
        const response = await fetch("http://13.201.82.2:5747/joystick/control", {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify({ linear: linear, angular: angular }),
        });
        if (!response.ok) {
          throw new Error("Network response was not ok");
        }
        const data = await response.json();
        console.log(data);
      } catch (error) {
        console.error("Error fetching robot location:", error);
      }
    };

    setvelocity();

    // const intervalId = setInterval(setvelocity, 200);

    // return () => clearInterval(intervalId);
  }, [linear, angular]);

  return (
    <div>
      <h1>Mapping Mode</h1>
      <input
        type="text"
        placeholder="Enter map name"
        value={mapName}
        onChange={(e) => setMapName(e.target.value)}
      />
      <button onClick={handleSaveMap} disabled={!mapName}>
        Save Map
      </button>
      <button onClick={handleStopMapping}>Stop Mapping</button>
      <button onClick={handleStartMapping}>Start Mapping</button>
      <button onClick={onBack}>Back</button>
      <JoystickControl onControl={handleJoystickControl} />
      <MappingMap />
    </div>
  );
}

export default MappingPage;
