import React, { useState, useEffect, useRef } from 'react';

const MapView = () => {
    const [mapData, setMapData] = useState(null);
    const canvasRef = useRef(null);

    useEffect(() => {
        const fetchMapData = async () => {
            try {
                const response = await fetch('http://localhost:8000/navigation/current/map');
                if (!response.ok) {
                    throw new Error('Network response was not ok');
                }
                const data = await response.json();
                setMapData(data);
                console.log(mapData);
            } catch (error) {
                console.error('Error fetching map data:', error);
            }
        };

        fetchMapData();

        const intervalId = setInterval(fetchMapData, 1000); 

        return () => clearInterval(intervalId);
    }, []);

    useEffect(() => {
        if (mapData) {
            const canvas = canvasRef.current;
            const ctx = canvas.getContext('2d');
            const { width, height } = mapData.info;
            
            // Set canvas dimensions
            canvas.width = width*2;
            canvas.height = height*2;

            // Define cell size based on canvas size and map dimensions
            const cellWidth = canvas.width / width;
            const cellHeight = canvas.height / height;

            // Clear the canvas
            ctx.clearRect(0, 0, canvas.width, canvas.height);

            // Draw the map data
            loadMap(ctx, mapData.data, mapData.info, cellWidth, cellHeight);
        }
    }, [mapData]);

    const loadMap = (ctx, mapData, mapInfo, cellWidth, cellHeight) => {
        // Loop through map data and draw onto canvas
        for (let y = 0; y < mapInfo.height; y++) {
            for (let x = 0; x < mapInfo.width; x++) {
                const val = mapData[x + y * mapInfo.width];

                // Set fill color based on occupancy value
                ctx.fillStyle = getColorForOccupancy(val);

                // Draw rectangle representing map cell
                ctx.fillRect(x * cellWidth, y * cellHeight, cellWidth, cellHeight);

                // Optional: Draw border for occupied cells
                if (isOccupied(val)) {
                    ctx.strokeStyle = 'black';
                    ctx.lineWidth = 1; // Set the border width to 1 pixel
                    ctx.strokeRect(x * cellWidth, y * cellHeight, cellWidth, cellHeight);
                }
            }
        }
    };

    const getColorForOccupancy = (value) => {
        if (value === 0) return 'white';  // Free space
        if (value === 100) return 'black'; // Occupied
        return 'gray';                    // Unknown
    };

    const isOccupied = (value) => value === 100;

    return (
        <div>
            <canvas ref={canvasRef} style={{ border: '1px solid black' }} />
        </div>
    );
};

export default MapView;

