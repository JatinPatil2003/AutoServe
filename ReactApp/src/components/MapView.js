import React, { useState, useEffect, useRef } from 'react';
import robotIconSrc from './robot.png';

const MapView = ({ setSelectedPose, setOrientation, selectedPose, orientation }) => {
    const [mapData, setMapData] = useState(null);
    const [robotLocation, setRobotLocation] = useState(null);
    const [robotIcon, setRobotIcon] = useState(null);
    const [selectPoseMode, setSelectPoseMode] = useState(false);
    const [isDragging, setIsDragging] = useState(false);
    const [initialClick, setInitialClick] = useState(null);
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
            } catch (error) {
                console.error('Error fetching map data:', error);
            }
        };

        fetchMapData();

        const intervalId = setInterval(fetchMapData, 1000);

        return () => clearInterval(intervalId);
    }, []);

    useEffect(() => {
        const fetchRobotLocation = async () => {
            try {
                const response = await fetch('http://localhost:8000/navigation/current/location');
                if (!response.ok) {
                    throw new Error('Network response was not ok');
                }
                const data = await response.json();
                setRobotLocation(data);
            } catch (error) {
                console.error('Error fetching robot location:', error);
            }
        };

        fetchRobotLocation();

        const intervalId = setInterval(fetchRobotLocation, 1000);

        return () => clearInterval(intervalId);
    }, []);

    useEffect(() => {
        const loadRobotIcon = () => {
            const icon = new Image();
            icon.src = robotIconSrc; // Update with the correct path to your icon
            icon.onload = () => setRobotIcon(icon);
        };

        loadRobotIcon();
    }, []);

    useEffect(() => {
        if (mapData && robotIcon) {
            const canvas = canvasRef.current;
            const ctx = canvas.getContext('2d');
            const { width, height } = mapData.info;

            // Set canvas dimensions
            canvas.width = width * 2;
            canvas.height = height * 2;

            // Define cell size based on canvas size and map dimensions
            const cellWidth = canvas.width / width;
            const cellHeight = canvas.height / height;

            // Clear the canvas
            ctx.clearRect(0, 0, canvas.width, canvas.height);

            // Draw the map data
            loadMap(ctx, mapData.data, mapData.info, cellWidth, cellHeight);

            // Draw the robot location
            if (robotLocation) {
                drawRobot(ctx, robotLocation, mapData.info, cellWidth, cellHeight);
            }

            // Draw the selected pose
            if (selectedPose) {
                drawSelectedPose(ctx, selectedPose, orientation, mapData.info, cellWidth, cellHeight);
            }
        }
    }, [mapData, robotLocation, robotIcon, selectedPose, orientation]);

    const sendPose = async (pose) => {
        try {
            const response = await fetch('http://localhost:8000/navigation/new/pose', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(pose),
            });

            if (!response.ok) {
                throw new Error('Network response was not ok');
            }

            console.log('Pose sent successfully');
        } catch (error) {
            console.error('Error sending pose:', error);
        }
    };

    const loadMap = (ctx, mapData, mapInfo, cellWidth, cellHeight) => {
        // Loop through map data and draw onto canvas
        for (let y = 0; y < mapInfo.height; y++) {
            for (let x = 0; x < mapInfo.width; x++) {
                const val = mapData[(mapInfo.width - x) + y * mapInfo.width];

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

    const drawRobot = (ctx, location, mapInfo, cellWidth, cellHeight) => {
        const { x, y, theta } = location;
        // const robotX = (x - mapInfo.origin.position.x) / mapInfo.resolution * cellWidth;
        // const robotY = (y - mapInfo.origin.position.y) / mapInfo.resolution * cellHeight;
        const robotX = (-x - mapInfo.origin.position.x) / mapInfo.resolution * cellWidth;
        const robotY = (y - mapInfo.origin.position.y) / mapInfo.resolution * cellHeight;
        
        // console.log(x, y, robotX, robotY);
        // Draw robot icon
        const iconSize = 20; // Adjust the icon size if needed
        ctx.save();
        ctx.translate(robotX, robotY);
        ctx.rotate(-theta - Math.PI / 2); // Rotate the icon based on the robot's orientation
        ctx.drawImage(robotIcon, -iconSize / 2, -iconSize / 2, iconSize, iconSize);
        ctx.restore();
    };

    const drawSelectedPose = (ctx, pose, orientation, mapInfo, cellWidth, cellHeight) => {
        const { x, y } = pose;
        const poseX = (-x - mapInfo.origin.position.x) / mapInfo.resolution * cellWidth;
        const poseY = (y - mapInfo.origin.position.y) / mapInfo.resolution * cellHeight;

        if (isDragging) {
            const iconSize = 20; // Adjust the icon size if needed
            ctx.save();
            ctx.translate(poseX, poseY);
            ctx.rotate(-orientation - Math.PI / 2); // Rotate the icon based on the robot's orientation
            ctx.drawImage(robotIcon, -iconSize / 2, -iconSize / 2, iconSize, iconSize);
            ctx.restore();
        }
    };

    const handleCanvasMouseDown = (event) => {
        if (!selectPoseMode) return;

        const canvas = canvasRef.current;
        const rect = canvas.getBoundingClientRect();
        const x = event.clientX - rect.left;
        const y = event.clientY - rect.top;

        const mapX = (x / canvas.width) * mapData.info.width * mapData.info.resolution + mapData.info.origin.position.x;
        const mapY = ((canvas.height - y) / canvas.height) * mapData.info.height * mapData.info.resolution + mapData.info.origin.position.y;

        setSelectedPose({ x: -mapX, y: -mapY });
        // console.log({ x: mapX, y: mapY });
        setInitialClick({ x, y });
        setIsDragging(true);
        // console.log(selectPoseMode, isDragging, selectedPose);
    };

    const handleCanvasMouseMove = (event) => {
        console.log(selectPoseMode, isDragging, orientation);
        if (!selectPoseMode || !isDragging || !selectedPose) return;

        const canvas = canvasRef.current;
        const rect = canvas.getBoundingClientRect();
        const x = event.clientX - rect.left;
        const y = event.clientY - rect.top;
        // console.log(x, y);

        const dx = - x + initialClick.x;
        const dy = y - initialClick.y;
        const angle = Math.atan2(dy, dx);

        setOrientation(angle);
    };

    const handleCanvasMouseUp = () => {
        if (isDragging) {
            setIsDragging(false);
            console.log({ x: selectedPose.x, y: selectedPose.y, theta: orientation });
            // sendPose({ x: selectedPose.x, y: selectedPose.y, theta: orientation });
            setSelectPoseMode(false);
        }
    };

    const handleSelectPoseClick = () => {
        setSelectPoseMode(true);
        setSelectedPose(null);
        setOrientation(null);
    };

    return (
        <div>
            <button onClick={handleSelectPoseClick}>Select Pose</button>
            <canvas
                ref={canvasRef}
                style={{ border: '1px solid black' }}
                onMouseDown={handleCanvasMouseDown}
                onMouseMove={handleCanvasMouseMove}
                onMouseUp={handleCanvasMouseUp}
            />
        </div>
    );
};

export default MapView;