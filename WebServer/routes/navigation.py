#!/usr/bin/env python3
from fastapi import APIRouter
from fastapi import Request

from subprocess import Popen, PIPE
import os
import signal
import psutil
from ros.topics import get_map_msg, get_location_msg
from mongodb.db import listMaps, listGoal, saveGoal, getGoal
from models.model import MapName, Goal, Pose

router = APIRouter()
process = None

@router.get("/navigation/start")
async def start_navigation():
    global process
    if not process:
        process = Popen(['ros2', 'launch', 'autoserve_navigation', 'navigation.launch.py'], preexec_fn=os.setsid)
    # process = Popen(['pwd'], preexec_fn=os.setsid)
    return {'Started'}

@router.get("/navigation/stop")
async def stop_naviagtion():
    global process
    if process:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait()
        process = None
    process = None
    return {'Stopped'}

@router.get("/navigation/list/maps")
async def list_maps():
    return listMaps()

@router.post("/navigation/use_map")
async def save_map_data(name: MapName):
    global map_name
    map_name = name.name
    return {"message": "Map data saved successfully"}

@router.get("/navigation/list/pose/{name}")
async def list_poses(name: str):
    return listGoal(name)

@router.get("/navigation/current/map")
async def get_current_map():
    map_msg = get_map_msg()
    if map_msg:
        return map_msg
    return None

@router.get("/navigation/current/location")
async def get_current_location():
    location_msg = get_location_msg()
    if location_msg:
        return location_msg
    return None

@router.post("/navigation/new/pose")
async def save_map_data(goal: Goal):
    print(goal)
    saveGoal(goal)
    return {"message": "Map data saved successfully"}

@router.post("/navigation/pose")
async def save_map_data(goal: Goal):
    print(goal)
    return getGoal(goal.map_name, goal.name)

@router.post("/navigation/goal/start")
async def save_map_data(pose: Pose):
    print(pose)
    return {'Navigation Started'}

@router.get("/navigation/goal/cancel")
async def stop_navigation():
    return {'Navigation Stopped'}