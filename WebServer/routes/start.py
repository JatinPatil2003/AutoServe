from fastapi import APIRouter
from fastapi import Request

from subprocess import Popen, PIPE
import os
import signal
import psutil

start = APIRouter()
process = None

@start.get("/robot/start")
async def get_location():
    global process
    process = Popen(['ros2', 'launch', 'autoserve_gazebo', 'gazebo.launch.py'], preexec_fn=os.setsid)
    return {'Started'}

@start.get("/robot/stop")
async def get_location():
    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
    process.wait()
    return {'Stopped'}
