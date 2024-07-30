#!/usr/bin/env python3
from fastapi import APIRouter
from fastapi import Request

from subprocess import Popen, PIPE
import os
import signal
import psutil
from ros.node import ros_node

router = APIRouter()
process = None

@router.get("/navigation/start")
async def start_navigation():
    global process
    process = Popen(['ros2', 'launch', 'autoserve_navigation', 'navigation.launch.py'], preexec_fn=os.setsid)
    # process = Popen(['pwd'], preexec_fn=os.setsid)
    return {'Started'}

@router.get("/navigation/stop")
async def stop_naviagtion():
    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
    process.wait()
    return {'Stopped'}
