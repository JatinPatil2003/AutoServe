from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from ros.node import start_ros_node
import threading
from routes import start, navigation

ros_thread = threading.Thread(target=start_ros_node)
ros_thread.start()

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # Add the origin of your frontend
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(start.router)

app.include_router(navigation.router)


# uvicorn index:app --reload