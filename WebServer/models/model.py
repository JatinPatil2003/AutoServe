from pydantic import BaseModel

class Pose(BaseModel):
    name: str
    x: float
    y: float
    theta: float

class Goal(Pose):
    map_name: str

