from controller import Supervisor
import json
import os
import math

supervisor = Supervisor()
supervisor.step(100)

robot = supervisor.getFromDef("ROBOT")
if not robot:
    print("ERROR: Robot no encontrado")
    exit()

print("Robot encontrado")

ruta_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'ruta.json')
if not os.path.exists(ruta_path):
    ruta_path = os.path.join(os.path.dirname(__file__), '..', '..', 'ruta.json')
if not os.path.exists(ruta_path):
    ruta_path = os.path.join(os.path.dirname(__file__), '..', 'ruta.json')
if not os.path.exists(ruta_path):
    ruta_path = os.path.join(os.path.dirname(__file__), 'ruta.json')
if not os.path.exists(ruta_path):
    ruta_path = 'ruta.json'

print(f"Buscando en: {ruta_path}")

with open(ruta_path) as f:
    ruta = [(p["x"], p["y"]) for p in json.load(f)]

print(f"Ruta: {len(ruta)} puntos")

trans_field = robot.getField("translation")

i = 0
while supervisor.step(64) != -1:
    if i >= len(ruta):
        print("FIN")
        break

    x, y = ruta[i]
    pos = trans_field.getSFVec3f()
    px, py = pos[0], pos[1]

    dx = x - px
    dy = y - py
    dist = (dx**2 + dy**2)**0.5

    if dist < 0.08:
        print(f"OK {i+1}/{len(ruta)}")
        i += 1
    else:
        paso = 0.03
        nx = px + (dx / dist) * paso
        ny = py + (dy / dist) * paso
        trans_field.setSFVec3f([nx, ny, 0.1])