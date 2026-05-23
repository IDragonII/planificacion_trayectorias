from controller import Supervisor
import json
import os
import math

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

robot_node = robot.getSelf()
trans_field = robot_node.getField("translation")
rot_field = robot_node.getField("rotation")

print("Robot listo")

rutas = [
    "D:\\Universidad\\Robotica\\Practico\\ruta.json",
    os.path.join(os.path.dirname(__file__), "..", "..", "ruta.json"),
    "ruta.json",
]

ruta_path = None
for r in rutas:
    if os.path.exists(r):
        ruta_path = r
        print(f"Encontrado: {ruta_path}")
        break

if not ruta_path:
    print("ERROR: ruta.json no encontrado")
    exit()

with open(ruta_path) as f:
    ruta = [(p["x"], p["y"]) for p in json.load(f)]

print(f"Ruta cargada: {len(ruta)} puntos")

step = 0.03
turn = 0.08
i = 0
x = -2.0
y = -2.0
angle = 0.0

while robot.step(timestep) != -1:
    if i >= len(ruta):
        print("FIN")
        break

    tx, ty = ruta[i]
    dx = tx - x
    dy = ty - y
    dist = math.sqrt(dx*dx + dy*dy)

    if dist < 0.15:
        print(f"Punto {i+1}/{len(ruta)} ({x:.2f}, {y:.2f})")
        i += 1
    else:
        target = math.atan2(dy, dx)
        diff = target - angle

        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi

        if abs(diff) > 0.1:
            angle += turn * math.copysign(1, diff)
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
        else:
            x += math.cos(angle) * step
            y += math.sin(angle) * step

        trans_field.setSFVec3f([x, y, 0.12])
        rot_field.setSFRotation([0, 0, 1, angle])

print("Completado")