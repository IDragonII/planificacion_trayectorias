import numpy as np
import heapq
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import ListedColormap
import matplotlib.gridspec as gridspec
import json
import time

# ═══════════════════════════════════════════════════════════════
#  ALGORITMO A*
# ═══════════════════════════════════════════════════════════════

def h(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def astar(grid, start, goal):
    R, C = grid.shape
    heap = [(h(start,goal), 0, start, [start])]
    vis = set(); orden = []
    while heap:
        f, g, cur, path = heapq.heappop(heap)
        if cur in vis: continue
        vis.add(cur); orden.append(cur)
        if cur == goal: return path, orden
        r, c = cur
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            nr, nc = r+dr, c+dc; nb = (nr,nc)
            if 0<=nr<R and 0<=nc<C and grid[nr][nc]==0 and nb not in vis:
                ng = g+1
                heapq.heappush(heap, (ng+h(nb,goal), ng, nb, path+[nb]))
    return None, orden

def longitud_ruta(path, cell_m):
    return sum(abs(path[i][0]-path[i-1][0]) + abs(path[i][1]-path[i-1][1])
               for i in range(1,len(path))) * cell_m

# ═══════════════════════════════════════════════════════════════
#  MAPA 1 — Simple (15x20)
# ═══════════════════════════════════════════════════════════════

R1, C1 = 15, 20
grid1 = np.zeros((R1,C1), dtype=int)
grid1[0,:]=1; grid1[-1,:]=1; grid1[:,0]=1; grid1[:,-1]=1
for r in range(3,12):
    for c in range(3,17): grid1[r][c]=1
grid1[1][10]=1; grid1[1][11]=1; grid1[7][18]=1; grid1[13][9]=1
START1, GOAL1 = (13,1), (1,18)
OBS1 = {
    (1,10):('carrito','#F97316'), (1,11):('carrito','#F97316'),
    (7,18):('silla','#F97316'),   (13,9):('persona','#EF4444'),
}

# ═══════════════════════════════════════════════════════════════
#  MAPA 2 — Complejo (30x45) — tu version
# ═══════════════════════════════════════════════════════════════

R2, C2 = 30, 45
grid2 = np.zeros((R2,C2), dtype=int)

grid2[0,:]=1; grid2[-1,:]=1; grid2[:,0]=1; grid2[:,-1]=1

# Bloque central
grid2[7:23,16:29]=1
grid2[9:21,22]=0; grid2[14,18:27]=0; grid2[18,18:27]=0

# Bloques laterales
grid2[3:10,4:12]=1;  grid2[6,11]=0
grid2[3:10,33:41]=1; grid2[6,33]=0
grid2[20:27,4:12]=1; grid2[23,11]=0
grid2[20:27,33:41]=1;grid2[23,33]=0

# Muros internos horizontales
grid2[12,2:15]=1; grid2[12,30:43]=1
grid2[17,2:15]=1; grid2[17,30:43]=1
grid2[12,7]=0;  grid2[12,37]=0
grid2[17,10]=0; grid2[17,35]=0

# Muros internos verticales
grid2[4:26,14]=1; grid2[4:26,30]=1
grid2[8,14]=0;  grid2[15,14]=0; grid2[22,14]=0
grid2[8,30]=0;  grid2[15,30]=0; grid2[22,30]=0

# Mini laberinto izquierda
grid2[5:24,6]=1; grid2[8:27,9]=1
grid2[10,6]=0; grid2[16,6]=0; grid2[21,6]=0
grid2[13,9]=0; grid2[19,9]=0

# Mini laberinto derecha
grid2[5:24,35]=1; grid2[8:27,38]=1
grid2[9,35]=0;  grid2[15,35]=0; grid2[22,35]=0
grid2[12,38]=0; grid2[20,38]=0

OBS2 = {
    (4,18):('persona','#EF4444'),  (5,25):('persona','#EF4444'),
    (8,20):('persona','#EF4444'),  (11,33):('persona','#EF4444'),
    (13,5):('persona','#EF4444'),  (16,24):('persona','#EF4444'),
    (18,40):('persona','#EF4444'), (22,19):('persona','#EF4444'),
    (24,8):('persona','#EF4444'),  (25,36):('persona','#EF4444'),
    (3,15):('carrito','#F97316'),  (6,28):('carrito','#F97316'),
    (9,12):('carrito','#F97316'),  (14,21):('carrito','#F97316'),
    (17,32):('carrito','#F97316'), (19,26):('carrito','#F97316'),
    (23,14):('carrito','#F97316'), (26,29):('carrito','#F97316'),
    (7,41):('silla','#F97316'),    (10,17):('silla','#F97316'),
    (12,27):('silla','#F97316'),   (15,8):('silla','#F97316'),
    (18,14):('silla','#F97316'),   (21,34):('silla','#F97316'),
    (24,24):('silla','#F97316'),   (27,38):('silla','#F97316'),
}
for (r,c) in OBS2: grid2[r,c]=1

START2, GOAL2 = (28,2), (2,42)
grid2[START2]=0; grid2[GOAL2]=0

# ═══════════════════════════════════════════════════════════════
#  CALCULAR RUTAS Y METRICAS
# ═══════════════════════════════════════════════════════════════

print("Calculando rutas...")

t1i = time.perf_counter()
path1, visited1 = astar(grid1, START1, GOAL1)
t1_ms = (time.perf_counter()-t1i)*1000

t2i = time.perf_counter()
path2, visited2 = astar(grid2, START2, GOAL2)
t2_ms = (time.perf_counter()-t2i)*1000

dist1 = longitud_ruta(path1, 0.25)
dist2 = longitud_ruta(path2, 0.22)

print(f"  Mapa 1: {len(path1)} pasos | {len(visited1)} exploradas | {t1_ms:.3f} ms | {dist1:.2f} m")
print(f"  Mapa 2: {len(path2)} pasos | {len(visited2)} exploradas | {t2_ms:.3f} ms | {dist2:.2f} m")

# Exportar ruta.json para Webots
def celda_a_real(r, c): return (c*0.22, (R2-1-r)*0.22)
waypoints = [{"x":round(celda_a_real(r,c)[0],3),"y":round(celda_a_real(r,c)[1],3)} for r,c in path2]
with open("ruta.json","w") as f:
    json.dump(waypoints, f, indent=2)
print("  ruta.json exportado.")

# ═══════════════════════════════════════════════════════════════
#  MOTOR DE ANIMACION
# ═══════════════════════════════════════════════════════════════

cmap_grid = ListedColormap(['#F1F5F9','#CBD5E1'])

def dibujar_base(ax, grid, R, C, obs, start, goal):
    ax.clear()
    ax.set_facecolor('#F8F9FA')
    ax.imshow(grid, cmap=cmap_grid, vmin=0, vmax=1, origin='upper', aspect='equal')
    for r in range(R+1): ax.axhline(r-0.5, color='#E2E8F0', lw=0.3)
    for c in range(C+1): ax.axvline(c-0.5, color='#E2E8F0', lw=0.3)
    for (r,c),(tipo,color) in obs.items():
        if tipo=='persona':
            ax.add_patch(plt.Circle((c,r),0.38,facecolor='#FCA5A5',edgecolor='#EF4444',lw=1.5,zorder=3))
            ax.text(c,r,'P',ha='center',va='center',fontsize=6,color='#EF4444',fontweight='bold',zorder=4)
        else:
            ax.add_patch(plt.Rectangle((c-0.42,r-0.42),0.84,0.84,facecolor='#FED7AA',edgecolor='#F97316',lw=1.5,zorder=3))
            ax.text(c,r,'C' if tipo=='carrito' else 'S',ha='center',va='center',fontsize=6,color='#F97316',fontweight='bold',zorder=4)
    for pos,lbl,color in [(start,'S','#10B981'),(goal,'G','#EF4444')]:
        ax.add_patch(mpatches.FancyBboxPatch((pos[1]-0.45,pos[0]-0.45),0.9,0.9,
            boxstyle='round,pad=0.05',facecolor=color,edgecolor=color,lw=0,zorder=5))
        ax.text(pos[1],pos[0],lbl,ha='center',va='center',fontsize=8,color='white',fontweight='bold',zorder=6)
    ax.set_xlim(-0.5,C-0.5); ax.set_ylim(R-0.5,-0.5)
    ax.set_xticks([]); ax.set_yticks([])
    ax.spines[:].set_visible(False)

def simular(grid, R, C, obs, start, goal, path, visited, titulo, vel=0.06):
    path_cols=[c for r,c in path]; path_rows=[r for r,c in path]
    STEP=max(1,len(visited)//100)
    fig,ax=plt.subplots(figsize=(max(11,C*0.42),max(8,R*0.42)))
    fig.patch.set_facecolor('#F8F9FA')
    fig.suptitle(f'Planificacion de Trayectorias — Algoritmo A*\n{titulo}',
                 fontsize=12,fontweight='bold',color='#1E3A5F',y=0.99)

    # Fase 1 — exploracion
    for i in range(0,len(visited),STEP):
        dibujar_base(ax,grid,R,C,obs,start,goal)
        for r,c in visited[:i+1]:
            if (r,c)!=start and (r,c)!=goal:
                ax.add_patch(plt.Rectangle((c-0.5,r-0.5),1,1,facecolor='#BAE6FD',alpha=0.6,zorder=2))
        pct=int((i+1)/len(visited)*100)
        ax.set_title(f'Fase 1 — A* explorando...  {i+1}/{len(visited)} celdas ({pct}%)',
                     color='#2563EB',fontsize=10,pad=5)
        plt.tight_layout(); plt.pause(vel*0.35)

    # Fase 2 — ruta optima
    dibujar_base(ax,grid,R,C,obs,start,goal)
    for r,c in visited:
        if (r,c)!=start and (r,c)!=goal:
            ax.add_patch(plt.Rectangle((c-0.5,r-0.5),1,1,facecolor='#DBEAFE',alpha=0.4,zorder=2))
    for r,c in path:
        if (r,c)!=start and (r,c)!=goal:
            ax.add_patch(plt.Rectangle((c-0.5,r-0.5),1,1,facecolor='#BBF7D0',alpha=0.9,zorder=3))
    ax.plot(path_cols,path_rows,color='#4ADE80',lw=2.5,zorder=4,alpha=0.9)
    ax.set_title(f'Fase 2 — Ruta optima: {len(path)} pasos  |  {len(visited)} exploradas',
                 color='#16A34A',fontsize=10,pad=5)
    plt.tight_layout(); plt.pause(2.0)

    # Fase 3 — robot
    trail=[]
    for idx,(r,c) in enumerate(path):
        dibujar_base(ax,grid,R,C,obs,start,goal)
        for er,ec in visited:
            if (er,ec)!=start and (er,ec)!=goal:
                ax.add_patch(plt.Rectangle((ec-0.5,er-0.5),1,1,facecolor='#DBEAFE',alpha=0.08,zorder=2))
        ax.plot(path_cols,path_rows,color='#4ADE80',lw=1.5,zorder=3,alpha=0.3,linestyle='--')
        trail.append((r,c))
        for tr,tc in trail[:-1]:
            if (tr,tc)!=start and (tr,tc)!=goal:
                ax.add_patch(plt.Rectangle((tc-0.5,tr-0.5),1,1,facecolor='#D1FAE5',alpha=0.7,zorder=4))
        ax.add_patch(plt.Circle((c,r),0.42,facecolor='#FDE68A',edgecolor='#F59E0B',lw=2.5,zorder=5))
        ax.text(c,r,'R',ha='center',va='center',fontsize=8,color='#1E293B',fontweight='bold',zorder=6)
        pct=int(idx/(len(path)-1)*100) if len(path)>1 else 100
        ax.set_title(f'Fase 3 — Robot navegando...  Paso {idx+1}/{len(path)} ({pct}%)',
                     color='#D97706',fontsize=10,pad=5)
        plt.tight_layout(); plt.pause(vel)

    # Llegada
    dibujar_base(ax,grid,R,C,obs,start,goal)
    for tr,tc in trail:
        ax.add_patch(plt.Rectangle((tc-0.5,tr-0.5),1,1,facecolor='#D1FAE5',alpha=0.7,zorder=4))
    ax.plot(path_cols,path_rows,color='#4ADE80',lw=2,zorder=3,alpha=0.6)
    ax.set_title(f'Destino alcanzado!   {len(path)} pasos  |  {len(visited)} exploradas',
                 color='#10B981',fontsize=11,pad=5,fontweight='bold')
    leyenda=[
        mpatches.Patch(color='#CBD5E1',label='Pared'),
        mpatches.Patch(color='#BAE6FD',label='Explorado A*'),
        mpatches.Patch(color='#BBF7D0',label='Ruta optima'),
        mpatches.Patch(color='#FDE68A',label='Robot (R)'),
        mpatches.Patch(color='#FCA5A5',label='Persona (P)'),
        mpatches.Patch(color='#FED7AA',label='Carrito/Silla (C/S)'),
    ]
    ax.legend(handles=leyenda,loc='lower center',bbox_to_anchor=(0.5,-0.05),
              ncol=6,fontsize=7.5,framealpha=0.5)
    plt.tight_layout(); plt.pause(2.5)
    plt.close()

# ═══════════════════════════════════════════════════════════════
#  ANALISIS COMPARATIVO — Punto 3 del enunciado
# ═══════════════════════════════════════════════════════════════

def mostrar_comparativo():
    fig = plt.figure(figsize=(14,9))
    fig.patch.set_facecolor('#F8F9FA')
    gs = gridspec.GridSpec(2,3,figure=fig,hspace=0.45,wspace=0.38)

    datos = [
        ([len(path1),    len(path2)],    'Pasos de la ruta',        'Pasos',    '#3B82F6','#2563EB'),
        ([len(visited1), len(visited2)], 'Celdas exploradas por A*','Celdas',   '#8B5CF6','#7C3AED'),
        ([t1_ms,         t2_ms],         'Tiempo de computo',       'ms',       '#10B981','#059669'),
        ([dist1,         dist2],         'Longitud de trayectoria', 'metros',   '#F59E0B','#D97706'),
        ([len(OBS1),     len(OBS2)],     'Obstaculos dinamicos',    'Cantidad', '#EF4444','#DC2626'),
        ([R1*C1,         R2*C2],         'Tamano del mapa',         'Celdas',   '#6B7280','#4B5563'),
    ]
    mapas = ['Mapa 1\nSimple (15x20)','Mapa 2\nComplejo (30x45)']
    fig.suptitle('Analisis Comparativo — Algoritmo A* en dos configuraciones',
                 fontsize=14,fontweight='bold',color='#1E3A5F',y=0.98)

    for i,(vals,titulo,unidad,c1,c2) in enumerate(datos):
        ax = fig.add_subplot(gs[i//3,i%3])
        ax.set_facecolor('#FFFFFF')
        bars = ax.bar(mapas,vals,color=[c1,c2],width=0.5,edgecolor='white',linewidth=1.5)
        for bar,val in zip(bars,vals):
            lbl = f'{val:.3f} {unidad}' if unidad=='ms' else (f'{val:.2f} {unidad}' if unidad=='metros' else f'{int(val)} {unidad}')
            ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+max(vals)*0.02,
                    lbl, ha='center',va='bottom',fontsize=9,fontweight='bold',color='#1E293B')
        ax.set_title(titulo,fontsize=10,fontweight='bold',color='#1E293B',pad=8)
        ax.set_ylabel(unidad,fontsize=8,color='#475569')
        ax.tick_params(labelsize=8,colors='#475569')
        ax.set_ylim(0,max(vals)*1.22)
        ax.spines['top'].set_visible(False); ax.spines['right'].set_visible(False)
        ax.spines['left'].set_color('#E2E8F0'); ax.spines['bottom'].set_color('#E2E8F0')
        ax.yaxis.grid(True,color='#F1F5F9',linewidth=0.8); ax.set_axisbelow(True)

    # Tabla resumen
    ax_t = fig.add_axes([0.08,0.01,0.84,0.12])
    ax_t.axis('off')
    headers=['','Grilla','Pasos','Exploradas','Tiempo (ms)','Distancia (m)','Obstaculos']
    rows=[
        ['Mapa 1 — Simple',   f'{R1}x{C1}', str(len(path1)), str(len(visited1)), f'{t1_ms:.3f}', f'{dist1:.2f}', str(len(OBS1))],
        ['Mapa 2 — Complejo', f'{R2}x{C2}', str(len(path2)), str(len(visited2)), f'{t2_ms:.3f}', f'{dist2:.2f}', str(len(OBS2))],
        ['Diferencia (+)', '—',
         f'+{len(path2)-len(path1)}',
         f'+{len(visited2)-len(visited1)}',
         f'+{t2_ms-t1_ms:.3f}',
         f'+{dist2-dist1:.2f}',
         f'+{len(OBS2)-len(OBS1)}'],
    ]
    tabla = ax_t.table(cellText=rows,colLabels=headers,loc='center',cellLoc='center')
    tabla.auto_set_font_size(False); tabla.set_fontsize(8.5); tabla.scale(1,1.5)
    for (row,col),cell in tabla.get_celld().items():
        if row==0:   cell.set_facecolor('#1E3A5F'); cell.set_text_props(color='white',fontweight='bold')
        elif row==3: cell.set_facecolor('#EFF6FF'); cell.set_text_props(color='#1E3A5F',fontweight='bold')
        elif row%2==0: cell.set_facecolor('#F8FAFF')
        cell.set_edgecolor('#E2E8F0')

    plt.savefig('comparativo_astar.png',dpi=150,bbox_inches='tight',facecolor='#F8F9FA')
    print("  comparativo_astar.png guardado.")
    plt.show()

# ═══════════════════════════════════════════════════════════════
#  MENU
# ═══════════════════════════════════════════════════════════════

while True:
    print("\n" + "="*55)
    print("  PLANIFICACION DE TRAYECTORIAS — Algoritmo A*")
    print("  Pasillo Universitario")
    print("="*55)
    print()
    print("  [1]  Mapa 1 — Simple          (15x20 |  4 obstaculos)")
    print("  [2]  Mapa 2 — Complejo        (30x45 | 26 obstaculos)")
    print("  [3]  Ambos mapas en secuencia")
    print("  [4]  Analisis comparativo")
    print()
    print("  [0]  Salir")
    print("="*55)
    op = input("  Opcion: ").strip()

    if op=="0":
        print("  Hasta luego."); break
    elif op=="1":
        simular(grid1,R1,C1,OBS1,START1,GOAL1,path1,visited1,
                "Mapa 1 — Piso Simple (dificultad: BASICA)",vel=0.08)
    elif op=="2":
        simular(grid2,R2,C2,OBS2,START2,GOAL2,path2,visited2,
                "Mapa 2 — Edificio Complejo (dificultad: ALTA)",vel=0.05)
    elif op=="3":
        simular(grid1,R1,C1,OBS1,START1,GOAL1,path1,visited1,
                "Mapa 1 — Piso Simple (dificultad: BASICA)",vel=0.08)
        simular(grid2,R2,C2,OBS2,START2,GOAL2,path2,visited2,
                "Mapa 2 — Edificio Complejo (dificultad: ALTA)",vel=0.05)
    elif op=="4":
        print("\n  Generando analisis comparativo...")
        mostrar_comparativo()
    else:
        print("  Opcion invalida.")

    if op in ("1","2","3","4"):
        input("\n  Presiona ENTER para volver al menu...")