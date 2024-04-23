import heapq
import time

grid = [
    ['A', 'B', 'C', 'D', 'E'],
    ['F', 'G', 'H', 'I', 'J'],
    ['K', 'L', 'M', 'N', 'Ñ'],
    ['O', 'P', 'Q', 'R', 'S'],
    ['T', 'U', 'V', 'W', 'X']
]

# Convertimos los puntos de inicio y fin a coordenadas
# Definimos la posición de cada coordenada
locations = {
    'A': (0, 0), 'B': (0, 1), 'C': (0, 2), 'D': (0, 3), 'E': (0, 4),
    'F': (1, 0), 'G': (1, 1), 'H': (1, 2), 'I': (1, 3), 'J': (1, 4),
    'K': (2, 0), 'L': (2, 1), 'M': (2, 2), 'N': (2, 3), 'Ñ': (2, 4),
    'O': (3, 0), 'P': (3, 1), 'Q': (3, 2), 'R': (3, 3), 'S': (3, 4),
    'T': (4, 0), 'U': (4, 1), 'V': (4, 2), 'W': (4, 3), 'X': (4, 4)
}

# Función para calcular la distancia de Manhattan entre dos puntos
def manhattan_distance(start, goal):
    dx = abs(start[0] - goal[0])  # Absoluto
    dy = abs(start[1] - goal[1])
    return dx + dy

# Función para obtener los vecinos de un nodo en el grid
def get_neighbors(node, grid, obstacle=None):
    neighbors = []
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Arriba, Abajo, Izquierda, Derecha
    for direction in directions:
        neighbor = (node[0] + direction[0], node[1] + direction[1])
        # Si los nodos se encuentran en las esquinas, no agregues vecinos fuera del grid
        if 0 <= neighbor[0] < len(grid) and 0 <= neighbor[1] < len(grid[0]):
            # Si el vecino es el obstáculo, saltarlo
            if obstacle and neighbor == locations[obstacle]:
                continue
            neighbors.append(neighbor)
    return neighbors

# ALGORITMOS

## A* Algorithm
def astar(grid, start, goal, obstacle=None):
    open_set = {start: (0, manhattan_distance(start, goal))} # Costo heurístico
    came_from = {}
    g_score = {start: 0} # Costo real
    moves = 0
    steps = [] ### Pasos para llegar al objetivo

    while open_set:
        # Escoger el nodo con el menor precio
        current = min(open_set, key=lambda x: open_set[x][1])
        steps.append(current) ### Agregar nodo actual a los pasos

        if current == goal:
            # Reconstruir camino
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            # Devoler camino ordenado, costo y número de movimientos
            return path[::-1], g_score[goal], moves, steps

        del open_set[current]

        for neighbor in get_neighbors(current, grid, obstacle):
            # Costo al moverse una cuadrícula
            tentative_g_score = g_score[current] + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + manhattan_distance(neighbor, goal)
                open_set[neighbor] = (tentative_g_score, f_score)
                moves += 1
    return "No se encontró un camino", 0, 0, steps

## Beam Search Algorithm
def beamSearch(grid, start, goal, k, obstacle=None):
    open_set = {start: (0, manhattan_distance(start, goal))} # Heuristic cost
    came_from = {}
    g_score = {start: 0} # Actual cost
    moves = 0
    steps = [] ### Steps to reach the goal

    while open_set:
        # Choose the node with the lowest cost
        current = min(open_set, key=lambda x: open_set[x][1])
        steps.append(current) ### Add current node to steps

        if current == goal:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            # Return ordered path, cost, and number of moves
            return path[::-1], g_score[goal], moves, steps

        del open_set[current]

        for neighbor in get_neighbors(current, grid, obstacle):
            # Cost to move one grid
            tentative_g_score = g_score[current] + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + manhattan_distance(neighbor, goal)
                open_set[neighbor] = (tentative_g_score, f_score)
                moves += 1
        # Beam Search
        if len(open_set) > k:
            open_set = dict(heapq.nsmallest(k, open_set.items(), key=lambda x: x[1][1]))
    return "No path found", 0, 0, steps

# Función para imprimir el camino encontrado
def print_path(path):
    for node in path:
        for key, value in locations.items():
            if value == node:
                print(key, end=' -> ')
                break

#función para regresar el path como un string
def pathToString(path):
    pathString = ""
    for node in path:
        for key, value in locations.items():
            if value == node:
                pathString += key + " -> "
                break
    return pathString

## ------------------------------ A* Algorithm Solving Example ------------------------------ ##
# Seleccionar nodos y obstáculo
start_letter = 'O'
goal_letter = 'I'
obstacle_letter = 'H'

start = locations[start_letter]
goal = locations[goal_letter]

# Verificar si se ingresó un obstáculo
if obstacle_letter == '':
    obstacle = None
else:
    obstacle = obstacle_letter

# Calcular tiempo
start_time = time.perf_counter_ns()
path, cost, moves, steps = astar(grid, start, goal, obstacle)
astarSolverTime = (time.perf_counter_ns() - start_time) * 1e-9

print("Camino encontrado: ")
print(pathToString(path))
print(f"Costo total: {cost}")
print(f"Número de movimientos: {moves}")
print(f"Tiempo de ejecución: {astarSolverTime}s")

## ------------------------------ Beam Search Algorithm Solving Example ------------------------------ ##
# Seleccionar nodos y obstáculo
start_letter = 'O'
goal_letter = 'I'
obstacle_letter = 'H'

start = locations[start_letter]
goal = locations[goal_letter]

# Verificar si se ingresó un obstáculo
if obstacle_letter == '':
    obstacle = None
else:
    obstacle = obstacle_letter

# Calcular tiempo
start_time = time.perf_counter_ns()
path, cost, moves, steps = beamSearch(grid, start, goal, 13, obstacle)
astarSolverTime = (time.perf_counter_ns() - start_time) * 1e-9

print("Camino encontrado: ")
print_path(path)
print(f"Costo total: {cost}")
print(f"Número de movimientos: {moves}")
print(f"Tiempo de ejecución: {astarSolverTime}s")

# Poner todo esto en una función para poder llamarla desde otro script
