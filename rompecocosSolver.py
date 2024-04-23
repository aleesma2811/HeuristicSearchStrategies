import heapq
import time

# Obtener distancia Manhattan (heurístca)

def heuristica(puzzleState):
    distanciaTotal = 0
    for i in range(3): # Fila
        for j in range(3): # Columna

            # Si la pieza no es hueco, calcular la fila y columna objetivo de cada número
            if puzzleState[i][j] != 0:
                filaObjetivo = (puzzleState[i][j] - 1) // 3
                columnaObjetivo = (puzzleState[i][j] - 1) % 3
                distanciaTotal += abs(i - filaObjetivo) + abs(j - columnaObjetivo)
    return distanciaTotal

# Obtener posición vacía del tablero

def obtenerHueco(puzzleState):
    for i in range(3):
        for j in range(3):
            # Si encuentra 0 (hueco), devolver posición del hueco
            if puzzleState[i][j] == 0:
                return i, j
            
# Obtener sucesores y verificar movimientos del hueco

def obtenerSucesores(puzzleState):
    sucesores = []
    filaV, columnaV = obtenerHueco(puzzleState) #Fila vacía, columna vacía
    movimientos = [(0, 1), (0, -1), (1, 0), (-1, 0)] # Derecha, izquierda, abajo, arriba
    
    for df, dc in movimientos:
        newFila, newColumna = filaV + df, columnaV + dc
        # Verificar que los movimientos no salgan del tablero y sean horizontales o verticales
        if 0 <= newFila < 3 and 0 <= newColumna < 3:
            newState = [fila[:] for fila in puzzleState] # Copia del tablero actual
            newState[filaV][columnaV], newState[newFila][newColumna] = newState[newFila][newColumna], newState[filaV][columnaV]
            sucesores.append((newState, (newFila, newColumna)))
    
    return sucesores

# Resolver Puzzle

## Algoritmo A*
def astarSolver(initialState):
    currentState = initialState
    priorityQueue = [(heuristica(initialState), initialState)]
    visited = set()
    visitedStates = []
    solution = None

    while priorityQueue:
        # Extraer estado del tablero con menor heurística
        h, currentState = heapq.heappop(priorityQueue)
        # Agregar estados a visited y visitedStates
        visited.add(tuple(map(tuple, currentState)))
        visitedStates.append([fila[:] for fila in currentState])

        # Si se encuentra la solución, actualizar solución
        if currentState == [[1, 2, 3], [4, 5, 6], [7, 8, 0]]:
            solution = currentState
            break
        successors = obtenerSucesores(currentState)
        for child, _ in successors:
            # Si no se ha explorado el estado actual, agregar a la cola de prioridad
            if tuple(map(tuple, child)) not in visited:
                heapq.heappush(priorityQueue, (obtenerSucesores(child), child))

    # Devolver todos los movimientos realizados y la solución
    return visitedStates, solution

## Algoritmo Beam 
def beamSearch(initialState, beamWidth):
    currentState = [(heuristica(initialState), initialState)]
    visited = set()
    visitedStates = []
    solution = None

    while currentState:
        allSuccessors = []
        for h, state in currentState:
            visited.add(tuple(map(tuple, state)))
            visitedStates.append([fila[:] for fila in state])
            successors = [(heuristica(child), child) for child, _ in obtenerSucesores(state) if tuple(map(tuple, child)) not in visited]
            allSuccessors.extend(successors)

        allSuccessors.sort(key=lambda x: x[0])
        currentState = allSuccessors[:beamWidth]

        for _, state in currentState:
            if state == [[1, 2, 3], [4, 5, 6], [7, 8, 0]]:
                solution = state
                break

        if solution:
            break

    return visitedStates, solution

# Imprimir cada movimiento

def mostrarMovimientos(visitedStates):
    for movimiento, estado in enumerate(visitedStates, start=1):
        print(f"--- Movimiento {movimiento} ---")
        for fila in estado:
            print(" ".join(map(str, fila)))
        print()

tableroInicial = [
    [3, 1, 2],
    [4, 0, 5],
    [6, 7, 8]
]

print("---- A* Algorithm ---- ")
start_time = time.perf_counter_ns()      # Agregamos 'start_time' para medir el tiempo de ejecución por cada algoritmo, se repitira esta línea por cada algoritmo
movimientos, solucion = astarSolver(tableroInicial)
astarSolver_time = (time.perf_counter_ns() - start_time)*1e-9     # Current time - start time = tiempo de ejecución
mostrarMovimientos(movimientos)
print(f"Solución en {len(movimientos)} movimientos:         Tiempo de ejecución: {astarSolver_time} s")

for fila in solucion:
    print(" ".join(map(str, fila)))

print("\n\n ---- Beam Search Algorithm: ---- ")
start_time = time.perf_counter_ns()
movimientos, solucion = beamSearch(tableroInicial, 10)
beamSearch_time = (time.perf_counter_ns() - start_time)*1e-9
mostrarMovimientos(movimientos)
print(f"Solución en {len(movimientos)} movimientos:         Tiempo de ejecución: {beamSearch_time} s")

for fila in solucion:
    print(" ".join(map(str, fila)))

[[3, 1, 2],[4, 0, 5],[6, 7, 8]]