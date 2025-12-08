# 安装依赖
#   pip install pygame
#
# 运行程序
#   python app.py
#
# TODO:
#   1. 完成 reconstruct() 函数，实现从 came 中恢复路径
#   2. 完成 octile() 函数，实现启发函数
#   3. 完成 create_astar_stepper() 函数，实现 A* 搜索
#   4. 完成 create_dijkstra_stepper() 函数，实现 Dijkstra 搜索
#   5. 完成 identify_successors() 函数，实现跳跃点识别
#   6. 完成 create_jps_stepper() 函数，实现 JPS 搜索

import pygame
import sys
import time
import math
import heapq

pygame.init()

COLS = 48
ROWS = 27
CELL = 24
W = COLS * CELL
H = ROWS * CELL
step_interval_ms = 5
screen = pygame.display.set_mode((W, H), pygame.DOUBLEBUF)
pygame.display.set_caption("栅格地图搜索对比")


grid = [[0 for _ in range(COLS)] for _ in range(ROWS)]
start = (2, 2)
goal = (COLS - 3, ROWS - 3)
mode = "wall"
painting = False
paint_value = 1
steppers = None
step_vis = None
step_done = None
last_states = None
paused = False


def draw_rect(x, y, color):
    pygame.draw.rect(
        screen, color, pygame.Rect(x * CELL + 1, y * CELL + 1, CELL - 2, CELL - 2)
    )


def draw():
    grid[start[1]][start[0]] = 0
    grid[goal[1]][goal[0]] = 0
    screen.fill((255, 255, 255))
    for y in range(ROWS):
        for x in range(COLS):
            c = (51, 51, 51) if grid[y][x] else (255, 255, 255)
            pygame.draw.rect(screen, c, pygame.Rect(x * CELL, y * CELL, CELL, CELL))
            pygame.draw.rect(
                screen, (238, 238, 238), pygame.Rect(x * CELL, y * CELL, CELL, CELL), 1
            )
    if step_vis:
        # dijkstra visited
        for p in step_vis["dijkstra"]["closed"]:
            draw_rect(p[0], p[1], (230, 244, 255))
        for p in step_vis["dijkstra"]["open"]:
            draw_rect(p[0], p[1], (240, 247, 255))
        # astar visited
        for p in step_vis["astar"]["closed"]:
            draw_rect(p[0], p[1], (255, 236, 236))
        for p in step_vis["astar"]["open"]:
            draw_rect(p[0], p[1], (255, 245, 245))
        # jps visited
        for p in step_vis["jps"]["closed"]:
            draw_rect(p[0], p[1], (234, 255, 234))
        for p in step_vis["jps"]["open"]:
            draw_rect(p[0], p[1], (246, 255, 237))
        # path
        draw_path(step_vis["dijkstra"]["path"], (45, 127, 255))
        draw_path(step_vis["astar"]["path"], (255, 77, 79))
        draw_path(step_vis["jps"]["path"], (82, 196, 26))
    # 最后画起点和终点（最上层）
    draw_rect(start[0], start[1], (250, 219, 20))
    draw_rect(goal[0], goal[1], (114, 46, 209))
    pygame.draw.line(screen, (238, 238, 238), (COLS * CELL, 0), (COLS * CELL, H), 2)


def draw_path(path, color):
    if not path or len(path) == 0:
        return
    pts = [(x * CELL + CELL / 2, y * CELL + CELL / 2) for x, y in path]
    pygame.draw.lines(screen, color, False, pts, 3)


def mouse_cell(pos):
    x = pos[0] // CELL
    y = pos[1] // CELL
    return x, y


def main():
    global painting, paint_value, mode, start, goal, steppers, step_vis, step_done, last_states, step_interval_ms, paused
    clock = pygame.time.Clock()
    last_step_ms = 0
    draw()
    print(
        "W: 编辑障碍; S: 设置起点; G: 设置终点; C: 清除; N: 开始; R: 重置; Space: 暂停; Esc: 退出"
    )
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    sys.exit()
                if event.key == pygame.K_w:
                    mode = "wall"
                if event.key == pygame.K_s:
                    mode = "start"
                if event.key == pygame.K_g:
                    mode = "goal"
                if event.key == pygame.K_c:
                    for y in range(ROWS):
                        for x in range(COLS):
                            grid[y][x] = 0
                    steppers = None
                    step_done = None
                    last_states = None
                    step_vis = None
                    paused = False
                if event.key == pygame.K_n:
                    steppers = {
                        "dijkstra": create_dijkstra_stepper(),
                        "astar": create_astar_stepper(),
                        "jps": create_jps_stepper(),
                    }
                    step_done = {"dijkstra": False, "astar": False, "jps": False}
                    last_states = {
                        "dijkstra": {"open": [], "closed": [], "path": []},
                        "astar": {"open": [], "closed": [], "path": []},
                        "jps": {"open": [], "closed": [], "path": []},
                    }
                    step_vis = None
                    paused = False
                if event.key == pygame.K_r:
                    if steppers is None:
                        steppers = {
                            "dijkstra": create_dijkstra_stepper(),
                            "astar": create_astar_stepper(),
                            "jps": create_jps_stepper(),
                        }
                        step_done = {"dijkstra": False, "astar": False, "jps": False}
                        last_states = {
                            "dijkstra": {"open": [], "closed": [], "path": []},
                            "astar": {"open": [], "closed": [], "path": []},
                            "jps": {"open": [], "closed": [], "path": []},
                        }
                        step_vis = None
                    paused = False
                if event.key == pygame.K_m:
                    mode = "wall" if mode != "wall" else "start"
                if event.key == pygame.K_SPACE:
                    paused = not paused
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = pygame.mouse.get_pos()
                if mx < COLS * CELL:
                    cx, cy = mouse_cell((mx, my))
                    if cx < 0 or cy < 0 or cx >= COLS or cy >= ROWS:
                        pass
                    else:
                        if mode == "wall":
                            painting = True
                            paint_value = 0 if grid[cy][cx] else 1
                            if (cx, cy) != start and (cx, cy) != goal:
                                grid[cy][cx] = paint_value
                        elif mode == "start":
                            if grid[cy][cx] == 0:
                                start = (cx, cy)
                        elif mode == "goal":
                            if grid[cy][cx] == 0:
                                goal = (cx, cy)
            if event.type == pygame.MOUSEMOTION and painting:
                mx, my = pygame.mouse.get_pos()
                if mx < COLS * CELL:
                    cx, cy = mouse_cell((mx, my))
                    if 0 <= cx < COLS and 0 <= cy < ROWS:
                        if (cx, cy) != start and (cx, cy) != goal:
                            grid[cy][cx] = paint_value
            if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                painting = False
        now_ms = pygame.time.get_ticks()
        if steppers and not paused and now_ms - last_step_ms >= step_interval_ms:
            last_step_ms = now_ms
            # dijkstra
            if not step_done["dijkstra"] and steppers["dijkstra"]:
                rd = steppers["dijkstra"]()
                last_states["dijkstra"] = rd
                if rd["done"]:
                    step_done["dijkstra"] = True
                    steppers["dijkstra"] = None
            # astar
            if not step_done["astar"] and steppers["astar"]:
                ra = steppers["astar"]()
                last_states["astar"] = ra
                if ra["done"]:
                    step_done["astar"] = True
                    steppers["astar"] = None
            # jps
            if not step_done["jps"] and steppers["jps"]:
                rj = steppers["jps"]()
                last_states["jps"] = rj
                if rj["done"]:
                    step_done["jps"] = True
                    steppers["jps"] = None
            step_vis = {
                "dijkstra": {
                    "open": last_states["dijkstra"]["open"],
                    "closed": last_states["dijkstra"]["closed"],
                    "path": last_states["dijkstra"]["path"],
                },
                "astar": {
                    "open": last_states["astar"]["open"],
                    "closed": last_states["astar"]["closed"],
                    "path": last_states["astar"]["path"],
                },
                "jps": {
                    "open": last_states["jps"]["open"],
                    "closed": last_states["jps"]["closed"],
                    "path": last_states["jps"]["path"],
                },
            }
            if step_done["dijkstra"] and step_done["astar"] and step_done["jps"]:
                steppers = None
        draw()
        pygame.display.flip()
        clock.tick(60)


def in_bounds(x, y):
    return 0 <= x < COLS and 0 <= y < ROWS


def passable(x, y):
    if not in_bounds(x, y):
        return False
    if grid[y][x] == 0:
        return True
    if (x, y) == start or (x, y) == goal:
        return True
    return False


dirs8 = [
    (1, 0, 1),
    (-1, 0, 1),
    (0, 1, 1),
    (0, -1, 1),
    (1, 1, math.sqrt(2)),
    (1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)),
    (-1, -1, math.sqrt(2)),
]


def neighbors8(x, y):
    n = []
    for dx, dy, c in dirs8:
        nx = x + dx
        ny = y + dy
        if not passable(nx, ny):
            continue
        if dx != 0 and dy != 0:
            if not passable(x, ny) or not passable(nx, y):
                continue
        n.append((nx, ny, c))
    return n


def reconstruct(came, to):
    path = []
    cur = f"{to[0]},{to[1]}"
    ##########################################
    # TODO: 从终点开始，根据 came 字典， 恢复出从起点到终点的路径
    ##########################################
    return path


def octile(a, b) -> float:
    ##########################################
    # TODO: 计算当前节点到终点的估计值
    ##########################################
    return 0.0


def create_astar_stepper():
    open = []
    gScore = {f"{start[0]},{start[1]}": 0}
    came = {}
    closed = set()
    heapq.heappush(open, (octile(start, goal), 0, start[0], start[1]))

    def step():
        nonlocal open, gScore, came, closed
        if not open:
            return {"done": True, "open": [], "closed": [], "path": []}
        f, cg, x, y = heapq.heappop(open)
        k = f"{x},{y}"
        if k in closed:
            return {
                "done": False,
                "open": [tuple(map(int, i.split(","))) for i in gScore.keys()],
                "closed": [tuple(map(int, i.split(","))) for i in closed],
                "path": [],
            }
        closed.add(k)
        if (x, y) == goal:
            return {
                "done": True,
                "open": [tuple(map(int, i.split(","))) for i in gScore.keys()],
                "closed": [tuple(map(int, i.split(","))) for i in closed],
                "path": reconstruct(came, goal),
            }
        for nx, ny, c in neighbors8(x, y):
            nk = f"{nx},{ny}"
            ng = gScore[k] + c
            #################################
            # TODO: 更新 fScore, gScore, came, open
            #################################
        return {
            "done": False,
            "open": [tuple(map(int, i.split(","))) for i in gScore.keys()],
            "closed": [tuple(map(int, i.split(","))) for i in closed],
            "path": [],
        }

    return step


def create_dijkstra_stepper():
    open = []
    gScore = {f"{start[0]},{start[1]}": 0}
    came = {}
    closed = set()
    heapq.heappush(open, (0, start[0], start[1]))

    def step():
        nonlocal open, gScore, came, closed
        if not open:
            return {
                "done": True,
                "open": [],
                "closed": [tuple(map(int, i.split(","))) for i in closed],
                "path": [],
            }
        cg, x, y = heapq.heappop(open)
        k = f"{x},{y}"
        if k in closed:
            return {
                "done": False,
                "open": [tuple(map(int, i.split(","))) for i in gScore.keys()],
                "closed": [tuple(map(int, i.split(","))) for i in closed],
                "path": [],
            }
        closed.add(k)
        if (x, y) == goal:
            return {
                "done": True,
                "open": [tuple(map(int, i.split(","))) for i in gScore.keys()],
                "closed": [tuple(map(int, i.split(","))) for i in closed],
                "path": reconstruct(came, goal),
            }
        for nx, ny, c in neighbors8(x, y):
            nk = f"{nx},{ny}"
            ng = gScore[k] + c
            #################################
            # TODO: 更新 gScore, came, open
            #################################
        return {
            "done": False,
            "open": [tuple(map(int, i.split(","))) for i in gScore.keys()],
            "closed": [tuple(map(int, i.split(","))) for i in closed],
            "path": [],
        }

    return step


def identify_successors(p, parent, goal):
    succ = []
    #########################################
    # TODO: 找到所有从 p 到 goal 的合法跳跃点
    #########################################
    return succ


def create_jps_stepper():
    open = []
    gScore = {f"{start[0]},{start[1]}": 0}
    came = {}
    closed = set()
    heapq.heappush(open, (octile(start, goal), 0, start[0], start[1], None))

    def step():
        nonlocal open, gScore, came, closed
        if not open:
            return {
                "done": True,
                "open": [],
                "closed": [tuple(map(int, i.split(","))) for i in closed],
                "path": [],
            }
        f, cg, x, y, parent = heapq.heappop(open)
        ck = f"{x},{y}"
        if ck in closed:
            return {
                "done": False,
                "open": [tuple(map(int, i.split(","))) for i in gScore.keys()],
                "closed": [tuple(map(int, i.split(","))) for i in closed],
                "path": [],
            }
        closed.add(ck)
        if (x, y) == goal:
            return {
                "done": True,
                "open": [tuple(map(int, i.split(","))) for i in gScore.keys()],
                "closed": [tuple(map(int, i.split(","))) for i in closed],
                "path": reconstruct(came, goal),
            }
        for sx, sy, addg in identify_successors((x, y), parent, goal):
            nk = f"{sx},{sy}"
            ng = gScore[ck] + addg
            #################################
            # TODO: 更新 fScore, gScore, came, open
            #################################
        return {
            "done": False,
            "open": [tuple(map(int, i.split(","))) for i in gScore.keys()],
            "closed": [tuple(map(int, i.split(","))) for i in closed],
            "path": [],
        }

    return step


if __name__ == "__main__":
    main()
