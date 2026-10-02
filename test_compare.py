import grid, astar

g = grid.Grid(15, 20)
g.set_start(0, 0)
g.set_goal(14, 19)

for r in range(15):
    if r != 7:
        g.set_wall(r, 10)

r1 = list(astar.astar(g))[-1]

g2 = grid.Grid(15, 20)
g2.set_start(0, 0)
g2.set_goal(14, 19)
for r in range(15):
    if r != 7:
        g2.set_wall(r, 10)
r2 = list(astar.bfs(g2))[-1]

print("A*  found:", r1["found"], "| path:", r1["stats"]["path_length"], "| nodes:", r1["stats"]["nodes_explored"])
print("BFS found:", r2["found"], "| path:", r2["stats"]["path_length"], "| nodes:", r2["stats"]["nodes_explored"])
diff = r2["stats"]["nodes_explored"] - r1["stats"]["nodes_explored"]
print("A* explored", diff, "fewer nodes than BFS")
