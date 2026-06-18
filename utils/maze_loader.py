def load_maze(filename):

    maze = []

    with open(filename, "r") as file:

        for line in file:

            row = []

            for cell in line.strip().split():

                if cell == "1":
                    row.append(1)

                elif cell == "0":
                    row.append(0)

                else:
                    row.append(cell)

            maze.append(row)

    return maze


def find_start_goal(maze):

    start = None
    goal = None

    for r in range(len(maze)):
        for c in range(len(maze[0])):

            if maze[r][c] == "S":
                start = (r, c)

            elif maze[r][c] == "G":
                goal = (r, c)

    return start, goal


def find_checkpoint(maze):
    for r in range(len(maze)):
        for c in range(len(maze[0])):
            if maze[r][c] == "C":
                return (r, c)
    return None