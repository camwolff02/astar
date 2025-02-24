import sys
import math
import numpy as np
import numpy.typing as npt
import pygame
from queue import PriorityQueue
from typing import Callable
from functools import partial


# Useful class for keeping track of the nodes being added
class Node:
    def __init__(
        self,
        position: tuple[int, int],
        parent: "Node | None" = None,
        h: Callable[["Node"], int] = lambda n: 0,
    ):
        self.position = position  # Tuple (x, y)
        self.parent = parent

        # Cost from start to current node. Assume every cost is +1
        self.g = parent.g + 1 if parent else sys.maxsize
        self.h = h(self)  # Heuristic cost from current node to goal
        self.f = self.g + self.h  # Total cost (g + h). Key to use for the queue.

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Node):
            return False
        return self.position == other.position

    def __lt__(self, other: "Node") -> bool:
        return self.f < other.f


# A* class for implementation
class AStar:
    # DO NOT modify this code
    def __init__(self):
        self.robot_start = (1, 28)
        self.goal = (20, 10)
        self.map_size = (30, 30)

        self.map_resolution = 0.05  # meters

    # Dont NOT modify this code
    def build_map(self) -> npt.NDArray[np.int_]:
        # Initialize a 30x30 map with all cells set to 0 (free space)
        # NOTE: the resolution of the map of one cell = 0.05m
        # NOTE: the index (0,0) is in the top left corner went visualized using PyGame
        occupancy_grid = np.zeros(self.map_size, dtype=int)

        # Optionally, add some obstacles (set specific cells to 100)
        # Add a vertical wall in the middle of the map
        occupancy_grid[7:15, 12] = 100

        # Add a horizontal wall
        occupancy_grid[12, 7:15] = 100

        # Add a small obstacle
        occupancy_grid[10:20, 7:8] = 100
        occupancy_grid[7:9, 20:28] = 100
        occupancy_grid[15:17, 20:28] = 100

        return occupancy_grid

    # Do NOT modify this code
    def visualize_map(
        self,
        occupancy_grid: npt.NDArray[np.int_],
        path: npt.NDArray[np.int_] | None = None,
    ) -> None:

        # Initialize Pygame
        pygame.init()

        # Define map dimensions and cell size
        map_size = (occupancy_grid.shape[0], occupancy_grid.shape[1])
        cell_size = 8  # Size of each cell in pixels
        width, height = map_size[0] * cell_size, map_size[1] * cell_size

        # Create a Pygame window
        screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("2D Map Visualization")

        # Define colors
        WHITE = (255, 255, 255)
        BLACK = (0, 0, 0)
        GRAY = (200, 200, 200)
        GREEN = (0, 255, 0)
        BLUE = (0, 0, 255)
        RED = (255, 0, 0)

        # Main loop
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Clear the screen
            screen.fill(GRAY)

            # Draw the grid
            for y in range(map_size[1]):
                for x in range(map_size[0]):
                    if occupancy_grid[y, x] == 100:
                        color = BLACK  # Obstacle
                    else:
                        color = WHITE  # Free space

                    # Draw the cell
                    pygame.draw.rect(
                        screen,
                        color,
                        (x * cell_size, y * cell_size, cell_size, cell_size),
                    )

            # Draw START location
            pygame.draw.rect(
                screen,
                GREEN,
                (
                    self.robot_start[0] * cell_size,
                    self.robot_start[1] * cell_size,
                    cell_size,
                    cell_size,
                ),
            )

            # Draw GOAL location
            pygame.draw.rect(
                screen,
                BLUE,
                (
                    self.goal[0] * cell_size,
                    self.goal[1] * cell_size,
                    cell_size,
                    cell_size,
                ),
            )

            # Draw grid lines
            for x in range(0, width, cell_size):
                pygame.draw.line(screen, BLACK, (x, 0), (x, height))
            for y in range(0, height, cell_size):
                pygame.draw.line(screen, BLACK, (0, y), (width, y))

            if not path is None:
                for i in range(len(path)):
                    pygame.draw.rect(
                        screen,
                        RED,
                        (
                            path[i][0] * cell_size,
                            path[i][1] * cell_size,
                            cell_size,
                            cell_size,
                        ),
                    )

            # Update the display
            pygame.display.flip()

        # Quit Pygame
        pygame.quit()

    def heuristic(self, a: Node, b: Node) -> int:
        """L2 Distance between A and B"""
        return int(
            math.sqrt(
                (b.position[0] - a.position[0]) ** 2
                + (b.position[1] - a.position[1]) ** 2
            )
        )

    def astar(self, occupancy_grid: npt.NDArray[np.int_]) -> npt.NDArray[np.int_] | None:
        # Implement your A* search here
        xg = Node(self.goal)
        h = partial(self.heuristic, b=xg)
        xs = Node(self.robot_start, h=h)

        open: PriorityQueue[Node] = PriorityQueue()
        open_set: dict[tuple[int, int], Node] = {}
        closed: set[tuple[int, int]] = set()
        open.put(xs)
        open_set[xs.position] = xs

        while not open.empty():
            x = open.get()
            del open_set[x.position]
            closed.add(x.position)

            if x.position == self.goal:
                return self.construct_path(x)

            for dir in ((0, 1), (1, 0), (0, -1), (-1, 0)):
                pos = self.add_positions(x.position, dir)
                if (
                    not pos in closed
                    and pos[0] >= 0
                    and pos[1] >= 0
                    and pos[0] < self.map_size[0]
                    and pos[1] < self.map_size[1]
                    and occupancy_grid[pos[0], pos[1]] == 0
                ):
                    self.add_to_open(open, open_set, Node(pos, x, h=h))

        return None  # no path found

    @staticmethod
    def add_to_open(
        open: PriorityQueue[Node],
        open_set: dict[tuple[int, int], Node],
        neighbor: Node,
    ) -> None:
        # Check if a neighbor should be added to the open list.
        if not neighbor.position in open_set:
            open.put(neighbor)
            open_set[neighbor.position] = neighbor
        elif open_set[neighbor.position].g > neighbor.g:
            open.queue.remove(open_set[neighbor.position])
            open.put(neighbor)
            open_set[neighbor.position] = neighbor

    @staticmethod
    def construct_path(node: Node) -> npt.NDArray[np.int_]:
        """backtrace parents of node to construct path to passed node"""
        path: list[tuple[int, int]] = [node.position]
        while node.parent:
            node = node.parent
            path.append(node.position)
        path.reverse()
        return np.array(path)

    @staticmethod
    def add_positions(p1: tuple[int, int], p2: tuple[int, int]) -> tuple[int, int]:
        return (p1[0] + p2[0], p1[1] + p2[1])


# Example usage
if __name__ == "__main__":
    # Example occupancy grid (0 = free, 100 = occupied)
    astar_instance = AStar()
    occupancy_grid = astar_instance.build_map()

    # Print map before search
    # Close the window before you can move onto the next part of the code
    astar_instance.visualize_map(occupancy_grid=occupancy_grid)

    path = astar_instance.astar(occupancy_grid)
    if not path is None:
        print("Path found:", path)
    else:
        print("No path found.")

    # Print the solution found for your robot
    astar_instance.visualize_map(occupancy_grid=occupancy_grid, path=path)
