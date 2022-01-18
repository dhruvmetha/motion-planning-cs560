from matplotlib import pyplot as plt, colors, patches
from matplotlib.path import Path
import matplotlib.animation as animation
import numpy as np

# from rrt import make_configuration_space, rrt, rrt_star
FFwriter = animation.FFMpegWriter

def visualize_problem(robot, obstacles, start, goal, old_obstacles = None, show=True, fig=None, ax=None):

    if fig is None:
        fig = plt.figure()
        ax = fig.gca()

    polygons = []

    # rob = np.array(robot.transform())
    
    if goal is not None:
        robot.set_pose(tuple(goal))
        rob_goal = robot.transform()
        # print(rob_goal)
        poly = patches.Polygon(rob_goal, True, facecolor='g')
        ax.add_patch(poly)

    if start is not None:
        robot.set_pose(tuple(start))
        rob_start = robot.transform()
        poly = patches.Polygon(rob_start, True, facecolor='r')
        ax.add_patch(poly)
    

    if old_obstacles:
        for obs in old_obstacles:
            ob = np.array(obs)
            poly = patches.Polygon(ob, True, facecolor='slategrey', zorder=1)
            ax.add_patch(poly)

    for obs in obstacles:
        ob = np.array(obs)
        poly = patches.Polygon(ob, True, facecolor='pink', zorder=0, edgecolor='g')
        ax.add_patch(poly)
        # polygons.append(ob[:, 0])
        # polygons.append(ob[:, 1])
        # polygons.append('slategrey')

    # plt.fill(*polygons, zorder=0)

    ax.set_xticks(np.arange(0, 11, 1))
    ax.set_yticks(np.arange(0, 11, 1))
    ax.set_xlim([0, 10])
    ax.set_ylim([0, 10])

    if show:
        # plt.grid()
        
        plt.show()

def visualize_points(points, robot, obstacles, start, goal, old_obstacles=None, show=True, free=True, fig=None, ax=None):
    if type(points) == dict:
        points_arr = np.array(points['free'])
        points_collide_arr = np.array(points['collide'])
    else:
        points_arr = points
        points_collide_arr = []

    visualize_problem(robot, obstacles, start, goal, old_obstacles, show=False, fig=fig, ax=ax)
    if len(points_arr) > 0:
        plt.scatter(points_arr[:, 0], points_arr[:, 1], color='black', zorder=2)
    if len(points_collide_arr) > 0:
        plt.scatter(points_collide_arr[:, 0], points_collide_arr[:, 1], color='red', zorder=1)
    
    if show:
        # polygons = []
        # for p in [*points_arr, *points_collide_arr]:
        #     rob = np.array(robot)
        #     rob_start = rob + np.array(p)
        #     polygons.append(rob_start[:, 0])
        #     polygons.append(rob_start[:, 1])
        #     polygons.append('b')
        
        # plt.fill(*polygons, zorder=2)
        
        plt.show()

def visualize_path(robot, obstacles, path, old_obstacles=None, tree=None, show=True, fig=None, ax=None):
    if tree:
        visualize_tree(tree, robot, obstacles, path[0], path[-1], old_obstacles, show=False, fig=fig, ax=ax)
    else:
        visualize_points(np.array(path), robot, obstacles, path[0], path[-1], old_obstacles, show=False, fig=fig, ax=ax)

    for i in range(len(path) - 1):
        if path[i] is not None and path[i+1] is not None:
            x1, y1, a1 = path[i]
            x2, y2, a2 = path[i+1]

            robot.set_pose((x2, y2, a2))
            rob_start = np.array(robot.transform())

            polygons = []
            polygons.append(rob_start[:, 0])
            polygons.append(rob_start[:, 1])
            polygons.append('b')

            plt.fill(*polygons, zorder=3, alpha = 0.7)

            plt.plot([x1, x2], [y1, y2], color='red')

    if show:
        
        plt.show()

def visualize_tree(tree, robot, obstacles, start, goal, old_obstacles=None, show=True, fig=None, ax=None):
    visualize_problem(robot, obstacles, start, goal, old_obstacles, show=False, fig=fig, ax=ax)
    points = np.array(list(tree.tree_nodes.keys()))
    if len(points) > 0:
        plt.scatter(points[:, 0], points[:, 1], color='black', zorder=1)

    closed_list = []
    open_list = [start]

    # print(open_list)
    while len(open_list) != 0:
        node = open_list.pop(0)
        # print('here', node)
        if node:
            x1, y1, a1 = tuple(node)
            # print('heree', tree.exists((x1, y1, a1)))
            if tree.exists((x1, y1, a1)):
                # print('here')
                closed_list.append(node)
                for child in tree.tree_nodes[tuple(node)].children:
                    if child in closed_list:
                        continue
                    x2, y2, a2 = child
                    # print('here')
                    plt.plot([x1, x2], [y1, y2], color='black')
                    open_list.append(child)
        
    if show:
        
        plt.show()

# def visualize_configuration(robot, obstacles, start, goal):
#     new_obstacles = make_configuration_space(robot, obstacles)
#     visualize_problem(robot, new_obstacles, start, goal, obstacles)
#     # tree, path = rrt(robot, new_obstacles, start, goal, 500, return_tree=True)

# def visualize_rrt(robot, obstacles, start, goal, iter_n):
#     tree, path = rrt(robot, new_obstacles, start, goal, iter_n, return_tree=True)
#     visualize_path(robot, new_obstacles, path, obstacles, tree)

# def visualize_rrt_star(robot, obstacles, start, goal , iter_n):
#     new_obstacles = make_configuration_space(robot, obstacles)
#     tree, path = rrt_star(robot, new_obstacles, start, goal, iter_n, return_tree=True)
#     visualize_path(robot, new_obstacles, path, obstacles, tree)

def visualize_animations(robot, obstacles, start, goal, path, tree, interval=10):
    fig, ax = plt.subplots(nrows=1, ncols=1)
    
    visualize_problem(robot, obstacles, start, goal, show=False, fig=fig, ax=ax)
    # visualize_tree(tree, robot, obstacles, start, goal, None, show=False, fig=fig, ax=ax)

    patches_stored = {}
    last_patch = []

    def animate_tree(frame1):
        print(frame1)
        make, frame = frame1
        if make == 0:
            _, wire, point1, point2 = frame
            if wire:
                verts = [list(point1[:2]), list(point1[:2]), list(point2[:2])]
                codes = [Path.MOVETO, Path.LINETO, Path.LINETO]
                path = Path(verts, codes)
                patch = patches.PathPatch(path, edgecolor='black', facecolor='none', lw=1, zorder=10)
                ax.add_patch(patch)
                patches_stored[(point1, point2)] = patch
            
            else:
                patch = patches_stored[(point1, point2)]
                patch.remove()

        if make == 1:
            if len(last_patch) != 0:
                last_patch[0].remove()
                last_patch.clear()

            x1, y1, a1 = frame

            robot.set_pose((x1, y1, a1))
            rob_start = np.array(robot.transform())
            
            ob = np.array(rob_start)
            poly = patches.Polygon(ob, True, facecolor='blue', zorder=10, alpha=0.7)
            ax.add_patch(poly)
            last_patch.append(poly)

        # print(frame)
        # t = frame[0]
        # if (type(t)==int) and (t == -1):
        #     switch.append(-1)
        #     return
        
        # if len(switch) == 0:
        #     _, wire, point1, point2 = frame
        #     if wire:
        #         verts = [point1, point1, point2]
        #         codes = [Path.MOVETO, Path.LINETO, Path.LINETO]
        #         path = Path(verts, codes)
        #         patch = patches.PathPatch(path, edgecolor='black', facecolor='none', lw=1, zorder=10)
        #         ax.add_patch(patch)
        #         patches_stored[(point1, point2)] = patch
            
        #     else:
        #         patch = patches_stored[(point1, point2)]
        #         patch.remove()
        # else:
        #     point1, point2 = list(frame[0]), list(frame[1])
        #     print(frame)
        #     verts = [point1, point1, point2]
        #     codes = [Path.MOVETO, Path.LINETO, Path.LINETO]
        #     path = Path(verts, codes)
        #     patch = patches.PathPatch(path, edgecolor='r', facecolor='none', lw=1.3, zorder=11)
        #     ax.add_patch(patch)

    # new_path = zip(path[:-1], path[1:])

    tree_anim = [(0, an) for an in tree.animation]
    path_anim = [(1, an) for an in path]

    ani_tree = animation.FuncAnimation(fig, animate_tree, frames=[*tree_anim, *path_anim] , interval=interval, repeat=False)

    ani_tree.save('3_rrt.mp4', writer = FFwriter(15))
    # plt.show(block=True)

