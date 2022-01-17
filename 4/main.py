from numpy.core.numeric import ones
from numpy.lib.arraypad import pad
from file_parse import *
from rrt import get_bounds, make_configuration_space, rrt
from tree import *
from visualizer import *
from sampler import *
from collision import *

def main():
    robot, obstacles, problems = parse_problem('robot_env_03.txt', 'probs_01.txt')
    start, goal = problems[0][0], problems[0][1]

    # print(start, goal)

    # print(robot.transform())

    # tree = Tree(robot, obstacles, tuple(start), tuple(goal))
    # tree.extend(tuple(start), 100, 2000, 1e-3)
    # tree.extend(tuple(start), 100, 2000, 1e-3)
    # tree.extend(tuple(start), 100, 2000, 1e-3)
    # tree.extend(tuple(start), 100, 2000, 1e-3)
    # tree.extend(tuple(start), 100, 2000, 1e-3)
    # print(tree.durations[tree.root.point])

    # points = [start, None]
    # q, q_dot = robot.propagate(tuple(points[-1]), [tree.controls[tree.root.point]], [tree.durations[tree.root.point]], 1)

    # points.append(q)

    # c = tree.controls[tree.root.point]

    # q, q_dot = robot.propagate(tuple(points[-1]), [[c[0], 0]], [tree.durations[tree.root.point]], 1, q_dot)

    # points.append(q)

    # for i in range(1):
    #     print()
    #     points.append(robot.propagate(tuple(points[-1]), [[*tree.controls[tree.root.point][:1], 0]], [tree.durations[tree.root.point]], 1))


    # visualize_points([], robot, obstacles, start, goal, tree)
    # visualize_path(robot, obstacles, points, tree=tree)

    # print(start)
    # tree.extend(tuple(start), (2.0, 2.0, 0.0))
    # tree.extend(tuple(start), (1.2, 1.2, 0.0))

    # print(tree.tree_nodes[tuple(start)].children)
    # visualize_problem(robot, obstacles, start, goal)
    # new_obstacles = make_configuration_space(robot, obstacles)

    # tree, path = rrt_star(robot, obstacles, start, goal, 500, return_tree=True)
    # visualize_path(robot, obstacles, path, tree=tree)
    
    tree, path = rrt(robot, obstacles, start, goal, 5000, return_tree=True)
    visualize_path(robot, obstacles, path, tree=tree)

    # visualize_configuration(robot, obstacles, start, goal)
    # visualize_problem(robot, obstacles, start, goal)
    
    # visualize_rrt(robot, obstacles, start, goal, 100)
    # visualize_rrt_star(robot, obstacles, start, goal, 400)


    # new_obstacles = make_configuration_space([[0., 0.], [0.2, 0.2], [.4, 0.]], [[[4.0, 4.0], [4.0, 6.0], [6.0, 6.0], [6.0, 4.0]]])
    # visualize_points(np.array(new_obstacles[0]), [[0., 0.], [0.2, 0.2], [.4, 0.]], [[[4.0, 4.0], [4.0, 6.0], [6.0, 6.0], [6.0, 4.0]]], start, goal)
    # visualize_problem([[0., 0.], [0.2, 0.2], [.4, 0.]], new_obstacles, start, goal, [[[4.0, 4.0], [4.0, 6.0], [6.0, 6.0], [6.0, 4.0]]])
    
    # new_obstacles = make_configuration_space(robot, obstacles)
    # # visualize_points(np.array(new_obstacles[0]), robot, new_obstacles, start, goal, [obstacles[2]])
    # # visualize_problem(robot, new_obstacles, start, goal, obstacles)
    # tree, path = rrt(robot, new_obstacles, start, goal, 500, return_tree=True)
    # visualize_path(robot, new_obstacles, path, obstacles, tree)
    # visualize_path(robot, obstacles, path)
    # 7.62, 8.77
    # 6.96, 4.88

    # tree = Tree(robot, obstacles, (7.62, 8.77), goal)

    # tree.extend((7.62, 8.77), (6.96, 4.88))

    # visualize_tree(tree, robot, obstacles, (7.62, 8.77), goal)
    

    # points_free = []
    # points_collide = []
    # for _ in range(100):
    #     point = sample()
    #     if isCollisionFree(robot, point, obstacles):
    #         points_free.append(point)
    #     else:
    #         points_collide.append(point)
    
    # points = {'collide':points_collide, 'free': points_free}
    
    # visualize_points(points, robot, obstacles, start, goal)

if __name__  == '__main__':
    main()