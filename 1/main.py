from file_parse import *
from rrt import get_bounds, make_configuration_space, rrt
from tree import *
from visualizer import *
from sampler import *
from collision import *

def main():
    robot, obstacles, problems = parse_problem('robot_env_01.txt', 'probs_01.txt')
    start, goal = problems[0][0], problems[0][1]

    # new_obstacles = make_configuration_space(robot, obstacles)
    # visualize_points(np.array(new_obstacles[0]), robot, new_obstacles, start, goal, obstacles)

    # tree, path = rrt_star(robot, obstacles, start, goal, 10, return_tree=True)
    # visualize_path(robot, obstacles, path, tree=tree)

    # tree, path = rrt(robot, obstacles, start, goal, 500, return_tree=True)
    # visualize_path(robot, obstacles, path, tree=tree)

    # visualize_configuration(robot, obstacles, start, goal)
    # visualize_problem(robot, obstacles, start, goal)
    
    animate_rrt(robot, obstacles, start, goal, 500)
    # visualize_rrt(robot, obstacles, start, goal, 100)
    # visualize_rrt_star(robot, obstacles, start, goal, 400)
    animate_rrt_star(robot, obstacles, start, goal, 400)


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