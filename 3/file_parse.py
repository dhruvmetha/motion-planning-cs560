from robot import *
def parse_problem(world_file, problem_file):

    robot = []    
    obstacles = []
    problems = []

    lines = None
    with open(world_file, 'r')  as f:
        lines = f.readlines()

    rob = list(map(float, lines[0].strip().split()))
    robot = Robot(rob[0], rob[1])

    # pivot_x, pivot_y = rob[0], rob[1]
    # for i,j in zip(rob[::2], rob[1::2]):
    #     robot.append([i - pivot_x, j - pivot_y])

    for line in lines[1:]:
        obs = []
        line_f = list(map(float, line.strip().split()))
        for i,j in zip(line_f[::2], line_f[1::2]):
            obs.append([i, j])
        obstacles.append(obs)
    
    
    with open(problem_file, 'r')  as f:
        lines = f.readlines()

    for line in lines:
        prob = []
        line_f = list(map(float, line.strip().split()))
        for i,j,k in zip(line_f[::3], line_f[1::3], line_f[2::3]):
            prob.append([i, j, k])
        problems.append(prob)
    
    return (robot, obstacles, problems)

# print(parse_problem('robot_env_01.txt', 'probs_01.txt'))