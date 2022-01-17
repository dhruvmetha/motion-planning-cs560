import numpy as np
from numpy.core.defchararray import count

def is_invertible(a):
    a = np.array(a)
    return a.shape[0] == a.shape[1] and np.linalg.matrix_rank(a) == a.shape[0]

def isCollisionFree(robot, point, obstacles):

    # (move/imagine) project the robot onto the point
    rob = np.array(robot)
    projected_robot = rob + np.array(point)

    # boundary check
    for v in projected_robot:
        x, y = v[0], v[1]
        if x <= 0 or y <= 0 or x >= 10 or y >= 10:
            return False 

    # TODO obstacle inside the polygon check
    

    # obstacle check
    edges_rob  = []
    b = []
    for i in range(len(projected_robot) - 1):
        p1, p2 = projected_robot[i], projected_robot[i+1]
        x1, y1, x2, y2 = p1[0], p1[1], p2[0], p2[1]
        edges_rob.append([y1-y2, x2-x1, x2*y1 - x1*y2])
    

    for m, obs in enumerate(obstacles):
        # print('obstacle', m)
        for idx, edge in enumerate(edges_rob):
            # print('eddgee', idx, projected_robot[idx], projected_robot[(idx+1)%len(projected_robot)])
            count_inter = 0
            A = []
            b = []
            inter_set = set()
            for i in range(len(obs)):
                p1, p2 = obs[i], obs[(i+1)% len(obs)]
                x1, y1, x2, y2 = p1[0], p1[1], p2[0], p2[1]
                A = [edge[:-1], [y1-y2, x2-x1]]
                b = [edge[-1], x2*y1 - x1*y2]
 
                # print(p1, p2, 'edge count', i)
                # print(A)
                if not is_invertible(A):
                    # print('parallel lines', idx)
                    continue
                
                inter = np.around(np.matmul(np.linalg.inv(A),  np.array(b)), 5)
                # print(inter)
                ub_x, lb_x = max(x1, x2), min(x1, x2)
                ub_y, lb_y = max(y1, y2), min(y1, y2)

                # print('intersect', inter, x1, y1, x2, y2,' ,', inter[1], lb_y , inter[0] >= lb_x, inter[0] <= ub_x, inter[1] >= lb_y, inter[1] <= ub_y)

                if inter[0] >= lb_x and inter[0] <= ub_x and inter[1] >= lb_y and inter[1] <= ub_y:
                    p1, p2 = projected_robot[idx], projected_robot[(idx+1) % len(projected_robot)]
                    x1, y1, x2, y2 = p1[0], p1[1], p2[0], p2[1]
                    ub_x, lb_x = max(x1, x2), min(x1, x2)
                    ub_y, lb_y = max(y1, y2), min(y1, y2)
                    # print('intersect yes')
                    if inter[0] > ub_x:
                        count_inter += 1
                        inter_set.add(tuple(inter.tolist()))
                        # print('here111', len(inter_set), count_inter)

                    # print(inter, count_inter)
                        
                    if inter[0] >= lb_x and inter[0] <= ub_x and inter[1] >= lb_y and inter[1] <= ub_y:
                        return False
            if (len(inter_set) % 2 == 1 and count_inter % 2 == 1):
                # print('here here', count_inter, projected_robot[idx], projected_robot[(idx+1) % len(projected_robot)])
                return False    
    return True