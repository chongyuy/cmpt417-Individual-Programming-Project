import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    for time in range(0, max(len(path1), len(path2))):
        # vertex collision
        if get_location(path1, time) == get_location(path2, time):
            return [[get_location(path1, time)], time]
        # edge collision

        if get_location(path1, time) == get_location(path2, time + 1) and get_location(path1,
                                                                                       time + 1) == get_location(
            path2, time):
            return [[get_location(path1, time), get_location(path2, time)], time + 1]
    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []
    for agent1 in range(len(paths)):
        for agent2 in range(agent1 + 1, len(paths)):
            collision = detect_collision(paths[agent1], paths[agent2])
            if collision:
                collisions += [{
                    'a1': agent1,
                    'a2': agent2,
                    'loc': collision[0],
                    'timestep': collision[1]
                }]

    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    constrain = [
        {
            'agent': collision['a1'],
            'loc': collision['loc'],
            'timestep': collision['timestep'],
            'positive': False
        },
        {
            'agent': collision['a2'],
            'loc': collision['loc'][::-1],
            'timestep': collision['timestep'],
            'positive': False
        }
    ]
    return constrain


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    if random.randint(0, 1) == 0:
        constrain = [
            {
                'agent': collision['a1'],
                'loc': collision['loc'],
                'timestep': collision['timestep'],
                'positive': False
            },
            {
                'agent': collision['a1'],
                'loc': collision['loc'],
                'timestep': collision['timestep'],
                'positive': True
            }
        ]
    else:
        constrain = [
            {
                'agent': collision['a2'],
                'loc': collision['loc'][::-1],
                'timestep': collision['timestep'],
                'positive': False
            },
            {
                'agent': collision['a2'],
                'loc': collision['loc'][::-1],
                'timestep': collision['timestep'],
                'positive': True
            }
        ]
    return constrain

# the given helper function
def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        while self.open_list:
            p = self.pop_node()
            if len(p['collisions']) == 0:
                self.print_results(p)
                return p['paths']
            collision = p['collisions'][0]
            if not disjoint:
                print('use standard_splitting')
                constrains = standard_splitting(collision)
            else:
                print('use disjoint_splitting')
                constrains = disjoint_splitting(collision)

            for constraint in constrains:
                q = {
                    'cost': 0,
                    'constraints': p['constraints'] + [constraint],
                    'paths': [] + p['paths'],
                    'collisions': []
                }
                if not constraint['positive']:
                    a = constraint['agent']
                    path = a_star(self.my_map, self.starts[a], self.goals[a], self.heuristics[a], a, q['constraints'])
                    if path:
                        q['paths'][a] = path
                        q['collisions'] = detect_collisions(q['paths'])
                        q['cost'] = get_sum_of_cost(q['paths'])
                        self.push_node(q)
                else:
                    whether_add_node = True
                    other_agents = paths_violate_constraint(constraint, p['paths'])
                    # modify all other agents' path
                    for agent in other_agents:
                        path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], agent,
                                      q['constraints'])
                        if path:
                            q['paths'][agent] = path
                        else:
                            whether_add_node = False
                            break
                    a = constraint['agent']
                    # modify target agent's path
                    path = a_star(self.my_map, self.starts[a], self.goals[a], self.heuristics[a], a, q['constraints'])
                    if path:
                        q['paths'][a] = path
                        q['collisions'] = detect_collisions(q['paths'])
                        q['cost'] = get_sum_of_cost(q['paths'])
                    else:
                        whether_add_node = False
                    # if all paths are exist, then we can add node to the tree
                    if whether_add_node:
                        self.push_node(q)

        self.print_results(root)
        return root['paths']

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
