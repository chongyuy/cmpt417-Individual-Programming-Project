import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        # test your code by adding a constraint in prioritized.py that prohibits agent 0 from
        # being at its goal cell (1, 5)
        # constraints = [
        #     {
        #         'agent': 0,
        #         'loc': [(1, 5)],
        #         'timestep': 4
        #     }
        # ]
        # 1.3
        # constraints = [
        #     {
        #         'agent': 1,
        #         'loc': [(1, 2), (1, 3)],
        #         'timestep': 1
        #     }
        # ]
        # constraints = [
        #     {
        #         'agent': 0,
        #         'loc': [(1, 5)],
        #         'timestep': 10
        #     }
        # ]
        # question 1.5
        # constraints = [
        #     {
        #         'agent': 1,
        #         'loc': [(1, 4)],
        #         'timestep': 2
        #     },
        #     {
        #         'agent': 1,
        #         'loc': [(1, 2)],
        #         'timestep': 2
        #     },
        #     {
        #         'agent': 1,
        #         'loc': [(1, 3)],
        #         'timestep': 2
        #     }
        # ]
        # iterate over the path of the current
        constraints = []
        max_step = len(max(self.my_map)) * len(self.my_map)
        # print(mymap_size)
        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            temp_goal = self.goals[i]
            self.my_map[temp_goal[0]][temp_goal[1]] = True
            if path is None:
                raise BaseException('No solutions')
            # adds all necessary vertex constraints.
            # this is the loop for all other agents
            for j in range(self.num_of_agents):
                if i == j:
                    continue
                else:
                    time_step = 0
                    for node in path:
                        # add vertex constraints for all future agents
                        constraints.append(
                            {
                                'agent': j,
                                'loc': [node],
                                'timestep': time_step,
                                'positive': False
                            }
                        )
                        time_step += 1
                        # add edge constraints for all future agents

                        if time_step < len(path):
                            constraints.append(
                                {
                                    'agent': j,
                                    'loc': [path[time_step], node],
                                    'timestep': time_step,
                                    'positive': False
                                }
                            )
                # if the Higher priority agent arrive the goal first, the low priority agents still can
                # not move on the top of it
                goal_timestep = time_step-1
                while time_step <= max_step:
                    constraints.append(
                        {
                            'agent': j,
                            'loc': [path[goal_timestep]],
                            'timestep': time_step,
                            'positive': False
                        }
                    )
                    time_step += 1
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches

            ##############################
        for goal in self.goals:
            self.my_map[goal[0]][goal[1]] = False
        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        return result
