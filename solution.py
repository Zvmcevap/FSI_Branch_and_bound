from heapq import heappop, heappush
from search import *


def branch_and_bound_search(problem, underestimation=False):
    node = Node(problem.initial)
    generated = 0
    closed_list = set()
    open_list = []
    cost = node.path_cost + (problem.h(node) if underestimation else 0)
    heappush(open_list, (cost, node))

    while open_list:
        _, node = heappop(open_list)
        closed_list.add(node.state)

        if problem.goal_test(node.state):
            return [n.state for n in node.path()], generated, len(closed_list), node.path_cost

        for child in node.expand(problem):
            generated += 1
            if child.state not in closed_list:
                child_cost = child.path_cost + (problem.h(child) if underestimation else 0)
                heappush(open_list, (child_cost, child))

    return None, generated, len(closed_list), 0


def run_search_cases(initial_state, goal_state, problem):
    print(f"Road from {initial_state} to: {goal_state}")

    result, generated, visited, total_cost = branch_and_bound_search(problem)
    result_with_underestimation, g_ue, v_ue, tc_ue = branch_and_bound_search(problem, underestimation=True)

    result_bfs = breadth_first_graph_search(problem)
    result_dfs = depth_first_graph_search(problem)

    print_search_results("Breadth-first Search", result_bfs)
    print_search_results("Depth-first Search", result_dfs)
    print_search_results("Branch and Bound", result, generated, visited, total_cost)
    print_search_results("Branch and Bound with Underestimation", result_with_underestimation, g_ue, v_ue, tc_ue)


def print_search_results(name, result, generated=None, visited=None, total_cost=None):
    print(f"{name}:")
    if visited:
        print(
            f"Route: {result}, Generated: {generated}, Visited: {visited}, Total Cost: {total_cost}")
    elif result:
        print(
            f"Route: {result.path()}, Generated: {len(result.path())}, Visited: {result.depth}, Total Cost: {result.path_cost}")
    else:
        print("No solution found.")


# Define the search cases
initial_state_goal_state_pairs = [
    ('A', 'B'),
    ('O', 'E'),
    ('G', 'Z'),
    ('N', 'D'),
    ('M', 'F')
]

for initial_state, goal_state in initial_state_goal_state_pairs:
    problem = GPSProblem(initial_state, goal_state, romania)
    run_search_cases(initial_state, goal_state, problem)

with open('solutions.txt') as f:
    sol_1 = f.readlines()
with open('solutions2.txt') as f:
    sol_2 = f.readlines()

for i in range(len(sol_1)):
    l1, l2 = sol_1[i].strip(), sol_2[i].strip()
    print(l1 == l2)
    if l1 != l2:
        print(l1, l2)
