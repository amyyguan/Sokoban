from algorithms import *
from sokoban import sokoban_goal_state, PROBLEMS
import os

# Select what to test
test_iterative_gbfs = False
test_iterative_astar = False
test_weighted_astar = True

def test_iterative_gbfs_fun():
    print('Testing iterative GBFS')

    solved = 0
    unsolved = []
    timebound = 2  # 2 second time limit
    for i in range(0, len(PROBLEMS)):
        print("*************************************")
        print("PROBLEM {}".format(i))

        s0 = PROBLEMS[i]  # Problems get harder as i gets bigger
        final, stats = iterative_gbfs(s0, heur_fn=heur_alternate, timebound=timebound)

        if final:
            final.print_path()
            solved += 1
        else:
            unsolved.append(i)

    print("\n*************************************")
    print("Of {} initial problems, {} were solved in less than {} seconds by this solver.".format(len(PROBLEMS), solved,
                                                                                                  timebound))

def test_iterative_astar_fun():
    print('Testing iterative Weighted A Star')

    solved = 0
    unsolved = []
    timebound = 2 # 2 second time limit

    gvals = []
    for i in range(0, len(PROBLEMS)):
        print("*************************************")
        print("PROBLEM {}".format(i))

        s0 = PROBLEMS[i]  # Problems get harder as i gets bigger
        weight = 10  # note that if you want to over-ride this initial weight in your implementation, you are welcome to!
        final, stats = iterative_astar(s0, heur_fn=heur_alternate, weight=weight, timebound=timebound)

        if final:
            final.print_path()
            solved += 1
        else:
            unsolved.append(i)

    print("\n*************************************")
    print("Of {} initial problems, {} were solved in less than {} seconds by this solver.".format(len(PROBLEMS), solved,
                                                                                                  timebound))
    print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))

def test_weighted_astar_fun():
    solved, score1 = 0, 0
    weights = [10, 5, 2, 1]

    for j in range(0, 5):  # tiny problems only!
        m = PROBLEMS[j]  # Problems get harder as j gets bigger
        state_counts = []
        gvals = []
        for weight in weights:
            final, stats = weighted_astar(m, heur_fn=heur_manhattan_distance, weight=weight,
                                          timebound=5)
            if final:
                solved += 1 #must solve one
                state_counts.append(stats.states_expanded)
                gvals.append(final.gval)
            else:
                state_counts.append(-99)
                gvals.append(-99)

        # now test the state_counts and gvals
        if solved == 0:
            flag = False  # solved nothing!
        else:
            flag = True

        for i in range(0, len(state_counts) - 2):  # forward check
            if state_counts[i + 1] != -99 and gvals[i + 1] == -99:  # no solution, means no comparison to be made
                if state_counts[i] > state_counts[i + 1] or gvals[i] < gvals[
                    i + 1]:  # state counts should be increasing and gvals decreasing
                    flag = False
        if flag: score1 += 1  # did we pass?

        for i in range(len(state_counts) - 1, 0, -1):  # backward check
            if state_counts[i - 1] != -99 and gvals[i - 1] == -99:  # no solution, means no comparison to be made
                if gvals[i - 1] == -99 and gvals[
                    i] != -99:  # no solution with a lower weight, but a solution with a higher one
                    flag = False

    summary_score = score1
    print("\n*************************************")
    print("Of the 20 runs over 5 problems, {} solutions were found with weighted a star in the time allotted.".format(
        solved))
    print("Weighted a-star expanded more nodes as weights decreased {} of 5 times".format(score1))
    print("*************************************\n")

def test_all():
    if test_iterative_gbfs: test_iterative_gbfs_fun()
    if test_iterative_astar: test_iterative_astar_fun()
    if test_weighted_astar: test_weighted_astar_fun()

if __name__=='__main__':
    test_all()

