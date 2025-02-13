import os  # for time functions
import math  # for infinity
from search import *  # for search engines
from sokoban import sokoban_goal_state, SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems
from collections import deque
import random


def heur_alternate(state):
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    dist = 0
    s_list = MinHeap(D(0, 0, -1))
    r_list = MinHeap(D(0, 0, -1))

    for b in state.boxes:
        # check for deadlock
        if lock(b, state):
            return math.inf

        # store manhattan distance from nearest available storage location and robot in min heap
        for s in state.storage:
            d = abs(b[0] - s[0]) + abs(b[1] - s[1])
            s_list.insert(D(b, s, d))
        for r in state.robots:
            r_list.insert(D(b, r, abs(b[0] - r[0]) + abs(b[1] - r[1])))

    stored = []
    boxed = []
    i = 0

    # check for box against wall that can only be pushed into wall storage location
    for m in s_list.heap[1:]:
        if m.box[0] == 0 and m.x[0] == 0 or m.box[1] == 0 and m.x[1] == 0 or m.box[0] == state.width and m.x[
            0] == state.width or m.box[1] == state.height and m.x[0] == state.height:
            if m.x not in stored and m.box not in boxed:
                stored.append(m.x)
                boxed.append(m.box)
                dist += m.d
                i += 1

    # find distance through min heap
    while i < len(state.boxes):
        m = s_list.extract_min()
        if m.x not in stored and m.box not in boxed:
            stored.append(m.x)
            boxed.append(m.box)
            dist += m.d
            i += 1

    # get distance from robots
    bots = []
    i = 0
    while i < len(state.robots):
        m = r_list.extract_min()
        if m.x not in bots:
            bots.append(m.x)
            dist += m.d / 1.3
            i += 1
    return dist

def lock(b, state):

    if b in state.storage:
        return False

    #check for corner/wall
    if b[0] == 0:
        if b[1] == 0:
            return True
        else:
            for i in range(state.height):
                if (0, i) in state.storage:
                    return False
            return True
    elif b[1] == 0:
        for i in range(state.width):
            if (i, 0) in state.storage:
                return False
        return True

    if b[0] == state.width:
        if b[1] == state.height:
            return True
        else:
            for i in range(state.height):
                if (state.width, i) in state.storage:
                    return False
            return True
    elif b[1] == state.height:
        for i in range(state.width):
            if (i, state.width) in state.storage:
                return False
        return True

    #check for cornered by obstacles
    ud = False
    lr = False
    for o in state.obstacles:
        if o[0] == b[0] and abs(o[1] - b[1]) == 1:
            ud = True
        if o[1] == b[1] and abs(o[0] - b[0]) == 1:
            lr = True
    if ud and lr: return True

class D:
    def __init__(self, box, x, d):
        self.box = box
        self.x = x
        self.d = d
    def __lt__(self, other):
        return self.d < other.d
    def __str__(self):
        #return 'Box: ' + str(self.box) + ' Storage: ' + str(self.storage) + ' Distance: ' + str(self.d)
        return str((self.box, self.x, self.d))

class MinHeap:
    def __init__(self, type_min):
        self.heap = [type_min]
        self.size = 0

    def swap(self, a, b):
        self.heap[a], self.heap[b] = self.heap[b], self.heap[a]

    def __str__(self):
        s = 'Heap:\n'
        for i in range(self.size):
            s += str(self.heap[i+1]) + '\n'
        return s

    def insert(self, x):
        self.heap.append(x)
        self.size += 1
        i = self.size
        while self.heap[i] < self.heap[i//2]:
            self.swap(i, i//2)
            i = i//2
        return self.heap

    def extract_min(self):
            if len(self.heap) <= 1:
                print('empty')
                return -1
            m = self.heap[1]
            if len(self.heap) > 2:
                self.heap[1] = self.heap.pop()
            self.size -= 1
            self.heapify(1)
            return m

    def heapify(self, x):
        if 2*x < self.size: # not a leaf
            if (self.heap[x] > self.heap[2*x] or
                    self.heap[x] > self.heap[2*x + 1]):

                # Swap with the left child and heapify the left child
                if self.heap[2*x] < self.heap[2*x + 1]:
                    self.swap(x, 2*x)
                    self.heapify(2*x)

                # Swap with the right child and heapify the right child
                else:
                    self.swap(x, 2*x + 1)
                    self.heapify(2*x + 1)


def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def heur_manhattan_distance(state):
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    dist = 0
    for b in state.boxes:
        dist += min([abs(b[0] - s[0]) + abs(b[1] - s[1]) for s in state.storage])
    return dist

def fval_function(sN, weight):
    return sN.gval + weight * sN.hval


def weighted_astar(initial_state, heur_fn, weight, timebound):
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of weighted astar algorithm'''
    se = SearchEngine('custom', 'default')
    fval = (lambda sN: fval_function(sN, weight))
    se.init_search(initial_state, goal_fn=sokoban_goal_state, heur_fn=heur_fn, fval_function=fval)
    goal, stats = se.search(timebound)

    if goal:
        return goal, stats
    else:
        return False, stats

def iterative_astar(initial_state, heur_fn, weight=1, timebound=5):
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of iterative astar algorithm'''

    search_start_time = os.times()[0]
    search_stop_time = search_start_time + timebound
    goal, stats = None, None
    min_path = math.inf

    weight = 5

    while os.times()[0] < search_stop_time:
        se = SearchEngine('custom', 'default')
        fval = (lambda sN: fval_function(sN, weight))
        se.init_search(initial_state, goal_fn=sokoban_goal_state, heur_fn=heur_fn, fval_function=fval)
        goal_, stats_ = se.search(search_stop_time - os.times()[0], (min_path, min_path/2, min_path))
        if goal_:
            goal, stats = goal_, stats_
            min_path = goal.gval
            #print(weight, goal.gval)
        weight = weight / 2
    if goal:
        return goal, stats
    else:
        return False, stats

def iterative_gbfs(initial_state, heur_fn, timebound=5):
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of iterative gbfs algorithm'''
    search_start_time = os.times()[0]
    search_stop_time = search_start_time + timebound
    goal, stats = None, None
    min_path = math.inf

    se = SearchEngine('best_first', 'default')
    se.init_search(initial_state, goal_fn=sokoban_goal_state, heur_fn=heur_fn)

    while os.times()[0] < search_stop_time:
        goal_, stats_ = se.search(search_stop_time - os.times()[0], (min_path, math.inf, math.inf))
        if goal_:
            goal, stats = goal_, stats_
            min_path = goal.gval - 1
            #print(min_path, goal.gval)
    if goal:
        return goal, stats
    else:
        return False, stats



