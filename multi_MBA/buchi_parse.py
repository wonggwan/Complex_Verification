# # -*- coding: utf-8 -*-
# import subprocess
# import os.path
# import re
# import networkx as nx
# import numpy as np
# from networkx.classes.digraph import DiGraph
# from sympy import satisfiable
# from sympy.parsing.sympy_parser import parse_expr  # samarth change
# from itertools import combinations
#
#
# class Buchi(object):
#     """
#     construct buchi automaton graph
#     """
#
#     def __init__(self, task):
#         """
#         initialization
#         :param task: task specified in LTL
#         """
#         # task specified in LTL
#         self.formula = task.formula
#         self.subformula = task.subformula
#         self.number_of_robots = task.number_of_robots
#         # graph of buchi automaton
#         self.buchi_graph = DiGraph(type='buchi', init=[], accept=[])
#
#         # minimal length (in terms of number of transitions) between a pair of nodes
#         self.min_length = dict()
#
#     def construct_buchi_graph(self):
#         """
#         parse the output of the program ltl2ba and build the buchi automaton
#         """
#         # directory of the program ltl2ba
#         dirname = os.path.dirname(__file__)
#         # output of the program ltl2ba
#         output = subprocess.check_output(dirname + "/ltl2ba -f \"" + self.formula + "\"", shell=True).decode(
#             "utf-8")
#         # print(self.formula)
#         # output = subprocess.check_output("ltl2ba -f '<>e2 && <> (e1 || e4)'").decode("utf-8")
#
#         # find all states/nodes in the buchi automaton
#         state_re = re.compile(r'\n(\w+):\n\t')
#         state_group = re.findall(state_re, output)
#
#         # find initial and accepting states
#         init = [s for s in state_group if 'init' in s]
#         accept = [s for s in state_group if 'accept' in s]
#         # finish the inilization of the graph of the buchi automaton
#         self.buchi_graph.graph['init'] = init
#         self.buchi_graph.graph['accept'] = accept
#
#         order_key = list(self.subformula.keys())
#         order_key.sort(reverse=True)
#         # for each state/node, find it transition relations
#         for state in state_group:
#             # add node
#             self.buchi_graph.add_node(state)
#             # loop over all transitions starting from current state
#             state_if_fi = re.findall(state + r':\n\tif(.*?)fi', output, re.DOTALL)
#             if state_if_fi:
#                 relation_group = re.findall(r':: (\(.*?\)) -> goto (\w+)\n\t', state_if_fi[0])
#                 for symbol, next_state in relation_group:  # symbol that enables state to next_state
#                     # delete edges with multiple subformulas
#                     # if ' && ' in symbol: continue
#                     # whether the edge is feasible in terms of atomic propositions
#                     symbol_copy = symbol
#                     for k in order_key:
#                         symbol = symbol.replace('e{0}'.format(k), self.subformula[k][0])
#                     # get the trurh assignment
#                     truth_table_all, num_of_truths = self.get_truth_assignment(symbol)
#
#                     # infeasible transition
#
#                     if not truth_table_all: continue
#
#                     # list all regions to be avoided to ensure transition
#                     symbol_keys = re.findall(r'[0-9]+', symbol_copy)
#                     avoid_regions_all = []
#                     for truth_num in range(len(truth_table_all)):
#                         avoid_regions = {}
#                         truth_table = truth_table_all[truth_num]
#                         # print(truth_table)
#                         for i in range(self.number_of_robots):
#                             avoid_regions[i] = []
#                         for key in truth_table:
#                             if key != '1':
#                                 if truth_table[key] == False:
#                                     pair = key.split('_')  # region-robot pair
#                                     robot_index = int(pair[1]) - 1
#                                     distance = 0
#                                     for sub_f in symbol_keys:
#                                         if key in self.subformula[int(sub_f)][0]:
#                                             distance = self.subformula[int(sub_f)][2]
#                                     avoid_regions[robot_index].append((pair[0], distance))
#                         avoid_regions_all.append(avoid_regions)
#
#                     # add edge in NBA
#                     avoid_current_state = {}
#                     if state != next_state:
#                         avoid_current_state = self.buchi_graph.edges[(state, state)]['avoid']
#                     self.buchi_graph.add_edge(state, next_state, AP=symbol_copy, AP_keys=symbol_keys,
#                                               truth=truth_table_all[0], avoid=avoid_regions_all[0],
#                                               all_truth=truth_table_all, all_avoid=avoid_regions_all,
#                                               counter=0, avoid_self_loop=avoid_current_state)
#             else:
#                 state_skip = re.findall(state + r':\n\tskip\n', output, re.DOTALL)
#                 if state_skip:
#                     avoid_regions = {}
#                     avoid_current_state = {}
#                     for i in range(self.number_of_robots):
#                         avoid_regions[i] = []
#                     self.buchi_graph.add_edge(state, state, AP='1', AP_keys=[], truth='1',
#                                               avoid=avoid_regions, avoid_self_loop=avoid_current_state)
#
#     def get_truth_assignment(self, symbol):
#         """
#         get one set of truth assignment that makes the symbol true
#         :param symbol: logical expression which controls the transition
#         :rrn: a set of truth assignment enables the symbol
#         """
#         # empty symbol
#         if symbol == '(1)':
#             return '1', 1
#         # non-empty symbol
#         else:
#             exp = symbol.replace('||', '|').replace('&&', '&').replace('!', '~')
#             # add extra constraints: a single robot can reside in at most one region
#             if '|' not in exp:
#                 robot_region = self.robot2region(exp)
#                 for robot, region in robot_region.items():
#                     mutual_execlusion = list(combinations(region, 2))
#                     # single label in the symbol
#                     if not mutual_execlusion: continue
#                     for i in range(len(mutual_execlusion)):
#                         mutual_execlusion[i] = '(~(' + ' & '.join(list(mutual_execlusion[i])) + '))'
#                     exp = '(' + exp + ') & ' + ' & '.join(mutual_execlusion)
#
#                 exp = parse_expr(exp)
#
#                 # find one truth assignment that makes symbol true using function satisfiable
#                 truth = satisfiable(exp)
#                 try:
#                     truth_table = dict()
#                     for key, value in truth.items():
#                         truth_table[key.name] = value
#                 except AttributeError:
#                     return False, 0
#                 else:
#                     return [truth_table], 1
#             # In case of or condition in logic multiple truth values may be possible
#             else:
#                 robot_region = self.robot2region(exp)
#                 for robot, region in robot_region.items():
#                     mutual_execlusion = list(combinations(region, 2))
#                     # single label in the symbol
#                     if not mutual_execlusion: continue
#                     for i in range(len(mutual_execlusion)):
#                         mutual_execlusion[i] = '(~(' + ' & '.join(list(mutual_execlusion[i])) + '))'
#                     exp = '(' + exp + ') & ' + ' & '.join(mutual_execlusion)
#
#                 exp = parse_expr(exp)  # samarth change
#                 # find one truth assignment that makes symbol true using function satisfiable
#                 # truth = satisfiable(exp, algorithm="dpll")
#                 # truth_simple = satisfiable(exp)
#                 truth = satisfiable(exp, all_models=True)
#                 try:
#                     all_truths = []
#                     for i in truth:
#                         truth_table = dict()
#                         for key, value in i.items():
#                             truth_table[key.name] = value
#                         all_truths.append(truth_table)
#                 except AttributeError:
#                     return False, 0
#                 else:
#                     return all_truths, len(all_truths)
#
#     def get_minimal_length(self):
#         """
#         search the shortest path from a node to another, i.e., # of transitions in the path
#         :return:
#         """
#         # loop over pairs of buchi states
#         for head_node in self.buchi_graph.nodes():
#             for tail_node in self.buchi_graph.nodes():
#                 # head_node = tail_node, and tail_node is an accepting state
#                 if head_node != tail_node and 'accept' in tail_node:
#                     try:
#                         length, _ = nx.algorithms.single_source_dijkstra(self.buchi_graph,
#                                                                          source=head_node, target=tail_node)
#                     # couldn't find a path from head_node to tail_node
#                     except nx.exception.NetworkXNoPath:
#                         length = np.inf
#                     self.min_length[(head_node, tail_node)] = length
#                 # head_node != tail_node and tail_node is an accepting state
#                 # move 1 step forward to all reachable states of head_node then calculate the minimal length
#                 elif head_node == tail_node and 'accept' in tail_node:
#                     length = np.inf
#                     for suc in self.buchi_graph.succ[head_node]:
#                         try:
#                             len1, _ = nx.algorithms.single_source_dijkstra(self.buchi_graph,
#                                                                            source=suc, target=tail_node)
#                         except nx.exception.NetworkXNoPath:
#                             len1 = np.inf
#                         if len1 < length:
#                             length = len1 + 1
#                     self.min_length[(head_node, tail_node)] = length
#
#     def get_feasible_accepting_state(self):
#         """
#         get feasbile accepting/final state, or check whether an accepting state is feaasible
#         :return:
#         """
#         accept = self.buchi_graph.graph['accept']
#         self.buchi_graph.graph['accept'] = []
#         for ac in accept:
#             for init in self.buchi_graph.graph['init']:
#                 if self.min_length[(init, ac)] < np.inf and self.min_length[(ac, ac)] < np.inf:
#                     self.buchi_graph.graph['accept'].append(ac)
#                     break
#
#     def robot2region(self, symbol):
#         """
#         pair of robot and corresponding regions in the expression
#         :param symbol: logical expression
#         :return: robot index : regions
#         eg: input:  exp = 'l1_1 & l3_1 & l4_1 & l4_6 | l3_4 & l5_6'
#             output: {1: ['l1_1', 'l3_1', 'l4_1'], 4: ['l3_4'], 6: ['l4_6', 'l5_6']}
#         """
#
#         robot_region = dict()
#         for r in range(self.number_of_robots):
#             findall = re.findall(r'(l\d+?_{0})[^0-9]'.format(r + 1), symbol) \
#                       + re.findall(r'(m\d+?_{0})[^0-9]'.format(r + 1), symbol) \
#                       + re.findall(r'(m\d+?r\d+?_{0})[^0-9]'.format(r + 1), symbol)
#             if findall:
#                 findall = list(dict.fromkeys(findall))  # remove duplicates from list
#                 robot_region[str(r + 1)] = findall
#
#         return robot_region
#
#     def update_alternate_transition(self, state, next_state):
#         """
#         updates the truth to next alternate possiblity
#         that enables transition from state to next_state.
#         :param state: current state
#         :param next_state: next state
#         :return: success : true if success in finding alternate transition
#         """
#         edge_info = self.buchi_graph.edges[state, next_state]
#         next_counter = edge_info['counter'] + 1
#         possible_truths = len(edge_info['all_truth'])
#         # if all possible transitions have been used then return false
#         if next_counter >= possible_truths:
#             return False
#
#         self.buchi_graph.edges[state, next_state]['truth'] = edge_info['all_truth'][next_counter]
#         self.buchi_graph.edges[state, next_state]['avoid'] = edge_info['all_avoid'][next_counter]
#         self.buchi_graph.edges[state, next_state]['counter'] = next_counter
#         return True
#
#     def previous_alternate_transition(self, state, next_state):
#         """
#         updates the truth to previous alternate possiblity
#         that enables transition from state to next_state.
#         :param state: current state
#         :param next_state: next state
#         :return: success : true if success in finding alternate transition
#         """
#         edge_info = self.buchi_graph.edges[state, next_state]
#         next_counter = edge_info['counter'] - 1
#         # if all possible transitions have been used then return false
#         if next_counter < 0:
#             return False
#
#         self.buchi_graph.edges[state, next_state]['truth'] = edge_info['all_truth'][next_counter]
#         self.buchi_graph.edges[state, next_state]['avoid'] = edge_info['all_avoid'][next_counter]
#         self.buchi_graph.edges[state, next_state]['counter'] = next_counter
#         return True
#
#     def ctr_alternate_transition(self, state, next_state, next_counter):
#         """
#         updates the truth to alternate possiblity pointed by next_counter
#         that enables transition from state to next_state.
#         :param state: current state
#         :param next_state: next state
#         :param next_counter: will set counter to this value
#         :return: success : true if success in finding alternate transition
#         """
#         edge_info = self.buchi_graph.edges[state, next_state]
#         possible_truths = len(edge_info['all_truth'])
#         # if all possible transitions have been used then return false
#         if next_counter < 0 and next_counter >= possible_truths:
#             return False
#
#         self.buchi_graph.edges[state, next_state]['truth'] = edge_info['all_truth'][next_counter]
#         self.buchi_graph.edges[state, next_state]['avoid'] = edge_info['all_avoid'][next_counter]
#         self.buchi_graph.edges[state, next_state]['counter'] = next_counter
#         return True
#
#     def delete_transition(self, state, next_state):
#         """
#         deletes edge.
#         :param state: current state
#         :param next_state: next state
#         Throws error if deleting edge results in no possible paths from state to accepting_state
#         """
#         self.buchi_graph.remove_edge(state, next_state)
#         self.get_minimal_length()
#         self.get_feasible_accepting_state()
#         if not self.buchi_graph.graph['accept']:
#             # raise Exception("No more transitions to accepting state")
#             return True  # verification failed
#         final_state = self.buchi_graph.graph['accept'][0]
#         if self.min_length[state, final_state] == np.inf:
#             # raise Exception("No more transitions to accepting state")
#             return True  # verification failed
#
#     def get_next_NBA_state(self, curr_state, accepting_state):
#         """
#         returns next state closest to accepting/final state.
#         :param curr_state: current state in graph
#         :param accepting_state: current state in graph
#         :return: next_State : state closest to accepting/final state
#         """
#         print("nx.shortest_path -> {}".format(nx.shortest_path(self.buchi_graph, curr_state, accepting_state)))
#         return nx.shortest_path(self.buchi_graph, curr_state, accepting_state)[1]
#         # return nx.shortest_path(self.buchi_graph, curr_state, accepting_state)
#
#     def get_next_action(self, curr_state, accepting_state):
#         """
#         returns the truth value or action that enables the transition between states.
#         :param curr_state: current state in graph
#         :param accepting_state: current state in graph
#         :return: next_State : target state
#         """
#         return self.buchi_graph.edges[curr_state, accepting_state]['truth']


# -*- coding: utf-8 -*-

import subprocess
import os.path
import re
import networkx as nx
import numpy as np
from networkx.classes.digraph import DiGraph
from sympy import satisfiable
from sympy.parsing.sympy_parser import parse_expr  # samarth change

from itertools import combinations


class Buchi(object):
    """
    construct buchi automaton graph
    """

    def __init__(self, task):
        """
        initialization
        :param task: task specified in LTL
        """
        # task specified in LTL
        self.formula = task.formula
        self.subformula = task.subformula
        self.number_of_robots = task.number_of_robots
        # graph of buchi automaton
        self.buchi_graph = DiGraph(type='buchi', init=[], accept=[])

        # minimal length (in terms of number of transitions) between a pair of nodes
        self.min_length = dict()

    def construct_buchi_graph(self):
        """
        parse the output of the program ltl2ba and build the buchi automaton
        """
        # directory of the program ltl2ba
        dirname = os.path.dirname(__file__)
        # output of the program ltl2ba
        output = subprocess.check_output(dirname + "/./ltl2ba.exe -f \"" + self.formula + "\"", shell=True).decode(
            "utf-8")

        # find all states/nodes in the buchi automaton
        state_re = re.compile(r'\n(\w+):\n\t')
        state_group = re.findall(state_re, output)

        # find initial and accepting states
        init = [s for s in state_group if 'init' in s]
        accept = [s for s in state_group if 'accept' in s]
        # finish the inilization of the graph of the buchi automaton
        self.buchi_graph.graph['init'] = init
        self.buchi_graph.graph['accept'] = accept

        order_key = list(self.subformula.keys())
        order_key.sort(reverse=True)
        # for each state/node, find it transition relations
        for state in state_group:
            # add node
            self.buchi_graph.add_node(state)
            # loop over all transitions starting from current state
            state_if_fi = re.findall(state + r':\n\tif(.*?)fi', output, re.DOTALL)
            if state_if_fi:
                relation_group = re.findall(r':: (\(.*?\)) -> goto (\w+)\n\t', state_if_fi[0])
                for symbol, next_state in relation_group:  # symbol that enables state to next_state
                    # delete edges with multiple subformulas
                    # if ' && ' in symbol: continue
                    # whether the edge is feasible in terms of atomic propositions
                    symbol_copy = symbol
                    for k in order_key:
                        symbol = symbol.replace('e{0}'.format(k), self.subformula[k][0])
                    # get the trurh assignment
                    truth_table_all, num_of_truths = self.get_truth_assignment(symbol)

                    # infeasible transition

                    if not truth_table_all: continue

                    # list all regions to be avoided to ensure transition
                    symbol_keys = re.findall(r'[0-9]+', symbol_copy)
                    avoid_regions_all = []
                    for truth_num in range(len(truth_table_all)):
                        avoid_regions = {}
                        truth_table = truth_table_all[truth_num]
                        for i in range(self.number_of_robots):
                            avoid_regions[i] = []
                        for key in truth_table:
                            if key != '1':
                                if truth_table[key] == False:
                                    pair = key.split('_')  # region-robot pair
                                    robot_index = int(pair[1]) - 1
                                    distance = 0
                                    for sub_f in symbol_keys:
                                        if key in self.subformula[int(sub_f)][0]:
                                            distance = self.subformula[int(sub_f)][2]
                                    avoid_regions[robot_index].append((pair[0], distance))
                        avoid_regions_all.append(avoid_regions)

                    # add edge in NBA
                    avoid_current_state = {}
                    if state != next_state:
                        avoid_current_state = self.buchi_graph.edges[(state, state)]['avoid']
                    self.buchi_graph.add_edge(state, next_state, AP=symbol_copy, AP_keys=symbol_keys,
                                              truth=truth_table_all[0], avoid=avoid_regions_all[0],
                                              all_truth=truth_table_all, all_avoid=avoid_regions_all,
                                              counter=0, avoid_self_loop=avoid_current_state)
            else:
                state_skip = re.findall(state + r':\n\tskip\n', output, re.DOTALL)
                if state_skip:
                    avoid_regions = {}
                    avoid_current_state = {}
                    for i in range(self.number_of_robots):
                        avoid_regions[i] = []
                    self.buchi_graph.add_edge(state, state, AP='1', AP_keys=[], truth='1',
                                              avoid=avoid_regions, avoid_self_loop=avoid_current_state)

    def get_truth_assignment(self, symbol):
        """
        get one set of truth assignment that makes the symbol true
        :param symbol: logical expression which controls the transition
        :rrn: a set of truth assignment enables the symbol
        """
        # empty symbol
        if symbol == '(1)':
            return '1', 1
        # non-empty symbol
        else:
            exp = symbol.replace('||', '|').replace('&&', '&').replace('!', '~')
            # add extra constraints: a single robot can reside in at most one region
            if '|' not in exp:
                robot_region = self.robot2region(exp)
                for robot, region in robot_region.items():
                    mutual_execlusion = list(combinations(region, 2))
                    # single label in the symbol
                    if not mutual_execlusion: continue
                    for i in range(len(mutual_execlusion)):
                        mutual_execlusion[i] = '(~(' + ' & '.join(list(mutual_execlusion[i])) + '))'
                    exp = '(' + exp + ') & ' + ' & '.join(mutual_execlusion)

                exp = parse_expr(exp)

                # find one truth assignment that makes symbol true using function satisfiable
                truth = satisfiable(exp)
                try:
                    truth_table = dict()
                    for key, value in truth.items():
                        truth_table[key.name] = value
                except AttributeError:
                    return False, 0
                else:
                    return [truth_table], 1
            # In case of or condition in logic multiple truth values may be possible
            else:
                robot_region = self.robot2region(exp)
                for robot, region in robot_region.items():
                    mutual_execlusion = list(combinations(region, 2))
                    # single label in the symbol
                    if not mutual_execlusion: continue
                    for i in range(len(mutual_execlusion)):
                        mutual_execlusion[i] = '(~(' + ' & '.join(list(mutual_execlusion[i])) + '))'
                    exp = '(' + exp + ') & ' + ' & '.join(mutual_execlusion)

                exp = parse_expr(exp)  # samarth change
                # find one truth assignment that makes symbol true using function satisfiable
                # truth = satisfiable(exp, algorithm="dpll")
                # truth_simple = satisfiable(exp)
                truth = satisfiable(exp, all_models=True)
                try:
                    all_truths = []
                    for i in truth:
                        truth_table = dict()
                        for key, value in i.items():
                            truth_table[key.name] = value
                        all_truths.append(truth_table)
                except AttributeError:
                    return False, 0
                else:
                    return all_truths, len(all_truths)

    def get_minimal_length(self):
        """
        search the shortest path from a node to another, i.e., # of transitions in the path
        :return:
        """
        # loop over pairs of buchi states
        for head_node in self.buchi_graph.nodes():
            for tail_node in self.buchi_graph.nodes():
                # head_node = tail_node, and tail_node is an accepting state
                if head_node != tail_node and 'accept' in tail_node:
                    try:
                        length, _ = nx.algorithms.single_source_dijkstra(self.buchi_graph,
                                                                         source=head_node, target=tail_node)
                    # couldn't find a path from head_node to tail_node
                    except nx.exception.NetworkXNoPath:
                        length = np.inf
                    self.min_length[(head_node, tail_node)] = length
                # head_node != tail_node and tail_node is an accepting state
                # move 1 step forward to all reachable states of head_node then calculate the minimal length
                elif head_node == tail_node and 'accept' in tail_node:
                    length = np.inf
                    for suc in self.buchi_graph.succ[head_node]:
                        try:
                            len1, _ = nx.algorithms.single_source_dijkstra(self.buchi_graph,
                                                                           source=suc, target=tail_node)
                        except nx.exception.NetworkXNoPath:
                            len1 = np.inf
                        if len1 < length:
                            length = len1 + 1
                    self.min_length[(head_node, tail_node)] = length

    def get_feasible_accepting_state(self):
        """
        get feasbile accepting/final state, or check whether an accepting state is feaasible
        :return:
        """
        accept = self.buchi_graph.graph['accept']
        self.buchi_graph.graph['accept'] = []
        for ac in accept:
            for init in self.buchi_graph.graph['init']:
                if self.min_length[(init, ac)] < np.inf and self.min_length[(ac, ac)] < np.inf:
                    self.buchi_graph.graph['accept'].append(ac)
                    break

    def robot2region(self, symbol):
        """
        pair of robot and corresponding regions in the expression
        :param symbol: logical expression
        :return: robot index : regions
        eg: input:  exp = 'l1_1 & l3_1 & l4_1 & l4_6 | l3_4 & l5_6'
            output: {1: ['l1_1', 'l3_1', 'l4_1'], 4: ['l3_4'], 6: ['l4_6', 'l5_6']}
        """

        robot_region = dict()
        for r in range(self.number_of_robots):
            findall = re.findall(r'(l\d+?_{0})[^0-9]'.format(r + 1), symbol) \
                      + re.findall(r'(m\d+?_{0})[^0-9]'.format(r + 1), symbol) \
                      + re.findall(r'(m\d+?r\d+?_{0})[^0-9]'.format(r + 1), symbol)
            if findall:
                findall = list(dict.fromkeys(findall))  # remove duplicates from list
                robot_region[str(r + 1)] = findall

        return robot_region

    def update_alternate_transition(self, state, next_state):
        """
        updates the truth to next alternate possiblity
        that enables transition from state to next_state.
        :param state: current state
        :param next_state: next state
        :return: success : true if success in finding alternate transition
        """
        edge_info = self.buchi_graph.edges[state, next_state]
        next_counter = edge_info['counter'] + 1
        possible_truths = len(edge_info['all_truth'])
        # if all possible transitions have been used then return false
        if next_counter >= possible_truths:
            return False

        self.buchi_graph.edges[state, next_state]['truth'] = edge_info['all_truth'][next_counter]
        self.buchi_graph.edges[state, next_state]['avoid'] = edge_info['all_avoid'][next_counter]
        self.buchi_graph.edges[state, next_state]['counter'] = next_counter
        return True

    def previous_alternate_transition(self, state, next_state):
        """
        updates the truth to previous alternate possiblity
        that enables transition from state to next_state.
        :param state: current state
        :param next_state: next state
        :return: success : true if success in finding alternate transition
        """
        edge_info = self.buchi_graph.edges[state, next_state]
        next_counter = edge_info['counter'] - 1
        # if all possible transitions have been used then return false
        if next_counter < 0:
            return False

        self.buchi_graph.edges[state, next_state]['truth'] = edge_info['all_truth'][next_counter]
        self.buchi_graph.edges[state, next_state]['avoid'] = edge_info['all_avoid'][next_counter]
        self.buchi_graph.edges[state, next_state]['counter'] = next_counter
        return True

    def ctr_alternate_transition(self, state, next_state, next_counter):
        """
        updates the truth to alternate possiblity pointed by next_counter
        that enables transition from state to next_state.
        :param state: current state
        :param next_state: next state
        :param next_counter: will set counter to this value
        :return: success : true if success in finding alternate transition
        """
        edge_info = self.buchi_graph.edges[state, next_state]
        possible_truths = len(edge_info['all_truth'])
        # if all possible transitions have been used then return false
        if next_counter < 0 and next_counter >= possible_truths:
            return False

        self.buchi_graph.edges[state, next_state]['truth'] = edge_info['all_truth'][next_counter]
        self.buchi_graph.edges[state, next_state]['avoid'] = edge_info['all_avoid'][next_counter]
        self.buchi_graph.edges[state, next_state]['counter'] = next_counter
        return True

    def delete_transition(self, state, next_state):
        """
        deletes edge.
        :param state: current state
        :param next_state: next state
        Throws error if deleting edge results in no possible paths from state to accepting_state
        """
        self.buchi_graph.remove_edge(state, next_state)
        self.get_minimal_length()
        self.get_feasible_accepting_state()
        if not self.buchi_graph.graph['accept']:
            # raise Exception("No more transitions to accepting state")
            return True
        final_state = self.buchi_graph.graph['accept'][0]
        if self.min_length[state, final_state] == np.inf:
            # raise Exception("No more transitions to accepting state")
            return True

    def get_next_NBA_state(self, curr_state, accepting_state):
        """
        returns next state closest to accepting/final state.
        :param curr_state: current state in graph
        :param accepting_state: current state in graph
        :return: next_State : state closest to accepting/final state
        """
        # print(nx.shortest_path(self.buchi_graph, curr_state, accepting_state))
        return nx.shortest_path(self.buchi_graph, curr_state, accepting_state)[1]

    def get_next_action(self, curr_state, accepting_state):
        """
        returns the truth value or action that enables the transition between states.
        :param curr_state: current state in graph
        :param accepting_state: current state in graph
        :return: next_State : target state
        """
        # print("buchi_graph edges",self.buchi_graph.edges)
        return self.buchi_graph.edges[curr_state, accepting_state]['truth']