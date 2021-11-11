from multi_MBA.buchi_parse import Buchi
from multi_MBA.task import Task


def run_test():
    task = Task()
    buchi = Buchi(task)
    buchi.construct_buchi_graph()
    buchi.get_minimal_length()
    buchi.get_feasible_accepting_state()

    '''
    Initialization:
        acceptingNBAState = buchi.buchi_graph.graph['accept'][0]
        nextNBAState = buchi.get_next_NBA_state(currentNBAState, acceptingNBAState)
        nextAction = buchi.get_next_action(currentNBAState, nextNBAState)
        
    Give a subtask:
        nextAction = buchi.get_next_action(currentNBAState, nextNBAState)
    
    If reachSDP says that you cannot safely accomplish it
          flagAlternate = buchi.update_alternate_transition(currentNBAState, nextNBAState)
          If flagAlternate=False
             buchi.delete_transition(currentNBAState, nextNBAState)
          endif
         nextAction = buchi.get_next_action(currentNBAState, nextNBAState)
    else
        apply your controller to accomplish the subtask.
        once your subtask is done (i.e., you reach a region) do: 
            currentNBAState = nextNBAState
    
    Success: currentNBAState is the same as "acceptingNBAState"
    '''
    flagFeasible = 1
    currentNBAState = "T0_init"
    acceptingNBAState = buchi.buchi_graph.graph['accept'][0]
    print(acceptingNBAState)
    nextNBAState = buchi.get_next_NBA_state(currentNBAState, acceptingNBAState)
    nextAction = buchi.get_next_action(currentNBAState, nextNBAState)
    print("\ncurrentNBAState: ", currentNBAState)
    print("nextNBAState: ", nextNBAState)
    print("nextAction: ", nextAction, type(nextAction))

    # Suppose above action cannot be taken i.e. if we get flagFeasible=0. We will select the alternative action
    flagAlternate = buchi.update_alternate_transition(currentNBAState, nextNBAState)  # update alternate action
    nextAction = buchi.get_next_action(currentNBAState, nextNBAState)
    print("\nAfter updating action... \ncurrentNBAState: ", currentNBAState)
    print("nextNBAState: ", nextNBAState)
    print("nextAction: ", nextAction)

    # Suppose the action is again infeasible, i.e. flafFeasible = 0
    flagAlternate = buchi.update_alternate_transition(currentNBAState, nextNBAState)
    print("flagAlternate: -> ", flagAlternate)
    # flagAlternate will be False as both actions are not possible and thus we delete this edge in the NBA
    buchi.delete_transition(currentNBAState, nextNBAState)

    nextNBAState = buchi.get_next_NBA_state(currentNBAState, acceptingNBAState)
    nextAction = buchi.get_next_action(currentNBAState, nextNBAState)
    print("\nT0_init to accept_all edge is removed from NBA")
    print("currentNBAState: ", currentNBAState)
    print("nextNBAState: ", nextNBAState)
    print("nextAction: ", nextAction)

    currentNBAState = nextNBAState
    nextNBAState = buchi.get_next_NBA_state(currentNBAState, acceptingNBAState)
    nextAction = buchi.get_next_action(currentNBAState, nextNBAState)
    print("\ncurrentNBAState is updated")
    print("currentNBAState: ", currentNBAState)
    print("nextNBAState: ", nextNBAState)
    print("nextAction: ", nextAction)


if __name__ == "__main__":
    run_test()
