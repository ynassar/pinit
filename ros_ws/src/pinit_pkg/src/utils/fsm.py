#!/usr/bin/env python

class FSM():
    class FSMTransition():

        def __init__(self, source, target, call_back):
            self.source = source
            self.target = target
            self.call_back = call_back


        def execute(self):
            try:
                self.call_back()
            except Exception:
                raise


    def __init__(self):
        self.transitions = []
        self.current_state = None
        self.states = []


    def add_state(self, state):
        if not self.state_exist(state):
            self.states.append(state)
            print "adding state {s}".format(s=state)


    def state_exist(self, state):
        exist = False
        if state in self.states:
            exist = True
        else:
            exist = False
        return exist


    def set_default(self, state):
        if self.state_exist(state):
            self.current_state = state
        else:
            print "State {s} doesn't exist".format(s=state)


    def go_to(self, target_state):
        transtition = self.get_transition(self.current_state, target_state)
        if transtition is not None:
            try:
                transtition.execute()
                self.current_state = target_state
            except Exception:
                raise
        else:
            print "Invalid transtion from {source} to {target}".format(
                source=self.current_state,
                target=target_state)


    def get_transition(self, source, target):
        result = None
        for transition in self.transitions:
            if transition.source == source and transition.target == target:
                result = transition
                break

        return result


    def add_transition(self, source, target, call_back):
        if self.get_transition(source, target) is None:
            transition = self.FSMTransition(source, target, call_back)
            self.transitions.append(transition)
        else:
            print "Transition already exists"


if __name__ == '__main__':
    state1 = 's1'
    state2 = 's2'
    test_fsm = FSM()

    def cb1():
        print "cb1"
    def cb2():
        print "cb2"

    test_fsm.add_state(state1)
    test_fsm.add_state(state2)
    test_fsm.set_default(state1)
    test_fsm.add_transition(state1, state2, cb1)
    test_fsm.go_to(state2)
    test_fsm.go_to(state1)
