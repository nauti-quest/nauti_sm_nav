
class State(object):
    def execute(self):
        """Perform state actions."""
        raise NotImplementedError

    def check_transition(self):
        """Determine if a state transition is needed.
           Return self if no transition, or a new State instance otherwise."""
        raise NotImplementedError