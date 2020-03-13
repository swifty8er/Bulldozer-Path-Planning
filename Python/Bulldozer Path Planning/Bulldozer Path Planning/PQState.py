class PQState:
    def __init__(self,f,state,g):
        self._f = f
        self._state = state
        self._g = g

    def __lt__(self,pq_state):
        return self._f < pq_state.f

    @property
    def f(self):
        return self._f

    @property
    def state(self):
        return self._state

    @property
    def g(self):
        return self._g


