from transitions import Machine

class ControllerFSM():

    def __init__(self):
        states = [
            {'name': 'landed', 'on_enter': ['print_state']},
            {'name': 'taking_off', 'on_enter': ['print_state']},
            {'name': 'hovering', 'on_enter': ['print_state']},
            {'name': 'flying', 'on_enter': ['print_state']},
            {'name': 'landing', 'on_enter': ['print_state']},
            {'name': 'racing', 'on_enter': ['print_state']},
        ]
        transitions = [
            {'trigger': 'takeoff',          'source': 'landed',                         'dest': 'taking_off'},
            {'trigger': 'launch',           'source': 'landed',                         'dest': 'hovering'},
            {'trigger': 'in_position',      'source': 'taking_off',                     'dest': 'hovering'},
            {'trigger': 'land',             'source': 'hovering',                       'dest': 'landing'},
            {'trigger': 'landing_complete', 'source': 'landing',                        'dest': 'landed'},
            {'trigger': 'move',             'source': 'hovering',                       'dest': 'flying'},
            {'trigger': 'stop',             'source': ['flying', 'racing', 'hovering'], 'dest': 'hovering'},
            {'trigger': 'race',             'source': ['hovering', 'landed'],           'dest': 'racing'},
        ]
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='landed')

    def print_state(self):
        print(f'Entering state: {self.state}')
