import random
import numpy as np

teams = [1, 4]
rand_team = np.random.choice(teams, replace=False)
print(rand_team)
