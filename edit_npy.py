import numpy as np

n = 30
np_arr = np.ones((n, n), dtype=int)

np.save(f"maps/maps_{n}x{n}", np_arr)