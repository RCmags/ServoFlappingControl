import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from math import pi as PI

# 1. truncated wave, tri wave, partial truncation
plt.plot( [0, 0.25, 0.5, 0.75, 1], [0, 1, 0, -1, 0], c='black', label='0%' ) 
plt.plot( [0, 0.125, 0.375, 0.5, 0.625, 0.875, 1], [0, 1, 1, 0, -1,-1, 0], c='blue', linestyle='--', label='50%' )
plt.plot( [0, 0, 0.5, 0.5, 1, 1], [0,1, 1, -1,-1,0], c='red', label='100%') 
	# format
plt.title("triangle wave - truncation")
plt.xlabel("time (s)")
plt.ylabel("waveform")
plt.yticks( [-1, -0.5, 0, 0.5, 1] )
plt.xticks( [0, 0.25, 0.5, 0.75, 1] )
plt.legend()

# 2. frequency modulation, assymetric
plt.figure()
plt.plot( [0, 0.25, 0.5, 0.75, 1], [0, 1, 0, -1, 0], c='black', label='0%' ) 
plt.plot( [0, 0.125, 0.5, 0.875, 1], [0, 1, 0,-1, 0], c='blue', linestyle='--', label='50%' ) 
plt.plot( [0, 0.375, 0.5, 0.625, 1], [0, 1, 0,-1, 0], c='red', linestyle='--', label='-50%' ) 
	# format
plt.title("triangle wave - frequency modulation")
plt.xlabel("time (s)")
plt.ylabel("waveform")
plt.yticks( [-1, -0.5, 0, 0.5, 1] )
plt.xticks( [0, 0.25, 0.5, 0.75, 1] )
plt.legend()

plt.show()
