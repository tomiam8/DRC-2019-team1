import numpy as np
import matplotlib.pyplot as plt

coefficients = np.polynomial.polynomial.Polynomial.fit([1,2],[2,-3],1)
coefficients = np.polyfit([1,2],[2,-3],1)
print(coefficients)
x_values = []
y_values = []
step = 10
lower = -1
upper = 3
for x in [p/step for p in range(lower*step, upper*step+1, 1)]:
    x_values.append(x)
    temp_sum = 0
    for i in range(len(list(coefficients))):
        temp_sum += list(coefficients)[i]*(x**(len(coefficients)-i-1))
    y_values.append(temp_sum)
print(x_values)
print(y_values)
plt.plot(x_values, y_values)
plt.show()
