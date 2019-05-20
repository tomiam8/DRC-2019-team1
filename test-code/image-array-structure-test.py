import numpy as np
import cv2
import matplotlib.pyplot as plt

img = cv2.imread('pixel-coords-wide.png')
img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
hsv_img = cv2.inRange(img, (90, 250, 200), (110, 255, 210))

print(img)
print(hsv_img)
non_zero_pixels = np.transpose(np.nonzero(hsv_img))
print("#"*8)
print(non_zero_pixels)
#reshaped = np.reshape(non_zero_pixels, (1,3))
reshaped = np.fliplr(non_zero_pixels)
#first_col = np.unique(reshaped[:,0])
x_values  = reshaped[:,0]
y_values = reshaped[:,1]
'''y_values = [[] for i in range(len(first_col))]
for i in range(len(first_col)):
    for y in non_zero_pixels:
        if y[1]==first_col[i]:
            y_values[i].append(y[0])'''

coefficients = np.polyfit(y_values, x_values, 2)
print(x_values)
print(y_values)
print(coefficients)
plt.imshow(hsv_img)

#Plot a bunch of values to see the polyfitting
x_values = []
y_values = []
step = 10
lower = 0
upper = len(img)
for x in [p/step for p in range(lower*step, upper*step+1, 1)]:
    x_values.append(x)
    temp_sum = 0
    for i in range(len(coefficients)):
        temp_sum += list(coefficients)[i]*(x**(len(coefficients)-i-1))
    y_values.append(temp_sum)
#print([x for x in coefficients])
#plt.plot([x for x in range(1, len(hsv_img))], [sum([list(coefficients)[i]*(x**i) for i in range(len(coefficients))]) for x in range(1, len(hsv_img))])
plt.imshow(img)
plt.plot(y_values, x_values)
plt.show()
#cv2.imshow('image',img)
#cv2.waitKey(0)
cv2.destroyAllWindows()
