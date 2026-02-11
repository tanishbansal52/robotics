## Our findings

# 1
Below 10cm, it starts to get less reliable, and below 5cm, values stop decreasing and we get incorrect measurements.
After 220cm, we get incorrect measurements.

# 2
We tested with a radius of 50cm, with 90° being perpendicular to the wall and 0° being parallel to the wall.
At 90° = 50cm
At 85° = 50cm
At 80° = 49cm
At 70° = 48cm
At 67.5° = 46cm
At 45° = 43cm
At 22.5° = 29cm
Additionally, we also tested with a radius of 90cm to have more datapoints by controlling our radius variable:
At 90° = 100cm
At 85° = 100cm
At 80° = 98cm
At 70° = 95cm
The maximum angular deviation from perpendicular to the wall at which it will still give sensible readings is 85°. There is also some human error.

# 3
True | Reported
20cm = 20 
40cm = 40
60cm = 60
80cm = 80
100cm = 100

# 4
True | Reported
40cm = 40, 40, 39, 40, 40, 39, 41, 40, 40, 41
100cm = 99, 100, 100, 99, 99, 102, 99, 101, 99, 100  

# 5
Within 10cm and 220cm, and a 5-10 degree range on either side, and ensuring there are no obstructions in the way of the sonar sensor, we get accurate readings about 98% of the time.
Outside of 10-220cm, we get accurate readings 30% (varies a lot) of the time.
Outside of the 5-10 degree range, we get increasingly inaccurate readings as the angle increases; so about ~5% of readings are close enough to the ground truth but otherwise readings are garbage.
