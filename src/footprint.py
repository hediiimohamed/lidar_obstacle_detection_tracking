import matplotlib.pyplot as plt

# Points from the last polygon in your data
points_x = [
    18.305936813354492, 17.840513229370117, 16.906570434570312, 
    15.906705856323242, 15.810826301574707, 15.519956588745117, 
    15.253161430358887, 15.544031143188477, 15.523187637329102, 
    16.46851348876953,  16.281429290771484, 16.746850967407227, 
    17.17333984375,     17.55685806274414,  18.305936813354492 # Repeat first point to close
]
points_y = [
    -3.0919275283813477, -2.69145131111145,   -3.792337417602539, 
    -3.3337831497192383, -3.5428457260131836, -3.4094481468200684, 
    -3.99118709564209,   -4.124584674835205,  -4.170032978057861, 
    -4.603575229644775,  -6.090988636016846,  -6.491465091705322, 
    -4.937821865081787,  -4.101571559906006,  -3.0919275283813477 # Repeat first point to close
]

# Create the plot
plt.figure(figsize=(8, 8)) # Adjust figure size as needed
plt.fill(points_x, points_y, 'lightblue', edgecolor='blue', linewidth=1.5, alpha=0.7) # Fill the polygon
plt.plot(points_x, points_y, 'bo-') # Plot vertices and edges

# Add labels and title
plt.xlabel("X coordinate (odom frame)")
plt.ylabel("Y coordinate (odom frame)")
plt.title("Robot Footprint (Timestamp: 1743758559.392)")
plt.grid(True)
plt.axis('equal') # Ensure aspect ratio is correct

# Show the plot
plt.show()
