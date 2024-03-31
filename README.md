# Motion-Planner

This repository offers a reimplementation of classical motion planning algorithms, focusing on navigating through constrained 2D and 3D spaces. By assuming the availability of a vector map detailing obstacles and free spaces, this project aims to solve the fundamental problem of finding feasible paths from start to goal locations while avoiding collisions. 

The project currently includes an implementation of the Rapidly-exploring Random Tree (RRT) algorithm in a 2D environment, encapsulated within a C++ script (`rrt_2d.cpp`). Additionally, a Python script (`visualization.py`) is provided to visually demonstrate the paths generated by the algorithm amidst the defined obstacles. This algorithm is used in the course assignment of [CS 393R Autonomous Robots](https://amrl.cs.utexas.edu/CS393R-S24/).

## Requirements

- C++ Compiler (C++11 or newer)
- Python 3
  - matplotlib (for visualization)

## Quick Start Example

To quickly see the motion planner in action, follow these steps after setting up:

1. **Compile the RRT Algorithm:** Ensure you are in the project's root directory and compile the C++ code:

```g++ -std=c++11 -o main rrt_2d.cpp```

2. **Run the Compiled Program:** Execute the compiled binary to generate the path data:

```./main```

This step will create output files (final_path.txt, tree_structure.txt) used by the visualization script and print a message like "Total obstacles loaded: XXXX" and "Goal reached." upon success.

3. **Visualize the Results:** With the output files generated, run the visualization script:

```python visualization.py```

If everything is set up correctly, the Python script will display a visualization of the generated path and obstacles.


![GDC3-1](example_results/GDC3_stepSize=1.0.png "Path Planning under Constraints using RRT Algorithm (stepSize = 1.0)")


![GDC3-2](example_results/GDC3_stepSize=0.2.png "Path Planning under Constraints using RRT Algorithm (stepSize = 0.2)")
