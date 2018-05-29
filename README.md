# CarND-Controls-MPC
Here you can find my solution to the CarND Model Predictive Controller project.

This MPC project was the task #5 of the [Udacity Self-Driving Car Engineer Nanodegree program](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013).
For details, see [article by Nick Hortovanyi](https://medium.com/@NickHortovanyi/carnd-controls-mpc-2f456ce658f).

The original sources are available here: [https://github.com/udacity/CarND-MPC-Project].

On video, you can see the algorithm running with 90mph speed limit:

[![Youtube video](http://img.youtube.com/vi/Jhlxhii8HoU/0.jpg)](https://www.youtube.com/watch?v=Jhlxhii8HoU)

## Build Instructions for Linux

1. Clone this repo.

2. Install required packages:
```
    sudo apt-get install cppad libuv1-dev libssl-dev nlohmann-json-dev
```

3. Install [uWebSockets](https://github.com/uWebSockets/uWebSockets)
```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
```

4. Compile:
```
    cd src
    make
```

## Run The Simulator

1. Install the simulator from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

2. Run the control unit: `./mpc`.

3. Run the simulator: `./term2_sim.x86_64`. Press OK to set window size. Press Previous button to select 'Project 5: MPC Controller' task. Press SELECT to start.
