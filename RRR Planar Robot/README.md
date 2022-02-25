## Introduction
A three-link robot arm trying to reach a randomized goal.

## Setup and Run

### Known Good Platforms

| OS     | Versions |
|--------|----------|
| Ubuntu | 18.04    |


### Setup the Development Environment

```
$ ./scripts/bootstrap
```

### Enter the virtual environment

```
$ source ./scripts/init
```

### Run

```
$ python challenge.py
```


## Backlog

* Add borders and wall to the world
  * Bonus: make a more easily configurable representation
* Add a third link to the arm
* Make a moving goal
* Improve the visualization implementation
* Visualize Paths, Velocities, Accelerations, over time in the sandbox window
  * Note: Not as good to render several separate visualizations
* "Cloudify" robot's configuration
* Use [py-trees](https://py-trees.readthedocs.io/en/devel/) to make complex behaviors
* Add unit tests and clean up the code (note: don't just do this item!)
* Change system from top down to on the wall - add gravity!
  * This will require changing robot to use torque control
* Replace the controller with a learned policy
