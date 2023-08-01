# Cassie Control


Initial part of the code is adapted from [Cassie IK/ID](https://github.com/p-morais/cassie-ik-id/tree/master).
This repository contains the code for inverse kinematics and inverse dynamics of a Cassie robot, mainly using MuJoCo and Eigen.

### Installation

To setup the project, you will need to install Bazel in your system. You can find instructions [here](https://docs.bazel.build/versions/master/install.html).

After installing Bazel, you will need to install the dependencies and build the workspace. You can do this by running the following command:

```bash
bazel build --enable_bzlmod control:main
```
Bazel will automatically download Mujoco and build Eigen for you.

### Running the code

To run the code, you can use the following command:

```bash
bazel run --enable_bzlmod control:main
```
