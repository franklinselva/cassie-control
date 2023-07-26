# Cassie Control


Credit to [Cassie IK/ID](https://github.com/p-morais/cassie-ik-id/tree/master) for the most part of code.
This repository contains the code for inverse kinematics and inverse dynamics of a Cassie robot, mainly using MuJoCo and Eigen.

### Installation

To setup the project, you will need to install Bazel in your system. You can find instructions [here](https://docs.bazel.build/versions/master/install.html).

After installing Bazel, you will need to install the dependencies. You can do this by running the following command:

```bash
bazel build --enable_bzlmod src/...
```
Bazel will automatically download Mujoco and build Eigen for you.

### Running the code

To run the code, you can use the following command:

```bash
bazel run src:main
```
