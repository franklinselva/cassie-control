#include <iostream>
#include <thread>
#include <chrono>
#include <GLFW/glfw3.h>
#include "mujoco/mujoco.h"
#include "render.hpp"

static std::mutex mtx;

int main(int argc, const char **argv)
{
    // Load the MuJoCo model
    mjModel *model = mj_loadXML("/home/shasthamsa/cassie-control/assets/cassie.xml", NULL, NULL, 0);
    static mjData *model_data;
    if (!model)
    {
        std::cerr << "Could not load model" << std::endl;
        return 1;
    }

    // Call the render function
    std::thread rendering_thread(render, model, model_data, std::ref(mtx));

    // Wait for the rendering thread to finish
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Amount of time to sleep to keep the simulation roughly
    // realtime
    float slowdown = 1;
    int ts = static_cast<int>(slowdown * 1e6 * model->opt.timestep);
    std::chrono::microseconds timestep(ts);

    while (true)
    {
        auto start_time = std::chrono::steady_clock::now();

        std::this_thread::sleep_until(start_time + timestep);
    }

    // Wait for the rendering thread to finish
    rendering_thread.join();

    // Free the MuJoCo model
    mj_deleteModel(model);

    // Terminate GLFW
    glfwTerminate();

    return 0;
}