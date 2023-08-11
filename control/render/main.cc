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
    const char *model_path = "assets/cassie.xml";
    mjModel *model = mj_loadXML(model_path, NULL, NULL, 0);
    static mjData *model_data = mj_makeData(model);

    double qpos_init[] = {
        0.0045, 0, 0.4973, 0.9785, -0.0164, 0.0178, -0.2049,
        -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968,
        -0.0045, 0, 0.4973, 0.9786, 0.0038, -0.0152, -0.2051,
        -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968};

    mju_copy(&model_data->qpos[7], qpos_init, 28);
    mj_forward(model, model_data);

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