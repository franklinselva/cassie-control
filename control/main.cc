#include "render/render.hpp"

#include "mujoco/mujoco.h"
#include <Eigen/Dense>

#include <iostream>
#include <thread>
#include <chrono>
#include <utility>

// Macros
typedef Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>> mjMapVector_t;
typedef Eigen::Map<
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
    mjMapMatrix_t;

class MujocoControl
{

public:
    MujocoControl(mjModel *model, mjData *data, std::mutex *mutex)
        : model_(model), data_(data), mutex_(mutex) {}
    ~MujocoControl() {}

    /*
     * Some joints are "independent" in the sense that they are not actuated
     * (e.g. floating base, free joints, springs)
     */
    std::pair<Eigen::VectorXi, Eigen::VectorXi> split_idcs()
    {
        Eigen::VectorXi ind_idx(model_->nu);
        Eigen::VectorXi dep_idx(model_->nv - model_->nu);

        int i = 0, d = 0;
        for (int dof = 0; dof < model_->nv; dof++)
        {
            int jnt = model_->dof_jntid[dof];
            auto name = mj_id2name(model_, mjOBJ_JOINT, jnt);

            // TODO?: Assumes actuator names correspond to joint names
            // Could use trnid instead

            // Joint is independent (floating base || actuated || spring)
            if (name == nullptr || mj_name2id(model_, mjOBJ_ACTUATOR, name) != -1 || model_->jnt_stiffness[jnt] > 0)
                ind_idx[i++] = dof;

            else
                dep_idx[d++] = dof;
        }

        return std::make_pair(ind_idx, dep_idx);
    }

    Eigen::MatrixXd getSubmatrixByCols(const Eigen::VectorXi &col_idcs, const Eigen::MatrixXd &mat)
    {
        // TODO: avoid copies?
        Eigen::MatrixXd submat = Eigen::MatrixXd::Zero(mat.rows(), col_idcs.size());

        for (int i = 0; i < col_idcs.size(); i++)
            submat.col(i) = mat.col(col_idcs[i]);

        return submat;
    }

    Eigen::MatrixXd getInertiaMatrix()
    {
        mjtNum H[model_->nv * model_->nv];

        std::lock_guard<std::mutex> lock(*mutex_);
        mj_fullM(model_, H, data_->qM);

        Eigen::MatrixXd M = mjMapMatrix_t(H, model_->nv, model_->nv);
        return M;
    }

    Eigen::VectorXd getBiasVector()
    {
        Eigen::VectorXd c;

        std::lock_guard<std::mutex> lock(*mutex_);
        mj_forward(model_, data_);
        c = mjMapVector_t(data_->qfrc_bias, model_->nv);

        return c;
    }

    // solves [qd_ind qd_dep] = gamma * qd_ind
    Eigen::MatrixXd getConstraintProjectionMatrix(const Eigen::MatrixXd &G)
    {
        Eigen::VectorXi ind_idx, dep_idx;
        std::tie(ind_idx, dep_idx) = this->split_idcs();

        auto G_ind = getSubmatrixByCols(ind_idx, G);
        auto G_dep = getSubmatrixByCols(dep_idx, G);

        auto K = -G_dep.inverse() * G_ind;

        Eigen::MatrixXd gamma = Eigen::MatrixXd::Identity(model_->nv, K.cols());
        gamma.bottomRows(K.rows()) = K;

        Eigen::VectorXi order(ind_idx.size() + dep_idx.size());
        order << ind_idx, dep_idx;

        Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> P(order);

        return P * gamma; // Permute gamma so it's in original q order
    }

    Eigen::MatrixXd getConstraintJacobian()
    {
        std::lock_guard<std::mutex> lock(*mutex_);

        Eigen::MatrixXd G = mjMapMatrix_t(data_->efc_J, model_->neq * 3, model_->nv);

        // Zero out negligible entries for numerical stability
        G = (G.array().abs() < 1e-4).select(0, G);

        return G;
    }

    // For a rank deficient matrix M this function returns a
    // matrix R that is row equivalent to M and has full rank
    Eigen::MatrixXd getFullRankRowEquivalent(const Eigen::MatrixXd &M)
    {
        // TODO: investigate using QR decomp

        // Get M = P^(-1)LUQ^(-1) decomposition
        Eigen::FullPivLU<Eigen::MatrixXd> lu(M);

        // Extract R = UQ^(-1), which is row equivalent to M
        Eigen::MatrixXd U = lu.matrixLU().triangularView<Eigen::Upper>();
        Eigen::MatrixXd R = U * lu.permutationQ().inverse();

        // Note: U is rank revealing (all rows are linearly independent or 0)
        // and also sorted so the "smallest" rows are last

        // Get rid of zero rows so U is no longer rank deficient
        // Since U is sorted we only need to check bottom (m - rank(U)) rows
        // Calling lu.rank() directly won't account for numerical precision
        int rank_estimate = R.rows();
        while (R.row(rank_estimate - 1).isZero(1e-5))
            rank_estimate -= 1;

        return R.block(0, 0, rank_estimate, R.cols());
    }

    Eigen::MatrixXd constrainedInverseDynamics(const Eigen::VectorXd &v_dot)
    {
        auto G = getConstraintJacobian();
        auto R = getFullRankRowEquivalent(G);

        auto gamma = getConstraintProjectionMatrix(R);
        auto M = getInertiaMatrix();
        auto c = getBiasVector();

        auto tau = M * v_dot + c;
        auto u = gamma.transpose() * tau;

        return u;
    }

private:
    mjModel *model_;
    mjData *data_;
    std::mutex *mutex_;
};

int main()
{
    static std::mutex mtx;

    // Spawn model and data
    const char *model_path = "assets/scene.xml";
    mjModel *model = mj_loadXML(model_path, NULL, NULL, 0);
    static mjData *model_data = mj_makeData(model);

    if (!model)
    {
        std::cerr << "Could not load model" << std::endl;
        return 1;
    }

    // Initialize joint positions
    double qpos_init[] = {
        0.0045, 0, 0.4973, 0.9785, -0.0164, 0.0178, -0.2049,
        -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968,
        -0.0045, 0, 0.4973, 0.9786, 0.0038, -0.0152, -0.2051,
        -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968};
    mju_copy(&model_data->qpos[7], qpos_init, 28);
    mjMapVector_t q_targ(model_data->qpos, model->nq);

    // Initialize joint velocities
    mj_forward(model, model_data);

    std::thread render_thread(render, model, model_data, std::ref(mtx));

    // Give some time for the rendering thread to initialize
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Amount of time to sleep to keep the simulation roughly
    // realtime
    float slowdown = 1;
    int ts = static_cast<int>(slowdown * 1e6 * model->opt.timestep);
    std::chrono::microseconds timestep(ts);

    // Mujoco control input
    mjMapVector_t u(model_data->ctrl, model->nu);

    // Initialize Mujoco Control
    MujocoControl mujoco_control(model, model_data, &mtx);

    while (true)
    {
        auto start_time = std::chrono::steady_clock::now();

        // Joint space stiffness (glorified PD control)
        std::lock_guard<std::mutex> lock(mtx);
        mjMapVector_t q(model_data->qpos, model->nq);
        mjMapVector_t qd(model_data->qvel, model->nv);
        Eigen::VectorXd v_dot = 500 * (q_targ - q) + 10 * -qd;
        // std::cout << (q_targ - q).norm() << std::endl;

        Eigen::MatrixXd tau = mujoco_control.constrainedInverseDynamics(v_dot);
        mj_step(model, model_data);
        u = tau;

        std::cout << tau.transpose() << std::endl;

        std::this_thread::sleep_until(start_time + timestep);
    }

    render_thread.join();

    mj_deleteData(model_data);
    mj_deleteModel(model);
}