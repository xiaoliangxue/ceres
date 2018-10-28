//
// Created by chen on 18-7-19.
//

#ifndef LOCALIZER_PARTICLE_HPP
#define LOCALIZER_PARTICLE_HPP

#include <global.hpp>

class Particle
{
public:
    Particle(int idx = 0, double x = 0.0, double y = 0.0, double heading = 0.0, double weight = 0.01):
        m_nIdx{idx},
        m_evPose{x, y, heading},
        m_fWeight{weight},
        m_fInlierRatio{0.0}
    {

    }

    ~Particle()
    {

    }

    /**@brief forward predict the particle;
     *
     * forward predict the particle by accumulating velocity;
     * @param delta_t [IN]: delta time;
     * @param vel_x [IN]: east velocity;
     * @param vel_y [IN]: north velocity;
     * @param yaw_d [IN]: yaw angular velocity;
     * @param std_var_vel_x [IN]: east velocity standard variation;
     * @param std_var_vel_y [IN]: north velocity standard variation;
     */
    void Prediction(double delta_t, double vel_x, double vel_y, double yaw_d, double std_var_vel_x, double std_var_vel_y, double std_var_yaw_d)
    {
        static std::default_random_engine random_engine;
        std::normal_distribution<double> dist_vel_x(vel_x, std_var_vel_x);
        std::normal_distribution<double> dist_vel_y(vel_y, std_var_vel_y);
        std::normal_distribution<double> dist_yaw_d(yaw_d, std_var_yaw_d);

        m_evPose(0) += dist_vel_x(random_engine)*delta_t;
        m_evPose(1) += dist_vel_y(random_engine)*delta_t;
        m_evPose(2) += dist_yaw_d(random_engine)*delta_t;
    }

    void CTRVPrediction(double delta_t, double vel, double yaw_d, double std_var_vel, double std_var_yaw_d)
    {
        static std::default_random_engine random_engine;
        std::normal_distribution<double> dist_vel(vel, std_var_vel);
        std::normal_distribution<double> dist_yaw_d(yaw_d, std_var_yaw_d);

        double vel_ = dist_vel(random_engine);
        double yaw_d_ = dist_yaw_d(random_engine);

        double delta_x = 0.0;
        double delta_y = 0.0;
        double last_yaw = m_evPose(2);
        double new_yaw = last_yaw + yaw_d_*delta_t;
        if (fabs(yaw_d_) > 1e-3)
        {
            delta_x = vel_ * (sin(new_yaw) - sin(last_yaw)) / yaw_d_;
            delta_y = vel_ * (-cos(new_yaw) + cos(last_yaw)) / yaw_d_;
        }
        else
        {
            delta_x = vel_ * cos(last_yaw) * delta_t;
            delta_y = vel_ * sin(last_yaw) * delta_t;
        }

        m_evPose(0) += delta_x;
        m_evPose(1) += delta_y;

        NormalizeYaw(new_yaw);
        m_evPose(2) = new_yaw;
    }

    static bool CompareByWeight(const Particle &p1, const Particle &p2)
    {
        return p1.m_fWeight > p2.m_fWeight;
    }

    void NormalizeYaw(double &yaw)
    {
        if (yaw < 0.0)
        {
            yaw = PI_DOUBLE + yaw;
        }

        if (yaw > PI_DOUBLE)
        {
            yaw -= PI_DOUBLE;
        }
    }
public:
    int m_nIdx;

    Eigen::Vector3d m_evPose;

    double m_fWeight;

    double m_fInlierRatio;//ratio of inliers;
};


#endif //LOCALIZER_PARTICLE_HPP
