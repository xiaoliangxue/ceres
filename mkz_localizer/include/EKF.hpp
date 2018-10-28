//
// Created by chen on 18-7-27.
//

#ifndef LOCALIZER_EKF_HPP
#define LOCALIZER_EKF_HPP

#include <eigen3/Eigen/Dense>

class EKF
{
public:
public:
    EKF()
    {
    }

    /**@brief initialize the filter;
     *
     * @param timestamp [IN]: the timestamp of initialization;
     * @param px [IN]: the vehicle position x in engineer frame;
     * @param py [IN]: the vehicle position y in engineer frame;
     * @param velocity [IN]: the vehicle velocity;
     * @param yaw [IN]: the vehicle yaw angle;
     * @param yaw_vel [IN]: the vehicle yaw velocity;
     */
    void Initialize(double timestamp, double px, double py, double velocity, double yaw, double yaw_vel)
    {
        m_fTimeStamp = timestamp;

        m_evX.setZero();
        m_evX(0) = px;
        m_evX(1) = py;
        m_evX(2) = yaw;
        m_evX(3) = velocity;
        m_evX(4) = yaw_vel;

        m_emP.setZero();
        //inital state covariance;
        m_emP(0, 0) = 5.0 * 5.0;//gnss position variance, 5.0m;
        m_emP(1, 1) = 5.0 * 5.0;//gnss position variance, 5.0m;
        m_emP(2, 2) = 0.174 * 0.174;//yaw angle variance, 10deg;
        m_emP(3, 3) = 0.5 * 0.5;//velocity variance, 0.5m/s;
        m_emP(4, 4) = 0.017 * 0.017;//yaw angular velocity variance, 1deg/s;

        m_emQ.setZero();

        m_emR.setZero();

    }

    /**@brief model prediction;
     *
     * model prediction by velocity and yaw velocity;
     * @param timestamp [IN]: current timestamp;
     * @param velocity [IN]: velocity in m/s;
     * @param yaw_vel [IN]: yaw angular velocity, in rad/s;
     */
    void Predict(double timestamp, double yaw_vel)
    {
        m_mtxFilter.lock();
        double delta_time = timestamp - m_fTimeStamp;
        //set Q according to the delta time;
        m_emQ(0, 0) = pow(0.5 * 4.0 * delta_time * delta_time, 2.0);//maximum 8m/s2 acceleration;
        m_emQ(1, 1) = m_emQ(0, 0);
        m_emQ(2, 2) = pow(0.1 * delta_time, 2.0);//maximum 0.5rad/s;
        m_emQ(3, 3) = pow(4.0 * delta_time, 2.0);//maximum 8m/s acceleration;
        m_emQ(4, 4) = pow(0.1 * delta_time, 2.0);//maximum 1.0rad/s2 yaw rate acceleration;

        //predict system state;
        double delta_x = 0.0;
        double delta_y = 0.0;
        double last_yaw = m_evX(2);
        double new_yaw = last_yaw + yaw_vel * delta_time;

        Eigen::Matrix<double, 5, 5> jacobian;
        jacobian.setZero();

        auto velocity = m_evX(3);

        jacobian(0, 0) = 1.0;
        jacobian(1, 1) = 1.0;
        jacobian(2, 2) = 1.0;
        jacobian(3, 3) = 1.0;
        jacobian(4, 4) = 1.0;
        jacobian(2, 4) = delta_time;

        if (fabs(yaw_vel) > 1e-3)
        {
            delta_x = velocity * (sin(new_yaw) - sin(last_yaw)) / yaw_vel;
            delta_y = velocity * (-cos(new_yaw) + cos(last_yaw)) / yaw_vel;

            jacobian(0, 2) = velocity * (-cos(last_yaw) + cos(new_yaw)) / yaw_vel;
            jacobian(0, 3) = (-sin(last_yaw) + sin(new_yaw)) / yaw_vel;
            jacobian(0, 4) = delta_time * velocity * cos(new_yaw) / yaw_vel - velocity * (-sin(last_yaw) + sin(new_yaw)) / (yaw_vel * yaw_vel);
            jacobian(1, 2) = velocity * (-sin(last_yaw) + sin(new_yaw)) / yaw_vel;
            jacobian(1, 3) = (cos(last_yaw) - cos(new_yaw)) / yaw_vel;
            jacobian(1, 4) = delta_time * velocity * sin(new_yaw) / yaw_vel - velocity * (cos(last_yaw) - cos(new_yaw)) / (yaw_vel * yaw_vel);

        }
        else
        {
            delta_x = velocity * cos(last_yaw) * delta_time;
            delta_y = velocity * sin(last_yaw) * delta_time;

            jacobian(0, 2) = -velocity * sin(last_yaw) * delta_time;
            jacobian(0, 3) = cos(last_yaw) * delta_time;
            jacobian(1, 2) = velocity * cos(last_yaw) * delta_time;
            jacobian(1, 3) = sin(last_yaw) * delta_time;
        }

        m_evX(0) += delta_x;
        m_evX(1) += delta_y;
        m_evX(2) += yaw_vel * delta_time;
        NormalizeYaw(m_evX(2));
        m_evX(3) = m_evX(3);
        m_evX(4) = yaw_vel;

        if (fabs(delta_x) > 2.0 || fabs(delta_y) > 2.0)
        {
            LOG(WARNING) << "jacobian:\n" << jacobian << endl;
            LOG(WARNING) << "predicted P:\n" << m_emP << endl;
            LOG(WARNING) << "predicted pose:\n" << m_evX << endl;
            //exit(EXIT_FAILURE);
        }

        //        cerr<<"jacobian:\n"<<jacobian<<endl;
        //        cerr<<"predicted P:\n"<<m_emP<<endl;

        //predict state covariance;
        m_emP = jacobian * m_emP * jacobian.transpose() + m_emQ;
        m_fTimeStamp = timestamp;

        //        cerr<<"updated P:\n"<<m_emP<<endl;

        //        cerr<<"prediction yaw rate:"<<yaw_vel<<", covariance at "<<m_fTimeStamp<<"\n"<<m_emP<<endl;
        //        cerr<<"yaw rate:"<<yaw_vel<<", predicted pose:\n"<<m_evX<<endl;
        m_mtxFilter.unlock();
    }

    /**@brief measurement update;
     *
     * @param timestamp [IN]: current timestamp;
     * @param px [IN]: current vehicle position x in engineer frame;
     * @param py [IN]: current vehicle position y in engineer frame;
     * @param yaw [IN]: current vehicle heading;
     * @param velocity [IN]: velocity;
     * @param confidence [IN]: measurement confidence;
     */
    void Update(double timestamp, double px, double py, double yaw, double velocity, double confidence)
    {
        m_mtxFilter.lock();
        double delta_time = timestamp - m_fTimeStamp;

        double std_var_x = 0.05 + (1 - confidence) * (1 - confidence) * 0.2;
        double std_var_y = 0.05 + (1 - confidence) * (1 - confidence) * 0.2;
        double std_var_yaw = 0.01745 + (1 - confidence) * (1 - confidence) * 0.0349;
        double std_var_vel = 0.05 + (1 - confidence) * (1 - confidence) * 0.2;

        m_emR(0, 0) = std_var_x * std_var_x;
        m_emR(1, 1) = std_var_y * std_var_y;
        m_emR(2, 2) = std_var_yaw * std_var_yaw;
        m_emR(3, 3) = std_var_vel * std_var_vel;


        Eigen::Matrix<double, 4, 5> H;
        H.setZero();
        H(0, 0) = 1.0;
        H(1, 1) = 1.0;
        H(2, 2) = 1.0;
        H(3, 3) = 1.0;
        auto K = m_emP * H.transpose() * (H * m_emP * H.transpose() + m_emR).inverse();

        Eigen::Matrix<double, 4, 1> z;
        z << px, py, yaw, velocity;
        Eigen::Matrix<double, 4, 1> hx;
        hx << m_evX(0), m_evX(1), m_evX(2), m_evX(3);

        auto last_px = m_evX(0);
        auto last_py = m_evX(1);
        auto last_heading = m_evX(2);
        auto last_velocity = m_evX(3);

        Eigen::Matrix<double, 4, 1> delta = z - hx;
        if (yaw - 6.0 > m_evX(2))
        {
            delta(2) -= PI_DOUBLE;
        }

        if (m_evX(2) - 6.0 > yaw)
        {
            delta(2) += PI_DOUBLE;
        }

        m_evX += K * delta;
        NormalizeYaw(m_evX(2));


        Eigen::Matrix<double, 5, 5> I;
        I.setIdentity();
        m_emP = (I - K * H) * m_emP * (I - K * H).transpose() + K * m_emR * K.transpose();
        m_fTimeStamp = timestamp;
        if (fabs(m_evX(0) - last_px) > 2.0 || fabs(m_evX(1) - last_py) > 2.0)
        {
            LOG(WARNING) << "K:" << K << endl;
            LOG(WARNING) << "predicted px:" << last_px << ", py:" << last_py << ", yaw:" << last_heading * RADIAN_TO_DEGREE << ", velocity:" << last_velocity << endl;
            LOG(WARNING) << "measured px:" << px << ", py:" << py << ", yaw:" << yaw * RADIAN_TO_DEGREE << ", velocity:" << velocity << endl;
            LOG(WARNING) << "updated px:" << m_evX(0) << ", py:" << m_evX(1) << ", yaw:" << m_evX(2) * RADIAN_TO_DEGREE << ", velocity:" << m_evX(3) << endl;
            //exit(EXIT_FAILURE);
        }

        m_mtxFilter.unlock();
    }

    void GetPose(Eigen::Matrix<double, 5, 1> &pose)
    {
        m_mtxFilter.lock_shared();
        pose = m_evX;
        NormalizeYaw(pose(2));
        m_mtxFilter.unlock_shared();
    }

private:
    void NormalizeYaw(double &yaw)
    {
        while (yaw < 0.0)
        {
            yaw = PI_DOUBLE + yaw;
        }

        while (yaw > PI_DOUBLE)
        {
            yaw -= PI_DOUBLE;
        }
    }

private:
    Eigen::Matrix<double, 5, 5> m_emQ;//model noise;
    Eigen::Matrix<double, 4, 4> m_emR;//measurement noise covariance;
    Eigen::Matrix<double, 5, 5> m_emP;//system state covariance;

    double m_fTimeStamp;

    Eigen::Matrix<double, 5, 1> m_evX;//system state order: px, py, heading, velocity, yaw velocity;

    boost::shared_mutex m_mtxFilter;
};


#endif //LOCALIZER_EKF_HPP
