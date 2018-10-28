//
// Created by chen on 18-8-27.
//

#ifndef SYNCEKF_HPP
#define SYNCEKF_HPP

#include <global.hpp>
#include <eigen3/Eigen/Dense>

class CSystemState
{
public:
    CSystemState(double timestamp, double velocity, double yaw_vel)
        : m_fTimeStamp{timestamp},
          m_bPredicted{false},
          m_bUpdated{false},
          m_emQ{Eigen::Matrix<double, 5, 5>::Zero()},
          m_emR{Eigen::Matrix<double, 4, 4>::Zero()}
    {
        m_evU<<velocity, yaw_vel;
    }

    ~CSystemState()
    {

    }

    double GetTimeStamp()
    {
        return this->m_fTimeStamp;
    }

    bool IsPredicted()
    {
        return m_bPredicted;
    }

    bool IsUpdated()
    {
        return m_bUpdated;
    }

    Eigen::Matrix<double, 5, 1> GetState()
    {
        if (m_bUpdated)
        {
            return m_evPostX;
        }
        else
        {
            return m_evPriorX;
        }
    }

    void SetPriorState(const Eigen::Matrix<double, 5, 1> &state, const Eigen::Matrix<double, 5, 5> &cov)
    {
        this->m_bPredicted = true;
        this->m_evPriorX = state;
        this->m_emPriorP = cov;
    }

    /**@brief forward predict system state from another state;
     *
     * forward predict system state from another state;
     * @param from [IN]: the state that system state predicted from;
     * @note if the from state is already updated by measurement, the prediction is made from the posterior state in state; else, the prediction is made from prior state;
     */
    void Predict(const CSystemState &from)
    {
        if (m_fTimeStamp < from.m_fTimeStamp)
        {
            LOG(WARNING) << "cannot forward predict from " << std::fixed << std::setprecision(6) << from.m_fTimeStamp << " to " << this->m_fTimeStamp;
            return;
        }

        double delta_time = this->m_fTimeStamp - from.m_fTimeStamp;

        //set Q according to the delta time;
        m_emQ(0, 0) = pow(0.5 * 4.0 * delta_time * delta_time, 2.0);//maximum 8m/s2 acceleration;
        m_emQ(1, 1) = m_emQ(0, 0);
        m_emQ(2, 2) = pow(0.2 * delta_time, 2.0);//maximum 0.5rad/s;
        m_emQ(3, 3) = pow(4.0 * delta_time, 2.0);//maximum 8m/s acceleration;
        m_emQ(4, 4) = pow(1.0 * delta_time, 2.0);//maximum 1.0rad/s2 yaw rate acceleration;

        //predict system state;
        double delta_x = 0.0;
        double delta_y = 0.0;

        double velocity = m_evU(0);
        double last_yaw = 0.0;
        double new_yaw = 0.0;
        double yaw_vel = m_evU(1);

        if (from.m_bUpdated)//from is updated, use post state;
        {
            last_yaw = from.m_evPostX(2);
//            velocity = from.m_evPostX(3);
//            yaw_vel = from.m_evPostX(4);
        }
        else//from is not updated, use prior state;
        {
            last_yaw = from.m_evPriorX(2);
//            velocity = from.m_evPriorX(3);
//            yaw_vel = from.m_evPriorX(4);
        }
        new_yaw = last_yaw + yaw_vel * delta_time;

        Eigen::Matrix<double, 5, 5> jacobian;
        jacobian.setZero();

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

        //predicted state and state covariance;
        if (from.m_bUpdated)
        {
            this->m_evPriorX(0) = from.m_evPostX(0) + delta_x;
            this->m_evPriorX(1) = from.m_evPostX(1) + delta_y;
            this->m_evPriorX(2) = new_yaw;
            NormalizeYaw(this->m_evPriorX(2));
            this->m_evPriorX(3) = velocity;
            this->m_evPriorX(4) = yaw_vel;

            this->m_emPriorP = jacobian * from.m_emPostP * jacobian.transpose() + this->m_emQ;
        }
        else
        {
            this->m_evPriorX(0) = from.m_evPriorX(0) + delta_x;
            this->m_evPriorX(1) = from.m_evPriorX(1) + delta_y;
            this->m_evPriorX(2) = new_yaw;
            NormalizeYaw(this->m_evPriorX(2));
            this->m_evPriorX(3) = velocity;
            this->m_evPriorX(4) = yaw_vel;

            this->m_emPriorP = jacobian * from.m_emPriorP * jacobian.transpose() + this->m_emQ;
        }

        if (fabs(delta_x) > 5.0 || fabs(delta_y) > 5.0)
        {
            LOG(WARNING)<<"last timestamp:"<<ios::fixed<<setprecision(6)<<from.m_fTimeStamp<<", current timestamp:"<<this->m_fTimeStamp<<", delta time:"<<this->m_fTimeStamp - from.m_fTimeStamp;
            LOG(WARNING) << "jacobian:\n" << jacobian << endl;
            LOG(WARNING) << "predicted P:\n" << m_emPriorP << endl;
            LOG(WARNING) << "predicted pose:\n" << m_evPriorX << endl;
            if (from.m_bUpdated)
            {
                LOG(WARNING) << "from pose:\n" << from.m_evPostX << endl;
            }
            else
            {
                LOG(WARNING) << "from pose:\n" << from.m_evPriorX << endl;
            }
            LOG(WARNING) << "predicted pose:\n" << this->m_evPriorX << endl;

            LOG_IF(ERROR, fabs(delta_x)>20.0 || fabs(delta_y)>20.0)<<"prediction jump";
        }
        this->m_bPredicted = true;
    }

    /**@brief update state;
     *
     * update state by new measurement;
     * @note ensure the measurement and measurement covariance matrix are set before calling this function;
     */
    void Update()
    {
        LOG_IF(FATAL, !m_bPredicted) << "state not predicted";

        //the measurement function;
        Eigen::Matrix<double, 4, 5> H;
        H.setZero();
        H(0, 0) = 1.0;
        H(1, 1) = 1.0;
        H(2, 2) = 1.0;
        H(3, 3) = 1.0;
        //H(4, 4) = 1.0;

        //calculate the kalman gain;
        auto K = this->m_emPriorP * H.transpose() * (H * this->m_emPriorP * H.transpose() + this->m_emR).inverse();

        auto prior_heading = this->m_evPriorX(2);

        Eigen::Matrix<double, 4, 1> temp;
        temp<<m_evPriorX(0), m_evPriorX(1), m_evPriorX(2), m_evPriorX(3);
        Eigen::Matrix<double, 4, 1> delta = this->m_evZ - temp;
        if (this->m_evZ(2) - 6.0 > this->m_evPriorX(2))
        {
            delta(2) -= PI_DOUBLE;
        }

        if (this->m_evPriorX(2) - 6.0 > this->m_evZ(2))
        {
            delta(2) += PI_DOUBLE;
        }

        this->m_evPostX = this->m_evPriorX + K * delta;
        NormalizeYaw(this->m_evPostX(2));

        Eigen::Matrix<double, 5, 5> I;
        I.setIdentity();

        this->m_emPostP = (I - K * H) * this->m_emPriorP * (I - K * H).transpose() + K * this->m_emR * K.transpose();

        LOG_IF(ERROR, fabs(delta(0))>20.0 || fabs(delta(1))>20.0)<<"update jump, measurement:\n"<<this->m_evZ<<"\npredicted:\n"<<this->m_evPriorX<<"updated:\n"<<this->m_evPostX<<endl;
        this->m_bUpdated = true;
    }

    /**@brief update system state by new measurement;
     *
     * update system state by new measurement;
     * @param measurement [IN]: the new measurement;
     * @param confidence [IN]: the confidence of the new measurement;
     */
    void Update(const Eigen::Matrix<double, 4, 1> &measurement, double confidence)
    {
        //copy measurement
        this->m_evZ = measurement;

        //prepare the measurement covariance
        auto std_var_x = 0.1 + (1 - confidence) * (1 - confidence) * 0.2;
        auto std_var_y = 0.1 + (1 - confidence) * (1 - confidence) * 0.2;
        auto std_var_yaw = 0.01745 + (1 - confidence) * (1 - confidence) * 0.0349;
        auto std_var_vel = 0.1 + (1 - confidence) * (1 - confidence) * 0.1;
        this->m_emR(0, 0) = std_var_x * std_var_x;
        this->m_emR(1, 1) = std_var_y * std_var_y;
        this->m_emR(2, 2) = std_var_yaw * std_var_yaw;
        this->m_emR(3, 3) = std_var_vel * std_var_vel;
        //this->m_emR(4, 4) = 0.1 * 0.1;

        this->Update();
    }

    /**@brief normalize yaw into 0~2*pi range;
     *
     * @param yaw [IN|OUT]: the yaw angle to be normalized;
     */
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

public:
    double m_fTimeStamp;

    /**@note state X order: px, py, heading, velocity, yaw velocity*/
    /**@note system state transition function: x_{k+1} = f(x_{k}) */
    /**@note system measurement function: z_{k+1} = g(x_{k})*/
    bool m_bPredicted;

    Eigen::Matrix<double, 5, 1> m_evPriorX;//predicted x at step k+1;
    Eigen::Matrix<double, 5, 5> m_emPriorP;;//predicted x covariance at step k+1;

    bool m_bUpdated;

    Eigen::Matrix<double, 5, 1> m_evPostX;//updated x at step k+1;
    Eigen::Matrix<double, 5, 5> m_emPostP;//updated p at step k+1;

    Eigen::Matrix<double, 5, 5> m_emQ;//model noise at step k+1;

    Eigen::Matrix<double, 4, 1> m_evZ;//measurement at step k+1, measurement order: px, py, heading (in rad), velocity (in m/s), yaw velocity (in rad/s);
    Eigen::Matrix<double, 4, 4> m_emR;//measurement noise;

    Eigen::Matrix<double, 2, 1> m_evU;//control at step k+1, actually velocity and yaw velocity;
};

class SyncEKF
{
public:
public:
    SyncEKF()
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
    void Initialize(double timestamp, double px, double py, double heading, double velocity, double yaw_vel)
    {
        CSystemState system_state(timestamp, velocity, yaw_vel);

        Eigen::Matrix<double, 5, 1> state;
        Eigen::Matrix<double, 5, 5> cov;

        state(0) = px;
        state(1) = py;
        state(2) = heading;
        state(3) = velocity;
        state(4) = yaw_vel;

        cov.setZero();
        //inital state covariance;
        cov(0, 0) = 5.0 * 5.0;//gnss position variance, 5.0m;
        cov(1, 1) = 5.0 * 5.0;//gnss position variance, 5.0m;
        cov(2, 2) = 0.174 * 0.174;//yaw angle variance, 10deg;
        cov(3, 3) = 0.5 * 0.5;//velocity variance, 0.5m/s;
        cov(4, 4) = 0.017 * 0.017;//yaw angular velocity variance, 1deg/s;

        system_state.SetPriorState(state, cov);

        m_dStates.emplace_back(system_state);
    }

    /**@brief forward predict system state to specified timestamp;
     *
     * forward predict system state to current timestamp;
     * @param timestamp [IN]: current timestamp;
     */
    void Predict(double timestamp, double velocity, double yaw_vel)
    {
        m_mtxFilter.lock();
        CSystemState system_state(timestamp, velocity, yaw_vel);
        system_state.Predict(m_dStates.back());

        m_dStates.emplace_back(system_state);
        if (m_dStates.size() > 200)
        {
            m_dStates.pop_front();
        }
        m_mtxFilter.unlock();
    }

    /**@brief measurement update;
     *
     * @param timestamp [IN]: current timestamp;
     * @param px [IN]: current vehicle position x in engineer frame;
     * @param py [IN]: current vehicle position y in engineer frame;
     * @param yaw [IN]: current vehicle heading;
     * @param velocity [IN]: velocity;
     * @param yaw_vel [IN]: yaw velocity;
     * @param confidence [IN]: measurement confidence;
     */
    void Update(double timestamp, double px, double py, double yaw, double velocity, double yaw_vel, double confidence)
    {
        m_mtxFilter.lock();

        int next_idx = 0;
        CSystemState system_state(timestamp, velocity, yaw_vel);
        if (!this->GetPredictedSystemState(timestamp, system_state, next_idx))
        {
            m_mtxFilter.unlock();
            return;
        }

        Eigen::Matrix<double, 4, 1> measurement;
        measurement << px, py, yaw, velocity;//, yaw_vel;
        system_state.Update(measurement, confidence);

        if (next_idx < m_dStates.size())
        {
            m_dStates[next_idx].Predict(system_state);
        }

        for (int i = next_idx + 1; i < m_dStates.size(); ++i)
        {
            m_dStates[i].Predict(m_dStates[i - 1]);
        }

        m_mtxFilter.unlock();
    }

    void GetPose(Eigen::Matrix<double, 5, 1> &pose)
    {
        m_mtxFilter.lock_shared();
        pose = m_dStates.back().GetState();
        m_mtxFilter.unlock_shared();
    }


private:
    bool GetPredictedSystemState(double timestamp, CSystemState &state, int &next_idx)
    {
        if (m_dStates.empty())
        {
            return false;
        }

        if (m_dStates[0].GetTimeStamp() > timestamp)
        {
            return false;
        }

        int last_idx = 0;
        for (int i = m_dStates.size() - 1; i >= 0; i--)
        {
            if (m_dStates[i].GetTimeStamp() <= timestamp)
            {
                last_idx = i;
                next_idx = i + 1;
                break;
            }
        }

        //forward predict the state
        state.Predict(m_dStates[last_idx]);
        return true;
    }

private:
    deque<CSystemState> m_dStates;

    boost::shared_mutex m_mtxFilter;
};


#endif //SYNCEKF_HPP
