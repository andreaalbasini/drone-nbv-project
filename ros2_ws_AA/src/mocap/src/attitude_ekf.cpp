/*!
 * \file attitude_ekf.cpp
 *
 * Attitude EKF for vicon
 *
 * Created on: Oct 14, 2013
 *     Author: Mark Cutler
 *      Email: markjcutler@gmail.com
 */

#include "mocap/attitude_ekf.h"
#include <functional>

/**
 * Initialize the attitude filter parameters.
 */
AttitudeEKF::AttitudeEKF()
{
    // initialize variables
    // flip = 1;

    // Set up process noise
    Wc = Eigen::MatrixXd::Zero(ATT_STATES, ATT_STATES);
    Wc(0, 0) = ATTITUDE_PROCESS_NOISE;
    Wc(1, 1) = ATTITUDE_PROCESS_NOISE;
    Wc(2, 2) = ATTITUDE_PROCESS_NOISE;
    Wc(3, 3) = ATTITUDE_FICITICOUS_PROCESS_NOISE;
    Wc(4, 4) = ATTITUDE_FICITICOUS_PROCESS_NOISE;
    Wc(5, 5) = ATTITUDE_FICITICOUS_PROCESS_NOISE;
    Wc(6, 6) = ATTITUDE_FICITICOUS_PROCESS_NOISE;

    // Set up measurment noise
    Rd = Eigen::MatrixXd::Zero(ATT_MEAS, ATT_MEAS);
    Rd(0, 0) = ATTITDUE_MEASURMENT_NOISE;
    Rd(1, 1) = ATTITDUE_MEASURMENT_NOISE;
    Rd(2, 2) = ATTITDUE_MEASURMENT_NOISE;
    Rd(3, 3) = ATTITDUE_MEASURMENT_NOISE;

    reset();
}

void AttitudeEKF::reset()
{
    // Initialize internal variables
    xhatm = Eigen::VectorXd::Zero(ATT_STATES);
    xhatm(6) = 1.0; // q0 term
    xhatp = Eigen::VectorXd::Zero(ATT_STATES);
    xhatp(6) = 1.0; //q0 term
    L = Eigen::MatrixXd::Zero(ATT_STATES, ATT_MEAS);
    Qm = Eigen::MatrixXd::Identity(ATT_STATES, ATT_STATES);
    Qp = Eigen::MatrixXd::Identity(ATT_STATES, ATT_STATES);
    C = Eigen::MatrixXd::Zero(ATT_MEAS, ATT_STATES);

    // Set up measurement matrix -- measure quaternion directly
    C(0, 3) = 1.0;
    C(1, 4) = 1.0;
    C(2, 5) = 1.0;
    C(3, 6) = 1.0;
    // Initialize q_prev to up
    q_prev = Eigen::Quaterniond::Identity();

    initialized_ = false;
}

/**
 * Propagate state forward using rk4 integration
 * @param dt Time step over which to propagate
 */
void AttitudeEKF::propUpdate(double dt)
{

    // if (not initialized_){
    //  return;
    // }

    // get xhatp and Qp in a valarray
    actual_dt = dt;
    std::valarray<double> Qp_vec = vec(Qp);
    std::valarray<double> x0, x1;
    x0.resize(ATT_STATES + Qp_vec.size());
    x1.resize(ATT_STATES + Qp_vec.size());

    for (int i = 0; i < ATT_STATES; i++)
        x0[i] = xhatp(i);
    for (unsigned int i = ATT_STATES; i < x0.size(); i++)
        x0[i] = Qp_vec[i-ATT_STATES];

    // call rk4 integration step
    /// @todo allow rk4 to do multiple step intregation to get better precision
    // x1 = acl::rk4(0, x0, dt, boost::bind(&AttitudeEKF::xQdot, this, _1, _2));
    x1 = acl::rk4(
      0,
      x0,
      dt,
      std::bind(&AttitudeEKF::xQdot,
                this,
                std::placeholders::_1,
                std::placeholders::_2)
    );

    // Get xhatm and Qm
    for (int i = 0; i < ATT_STATES; i++)
        xhatm(i) = x1[i];
    for (unsigned int i = ATT_STATES; i < x1.size(); i++)
        Qp_vec[i-ATT_STATES] = x1[i];
    Qm = unvec(Qp_vec);

    // Write xhat_plus just in case there is no measurement update.
    // The xhatp will be overwritten when a measUpdate is called.
    xhatp = xhatm;
}


void AttitudeEKF::measUpdate(geometry_msgs::msg::Quaternion q_meas)
{
    if (not initialized_){
        xhatp(6) = q_meas.w;
        xhatp(3) = q_meas.x;
        xhatp(4) = q_meas.y;
        xhatp(5) = q_meas.z;
        initialized_ = true;
    }
    else{   
        Eigen::Quaterniond q(q_meas.w, q_meas.x, q_meas.y, q_meas.z);
        Eigen::Matrix<double, ATT_MEAS, ATT_MEAS> tmp1;
        tmp1 = C * Qm * C.transpose() + Rd;
        Qp = Qm - Qm * C.transpose() * tmp1.inverse() * C * Qm;
        L = Qp * C.transpose() * Rd.inverse();
        xhatp = xhatm + L * (q.coeffs() - C * xhatm);
    }
}


geometry_msgs::msg::Quaternion AttitudeEKF::getAtt()
{
    // renormalize the quaternion just in case
    Eigen::Quaterniond q(xhatp(6), xhatp(3), xhatp(4), xhatp(5));
    q.normalize();

    geometry_msgs::msg::Quaternion qout;
    qout.w = q.w();
    qout.x = q.x();
    qout.y = q.y();
    qout.z = q.z();

    return qout;
}

geometry_msgs::msg::Vector3 AttitudeEKF::getRate()
{
    geometry_msgs::msg::Vector3 rate;
    rate.x = xhatp(0);
    rate.y = xhatp(1);
    rate.z = xhatp(2);

    return rate;
}

/**
 * Derivative calculations for rk4 solver
 * @param t Dummy time variable needed for rk4 integration
 * @param xAll valarray of state plus covariance matrix
 * @return Derivative of state
 */
std::valarray<double> AttitudeEKF::xQdot(double t, std::valarray<double> xAll)
{
    // local vars
    Eigen::Matrix<double, ATT_STATES, 1> x, xdot;
    Eigen::Matrix<double, ATT_STATES, ATT_STATES> Q, A, Qdot;

    // Get state and Q
    for (int i = 0; i < ATT_STATES; i++)
        x(i) = xAll[i];
    // pull off elements of xAll that contribute to Q
    std::valarray<double> xQ = xAll[std::slice(ATT_STATES, xAll.size(), 1)];
    Q = unvec(xQ);

    // xdot
    xdot = processModel(x);

    // qdot
    A = transitionMatrix(x);
    Qdot = A * Q + Q * A.transpose() + Wc;
    Qdot = (Qdot + Qdot.transpose()) / 2.0;
    std::valarray<double> Qvecdot = vec(Qdot);

    // get xdot and qdot as a single valarray
    std::valarray<double> xout;
    xout.resize(ATT_STATES + Qvecdot.size());
    for (unsigned int i = 0; i < ATT_STATES; i++)
        xout[i] = xdot(i);
    for (unsigned int i = ATT_STATES; i < xout.size(); i++)
        xout[i] = Qvecdot[i-ATT_STATES];

    return xout;

}

/**
 * Pull off the upper triangle of a square matrix and convert it to a vector.
 * @param m Square matrix (should be symmetric or this function isn't helpful)
 * @return valarray<double> of upper triangular components of m
 */
std::valarray<double> AttitudeEKF::vec(Eigen::MatrixXd m)
{
    // check that the matrix is square first
    if (m.cols() != m.rows())
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Matrix in vec command is not square!");
        std::valarray<double> v(0);
        return v;
    }

    // allocate output array
    unsigned int n = m.cols();
    std::valarray<double> v(n * (n + 1) / 2); // relies on fact that sum 1 to n = n*(n+1)/2

    // pull off upper triangle
    int index = 0;
    for (unsigned int i = 0; i < n; i++)
    {
        for (unsigned int j = i; j < n; j++)
        {
            v[index++] = m(i, j);
        }
    }

    return v;
}

/**
 * Create a symmetric matrix from a vector containing the upper triangular
 * entries of the matrix.
 * @param v Vector of matrix upper triangular entries
 * @return Symmetric matrix
 */
Eigen::MatrixXd AttitudeEKF::unvec(std::valarray<double> v)
{
    // figure out how large the output vector should be
    unsigned int lenV = v.size();

    // lenV = n*(n+1)/2 from http://en.wikipedia.org/wiki/1_%2B_2_%2B_3_%2B_4_%2B_%E2%8B%AF
    // So, n^2 + n - 2*lenV = 0 -> solve for largest root
    unsigned int n = largestRoot(lenV);

    // allocate output matrix
    Eigen::MatrixXd m = Eigen::MatrixXd::Zero(n,n);
    Eigen::MatrixXd m1 = Eigen::MatrixXd::Zero(n,n);

    // fill upper triangle of output matrix
    int index = 0;
    for (unsigned int i = 0; i < n; i++)
    {
        for (unsigned int j = i; j < n; j++)
        {
            m(i, j) = v[index++];
        }
    }

    // make matrix symmetric
    m1 += m + m.transpose();
    m1 -= m.diagonal().asDiagonal();
    return m1;
}

unsigned int AttitudeEKF::largestRoot(unsigned int lenV)
{
    // n^2 + n - 2*lenV = 0 -> solve for largest root
    double a = 1;
    double b = 1;
    double c = -2.0 * lenV;

    double d = b * b - 4.0 * a * c;

    // the roots had better be real
    if (d < 0)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Error in solving for largest root -- imaginary roots");
        return 0;
    }
    // return max root
    else if (d == 0)
    {
        return (unsigned int) (-b / (2.0 * a));
    }
    else
    {
        double r1, r2;
        r1 = (-b + sqrt(d)) / (2.0 * a);
        r2 = (-b - sqrt(d)) / (2.0 * a);
        if (r1 > r2)
            return r1;
        else
            return r2;
    }

}

/**
 * Process model for propagating forward angular rates and attitude quaternion
 *
 * @param x State vector
 * @return
 */
Eigen::Matrix<double, ATT_STATES, 1> AttitudeEKF::processModel(
        Eigen::Matrix<double, ATT_STATES, 1> x)
{
    double tau_rx = RATE_TIME_CONSTANT;
    double tau_ry = RATE_TIME_CONSTANT;
    double tau_rz = RATE_TIME_CONSTANT;

    // normalization value for attitude quaternion
    double n = sqrt(x(3) * x(3) + x(4) * x(4) + x(5) * x(5) + x(6) * x(6));

    Eigen::Matrix<double, ATT_STATES, 1> xprime;
    xprime(0) = -x(0) / tau_rx;
    xprime(1) = -x(1) / tau_ry;
    xprime(2) = -x(2) / tau_rz;
    xprime(3) = 0.5 * (x(2) * x(4) - x(1) * x(5) + x(0) * x(6)) / n;
    xprime(4) = 0.5 * (-x(2) * x(3) + x(0) * x(5) + x(1) * x(6)) / n;
    xprime(5) = 0.5 * (x(1) * x(3) - x(0) * x(4) + x(2) * x(6)) / n;
    xprime(6) = 0.5 * (-x(0) * x(3) - x(1) * x(4) - x(2) * x(5)) / n;

    return xprime;

}

/**
 * Calculates the derivative of the linearized process dynamics
 *
 * @param x State
 * @return
 */
Eigen::Matrix<double, ATT_STATES, ATT_STATES> AttitudeEKF::transitionMatrix(
        Eigen::Matrix<double, ATT_STATES, 1> x)
{
    double tau_rx = RATE_TIME_CONSTANT;
    double tau_ry = RATE_TIME_CONSTANT;
    double tau_rz = RATE_TIME_CONSTANT;

    // normalization value for attitude quaternion
    double n = sqrt(x(3) * x(3) + x(4) * x(4) + x(5) * x(5) + x(6) * x(6));

    // f0 = -x1/tau_rx;
    // f1 = -x1/tau_ry;
    // f2 = -x1/tau_rz;
    double f3 = 0.5 * (x(2) * x(4) - x(1) * x(5) + x(0) * x(6)) / n;
    double f4 = 0.5 * (-x(2) * x(3) + x(0) * x(5) + x(1) * x(6)) / n;
    double f5 = 0.5 * (x(1) * x(3) - x(0) * x(4) + x(2) * x(6)) / n;
    double f6 = 0.5 * (-x(0) * x(3) - x(1) * x(4) - x(2) * x(5)) / n;

    // derivatives for linearization
    Eigen::Matrix<double, ATT_STATES, ATT_STATES> phi = Eigen::MatrixXd::Zero(
            ATT_STATES, ATT_STATES);
    phi(0, 0) = 1 - actual_dt / tau_rx; // df0_x0
    phi(1, 1) = 1 - actual_dt / tau_ry; //df1_x1
    phi(2, 2) = 1 - actual_dt / tau_rz; // df2_x2

    phi(3, 0) = 0.5 * (x(6) / n); // df3_x0
    phi(3, 1) = 0.5 * (-x(5) / n); // df3_x1
    phi(3, 2) = 0.5 * (x(4) / n); // df3_x2
    phi(3, 3) = 1 + 0.5 * (-f3 * x(3) / n * n * n); // df3_x3
    phi(3, 4) = 0.5 * (x(2) / n - f3 * x(4) / n * n * n); // df3_x4
    phi(3, 5) = 0.5 * (-x(1) / n - f3 * x(5) / n * n * n); // df3_x5
    phi(3, 6) = 0.5 * (x(0) / n - f3 * x(6) / n * n * n); // df3_x6

    phi(4, 0) = 0.5 * (x(5) / n); // df4_x0
    phi(4, 1) = 0.5 * (x(6) / n); // df4_x1
    phi(4, 2) = 0.5 * (-x(3) / n); // df4_x2
    phi(4, 3) = 0.5 * (-x(2) / n - f4 * x(3) / n * n * n); // df4_x3
    phi(4, 4) = 1 + 0.5 * (-f4 * x(4) / n * n * n); // df4_x4
    phi(4, 5) = 0.5 * (x(0) / n - f4 * x(5) / n * n * n); // df4_x5
    phi(4, 6) = 0.5 * (x(1) / n - f4 * x(6) / n * n * n); // df4_x6

    phi(5, 0) = 0.5 * (-x(4) / n); // df5_x0
    phi(5, 1) = 0.5 * (x(3) / n); // df5_x1
    phi(5, 2) = 0.5 * (x(6) / n); // df5_x2
    phi(5, 3) = 0.5 * (x(1) / n - f5 * x(3) / n * n * n); // df5_x3
    phi(5, 4) = 0.5 * (-x(0) / n - f5 * x(4) / n * n * n); // df5_x4
    phi(5, 5) = 1 + 0.5 * (-f5 * x(5) / n * n * n); // df5_x5
    phi(5, 6) = 0.5 * (x(2) / n - f5 * x(6) / n * n * n); // df5_x6

    phi(6, 0) = 0.5 * (-x(3) / n); // df6_x0
    phi(6, 1) = 0.5 * (-x(4) / n); // df6_x1
    phi(6, 2) = 0.5 * (-x(5) / n); // df6_x2
    phi(6, 3) = 0.5 * (-x(0) / n - f6 * x(3) / n * n * n); // df6_x3
    phi(6, 4) = 0.5 * (-x(1) / n - f6 * x(4) / n * n * n); // df6_x4
    phi(6, 5) = 0.5 * (-f6 * x(6) / n * n * n); // df6_x5
    phi(6, 6) = 1 + 0.5 * (-x(2) / n - f6 * x(5) / n * n * n); // df6_x6

    return phi;
}
