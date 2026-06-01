/*
 * defines.h
 *
 *  Created on: Sep 23, 2013
 *      Author: mark
 */

#ifndef DEFINES_H_
#define DEFINES_H_

#define POSITION_THETA 0.5      ///< filter gain
#define RATE_TIME_CONSTANT 10   ///< assumed time constant on angular rate -- large = more responsive filter, but more noise
#define ATTITUDE_PROCESS_NOISE 1.0 ///< Noise on the angular rate process
#define ATTITUDE_FICITICOUS_PROCESS_NOISE 0.0 ///< noise added to the integration of the quaternions
#define ATTITDUE_MEASURMENT_NOISE 0.00001//1.0e-7 ///< Measurment noise of the vicon returned quaternion
#define ATT_STATES  7   ///< number of states for the attitude EKF
#define ATT_MEAS 4      ///< number of measurements for the attitude EKF


#endif /* DEFINES_H_ */
