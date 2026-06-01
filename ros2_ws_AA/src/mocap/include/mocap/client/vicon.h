/**
 * @file vicon.h
 * @brief Specialization for VICON client
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 11 Dec 2019
 */

#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <DataStreamClient.h>

#include "mocap/client.h"
#include "mocap/utils.h"

namespace acl {
namespace mocap {

  namespace ViconSDK = ViconDataStreamSDK::CPP;

  class VICON : public Client
  {
  public:
    struct Parameters {
      std::string host; ///< hostname and port of VICON server
    };

  public:
    VICON(const Parameters& params);
    ~VICON();

    /**
     * @brief      Name of server
     *
     * @return     The server name string
     */
    std::string getName() override { return "VICON"; }

    /**
     * @brief      Version of the underlying mocap SDK.
     *
     * @return     SDK version string
     */
    std::string getSDKVersion() override { return version_; }

    /**
     * @brief      Initialize communication / connection with server
     *
     * @return     True if successful
     */
    bool init() override;

    /**
     * @brief      Attempt to receive a data frame
     *
     * @return     True if successful
     */
    bool spinOnce() override;

    /**
     * @brief      Retrieve raw measurements of rigid bodies
     * 
     *             We assume that any body that is enabled (i.e., that could
     *             potentially be seen by mocap) will have a raw measurement
     *             in this list. For example, VICON and OptiTrack send every
     *             enabled rigid body even if the body is occluded and/or the
     *             measurement for this timestep/frame is not valid.
     *
     * @return     Rigid body raw measurements
     */
    std::vector<RigidBodyMeasurement> getRigidBodyMeasurements() override;

    /**
     * @brief      Retrieve raw measurements of unassociated rigid bodies.
     *             Note that these measurements may in fact be of a marker
     *             attached to a tracked rigid body, but the mocap software
     *             does not think so.
     *
     * @return     Marker measurements.
     */
    std::vector<Marker> getUnlabeledMarkers() override;

    /**
     * @brief      Retrieves a list of cameras from the mocap client. Each
     *             camera may optionally contain the pixel measurements of
     *             marker centroids.
     *
     * @return     The cameras.
     */
    std::vector<Camera> getCameras() override;

    /**
     * @brief      Gets the time of the current data frame being processed
     *
     * @return     Time in nanoseconds
     */
    uint64_t getCurrentTimestampNs() override;

    /**
     * @brief      Gets the total latency from tracking to publishing.
     *
     * @return     The total latency in seconds.
     *             If information is not available, -1.
     */
    double getTotalLatency() override;

    /**
     * @brief      Gets the estimated clock offset between ROS and mocap sw.
     *
     * @return     The estimated time offset in nanoseconds.
     */
    double getEstimatedTimeNsOffset() override;

  private:
    std::string version_; ///< version of datastream sdk
    Parameters params_; ///< user defined instance parameters
    ViconSDK::Client client_; ///< VICON client sdk to get data

    MovingAverage<double, 300> offset_ns_ma_; ///< moving average estimator for time offset, [ns]
    double t_current_frame_rx_ms_ = 0.0; ///< time of current frame in ms

    /**
     * @brief      Create a RigidBodyMeasurement object to encapsulate data
     *
     * @param[in]  name  Name of the subject
     *
     * @return     Populated RigidBodyMeasurement object
     */
    RigidBodyMeasurement getData(const std::string& name);
  };

} // ns mocap
} // ns acl
