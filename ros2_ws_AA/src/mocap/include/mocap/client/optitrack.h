/**
 * @file optitrack.h
 * @brief Specialization for OptiTrack client
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 8 Nov 2022
 */

#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <optitrack_sdk/optitrack_client.h>

#include "mocap/client.h"
#include "mocap/utils.h"

namespace acl {
namespace mocap {

  class OptiTrack : public Client
  {
  public:
    struct Parameters {
      std::string localIP; ///< Local (host) IP address for commands (e.g., 0.0.0.0)
      std::string serverIP; ///< IP address of OptiTrack computer
      std::string multicastIP; ///< Multicast group IP (for NatNet rigid body data)
      int commandPort; ///< Port number (server side?)
      int dataPort; ///< Multicast data port number (for rigid body data, client/server side)
    };

  public:
    OptiTrack(const Parameters& params);
    ~OptiTrack();

    /**
     * @brief      Name of server
     *
     * @return     The server name string
     */
    std::string getName() override { return "OptiTrack"; }

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
    std::unique_ptr<agile::OptiTrackClient> client_; ///< OptiTrack NatNet client

    uint64_t last_server_time_ns_ = 0; ///< (last) server time since Motive software start, [ns]
    uint64_t last_tx_timestamp_ns_ = 0; ///< (last) transmit time of rigid body, [ns] (measured there)
    uint64_t last_rx_timestamp_ns_ = 0; ///< (last) received time of rigid body, [ns] (measured here)
    uint64_t last_camera_mid_exposure_timestamp_ns_ = 0; ///< (last) pose estimation time, [ns]

    MovingAverage<double, 300> offset_ns_ma_; ///< moving average estimator for time offset, [ns]
  };

} // ns mocap
} // ns acl
