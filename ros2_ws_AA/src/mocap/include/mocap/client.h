/**
 * @file client.h
 * @brief Abstract base class for a mocap client
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 11 Dec 2019
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace acl {
namespace mocap {

  /**
   * @brief      Raw measurement of marker from mocap system (or model marker)
   */
  struct Marker {
    std::string name; ///< marker name

    // position, meters
    double x = 0, y = 0, z = 0;

    // true if marker was absent in a data frame
    bool occluded = false;

    Marker(const std::string& name) : name(name) {}
  };

  /**
   * @brief      Raw measurement of rigid body from mocap system
   */
  struct RigidBodyMeasurement {
    std::string name; ///< subject name
    uint64_t time_ns; ///< timestamp in nanoseconds

    // position, meters
    double x = 0, y = 0, z = 0;

    // orientation, quaternion
    double qw = 0, qx = 0, qy = 0, qz = 0;

    size_t nrMarkers = 0;
    size_t nrVisibleMarkers = 0;
    std::vector<Marker> markers;

    // true if subject was absent in a data frame
    bool occluded = false;

    RigidBodyMeasurement(const std::string& name) : name(name) {}
  };

  struct Centroid {
    double u = 0, v = 0; ///< centroid position in pixels
    double radius = 0; ///< radius of centroid
  };

  /**
   * @brief      Camera intrinsics and extrinsics after mocap calibration
   */
  struct Camera {
    std::string name;
    bool enabled;
    int index;
    int deviceid;
    std::string type;
    int image_width, image_height;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    double image_error;
    double focal_length;
    Eigen::Vector2d principal_point;
    Eigen::Vector2d radial_distortion;
    std::vector<Centroid> centroids;
  };


  class Client
  {
  public:

    /**
     * @brief      Name of server
     *
     * @return     The server name string
     */
    virtual std::string getName() = 0;

    /**
     * @brief      Version of the underlying mocap SDK.
     *
     * @return     SDK version string
     */
    virtual std::string getSDKVersion() = 0;
    
    /**
     * @brief      Initialize communication / connection with server
     *
     * @return     True if successful
     */
    virtual bool init() = 0;

    /**
     * @brief      Attempt to receive a data frame
     *
     * @return     True if successful
     */
    virtual bool spinOnce() = 0;

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
    virtual std::vector<RigidBodyMeasurement> getRigidBodyMeasurements() = 0;

    /**
     * @brief      Retrieve raw measurements of unassociated rigid bodies.
     *             Note that these measurements may in fact be of a marker
     *             attached to a tracked rigid body, but the mocap software
     *             does not think so.
     *
     * @return     Marker measurements.
     */
    virtual std::vector<Marker> getUnlabeledMarkers() = 0;

    /**
     * @brief      Retrieves a list of cameras from the mocap client. Each
     *             camera may optionally contain the pixel measurements of
     *             marker centroids.
     *
     * @return     The cameras.
     */
    virtual std::vector<Camera> getCameras() = 0;

    /**
     * @brief      Gets the time of the current data frame being processed
     *
     * @return     Time in nanoseconds
     */
    virtual uint64_t getCurrentTimestampNs() = 0;

    /**
     * @brief      Gets the total latency from tracking to publishing.
     *
     * @return     The total latency in seconds.
     *             If information is not available, -1.
     */
    virtual double getTotalLatency() = 0;

    /**
     * @brief      Gets the estimated clock offset between ROS and mocap sw.
     *
     * @return     The estimated time offset in nanoseconds.
     */
    virtual double getEstimatedTimeNsOffset() = 0;
    
  };

} // ns mocap
} // ns acl
