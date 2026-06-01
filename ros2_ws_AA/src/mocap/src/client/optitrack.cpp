/**
 * @file optitrack.h
 * @brief Specialization for OptiTrack client
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 8 Nov 2022
 */

#include <sstream>

#include "mocap/client/optitrack.h"

namespace acl {
namespace mocap {

OptiTrack::OptiTrack(const Parameters& params)
: params_(params)
{}

// ----------------------------------------------------------------------------

OptiTrack::~OptiTrack()
{
  // TODO: need to disconnect properly?
}

// ----------------------------------------------------------------------------

bool OptiTrack::init()
{
  // attempt to connect to server
  client_ = std::make_unique<agile::OptiTrackClient>(params_.localIP, params_.serverIP,
                                                    params_.multicastIP, params_.commandPort,
                                                    params_.dataPort);
  if (!client_->initConnection()) return false;

  int major, minor;
  client_->getVersion(major, minor);
  std::stringstream version;
  version << "OptiTrack NatNet SDK v" << major << "." << minor;
  version_ = version.str();

  return true;
}

// ----------------------------------------------------------------------------

bool OptiTrack::spinOnce()
{
  // make sure that there is a new frame of data to consume
  return client_->spinOnce();
}

// ----------------------------------------------------------------------------

std::vector<RigidBodyMeasurement> OptiTrack::getRigidBodyMeasurements()
{
  std::vector<RigidBodyMeasurement> bodies;

  const std::vector<agile::Packet> pkts = client_->getPackets();
  for (const auto& pkt : pkts) {
    last_tx_timestamp_ns_ = pkt.transmit_timestamp;
    last_rx_timestamp_ns_ = pkt.receive_timestamp;
    last_camera_mid_exposure_timestamp_ns_ = pkt.mid_exposure_timestamp;

    RigidBodyMeasurement rbm(pkt.model_name);
    rbm.time_ns = getEstimatedTimeNsOffset() + pkt.mid_exposure_timestamp;

    rbm.x = pkt.pos[0];
    rbm.y = pkt.pos[1];
    rbm.z = pkt.pos[2];

    rbm.qx = pkt.orientation[0];
    rbm.qy = pkt.orientation[1];
    rbm.qz = pkt.orientation[2];
    rbm.qw = pkt.orientation[3];

    rbm.nrMarkers = pkt.expected_markers.size();
    // TODO: extract this rigid body's markers from labeled markers in sdk?
    rbm.nrVisibleMarkers = pkt.expected_markers.size(); //pkt.markers.size();

    for (size_t i=0; i<pkt.markers.size(); i++) {
      Marker m("marker" + std::to_string(i));
      m.x = pkt.markers[i].x;
      m.y = pkt.markers[i].y;
      m.z = pkt.markers[i].z;
      rbm.markers.push_back(m);
    }

    rbm.occluded = !pkt.tracking_valid;

    bodies.push_back(rbm);
  }

  return bodies;
}

// ----------------------------------------------------------------------------

std::vector<Marker> OptiTrack::getUnlabeledMarkers()
{
  return {};
}

// ----------------------------------------------------------------------------

std::vector<Camera> OptiTrack::getCameras()
{
  return {};
}

// ----------------------------------------------------------------------------

uint64_t OptiTrack::getCurrentTimestampNs()
{
  return client_->getLatestServerTimeNs();
}

// ----------------------------------------------------------------------------

double OptiTrack::getTotalLatency()
{
  return (last_tx_timestamp_ns_ - last_camera_mid_exposure_timestamp_ns_) * 1e-9;
}

// ----------------------------------------------------------------------------

double OptiTrack::getEstimatedTimeNsOffset()
{
  // Assuming transmission time is negligible. At 100 Hz, pose solves happen
  // every 10 ms, and UDP transmission time is <1 ms, e.g., see:
  // https://ennerf.github.io/2016/11/23/Analyzing-the-viability-of-Ethernet-and-UDP-for-robot-control.html
  // Then ignoring transmission time is probably fine.
  const int64_t comm_delay = 0;

  const int64_t offset = last_rx_timestamp_ns_ - comm_delay - last_tx_timestamp_ns_;
  return offset_ns_ma_(offset);
}

} // ns mocap
} // ns acl
