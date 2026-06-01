/**
 * @file vicon.cpp
 * @brief Specialization for VICON client
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 11 Dec 2019
 */

#include <sstream>

#include "mocap/client/vicon.h"

namespace acl {
namespace mocap {

VICON::VICON(const Parameters& params)
: params_(params)
{}

// ----------------------------------------------------------------------------

VICON::~VICON()
{
  client_.Disconnect();
}

// ----------------------------------------------------------------------------

bool VICON::init()
{
  // attempt to connect to server
  client_.Connect(params_.host);
  if (!client_.IsConnected().Connected) return false;

  auto data = client_.GetVersion();
  std::stringstream version;
  version << "VICON DataStream SDK v" << data.Major << "." << data.Minor << "." << data.Point;
  version_ = version.str();

  // enable useful data streams
  client_.EnableSegmentData();
  client_.EnableMarkerData();
  client_.EnableUnlabeledMarkerData();
  // client_.EnableDeviceData();
  client_.EnableCentroidData();

  // set the streaming mode
  // client_.SetStreamMode( ViconSDK::StreamMode::ClientPull );
  // client_.SetStreamMode( ViconSDK::StreamMode::ClientPullPreFetch );
  client_.SetStreamMode(ViconSDK::StreamMode::ServerPush); // this is the lowest latency version

  // Set the global up axis (Z-up)
  client_.SetAxisMapping(ViconSDK::Direction::Forward,
                          ViconSDK::Direction::Left, ViconSDK::Direction::Up);

  // Skip some frames to avoid the "leftover" subject names from last conneciton.
  // This is perhaps due to the ServerPush mode.
  for (int i=0; i<25; i++) {
      while (client_.GetFrame().Result != ViconSDK::Result::Success);
  }

  return client_.IsConnected().Connected;
}

// ----------------------------------------------------------------------------

bool VICON::spinOnce()
{
  // make sure that there is a new frame of data to consume
  return client_.GetFrame().Result == ViconSDK::Result::Success;
}

// ----------------------------------------------------------------------------

std::vector<RigidBodyMeasurement> VICON::getRigidBodyMeasurements()
{
  std::vector<RigidBodyMeasurement> bodies;

  // get the timestamp of this data frame
  uint64_t time_ns = getEstimatedTimeNsOffset() + getCurrentTimestampNs();

  // loop through each of the tracked rigid bodies in the current frame
  size_t n = client_.GetSubjectCount().SubjectCount;
  for (size_t i=0; i<n; ++i) {
    std::string name = client_.GetSubjectName(i).SubjectName;

    // extract raw rigid body measurement and add timestamp
    bodies.push_back(getData(name));
    bodies.back().time_ns = time_ns;
  }

  return bodies;
}

// ----------------------------------------------------------------------------

std::vector<Marker> VICON::getUnlabeledMarkers()
{
  std::vector<Marker> markers;

  const size_t n = client_.GetUnlabeledMarkerCount().MarkerCount;
  for (size_t i=0; i<n; ++i) {
    auto data = client_.GetUnlabeledMarkerGlobalTranslation(i);
    if (data.Result == ViconSDK::Result::Success) {
      Marker marker(std::to_string(i));
      marker.x = data.Translation[0]*1e-3;
      marker.y = data.Translation[1]*1e-3;
      marker.z = data.Translation[2]*1e-3;
      marker.occluded = false;
      markers.push_back(marker);
    }
  }

  return markers;
}

// ----------------------------------------------------------------------------

std::vector<Camera> VICON::getCameras()
{
  std::vector<Camera> cameras;

  const size_t n = client_.GetCameraCount().CameraCount;
  for (size_t i=0; i<n; i++) {
    auto data = client_.GetCameraName(i);
    if (data.Result == ViconSDK::Result::Success) {
      Camera camera;
      // unnamed cameras are by default named "{Type} ({DeviceId})"
      camera.name = data.CameraName;

      {
        auto data = client_.GetCameraType(camera.name);
        camera.type = data.CameraType;
      }

      {
        auto data = client_.GetCameraId(camera.name);
        camera.deviceid = data.CameraId;
      }

      const size_t N = client_.GetCentroidCount(camera.name).CentroidCount;
      for (size_t j=0; j<N; j++) {
        auto data = client_.GetCentroidPosition(camera.name, j);
        if (data.Result == ViconSDK::Result::Success) {
          Centroid centroid;
          centroid.u = data.CentroidPosition[0];
          centroid.v = data.CentroidPosition[1];
          centroid.radius = data.Radius;
          camera.centroids.push_back(centroid);
        }
      }

      cameras.push_back(camera);
    }
  }

  return cameras;
}

// ----------------------------------------------------------------------------

uint64_t VICON::getCurrentTimestampNs()
{
  auto time = client_.GetTimecode();

  // get fps
  double fps = 0;
  switch (time.Standard) {
    case ViconSDK::TimecodeStandard::PAL:
      fps = 25.0;
      break;
    case ViconSDK::TimecodeStandard::NTSC:
      fps = 29.97;
      break;
    case ViconSDK::TimecodeStandard::NTSCDrop:
      throw std::runtime_error("NTSCDrop: invalid timecode standard");
      break;
    case ViconSDK::TimecodeStandard::Film:
      fps = 24.0;
      break;
    default:
      throw std::runtime_error("unknown: invalid timecode standard");
      break;
  }

  // base time (second resolution)
  uint64_t time_ns = (time.Seconds + 60*time.Minutes + 3600*time.Hours)*1e9;

  // subsecond resolution
  double frames = time.Frames + static_cast<double>(time.SubFrame)/time.SubFramesPerFrame;
  time_ns += static_cast<uint64_t>((frames/fps)*1e9);

  // used in getEstimatedTimeNsOffset()
  t_current_frame_rx_ms_ = time.PacketReceiptTime;

  return time_ns;
}

// ----------------------------------------------------------------------------

double VICON::getTotalLatency()
{
  // GetLatencyTotal returns 0.0 if info not available
  double latency = client_.GetLatencyTotal().Total;
  return (latency == 0) ? -1 : latency;
}

// ----------------------------------------------------------------------------

double VICON::getEstimatedTimeNsOffset()
{
  // See diagram at https://gitlab.com/mit-acl/fsw/mocap/-/issues/4#note_1164956152

  // Assuming transmission time is negligible. At 100 Hz, pose solves happen
  // every 10 ms, and UDP transmission time is <1 ms, e.g., see:
  // https://ennerf.github.io/2016/11/23/Analyzing-the-viability-of-Ethernet-and-UDP-for-robot-control.html
  // Then ignoring transmission time is probably fine.
  const int64_t comm_delay = 0;

  /* Original code gets time code here, but that doesn't work - so instead expose t_current_frame_rx_ms_ as global variable in getCurrentTimestampNs() and use it here. Kota Kondo May 2025*/
  // Get time that the current frame was received in linux
  // auto time = client_.GetTimecode();
  // const double t_current_frame_rx_ms = time.PacketReceiptTime;

  const double t_current_frame_rx_ns = t_current_frame_rx_ms_ * 1e6;

  // Latency between the camera observation and the data transmission,
  // see https://gitlab.com/mit-acl/fsw/mocap/-/issues/8
  const int64_t t_capture_to_transmit_ns = getTotalLatency()*1e9;

  // Assuming that the current frame timestamp is the time of camera observation, then
  // this quantity brings us to the time at which the data was transmitted.
  const int64_t t_current_frame_tx_ns = (getCurrentTimestampNs() + t_capture_to_transmit_ns);

  const int64_t offset = t_current_frame_rx_ns - comm_delay - t_current_frame_tx_ns;
  return offset_ns_ma_(offset);
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

RigidBodyMeasurement VICON::getData(const std::string& name)
{
  RigidBodyMeasurement body(name);
  
  // we only care about segment 0 (TODO: same as GetRootSegment?)
  constexpr unsigned int SegIdx = 0;
  std::string SegName = client_.GetSegmentName(name, SegIdx).SegmentName;

  { // position of root segment w.r.t global VICON frame
    auto data = client_.GetSegmentGlobalTranslation(name, SegName);
    body.x = data.Translation[0]*1e-3;
    body.y = data.Translation[1]*1e-3;
    body.z = data.Translation[2]*1e-3;

    // true if segment was absent in this frame
    body.occluded = data.Occluded;
  }

  { // orientation of root segment w.r.t global VICON frame
    auto data = client_.GetSegmentGlobalRotationQuaternion(name, SegName);
    body.qw = data.Rotation[3];
    body.qx = data.Rotation[0];
    body.qy = data.Rotation[1];
    body.qz = data.Rotation[2];
  }

  // get total number of markers
  body.nrMarkers = client_.GetMarkerCount(SegName).MarkerCount;

  // get number of visible markers
  body.nrVisibleMarkers = 0;
  for (size_t i=0; i<body.nrMarkers; ++i) {
    std::string mName = client_.GetMarkerName(SegName, i).MarkerName;
    Marker m(mName);

    // get position of markers w.r.t global VICON frame
    auto data = client_.GetMarkerGlobalTranslation(SegName, mName);
    m.x = data.Translation[0]*1e-3;
    m.y = data.Translation[1]*1e-3;
    m.z = data.Translation[2]*1e-3;

    // true if marker was absent in this frame
    m.occluded = data.Occluded;

    // add marker to body
    body.markers.push_back(m);

    if (!data.Occluded) body.nrVisibleMarkers++;
  }

  return body;
}

} // ns mocap
} // ns acl
