/**
 * @file rpc_client.h
 * @brief RPC client to access model markers (VSK) and camera calibration (XCD)
 * @author Parker Lusk <plusk@mit.edu>
 * @date 12 March 2022
 */

#include "mocap/rpc_client.h"

#include <frpc.h>
#include <frpcserverproxy.h>

#include <frpchttperror.h>
#include <frpcfault.h>
#include <frpcstreamerror.h>

Eigen::VectorXd frpcarray2eigen(const FRPC::Value_t& value)
{
  const FRPC::Array_t& array = FRPC::Array(value);
  Eigen::VectorXd vec = Eigen::VectorXd::Zero(array.size());
  size_t i = 0;
  for (const auto& vptr : array) {
    vec(i++) = FRPC::Double(*vptr).getValue();
  }
  return vec;
}

namespace acl {
namespace mocap {

RPCClient::RPCClient(const std::string& uri, bool enabled)
: uri_(uri), enabled_(enabled)
{}

// ----------------------------------------------------------------------------

std::vector<Camera> RPCClient::getCameras()
{
  if (!enabled_) return {};

  FRPC::Pool_t pool;
  FRPC::ServerProxy_t::Config_t config;
  FRPC::ServerProxy_t client(uri_.c_str(), config);

  std::vector<Camera> cameras;

  try {
    const FRPC::Array_t& camarray = FRPC::Array(client(pool, "getCameras"));

    for (const auto& vptr : camarray) {
      const FRPC::Struct_t& camstruct = FRPC::Struct(*vptr);

      Camera camera;

      // properties that are valid, even if camera is disabled
      camera.enabled = FRPC::Bool(camstruct["enabled"]).getValue();
      camera.index = FRPC::Int(camstruct["index"]).getValue();
      camera.deviceid = FRPC::Int(camstruct["deviceid"]).getValue();
      camera.type = FRPC::String(camstruct["type"]).getValue();
      const FRPC::Array_t& imsizearray = FRPC::Array(camstruct["image_size"]);
      camera.image_width = FRPC::Int(imsizearray[0]).getValue();
      camera.image_height = FRPC::Int(imsizearray[1]).getValue();

      // properties only valid if camera is enabled
      if (camera.enabled) {
        camera.position = frpcarray2eigen(camstruct["position"]);
        camera.orientation = Eigen::Quaterniond(Eigen::Vector4d(frpcarray2eigen(camstruct["orientation"])));
        camera.image_error = FRPC::Double(camstruct["image_error"]).getValue();
        camera.focal_length = FRPC::Double(camstruct["focal_length"]).getValue();
        camera.principal_point = frpcarray2eigen(camstruct["principal_point"]);
        camera.radial_distortion = frpcarray2eigen(camstruct["vicon_radial"]);
      }

      cameras.push_back(camera);
    }

  } catch (const FRPC::Fault_t &f) {
    // printf("Fault Status:%d Message:%s \n", f.errorNum(), f.message().c_str() );
  } catch (const FRPC::HTTPError_t &e) {
    // printf("Http Error Status:%d Message:%s \n", e.errorNum(), e.message().c_str() );
  } catch (const FRPC::StreamError_t &e) {
    // printf("Stream Error Message:%s \n", e.message().c_str());
  }

  return cameras;
}

// ----------------------------------------------------------------------------

std::vector<Marker> RPCClient::getModelMarkers(const std::string& modelname)
{
  if (!enabled_) return {};

  FRPC::Pool_t pool;
  FRPC::ServerProxy_t::Config_t config;
  FRPC::ServerProxy_t client(uri_.c_str(), config);

  std::vector<Marker> markers;

  try {
    const FRPC::Array_t& markerarray = FRPC::Array(client(pool,
                                "getModelMarkers", pool.String(modelname)));

    for (const auto& vptr : markerarray) {
      const FRPC::Struct_t& mstruct = FRPC::Struct(*vptr);

      Eigen::Vector3d position = frpcarray2eigen(mstruct["position"]);

      Marker marker(FRPC::String(mstruct["name"]).getValue());
      marker.x = position.x();
      marker.y = position.y();
      marker.z = position.z();
      marker.occluded = false; // doesn't make sense for the model markers

      markers.push_back(marker);
    }

  } catch (const FRPC::Fault_t &f) {
    // printf("Fault Status:%d Message:%s \n", f.errorNum(), f.message().c_str() );
  } catch (const FRPC::HTTPError_t &e) {
    // printf("Http Error Status:%d Message:%s \n", e.errorNum(), e.message().c_str() );
  } catch (const FRPC::StreamError_t &e) {
    // printf("Stream Error Message:%s \n", e.message().c_str());
  }

  return markers;
}

} // ns mocap
} // ns acl