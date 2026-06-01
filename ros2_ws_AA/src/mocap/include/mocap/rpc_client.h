/**
 * @file rpc_client.h
 * @brief RPC client to access model markers (VSK) and camera calibration (XCD)
 * @author Parker Lusk <plusk@mit.edu>
 * @date 12 March 2022
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "mocap/client.h"

namespace acl {
namespace mocap {

/**
 * @brief      Wrapper for RPC communication with an XML-RPC server
 */
class RPCClient
{
public:
  RPCClient(const std::string& uri, bool enabled);
  ~RPCClient() = default;

  /**
   * @brief      Returns a vector of calibrated camera objects
   *
   * @return     The cameras.
   */
  std::vector<Camera> getCameras();

  /**
   * @brief      Returns a vector of model markers
   *
   * @param[in]  modelname  The rigid body model name to look up
   *
   * @return     The model markers.
   */
  std::vector<Marker> getModelMarkers(const std::string& modelname);

private:
  std::string uri_;
  bool enabled_;
};

using RPCClientPtr = std::shared_ptr<RPCClient>;

} // ns mocap
} // ns acl