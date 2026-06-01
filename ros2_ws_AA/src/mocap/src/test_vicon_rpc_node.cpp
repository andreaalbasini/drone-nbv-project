/**
 * @file test_vicon_rpc_node.cpp
 * @brief Test communication with our XMLRPC server on vicon computer
 * @author Parker Lusk <plusk@mit.edu>
 * @date 12 March 2022
 */

#include <cstdio>
#include <string>
#include <iostream>
#include <cstdlib>

#include <Eigen/Dense>

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

int main(int argc, char *argv[]) {
  FRPC::Pool_t pool; // memory pool
  FRPC::ServerProxy_t::Config_t config;

  std::string url = "http://192.168.0.9:1250";

  FRPC::ServerProxy_t client(url.c_str(), config);

  try  {
    
    //
    // Cameras
    //

    const FRPC::Array_t& cameras = FRPC::Array(client(pool, "getCameras"));
    std::cout << "Got " << cameras.size() << " cameras:" << std::endl;

    for (const auto& vptr : cameras) {
      const FRPC::Struct_t& camstruct = FRPC::Struct(*vptr);

      const int index = FRPC::Int(camstruct["index"]).getValue();
      const int id = FRPC::Int(camstruct["deviceid"]).getValue();
      const std::string type(FRPC::String(camstruct["type"]).getValue());
      const bool enabled = FRPC::Bool(camstruct["enabled"]).getValue();
      
      std::cout << "  - Camera #" << index << " (" << type << " - " << id << "): ";
      if (enabled) {
        Eigen::VectorXd position = frpcarray2eigen(camstruct["position"]);
        std::cout << position.transpose() << std::endl;
      } else {
        std::cout << "DISABLED" << std::endl;
      }
    }
    std::cout << std::endl << std::endl;

    //
    // Markers
    //

    const std::string modelname("HX18");
    const FRPC::Array_t& markers = FRPC::Array(client(pool, "getModelMarkers", pool.String(modelname)));
    std::cout << "Got " << markers.size() << " markers for " << modelname << std::endl;

    for (const auto& vptr : markers) {
      const FRPC::Struct_t& mstruct = FRPC::Struct(*vptr);

      const std::string name(FRPC::String(mstruct["name"]).getValue());
      const Eigen::VectorXd position = frpcarray2eigen(mstruct["position"]);
      std::cout << "  - " << name << ": " << position.transpose() << std::endl;
    }

  } catch (const FRPC::Fault_t &f) {
    printf("Fault Status:%d Message:%s \n", f.errorNum(), f.message().c_str() );
  } catch (const FRPC::HTTPError_t &e) {
    printf("Http Error Status:%d Message:%s \n", e.errorNum(), e.message().c_str() );
  } catch (const FRPC::StreamError_t &e) {
    printf("Stream Error  Message:%s \n", e.message().c_str());
  }

  return 0;
}
