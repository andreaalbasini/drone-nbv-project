/**
 * MIT License
 * 
 * Copyright (c) 2018 AgileDrones
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * Originally from: https://github.com/mit-fast/OptiTrack-Motive-2-Client
 * @editedby Parker Lusk <parkerclusk@gmail.com>
 */

#include "optitrack_sdk/optitrack_client.h"

namespace agile {

OptiTrackClient::OptiTrackClient(const std::string& localIP,
                                 const std::string& serverIP,
                                 const std::string& multicastGroupIP,
                                 const int commandPort,
                                 const int dataPort)
: localIP_(localIP), serverIP_(serverIP), multicastIP_(multicastGroupIP),
  commandPort_(commandPort), dataPort_(dataPort) {}

// ----------------------------------------------------------------------------

bool OptiTrackClient::initConnection() {
  
  try {
    cmdsock_.reset(new acl::utils::UDPSocket(localIP_, 0));
    cmdsock_->setReceiveTimeout(0,500000); // 500 msec

    datasock_.reset(new acl::utils::UDPSocket(dataPort_));
    datasock_->setReceiveTimeout(0,500000); // 500 msec
    datasock_->joinMulticastGroup(localIP_, multicastIP_);
  } catch (const std::exception &exc) {
    std::cerr << exc.what() << std::endl;;
    return false;
  }

  // attempt to connect to the server to retrieve basic info
  return getServerInfo(serverInfo_);
}

// ----------------------------------------------------------------------------

bool OptiTrackClient::spinOnce()
{
  // clear previous vector of processed packets
  processedPackets_.clear();

  // Receive one chunk of data (sRigidBodyData --- pose, mean error)
  {
    char data[MAX_PACKETSIZE];
    bool recvd = datasock_->receive(data, sizeof(data));

    if (recvd) Unpack(data, processedPackets_);
    else return false;
  }

  // Request current model descriptions from server (sRigidBodyDescription)
  {
    sPacket pkt{};
    pkt.iMessage = NAT_REQUEST_MODELDEF;
    pkt.nDataBytes = 0;
    const size_t pktlen = pkt.nDataBytes + 4;

    bool sent = cmdsock_->send(serverIP_, commandPort_, (char *)&pkt, pktlen);

    if (!sent) return false;
  }

  // Receive one chunk of command response (sRigidBodyDescription --- name)
  {
    char data[MAX_PACKETSIZE];
    bool recvd = cmdsock_->receive(data, sizeof(data));

    if (recvd) Unpack(data, processedPackets_);
    else return false;
  }

  return true;
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

bool OptiTrackClient::getServerInfo(sSender_Server& serverInfo)
{
  constexpr int MAX_NUM_TRIES = 3;

  // attempt to send the connection request to the server nrTries times.
  int nrTries = MAX_NUM_TRIES;
  while (nrTries--) {

    //
    // send a message through the socket (non-blocking)
    // 

    {
      // n.b.: the 4 is the size of the packet "header" (iMessage + nDataBytes)
      // and the nDataBytes is the actual size of the payload
      // create a packet with a connection request message
      sPacket pkt{};
      pkt.iMessage = NAT_CONNECT;
      pkt.nDataBytes = 0;
      const size_t pktlen = pkt.nDataBytes + 4;

      bool sent = cmdsock_->send(serverIP_, commandPort_, (char *)&pkt, pktlen);
      if (!sent) return false;
    }

    std::cout << "[OptiTrackClient] Attempting to connect "
                 "to OptiTrack server..." << std::flush;

    {
      sPacket pkt{};

      // wait (with timeout) for server response
      bool recvd = cmdsock_->receive((char *)&pkt, sizeof(pkt));

      if (!recvd) {
        std::cout << "timed out." << std::endl;
        continue;
      } else {
        std::cout << "done!" << std::endl;
      }

      // TODO: not sure why I can't just use the union def inside sPacket...
      unsigned char *ptr = (unsigned char *) &pkt;
      agile::sSender_Server *server_info = (agile::sSender_Server *) (ptr + 4);
      serverInfo = *server_info;

      // TODO: broken?
      // std::cout << "NatNet version: " << server_info->Common.NatNetVersion[0] << "."
      //                                 << server_info->Common.NatNetVersion[1] << "."
      //                                 << server_info->Common.NatNetVersion[2] << "."
      //                                 << server_info->Common.NatNetVersion[3] << std::endl;
      
      // serverInfo = pkt.Data.SenderServer;
    }

    return true;
  }

  // number of tries exceeded
  return false;
}

// ----------------------------------------------------------------------------

uint64_t OptiTrackClient::getTimestamp()
{
  return std::chrono::high_resolution_clock::now().time_since_epoch() / std::chrono::nanoseconds(1);
}

// ----------------------------------------------------------------------------

void OptiTrackClient::Unpack(char *pData, std::vector<Packet> &outputs) {
  // Checks for NatNet Version number. Used later in function.
  // Packets may be different depending on NatNet version.
  // TODO: Check 'getServerInformation' about why this is hard coded
  int major; // serverInfo_.Common.NatNetVersion[0];
  int minor; // serverInfo_.Common.NatNetVersion[1];
  getVersion(major, minor);

  char *ptr = pData;

  // printf("Begin Packet\n-------\n");

  uint64_t receiveTimestamp = getTimestamp();
  // First 2 Bytes is message ID
  int MessageID = 0;
  memcpy(&MessageID, ptr, 2);
  ptr += 2;


  // Second 2 Bytes is the size of the packet
  int nBytes = 0;
  memcpy(&nBytes, ptr, 2);
  ptr += 2;

  // printf("Message ID: %d\n", MessageID);
  
  // For Frame of Data definition and structure, see
  // https://v22.wiki.optitrack.com/index.php?title=NatNet:_Data_Types#Frame_of_Mocap_Data

  if (MessageID == NAT_FRAMEOFDATA) {
    // printf("Starting mocap data packet\n");
    // Next 4 Bytes is the frame number
    int frameNumber = 0;
    memcpy(&frameNumber, ptr, 4);
    ptr += 4;
    // printf("Frame # : %d\n", frameNumber);


    // Next 4 Bytes is the number of data sets (markersets, rigidbodies, etc)
    int nMarkerSets = 0;
    memcpy(&nMarkerSets, ptr, 4);
    ptr += 4;

    //
    // Marker Sets
    //

    // Loop through number of marker sets and get name and data
    for (int i = 0; i < nMarkerSets; i++) {
      // Markerset name
      char szName[256];
      strcpy(szName, ptr);
      int nDataBytes = (int) strlen(szName) + 1;
      ptr += nDataBytes;

      // marker data
      int nMarkers = 0;
      memcpy(&nMarkers, ptr, 4);
      ptr += 4;
      // printf("Marker Count : %d\n", nMarkers);

      for (int j = 0; j < nMarkers; j++) {
        float x = 0;
        memcpy(&x, ptr, 4);
        ptr += 4;
        float y = 0;
        memcpy(&y, ptr, 4);
        ptr += 4;
        float z = 0;
        memcpy(&z, ptr, 4);
        ptr += 4;
        // printf("\tMarker %d : [x=%3.2f,y=%3.2f,z=%3.2f]\n", j, x, y, z);
      }
    }

    //
    // Unlabeled Markers
    //

    // Loop through unlabeled markers
    int nOtherMarkers = 0;
    memcpy(&nOtherMarkers, ptr, 4);
    ptr += 4;
    // OtherMarker list is Deprecated
    // // printf("Unidentified Marker Count : %d\n", nOtherMarkers);
    for (int j = 0; j < nOtherMarkers; j++) {
      float x = 0.0f;
      memcpy(&x, ptr, 4);
      ptr += 4;
      float y = 0.0f;
      memcpy(&y, ptr, 4);
      ptr += 4;
      float z = 0.0f;
      memcpy(&z, ptr, 4);
      ptr += 4;

      // Deprecated
      // // printf("\tMarker %d : pos = [%3.2f,%3.2f,%3.2f]\n",j,x,y,z);
    }

    //
    // Rigid Bodies
    //

    // Loop through rigidbodies
    int nRigidBodies = 0;
    memcpy(&nRigidBodies, ptr, 4);
    ptr += 4;

    // printf("Rigid Body Count : %d\n", nRigidBodies);
    for (int j = 0; j < nRigidBodies; j++) {
      // Rigid body position and orientation
      int ID = 0;
      memcpy(&ID, ptr, 4);
      ptr += 4;
      // printf("Rigid body ID: %d\n", ID);
      float x = 0.0f;
      memcpy(&x, ptr, 4);
      ptr += 4;
      float y = 0.0f;
      memcpy(&y, ptr, 4);
      ptr += 4;
      float z = 0.0f;
      memcpy(&z, ptr, 4);
      ptr += 4;
      float qx = 0;
      memcpy(&qx, ptr, 4);
      ptr += 4;
      float qy = 0;
      memcpy(&qy, ptr, 4);
      ptr += 4;
      float qz = 0;
      memcpy(&qz, ptr, 4);
      ptr += 4;
      float qw = 0;
      memcpy(&qw, ptr, 4);
      ptr += 4;
      // printf("ID : %d\n", ID);
      // printf("pos: [%3.2f,%3.2f,%3.2f]\n", x, y, z);
      // printf("ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", qx, qy, qz, qw);

      Packet output_packet;
      output_packet.receive_timestamp = receiveTimestamp;
      output_packet.message_id = MessageID;
      output_packet.frame_number = frameNumber;

      output_packet.rigid_body_id = ID;
      output_packet.pos[0] = x;
      output_packet.pos[1] = y;
      output_packet.pos[2] = z;
      output_packet.orientation[0] = qx;
      output_packet.orientation[1] = qy;
      output_packet.orientation[2] = qz;
      output_packet.orientation[3] = qw;

      // // Before NatNet 3.0, marker data was here
      // if (major < 3) {
      //   // associated marker positions
      //   int nRigidMarkers = 0;
      //   memcpy(&nRigidMarkers, ptr, 4);
      //   ptr += 4;
      //   // printf("Marker Count: %d\n", nRigidMarkers);
      //   int nBytes = nRigidMarkers * 3 * sizeof(float);
      //   float *markerData = (float *) malloc(nBytes);
      //   memcpy(markerData, ptr, nBytes);
      //   ptr += nBytes;

      //   if (major >= 2) {
      //     // associated marker IDs
      //     nBytes = nRigidMarkers * sizeof(int);
      //     int *markerIDs = (int *) malloc(nBytes);
      //     memcpy(markerIDs, ptr, nBytes);
      //     ptr += nBytes;

      //     // associated marker sizes
      //     nBytes = nRigidMarkers * sizeof(float);
      //     float *markerSizes = (float *) malloc(nBytes);
      //     memcpy(markerSizes, ptr, nBytes);
      //     ptr += nBytes;

      //     for (int k = 0; k < nRigidMarkers; k++) {
      //       // printf("\tMarker %d: id=%d\tsize=%3.1f\tpos=[%3.2f,%3.2f,%3.2f]\n",
      //              // k,
      //              // markerIDs[k],
      //              // markerSizes[k],
      //              // markerData[k * 3],
      //              // markerData[k * 3 + 1],
      //              // markerData[k * 3 + 2]);
      //     }

      //     if (markerIDs)
      //       free(markerIDs);
      //     if (markerSizes)
      //       free(markerSizes);

      //   } else {
      //     for (int k = 0; k < nRigidMarkers; k++) {
      //       // printf("\tMarker %d: pos = [%3.2f,%3.2f,%3.2f]\n", k,
      //              // markerData[k * 3], markerData[k * 3 + 1],
      //              // markerData[k * 3 + 2]);
      //     }
      //   }
      //   if (markerData)
      //     free(markerData);
      // }

      // NatNet version 2.0 and later
      if (major >= 2) {
        // Mean marker error
        float fError = 0.0f;
        memcpy(&fError, ptr, 4);
        ptr += 4;
        // printf("Mean marker error: %3.2f\n", fError);

        output_packet.mean_marker_error = fError;
      }

      // NatNet version 2.6 and later
      if (((major == 2) && (minor >= 6)) || (major > 2)) {
        // params
        short params = 0;
        memcpy(&params, ptr, 2);
        ptr += 2;
        // 0x01 : rigid body was successfully tracked in this frame
        const bool bTrackingValid = params & 0x01;
        output_packet.tracking_valid = bTrackingValid;
      }

      outputs.push_back(output_packet);

    } // Go to next rigid body

    //
    // Skeletons (NatNet version 2.1 and later)
    //

    if (((major == 2) && (minor > 0)) || (major > 2)) {
      int nSkeletons = 0;
      memcpy(&nSkeletons, ptr, 4);
      ptr += 4;
      // printf("Skeleton Count : %d\n", nSkeletons);

      // Loop through skeletons
      for (int j = 0; j < nSkeletons; j++) {
        // skeleton id
        int skeletonID = 0;
        memcpy(&skeletonID, ptr, 4);
        ptr += 4;

        // Number of rigid bodies (bones) in skeleton
        int nRigidBodies = 0;
        memcpy(&nRigidBodies, ptr, 4);
        ptr += 4;
        // printf("Rigid Body Count : %d\n", nRigidBodies);

        // Loop through rigid bodies (bones) in skeleton
        for (int j = 0; j < nRigidBodies; j++) {
          // Rigid body position and orientation
          int ID = 0;
          memcpy(&ID, ptr, 4);
          ptr += 4;
          float x = 0.0f;
          memcpy(&x, ptr, 4);
          ptr += 4;
          float y = 0.0f;
          memcpy(&y, ptr, 4);
          ptr += 4;
          float z = 0.0f;
          memcpy(&z, ptr, 4);
          ptr += 4;
          float qx = 0;
          memcpy(&qx, ptr, 4);
          ptr += 4;
          float qy = 0;
          memcpy(&qy, ptr, 4);
          ptr += 4;
          float qz = 0;
          memcpy(&qz, ptr, 4);
          ptr += 4;
          float qw = 0;
          memcpy(&qw, ptr, 4);
          ptr += 4;
          // printf("ID : %d\n", ID);
          // printf("pos: [%3.2f,%3.2f,%3.2f]\n", x, y, z);
          // printf("ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", qx, qy, qz, qw);

          // Before NatNet 3.0, marker data was here
          if (major < 3) {
            // associated marker positions
            int nRigidMarkers = 0;
            memcpy(&nRigidMarkers, ptr, 4);
            ptr += 4;
            // printf("Marker Count: %d\n", nRigidMarkers);
            int nBytes = nRigidMarkers * 3 * sizeof(float);
            float *markerData = (float *) malloc(nBytes);
            memcpy(markerData, ptr, nBytes);
            ptr += nBytes;

            if (major >= 2) {
              // associated marker IDs
              nBytes = nRigidMarkers * sizeof(int);
              int *markerIDs = (int *) malloc(nBytes);
              memcpy(markerIDs, ptr, nBytes);
              ptr += nBytes;

              // associated marker sizes
              nBytes = nRigidMarkers * sizeof(float);
              float *markerSizes = (float *) malloc(nBytes);
              memcpy(markerSizes, ptr, nBytes);
              ptr += nBytes;

              for (int k = 0; k < nRigidMarkers; k++) {
                // printf("\tMarker %d: id=%d\tsize=%3.1f\tpos=[%3.2f,%3.2f,%3.2f]\n",
                       // k,
                       // markerIDs[k],
                       // markerSizes[k],
                       // markerData[k * 3],
                       // markerData[k * 3 + 1],
                       // markerData[k * 3 + 2]);
              }

              if (markerIDs)
                free(markerIDs);
              if (markerSizes)
                free(markerSizes);

            } else {
              for (int k = 0; k < nRigidMarkers; k++) {
                // printf("\tMarker %d: pos = [%3.2f,%3.2f,%3.2f]\n", k,
                       // markerData[k * 3], markerData[k * 3 + 1],
                       // markerData[k * 3 + 2]);
              }
            }
            if (markerData)
              free(markerData);
          }

          // Mean marker error (NatNet version 2.0 and later)
          if (major >= 2) {
            float fError = 0.0f;
            memcpy(&fError, ptr, 4);
            ptr += 4;
            // printf("Mean marker error: %3.2f\n", fError);
          }

          // Tracking flags (NatNet version 2.6 and later)
          if (((major == 2) && (minor >= 6)) || (major > 2)) {
            // params
            short params = 0;
            memcpy(&params, ptr, 2);
            ptr += 2;
            // 0x01 : rigid body was successfully tracked in this frame
            bool bTrackingValid = params & 0x01;
            // output_packet_.tracking_valid = bTrackingValid;
          }

        } // next rigid body

      } // next skeleton
    }

    //
    // Labeled Markers
    //

    if (((major == 2) && (minor >= 3)) || (major > 2)) {
      int nLabeledMarkers = 0;
      memcpy(&nLabeledMarkers, ptr, 4);
      ptr += 4;

      // Loop through labeled markers
      for (int j = 0; j < nLabeledMarkers; j++) {
        // id
        // Marker ID Scheme:
        // Active Markers:
        //   ID = ActiveID, correlates to RB ActiveLabels list
        // Passive Markers:
        //   If Asset with Legacy Labels
        //      AssetID   (Hi Word)
        //      MemberID  (Lo Word)
        //   Else
        //      PointCloud ID
        int ID = 0;
        memcpy(&ID, ptr, 4);
        ptr += 4;
        int modelID, markerID;
        DecodeMarkerID(ID, &modelID, &markerID);

        // x
        float x = 0.0f;
        memcpy(&x, ptr, 4);
        ptr += 4;
        // y
        float y = 0.0f;
        memcpy(&y, ptr, 4);
        ptr += 4;
        // z
        float z = 0.0f;
        memcpy(&z, ptr, 4);
        ptr += 4;
        // size
        float size = 0.0f;
        memcpy(&size, ptr, 4);
        ptr += 4;

        // NatNet version 2.6 and later
        if (((major == 2) && (minor >= 6)) || (major > 2)) {
          // marker params
          short params = 0;
          memcpy(&params, ptr, 2);
          ptr += 2;
          // marker was not visible (occluded) in this frame
          bool bOccluded = (params & 0x01) != 0;
          // position provided by point cloud solve
          bool bPCSolved = (params & 0x02) != 0;
          // position provided by model solve
          bool bModelSolved = (params & 0x04) != 0;
          if (major >= 3) {
            // marker has an associated model
            bool bHasModel = (params & 0x08) != 0;
            // marker is an unlabeled marker
            bool bUnlabeled = (params & 0x10) != 0;
            // marker is an active marker
            bool bActiveMarker = (params & 0x20) != 0;
          }

        }

        // NatNet version 3.0 and later
        float residual = 0.0f;
        if (major >= 3) {
          // Marker residual
          memcpy(&residual, ptr, 4);
          ptr += 4;
        }

        Marker marker_;
        marker_.id = markerID;
        marker_.residual = residual;
        marker_.size = size;
        marker_.x = x;
        marker_.y = y;
        marker_.z = z;

        // output_packet_.markers_.push_back(marker_);
        // printf("ID  : [MarkerID: %d] [ModelID: %d]\n", markerID, modelID);
        // printf("pos : [%3.2f,%3.2f,%3.2f]\n", x, y, z);
        // printf("size: [%3.2f]\n", size);
        // printf("err:  [%3.2f]\n", residual);
      }
    }

    //
    // Force Plate data (NatNet version 2.9 and later)
    //

    if (((major == 2) && (minor >= 9)) || (major > 2)) {
      int nForcePlates;
      memcpy(&nForcePlates, ptr, 4);
      ptr += 4;
      for (int iForcePlate = 0; iForcePlate < nForcePlates; iForcePlate++) {
        // ID
        int ID = 0;
        memcpy(&ID, ptr, 4);
        ptr += 4;
        // printf("Force Plate : %d\n", ID);

        // Channel Count
        int nChannels = 0;
        memcpy(&nChannels, ptr, 4);
        ptr += 4;

        // Channel Data
        for (int i = 0; i < nChannels; i++) {
          // printf(" Channel %d : ", i);
          int nFrames = 0;
          memcpy(&nFrames, ptr, 4);
          ptr += 4;
          for (int j = 0; j < nFrames; j++) {
            float val = 0.0f;
            memcpy(&val, ptr, 4);
            ptr += 4;
            printf("%3.2f   ", val);
          }
          printf("\n");
        }
      }
    }

    //
    // Device data (NatNet version 3.0 and later)
    //

    if (((major == 2) && (minor >= 11)) || (major > 2)) {
      int nDevices;
      memcpy(&nDevices, ptr, 4);
      ptr += 4;
      for (int iDevice = 0; iDevice < nDevices; iDevice++) {
        // ID
        int ID = 0;
        memcpy(&ID, ptr, 4);
        ptr += 4;
        // printf("Device : %d\n", ID);

        // Channel Count
        int nChannels = 0;
        memcpy(&nChannels, ptr, 4);
        ptr += 4;

        // Channel Data
        for (int i = 0; i < nChannels; i++) {
          // printf(" Channel %d : ", i);
          int nFrames = 0;
          memcpy(&nFrames, ptr, 4);
          ptr += 4;
          for (int j = 0; j < nFrames; j++) {
            float val = 0.0f;
            memcpy(&val, ptr, 4);
            ptr += 4;
            printf("%3.2f   ", val);
          }
          printf("\n");
        }
      }
    }

    //
    // software latency (removed in version 3.0)
    //

    if (major < 3) {
      float softwareLatency = 0.0f;
      memcpy(&softwareLatency, ptr, 4);
      ptr += 4;
      // printf("software latency : %3.3f\n", softwareLatency);
    }

    //
    // timecode
    //

    unsigned int timecode = 0;
    memcpy(&timecode, ptr, 4);
    ptr += 4;
    unsigned int timecodeSub = 0;
    memcpy(&timecodeSub, ptr, 4);
    ptr += 4;
    char szTimecode[128] = "";
    TimecodeStringify(timecode, timecodeSub, szTimecode, 128);

    // timestamp
    double timestamp = 0.0f;

    // NatNet version 2.7 and later - increased from single to double precision
    if (((major == 2) && (minor >= 7)) || (major > 2)) {
      memcpy(&timestamp, ptr, 8);
      ptr += 8;
    } else {
      float fTemp = 0.0f;
      memcpy(&fTemp, ptr, 4);
      ptr += 4;
      timestamp = (double) fTemp;
    }

    // Convert to nanoseconds: timestamp since software start
    latest_server_timestamp_ns_ = timestamp * 1e9;

    //
    // high res timestamps (version 3.0 and later)
    //

    if (major >= 3) {
      uint64_t cameraMidExposureTimestamp = 0;
      memcpy(&cameraMidExposureTimestamp, ptr, 8);
      ptr += 8;
      // printf("Mid-exposure timestamp : %" PRIu64 "\n",
      //        cameraMidExposureTimestamp);

      uint64_t cameraDataReceivedTimestamp = 0;
      memcpy(&cameraDataReceivedTimestamp, ptr, 8);
      ptr += 8;
      // printf("Camera data received timestamp : %" PRIu64 "\n",
      //        cameraDataReceivedTimestamp);

      uint64_t transmitTimestamp = 0;
      memcpy(&transmitTimestamp, ptr, 8);
      ptr += 8;
      // printf("Transmit timestamp : %" PRIu64 "\n", transmitTimestamp);

      // Convert timestamps to nanoseconds and save them in the output packets
      const uint64_t server_frequency = serverInfo_.HighResClockFrequency;
      for(int i = 0; i < outputs.size(); i++){
        outputs[i].mid_exposure_timestamp = (cameraMidExposureTimestamp*1e9)/server_frequency;
        outputs[i].camera_data_received_timestamp = (cameraDataReceivedTimestamp*1e9)/server_frequency;
        outputs[i].transmit_timestamp = (transmitTimestamp*1e9)/server_frequency; 
      }
    }

    //
    // frame params
    //

    short params = 0;
    memcpy(&params, ptr, 2);
    ptr += 2;
    // 0x01 Motive is recording
    const bool bIsRecording = (params & 0x01) != 0;
    // 0x02 Actively tracked model list has changed
    const bool bTrackedModelsChanged = (params & 0x02) != 0;


    // end of data tag
    int eod = 0;
    memcpy(&eod, ptr, 4);
    ptr += 4;
    // printf("End Packet\n-------------\n");

  } else if (MessageID == NAT_MODELDEF) { // Data Descriptions

    // Used to convert rigid body ID to model name
    std::unordered_map<int, std::string> rigid_body_map;
    std::unordered_map<int, std::vector<Marker>> markers_map;

    // number of datasets
    int nDatasets = 0;
    memcpy(&nDatasets, ptr, 4);
    ptr += 4;

    for (size_t i = 0; i < nDatasets; i++) {
      int type = 0;
      memcpy(&type, ptr, 4);
      ptr += 4;


      if (type == 0) {   // markerset
        // name
        char szName[256];
        strcpy(szName, ptr);
        int nDataBytes = (int) strlen(szName) + 1;
        ptr += nDataBytes;
        

        // marker data
        int nMarkers = 0;
        memcpy(&nMarkers, ptr, 4);
        ptr += 4;
        

        for (int j = 0; j < nMarkers; j++) {
          char szName[256];
          strcpy(szName, ptr);
          int nDataBytes = (int) strlen(szName) + 1;
          ptr += nDataBytes;
          
        }
      } else if (type == 1) {  // rigid body
        char szName[MAX_NAMELENGTH];  
        if (major >= 2) {
          // name
          strcpy(szName, ptr);
          ptr += strlen(ptr) + 1;
        }

        int ID = 0;
        memcpy(&ID, ptr, 4);
        ptr += 4;
        

        // Add rigid body - name pair
        rigid_body_map[ID] = szName;

        int parentID = 0;
        memcpy(&parentID, ptr, 4);
        ptr += 4;

        float xoffset = 0;
        memcpy(&xoffset, ptr, 4);
        ptr += 4;

        float yoffset = 0;
        memcpy(&yoffset, ptr, 4);
        ptr += 4;

        float zoffset = 0;
        memcpy(&zoffset, ptr, 4);
        ptr += 4;

        // Per-marker data (NatNet 3.0 and later)
        if (major >= 3) {
          int nMarkers = 0;
          memcpy(&nMarkers, ptr, 4);
          ptr += 4;

          // Marker positions
          nBytes = nMarkers * 3 * sizeof(float);
          float *markerPositions = (float *) malloc(nBytes);
          memcpy(markerPositions, ptr, nBytes);
          ptr += nBytes;

          // Marker required active labels
          nBytes = nMarkers * sizeof(int);
          int *markerRequiredLabels = (int *) malloc(nBytes);
          memcpy(markerRequiredLabels, ptr, nBytes);
          ptr += nBytes;

          for (int markerIdx = 0; markerIdx < nMarkers; ++markerIdx) {
            float *markerPosition = markerPositions + markerIdx * 3;
            const int markerRequiredLabel = markerRequiredLabels[markerIdx];

            Marker m;
            m.id = markerIdx;
            m.x = markerPosition[0];
            m.y = markerPosition[1];
            m.z = markerPosition[2];
            markers_map[ID].push_back(m);
          }

          free(markerPositions);
          free(markerRequiredLabels);
        }

      } else if (type == 2)   // skeleton
      {
        char szName[MAX_NAMELENGTH];
        strcpy(szName, ptr);
        ptr += strlen(ptr) + 1;
        

        int ID = 0;
        memcpy(&ID, ptr, 4);
        ptr += 4;
        

        int nRigidBodies = 0;
        memcpy(&nRigidBodies, ptr, 4);
        ptr += 4;
        

        for (int i = 0; i < nRigidBodies; i++) {
          if (major >= 2) {
            // RB name
            char szName[MAX_NAMELENGTH];
            strcpy(szName, ptr);
            ptr += strlen(ptr) + 1;
          }

          int ID = 0;
          memcpy(&ID, ptr, 4);
          ptr += 4;

          int parentID = 0;
          memcpy(&parentID, ptr, 4);
          ptr += 4;

          float xoffset = 0;
          memcpy(&xoffset, ptr, 4);
          ptr += 4;

          float yoffset = 0;
          memcpy(&yoffset, ptr, 4);
          ptr += 4;

          float zoffset = 0;
          memcpy(&zoffset, ptr, 4);
          ptr += 4;
        }
      }

    }   // next dataset

    // Add names to rigid bodies
    for (int i = 0; i < outputs.size(); i++) {
      outputs[i].model_name = rigid_body_map[outputs[i].rigid_body_id];
      outputs[i].expected_markers = markers_map[outputs[i].rigid_body_id];
    }
  } else {
    // printf("Unrecognized Packet Type.\n");
  }

}

// ----------------------------------------------------------------------------

// Funtion that assigns a time code values to 5 variables passed as arguments
// Requires an integer from the packet as the timecode and timecodeSubframe
bool OptiTrackClient::DecodeTimecode(unsigned int inTimecode,
                                     unsigned int inTimecodeSubframe,
                                     int *hour, int *minute, int *second,
                                     int *frame, int *subframe)
{
  bool bValid = true;

  *hour = (inTimecode >> 24) & 255;
  *minute = (inTimecode >> 16) & 255;
  *second = (inTimecode >> 8) & 255;
  *frame = inTimecode & 255;
  *subframe = inTimecodeSubframe;

  return bValid;
}

// ----------------------------------------------------------------------------

// Takes timecode and assigns it to a string
bool OptiTrackClient::TimecodeStringify(unsigned int inTimecode,
                                        unsigned int inTimecodeSubframe,
                                        char *Buffer, size_t BufferSize)
{
  bool bValid = false;
  int hour, minute, second, frame, subframe;
  bValid = DecodeTimecode(inTimecode, inTimecodeSubframe,
                          &hour, &minute, &second, &frame, &subframe);

  snprintf(Buffer, BufferSize, "%2d:%2d:%2d:%2d.%d",
           hour, minute, second, frame, subframe);
  for (unsigned int i=0; i<strlen(Buffer); i++)
    if (Buffer[i] == ' ')
      Buffer[i] = '0';

  return bValid;
}

// ----------------------------------------------------------------------------

void OptiTrackClient::DecodeMarkerID(int sourceID, int *pOutEntityID, int *pOutMemberID)
{
  if (pOutEntityID)
    *pOutEntityID = sourceID >> 16;

  if (pOutMemberID)
    *pOutMemberID = sourceID & 0x0000ffff;
}

} // ns agile
