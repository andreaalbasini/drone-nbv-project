include(FetchContent)
FetchContent_Declare(vicon_datastream_sdk
    GIT_REPOSITORY https://github.com/whoenig/vicon-datastream-sdk
    GIT_TAG 1.11.0_improved_build
    PATCH_COMMAND git apply --ignore-whitespace "${CMAKE_CURRENT_LIST_DIR}/expose_receive_pkt_time.patch")
FetchContent_MakeAvailable(vicon_datastream_sdk)

add_library(vicon_sdk ALIAS ViconDataStreamSDK_CPP)