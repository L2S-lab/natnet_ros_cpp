#include "ros/ros.h"
#include "natnet_ros_cpp/MarkerPoses.h"
#include "set_param.h"
#include <iostream>
#include <vector>

#include <NatNetCAPI.h>
#include <NatNetClient.h>
natnet_ros_cpp::MarkerPoses::Response response;

class GetMarkerPosesServer
{
    public:
    GetMarkerPosesServer(ros::NodeHandle nh_);
    void startServer();

    private:
    ros::NodeHandle nh_;
    ros::ServiceServer service_;
    NatNetClient* natnet_client_ = new NatNetClient();
    SetParam params;
    
    sNatNetClientConnectParams g_connectParams;
    ConnectionType kDefaultConnectionType = ConnectionType_Multicast;

    static void FrameCallback(sFrameOfMocapData *data,void* pUserData);

    bool update(natnet_ros_cpp::MarkerPoses::Request &req,
                        natnet_ros_cpp::MarkerPoses::Response &res);
};

GetMarkerPosesServer::GetMarkerPosesServer(ros::NodeHandle n)
{
    this->nh_ = n;
}
void GetMarkerPosesServer::startServer()
{
    // Create ROS service
    service_ = nh_.advertiseService("get_marker_position", &GetMarkerPosesServer::update, this);
}

void GetMarkerPosesServer::FrameCallback(sFrameOfMocapData *data,void* pUserData)
{
    response.x_position.clear();
    response.y_position.clear();
    response.z_position.clear();
    response.num_of_markers=0;
    for (int i = 0; i < data->nLabeledMarkers; i++) {
        bool bUnlabeled = ((data->LabeledMarkers[i].params & 0x10) != 0);
        if (bUnlabeled) {
            //ROS_INFO("n: %i, x: %f, y: %f, z: %f", data->nLabeledMarkers, data->LabeledMarkers[i].x, data->LabeledMarkers[i].y, data->LabeledMarkers[i].z);
            response.x_position.push_back(data->LabeledMarkers[i].x);
            response.y_position.push_back(data->LabeledMarkers[i].y);
            response.z_position.push_back(data->LabeledMarkers[i].z);
            response.num_of_markers+=1;
        }
    }
};

bool GetMarkerPosesServer::update(natnet_ros_cpp::MarkerPoses::Request &req,
                        natnet_ros_cpp::MarkerPoses::Response &res)
{
    natnet_client_->Disconnect();

    params.getConnectionParams(nh_);
    if (params.serverType == "unicast")
        kDefaultConnectionType = ConnectionType_Unicast;
    g_connectParams.connectionType = kDefaultConnectionType;
    g_connectParams.serverCommandPort = params.serverCommandPort;
    g_connectParams.serverDataPort = params.serverDataPort;
    g_connectParams.serverAddress = params.serverIP.c_str();
    g_connectParams.localAddress = params.clientIP.c_str();
    g_connectParams.multicastAddress = params.serverType=="multicast" ? params.multicastAddress.c_str() : NULL;

    int iResult = natnet_client_->Connect(g_connectParams);
    if (iResult != ErrorCode_OK)
    {
        ROS_ERROR("Error connecting client. Exiting. Try again");
        return 1;
    }
    else
    {
        ROS_INFO("Client initialized and ready.");
    }

    auto _start = std::chrono::steady_clock::now();
    while (ros::ok())
    {
        ros::Time begin = ros::Time::now();
        natnet_client_->SetFrameReceivedCallback(FrameCallback,natnet_client_);
        ros::Duration(0.5).sleep();
        res = response;
        natnet_client_->Disconnect();
        return true;
    }
    natnet_client_->Disconnect();
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "marker_poses_server");
    ros::NodeHandle n;
    GetMarkerPosesServer server(n);
    server.startServer();
    ros::spin();
    return 0;
}