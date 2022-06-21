
#include "internal.h"
#include "ros/ros.h"

Internal internal;

//Function to make callback to the datahandler when recieve the new frame.
void FrameCallback(sFrameOfMocapData *data, void* pUserData)
{
    auto pClient = reinterpret_cast<Internal *>(pUserData);
    if (internal.rosparam.log_latencies)
        pClient->LatenciInfo(data, pUserData, internal);
    pClient->DataHandler(data, pUserData, internal);
}

int main( int argc, char **argv)
{
    // defining the default connection type ConnectionType_Multicast/ConnectionType_Unicast
    ConnectionType kDefaultConnectionType = ConnectionType_Multicast;
    // variable to handle the parameters to be passed to the natnet
    sNatNetClientConnectParams g_connectParams;
    // variable to store the server description 
    //sServerDescription g_serverDescription;

    ros::init(argc, argv,"natnet_ros_cpp");
    ros::NodeHandle n("~");
    // create NatNet client
    NatNetClient* g_pClient = new NatNetClient();
    // print version info
    unsigned char ver[4];
    NatNet_GetVersion( ver );
    ROS_INFO("NatNet Sample Client (NatNet ver. %d.%d.%d.%d)", ver[0], ver[1], ver[2], ver[3]);
    
    // You must init before using the object to get required parameters from the rosparam server
    internal.Init(n);

    if (internal.rosparam.serverType == "unicast")
        kDefaultConnectionType = ConnectionType_Unicast;

    // Setting up parameters for the natnet connection
    g_connectParams.connectionType = kDefaultConnectionType;
    g_connectParams.serverCommandPort = internal.rosparam.serverCommandPort;
    g_connectParams.serverDataPort = internal.rosparam.serverDataPort;
    g_connectParams.serverAddress = internal.rosparam.serverIP.c_str();
    g_connectParams.localAddress = internal.rosparam.clientIP.c_str();
    g_connectParams.multicastAddress = internal.rosparam.serverType=="multicast" ? internal.rosparam.multicastAddress.c_str() : NULL;

    int iResult;
    // Connect to Motive
    iResult = internal.ConnectClient(g_pClient, g_connectParams);
    if (iResult != ErrorCode_OK)
    {
        ROS_ERROR("Error initializing client. See log for details. Exiting.");
        return 1;
    }
    else
    {
        ROS_INFO("Client initialized and ready.");
    }

    // Assembling the info from the optitrack, counting number of bodies etc..
    // Must extract the info before starting to publish the data.
    internal.Info(g_pClient, n);

    ROS_INFO("Client is connected to server and listening for data...");

    // Install natnet logging callback for some internal details
    internal.rosparam.log_internals ? NatNet_SetLogCallback( internal.MessageHandler ): internal.Pass();

    while(ros::ok())
    {   // set the frame callback handler
        g_pClient->SetFrameReceivedCallback( FrameCallback, g_pClient);  // this function will receive data from the server
    }

    return ErrorCode_OK;
}

