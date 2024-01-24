#include "internal.h"

void Internal::Init(ros::NodeHandle &n)
{
    this->rosparam.getNset(n);
}

int Internal::ConnectClient(NatNetClient* g_pClient, sNatNetClientConnectParams &g_connectParams)
{
    // Release previous server
    g_pClient->Disconnect();

    // Init Client and connect to NatNet server
    int retCode = g_pClient->Connect( g_connectParams );
    if (retCode != ErrorCode_OK)
    {
        ROS_ERROR("Unable to connect to server.  Error code: %d. Exiting.", retCode);
        return ErrorCode_Internal;
    }
    else
    {
        // connection succeeded
        void* pResult;
        int nBytes = 0;
        ErrorCode ret = ErrorCode_OK;

        // print server info
        memset( &g_serverDescription, 0, sizeof( g_serverDescription ) );
        ret = g_pClient->GetServerDescription( &g_serverDescription );
        if ( ret != ErrorCode_OK || ! g_serverDescription.HostPresent )
        {
            ROS_ERROR("Unable to connect to server. Host not present. Exiting.");
            return 1;
        }
        

        // get mocap frame rate
        ret = g_pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            float fRate = *((float*)pResult);
            ROS_INFO("Mocap Framerate : %3.2f", fRate);
        }
        else
            ROS_ERROR("Error getting frame rate.");

        // get # of analog samples per mocap frame of data
        ret = g_pClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            //g_analogSamplesPerMocapFrame = *((int*)pResult);
            ROS_INFO("Analog Samples Per Mocap Frame : %d", *((int*)pResult));
        }
        else
            ROS_ERROR("Error getting Analog frame rate.");
    }

    return ErrorCode_OK;
}

void Internal::MessageHandler( Verbosity msgType, const char* msg )
{
    // Optional: Filter out debug messages
    if ( msgType < Verbosity_Info )
    {
        return;
    }

    printf( "\n[NatNetLib]" );

    switch ( msgType )
    {
        case Verbosity_Debug:
            printf( " [DEBUG]" );
            break;
        case Verbosity_Info:
            printf( "  [INFO]" );
            break;
        case Verbosity_Warning:
            printf( "  [WARN]" );
            break;
        case Verbosity_Error:
            printf( " [ERROR]" );
            break;
        default:
            printf( " [?????]" );
            break;
    }

    printf( ": %s", msg );
}

void Internal::Info(NatNetClient* g_pClient, ros::NodeHandle &n)
{
    ROS_INFO("[SampleClient] Requesting Data Descriptions...");
    sDataDescriptions* pDataDefs = NULL;
    int iResult = g_pClient->GetDataDescriptionList(&pDataDefs);
    if (iResult != ErrorCode_OK || pDataDefs == NULL)
    {
        ROS_INFO("[SampleClient] Unable to retrieve Data Descriptions.");
    }
    else
    {
        ROS_INFO("[SampleClient] Received %d Data/Devices Descriptions:", pDataDefs->nDataDescriptions );

        for(int i=0; i < pDataDefs->nDataDescriptions; i++)
        {
            if(pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
            {
                // RigidBody
                sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
                ROS_INFO("RigidBody found : %s", pRB->szName);
                ROS_INFO_COND(rosparam.log_internals, "RigidBody ID : %d", pRB->ID);
                ROS_INFO_COND(rosparam.log_internals, "RigidBody Parent ID : %d", pRB->parentID);
                ROS_INFO_COND(rosparam.log_internals, "Parent Offset : %3.2f,%3.2f,%3.2f", pRB->offsetx, pRB->offsety, pRB->offsetz);
                
                // Creating publisher for the rigid bodies if found any
                std::string body_name(pRB->szName);
                if(rosparam.pub_rigid_body)
                {
                    this->ListRigidBodies[pRB->ID] = body_name;
                    this->RigidbodyPub[pRB->szName] = n.advertise<geometry_msgs::PoseStamped>(body_name+"/pose", 50);
                }
                if ( pRB->MarkerPositions != NULL && pRB->MarkerRequiredLabels != NULL )
                {
                    for ( int markerIdx = 0; markerIdx < pRB->nMarkers; ++markerIdx )
                    {
                        const MarkerData& markerPosition = pRB->MarkerPositions[markerIdx];
                        const int markerRequiredLabel = pRB->MarkerRequiredLabels[markerIdx];
                        // Creating publisher for the markers of the rigid bodies
                        if(rosparam.pub_rigid_body_marker)
                            this->RigidbodyMarkerPub[std::to_string(pRB->ID)+std::to_string(markerIdx+1)] = n.advertise<geometry_msgs::PointStamped>(body_name+"/marker"+std::to_string(markerIdx)+"/pose", 50);
                        ROS_INFO_COND(rosparam.log_internals,  "\tMarker #%d:", markerIdx );
                        ROS_INFO_COND(rosparam.log_internals,  "\t\tPosition: %.2f, %.2f, %.2f", markerPosition[0], markerPosition[1], markerPosition[2] );

                        if ( markerRequiredLabel != 0 )
                        {
                            ROS_INFO_COND(rosparam.log_internals,  "\t\tRequired active label: %d", markerRequiredLabel );
                        }
                    }
                }
            }
            else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Camera)
            {
                // Camera
                sCameraDescription* pCamera = pDataDefs->arrDataDescriptions[i].Data.CameraDescription;
                ROS_INFO_COND(rosparam.log_internals, "Camera Name : %s", pCamera->strName);
                ROS_INFO_COND(rosparam.log_internals, "Camera Position (%3.2f, %3.2f, %3.2f)", pCamera->x, pCamera->y, pCamera->z);
                ROS_INFO_COND(rosparam.log_internals, "Camera Orientation (%3.2f, %3.2f, %3.2f, %3.2f)", pCamera->qx, pCamera->qy, pCamera->qz, pCamera->qw);
            }
            else
            {
                ROS_WARN_COND(rosparam.log_internals, "Unknown data type detected.");
                // Unknown
            }
        }
        if (rosparam.pub_pointcloud)
        {
            this->PointcloudPub = n.advertise<sensor_msgs::PointCloud>("pointcloud",50);
        }
        if (rosparam.pub_individual_marker)
        {
            for (int i=0; i<(int) rosparam.object_list.size(); i++)
            {
                this->IndividualMarkerPub[rosparam.object_list[i].name] = n.advertise<geometry_msgs::PoseStamped>(rosparam.object_list[i].name+"/pose", 50);
            }
        }
    }
}

void Internal::LatenciInfo(sFrameOfMocapData* data, void* pUserData, Internal &internal)
{
    NatNetClient* pClient = (NatNetClient*) pUserData;
    // This figure may appear slightly higher than the "software latency" reported in the Motive user interface,
    // because it additionally includes the time spent preparing to stream the data via NatNet.
    const uint64_t softwareLatencyHostTicks = data->TransmitTimestamp - data->CameraDataReceivedTimestamp;
    const double softwareLatencyMillisec = (softwareLatencyHostTicks * 1000) / static_cast<double>(internal.g_serverDescription.HighResClockFrequency);
    // Transit latency is defined as the span of time between Motive transmitting the frame of data, and its reception by the client (now).
    // The SecondsSinceHostTimestamp method relies on NatNetClient's internal clock synchronization with the server using Cristian's algorithm.
    const double transitLatencyMillisec = pClient->SecondsSinceHostTimestamp( data->TransmitTimestamp ) * 1000.0;
    ROS_INFO_COND(internal.rosparam.log_latencies, "Software latency : %.2lf milliseconds", softwareLatencyMillisec);
    // If it's unavailable (for example, with USB camera systems, or during playback), this field will be zero.
    const bool bSystemLatencyAvailable = data->CameraMidExposureTimestamp != 0;
    if ( bSystemLatencyAvailable )
    {
        const uint64_t systemLatencyHostTicks = data->TransmitTimestamp - data->CameraMidExposureTimestamp;
        const double systemLatencyMillisec = (systemLatencyHostTicks * 1000) / static_cast<double>(internal.g_serverDescription.HighResClockFrequency);
        
        const double clientLatencyMillisec = pClient->SecondsSinceHostTimestamp( data->CameraMidExposureTimestamp ) * 1000.0;
        ROS_INFO_COND(internal.rosparam.log_latencies,  "System latency : %.2lf milliseconds", systemLatencyMillisec );
        ROS_INFO_COND(internal.rosparam.log_latencies,  "Total client latency : %.2lf milliseconds (transit time +%.2lf ms)", clientLatencyMillisec, transitLatencyMillisec );
    }
    else
    {
        ROS_INFO_COND(internal.rosparam.log_latencies,  "Transit latency : %.2lf milliseconds", transitLatencyMillisec );
    }
}

void Internal::DataHandler(sFrameOfMocapData* data, void* pUserData, Internal &internal)
{   
    int i=0;
    ROS_INFO_COND(internal.rosparam.log_frames, "FrameID : %d", data->iFrame);

    // Rigid Bodies
    ROS_INFO_COND(internal.rosparam.log_frames, "Rigid Bodies [Count=%d]", data->nRigidBodies);
    for(i=0; i < data->nRigidBodies; i++)
        {
            if(internal.rosparam.pub_rigid_body)
            {
                PubRigidbodyPose(data->RigidBodies[i], internal);
            }
        ROS_INFO_COND(internal.rosparam.log_frames, "Rigid Body [ID=%d  Error=%3.2f]", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError);//, bTrackingValid);
        ROS_INFO_COND(internal.rosparam.log_frames, "x\ty\tz\tqx\tqy\tqz\tqw");
        ROS_INFO_COND(internal.rosparam.log_frames, "%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f",
            data->RigidBodies[i].x, data->RigidBodies[i].y, data->RigidBodies[i].z,
            data->RigidBodies[i].qx, data->RigidBodies[i].qy, data->RigidBodies[i].qz, data->RigidBodies[i].qw);
        }

    // Markers
    for(i=0; i < data->nLabeledMarkers; i++) 
    {  
        ROS_INFO_COND(internal.rosparam.log_frames, "Markers [Count=%i]", i);
        ROS_INFO_COND(internal.rosparam.log_frames, "x\ty\tz");
        ROS_INFO_COND(internal.rosparam.log_frames, "%3.2f\t%3.2f\t%3.2f", data->LabeledMarkers[i].x, data->LabeledMarkers[i].y, data->LabeledMarkers[i].z);
        bool bUnlabeled = ((data->LabeledMarkers[i].params & 0x10) != 0);
        if(internal.rosparam.pub_individual_marker && bUnlabeled)
        {   
            PubMarkerPose(data->LabeledMarkers[i], internal);
        }
        if (internal.rosparam.pub_pointcloud)
        {
            PubPointCloud(data->LabeledMarkers[i], internal);
        }
        if(internal.rosparam.pub_rigid_body_marker && !bUnlabeled)
        {
            PubRigidbodyMarker(data->LabeledMarkers[i], internal);
        }
    }
    if (internal.rosparam.pub_individual_marker)
    {
        internal.rosparam.object_list_prev = internal.rosparam.object_list;
        if(internal.UnlabeledCount < (int)internal.rosparam.object_list.size() && internal.rosparam.error_amp==1.0)
            internal.rosparam.error_amp = internal.rosparam.error_amp*2;
        else   
            internal.rosparam.error_amp = 1.0;
    }
    if(internal.rosparam.pub_pointcloud)
    {
        internal.msgPointcloud.header.frame_id= internal.rosparam.globalFrame;
        internal.msgPointcloud.header.stamp=ros::Time::now();
        internal.PointcloudPub.publish(internal.msgPointcloud);
    }
    internal.msgPointcloud.points.clear() ;
    internal.UnlabeledCount = 0; 
}

void Internal::PubRigidbodyPose(sRigidBodyData &data, Internal &internal)
{
    // Creating a msg to put data related to the rigid body and 
    geometry_msgs::PoseStamped msgRigidBodyPose;
    msgRigidBodyPose.header.frame_id = internal.rosparam.globalFrame;
    msgRigidBodyPose.header.stamp = ros::Time::now();
    msgRigidBodyPose.pose.position.x = data.x;
    msgRigidBodyPose.pose.position.y = data.y;
    msgRigidBodyPose.pose.position.z = data.z;
    msgRigidBodyPose.pose.orientation.x = data.qx;
    msgRigidBodyPose.pose.orientation.y = data.qy;
    msgRigidBodyPose.pose.orientation.z = data.qz;
    msgRigidBodyPose.pose.orientation.w = data.qw;
    internal.RigidbodyPub[internal.ListRigidBodies[data.ID]].publish(msgRigidBodyPose);
    
    // creating tf frame to visualize in the rviz
    static tf2_ros::TransformBroadcaster tfRigidBodies;
    geometry_msgs::TransformStamped msgTFRigidBodies;
    msgTFRigidBodies.header.stamp = ros::Time::now();
    msgTFRigidBodies.header.frame_id = internal.rosparam.globalFrame;
    msgTFRigidBodies.child_frame_id = internal.ListRigidBodies[data.ID];
    msgTFRigidBodies.transform.translation.x = data.x;
    msgTFRigidBodies.transform.translation.y = data.y;
    msgTFRigidBodies.transform.translation.z = data.z;
    msgTFRigidBodies.transform.rotation.x = data.qx;
    msgTFRigidBodies.transform.rotation.y = data.qy;
    msgTFRigidBodies.transform.rotation.z = data.qz;
    msgTFRigidBodies.transform.rotation.w = data.qw;
    tfRigidBodies.sendTransform(msgTFRigidBodies);
    
}

void Internal::PubMarkerPose(sMarker &data, Internal &internal)
{   
    int update = nn_filter(internal.rosparam.object_list, data, internal.rosparam.E,  internal.rosparam.E_x, internal.rosparam.E_y, internal.rosparam.E_z, internal.rosparam.individual_error, internal.rosparam.error_amp);
    if (update>=0)
    {   internal.UnlabeledCount+=1;
        internal.rosparam.object_list[update].detected = true;
        internal.rosparam.object_list[update].x = data.x;
        internal.rosparam.object_list[update].y = data.y;
        internal.rosparam.object_list[update].z = data.z;
    
        geometry_msgs::PoseStamped msgMarkerPose;
        msgMarkerPose.header.frame_id = internal.rosparam.globalFrame;

        msgMarkerPose.header.stamp = ros::Time::now();
        msgMarkerPose.pose.position.x = data.x;
        msgMarkerPose.pose.position.y = data.y;
        msgMarkerPose.pose.position.z = data.z;
        msgMarkerPose.pose.orientation.x = 0.0;
        msgMarkerPose.pose.orientation.y = 0.0;
        msgMarkerPose.pose.orientation.z = 0.0;
        msgMarkerPose.pose.orientation.w = 1.0;
        internal.IndividualMarkerPub[internal.rosparam.object_list[update].name].publish(msgMarkerPose);

        // creating tf frame to visualize in the rviz
        static tf2_ros::TransformBroadcaster tfMarker;
        geometry_msgs::TransformStamped msgTFMarker;
        msgTFMarker.header.stamp = ros::Time::now();
        msgTFMarker.header.frame_id = internal.rosparam.globalFrame;
        msgTFMarker.child_frame_id = internal.rosparam.object_list[update].name;
        msgTFMarker.transform.translation.x = data.x;
        msgTFMarker.transform.translation.y = data.y;
        msgTFMarker.transform.translation.z = data.z;
        msgTFMarker.transform.rotation.x = 0;
        msgTFMarker.transform.rotation.y = 0;
        msgTFMarker.transform.rotation.z = 0;
        msgTFMarker.transform.rotation.w = 1;
        tfMarker.sendTransform(msgTFMarker);
    }
}

void Internal::PubPointCloud(sMarker &data, Internal &internal)
{

    geometry_msgs::Point32 msgPoint;
    msgPoint.x = data.x;
    msgPoint.y = data.y;
    msgPoint.z = data.z;
    internal.msgPointcloud.points.push_back(msgPoint);
}

void Internal::PubRigidbodyMarker(sMarker &data, Internal &internal)
{
    int modelID, markerID;
    NatNet_DecodeID( data.ID, &modelID, &markerID );

    geometry_msgs::PointStamped msgMarkerPose;
    msgMarkerPose.header.frame_id = std::to_string(modelID)+std::to_string(markerID);
    msgMarkerPose.header.stamp = ros::Time::now();

    msgMarkerPose.point.x = data.x;
    msgMarkerPose.point.y = data.y;
    msgMarkerPose.point.z = data.z;

    internal.RigidbodyMarkerPub[std::to_string(modelID)+std::to_string(markerID)].publish(msgMarkerPose);

}