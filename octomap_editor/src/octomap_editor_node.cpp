
#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/BoundingBoxQuery.h>

using namespace visualization_msgs;

ros::Publisher selection_box_publisher;
ros::ServiceClient delete_box_client;
ros::Publisher add_point_cloud_publisher;
octomap::point3d selection_point1;
octomap::point3d selection_point2;

// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
// %EndTag(vars)%


void selectionBoxCallback(const ros::TimerEvent&)
{

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(selection_point1.x(), selection_point1.y(), selection_point1.z()) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "selection_point1"));

    visualization_msgs::MarkerArray marker_array;
    octomap::point3d selection_box_pose = (selection_point1 + selection_point2) * 0.5;
    octomap::point3d selection_box_delta = selection_point1 - selection_point2;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.id = 0;
    marker.ns = "selection_box";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = selection_box_pose.x();
    marker.pose.position.y = selection_box_pose.y();
    marker.pose.position.z = selection_box_pose.z();
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = abs(selection_box_delta.x());
    marker.scale.y = abs(selection_box_delta.y());
    marker.scale.z = abs(selection_box_delta.z());
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.3;

    marker_array.markers.emplace_back(marker);

    selection_box_publisher.publish(marker_array);
}

// %Tag(Box)%
Marker makeBox( InteractiveMarker &msg )
{
    Marker marker;

    marker.type = Marker::CUBE;
    marker.scale.x = msg.scale * 0.45;
    marker.scale.y = msg.scale * 0.45;
    marker.scale.z = msg.scale * 0.45;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.0;

    return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( makeBox(msg) );
    msg.controls.push_back( control );

    return msg.controls.back();
}
// %EndTag(Box)%

// %Tag(processFeedback)%
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

    std::ostringstream mouse_point_ss;
    if( feedback->mouse_point_valid )
    {
        mouse_point_ss << " at " << feedback->mouse_point.x
                       << ", " << feedback->mouse_point.y
                       << ", " << feedback->mouse_point.z
                       << " in frame " << feedback->header.frame_id;
    }

    switch ( feedback->event_type )
    {
        case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
//            ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
            ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
            if(feedback->menu_entry_id == 1){
                ROS_INFO_STREAM("Delete markers in selection box");
                geometry_msgs::Point min, max;
                min.x = std::min(selection_point1.x(), selection_point2.x());
                min.y = std::min(selection_point1.y(), selection_point2.y());
                min.z = std::min(selection_point1.z(), selection_point2.z());
                max.x = std::max(selection_point1.x(), selection_point2.x());
                max.y = std::max(selection_point1.y(), selection_point2.y());
                max.z = std::max(selection_point1.z(), selection_point2.z());
                octomap_msgs::BoundingBoxQuery srv;
                srv.request.min = min;
                srv.request.max = max;
                if (not delete_box_client.call(srv)) {
                    ROS_ERROR("Failed to call service clear_bbx");
                }
            }
            else if(feedback->menu_entry_id == 2){
                ROS_INFO_STREAM("Add markers in selection box");
                pcl::PointCloud<pcl::PointXYZ> new_point_cloud;
                new_point_cloud.points.clear();

                geometry_msgs::Point min, max;
                min.x = std::min(selection_point1.x(), selection_point2.x());
                min.y = std::min(selection_point1.y(), selection_point2.y());
                min.z = std::min(selection_point1.z(), selection_point2.z());
                max.x = std::max(selection_point1.x(), selection_point2.x());
                max.y = std::max(selection_point1.y(), selection_point2.y());
                max.z = std::max(selection_point1.z(), selection_point2.z());

                float res = 0.1; //TODO: param

                for(float x = min.x; x < max.x; x += res){
                    for(float y = min.y; y < max.y; y += res){
                        for(float z = min.z; z < max.z; z += res){
                            pcl::PointXYZ point(x - selection_point1.x(),
                                                y - selection_point1.y(),
                                                z - selection_point1.z());
                            new_point_cloud.points.push_back(point);
                        }
                    }
                }

                sensor_msgs::PointCloud2 msg_point_cloud;
                pcl::toROSMsg(new_point_cloud, msg_point_cloud);
                msg_point_cloud.header.frame_id = "selection_point1";
                msg_point_cloud.header.stamp = ros::Time::now();
                ros::Rate rate(50);
                for(int i = 0; i < 100; i++){
                    add_point_cloud_publisher.publish(msg_point_cloud);
                    rate.sleep();
                }
            }
            break;

        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
            if(feedback->marker_name == "selection_point1"){
                selection_point1 = octomap::point3d(feedback->pose.position.x,
                                                    feedback->pose.position.y,
                                                    feedback->pose.position.z);
            }
            else if(feedback->marker_name == "selection_point2"){
                selection_point2 = octomap::point3d(feedback->pose.position.x,
                                                    feedback->pose.position.y,
                                                    feedback->pose.position.z);
            }
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
//            ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
//            ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
            break;
    }

    server->applyChanges();
}
// %EndTag(processFeedback)%

////////////////////////////////////////////////////////////////////////////////////

// %Tag(6DOF)%
void make6DofMarker(const tf::Vector3& position, const std::string& marker_name)
{
    unsigned int interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;

    InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 0.3;

    int_marker.name = marker_name;
    int_marker.description = marker_name;

    // insert a box
    makeBoxControl(int_marker);
    int_marker.controls[0].interaction_mode = interaction_mode;

    InteractiveMarkerControl control;
    control.orientation_mode = InteractiveMarkerControl::FIXED;

    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);


    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
    menu_handler.apply( *server, int_marker.name );
}
// %EndTag(6DOF)%

// %Tag(Moving)%
void makeMovingMarker( const tf::Vector3& position )
{
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "moving_frame";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "moving";
    int_marker.description = "Marker Attached to a\nMoving Frame";

    InteractiveMarkerControl control;

    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
    control.always_visible = true;
    control.markers.push_back( makeBox(int_marker) );
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(Moving)%

// %Tag(main)%
int main(int argc, char** argv)
{
    ros::init(argc, argv, "basic_controls");
    ros::NodeHandle n;

    selection_box_publisher = n.advertise<visualization_msgs::MarkerArray>("/selection_box", 1);
    ros::Timer selection_box_timer = n.createTimer(ros::Duration(0.01), selectionBoxCallback);

    delete_box_client = n.serviceClient<octomap_msgs::BoundingBoxQuery>("/octomap_server/clear_bbx");
    add_point_cloud_publisher = n.advertise<sensor_msgs::PointCloud2>("/cloud_in", 1);

    server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );

    ros::Duration(0.1).sleep();

    menu_handler.insert( "Delete markers in selection box", &processFeedback );
    menu_handler.insert( "Add markers in selection box", &processFeedback );

    tf::Vector3 position;
    position = tf::Vector3(0, 0, 0);
    selection_point1 = octomap::point3d(0, 0, 0);
    make6DofMarker(position, "selection_point1");
    position = tf::Vector3(1, 1, 1);
    selection_point2 = octomap::point3d(1, 1, 1);
    make6DofMarker(position, "selection_point2");

    server->applyChanges();

    ros::spin();

    server.reset();
}
// %EndTag(main)%