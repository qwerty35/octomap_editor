
#include <ros/ros.h>
#include <ros/package.h>

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

ros::ServiceClient delete_box_client;
ros::ServiceClient save_octomap_client;
ros::Publisher add_point_cloud_publisher;
octomap::point3d selection_point1;
octomap::point3d selection_point2;
std::string save_file_name;
double resolution;

// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
// %EndTag(vars)%


void publishPointCloud(const octomap::point3d& p1, const octomap::point3d& p2, const std::string& frame_id){
    pcl::PointCloud<pcl::PointXYZ> new_point_cloud;
    new_point_cloud.points.clear();

    geometry_msgs::Point min, max;
    min.x = std::min(p1.x(), p2.x());
    min.y = std::min(p1.y(), p2.y());
    min.z = std::min(p1.z(), p2.z());
    max.x = std::max(p1.x(), p2.x());
    max.y = std::max(p1.y(), p2.y());
    max.z = std::max(p1.z(), p2.z());

    float res = resolution;
    for(float x = min.x; x < max.x; x += res){
        for(float y = min.y; y < max.y; y += res){
            for(float z = min.z; z < max.z; z += res){
                pcl::PointXYZ point(x - p1.x(),
                                    y - p1.y(),
                                    z - p1.z());
                new_point_cloud.points.push_back(point);
            }
        }
    }

    sensor_msgs::PointCloud2 msg_point_cloud;
    pcl::toROSMsg(new_point_cloud, msg_point_cloud);
    msg_point_cloud.header.frame_id = frame_id;
    msg_point_cloud.header.stamp = ros::Time::now();
    ros::Rate rate(50);
    for(int i = 0; i < 20; i++){
        add_point_cloud_publisher.publish(msg_point_cloud);
        rate.sleep();
    }
}


void tfCallback(const ros::TimerEvent&)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(selection_point1.x(), selection_point1.y(), selection_point1.z()) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "selection_point1"));
}

// %Tag(Box)%
Marker makeBox( InteractiveMarker &msg, double a )
{
    Marker marker;

    marker.type = Marker::CUBE;
    marker.scale.x = msg.scale * 0.5;
    marker.scale.y = msg.scale * 0.5;
    marker.scale.z = msg.scale * 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = a;

    return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg, double a )
{
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( makeBox(msg, a) );
    msg.controls.push_back( control );

    return msg.controls.back();
}
// %EndTag(Box)%

// %Tag(processFeedback)%
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
//    std::ostringstream s;
//    s << "Feedback from marker '" << feedback->marker_name << "' "
//      << " / control '" << feedback->control_name << "'";

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
                publishPointCloud(selection_point1, selection_point2, "selection_point1");
            }
            else if(feedback->menu_entry_id == 3){
                ROS_INFO_STREAM("Save octomap");
                std::string command = "rosrun octomap_server octomap_saver " + save_file_name;
                system(command.c_str());
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

            {
                visualization_msgs::InteractiveMarker menu_marker;
                server->get("selected_region", menu_marker );
                octomap::point3d selection_box_pose = (selection_point1 + selection_point2) * 0.5;
                octomap::point3d selection_box_delta = selection_point1 - selection_point2;
                menu_marker.pose.position.x = selection_box_pose.x();
                menu_marker.pose.position.y = selection_box_pose.y();
                menu_marker.pose.position.z = selection_box_pose.z();
                menu_marker.controls[0].markers[0].scale.x = abs(selection_box_delta.x());
                menu_marker.controls[0].markers[0].scale.y = abs(selection_box_delta.y());
                menu_marker.controls[0].markers[0].scale.z = abs(selection_box_delta.z());
                server->insert(menu_marker);
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
    makeBoxControl(int_marker, 0);
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
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);


    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
    menu_handler.apply( *server, int_marker.name );
}
// %EndTag(6DOF)%

void makeMenuMarker(const tf::Vector3& position, const std::string& marker_name)
{
    unsigned int interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;

    InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 2.0;

    int_marker.name = marker_name;
    int_marker.description = marker_name;

    // insert a box
    makeBoxControl(int_marker, 0.3);
    int_marker.controls[0].interaction_mode = interaction_mode;

    InteractiveMarkerControl control;
    control.orientation_mode = InteractiveMarkerControl::FIXED;

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
    menu_handler.apply( *server, int_marker.name );
}

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
    control.markers.push_back( makeBox(int_marker, 0.3) );
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(Moving)%

// %Tag(main)%
int main(int argc, char** argv)
{
    ros::init(argc, argv, "basic_controls");
    ros::NodeHandle n("~");

    ros::Timer tf_broadcaster_timer = n.createTimer(ros::Duration(0.01), tfCallback);
    delete_box_client = n.serviceClient<octomap_msgs::BoundingBoxQuery>("/octomap_server/clear_bbx");
    add_point_cloud_publisher = n.advertise<sensor_msgs::PointCloud2>("/cloud_in", 1);

    std::string package_path = ros::package::getPath("octomap_editor");
    n.param<std::string>("save_file_name", save_file_name, "default.bt");
    n.param<double>("resolution", resolution, 0.1);
    save_file_name = package_path + "/world/" + save_file_name;

    server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );

    ros::Duration(0.1).sleep();

    menu_handler.insert( "Delete markers in selection box", &processFeedback );
    menu_handler.insert( "Add markers in selection box", &processFeedback );
    menu_handler.insert( "Save octomap", &processFeedback );

    tf::Vector3 position;
    position = tf::Vector3(0, 0, 0);
    selection_point1 = octomap::point3d(0, 0, 0);
    make6DofMarker(position, "selection_point1");
    position = tf::Vector3(1, 1, 1);
    selection_point2 = octomap::point3d(1, 1, 1);
    make6DofMarker(position, "selection_point2");
    position = tf::Vector3(0.5, 0.5, 0.5);
    makeMenuMarker(position, "selected_region");

    server->applyChanges();

    ros::spin();

    server.reset();
}
// %EndTag(main)%