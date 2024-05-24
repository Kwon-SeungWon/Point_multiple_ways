#include "selection_saver/selection_saver.h"

SelectionSaver::SelectionSaver() : nh("~"), server("polygon_marker_server") {
    polygon_pub = nh.advertise<geometry_msgs::PolygonStamped>("selected_polygon", 1);

    // Initialize polygon message
    polygon.header.frame_id = "map"; // Change this frame id according to your setup

    // Create an interactive marker for polygon interaction
    visualization_msgs::InteractiveMarker polygon_marker;
    polygon_marker.header.frame_id = "map"; // Change this frame id according to your setup
    polygon_marker.name = "polygon_marker";
    polygon_marker.description = "Select Polygon";

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(marker);

    polygon_marker.controls.push_back(control);

    server.insert(polygon_marker, boost::bind(&SelectionSaver::processFeedback, this, _1));
    server.applyChanges();
}

void SelectionSaver::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP) {
        geometry_msgs::Point point;
        point.x = feedback->pose.position.x;
        point.y = feedback->pose.position.y;
        point.z = feedback->pose.position.z;
        polygon_points.push_back(point);

        visualization_msgs::Marker marker;
        // Clear existing points from the marker
        marker.points.clear();

        // Add points to the marker for the polygon
        for (const auto& pt : polygon_points) {
            marker.points.push_back(pt);
        }

        // Update the control with the modified marker
        polygon_marker.controls[0].markers.clear();
        polygon_marker.controls[0].markers.push_back(marker);

        // Update the marker in the server
        server.insert(polygon_marker, boost::bind(&SelectionSaver::processFeedback, this, _1));
        server.applyChanges();
    }
}

void SelectionSaver::publishPolygon() const {
    if (polygon_points.size() > 2) { // At least 3 points are required to form a polygon
        polygon.polygon.points = polygon_points;
        polygon_pub.publish(polygon);
    }
}
