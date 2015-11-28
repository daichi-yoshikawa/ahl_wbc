#include "ahl_robot_samples/marker/marker.hpp"

using namespace ahl_sample;

Marker::Marker(const std::string& name, const std::string& frame, uint32_t shape)
{
  ros::NodeHandle nh;
  pub_ = nh.advertise<visualization_msgs::Marker>("marker/" + name, 1);

  marker_.ns = name;
  marker_.id = 0;
  marker_.type = shape;
  marker_.header.frame_id = frame;
  marker_.header.stamp = ros::Time::now();
  marker_.action = visualization_msgs::Marker::DELETE;

  marker_.pose.position.x = 0.0;
  marker_.pose.position.y = 0.0;
  marker_.pose.position.z = 0.0;
  marker_.pose.orientation.x = 0.0;
  marker_.pose.orientation.y = 0.0;
  marker_.pose.orientation.z = 0.0;
  marker_.pose.orientation.w = 1.0;

  this->setColor(0, 0, 0, 0.3);
  this->setScale(0.2);

  marker_.lifetime = ros::Duration();

  pub_.publish(marker_);
  marker_.action = visualization_msgs::Marker::ADD;
}

void Marker::setColor(int r, int g, int b, double a)
{
  marker_.color.r = r;
  marker_.color.g = g;
  marker_.color.b = b;
  marker_.color.a = a;
}

void Marker::setPosition(double x, double y, double z)
{
  marker_.pose.position.x = x;
  marker_.pose.position.y = y;
  marker_.pose.position.z = z;
}

void Marker::setScale(double scale)
{
  marker_.scale.x = scale;
  marker_.scale.y = scale;
  marker_.scale.z = scale;
}

void Marker::publish()
{
  marker_.action = visualization_msgs::Marker::ADD;
  pub_.publish(marker_);
}

void Marker::remove()
{
  marker_.action = visualization_msgs::Marker::DELETE;
  pub_.publish(marker_);
}
