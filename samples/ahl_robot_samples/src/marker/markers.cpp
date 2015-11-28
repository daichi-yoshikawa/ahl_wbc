#include "ahl_robot_samples/marker/markers.hpp"

using namespace ahl_sample;

void Markers::add(const MarkerPtr& marker)
{
  marker_[marker->getName()] = marker;
}

void Markers::setColor(const std::string& name, int r, int g, int b, double a)
{
  marker_[name]->setColor(r, g, b, a);
}

void Markers::setColor(int r, int g, int b, double a)
{
  std::map<std::string, MarkerPtr>::iterator it;
  for(it = marker_.begin(); it != marker_.end(); ++it)
  {
    it->second->setColor(r, g, b, a);
  }
}

void Markers::setPosition(const std::string& name, double x, double y, double z)
{
  marker_[name]->setPosition(x, y, z);
}

void Markers::setScale(const std::string& name, double scale)
{
  marker_[name]->setScale(scale);
}

void Markers::setScale(double scale)
{
  std::map<std::string, MarkerPtr>::iterator it;
  for(it = marker_.begin(); it != marker_.end(); ++it)
  {
    it->second->setScale(scale);
  }
}

void Markers::publish()
{
  std::map<std::string, MarkerPtr>::iterator it;
  for(it = marker_.begin(); it != marker_.end(); ++it)
  {
    it->second->publish();
  }
}

void Markers::remove(const std::string& name)
{
  marker_[name]->remove();
}

void Markers::remove()
{
  std::map<std::string, MarkerPtr>::iterator it;
  for(it = marker_.begin(); it != marker_.end(); ++it)
  {
    it->second->remove();
  }
}
