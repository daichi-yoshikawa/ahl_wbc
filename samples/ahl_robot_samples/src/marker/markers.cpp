#include "ahl_robot_samples/marker/markers.hpp"

using namespace ahl_sample;

void Markers::add(const MarkerPtr& marker)
{
  marker_[marker->getName()] = marker;
}

void Markers::setColor(const std::string& name, int8_t r, int8_t g, int8_t b, double a)
{
  marker_[name]->setColor(r, g, b, a);
}

void Markers::setColor(int8_t r, int8_t g, int8_t b, double a)
{
  for(auto it = std::begin(marker_); it != std::begin(marker_); ++it)
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
  for(auto it = std::begin(marker_); it != std::end(marker_); ++it)
  {
    it->second->setScale(scale);
  }
}

void Markers::publish()
{
  for(auto it = std::begin(marker_); it != std::end(marker_); ++it)
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
  for(auto it = std::begin(marker_); it != std::end(marker_); ++it)
  {
    it->second->remove();
  }
}
