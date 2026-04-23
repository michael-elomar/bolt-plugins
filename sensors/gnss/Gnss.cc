#include "Gnss.hh"

namespace gz {
namespace sensors {

class GnssSensorPrivate {
	transport::Node node;

	transport::Node::Publisher pub;

	bool loaded = false;

	double latitude;

	double longitude;

	double altitude = 0.0;

	double velocity_east;

	double velocity_north;

	double velocity_up;
};

GnssSensor::GnssSensor() : mDataPtr(std::make_unique<GnssSensorPrivate>()) {}

GnssSensor::~GnssSensor() = default;

bool GnssSensor::Init()
{
	return this->Sensor::Init();
}

bool GnssSensor::Load(sdf::ElementPtr sdf)
{
	sdf::Sensor sdfSensor;
	sdfSensor.Load(sdf);
	return this->Load(sdfSensor);
}

bool GnssSensor::Load(const sdf::Sensor &sdfSensor)
{
	std::string type = customType(sdfSensor);
	if (type != "gnss") {
		gzerr << "Trying to load [gnss] sensor, but got type [" << type
		      << "] instead.\n";

		return false;
	}

	if (!Sensor::Load(sdfSensor)) {
		gzerr << "Failed to load Gnss sensor in simulation\n";
		return false;
	}

	return true;
}

} // namespace sensors
} // namespace gz
