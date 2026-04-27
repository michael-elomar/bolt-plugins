#include "Gnss.hh"

namespace gz {
namespace sensors {

class GnssSensorPrivate {
public:
	transport::Node node;

	transport::Node::Publisher pub;

	NoisePtr horizontalNoise, verticalNoise;

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

	if (this->Topic().empty())
		this->SetTopic("/gnss");

	mDataPtr->pub = mDataPtr->node.Advertise<msgs::NavSat>(this->Topic());

	if (!mDataPtr->pub) {
		gzerr << "Unable to publish topic " << this->Topic()
		      << std::endl;
		return false;
	}

	/* read custom element */
	sdfSensor.Element()->FindElement("gz:gnss");
	if (!sdfSensor.Element()->HasElement("gz:gnss")) {
		gzwarn << "No custom configuration for gnss sensor"
		       << std::endl;
		return true;
	}

	sdf::ElementPtr customElem =
		sdfSensor.Element()->FindElement("gz:gnss");

	/* read and store noise element */
	if (!customElem->HasElement("noise")) {
		gzwarn << "No noise defined" << std::endl;
		return true;
	}
	sdf::Noise noiseSdf;
	noiseSdf.Load(customElem->FindElement("noise"));
	mDataPtr->horizontalNoise = NoiseFactory::NewNoiseModel(noiseSdf);

	if (mDataPtr->horizontalNoise == nullptr) {
		gzerr << "Failed to load horizontal noise from sdf"
		      << std::endl;
		return false;
	}

	mDataPtr->verticalNoise = NoiseFactory::NewNoiseModel(noiseSdf);

	if (mDataPtr->verticalNoise == nullptr) {
		gzerr << "Failed to load vertical noise from sdf" << std::endl;
		return false;
	}

	gzmsg << "GnssSensor successfully loaded\n";
	mDataPtr->loaded = true;
	return true;
}

bool GnssSensor::Update(const std::chrono::steady_clock::duration &now)
{
	msgs::NavSat msg;
	*msg.mutable_header()->mutable_stamp() = msgs::Convert(now);
	msg.set_frame_id(this->FrameId());

	mDataPtr->latitude =
		mDataPtr->horizontalNoise->Apply(mDataPtr->latitude);

	mDataPtr->longitude =
		mDataPtr->horizontalNoise->Apply(mDataPtr->longitude);

	mDataPtr->altitude = mDataPtr->verticalNoise->Apply(mDataPtr->altitude);

	mDataPtr->velocity_east =
		mDataPtr->horizontalNoise->Apply(mDataPtr->velocity_east);

	mDataPtr->velocity_north =
		mDataPtr->horizontalNoise->Apply(mDataPtr->velocity_north);

	mDataPtr->velocity_up =
		mDataPtr->verticalNoise->Apply(mDataPtr->velocity_up);

	msg.set_velocity_north(mDataPtr->velocity_north);
	msg.set_velocity_east(mDataPtr->velocity_east);
	msg.set_velocity_up(mDataPtr->velocity_up);
	msg.set_altitude(mDataPtr->altitude);
	msg.set_latitude_deg(mDataPtr->latitude);
	msg.set_longitude_deg(mDataPtr->longitude);

	AddSequence(msg.mutable_header());
	mDataPtr->pub.Publish(msg);

	return true;
}

bool GnssSensor::HasConnections() const
{
	return mDataPtr->pub && mDataPtr->pub.HasConnections();
}

void GnssSensor::SetPosition(const math::Vector3d position)
{
	mDataPtr->latitude = position.X();
	mDataPtr->longitude = position.Y();
	mDataPtr->altitude = position.Z();
}

void GnssSensor::SetVelocity(const math::Vector3d velocity)
{
	mDataPtr->velocity_east = velocity.X();
	mDataPtr->velocity_north = velocity.Y();
	mDataPtr->velocity_up = velocity.Z();
}

} // namespace sensors
} // namespace gz
