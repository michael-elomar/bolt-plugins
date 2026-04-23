#pragma once

#include <gz/sensors.hh>
#include <gz/transport.hh>
#include <gz/msgs.hh>
#include <sdf/sdf.hh>

namespace gz {
namespace sensors {

class GnssSensorPrivate;

class GnssSensor : public Sensor {
public:
	GnssSensor();

	~GnssSensor();

	bool Load(const sdf::Sensor &sdfSensor) override;

	bool Load(sdf::ElementPtr sdf) override;

	bool Init() override;

	bool Update(const std::chrono::steady_clock::duration &now) override;

	void SetPosition(const double latitude,
			 const double longitude,
			 const double altitude);

	void SetVelocity(const double vel_east,
			 const double vel_north,
			 const double vel_up);

private:
	std::unique_ptr<GnssSensorPrivate> mDataPtr;
};

} // namespace sensors
} // namespace gz
