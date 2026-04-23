#pragma once

#include <gz/sensors.hh>
#include <gz/transport.hh>
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

	bool HasConnections() const override;

private:
	std::unique_ptr<GnssSensorPrivate> mDataPtr;
};

} // namespace sensors
} // namespace gz
