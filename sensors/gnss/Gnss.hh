#pragma once

#include <gz/sensors.hh>
#include <gz/transport.hh>
#include <gz/msgs.hh>
#include <sdf/sdf.hh>
#include <gz/common.hh>

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

	void SetPosition(const math::Vector3d position);

	void SetVelocity(const math::Vector3d velocity);

private:
	std::unique_ptr<GnssSensorPrivate> mDataPtr;
};

using GnssSensorPtr = std::shared_ptr<GnssSensor>;

} // namespace sensors
} // namespace gz
