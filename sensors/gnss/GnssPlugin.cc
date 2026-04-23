#include "Gnss.hh"
#include "GnssPlugin.hh"
#include <gz/plugin/Register.hh>

namespace gz {
namespace sim {
namespace systems {

#define LAT 35.6
#define LON 33.1
#define ALT 630.7

class GnssSystemPrivate {
public:
	Entity entity;
	components::WorldLinearVelocity *vel;
	std::unique_ptr<sensors::GnssSensor> sensor;
	sensors::SensorFactory sensorFactory;
};

GnssSystem::GnssSystem()
	: System(), mDataPtr(std::make_unique<GnssSystemPrivate>())
{
}

GnssSystem::~GnssSystem() {}

void GnssSystem::PreUpdate(const UpdateInfo &_info,
			   EntityComponentManager &_ecm)
{

	gzmsg << "PREUPDATE\n";
	_ecm.EachNew<components::CustomSensor,
		     components::ParentEntity,
		     components::WorldLinearVelocity>(
		[&](const gz::sim::Entity &_entity,
		    const gz::sim::components::CustomSensor *_custom,
		    const gz::sim::components::ParentEntity *_parent,
		    components::WorldLinearVelocity *_vel) -> bool {
			// Get sensor's scoped name without the world
			std::string sensorScopedName =
				gz::sim::removeParentScope(
					gz::sim::scopedName(
						_entity, _ecm, "::", false),
					"::");
			sdf::Sensor data = _custom->Data();
			data.SetName(sensorScopedName);

			// Default to scoped name as topic
			if (data.Topic().empty()) {
				std::string topic =
					scopedName(_entity, _ecm) + "/gnss";
				data.SetTopic(topic);
			}

			mDataPtr->vel = _vel;
			mDataPtr->entity = _entity;

			mDataPtr->sensor =
				mDataPtr->sensorFactory
					.CreateSensor<sensors::GnssSensor>(
						data);
			if (mDataPtr->sensor == nullptr) {
				gzerr << "Failed to create odometer ["
				      << sensorScopedName << "]" << std::endl;
				return false;
			}

			// Set sensor parent
			auto parentName =
				_ecm.Component<gz::sim::components::Name>(
					    _parent->Data())
					->Data();
			mDataPtr->sensor->SetParent(parentName);

			// Set topic on Gazebo
			_ecm.CreateComponent(
				_entity,
				gz::sim::components::SensorTopic(
					mDataPtr->sensor->Topic()));

			return true;
		});
}

void GnssSystem::PostUpdate(const UpdateInfo &_info,
			    const EntityComponentManager &_ecm)
{
	// Only update and publish if not paused.
	if (!_info.paused) {
		mDataPtr->sensor->SetPosition(LAT, LON, ALT);
		mDataPtr->sensor->SetVelocity(mDataPtr->vel->Data().X(),
					      mDataPtr->vel->Data().Y(),
					      mDataPtr->vel->Data().Z());

		mDataPtr->sensor->Update(_info.simTime);
	}
}

GZ_ADD_PLUGIN(GnssSystem,
	      System,
	      GnssSystem::ISystemPostUpdate,
	      GnssSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(GnssSystem, "gz::sim::systems::GnssSystem")

} // namespace systems
} // namespace sim
} // namespace gz
