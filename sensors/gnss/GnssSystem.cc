#include "Gnss.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Pose.hh"
#include "GnssPlugin.hh"

namespace gz {
namespace sim {
namespace systems {

#define LAT 35.6
#define LON 33.1
#define ALT 630.7

class GnssSystemPrivate {
public:
	std::unordered_map<Entity, std::shared_ptr<sensors::GnssSensor>>
		entitySensorMap;

	std::unordered_map<Entity, Entity> sensorParentMap;

	/// \brief gz-sensors sensor factory for creating sensors
	sensors::SensorFactory sensorFactory;

	/// \brief Keep list of sensors that were created during the previous
	/// `PostUpdate`, so that components can be created during the next
	/// `PreUpdate`.
	std::unordered_set<Entity> newSensors;

	Entity worldEntity = kNullEntity;

	Entity parentEntity = kNullEntity;

	/// When the system is just loaded, we loop over all entities to create
	/// sensors. After this initialization, we only check inserted entities.
	bool initialized = false;

	/// \brief Create sensors in gz-sensors
	/// \param[in] _ecm Immutable reference to ECM.
	void CreateSensors(const EntityComponentManager &_ecm);

	/// \brief Update sensor data based on physics data
	/// \param[in] _ecm Immutable reference to ECM.
	void Update(const EntityComponentManager &_ecm);

	void AddSensor(const EntityComponentManager &_ecm,
		       const Entity _entity,
		       const components::CustomSensor *_gnss,
		       const components::ParentEntity *_parent);

	/// \brief Remove sensors if their entities have been removed from
	/// simulation.
	/// \param[in] _ecm Immutable reference to ECM.
	void RemoveSensors(const EntityComponentManager &_ecm);
};

GnssSystem::GnssSystem()
	: System(), mDataPtr(std::make_unique<GnssSystemPrivate>())
{
}

GnssSystem::~GnssSystem() {}

void GnssSystem::PreUpdate(const UpdateInfo &_info,
			   EntityComponentManager &_ecm)
{

	// Create components
	for (auto entity : mDataPtr->newSensors) {
		auto it = mDataPtr->entitySensorMap.find(entity);
		if (it == mDataPtr->entitySensorMap.end()) {
			gzerr << "Entity [" << entity
			      << "] isn't in sensor map, this shouldn't happen."
			      << std::endl;
			continue;
		}
		// Set topic
		_ecm.CreateComponent(
			entity, components::SensorTopic(it->second->Topic()));

		// enable velocity checks on parent links
		for (const auto &[sensorEntity, parentEntity] :
		     mDataPtr->sensorParentMap) {
			Link link(parentEntity);
			link.EnableVelocityChecks(_ecm, true);
		}
	}
	mDataPtr->newSensors.clear();
}

void GnssSystem::PostUpdate(const UpdateInfo &_info,
			    const EntityComponentManager &_ecm)
{
	if (_info.dt < std::chrono::steady_clock::duration::zero()) {
		gzwarn << "Time jumpback detected";
	}

	mDataPtr->CreateSensors(_ecm);

	// Only update and publish if not paused.
	if (!_info.paused) {
		// bool needsUpdate = false;
		// for (auto &[entity, sensor] : mDataPtr->entitySensorMap) {
		// 	if (sensor->NextDataUpdateTime() <= _info.simTime
		// 	    && sensor->HasConnections())
		// 		needsUpdate = true;
		// 	break;
		// }
		//
		// if (!needsUpdate)
		// 	return;

		mDataPtr->Update(_ecm);

		for (auto &[entity, sensor] : mDataPtr->entitySensorMap) {
			sensor->Update(_info.simTime);
		}
	}

	mDataPtr->RemoveSensors(_ecm);
}

/* Gnss Private Plugin implementation */
void GnssSystemPrivate::AddSensor(const EntityComponentManager &_ecm,
				  const Entity _entity,
				  const components::CustomSensor *_gnss,
				  const components::ParentEntity *_parent)

{
	std::string sensorScopedName =
		removeParentScope(scopedName(_entity, _ecm, "::", false), "::");
	sdf::Sensor data = _gnss->Data();
	data.SetName(sensorScopedName);

	if (data.Topic().empty()) {
		std::string topic = scopedName(_entity, _ecm);
		data.SetTopic(topic);
	}

	std::unique_ptr<sensors::GnssSensor> sensor =
		sensorFactory.CreateSensor<sensors::GnssSensor>(data);

	if (sensor == nullptr) {
		gzerr << "Failed to create sensor [" << sensorScopedName
		      << "]\n";
		return;
	}

	std::string parentName =
		_ecm.Component<components::Name>(_parent->Data())->Data();
	sensor->SetParent(parentName);

	entitySensorMap.try_emplace(_entity, std::move(sensor));
	sensorParentMap.try_emplace(_entity, _parent->Data());
	newSensors.insert(_entity);
}

void GnssSystemPrivate::CreateSensors(const EntityComponentManager &_ecm)
{
	if (!this->initialized) {
		_ecm.Each<components::CustomSensor, components::ParentEntity>(
			[&](const Entity &_entity,
			    const components::CustomSensor *_gnss,
			    const components::ParentEntity *_parent) -> bool {
				AddSensor(_ecm, _entity, _gnss, _parent);
				return true;
			});
		this->initialized = true;
	} else {
		_ecm.EachNew<components::CustomSensor,
			     components::ParentEntity>(
			[&](const Entity &_entity,
			    const components::CustomSensor *_gnss,
			    const components::ParentEntity *_parent) -> bool {
				AddSensor(_ecm, _entity, _gnss, _parent);
				return true;
			});
	}
}

void GnssSystemPrivate::Update(const EntityComponentManager &_ecm)
{
	for (const auto &[sensorEntity, parentEntity] : sensorParentMap) {
		if (parentEntity == kNullEntity) {
			gzerr << "Sensor has no parent. This should not happen\n";
			return;
		}

		auto worldPose =
			_ecm.Component<components::WorldPose>(parentEntity);
		if (!worldPose) {
			gzerr << "WorldPose component not found on parent link\n";
			return;
		}
		auto worldVel = _ecm.Component<components::WorldLinearVelocity>(
			parentEntity);
		if (!worldVel) {
			gzerr << "LinearVelocity component not found on parent link\n";
			return;
		}

		sensors::GnssSensorPtr gnss = entitySensorMap.at(sensorEntity);
		gnss->SetPosition(worldPose->Data().Pos());
		gnss->SetVelocity(worldVel->Data());
	}
}

void GnssSystemPrivate::RemoveSensors(const EntityComponentManager &_ecm)
{
	_ecm.EachRemoved<
		components::CustomSensor>([&](const Entity &_entity,
					      const components::CustomSensor *)
						  -> bool {
		auto sensorId = this->entitySensorMap.find(_entity);
		if (sensorId == this->entitySensorMap.end()) {
			gzerr << "Internal error, missing Gnss sensor for entity ["
			      << _entity << "]" << std::endl;
			return true;
		}

		this->entitySensorMap.erase(sensorId);

		return true;
	});
}

GZ_ADD_PLUGIN(GnssSystem,
	      System,
	      GnssSystem::ISystemPostUpdate,
	      GnssSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(GnssSystem, "gz::sim::systems::GnssSystem")

} // namespace systems
} // namespace sim
} // namespace gz
