#include "Example.hh"
#include "gz/sim/components/JointVelocityCmd.hh"

#include <gz/common.hh>
#include <gz/sim/Joint.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components.hh>

namespace gz {
namespace sim {
namespace systems {

Example::Example()
{
	std::cout << "Example Constructor" << std::endl;
}

Example::~Example()
{
	std::cout << "Example Destructor" << std::endl;
}

void Example::Configure(const Entity &_entity,
			const std::shared_ptr<const sdf::Element> &_sdf,
			EntityComponentManager &_ecm,
			EventManager &_eventMgr)
{
	std::cout << "Configure" << std::endl;

	mModel = Model(_entity);

	if (!mModel.Valid(_ecm)) {
		gzerr << "Failed to attach plugin to model" << std::endl;
		return;
	}

	auto sdfClone = _sdf->Clone();

	auto sdfElement = sdfClone->GetElement("rotor_speed");
	if (sdfElement)
		mRotorSpeed = sdfElement->Get<double>();

	auto topic = transport::TopicUtils::AsValidTopic(
		"/model/" + mModel.Name(_ecm) + "/speed");

	if (topic.empty()) {
		gzerr << "Failed to create topic" << std::endl;
		return;
	}

	mNode.Subscribe(topic, &Example::OnCmdSpeed, this);

	std::cout << "Example plugin subscribed to a topic" << std::endl;
}

void Example::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
	if (_info.paused)
		return;

	for (const auto &joint_entity : mModel.Joints(_ecm)) {
		if (joint_entity == kNullEntity)
			continue;

		auto velocity =
			_ecm.ComponentDefault<components::JointVelocityCmd>(
				joint_entity, {0.0});

		velocity->Data()[0] += mRotorSpeed;
	}
}

void Example::OnCmdSpeed(const msgs::Double &_msg)
{
	std::cout << "Received: " << _msg.data() << std::endl;
	mRotorSpeed = _msg.data();
}

GZ_ADD_PLUGIN(Example,
	      System,
	      Example::ISystemConfigure,
	      Example::ISystemPreUpdate);

GZ_ADD_PLUGIN_ALIAS(Example, "gz::sim::systems::Example")

} // namespace systems
} // namespace sim
} // namespace gz
