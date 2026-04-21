#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/transport.hh>

#include <gz/msgs.hh>

namespace gz {
namespace sim {
namespace systems {

class Example : public System,
		public ISystemConfigure,
		public ISystemPreUpdate {

public:
	Example();

	~Example() override;

	void Configure(const Entity &_entity,
		       const std::shared_ptr<const sdf::Element> &_sdf,
		       EntityComponentManager &_ecm,
		       EventManager &_eventMgr) override;

	void PreUpdate(const UpdateInfo &_info,
		       EntityComponentManager &_ecm) override;

private:
	void OnCmdSpeed(const msgs::Double &_msg);

private:
	Model mModel;
	double mRotorSpeed;
	transport::Node mNode;
};

} // namespace systems
} // namespace sim
} // namespace gz
