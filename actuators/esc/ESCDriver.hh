#pragma once

#include <Eigen/Dense>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components.hh>
#include <gz/transport.hh>
#include <gz/msgs.hh>
#include <gz/plugin/Register.hh>

namespace gz {
namespace sim {
namespace systems {

class ESCDriverPrivate;

class ESCDriver : public System,
		  public ISystemConfigure,
		  public ISystemPreUpdate {

public:
	ESCDriver();

	~ESCDriver() override;

	void Configure(const Entity &_entity,
		       const std::shared_ptr<const sdf::Element> &_sdf,
		       EntityComponentManager &_ecm,
		       EventManager &_eventMgr) override;

	void PreUpdate(const UpdateInfo &_info,
		       EntityComponentManager &_ecm) override;

private:
	double GetPropellerInertia(const Entity &link_entity,
				   EntityComponentManager &_ecm);

	void OnInputVoltage(const msgs::Double &_msg);

private:
	std::unique_ptr<ESCDriverPrivate> mDataPtr;
};

} // namespace systems
} // namespace sim
} // namespace gz
