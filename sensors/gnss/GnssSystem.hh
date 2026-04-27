#pragma once

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

class GnssSystemPrivate;

class GnssSystem : public System,
		   public ISystemPreUpdate,
		   public ISystemPostUpdate {

public:
	GnssSystem();

	~GnssSystem() override;

	void PreUpdate(const UpdateInfo &_info,
		       EntityComponentManager &_ecm) override;

	void PostUpdate(const UpdateInfo &_info,
			const EntityComponentManager &_ecm) override;

private:
	std::unique_ptr<GnssSystemPrivate> mDataPtr;
};

} // namespace systems
} // namespace sim
} // namespace gz
