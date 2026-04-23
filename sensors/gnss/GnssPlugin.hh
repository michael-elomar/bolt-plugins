#pragma once

#include <gz/sim.hh>
#include <gz/transport.hh>
#include <gz/msgs.hh>

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
