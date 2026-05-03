#include "ESCDriver.hh"

namespace gz {
namespace sim {
namespace systems {

class ESCDriverPrivate {
public:
	Entity mParentEntity = kNullEntity;

	Entity linkEntity = kNullEntity;

	Entity jointEntity = kNullEntity;

	std::string modelName;

	std::string jointName;

	std::string linkName;

	double vBus;

	double linkInertia;

	double thrustCoeff;

	double dampingCoeff;

	double torqueCoeff;

	double backEMFCoeff;

	double resistance;

	double inductance;

	transport::Node node;

	/* state space model variables */

	double inputVoltage;

	Eigen::Matrix2d A;
	Eigen::Vector2d X, u;
};

ESCDriver::ESCDriver() : mDataPtr(std::make_unique<ESCDriverPrivate>()) {}

ESCDriver::~ESCDriver() {}

void ESCDriver::Configure(const Entity &_entity,
			  const std::shared_ptr<const sdf::Element> &_sdf,
			  EntityComponentManager &_ecm,
			  EventManager &_eventMgr)
{
	mDataPtr->mParentEntity = _entity;

	if (mDataPtr->mParentEntity == kNullEntity) {
		gzerr << "Failed to attach plugin to model" << std::endl;
		return;
	}

	mDataPtr->modelName =
		_ecm.Component<components::Name>(mDataPtr->mParentEntity)
			->Data();

	/* load sdf parameters */
	sdf::ElementPtr sdfClone = _sdf->Clone();

	if (sdfClone->HasElement("vBus"))
		mDataPtr->vBus = sdfClone->GetElement("vBus")->Get<double>();

	if (sdfClone->HasElement("jointName"))
		mDataPtr->jointName =
			sdfClone->GetElement("jointName")->Get<std::string>();

	if (sdfClone->HasElement("linkName"))
		mDataPtr->linkName =
			sdfClone->GetElement("linkName")->Get<std::string>();

	if (sdfClone->HasElement("torqueCoeff"))
		mDataPtr->torqueCoeff =
			sdfClone->GetElement("torqueCoeff")->Get<double>();

	if (sdfClone->HasElement("backEMFCoeff"))
		mDataPtr->backEMFCoeff =
			sdfClone->GetElement("backEMFCoeff")->Get<double>();

	if (sdfClone->HasElement("dampingCoeff"))
		mDataPtr->dampingCoeff =
			sdfClone->GetElement("dampingCoeff")->Get<double>();

	if (sdfClone->HasElement("thrustCoeff"))
		mDataPtr->thrustCoeff =
			sdfClone->GetElement("thrustCoeff")->Get<double>();

	if (sdfClone->HasElement("resistance"))
		mDataPtr->resistance =
			sdfClone->GetElement("resistance")->Get<double>();

	if (sdfClone->HasElement("inductance"))
		mDataPtr->inductance =
			sdfClone->GetElement("inductance")->Get<double>();

	auto linkEntitySet = entitiesFromScopedName(mDataPtr->linkName, _ecm);
	if (linkEntitySet.empty()) {
		gzerr << "No link by name " << mDataPtr->linkName
		      << "exists in model " << mDataPtr->modelName;
		return;
	}
	if (linkEntitySet.size() > 1) {
		gzerr << "Can't have more than one link by name "
		      << mDataPtr->linkName << " in model "
		      << mDataPtr->modelName;
		return;
	}

	mDataPtr->linkEntity = *linkEntitySet.begin();

	auto jointEntitySet = entitiesFromScopedName(mDataPtr->jointName, _ecm);
	if (jointEntitySet.empty()) {
		gzerr << "No joint by name " << mDataPtr->jointName
		      << "exists in model " << mDataPtr->modelName << std::endl;
		return;
	}
	if (jointEntitySet.size() > 1) {
		gzerr << "Can't have more than one joint by name "
		      << mDataPtr->jointName << " in model "
		      << mDataPtr->modelName << std::endl;
		return;
	}
	mDataPtr->jointEntity = *jointEntitySet.begin();

	double J = GetPropellerInertia(mDataPtr->linkEntity, _ecm);
	mDataPtr->A = Eigen::Matrix2d({
		{-mDataPtr->resistance / mDataPtr->inductance,
		 -mDataPtr->backEMFCoeff / mDataPtr->inductance},
		{-mDataPtr->torqueCoeff / J, -mDataPtr->dampingCoeff / J},
	});

	mDataPtr->u = Eigen::Vector2d{mDataPtr->inputVoltage, 0};

	mDataPtr->X = Eigen::Vector2d{0, 0};

	std::string topic = transport::TopicUtils::AsValidTopic(
		"/" + mDataPtr->modelName + "/motors/" + mDataPtr->jointName
		+ "/input_voltage");

	if (topic.empty()) {
		gzerr << "Failed to create topic" << std::endl;
		return;
	}

	mDataPtr->node.Subscribe(topic, &ESCDriver::OnInputVoltage, this);
}

void ESCDriver::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
	if (_info.paused)
		return;
	//
	// auto K1 = (mDataPtr->A * mDataPtr->X + mDataPtr->u) *
	// _info.dt.count(); auto K2 = (mDataPtr->A * (mDataPtr->X + 0.5 * K1) +
	// mDataPtr->u)
	// 	  * _info.dt.count();
	// auto K3 = (mDataPtr->A * (mDataPtr->X + 0.5 * K2) + mDataPtr->u)
	// 	  * _info.dt.count();
	// auto K4 = (mDataPtr->A * (mDataPtr->X + K3) + mDataPtr->u)
	// 	  * _info.dt.count();
	//
	// Eigen::Vector2d X_dot = (K1 + 2 * K2 + 2 * K3 + K4) / 6;
	//
	// mDataPtr->X = mDataPtr->X + X_dot * _info.dt.count();
}

double ESCDriver::GetPropellerInertia(const Entity &link_entity,
				      EntityComponentManager &_ecm)
{
	math::Inertiald inertial =
		_ecm.Component<components::Inertial>(link_entity)->Data();

	return inertial.MassMatrix().Izz();
}

void ESCDriver::OnInputVoltage(const msgs::Double &_msg)
{
	mDataPtr->inputVoltage = _msg.data();

	if (mDataPtr->inputVoltage >= mDataPtr->vBus)
		mDataPtr->inputVoltage = mDataPtr->vBus;

	else if (mDataPtr->inputVoltage <= 0.0)
		mDataPtr->inputVoltage = 0.0;

	mDataPtr->u[0] = mDataPtr->inputVoltage;
}

GZ_ADD_PLUGIN(ESCDriver,
	      System,
	      ESCDriver::ISystemConfigure,
	      ESCDriver::ISystemPreUpdate);

GZ_ADD_PLUGIN_ALIAS(ESCDriver, "gz::sim::systems::ESCDriver")

} // namespace systems
} // namespace sim
} // namespace gz
