#include "KER_constraint_fixed_position.hh"
#include "KER_cylinder_bank.hh"

CylinderBank::CylinderBank(PhysicsWorld *world, const CylinderBankDescr &descr) : RigidBody(descr.position, descr.angle, 1.0f, 1.0f) {
	if (world) {
		world->add_rigid_body(this, ePhysicsWorld_Flag::Owned);
		/**
		 * The center of the cylinder bank (0, 0) is considered to be held stable in its starting position (this->position) using a force
		 * of 5k neuton (equal to ~510kg) and a damping factor of 200 neuton as friction force.
		 */
		world->add_constraint(new FixedPositionConstraint(this, float2(0.0f, 0.0f), this->position, 5e3f, 2e2f), ePhysicsWorld_Flag::Owned);
	}
}

CylinderBank::~CylinderBank() {
}

/* -------------------------------------------------------------------- */
/** \name Cylinder Bank Utilities
 * \{ */

CylinderBank *KER_cylinder_bank_new(PhysicsWorld *world, float2 position, float angle, float bore, float deck_height) {
	CylinderBankDescr descr;

	descr.position = position;
	descr.angle = angle;
	descr.bore = bore;
	descr.deck_height = deck_height;

	return new CylinderBank(world, descr);
}

/** \} */
