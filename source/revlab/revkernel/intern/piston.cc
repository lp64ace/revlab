#include "KER_constraint_line.hh"
#include "KER_piston.hh"

Piston::Piston(PhysicsWorld *world, const PistonDescr &descr) : RigidBody(descr.position, 0.0f, descr.mass, 1.0f), bank(descr.bank), pin(descr.pin) {
	if (world) {
		world->add_rigid_body(this, ePhysicsWorld_Flag::Owned);

		/**
		 * The piston is considered to be held stable within the walls of the cylinder bank with a force
		 * of 5k neuton (equal to ~510kg) and a damping factor of 200 neuton as friction force.
		 */
		world->add_constraint(new LineConstraint(this, wrist_pin_local(), bank->position, bank->theta, 5e3f, 2e2f), ePhysicsWorld_Flag::Owned);
	}
}

Piston::~Piston() {
}

float2 Piston::wrist_pin_local() const {
	return float2(0.0f, pin);
}

/* -------------------------------------------------------------------- */
/** \name Piston Utilities
 * \{ */

Piston *KER_piston_new(PhysicsWorld *world, CylinderBank *bank, float2 position, float mass, float wrist_pin) {
	PistonDescr descr;

	descr.bank = bank;
	descr.position = position;
	descr.mass = mass;
	descr.pin = wrist_pin;

	return new Piston(world, descr);
}

/** \} */
