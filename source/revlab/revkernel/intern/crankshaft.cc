#include "KER_constraint_fixed_position.hh"
#include "KER_crankshaft.hh"
#include "KER_physics.hh"

Crankshaft::Crankshaft(PhysicsWorld *world, const CrankshaftDescr &descr) : RigidBody(descr.position, 0.0f, descr.mass, descr.intertia), journals(descr.journals) {
	if (world) {
		world->add_rigid_body(this, ePhysicsWorld_Flag::Owned);
		/**
		 * The center of the crankshaft (0, 0) is considered to be held stable in its starting position (this->position) using a force
		 * of 5k neuton (equal to ~510kg) and a damping factor of 100 neuton as friction force.
		 */
		world->add_constraint(new FixedPositionConstraint(this, float2(0.0f, 0.0f), this->position, 5e3f, 1e2f), ePhysicsWorld_Flag::Owned);
	}
}

Crankshaft::~Crankshaft() {
}

float2 Crankshaft::journal_local(const RodJournal &descr) const {
	return float2(cosf(descr.theta) * descr.radius, sinf(descr.theta) * descr.radius);
}

float2 Crankshaft::journal_local(const size_t index) const {
	assert(index >= 0);
	assert(index < journals.size());
	return journal_local(journals[index]);
}

/* -------------------------------------------------------------------- */
/** \name Crankshaft Utilities
 * \{ */

Crankshaft *KER_crankshaft_new(PhysicsWorld *world, float2 position, float mass, float intertia, const std::vector<RodJournal> &journals) {
	CrankshaftDescr descr;

	descr.position = position;
	descr.mass = mass;
	descr.intertia = intertia;
	descr.journals = journals;

	return new Crankshaft(world, descr);
}

/** \} */
