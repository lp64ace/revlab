#include "KER_constraint_link.hh"
#include "KER_crankshaft.hh"
#include "KER_physics.hh"
#include "KER_piston.hh"
#include "KER_rod.hh"

Rod::Rod(PhysicsWorld *world, const RodDescr &descr) : RigidBody(float2(0.0f), 0.0f, descr.mass, descr.interia), piston(descr.piston), crankshaft(descr.crankshaft), length(descr.length) {
	assert(descr.journal >= 0);
	assert(descr.journal < crankshaft->journals.size());

	journal = -1;
	if (0 <= descr.journal && descr.journal < crankshaft->journals.size()) {
		journal = descr.journal;
	}

	if (world) {
		world->add_rigid_body(this, ePhysicsWorld_Flag::Owned);
		/**
		 * There is a link between the rod and the piston.
		 * We consider the link to be a very stiff spring of length zero.
		 * The stiffness is 5k neuton (equal to ~510kg) and a damping factor of 200 neuton as friction force.
		 */
		world->add_constraint(new LinkConstraint(
								  /** The first point of contact of the "spring"/"link" is the small end of the rod. */
								  this,
								  this->small_end_local(),
								  /** The second point of contact of the "spring"/"link" is the wrist pin of the piston. */
								  piston,
								  piston->wrist_pin_local(),
								  5e3f,
								  2e2f),
							  ePhysicsWorld_Flag::Owned);
		/**
		 * There is a link between the rod and the journal on the crankshaft.
		 * We consider the link to be a very stiff spring of length zero.
		 * The stiffness is 5k neuton (equal to ~510kg) and a damping factor of 200 neuton as friction force.
		 */
		world->add_constraint(new LinkConstraint(
								  /** The first point of contact of the "spring"/"link" is the big end of the rod. */
								  this,
								  this->big_end_local(),
								  /** The second point of contact of the "spring"/"link" is the journal of the crankshaft. */
								  crankshaft,
								  crankshaft->journal_local(journal),
								  5e3f,
								  2e2f),
							  ePhysicsWorld_Flag::Owned);
	}
}

Rod::~Rod() {
}

float2 Rod::big_end_local() const {
	return float2(0.0f, -(length) / 2);
}

float2 Rod::small_end_local() const {
	return float2(0.0f, +(length) / 2);
}

/* -------------------------------------------------------------------- */
/** \name Rod Utilities
 * \{ */

Rod *KER_rod_new(PhysicsWorld *world, Piston *piston, Crankshaft *crankshaft, size_t journal, float mass, float inertia, float length) {
	if (!(0 <= journal && journal < crankshaft->journals.size())) {
		return nullptr;
	}

	RodDescr descr;
	descr.piston = piston;
	descr.crankshaft = crankshaft;
	descr.journal = journal;
	descr.mass = mass;
	descr.interia = inertia;
	descr.length = length;

	return new Rod(world, descr);
}

/** \} */
