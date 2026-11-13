#include "LIB_math_matrix.hh"

#include "KER_force_generator.hh"
#include "KER_physics.hh"
#include "KER_rigid_body.hh"

#include <assert.h>

/* -------------------------------------------------------------------- */
/** \name Physics World Rigid Body Store
 * \{ */

/**
 * We use shallow copies to apply forces and solve constraints without destroying the original rigid bodies.
 * These shallow copies are initialzed in #PhysicsWorld::prepare.
 */
static void update_ridig_body_store(PhysicsRigidBodyStore *store) {
	if (!store->shallow_copy) {
		store->shallow_copy = new RigidBody();
	}

	store->shallow_copy->position = store->body->position;
	store->shallow_copy->velocity = store->body->velocity;
	store->shallow_copy->theta = store->body->theta;
	store->shallow_copy->v_theta = store->body->v_theta;

	store->shallow_copy->mass = store->body->mass;
	store->shallow_copy->inertia = store->body->inertia;

	store->force = float2(0.0f);
	store->torque = 0.0f;
}

static PhysicsRigidBodyStore init_rigid_body_store(RigidBody *reference, ePhysicsWorld_Flag flag) {
	PhysicsRigidBodyStore store;
	store.body = reference;
	store.shallow_copy = nullptr;
	store.flag = flag;
	return store;
}

/** Does not free the #PhysicsRigidBodyStore itself, returns true if the referenced pointer was freed too. */
static bool free_rigid_body_store_data(PhysicsRigidBodyStore *store, bool handle_own_pointer = true) {
	if (store->shallow_copy) {
		delete store->shallow_copy;
	}

	if (handle_own_pointer) {
		if ((store->flag & ePhysicsWorld_Flag::Owned) != ePhysicsWorld_Flag::None) {
			delete store->body;
			return true;
		}
	}
	return false;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Physics World Constraint Store
 * \{ */

static PhysicsConstraintStore init_constraint_store(Constraint *reference, ePhysicsWorld_Flag flag) {
	PhysicsConstraintStore store;
	store.constraint = reference;
	store.flag = flag;
	return store;
}

/** Does not free the #PhysicsConstraintStore itself, returns true if the referenced pointer was freed too. */
static bool free_constraint_store_data(PhysicsConstraintStore *store, bool handle_own_pointer = true) {
	if (handle_own_pointer) {
		if ((store->flag & ePhysicsWorld_Flag::Owned) != ePhysicsWorld_Flag::None) {
			delete store->constraint;
			return true;
		}
	}
	return false;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Physics World Force Generator Store
 * \{ */

static PhysicsForceGeneratorStore init_force_generator_store(ForceGenerator *reference, ePhysicsWorld_Flag flag) {
	PhysicsForceGeneratorStore store;
	store.force_generator = reference;
	store.flag = flag;
	return store;
}

/** Does not free the #PhysicsForceGeneratorStore itself, returns true if the referenced pointer was freed too. */
static bool free_force_generator_store_data(PhysicsForceGeneratorStore *store, bool handle_own_pointer = true) {
	if (handle_own_pointer) {
		if ((store->flag & ePhysicsWorld_Flag::Owned) != ePhysicsWorld_Flag::None) {
			delete store->force_generator;
			return true;
		}
	}
	return false;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Physics World
 * \{ */

PhysicsWorld::PhysicsWorld() = default;

PhysicsWorld::~PhysicsWorld() {
	for (std::vector<PhysicsRigidBodyStore>::iterator i = rigid_bodies.begin(); i != rigid_bodies.end(); i++) {
		PhysicsRigidBodyStore &store = *i;
		if (free_rigid_body_store_data(&store)) {
			i->body = nullptr;
		}
	}
	rigid_bodies.clear();

	for (std::vector<PhysicsConstraintStore>::iterator i = constraints.begin(); i != constraints.end(); i++) {
		PhysicsConstraintStore &store = *i;
		if (free_constraint_store_data(&store)) {
			i->constraint = nullptr;
		}
	}
	constraints.clear();

	for (std::vector<PhysicsForceGeneratorStore>::iterator i = force_generators.begin(); i != force_generators.end(); i++) {
		PhysicsForceGeneratorStore &store = *i;
		if (free_force_generator_store_data(&store)) {
			i->force_generator = nullptr;
		}
	}
	force_generators.clear();
}

void PhysicsWorld::process(float dt) {
	for (accumulated += dt; accumulated >= PhysicsWorld::delta_time; accumulated -= PhysicsWorld::delta_time) {
		update(PhysicsWorld::delta_time);
		frame++;
	}
}

void PhysicsWorld::update(float dt) {
	/* No-op */
}

void PhysicsWorld::prepare() {
	for (std::vector<PhysicsRigidBodyStore>::iterator i = rigid_bodies.begin(); i != rigid_bodies.end(); i++) {
		PhysicsRigidBodyStore &store = *i;

		/**
		 * Update the shallow copies so that we can start computing the new frame.
		 */
		update_ridig_body_store(&store);
	}
}

// RigidBody

PhysicsRigidBodyStore *PhysicsWorld::add_rigid_body(RigidBody *body, ePhysicsWorld_Flag flag) {
	assert(!has_rigid_body(body));

	rigid_bodies.push_back(init_rigid_body_store(body, flag));
	return &rigid_bodies[rigid_bodies.size() - 1];
}

PhysicsRigidBodyStore *PhysicsWorld::get_rigid_body_store(const RigidBody *body) {
	for (std::vector<PhysicsRigidBodyStore>::iterator i = rigid_bodies.begin(); i != rigid_bodies.end(); i++) {
		if (i->body == body) {
			PhysicsRigidBodyStore &store = *i;
			return &store;
		}
	}

	return NULL;
}

RigidBody *PhysicsWorld::remove_rigid_body(const RigidBody *body) {
	assert(has_rigid_body(body));

	for (std::vector<PhysicsRigidBodyStore>::iterator i = rigid_bodies.begin(); i != rigid_bodies.end(); i++) {
		if (i->body == body) {
			PhysicsRigidBodyStore &store = *i;
			if ((store.flag & ePhysicsWorld_Flag::Owned) != ePhysicsWorld_Flag::None) {
				delete store.body;
			}
			free_rigid_body_store_data(&store);
			rigid_bodies.erase(i);
			if ((store.flag & ePhysicsWorld_Flag::Owned) == ePhysicsWorld_Flag::None) {
				return store.body;
			}
			break;
		}
	}

	return NULL;
}

RigidBody *PhysicsWorld::detach_rigid_body(const RigidBody *body) {
	assert(has_rigid_body(body));

	for (std::vector<PhysicsRigidBodyStore>::iterator itr = rigid_bodies.begin(); itr != rigid_bodies.end(); itr++) {
		if (itr->body == body) {
			PhysicsRigidBodyStore store = *itr;
			free_rigid_body_store_data(&store, false);
			rigid_bodies.erase(itr);
			return store.body;
		}
	}

	return NULL;
}

bool PhysicsWorld::has_rigid_body(const RigidBody *body) {
	for (size_t index = 0; index < rigid_bodies.size(); index++) {
		if (rigid_bodies[index].body == body) {
			return true;
		}
	}
	return false;
}

// Constraint

PhysicsConstraintStore *PhysicsWorld::add_constraint(Constraint *constraint, ePhysicsWorld_Flag flag) {
	assert(!has_constraint(constraint));

	constraints.push_back(init_constraint_store(constraint, flag));
	return &constraints[constraints.size() - 1];
}

Constraint *PhysicsWorld::remove_constraint(const Constraint *constraint) {
	assert(has_constraint(constraint));

	for (std::vector<PhysicsConstraintStore>::iterator i = constraints.begin(); i != constraints.end(); i++) {
		if (i->constraint == constraint) {
			PhysicsConstraintStore &store = *i;
			if ((store.flag & ePhysicsWorld_Flag::Owned) != ePhysicsWorld_Flag::None) {
				delete store.constraint;
			}
			free_constraint_store_data(&store);
			constraints.erase(i);
			if ((store.flag & ePhysicsWorld_Flag::Owned) == ePhysicsWorld_Flag::None) {
				return store.constraint;
			}
			break;
		}
	}

	return NULL;
}

Constraint *PhysicsWorld::detach_constraint(const Constraint *constraint) {
	assert(has_constraint(constraint));

	for (std::vector<PhysicsConstraintStore>::iterator itr = constraints.begin(); itr != constraints.end(); itr++) {
		if (itr->constraint == constraint) {
			PhysicsConstraintStore store = *itr;
			free_constraint_store_data(&store, false);
			constraints.erase(itr);
			return store.constraint;
		}
	}

	return NULL;
}

bool PhysicsWorld::has_constraint(const Constraint *constraint) {
	for (size_t index = 0; index < constraints.size(); index++) {
		if (constraints[index].constraint == constraint) {
			return true;
		}
	}
	return false;
}

// ForceGenerators

PhysicsForceGeneratorStore *PhysicsWorld::add_force_generator(ForceGenerator *force_generator, ePhysicsWorld_Flag flag) {
	assert(!has_force_generator(force_generator));

	force_generators.push_back(init_force_generator_store(force_generator, flag));
	return &force_generators[force_generators.size() - 1];
}

ForceGenerator *PhysicsWorld::remove_force_generator(const ForceGenerator *force_generator) {
	assert(has_force_generator(force_generator));

	for (std::vector<PhysicsForceGeneratorStore>::iterator i = force_generators.begin(); i != force_generators.end(); i++) {
		if (i->force_generator == force_generator) {
			PhysicsForceGeneratorStore &store = *i;
			if ((store.flag & ePhysicsWorld_Flag::Owned) != ePhysicsWorld_Flag::None) {
				delete store.force_generator;
			}
			free_force_generator_store_data(&store);
			force_generators.erase(i);
			if ((store.flag & ePhysicsWorld_Flag::Owned) == ePhysicsWorld_Flag::None) {
				return store.force_generator;
			}
			break;
		}
	}

	return NULL;
}

ForceGenerator *PhysicsWorld::detach_force_generator(const ForceGenerator *force_generator) {
	assert(has_force_generator(force_generator));

	for (std::vector<PhysicsForceGeneratorStore>::iterator itr = force_generators.begin(); itr != force_generators.end(); itr++) {
		if (itr->force_generator == force_generator) {
			PhysicsForceGeneratorStore store = *itr;
			free_force_generator_store_data(&store, false);
			force_generators.erase(itr);
			return store.force_generator;
		}
	}

	return NULL;
}

bool PhysicsWorld::has_force_generator(const ForceGenerator *force_generator) {
	for (size_t index = 0; index < force_generators.size(); index++) {
		if (force_generators[index].force_generator == force_generator) {
			return true;
		}
	}
	return false;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Physics Store Utilities
 * \{ */

/** This function should only be called during evaluation. */
float2 KER_physics_store_local_to_world(PhysicsRigidBodyStore *store, float2 local) {
	assert(store->shallow_copy != nullptr);

	RigidBody *erb = store->shallow_copy;

	float2 p;
	p.x = cosf(erb->theta) * local.x - sinf(erb->theta) * local.y + erb->position.x;
	p.y = sinf(erb->theta) * local.x + cosf(erb->theta) * local.y + erb->position.y;
	return p;
}

/** This function should only be called during evaluation. */
void KER_physics_store_apply_force(PhysicsRigidBodyStore *store, float2 local, float2 force) {
	float2 global = KER_physics_store_local_to_world(store, local);

	store->force += force;
	/**
	 * TODO: move this calculation to revlib, this is the determinant of the following matrix;
	 *
	 * 	   [       \vec{x}             \vec{y}             \vec{z}      ]
	 * M = [ global.x - local.x  global.y - local.y  global.z - local.z ]
	 * 	   [      force.x              force.y             force.z      ]
	 *
	 * Since we are working on 2D this becomes (local.y - global.y) * force.x + (global.x - local.x) * force.y
	 */
	store->torque += (local.y - global.y) * force.x + (global.x - local.y) * force.y;
}

/** \} */
