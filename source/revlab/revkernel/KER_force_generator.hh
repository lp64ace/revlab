#pragma once

#include "KER_physics.hh"

struct ForceGenerator;

/* -------------------------------------------------------------------- */
/** \name ForceGenerator Structure/Type
 * \{ */

struct ForceGenerator {
	ForceGenerator();
	virtual ~ForceGenerator();

	virtual void apply(PhysicsWorld *world) = 0;
};

/** \} */
