#pragma once

#include "KER_cylinder_bank.hh"
#include "KER_physics.hh"
#include "KER_rigid_body.hh"

struct Piston;

/* -------------------------------------------------------------------- */
/** \name Rod Structure/Type
 * \{ */

struct PistonDescr {
	CylinderBank *bank;
	float2 position;
	float mass;
	float pin;
};

struct Piston : public RigidBody {
	Piston(PhysicsWorld *world, const PistonDescr &descr);
	virtual ~Piston();

	/**
	 * The "Wist Pin" is the connection point of the piston to the rod.
	 */
	float2 wrist_pin_local() const;

	CylinderBank *bank;
	float pin;
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Piston Utilities
 * \{ */

/**
 * Function to quickly create a new piston.
 * \param world The physics world that governs this piston.
 * \param bank The cylinder bank of this piston.
 * \param position The initial position of the piston.
 * \param mass The mass of the piston.
 * \param wrist_pin The location of the wrist pin on the piston (along the Y axis).
 * \return The newly created piston.
 */
Piston *KER_piston_new(PhysicsWorld *world, CylinderBank *bank, float2 position, float mass, float wrist_pin);

/** \} */
