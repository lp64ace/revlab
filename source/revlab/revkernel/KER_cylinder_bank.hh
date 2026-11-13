#pragma once

#include "KER_physics.hh"
#include "KER_rigid_body.hh"

struct CylinderBank;

/* -------------------------------------------------------------------- */
/** \name Cylinder Bank Structure/Type
 * \{ */

struct CylinderBankDescr {
	float2 position;
	float angle;
	/**
	 * The bore is the internal diameter of each individual cylinder where the piston moves up and down.
	 * here we use it as the radius instead.
	 */
	float bore;
	/**
	 * Deck height is the exact distance from the centerline of the crankshaft to the top flat surface of the engine block.
	 */
	float deck_height;
};

struct CylinderBank : public RigidBody {
	CylinderBank(PhysicsWorld *world, const CylinderBankDescr &descr);
	virtual ~CylinderBank();

	float bore;
	float deck_height;
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Cylinder Bank Utilities
 * \{ */

/**
 * Function to quickly create a new cylinder bank.
 * \param world The physics world that governs this crankshaft.
 * \param position The position of the crankshaft in the world.
 * \param angle The angle of the cylinder bank.
 * \param bore The radius of each individual cylinder where the piston moves up and down.
 * \param deck_height the exact distance from the centerline of the crankshaft to the top flat surface of the engine block
 * \return The newly created crankshaft.
 */
CylinderBank *KER_cylinder_bank_new(PhysicsWorld *world, float2 position, float angle, float bore, float deck_height);

/** \} */
