#pragma once

#include "LIB_math_matrix.hh"
#include "LIB_math_vector.hh"

struct RigidBody;

/* -------------------------------------------------------------------- */
/** \name RigidBody Structure/Type
 * \{ */

struct RigidBody {
	RigidBody();
	RigidBody(float2 position, float theta, float mass, float inertia);
	virtual ~RigidBody();

	/**
	 * \brief The current position of the rigid body in the physics world.
	 * This value is measured in (m) meters.
	 */
	float2 position = float2(0.0f, 0.0f);
	/**
	 * \brief The current translation velocity of the rigid body.
	 * This value is measured in (m/s) meters per second.
	 */
	float2 velocity = float2(0.0f, 0.0f);
	/**
	 * \brief The current angle of the rigid body.
	 * This value is measured in (rad) radians.
	 */
	float theta = 0.0f;
	/**
	 * \brief The current rotational velocity (in the Z axis) of the rigid body.
	 * This value is measured in (rad/s) radians per second.
	 */
	float v_theta = 0.0f;
	/** Mass tells you how hard it is to speed up or slow down a sliding box (F = ma). */
	float mass = 1.0f;
	/**
	 * Inertia acts as the rotational counterpart to regular mass in linear motion.
	 * While linear mass tells you how hard it is to speed up or slow down a sliding box (F = ma),
	 * the moment of inertia tells you how hard it is to twist or spin an object (T = Ia),
	 */
	float inertia = 1.0f;
};

/** \} */
