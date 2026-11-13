#include "LIB_math_matrix.hh"

#include "KER_constraint_fixed_position.hh"

FixedPositionConstraint::FixedPositionConstraint(RigidBody *body, float2 local, float2 global, float stiffness, float damping) : Constraint(1, 2) {
	this->body[0] = body;
	this->local = local;
	this->global = global;
	this->stiffness = stiffness;
	this->damping = damping;
}

FixedPositionConstraint::~FixedPositionConstraint() {
}

void FixedPositionConstraint::apply(PhysicsWorld *world, ConstraintMatrices *r_output) {
	/**
	 * Convert the local anchor position to global world position using the rotation angle from the RigidBody.
	 * The transformation matrix would be;
	 *
	 * [ +cos(\theta) -sin(\theta) 0 ]
	 * [ +sin(\theta) +cos(\theta) 0 ] = R_z(\theta)
	 * [      0             0      1 ]
	 *
	 * This is the most simple matrix we have for rotating around center of mass.
	 */

	float4 c = float4(this->local, 0.0f, 1.0f);
	float4 t = float4(this->global, 0.0f, 1.0f);

	/** Transform the local position to global coordinates. */
	c.x = cosf(body[0]->theta) * local.x - sinf(body[0]->theta) * local.y + body[0]->position.x;
	c.y = sinf(body[0]->theta) * local.x + cosf(body[0]->theta) * local.y + body[0]->position.y;

	/**
	 * The constraint equation/function we are trying to solve here is the following;
	 *
	 * 					 [ x + cos(\theta) * local.x - sin(\theta) * local.y - body->position.x ]   [ 0 ]
	 * C(x, y, \theta) = [ y + sin(\theta) * local.x + cos(\theta) * local.y - body->position.y ] = [ 0 ]
	 * 					 [                  < no constraint applied on \theta >                 ]   [ 0 ]
	 *
	 * Deriving each function we get the following Jacobian matrix.
	 */

	/* clang-format off */
	r_output->J[0] = float3x3(
		float3(1.0f, 0.0f, -sinf(body[0]->theta) * local.x - cosf(body[0]->theta) * local.y),
		float3(0.0f, 1.0f, +cosf(body[0]->theta) * local.x - sinf(body[0]->theta) * local.y),
		float3(0.0f, 0.0f, 															   0.0f)
	);
	/** The derivative of the Jacobian matrix in respect to time, \dot{J} = \frac{dJ}{dt} = \frac{dJ}{d\theta} * \frac{d\theta}{dt} */
	r_output->dJ[0] = float3x3(
		float3(0.0f, 0.0f, -cosf(body[0]->theta) * local.x + sinf(body[0]->theta) * local.y) * body[0]->v_theta,
		float3(0.0f, 0.0f, -sinf(body[0]->theta) * local.x - cosf(body[0]->theta) * local.y) * body[0]->v_theta,
		float3(0.0f, 0.0f, 															   0.0f) * body[0]->v_theta
	);
	/* clang-format on */

	/** The error is the offset from the target position to the current position. */
	r_output->C = float3(c - t);
	r_output->stiffness = float3(stiffness);
	r_output->damping = float3(damping);
}
