#include "KER_constraint_link.hh"

LinkConstraint::LinkConstraint(RigidBody *body1, float2 local1, RigidBody *body2, float2 local2, float stiffness, float damping) : Constraint(2, 2) {
	this->body[0] = body1;
	this->body[1] = body2;
	this->local[0] = local1;
	this->local[1] = local2;
	this->stiffness = stiffness;
	this->damping = damping;
}

LinkConstraint::~LinkConstraint() {
}

void LinkConstraint::apply(PhysicsWorld *world, ConstraintMatrices *r_output) {
	/**
	 * Convert the local positions to global world positions using the rotation angle from the RigidBody.
	 * The transformation matrix would be;
	 *
	 * [ +cos(\theta) -sin(\theta) 0 ]
	 * [ +sin(\theta) +cos(\theta) 0 ] = R_z(\theta)
	 * [      0             0      1 ]
	 *
	 * This is the most simple matrix we have for rotating around center of mass.
	 */
	float4 p1 = float4(local[0], 0.0f, 1.0f);
	float4 p2 = float4(local[1], 0.0f, 1.0f);
	p1.x = cosf(body[0]->theta) * local[0].x - sinf(body[0]->theta) * local[0].y + body[0]->position.x;
	p1.y = sinf(body[0]->theta) * local[0].x + cosf(body[0]->theta) * local[0].y + body[0]->position.y;
	p2.x = cosf(body[1]->theta) * local[1].x - sinf(body[1]->theta) * local[1].y + body[1]->position.x;
	p2.y = sinf(body[1]->theta) * local[1].x + cosf(body[1]->theta) * local[1].y + body[1]->position.y;

	/**
	 * The constraint equation/function we are trying to solve here is the following;
	 *
	 * 					 [ p1.x - p2.x ]   [ 0 ]
	 * C(x, y, \theta) = [ p1.y - p2.y ] = [ 0 ]
	 * 					 [  < no-op >  ]   [ 0 ]
	 *
	 * Deriving each function we get the following Jacobian matrix.
	 */

	/* clang-format off */
	r_output->J[0] = float3x3(
		float3(1.0f, 0.0f, -sinf(body[0]->theta) * local[0].x - cosf(body[0]->theta) * local[0].y),
		float3(0.0f, 1.0f, +cosf(body[0]->theta) * local[0].x - sinf(body[0]->theta) * local[0].y),
		float3(0.0f, 0.0f, 																	 0.0f)
	);
	r_output->J[1] = float3x3(
		float3(1.0f, 0.0f, -sinf(body[1]->theta) * local[1].x - cosf(body[1]->theta) * local[1].y) * -1.0f,
		float3(0.0f, 1.0f, +cosf(body[1]->theta) * local[1].x - sinf(body[1]->theta) * local[1].y) * -1.0f,
		float3(0.0f, 0.0f, 																	 0.0f) * -1.0f
	);
	/** The derivative of the Jacobian matrix in respect to time, \dot{J} = \frac{dJ}{dt} = \frac{dJ}{d\theta} * \frac{d\theta}{dt} */
	r_output->dJ[0] = float3x3(
		float3(0.0f, 0.0f, -cosf(body[0]->theta) * local[0].x + sinf(body[0]->theta) * local[0].y) * body[0]->v_theta,
		float3(0.0f, 0.0f, -sinf(body[0]->theta) * local[0].x - cosf(body[0]->theta) * local[0].y) * body[0]->v_theta,
		float3(0.0f, 0.0f, 																	 0.0f) * body[0]->v_theta
	);
	r_output->dJ[1] = float3x3(
		float3(0.0f, 0.0f, +cosf(body[1]->theta) * local[1].x - sinf(body[1]->theta) * local[1].y) * body[1]->v_theta,
		float3(0.0f, 0.0f, +sinf(body[1]->theta) * local[1].x + cosf(body[1]->theta) * local[1].y) * body[1]->v_theta,
		float3(0.0f, 0.0f, 																	 0.0f) * body[1]->v_theta
	);
	/* clang-format on */

	/** The error is the distance between the two points. */
	r_output->C = float3(p1 - p2);
	r_output->stiffness = float3(stiffness);
	r_output->damping = float3(damping);
}
