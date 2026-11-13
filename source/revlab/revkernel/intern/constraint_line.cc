#include "KER_constraint_line.hh"

LineConstraint::LineConstraint(RigidBody *body, float2 local, float2 global, float angle, float stiffness, float damping) : Constraint(1, 1) {
	this->body[0] = body;
	this->local = local;
	this->global = global;
	this->angle = angle;
	this->stiffness = stiffness;
	this->damping = damping;
}
LineConstraint::~LineConstraint() {
}

void LineConstraint::apply(PhysicsWorld *world, ConstraintMatrices *r_output) {
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
	float4 p = float4(local, body[0]->theta, 1.0f);
	p.x = cosf(body[0]->theta) * local.x - sinf(body[0]->theta) * local.y + body[0]->position.x;
	p.y = sinf(body[0]->theta) * local.x + cosf(body[0]->theta) * local.y + body[0]->position.y;
	/** The normal of the line constraint. */
	float2 n = float2(-sinf(angle), +cosf(angle));

	/**
	 * The constraint equation/function we are trying to solve here is the following;
	 * C(x, y, \theta) = dot(vec2(p.x - global.x, p.y - global.y), n) = 0
	 * Deriving each function we get the following Jacobian matrix.
	 */

	const float jx = (-sinf(body[0]->theta) * local.x - cosf(body[0]->theta) * local.y) * n.x;
	const float jy = (+cosf(body[0]->theta) * local.x - sinf(body[0]->theta) * local.y) * n.y;

	const float djx = (-cosf(body[0]->theta) * local.x + sinf(body[0]->theta) * local.y) * n.x;	 // needs to be multiplied by d\theta
	const float djy = (-sinf(body[0]->theta) * local.x - cosf(body[0]->theta) * local.y) * n.y;	 // needs to be multiplied by d\theta

	/* clang-format off */
	r_output->J[0] = float3x3(
		float3( n.x,  n.y, jx + jy),
		float3(0.0f, 0.0f, 	  0.0f),
		float3(0.0f, 0.0f,    0.0f)
	);
	/** The derivative of the Jacobian matrix in respect to time, \dot{J} = \frac{dJ}{dt} = \frac{dJ}{d\theta} * \frac{d\theta}{dt} */
	r_output->dJ[0] = float3x3(
		float3(0.0f, 0.0f, djx + djy) * body[0]->v_theta,
		float3(0.0f, 0.0f, 		0.0f) * body[0]->v_theta,
		float3(0.0f, 0.0f, 		0.0f) * body[0]->v_theta
	);
	/* clang-format on */

	/** The error is the distance between the two points. */
	r_output->C = float3(revlab::math::dot(float2(p) - global, n), 0.0f, 0.0f);
	r_output->stiffness = float3(stiffness);
	r_output->damping = float3(damping);
}
