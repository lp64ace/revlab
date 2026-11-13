# -----------------------------------------------------------------------------
# Define Include Directories

set(INC
	# Internal Include Directories
	PUBLIC .
	
	# External Include Directories
	
)

# -----------------------------------------------------------------------------
# Define System Include Directories

set(INC_SYS
	# External System Include Directories
	
)

# -----------------------------------------------------------------------------
# Define Source Files

set(SRC
	KER_camshaft.hh
	KER_constraint.hh
	KER_constraint_fixed_position.hh
	KER_constraint_line.hh
	KER_constraint_link.hh
	KER_crankshaft.hh
	KER_cylinder_bank.hh
	KER_differential_equation_euler_solver.hh
	KER_differential_equation_rk4_solver.hh
	KER_differential_equation_solver.hh
	KER_force_generator.hh
	KER_linear_equation_gauss_solver.hh
	KER_linear_equation_solver.hh
	KER_physics.hh
	KER_physics_generic.hh
	KER_piston.hh
	KER_rigid_body.hh
	KER_rod.hh

	intern/camshaft.cc
	intern/constraint.cc
	intern/constraint_fixed_position.cc
	intern/constraint_line.cc
	intern/constraint_link.cc
	intern/crankshaft.cc
	intern/cylinder_bank.cc
	intern/differential_equation_euler_solver.cc
	intern/differential_equation_rk4_solver.cc
	intern/differential_equation_solver.cc
	intern/linear_equation_gauss_solver.cc
	intern/linear_equation_solver.cc
	intern/physics.cc
	intern/physics_generic.cc
	intern/piston.cc
	intern/rigid_body.cc
	intern/rod.cc
	
)

# -----------------------------------------------------------------------------
# Define Library Dependencies

set(LIB
	# Internal Library Dependencies
	PUBLIC revlab::source::revlib

	# External Library Dependencies
	
)

# -----------------------------------------------------------------------------
# Declare Library

revlab_add_lib(revkernel "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
add_library(revlab::source::revkernel ALIAS revkernel)

# -----------------------------------------------------------------------------
# Define Include Directories (Test)

set(INC
	# Internal Include Directories
	PUBLIC .
	
	# External Include Directories
	
)

# -----------------------------------------------------------------------------
# Define System Include Directories (Test)

set(INC_SYS
	# External System Include Directories
	
)

# -----------------------------------------------------------------------------
# Define Source Files (Test)

set(TEST
	tests/crankshaft.cc

)

# -----------------------------------------------------------------------------
# Define Library Dependencies (Test)

set(LIB
	# Internal Library Dependencies
	revlab::source::revkernel
	
	# External Library Dependencies
	
)

# -----------------------------------------------------------------------------
# Declare Test

revlab_add_test_executable(revkernel "${TEST}" "${INC}" "${INC_SYS}" "${LIB}")
