/**
* DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.
*
* This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001.
* Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of
* the Under Secretary of Defense for Research and Engineering.
*
* © 2023 Massachusetts Institute of Technology.
*
* The software/firmware is provided to you on an As-Is basis
*
* Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014).
* Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above.
* Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.
*/

#=
Double Integrator Unicycle HOCBF source code generator

Uses Symbolics.jl to compute C source code for HOCBF

Contributors: 	James Usevitch (james.usevitch@ll.mit.edu)
=#

using LinearAlgebra, Symbolics, Latexify

@variables x1, y1, v1, θ1, u1, w1
@variables x2, y2, v2, θ2, u2, w2
@variables δ


# States and control inputs
z1 = [x1; y1; v1; θ1]
z2 = [x2; y2; v2; θ2]

symvars = [x1 y1 v1 θ1 u1 w1 x2 y2 v2 θ2 u2 w2 δ]

# Dynamics
function f(z)
	return [sin(z[4])*z[3]; cos(z[4])*z[3]; 0; 0]
end

function g(z)
	return [0 0;0 0;1 -1;0 z[3]]
end


# CBF
function h(state1, state2)
	return norm(state1[1:2] - state2[1:2], 2)
end

# Linear alpha function
function alpha(z,k)
	return k*z
end

# ψ0
ψ0 = h(z1, z2)

# ψ1
dh_dz1 = Symbolics.gradient(h(z1, z2), z1)
dh_dz2 = Symbolics.gradient(h(z1, z2), z2)

# ψ1 = dh_dz1'*(f(state1) + g(state1)*[u1; w1]) + dh_dz2'*(f(state2) + g(state2)*[u2;w2]) + alpha(h(state1, state2), δ)
ψ1 = dh_dz1'*f(z1) + dh_dz2'*f(z2) + alpha(ψ0, δ) # dh_dz*g(values) is zero, so we don't include it.


# ψ2

dψ1_dz1 = Symbolics.gradient(ψ1, z1)
dψ1_dz2 = Symbolics.gradient(ψ1, z2)

ψ2 = dψ1_dz1'*(f(z1) + g(z1)*[u1; w1]) + dψ1_dz2'*(f(z2) + g(z2)*[u2; w2]) + alpha(ψ1, δ)


raw_ψ2_c_source = build_function(ψ2, symvars...; target=Symbolics.CTarget())

# Replace all double slashes `//` with a single one `/`. Bug due to rationals
ψ2_c_source = replace(raw_ψ2_c_source, "//"=>"/")

# Compute a, b
a = dψ1_dz1'*g(z1)

b = -(alpha(ψ1, δ) + dψ1_dz1'*f(z1) + dψ1_dz2'*(f(z2) + g(z2)*[u2; w2]))

raw_a_source = build_function(a, 1:len(symvars...; target=Symbolics.CTarget())
a_source = replace(raw_a_source, "//"=>"/")

raw_b_source = build_function(b, symvars...; target=Symbolics.CTarget())
b_source = replace(raw_b_source, "//"=>"/")