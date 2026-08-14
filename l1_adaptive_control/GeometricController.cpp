#include "GeometricController.hpp"

#include <math.h>

namespace
{

static constexpr float GRAVITY_MSS = 9.80665f;

float dot3(const float a[3], const float b[3])
{
return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void copy3(const float in[3], float out[3])
{
out[0] = in[0];
out[1] = in[1];
out[2] = in[2];
}

void cross3(const float a[3], const float b[3], float out[3])
{
out[0] = a[1] * b[2] - a[2] * b[1];
out[1] = a[2] * b[0] - a[0] * b[2];
out[2] = a[0] * b[1] - a[1] * b[0];
}

bool normalize3(float v[3])
{
const float n = sqrtf(dot3(v, v));

if (n < 1e-6f || !isfinite(n)) {
return false;
}

v[0] /= n;
v[1] /= n;
v[2] /= n;

return true;
}

bool is_finite3(const float v[3])
{
return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

bool is_finite_matrix3(const float M[3][3])
{
for (int row = 0; row < 3; row++) {
for (int col = 0; col < 3; col++) {
if (!isfinite(M[row][col])) {
return false;
}
}
}

return true;
}

void quat_to_rotation_matrix_body_to_ned(const float q[4], float R[3][3])
{
const float w = q[0];
const float x = q[1];
const float y = q[2];
const float z = q[3];

R[0][0] = 1.0f - 2.0f * (y * y + z * z);
R[0][1] = 2.0f * (x * y - w * z);
R[0][2] = 2.0f * (x * z + w * y);

R[1][0] = 2.0f * (x * y + w * z);
R[1][1] = 1.0f - 2.0f * (x * x + z * z);
R[1][2] = 2.0f * (y * z - w * x);

R[2][0] = 2.0f * (x * z - w * y);
R[2][1] = 2.0f * (y * z + w * x);
R[2][2] = 1.0f - 2.0f * (x * x + y * y);
}

void set_matrix_column(float R[3][3], int col, const float v[3])
{
R[0][col] = v[0];
R[1][col] = v[1];
R[2][col] = v[2];
}

void get_matrix_column(const float R[3][3], int col, float out[3])
{
out[0] = R[0][col];
out[1] = R[1][col];
out[2] = R[2][col];
}

void mat_transpose_mat(const float A[3][3], const float B[3][3], float out[3][3])
{
for (int row = 0; row < 3; row++) {
for (int col = 0; col < 3; col++) {
out[row][col] = 0.f;

for (int k = 0; k < 3; k++) {
out[row][col] += A[k][row] * B[k][col];
}
}
}
}

void mat_vec_mul(const float A[3][3], const float v[3], float out[3])
{
for (int row = 0; row < 3; row++) {
out[row] = A[row][0] * v[0] + A[row][1] * v[1] + A[row][2] * v[2];
}
}

void mat_sub(const float A[3][3], const float B[3][3], float out[3][3])
{
for (int row = 0; row < 3; row++) {
for (int col = 0; col < 3; col++) {
out[row][col] = A[row][col] - B[row][col];
}
}
}

void hat3(const float v[3], float out[3][3])
{
out[0][0] = 0.f;
out[0][1] = -v[2];
out[0][2] = v[1];
out[1][0] = v[2];
out[1][1] = 0.f;
out[1][2] = -v[0];
out[2][0] = -v[1];
out[2][1] = v[0];
out[2][2] = 0.f;
}

void mat_mul(const float A[3][3], const float B[3][3], float out[3][3])
{
for (int row = 0; row < 3; row++) {
for (int col = 0; col < 3; col++) {
out[row][col] = 0.f;

for (int k = 0; k < 3; k++) {
out[row][col] += A[row][k] * B[k][col];
}
}
}
}

void vee3(const float M[3][3], float out[3])
{
out[0] = M[2][1];
out[1] = M[0][2];
out[2] = M[1][0];
}

void inertia_mul(const float inertia_kg_m2[3], const float v[3], float out[3])
{
out[0] = inertia_kg_m2[0] * v[0];
out[1] = inertia_kg_m2[1] * v[1];
out[2] = inertia_kg_m2[2] * v[2];
}

void compute_rotation_error(const float R[3][3], const float Rd[3][3], float eR[3])
{
float A[3][3]{};

for (int i = 0; i < 3; i++) {
for (int j = 0; j < 3; j++) {
float RdT_R = 0.f;
float RT_Rd = 0.f;

for (int k = 0; k < 3; k++) {
RdT_R += Rd[k][i] * R[k][j];
RT_Rd += R[k][i] * Rd[k][j];
}

A[i][j] = 0.5f * (RdT_R - RT_Rd);
}
}

eR[0] = A[2][1];
eR[1] = A[0][2];
eR[2] = A[1][0];
}

bool unit_vec_with_derivatives(const float q[3],
       const float q_dot[3],
       const float q_ddot[3],
       float u[3],
       float u_dot[3],
       float u_ddot[3])
{
const float nq2 = dot3(q, q);
const float nq = sqrtf(nq2);

if (nq < 1e-6f || !isfinite(nq)) {
return false;
}

const float nq3 = nq2 * nq;
const float nq5 = nq3 * nq2;

const float q_qdot = dot3(q, q_dot);
const float qdot_qdot = dot3(q_dot, q_dot);
const float q_qddot = dot3(q, q_ddot);

for (int i = 0; i < 3; i++) {
u[i] = q[i] / nq;
u_dot[i] = q_dot[i] / nq - q[i] * q_qdot / nq3;
u_ddot[i] = q_ddot[i] / nq
    - q_dot[i] * 2.f * q_qdot / nq3
    - q[i] * (qdot_qdot + q_qddot) / nq3
    + q[i] * 3.f * q_qdot * q_qdot / nq5;
}

return is_finite3(u) && is_finite3(u_dot) && is_finite3(u_ddot);
}

bool output_is_finite(const GeometricController::Output &output)
{
return isfinite(output.target_thrust)
       && isfinite(output.target_thrust_dot)
       && is_finite3(output.r_error)
       && is_finite3(output.v_error)
       && is_finite3(output.a_error)
       && is_finite3(output.j_error)
       && is_finite3(output.target_force)
       && is_finite3(output.target_force_dot)
       && is_finite3(output.target_force_ddot)
       && is_finite3(output.z_axis)
       && is_finite3(output.b3_dot)
       && is_finite3(output.x_axis_desired)
       && is_finite3(output.y_axis_desired)
       && is_finite3(output.z_axis_desired)
       && is_finite3(output.x_axis_desired_dot)
       && is_finite3(output.y_axis_desired_dot)
       && is_finite3(output.z_axis_desired_dot)
       && is_finite3(output.x_axis_desired_ddot)
       && is_finite3(output.y_axis_desired_ddot)
       && is_finite3(output.z_axis_desired_ddot)
       && is_finite_matrix3(output.Rdes)
       && is_finite_matrix3(output.Rd_dot)
       && is_finite_matrix3(output.Rd_ddot)
       && is_finite3(output.b3c)
       && is_finite3(output.b3c_dot)
       && is_finite3(output.b3c_ddot)
       && is_finite3(output.b2c)
       && is_finite3(output.b2c_dot)
       && is_finite3(output.b2c_ddot)
       && is_finite3(output.eR)
       && is_finite3(output.Omegad)
       && is_finite3(output.Omegad_dot)
       && is_finite3(output.ew)
       && is_finite3(output.M_feedback)
       && is_finite3(output.M_feedforward)
       && is_finite3(output.momentAdd)
       && is_finite3(output.M);
}

}

bool GeometricController::update(const Input &input, Output &output)
{
_last_input = input;
output = Output{};
output.timestamp_us = input.timestamp_us;
const float kg_vehicleMass = _parameters.mass_kg;
const float *Kp = _parameters.position_gain;
const float *Kv = _parameters.velocity_gain;
const float *KR = _parameters.rotation_gain;
const float *KOmega = _parameters.angular_velocity_gain;
const float *J = _parameters.inertia_kg_m2;
const float *Omega = input.angular_velocity_body;

if (!input.state_valid_for_control || !input.armed || input.failsafe) {
output.valid = false;
_last_output = output;
return true;
}

for (int i = 0; i < 3; i++) {
output.r_error[i] = input.position_ned[i] - input.target_position_ned[i];
output.v_error[i] = input.velocity_ned[i] - input.target_velocity_ned[i];
}

output.target_force[0] =
kg_vehicleMass * input.target_acceleration_ned[0]
- Kp[0] * output.r_error[0]
- Kv[0] * output.v_error[0];

output.target_force[1] =
kg_vehicleMass * input.target_acceleration_ned[1]
- Kp[1] * output.r_error[1]
- Kv[1] * output.v_error[1];

output.target_force[2] =
kg_vehicleMass * (input.target_acceleration_ned[2] - GRAVITY_MSS)
- Kp[2] * output.r_error[2]
- Kv[2] * output.v_error[2];

if (input.manual_tilt_enabled) {
const float vertical_force_ned = output.target_force[2];
const float inverse_vertical_axis =
1.f / fmaxf(input.manual_desired_body_z_axis_ned[2], 0.5f);

for (int i = 0; i < 3; i++) {
output.target_force[i] =
input.manual_desired_body_z_axis_ned[i] * vertical_force_ned * inverse_vertical_axis;
}

} else if (!input.yaw_control_enabled) {
// A single-motor-out quadrotor cannot reliably hold horizontal position
// during the spin-up transient. Keep the desired thrust axis vertical and
// prioritize altitude plus reduced-attitude stabilization.
output.target_force[0] = 0.f;
output.target_force[1] = 0.f;
}

float R[3][3]{};
quat_to_rotation_matrix_body_to_ned(input.quat_body_to_ned, R);
get_matrix_column(R, 2, output.z_axis);

if (!is_finite_matrix3(R) || !is_finite3(output.z_axis)) {
output.valid = false;
_last_output = output;
return true;
}

output.target_thrust = input.yaw_control_enabled
	? -dot3(output.target_force, output.z_axis)
	: sqrtf(dot3(output.target_force, output.target_force));

output.a_error[0] =
-output.z_axis[0] * output.target_thrust / kg_vehicleMass
- input.target_acceleration_ned[0];

output.a_error[1] =
-output.z_axis[1] * output.target_thrust / kg_vehicleMass
- input.target_acceleration_ned[1];

output.a_error[2] =
GRAVITY_MSS
- output.z_axis[2] * output.target_thrust / kg_vehicleMass
- input.target_acceleration_ned[2];

output.target_force_dot[0] =
-Kp[0] * output.v_error[0]
-Kv[0] * output.a_error[0]
+ kg_vehicleMass * input.target_jerk_ned[0];

output.target_force_dot[1] =
-Kp[1] * output.v_error[1]
-Kv[1] * output.a_error[1]
+ kg_vehicleMass * input.target_jerk_ned[1];

output.target_force_dot[2] =
-Kp[2] * output.v_error[2]
-Kv[2] * output.a_error[2]
+ kg_vehicleMass * input.target_jerk_ned[2];

if (input.manual_tilt_enabled) {
output.target_force_dot[0] = 0.f;
output.target_force_dot[1] = 0.f;
output.target_force_dot[2] = 0.f;
}

const float omega_cross_e3[3] = {
Omega[1],
-Omega[0],
0.f
};
mat_vec_mul(R, omega_cross_e3, output.b3_dot);

output.target_thrust_dot =
-dot3(output.target_force_dot, output.z_axis)
-dot3(output.target_force, output.b3_dot);

if (input.manual_tilt_enabled) {
output.target_thrust_dot = 0.f;
}

output.j_error[0] =
-output.z_axis[0] * output.target_thrust_dot / kg_vehicleMass
-output.b3_dot[0] * output.target_thrust / kg_vehicleMass
-input.target_jerk_ned[0];

output.j_error[1] =
-output.z_axis[1] * output.target_thrust_dot / kg_vehicleMass
-output.b3_dot[1] * output.target_thrust / kg_vehicleMass
-input.target_jerk_ned[1];

output.j_error[2] =
-output.z_axis[2] * output.target_thrust_dot / kg_vehicleMass
-output.b3_dot[2] * output.target_thrust / kg_vehicleMass
-input.target_jerk_ned[2];

output.target_force_ddot[0] =
-Kp[0] * output.a_error[0]
-Kv[0] * output.j_error[0]
+ kg_vehicleMass * input.target_snap_ned[0];

output.target_force_ddot[1] =
-Kp[1] * output.a_error[1]
-Kv[1] * output.j_error[1]
+ kg_vehicleMass * input.target_snap_ned[1];

output.target_force_ddot[2] =
-Kp[2] * output.a_error[2]
-Kv[2] * output.j_error[2]
+ kg_vehicleMass * input.target_snap_ned[2];

if (input.manual_tilt_enabled) {
output.target_force_ddot[0] = 0.f;
output.target_force_ddot[1] = 0.f;
output.target_force_ddot[2] = 0.f;
}

const float minus_target_force[3] = {
-output.target_force[0],
-output.target_force[1],
-output.target_force[2]
};

const float minus_target_force_dot[3] = {
-output.target_force_dot[0],
-output.target_force_dot[1],
-output.target_force_dot[2]
};

const float minus_target_force_ddot[3] = {
-output.target_force_ddot[0],
-output.target_force_ddot[1],
-output.target_force_ddot[2]
};

if (!unit_vec_with_derivatives(minus_target_force,
			       minus_target_force_dot,
			       minus_target_force_ddot,
				       output.b3c,
				       output.b3c_dot,
				       output.b3c_ddot)) {
output.valid = false;
_last_output = output;
return true;
}

copy3(output.b3c, output.z_axis_desired);
copy3(output.b3c_dot, output.z_axis_desired_dot);
copy3(output.b3c_ddot, output.z_axis_desired_ddot);

const float yaw = input.target_yaw;
const float yaw_dot = input.target_yaw_rate;
const float yaw_ddot = input.target_yaw_accel;

const float x_c_des[3] = {
cosf(yaw),
sinf(yaw),
0.f
};

const float x_c_des_dot[3] = {
-sinf(yaw) * yaw_dot,
cosf(yaw) * yaw_dot,
0.f
};

const float x_c_des_ddot[3] = {
-cosf(yaw) * yaw_dot * yaw_dot - sinf(yaw) * yaw_ddot,
-sinf(yaw) * yaw_dot * yaw_dot + cosf(yaw) * yaw_ddot,
0.f
};

cross3(output.b3c, x_c_des, output.A2);

float A2_dot_part1[3]{};
float A2_dot_part2[3]{};
cross3(output.b3c, x_c_des_dot, A2_dot_part1);
cross3(output.b3c_dot, x_c_des, A2_dot_part2);

for (int i = 0; i < 3; i++) {
output.A2_dot[i] = A2_dot_part1[i] + A2_dot_part2[i];
}

float A2_ddot_part1[3]{};
float A2_ddot_part2[3]{};
float A2_ddot_part3[3]{};
cross3(output.b3c, x_c_des_ddot, A2_ddot_part1);
cross3(output.b3c_dot, x_c_des_dot, A2_ddot_part2);
cross3(output.b3c_ddot, x_c_des, A2_ddot_part3);

for (int i = 0; i < 3; i++) {
output.A2_ddot[i] =
A2_ddot_part1[i]
+ 2.f * A2_ddot_part2[i]
+ A2_ddot_part3[i];
}

if (!unit_vec_with_derivatives(output.A2,
			       output.A2_dot,
			       output.A2_ddot,
			       output.b2c,
			       output.b2c_dot,
			       output.b2c_ddot)) {
output.valid = false;
_last_output = output;
return true;
}

copy3(output.b2c, output.y_axis_desired);
copy3(output.b2c_dot, output.y_axis_desired_dot);
copy3(output.b2c_ddot, output.y_axis_desired_ddot);

cross3(output.y_axis_desired,
       output.z_axis_desired,
       output.x_axis_desired);

float b1c_dot_part1[3]{};
float b1c_dot_part2[3]{};
cross3(output.y_axis_desired_dot,
       output.z_axis_desired,
       b1c_dot_part1);
cross3(output.y_axis_desired,
       output.z_axis_desired_dot,
       b1c_dot_part2);

for (int i = 0; i < 3; i++) {
output.x_axis_desired_dot[i] = b1c_dot_part1[i] + b1c_dot_part2[i];
}

float b1c_ddot_part1[3]{};
float b1c_ddot_part2[3]{};
float b1c_ddot_part3[3]{};
cross3(output.y_axis_desired_ddot,
       output.z_axis_desired,
       b1c_ddot_part1);
cross3(output.y_axis_desired_dot,
       output.z_axis_desired_dot,
       b1c_ddot_part2);
cross3(output.y_axis_desired,
       output.z_axis_desired_ddot,
       b1c_ddot_part3);

for (int i = 0; i < 3; i++) {
output.x_axis_desired_ddot[i] =
b1c_ddot_part1[i]
+ 2.f * b1c_ddot_part2[i]
+ b1c_ddot_part3[i];
}

if (!normalize3(output.x_axis_desired)) {
output.valid = false;
_last_output = output;
return true;
}

set_matrix_column(output.Rdes, 0, output.x_axis_desired);
set_matrix_column(output.Rdes, 1, output.y_axis_desired);
set_matrix_column(output.Rdes, 2, output.z_axis_desired);

set_matrix_column(output.Rd_dot, 0, output.x_axis_desired_dot);
set_matrix_column(output.Rd_dot, 1, output.y_axis_desired_dot);
set_matrix_column(output.Rd_dot, 2, output.z_axis_desired_dot);

set_matrix_column(output.Rd_ddot, 0, output.x_axis_desired_ddot);
set_matrix_column(output.Rd_ddot, 1, output.y_axis_desired_ddot);
set_matrix_column(output.Rd_ddot, 2, output.z_axis_desired_ddot);

compute_rotation_error(R, output.Rdes, output.eR);

float RdT_Rd_dot[3][3]{};
mat_transpose_mat(output.Rdes, output.Rd_dot, RdT_Rd_dot);
vee3(RdT_Rd_dot, output.Omegad);

float RdT_Rd_ddot[3][3]{};
float Omegad_hat[3][3]{};
float Omegad_hat_sq[3][3]{};
float Omegad_dot_matrix[3][3]{};
mat_transpose_mat(output.Rdes, output.Rd_ddot, RdT_Rd_ddot);
hat3(output.Omegad, Omegad_hat);
mat_mul(Omegad_hat, Omegad_hat, Omegad_hat_sq);
mat_sub(RdT_Rd_ddot, Omegad_hat_sq, Omegad_dot_matrix);
vee3(Omegad_dot_matrix, output.Omegad_dot);

float RT_Rd[3][3]{};
mat_transpose_mat(R, output.Rdes, RT_Rd);

float RT_Rd_Omegad[3]{};
float RT_Rd_Omegad_dot[3]{};
mat_vec_mul(RT_Rd, output.Omegad, RT_Rd_Omegad);
mat_vec_mul(RT_Rd, output.Omegad_dot, RT_Rd_Omegad_dot);

for (int i = 0; i < 3; i++) {
output.ew[i] = Omega[i] - RT_Rd_Omegad[i];
}

if (!input.yaw_control_enabled) {
// Reduced-attitude error: align the current thrust axis with the desired
// thrust axis while leaving rotation about that axis unconstrained.
float reduced_rotation_error_ned[3]{};
cross3(output.z_axis_desired, output.z_axis, reduced_rotation_error_ned);

for (int i = 0; i < 3; i++) {
output.eR[i] =
R[0][i] * reduced_rotation_error_ned[0]
+ R[1][i] * reduced_rotation_error_ned[1]
+ R[2][i] * reduced_rotation_error_ned[2];
}

output.ew[0] = Omega[0];
output.ew[1] = Omega[1];
output.ew[2] = 0.f;
}

output.M_feedback[0] =
-KR[0] * output.eR[0]
-KOmega[0] * output.ew[0];

output.M_feedback[1] =
-KR[1] * output.eR[1]
-KOmega[1] * output.ew[1];

output.M_feedback[2] =
input.yaw_control_enabled
? -KR[2] * output.eR[2] - KOmega[2] * output.ew[2]
: 0.f;

float Omega_hat[3][3]{};
float Omega_hat_RT_Rd_Omegad[3]{};
hat3(Omega, Omega_hat);
mat_vec_mul(Omega_hat, RT_Rd_Omegad, Omega_hat_RT_Rd_Omegad);

const float feedforward_argument[3] = {
Omega_hat_RT_Rd_Omegad[0] - RT_Rd_Omegad_dot[0],
Omega_hat_RT_Rd_Omegad[1] - RT_Rd_Omegad_dot[1],
Omega_hat_RT_Rd_Omegad[2] - RT_Rd_Omegad_dot[2]
};

float J_feedforward_argument[3]{};
inertia_mul(J, feedforward_argument, J_feedforward_argument);

for (int i = 0; i < 3; i++) {
output.M_feedforward[i] = -J_feedforward_argument[i];
}

inertia_mul(J, Omega, output.JOmega);
cross3(Omega,
       output.JOmega,
       output.momentAdd);

for (int i = 0; i < 3; i++) {
output.M[i] =
output.M_feedback[i]
+ output.M_feedforward[i]
+ output.momentAdd[i];
}

if (!input.yaw_control_enabled) {
for (int i = 0; i < 3; i++) {
output.M_feedforward[i] = 0.f;
}

output.M[0] = output.M_feedback[0] + output.momentAdd[0];
output.M[1] = output.M_feedback[1] + output.momentAdd[1];
output.M[2] = 0.f;
}

output.valid = output_is_finite(output) && output.target_thrust > 0.f;

if (!output.valid) {
output.M[0] = 0.f;
output.M[1] = 0.f;
output.M[2] = 0.f;
output.target_thrust = 0.f;
}

_last_output = output;

return true;
}
