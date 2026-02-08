#include "control.hpp"
#include "structs.hpp"

extern State state;
extern Throttle throttle;

const float dt = 0.1;   // 100 ms control loop

/* ================= OUTER LOOP PI (ANGLES) ================= */
float Kp_ang = 5.0;
float Ki_ang = 1.0;
float rollInt = 0;
float pitchInt = 0;

/* ================= INNER LOOP LQR (RATES) ================= */
float K_lqr[3][2] = {
  { 1.5, -0.1},
  { -1.5, -0.1 },
  { 0.0, 2.0 }
};
const float U_MAX = 1.0;
float u_smooth[3] = { 0, 0, 0 };
const float beta = 0.2; // LQR output smoothing factor




/* ================= FUNCTIONS ================= */
int clampPWM(int value) {
    if (value >= 0)
        return constrain(value + 48, 0, 1000);
    else if (value <= 0)
        return constrain(-value + 1049, 1001, 2000);
}


void control::update() {


    // ================= OUTER LOOP PI =================
    float roll_rad = state.roll * DEG_TO_RAD;
    float pitch_rad = state.pitch * DEG_TO_RAD;

    rollInt += roll_rad * dt;
    pitchInt += pitch_rad * dt;

    // Limit integrator to prevent windup
    rollInt = constrain(rollInt, -0.02, 0.02);
    pitchInt = constrain(pitchInt, -0.02, 0.02);

    float wx_ref = Kp_ang * roll_rad + Ki_ang * rollInt;
    float wy_ref = Kp_ang * pitch_rad + Ki_ang * pitchInt;

    // ================= INNER LOOP LQR =================
    float omega_err[2] = { state.wx - wx_ref, state.wy - wy_ref };

    // Deadband for gyro errors
    for (int i = 0; i < 2; i++) {
        if (abs(omega_err[i]) < 0.01) omega_err[i] = 0;
    }

    float u[3];
    u[0] = -(K_lqr[0][0] * omega_err[0] + K_lqr[0][1] * omega_err[1]);
    u[1] = -(K_lqr[1][0] * omega_err[0] + K_lqr[1][1] * omega_err[1]);
    u[2] = -(K_lqr[2][0] * omega_err[0] + K_lqr[2][1] * omega_err[1]);

    // Normalize LQR outputs
    float umax = max(max(abs(u[0]), abs(u[1])), abs(u[2]));
    if (umax > U_MAX) {
        u[0] *= U_MAX / umax;
        u[1] *= U_MAX / umax;
        u[2] *= U_MAX / umax;
    }

    // ================= SMOOTH LQR OUTPUT =================
    for (int i = 0; i < 3; i++) {
        u_smooth[i] = beta * u[i] + (1 - beta) * u_smooth[i];
    }
    // ================= THRUSTER MIXING =================
    // Vertical thrusters: LQR only
    throttle.VB = clampPWM(u_smooth[0] * 150 + throttle.zoffset);
    throttle.VR = clampPWM(u_smooth[1] * 150 + throttle.zoffset);
    throttle.VL = clampPWM(u_smooth[2] * 150 + throttle.zoffset);

}
