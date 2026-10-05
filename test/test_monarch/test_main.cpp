#include <stdio.h>
#include <unity.h>

#include "GyroCalibrator.h"
#include "sim.h"

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------------------
// Attitude
// ---------------------------------------------------------------------------

// World "up" as the body would feel it: nose pitched up by phi, then spun by
// psi about the chassis normal (same construction the simulator uses).
static Vec3 upFor(float phiDeg, float psiDeg) {
    M3 R = mul(Ry(-phiDeg * RAD), Rz(psiDeg * RAD));
    return mulv(transpose(R), Vec3{0, 0, 1});
}

void test_static_pose_outputs() {
    Attitude a;

    a.reset(upFor(33, 0));                       // nose up the ramp
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 33.0f, a.tiltDeg());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 33.0f, a.pitchDeg());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, a.uphillBearingDeg());

    a.reset(upFor(33, 90));                      // turned 90 deg CCW: uphill is now to the RIGHT
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 33.0f, a.tiltDeg());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -90.0f, a.uphillBearingDeg());

    a.reset(upFor(33, -90));                     // turned 90 deg CW: uphill is to the LEFT
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 90.0f, a.uphillBearingDeg());

    a.reset(upFor(33, 180));                     // facing straight down the ramp
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 180.0f, fabsf(a.uphillBearingDeg()));

    a.reset(upFor(-33, 0));                      // nose down: far side of the hill
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 33.0f, a.tiltDeg());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -33.0f, a.pitchDeg());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 180.0f, fabsf(a.uphillBearingDeg()));

    a.reset(Vec3{0, 0, 1});                      // flat
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, a.tiltDeg());
}

// Spin the body about +Y (nose DOWN by the right-hand rule) while the
// accelerometer is useless (2 g, gated out): the estimate must follow the gyro.
void test_gyro_carries_estimate_when_accel_is_gated_out() {
    Attitude a;
    a.reset(Vec3{0, 0, 1});
    float dt = 0.01f;
    for (int i = 0; i < 150; i++) {                 // 1.5 s at 20 deg/s = 30 deg nose-down
        a.update(Vec3{0, 0, 2.0f}, Vec3{0, 20.0f, 0}, dt);
    }
    TEST_ASSERT_FLOAT_WITHIN(1.0f, -30.0f, a.pitchDeg());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 30.0f, a.tiltDeg());
}

void test_single_bad_accel_sample_barely_moves_estimate() {
    Attitude a;
    a.reset(Vec3{0, 0, 1});
    float dt = 0.01f;
    for (int i = 0; i < 100; i++) a.update(Vec3{0, 0, 1}, Vec3{0, 0, 0}, dt);
    a.update(Vec3{1.4f, 0, 1.0f}, Vec3{0, 0, 0}, dt);    // violent shove
    TEST_ASSERT_LESS_THAN_FLOAT(2.0f, a.tiltDeg());
}

// The failure that accelerometer-only tilt had: motor surges look like tilt.
void test_motor_surges_do_not_look_like_a_slope() {
    Attitude a;
    a.reset(Vec3{0, 0, 1});
    float dt = 0.01f;
    float worst = 0;
    for (int burst = 0; burst < 6; burst++) {
        for (int i = 0; i < 12; i++) {               // 0.12 s at 0.3 g forward
            a.update(Vec3{0.3f, 0, 1.0f}, Vec3{0, 0, 0}, dt);
            if (a.tiltDeg() > worst) worst = a.tiltDeg();
        }
        for (int i = 0; i < 40; i++) {
            a.update(Vec3{0, 0, 1.0f}, Vec3{0, 0, 0}, dt);
            if (a.tiltDeg() > worst) worst = a.tiltDeg();
        }
    }
    // Raw accelerometer tilt here would be atan(0.3) = 16.7 deg.
    TEST_ASSERT_LESS_THAN_FLOAT(5.0f, worst);
}

// Yaw must follow rotation about the WORLD vertical, which on a 33 deg slope
// is not the body Z axis.
void test_yaw_integrates_world_vertical_turn_on_slope() {
    Sim s;
    s.gyroNoiseDps = 0;
    s.accelNoiseG = 0;
    s.vibrationG = 0;
    s.ell = 30;                                       // mid-ramp
    s.start();
    float truePsiStart = s.psi;
    for (int i = 0; i < 150; i++) {                   // turn at ~60 deg/s for 1.5 s
        s.psi += 60.0f * RAD * s.dt;
        M3 R = s.orientation();
        Vec3 up = mulv(transpose(R), Vec3{0, 0, 1});
        Vec3 g = bodyRate(s.Rprev, R, s.dt) * DEGS;
        s.Rprev = R;
        s.att.update(up, g, s.dt);
    }
    // Spinning about the chassis normal on a 33 deg slope only turns the
    // heading by cos(33 deg) of the spin: that is the world-vertical part.
    float turned = (s.psi - truePsiStart) * DEGS;
    float expected = turned * cosf(s.pitchAt(s.ell));
    TEST_ASSERT_FLOAT_WITHIN(1.5f, expected, s.att.yawDeg());
}

// ---------------------------------------------------------------------------
// Gyro calibrator
// ---------------------------------------------------------------------------

void test_calibrator_finds_bias_and_accepts_calm_data() {
    GyroCalibrator c;
    Noise n;
    for (int i = 0; i < 400; i++) {
        c.add(Vec3{1.5f + n.gauss() * 0.3f, -0.8f + n.gauss() * 0.3f, 0.4f + n.gauss() * 0.3f});
    }
    TEST_ASSERT_TRUE(c.still(200, 1.5f));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1.5f, c.bias().x);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -0.8f, c.bias().y);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.4f, c.bias().z);
}

void test_calibrator_rejects_data_taken_while_handled() {
    GyroCalibrator c;
    for (int i = 0; i < 400; i++) {
        float swing = (i % 2 == 0) ? 40.0f : -40.0f;     // being waved around
        c.add(Vec3{swing, 0, 0});
    }
    TEST_ASSERT_FALSE(c.still(200, 1.5f));
}

// ---------------------------------------------------------------------------
// Strategy, run against the simulated hill
// ---------------------------------------------------------------------------

// Speed the sim robot actually moves at centerSpeed on flat ground.
static float centerSpeedInPerS(const Sim &s, const StrategyConfig &c) {
    return c.centerSpeed / 255.0f * s.vmaxInPerS;
}

static StrategyConfig calibratedConfig(const Sim &probe) {
    StrategyConfig c;
    c.measuredSpeedInPerS = centerSpeedInPerS(probe, c);
    return c;
}

static bool onMesa(const Sim &s) {
    return s.ell > s.hill.mesaStart() + 1.0f && s.ell < s.hill.mesaEnd() - 1.0f;
}

void test_waiting_robot_does_not_move_on_flat_ground() {
    Sim s;
    s.ell = -10;                     // on the flat approach
    s.start();
    s.run(3.0f);
    TEST_ASSERT_EQUAL(Phase::Waiting, s.strat.phase());
    TEST_ASSERT_EQUAL(0, s.cmd.left);
    TEST_ASSERT_EQUAL(0, s.cmd.right);
}

void test_climbs_and_stops_near_mesa_center() {
    Sim probe;
    StrategyConfig c = calibratedConfig(probe);
    // The chassis only reads level once both ends are on the mesa, so the CG is
    // already ~2 in past the edge by the time dead reckoning starts.
    c.centerOffsetIn = -2.0f;
    Sim s(c);
    s.gyroBiasDps = 0.2f;
    s.start();
    TEST_ASSERT_TRUE(s.runUntil(Phase::Holding, 25.0f));
    s.run(1.0f);
    printf("   climb: stopped at %.1f in (mesa center %.1f)\n", s.ell, s.hill.mesaCenter());
    TEST_ASSERT_EQUAL(Phase::Holding, s.strat.phase());
    TEST_ASSERT_FLOAT_WITHIN(3.0f, s.hill.mesaCenter(), s.ell);
    TEST_ASSERT_EQUAL(0, s.cmd.left);
}

// Speed estimate 30% off in either direction: it may land off-center but must
// stay on the mesa.
void test_wrong_speed_estimate_still_stays_on_mesa() {
    float errors[2] = {0.7f, 1.3f};
    for (int k = 0; k < 2; k++) {
        Sim probe;
        StrategyConfig c = calibratedConfig(probe);
        c.centerOffsetIn = -2.0f;
        c.measuredSpeedInPerS *= errors[k];
        Sim s(c);
        s.start();
        TEST_ASSERT_TRUE(s.runUntil(Phase::Holding, 25.0f));
        s.run(2.0f);
        printf("   speed x%.1f: stopped at %.1f in\n", errors[k], s.ell);
        TEST_ASSERT_TRUE(onMesa(s));
    }
}

// The scenario from the brief: it goes too far, the front drops over the far
// edge, and it has to notice and back up.
void test_overshoot_backs_up_onto_mesa() {
    Sim probe;
    StrategyConfig c = calibratedConfig(probe);
    c.centerOffsetIn = 7.0f;          // deliberately drive way too long
    Sim s(c);
    s.start();
    s.run(18.0f);
    printf("   overshoot: max tilt seen %.1f deg, ended at %.1f in, phase %s\n",
           s.maxTiltSeen, s.ell, s.strat.phaseName());
    TEST_ASSERT_TRUE(s.ell > s.hill.mesaEnd() - 1.0f || s.maxTiltSeen > 15.0f);  // it really did go over
    TEST_ASSERT_TRUE(onMesa(s));
    TEST_ASSERT_EQUAL(Phase::Holding, s.strat.phase());
}

void test_turns_toward_uphill_when_started_sideways() {
    float starts[3] = {90.0f, -90.0f, 135.0f};
    for (int k = 0; k < 3; k++) {
        Sim probe;
        StrategyConfig c = calibratedConfig(probe);
        c.centerOffsetIn = -2.0f;
        Sim s(c);
        s.psi = starts[k] * RAD;
        s.start();
        TEST_ASSERT_TRUE(s.runUntil(Phase::Holding, 30.0f));
        s.run(1.0f);
        printf("   start heading %+.0f: stopped at %.1f in, heading %.0f deg\n",
               starts[k], s.ell, s.psi * DEGS);
        TEST_ASSERT_TRUE(onMesa(s));
    }
}

// Facing straight down the ramp: it should back up the hill rather than
// spin around.
void test_backs_up_the_ramp_when_facing_downhill() {
    Sim probe;
    StrategyConfig c = calibratedConfig(probe);
    c.centerOffsetIn = -2.0f;
    Sim s(c);
    s.psi = 180.0f * RAD;
    s.start();
    bool sawReverse = false;
    int n = (int) (30.0f / s.dt);
    for (int i = 0; i < n; i++) {
        s.step();
        if (s.strat.reversing()) sawReverse = true;
        if (s.strat.phase() == Phase::Holding) break;
    }
    s.run(1.0f);
    printf("   facing downhill: reversed=%d, stopped at %.1f in, heading %.0f deg\n",
           sawReverse, s.ell, s.psi * DEGS);
    TEST_ASSERT_TRUE(sawReverse);
    TEST_ASSERT_TRUE(onMesa(s));
    TEST_ASSERT_FLOAT_WITHIN(25.0f, 180.0f, fabsf(s.psi * DEGS));   // did not spin around
}

void test_returns_to_mesa_after_being_pushed_off() {
    Sim probe;
    StrategyConfig c = calibratedConfig(probe);
    c.centerOffsetIn = -2.0f;
    Sim s(c);
    s.start();
    TEST_ASSERT_TRUE(s.runUntil(Phase::Holding, 25.0f));
    s.run(1.0f);

    s.ell = 28.0f;                    // shoved back down the near ramp...
    s.psi += 100.0f * RAD;            // ...and spun sideways on the way
    TEST_ASSERT_TRUE(s.runUntil(Phase::Slope, 3.0f));
    TEST_ASSERT_TRUE(s.runUntil(Phase::Holding, 30.0f));
    s.run(1.0f);
    printf("   pushed off: back at %.1f in\n", s.ell);
    TEST_ASSERT_TRUE(onMesa(s));
}

// A turn while holding the mesa gets corrected, a small nudge does not.
void test_holding_reaims_only_after_a_big_turn() {
    Sim probe;
    StrategyConfig c = calibratedConfig(probe);
    c.centerOffsetIn = -2.0f;
    Sim s(c);
    s.start();
    TEST_ASSERT_TRUE(s.runUntil(Phase::Holding, 25.0f));
    s.run(1.0f);

    float h0 = s.psi;
    s.psi += 10.0f * RAD;                         // small nudge
    s.run(1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, (h0 + 10.0f * RAD) * DEGS, s.psi * DEGS);

    s.psi += 50.0f * RAD;                         // big turn
    s.run(4.0f);
    printf("   re-aim: heading error now %.1f deg\n", (s.psi - h0) * DEGS);
    TEST_ASSERT_FLOAT_WITHIN(12.0f, 0.0f, (s.psi - h0) * DEGS);
}

// With the nudge off, the robot stops as soon as it reads level: it never
// enters Centering, and it ends up on the mesa near the near edge.
void test_stops_at_first_level_reading_when_nudge_is_off() {
    StrategyConfig c;
    c.centerNudge = false;
    Sim s(c);
    s.start();
    bool sawCentering = false;
    int n = (int) (25.0f / s.dt);
    for (int i = 0; i < n; i++) {
        s.step();
        if (s.strat.phase() == Phase::Centering) sawCentering = true;
        if (s.strat.phase() == Phase::Holding) break;
    }
    s.run(1.0f);
    printf("   nudge off: stopped at %.1f in (mesa %.0f to %.0f)\n",
           s.ell, s.hill.mesaStart(), s.hill.mesaEnd());
    TEST_ASSERT_FALSE(sawCentering);
    TEST_ASSERT_EQUAL(Phase::Holding, s.strat.phase());
    TEST_ASSERT_TRUE(onMesa(s));
    TEST_ASSERT_LESS_THAN_FLOAT(s.hill.mesaCenter(), s.ell);
    TEST_ASSERT_EQUAL(0, s.cmd.left);
    TEST_ASSERT_EQUAL(0, s.cmd.right);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_static_pose_outputs);
    RUN_TEST(test_gyro_carries_estimate_when_accel_is_gated_out);
    RUN_TEST(test_single_bad_accel_sample_barely_moves_estimate);
    RUN_TEST(test_motor_surges_do_not_look_like_a_slope);
    RUN_TEST(test_yaw_integrates_world_vertical_turn_on_slope);
    RUN_TEST(test_calibrator_finds_bias_and_accepts_calm_data);
    RUN_TEST(test_calibrator_rejects_data_taken_while_handled);
    RUN_TEST(test_waiting_robot_does_not_move_on_flat_ground);
    RUN_TEST(test_stops_at_first_level_reading_when_nudge_is_off);
    RUN_TEST(test_climbs_and_stops_near_mesa_center);
    RUN_TEST(test_wrong_speed_estimate_still_stays_on_mesa);
    RUN_TEST(test_overshoot_backs_up_onto_mesa);
    RUN_TEST(test_turns_toward_uphill_when_started_sideways);
    RUN_TEST(test_backs_up_the_ramp_when_facing_downhill);
    RUN_TEST(test_returns_to_mesa_after_being_pushed_off);
    RUN_TEST(test_holding_reaims_only_after_a_big_turn);
    return UNITY_END();
}
