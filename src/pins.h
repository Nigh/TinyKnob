#ifndef _PINS_H_
#define _PINS_H_

// Waveshare RP2350-Zero — TinyKnob

#define PIN_PWM_A 2
#define PIN_PWM_B 4
#define PIN_PWM_C 6
#define PIN_DRV_EN 8

#define PIN_MT6701_CSN 9
#define PIN_MT6701_CLK 10
#define PIN_MT6701_DO 11

// ADC: CSA/CSB/CSC + 0.1×VCC (DRV8316 SOx / bus divider)
#define PIN_CSA 26
#define PIN_CSB 27
#define PIN_CSC 28
#define PIN_VBUS 29

#define MOTOR_POLE_PAIRS 11

#define PWM_HZ 20000u
#define ALIGN_UD 0.75f
#define ALIGN_RAMP_MS 200u
#define ALIGN_DRAG_MS 2500u
#define ALIGN_DRAG_ELEC_REVS 2.f
#define ALIGN_DRAG_MIN_COUNTS 400
#define ALIGN_HOLD_MS 500u
#define DIR_PULSE_MS 600u
#define DIR_PULSE_UQ 0.18f
#define DIR_PULSE_COUNTS 80
#define PWM_MIN_MAG 0.012f
#define POS_KP 0.6f
#define POS_D 0.05f
#define TEST_MOVE_MS 1400u
#define TEST_HOLD_MS 200u
#define TEST_UQ_MAX 0.65f
// ponytail: open-loop Kv (V/(rad/s)); retune if TEST lags (Uq_ff ≈ lag_rad * POS_KP).
#define TEST_KV 0.07f
// ponytail: K matches POS_KP (known to move). Uq max > DIR_PULSE (cogging), < TEST.
// 20 kHz raw Δpos is 1ct≈7.7 rad/s — D must see LPF'd vel. Ceiling: lag on fast flicks; upgrade = encoder PLL.
#define SPRING_K 0.6f
#define SPRING_D 0.0f
#define SPRING_D_UQ_MAX 0.12f /* damping may slow return but must not reverse saturated spring effort */
#define SPRING_SETTLE_MS 2000u /* allow damped rotor to reach its true cog equilibrium */
#define SPRING_NEUTRAL_RAD 0.00262f /* +/-0.15 deg soft deadband after equilibrium capture */
#define SPRING_UQ_MAX 0.35f /* soft effort envelope inside blend */
#define SPRING_WALL_UQ 0.85f /* effort envelope past ±π blend (same sign as K·err) */
#define SPRING_WALL_BLEND_RAD 0.45f /* soft↔wall envelope over ±π (±~26°) */
#define SPRING_WALL_D 0.15f /* damping at full wall; lerped in blend */
#define SPRING_VEL_LP_HZ 50.f
// PWM-timed low-side ADC burst + DMA. TEST/POS/SPIN use PI when CTRL=1.
#define CUR_LOOP_EN 1
#ifndef CUR_LOOP_CTRL
#define CUR_LOOP_CTRL 1
#endif
#define CUR_LOOP_DIV 2u /* 10 kHz synchronized sample/PI vs 20 kHz PWM */
#define CUR_SAMPLE_STALE_TICKS 6u /* coast/reset PI after 3 missed current-loop slots */
#define CS_SAMPLE_BLANK_US 2u /* DRV edge/CSA settling after all three low sides turn on */
#define CS_SAMPLE_MARGIN_US 2u /* require this much common-low time after the ADC burst */
// SPIN: low-damping voltage FF normally; current-controlled overspeed brake.
#define SPIN_KV 0.018f /* coast vs creep; with B>0 net FF = KV−B */
#define SPIN_B 0.000075f /* low-speed damping: light enough to preserve flick inertia */
#define SPIN_B_HIGH 0.0015f /* smoothly reached at W_MAX so faster spins settle */
#define SPIN_W_REST 0.35f /* soft FF ramp starts here; wider → less detent hunting */
#define SPIN_W_MAX 22.f
#define SPIN_B_CAP 0.30f
#define SPIN_IQ_CAP 0.20f /* opposing Iq (A) when |ω|>SPIN_W_MAX under current loop */
#define SPIN_UQ_MAX 0.50f
#define SPIN_VEL_LP_HZ 120.f
// GEAR: match the dominant 24th mechanical cog harmonic so every virtual
// valley sees the same physical cog phase instead of alternating at 48 teeth.
#define GEAR_TEETH 24u
#define GEAR_UQ 0.30f
#define GEAR_PEAK_FRAC 0.86f
#define GEAR_CLICK_UQ 0.08f
#define GEAR_CLICK_US 50u
#define GEAR_CAPTURE_MIN_MS 250u
#define GEAR_CAPTURE_STABLE_MS 120u
#define GEAR_CAPTURE_TIMEOUT_MS 1200u
#define GEAR_CAPTURE_W_MAX 0.10f
#define GEAR_D 0.0008f
#define GEAR_D_UQ_MAX 0.03f
#define GEAR_VEL_LP_HZ 80.f
#define GEAR_COG_FF_SCALE 1.00f
// Stress: +full 3s / coast 1s / -full 3s / coast 1s; Uq smoothstep on start & stop.
#define STRESS_UQ_MAX 0.65f
#define STRESS_RUN_MS 3000u
#define STRESS_STOP_MS 1000u
#define STRESS_RAMP_MS 500u
#define CRC_FAIL_TRIP 200u

// DRV8316 CSA: SOX = VREF/2 ± GAIN·I. GAIN pin: AGND=0.15, Hi-Z=0.3, 47k→AVDD=0.6, AVDD=1.2.
#define CS_VREF 3.3f
#define CS_GAIN_V_PER_A 0.3f
#define CS_VBUS_DIV 0.1f
// Sense tuning (sense-only or closed loop): flip sign / permute ABC / shift Park angle if |Id|≫|Iq|.
#define CS_SIGN (-1.f)
#define CS_PHASE_ORD 4 /* 0=ABC 1=ACB 2=BAC 3=BCA 4=CAB 5=CBA */
// Must be 0 while using foc_sense_check sign(Iq)↔Uq; ≠0 parks currents off the voltage axis.
#define CS_TE_OFF (-0.70f) /* validated with synchronized foc_sense_check; keep 0 while sweeping ORD */
#define CS_IQ_LP_HZ 150.f
// Outer uq_cmd ∈ [-1,1] → Iq_ref (A). Retune with SPRING_K / feel.
#define IQ_CMD_A 1.0f /* TEST/POS normalized command to amperes */
#define SPRING_IQ_CMD_A 0.75f
#define STRESS_IQ_CMD_A 0.20f
#define I_TRIP_A 4.0f
// ponytail: PI at CUR_LOOP_DIV rate. Soft on 4015; raise after SOX RC + GAIN match.
#define CUR_KP 0.25f
#define CUR_KI 400.f
#define CUR_KAW 1000.f
#define CUR_U_MAX 0.60f /* guarantees room for blank + two-channel ADC burst */

// Cogging FF: flash default (cog_lut_default.h) + RAM override after COGCAL / host-learn.
#define COG_LUT_N 1024u /* 0.352 deg/bin, 16 encoder counts/bin */
#define COG_CAL_MS 8000u
#define SPIN_COG_FF_SCALE 0.650f
#define SPRING_COG_FF_SCALE 0.100f

#endif
