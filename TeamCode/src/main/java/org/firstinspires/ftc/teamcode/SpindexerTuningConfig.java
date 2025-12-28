package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class SpindexerTuningConfig {

    // ===== ABS encoder tuning =====
    public static double ABS_VREF = 3.3;

    // Set so that: absRaw == ABS_MECH_OFFSET_DEG  =>  internalAngle == 0° (slot0 at intake)
    public static double ABS_MECH_OFFSET_DEG = 245.0;

    // If encoder vs abs differ by more than this, we consider correction
    public static double ABS_REZERO_THRESHOLD_DEG = 3.0;

    // Require N consecutive "bad" readings before rezero (prevents single-sample noise triggers)
    public static int ABS_BAD_COUNT_REQUIRED = 3;

    // Don’t rezero more often than this (ms)
    public static long ABS_REZERO_COOLDOWN_MS = 800;

    // Only rezero when motor is basically stopped (ticks/sec)
    public static double ABS_REZERO_MAX_VEL_TPS = 15.0;

    // Optional filter (0..1). 0 = no update, 1 = no filtering
    public static double ABS_FILTER_ALPHA = 0.25;

    // Voltage sanity: ignore abs if out of range
    public static double ABS_MIN_VALID_V = 0.05;
    public static double ABS_MAX_VALID_V = 3.25;

    public static boolean ENABLE_ABS_SANITY = true;

    // ===== Motion tuning =====
    public static double MOVE_POWER = 0.5;
    public static int RUN_TO_TIMEOUT_MS = 700;

    // How close we must be to call a slot “aligned”
    public static double SLOT_ALIGN_TOL_DEG = 3.0;

    // Auto-rotate delay after intake detect
    public static long AUTO_ROTATE_DELAY_MS = 100;

    // ===== Sensors tuning =====
    public static double BALL_PRESENT_THRESH_CM = 5.0;
    public static int COLOR_SUM_MIN = 50;

    // ===== Debug / behavior =====
    public static boolean PRINT_ABS_REZERO_EVENTS = true;
}
