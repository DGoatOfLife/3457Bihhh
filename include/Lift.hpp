#pragma once

#include "api.h"
#include "lemlib/api.hpp"

using namespace pros;
using namespace lemlib;

class Lift {
    private:
        // Static function used to invoke lift thread.
        static void threadEntry(void* object);

    public:
        ////// PRESET CONSTANTS
        static constexpr double HOME = 0.0;
        // Intake Ready Pos
        static constexpr double A = 0.18;
        // Non-Middle goals.
        static constexpr double B = 0.24;
        static constexpr double C = 1.01;
        static constexpr double D = 0.62;
        // Middle goal.
        static constexpr double E = 0.39;
        static constexpr double F = 0.77;
        // Matchloading pos
        static constexpr double G = 0.1;
        // Initial preset to release passive toggle.
        static constexpr double Auton = 0.36;

        ////// stuff
        static MotorGroup motors;
        static Rotation liftRot;
        static PID liftPID;
        static Task liftThread;

        ////// STATE VARIABLES
        static bool useTarget;
        static double liftTarget;

        // // Basic control functions.
        // static void spin(
        //     vex::directionType dir,
        //     int pct,
        //     vex::percentUnits units
        // );
        static void spin(
            int voltage
        );

        // static void stop(
        //     vex::brakeType mode
        // );
        static void stop();

        static void resetPosition();

        static void liftUpdateLoop();

        static void setLiftTarget(double target);

        static void handleControllerInput(
            bool L2_pressing,
            bool L1_pressing
        );
};