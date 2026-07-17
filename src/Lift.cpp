#include "Lift.hpp"

int ROT_PORT = 16;

Motor LiftMotorA = Motor(-13, v5::MotorGears::red);
Motor LiftMotorB = Motor(1, v5::MotorGears::red);

bool Lift::useTarget = false;
double Lift::liftTarget = Lift::HOME;

// Initialize sensors & motors
MotorGroup Lift::motors({ -13, 1 }, v5::MotorGears::red);

Rotation Lift::liftRot(-16);
// TODO: Fix PID
// PID Lift::liftPID(0, 3, 0, 2, 0);
PID Lift::liftPID(3, 0, 2, 0, false);
Task Lift::liftThread = Task(Lift::threadEntry, nullptr);

void Lift::threadEntry(void* object) {
    Lift* self = static_cast<Lift*>(object);
    return self->liftUpdateLoop();
}

// void Lift::spin(
//     vex::directionType dir,
//     int pct,
//     vex::percentUnits units
// ) {
//     Lift::motors.spin(dir, pct, units);
// }
void Lift::spin(
    int voltage
) {
    Lift::motors.move(voltage);
}

void Lift::resetPosition() {
    Lift::liftRot.reset_position();
}

void Lift::stop() {
    Lift::motors.brake();
}

void Lift::liftUpdateLoop() {
    while (true) {
        double currentPos = liftRot.get_position() / 36000;
        double liftError = currentPos - liftTarget;
        double liftPower = -liftPID.update(liftError) * 100;

        if (useTarget) {
            if (liftPower < 5 && liftPower > -5) {
                Lift::stop();
            } else {
                motors.move(liftPower);
            }
        }

        delay(20);
    }
}

void Lift::setLiftTarget(double target) {
    useTarget = true;
    liftTarget = target;
}

void Lift::handleControllerInput(
    bool L2_pressing,
    bool L1_pressing
) {
    if (L2_pressing) {
        Lift::useTarget = false;
        motors.move(-108);
    } else if (L1_pressing) {
        Lift::useTarget = false;
        motors.move(83);
    } else {
        if (!useTarget) {
            Lift::stop();
        }
    }
}