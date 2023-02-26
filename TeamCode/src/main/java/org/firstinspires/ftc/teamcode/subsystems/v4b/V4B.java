package org.firstinspires.ftc.teamcode.subsystems.v4b;

import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.StepperServo;

public class V4B {
    public StepperServo v4b1;
    public StepperServo v4b2;

    public double currentAngle;

    // TARGETS
    public double zeroTarget = 95;
    public double groundTarget = 95;
    public double lowTarget = 198;
    public double midTarget = 204;
    public double highTarget = 205; //145 auton
    public double autoHigh = 200;
    public double autoInit = 160;
    public double autoDeposit = 207;

    public V4B(StepperServo servo1, StepperServo servo2) {
        this.v4b1 = servo1;
        this.v4b2 = servo2;
//        v4b1.servo.setDirection(Servo.Direction.REVERSE);
    }

    public void setAngle(double angle) {
        this.v4b1.setAngle((float) angle);
        this.v4b2.setAngle((float) angle);
        this.currentAngle = angle;
    }

    public void setZeroServo() {
        this.v4b1.setAngle(0);
        this.v4b2.setAngle(0);
    }

    public double getAngle() {
        return currentAngle;
    }

    public void runToPreset(Levels level) {
//        switch (level) {
//            case ZERO:
//                this.setAngle(zeroTarget);
//            case GROUND:
//                this.setAngle(groundTarget);
//            case LOW:
//                this.setAngle(lowTarget);
//            case MEDIUM:
//                this.setAngle(midTarget);
//            case HIGH:
//                this.setAngle(highTarget);
//        }
        if (level == Levels.ZERO) {
            this.setAngle(zeroTarget);
        } else if (level == Levels.GROUND) {
            this.setAngle(groundTarget);
        } else if (level == Levels.LOW) {
            this.setAngle(lowTarget);
        } else if (level == Levels.MEDIUM) {
            this.setAngle(midTarget);
        } else if (level == Levels.HIGH) {
            this.setAngle(highTarget);
        } else if (level == Levels.AUTOINIT) {
            this.setAngle(autoInit);
        } else if (level == Levels.AUTOHIGH) {
            this.setAngle(autoHigh);
        } else if (level == Levels.AUTODEPOSIT) {
            this.setAngle(autoDeposit);
        }
    }
}