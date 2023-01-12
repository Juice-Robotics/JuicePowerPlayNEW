package org.firstinspires.ftc.teamcode.subsystems.v4b;

import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.StepperServo;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class V4B {
    public StepperServo v4b1;
    public StepperServo v4b2;

    private MotionProfile profile;
    public MotionState curState;
    private ElapsedTime timer;
    double maxvel = 50;
    double maxaccel = 50;

    public double currentAngle;

    // TARGETS
    public double zeroTarget = 3;
    public double groundTarget = 3;
    public double lowTarget = 135;
    public double midTarget = 122;
    public double highTarget = 160;
    public double autoInit = 210;

    public V4B(StepperServo servo1, StepperServo servo2) {
        this.v4b1 = servo1;
        this.v4b2 = servo2;
        timer = new ElapsedTime();
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1, 0), new MotionState(0, 0), maxvel, maxaccel);
//        v4b1.servo.setDirection(Servo.Direction.REVERSE);
    }

    public void setAngle(double angle) {
        this.v4b1.setAngle((float) angle);
        this.v4b2.setAngle((float) angle);
        this.currentAngle = angle;
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
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(v4b1.getAngle(), 0), new MotionState(zeroTarget, 0), maxvel, maxaccel);
            timer.reset();
        } else if (level == Levels.GROUND) {
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(v4b1.getAngle(), 0), new MotionState(groundTarget, 0), maxvel, maxaccel);
            timer.reset();
        } else if (level == Levels.LOW) {
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(v4b1.getAngle(), 0), new MotionState(lowTarget, 0), maxvel, maxaccel);
            timer.reset();
        } else if (level == Levels.MEDIUM) {
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(v4b1.getAngle(), 0), new MotionState(midTarget, 0), maxvel, maxaccel);
            timer.reset();
        } else if (level == Levels.HIGH) {
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(v4b1.getAngle(), 0), new MotionState(highTarget, 0), maxvel, maxaccel);
            timer.reset();
        } else if (level == Levels.AUTOINIT) {
            this.setAngle(autoInit);
        }
    }
}