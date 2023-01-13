package org.firstinspires.ftc.teamcode.subsystems.v4b;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.StepperServo;

public class V4B {
    public StepperServo v4b1;
    public StepperServo v4b2;

    public double currentAngle;

    private MotionProfile profile;
    private ElapsedTime timer;
    double maxvel = 7;
    double maxaccel = 7;

    // TARGETS
    public double zeroTarget = 3;
    public double groundTarget = 3;
    public double lowTarget = 160;
    public double midTarget = 160;
    public double highTarget = 160;
    public double autoInit = 210;

    public V4B(StepperServo servo1, StepperServo servo2) {
        this.v4b1 = servo1;
        this.v4b2 = servo2;
        timer = new ElapsedTime();
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1, 0), new MotionState(0, 0), maxvel, maxaccel);
    }

    public void setAngle(double angle) {
//        this.v4b1.setAngle((float) angle);
//        this.v4b2.setAngle((float) angle);
        timer = new ElapsedTime();
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1, 0), new MotionState(0, 0), maxvel, maxaccel);
        this.currentAngle = angle;
    }

    public double getAngle() {
        return currentAngle;
    }

    public void runToPreset(Levels level) {
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
        }
    }

    public void update() {
        double angle = profile.get(timer.time()).getX();
        this.v4b1.setAngle((float) angle);
        this.v4b2.setAngle((float) angle);
    }
}