package org.firstinspires.ftc.teamcode.subsystems.v4b;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.StepperServo;

public class V4B {
    public StepperServo v4b1;
    public StepperServo v4b2;

    public double currentAngle;
    public boolean threadState;

    private MotionProfile profile;
    private ElapsedTime timer;
    double maxvel = 600;
    double maxaccel = 600;

    // TARGETS
    public double zeroTarget = 2;
    public double groundTarget = 2;
    public double lowTarget = 160;
    public double midTarget = 160;
    public double highTarget = 160;
    public double autoInit = 210;

    public V4B(StepperServo servo1, StepperServo servo2) {
        this.v4b1 = servo1;
        this.v4b2 = servo2;
        timer = new ElapsedTime();
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(this.v4b1.servo.getPosition()*255, 0), new MotionState(this.v4b1.servo.getPosition()*255, 0), maxvel, maxaccel);
    }

    public void setAngle(double angle) {
//        this.v4b1.setAngle((float) angle);
//        this.v4b2.setAngle((float) angle);
        timer = new ElapsedTime();
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState((this.v4b1.servo.getPosition()*255), 0), new MotionState(angle, 0), maxvel, maxaccel);
        timer.reset();
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

    public void launchAsThread(Telemetry telemetry) {
        threadState = true;
        telemetry.addData("V4B Threads Status:", "STARTING");
        telemetry.update();
        Thread t1 = new Thread(() -> {
            telemetry.addData("V4B Threads Status:", "STARTED");
            telemetry.update();
            while (threadState == true) {
                update();
            }
            telemetry.addData("V4B Threads Status:", "STOPPED");
            telemetry.update();
        });
        t1.start();
    }

    public void destroyThreads(Telemetry telemetry) {
        telemetry.addData("V4B Threads Status:", "STOPPING");
        telemetry.update();
        threadState = false;
    }
}