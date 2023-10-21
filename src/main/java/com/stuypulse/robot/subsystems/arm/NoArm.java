package com.stuypulse.robot.subsystems.arm;

public class NoArm extends Arm {

    @Override
    public void setVoltage(double voltage) {}

    @Override
    public void runMotor() {}

    @Override
    public void reverseMotor() {}

    @Override
    public void stop() {}

    @Override
    public double getAngle() {
        return 0;
    }

}
