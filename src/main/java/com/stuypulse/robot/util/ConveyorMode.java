package com.stuypulse.robot.util;

import java.util.function.Consumer;

import com.stuypulse.robot.subsystems.Conveyor;
import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.robot.subsystems.intake.Intake;

public enum ConveyorMode {
    // move cube to shooter/intake location to face grid unless holding two cubes
    INDEXING(conveyor -> {
        var angle = SwerveDrive.getInstance().getAngle().getDegrees();
        var isShooting = angle > -90 && angle < 90;

        if (!conveyor.hasShooterBall() && isShooting) {
            conveyor.runForward();
        } else if (!conveyor.hasIntakeBall() && !isShooting) {
            conveyor.runReverse();
        } else {
            conveyor.stop();
        }
    }),

    // shoot or outtake towards grid
    SHOOTING(conveyor -> { 
        var angle = SwerveDrive.getInstance().getAngle().getDegrees();

        if (angle > -90 && angle < 90) {
            conveyor.runForward();
        } else {
            conveyor.runReverse();
            Intake.getInstance().deacquire();
        }
    }),

    REVERSE(conveyor -> {
        conveyor.runReverse();
    }),
    
    STOP(conveyor -> {
        conveyor.stop();
    }),
    
    DEFAULT(INDEXING.action);
    
    private Consumer<Conveyor> action;

    private ConveyorMode(Consumer<Conveyor> action) {
        this.action = action;
    }

    public void run(Conveyor conveyor) {
        action.accept(conveyor);
    }
}