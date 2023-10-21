package com.stuypulse.robot.util;

import java.util.function.Consumer;

import com.stuypulse.robot.subsystems.Conveyor;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.intake.Intake;

public enum ConveyorMode {
    // move intaked cube to upper position
    AUTONINDEXING(conveyor -> {
        if (conveyor.hasBall() && !conveyor.hasShooterBall()) {
            conveyor.runForward();
        } else {
            conveyor.stop();
        }
    }),

    // move intaked cube to lower position
    INDEXING(conveyor -> {
        if (conveyor.hasBall() && !conveyor.hasIntakeBall()) {
            conveyor.runReverse();
        } else {
            conveyor.stop();
        }
    }),

    // shoot or outtake towards grid
    SHOOTING(conveyor -> { 
        var angle = SwerveDrive.getInstance().getAngle().getDegrees();
        boolean facingGrid = angle > -90 && angle < 90;

        if (facingGrid) {
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