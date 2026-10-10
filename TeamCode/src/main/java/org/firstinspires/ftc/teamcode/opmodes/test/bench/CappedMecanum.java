package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.revhub.drivetrains.Mecanum;

import java.util.Map;

final class CappedMecanum implements Drivetrain {
    private final Mecanum mecanum;
    private final double cap;

    CappedMecanum(Mecanum mecanum, double cap) {
        this.mecanum = mecanum;
        this.cap = cap;
    }

    @Override
    public void drive(DrivePowers powers, boolean manual) {
        double max = 1;
        for (double wheel : mecanum.computeWheelPowersUnnormalized(powers)) max = Math.max(max, Math.abs(wheel));
        double scale = cap / max;
        mecanum.drive(new DrivePowers(powers.forward() * scale, powers.strafe() * scale, powers.turn() * scale), manual);
    }

    @Override
    public double maxScaling(DrivePowers current, DrivePowers delta) {
        return mecanum.maxScaling(current, delta);
    }

    @Override
    public void stop() {
        mecanum.stop();
    }

    @Override
    public void stop(boolean brake) {
        mecanum.stop(brake);
    }

    @Override
    public Map<String, Object> debug() {
        return mecanum.debug();
    }

    @Override
    public double interpolateVelocity(double xRadius, double yRadius, double theta) {
        return mecanum.interpolateVelocity(xRadius, yRadius, theta);
    }
}
