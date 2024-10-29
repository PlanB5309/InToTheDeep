package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class DistanceToTarget {
    double diffx;
    double diffy;
    double diffh;
    double vector;

    public DistanceToTarget(double diffx, double diffy, double diffh, double vector) {
        this.diffx = diffx;
        this.diffy = diffy;
        this.diffh = diffh;
        this.vector = vector;
    }

    public DistanceToTarget() {
        diffx = 0;
        diffy = 0;
        diffh = 0;
        vector = 0;
    }

    public DistanceToTarget find(Pose2D pos, Target target) {
        diffx = target.x - pos.getX(DistanceUnit.INCH);
        diffy = target.y - pos.getY(DistanceUnit.INCH);
        diffh = -(target.h - pos.getHeading(AngleUnit.DEGREES));
        vector = Math.sqrt(diffx * diffx + diffy * diffy);
        return this;
    }

    public boolean closeEnough(DistanceToTarget dtt, Target target) {
        return (Math.abs(dtt.vector) <= target.tp.maxDistance && Math.abs(dtt.diffh) <= target.tp.maxAngle);
    }
}

