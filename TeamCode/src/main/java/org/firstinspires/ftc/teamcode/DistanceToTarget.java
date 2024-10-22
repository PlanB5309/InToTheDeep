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

    public boolean closeEnough(DistanceToTarget dtt, Target.Destination type) {
        if (type == Target.Destination.DESTINATION)
            return (Math.abs(dtt.diffx) < .5 && Math.abs(dtt.diffy) < .5 && Math.abs(dtt.diffh) < .5);

        return (Math.abs(dtt.diffx) < 2 && Math.abs(dtt.diffy) < 2 && Math.abs(dtt.diffh) < 2);
    }
}

