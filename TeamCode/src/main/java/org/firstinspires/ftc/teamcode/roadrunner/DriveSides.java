package org.firstinspires.ftc.teamcode.roadrunner;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.Arrays;
import java.util.List;

/**
 * The drive motors grouped by the WHEEL they turn, for code that writes wheel powers itself
 * instead of going through {@link MecanumDrive#setWheelPowers} - Road Runner's tuning opmodes.
 *
 * With {@link MecanumDrive#DRIVE_SIDES_SWAPPED} the frontLeft/backLeft ports drive the robot's
 * RIGHT wheels and vice versa (see the note on that flag). So the robot's left wheels are the
 * frontRight/backRight ports and its right wheels the frontLeft/backLeft ports, each with its power
 * NEGATED - exactly the mapping setWheelPowers uses, so tuning measures the same thing the autos
 * command. The ports' directions are not touched: the dead wheels share frontLeft/frontRight and
 * RawEncoder's sign follows the motor direction.
 */
public final class DriveSides {

    private DriveSides() {
    }

    /** Robot's left wheels, front then back. */
    public static List<DcMotorEx> left(MecanumDrive md) {
        return MecanumDrive.DRIVE_SIDES_SWAPPED
                ? Arrays.asList(negated(md.frontRight), negated(md.backRight))
                : Arrays.asList(md.frontLeft, md.backLeft);
    }

    /** Robot's right wheels, front then back. */
    public static List<DcMotorEx> right(MecanumDrive md) {
        return MecanumDrive.DRIVE_SIDES_SWAPPED
                ? Arrays.asList(negated(md.frontLeft), negated(md.backLeft))
                : Arrays.asList(md.frontRight, md.backRight);
    }

    /**
     * The same motor with setPower / getPower negated and everything else passed straight through
     * (direction, mode, zero-power behaviour, encoder reads, current, ...).
     */
    public static DcMotorEx negated(DcMotorEx motor) {
        InvocationHandler h = (proxy, method, args) -> {
            String name = method.getName();
            if (name.equals("setPower") && args != null && args.length == 1) {
                return invoke(motor, method, new Object[]{-(Double) args[0]});
            }
            if (name.equals("getPower") && (args == null || args.length == 0)) {
                return -(Double) invoke(motor, method, null);
            }
            if (name.equals("equals") && args != null && args.length == 1) {
                return proxy == args[0];
            }
            if (name.equals("hashCode") && (args == null || args.length == 0)) {
                return System.identityHashCode(proxy);
            }
            if (name.equals("toString") && (args == null || args.length == 0)) {
                return "negated(" + motor + ")";
            }
            return invoke(motor, method, args);
        };
        return (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(), new Class<?>[]{DcMotorEx.class}, h);
    }

    private static Object invoke(Object target, Method method, Object[] args) throws Throwable {
        try {
            return method.invoke(target, args);
        } catch (InvocationTargetException e) {
            throw e.getCause();
        }
    }
}
