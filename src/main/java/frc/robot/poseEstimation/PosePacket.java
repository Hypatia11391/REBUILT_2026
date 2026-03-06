package frc.robot.poseEstimation;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N4;

import java.nio.ByteBuffer;

/// As a buffer,
/// a pose packet consists of
///  - a 4x4 matrix written in left to right top to bottom order
///  - then the translational error as a double
///  - the rotational error in radians as a double
///  - the current time as a unix timestamp in milliseconds (milliseconds since 1970/01/01)
///
/// NOTE: time may not actually be calibrated on the RoboRIO??? I mean I don't know how it would sync???
/// If so, a shared other timestamp could be found and used
public record PosePacket(Pose3d pose, double translationErr, double rotationErr, long milliTimestamp) {
    //                   Rotation Quaternion| Position Vector       | Time of measurement
    public static int BYTES = Long.BYTES + 2 * Double.BYTES + 4 * 4 * Double.BYTES;

    public static PosePacket fromBuf(ByteBuffer buf) {
        ByteBuffer testBuf = buf.duplicate();
        while(testBuf.hasRemaining()) {
            System.out.print(testBuf.get());
        }
        System.out.println(buf);

        double[] matrixValues = new double[4 * 4];
        // reading order (left to right then top to bottom)
        for(int i=0;i<4;i++) {
            for(int j=0;j<4;j++) {
                matrixValues[j * 4 + i] = buf.getDouble();
            }
        }
        Matrix<N4,N4> matrix = new Matrix<>(Nat.N4(), Nat.N4(), matrixValues);

        long nanoTimestamp = buf.getLong();

        double translationErr = buf.getDouble();
        double rotationErr = buf.getDouble();

        return new PosePacket(new Pose3d(matrix),translationErr,rotationErr, nanoTimestamp / 1000);
    }

    public static PosePacket fromString(String str) {
        System.out.println(str);
        String[] strs = str.split(",");
        if(strs.length < 17) {
            throw new IllegalArgumentException("cannot convert string into pose");
        }
        double[] matrixValues = new double[4 * 4];
        long nanoTimestamp = -1;
        double err = -1;
        for(int i = 0;i<strs.length;i++) {
            if(i < 16) {
                matrixValues[i] = Double.parseDouble(strs[i]);
            }
            if(i == 16) {
                nanoTimestamp = Long.parseLong(strs[i]);
            }
            if(i == 17) {
                err = Double.parseDouble(strs[i]);
            }
        }
        Matrix<N4,N4> matrix = new Matrix<>(Nat.N4(), Nat.N4(), matrixValues);

        return new PosePacket(
            new Pose3d(matrix),
            err, err,
            nanoTimestamp / 1000
        );
    }
}
