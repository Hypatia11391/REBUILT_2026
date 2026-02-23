package frc.robot.poseEstimation;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.channels.ServerSocketChannel;
import java.nio.channels.SocketChannel;
import java.time.Duration;
import java.time.Instant;
import java.util.LinkedList;
import java.util.stream.Stream;

public class VisionPoseManager {
    private final ServerSocketChannel channel;
    private final LinkedList<BufferedFixedLengthChannel> channels = new LinkedList<>();
    private static final int PORT = 11211;

    private WeightedVec3 weighted = null;
    private Instant lastUpdate = null;

    // in milliseconds the time it takes for positions to
    // halve their weight in the pose estimation
    private static final double DATA_HALF_LIFE = 10;

    private static final double DATA_DECAY = Math.log(2)/DATA_HALF_LIFE;

    private final OdometryManager odometryManager;

    public VisionPoseManager(OdometryManager odometryManager) {
        this.odometryManager = odometryManager;
        try {
            this.channel = ServerSocketChannel.open();
            this.channel.configureBlocking(false);
            this.channel.bind(new InetSocketAddress(PORT));

        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private void acceptNewConnections() {
        SocketChannel channel;
        try {
            while ((channel = this.channel.accept()) != null) {
                channel.configureBlocking(false);
                this.channels.add(
                    new BufferedFixedLengthChannel(
                        channel,
                        PosePacket.BYTES
                    )
                );
            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }



    private Stream<PosePacket> pollForPose(BufferedFixedLengthChannel channel) {
        return channel.pollFullBuffers().map(PosePacket::fromBuf);
    }

    public Stream<PosePacket> poll() {
        acceptNewConnections();
        Stream<PosePacket> packetStream = Stream.empty();
        for(BufferedFixedLengthChannel channel : channels) {
            packetStream = Stream.concat(packetStream,pollForPose(channel));
        }
        return packetStream;
    }

    public Vector<N3> getPos() {
        return weighted.vec().div(weighted.weight);
    }

    public void update() {
        poll()
            .sorted(PosePacket.BY_TIME)
            .forEach((packet) -> {
                if(weighted == null || lastUpdate == null) {
                    weighted = new WeightedVec3(packet.getPos(),1);
                    lastUpdate = packet.time();
                    return;
                }
                double weightMult = Math.exp(-DATA_DECAY * Duration.between(lastUpdate,packet.time()).toMillis());

                Vector<N3> newVec = weighted.vec().times(weightMult);
                newVec = newVec.plus(packet.getPos());

                double newWeight = weighted.weight() * weightMult;
                newWeight += 1;

                weighted = new WeightedVec3(newVec,newWeight);
                lastUpdate = packet.time();
            })
        ;
        odometryManager.updateFromVision(this.getPos());
    }

    public record WeightedVec3(Vector<N3> vec, double weight) {}
}
