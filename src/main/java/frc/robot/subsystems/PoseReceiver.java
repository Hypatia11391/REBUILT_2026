package frc.robot.subsystems;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.channels.ServerSocketChannel;
import java.nio.channels.SocketChannel;
import java.util.LinkedList;
import java.util.List;
import java.util.stream.Stream;

public class PoseReceiver {
    private final ServerSocketChannel channel;
    private final LinkedList<BufferedFixedLengthChannel> channels = new LinkedList<>();
    private static final int PORT = 11211;

    public PoseReceiver() {
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



    private Stream<PosePacket> pollForPose(BufferedFixedLengthChannel channel) throws IOException {
        return channel.pollFullBuffers().map(PosePacket::fromBuf);
    }

    public List<PosePacket> poll() {
        acceptNewConnections();

    }
}
