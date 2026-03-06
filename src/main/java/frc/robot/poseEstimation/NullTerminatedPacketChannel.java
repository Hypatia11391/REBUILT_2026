package frc.robot.poseEstimation;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.SocketChannel;
import java.nio.charset.StandardCharsets;
import java.util.Objects;
import java.util.stream.Stream;

public class NullTerminatedPacketChannel {
    private ByteBuffer incompleteBuffer;
    private final SocketChannel channel;


    public NullTerminatedPacketChannel(SocketChannel channel) {
        this.channel = channel;
    }

    public Stream<String> pollFullPackets() {
        return Stream.iterate(
            this.pollString(),
            Objects::nonNull,
            (lastBuf) -> this.pollString()
        );
    }

    public String pollString() {
        if(this.incompleteBuffer == null) {
            this.incompleteBuffer = ByteBuffer.allocate(1024);
        }

        try {
            this.channel.read(incompleteBuffer);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        incompleteBuffer.flip();
        if(incompleteBuffer.hasRemaining()) {
            ByteBuffer currentStr = incompleteBuffer.slice();
            boolean terminated = false;
            int i;
            for (i = 0; incompleteBuffer.hasRemaining(); i++) {
                if(incompleteBuffer.get() == 0x00) {
                    terminated = true;
                    break;
                }
            }
            if(terminated) {
                currentStr.limit(i);
                if(!incompleteBuffer.hasRemaining()) {
                    incompleteBuffer = null;
                } else {
                    incompleteBuffer = deepCopy(incompleteBuffer.slice());
                }
                return StandardCharsets.UTF_8.decode(currentStr).toString();
            } else {
                return null;
            }
        }
        return null;
    }

    public ByteBuffer deepCopy(ByteBuffer source) {
        int sourceP = source.position();
        int sourceL = source.limit();

        ByteBuffer target = ByteBuffer.allocate(source.remaining());

        target.put(source);
        target.flip();

        source.position(sourceP);
        source.limit(sourceL);
        return target;
    }

    public boolean isOpen() {
        return channel.isOpen();
    }
}