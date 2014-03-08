package cadi.core;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

// Utility class providing stream wrappers, which handle packet prefixing and prefix escaping details.
public final class PacketStreamUtil {
    private static final byte[] PACKET_PREFIX = {'Z', 'X'};
    
    public static OutputStream outputStream(int packetType, OutputStream target) {
        return new PacketOutputStream(packetType, target);
    }
    
    public static InputStream inputStream(InputStream is) {
        return new PacketInputStream(is);
    }
    
    // tThis stream writes packet start prefix and escapes any packet start sequence in data.
    private static class PacketOutputStream extends OutputStream {
        private OutputStream target;
        private int idx;
        private int packetType;
        private boolean headerSent;
        
        public PacketOutputStream(int packetType, OutputStream target) {
            this.target = target;
            this.packetType = packetType;
        }

        @Override
        public void write(int b) throws IOException {
            if (!headerSent) {
                target.write(PACKET_PREFIX);
                target.write(packetType);
                headerSent = true;
            }
            target.write(b);
            if (b == PACKET_PREFIX[idx]) { // 
                idx++;
                if (idx >= PACKET_PREFIX.length) { // if we wrote same sequence as prefix ...
                    target.write(0); // ... write escaping zero
                    idx = 0;
                }
            } else if (idx > 0) {
                idx = 0;
            }
        }
    }
    
    /*
     * This stream skips all the rubbish before packet start prefix and returns bytes after packet start sequennce.
     * It also unescapes any properly escaped prefixes.
     */
    private static class PacketInputStream extends InputStream {
        private InputStream source;
        private int idx;
        private boolean packetStartReached;
        
        public PacketInputStream(InputStream source) {
            this.source = source;
        }

        @Override
        public int read() throws IOException {
            if (!packetStartReached) {
                return readInputTillPacketStart();
            }
            int b = source.read();
            if (b == PACKET_PREFIX[idx]) {
                idx++;
                if (idx >= PACKET_PREFIX.length) {
                    if (source.read() != 0) {
                        throw new IllegalStateException("unescaped packet start sequence found in packet body");
                    }
                    idx = 0;
                }
            } else if (idx > 0) {
                idx = 0;
            }
            return b;
        }

        // read input until packet start is reached
        private int readInputTillPacketStart() throws IOException {
            int idx = 0;
            while (true) {
                int b = source.read();
                if (b < 0) {
                    return b;
                }
                if (b == PACKET_PREFIX[idx]) {
                    idx++;
                    if (idx >= PACKET_PREFIX.length) {
                        b = source.read();
                        if (b != 0) {
                            packetStartReached = true;
                            return b;
                        }
                        idx = 0;
                    }
                } else if (idx > 0) {
                    idx = 0;
                }
            }
        }
    }
}
