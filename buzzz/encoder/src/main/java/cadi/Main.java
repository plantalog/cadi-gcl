package cadi;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.Arrays;

import cadi.core.Transport;
import cadi.core.Transport.PacketReceiveCallback;

public class Main {
    // this explains how Transport API works for sending/receiving packets
    public static void main(String[] args) throws IOException {
        // create output stream for writing data
        ByteArrayOutputStream os = new ByteArrayOutputStream();

        // write some crap bytes
        os.write('Z');
        os.write(123);
        os.write(1);
        os.write(0);
        
        // create instance of Transport to send packets and handle received packet events
        Transport transport = new Transport();
        
        //  write 2 packets with command 'get water'
        transport.sendCommand(os, Transport.COMMAND_GET_WATER, new byte[] {0, 1, 2, 3});
        transport.sendCommand(os, Transport.COMMAND_GET_WATER, new byte[] {123, 127, -128, 0});
        
        // some more crap
        os.write(12);
        
        // write packet with configuration
        transport.sendConfiguration(os, new byte[400]);
        
        // and a bit more crap
        os.write(100);
        os.write(102);
        
        // convert written data to byte array
        byte[] data = os.toByteArray();
        
        // feed data byte array as input to transport and see if packets will be handled correctly
        ByteArrayInputStream is = new ByteArrayInputStream(data);
        PacketReceiveCallback callback = new Transport.PacketReceiveCallback() {
            public void onConfiguration(byte[] payload) {
                System.out.println("Received Configuration: ");
                System.out.println(Arrays.toString(payload));
                System.out.println();
            }
            
            public void onCommand(int command, byte[] payload) {
                System.out.println("Received Command: ");
                System.out.println(command);
                System.out.println(Arrays.toString(payload));
                System.out.println();
            }
        };
        while (transport.receive(is, callback)) {
            // just process all the packets in input
        }
    }
}
