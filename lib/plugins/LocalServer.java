import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedDeque;

public class LocalServer {
    public static LocalServer INSTANCE = new LocalServer();

    public final Queue<String> messages = new ConcurrentLinkedDeque<String>();

    private static void warn(String message) {
        System.err.println("local-server: " + message);
    }

    private static void log(String message) {
        // System.err.println("local-server: " + message);
    }

    public void run() {
        new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    log("starting");
                    ServerSocket server = new ServerSocket(29292);
                    server.setReceiveBufferSize(1 << 20);
                    log("waiting for connection...");
                    Socket socket = server.accept();
                    log("connection accepted");
                    DataInputStream inputStream = new DataInputStream(socket.getInputStream());
                    while (true) {
                        String string;
                        try {
                            while (inputStream.available() == 0) {
                                Thread.sleep(15);
                            }
                            string = inputStream.readUTF();
                        } catch (Exception e) {
                            warn("could not receive message: " + e.getMessage() + " (" + e.getClass().getName() + ")");
                            continue;
                        }
                        // System.out.println("MESSAGE RECEIVED (length " + string.length() + "): " + string);
                        messages.add(string);
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                    throw new RuntimeException(e);
                }
            }
        }).start();
    }
}
