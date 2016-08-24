import java.io.DataInputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;

public class LocalServer {
    public static LocalServer INSTANCE = new LocalServer();

    private static final int PORT = 29292;
    private static final int BUFFER_SIZE = 1 << 20;
    private static final LogLevel LOG_LEVEL = LogLevel.WARN;

    public final Queue<String> messages = new ArrayBlockingQueue<>(1 << 10);

    private enum LogLevel {
        WARN,
        INFO,
        LOG,
    }

    private static void warn(String message) {
        if (LOG_LEVEL.ordinal() >= LogLevel.WARN.ordinal()) {
            System.err.println("local-server [WARN]: " + message);
        }
    }

    private static void info(String message) {
        if (LOG_LEVEL.ordinal() >= LogLevel.INFO.ordinal()) {
            System.err.println("local-server [info]: " + message);
        }
    }

    private static void log(String message) {
        if (LOG_LEVEL.ordinal() >= LogLevel.LOG.ordinal()) {
            System.err.println("local-server [log]: " + message);
        }
    }

    public void run() {
        new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    info("starting");
                    ServerSocket server = new ServerSocket(PORT);
                    server.setReceiveBufferSize(BUFFER_SIZE);
                    info("waiting for connection...");
                    Socket socket = server.accept();
                    info("connection accepted");
                    socket.setSendBufferSize(BUFFER_SIZE);
                    socket.setReceiveBufferSize(BUFFER_SIZE);
                    socket.setTcpNoDelay(true);
                    DataInputStream inputStream = new DataInputStream(socket.getInputStream());
                    while (true) {
                        try {
                            String string = inputStream.readUTF();
                            log("MESSAGE RECEIVED (length " + string.length() + "): " + string);
                            messages.add(string);
                        } catch (Exception e) {
                            warn("could not receive message: " + e.getMessage() + " (" + e.getClass().getName() + ")");
                        }
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                    throw new RuntimeException(e);
                }
            }
        }).start();
    }
}
