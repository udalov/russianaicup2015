import model.Game;

import java.io.File;
import java.io.PrintWriter;
import java.lang.reflect.Method;

public class ConstDump {
    private static boolean done = false;

    public static void run(Game game) {
        if (done) return;
        done = true;

        try {
            File file = new File("const.txt");
            PrintWriter out = new PrintWriter(file);
            for (Method method : game.getClass().getMethods()) {
                String name = method.getName();
                if (name.startsWith("get") && !name.equals("getClass")) {
                    String fieldName = Character.toLowerCase(name.charAt(3)) + name.substring(4);
                    Object value = method.invoke(game);
                    String valueStr =
                            value instanceof Double ? String.format("%.12f", (Double) value) : value.toString();
                    out.println(fieldName + " " + valueStr);
                }
            }
            out.close();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
