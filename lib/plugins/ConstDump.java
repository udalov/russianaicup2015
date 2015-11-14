import model.Game;

import java.io.File;
import java.io.PrintWriter;
import java.lang.reflect.Method;
import java.util.*;

public class ConstDump {
    private static boolean done = false;

    private static Set<String> skipFields = new HashSet<>(Arrays.asList(
            "class", "tickCount", "lapTickCount", "worldHeight", "worldWidth"
    ));

    public static void run(Game game) {
        if (done) return;
        done = true;

        try {
            List<String> result = new ArrayList<>();
            for (Method method : game.getClass().getMethods()) {
                String name = method.getName();
                if (!name.startsWith("get")) continue;

                String fieldName = Character.toLowerCase(name.charAt(3)) + name.substring(4);
                if (skipFields.contains(fieldName)) continue;

                Object value = method.invoke(game);
                result.add(fieldName + " " + renderValue(value));
            }
            Collections.sort(result);

            File file = new File("const.txt");
            PrintWriter out = new PrintWriter(file);
            for (String string : result) {
                out.println(string);
            }
            out.close();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    private static String renderValue(Object value) {
        if (value instanceof Double) {
            return String.format("%.12f", (Double) value);
        }
        if (value instanceof int[]) {
            return Arrays.toString((int[]) value);
        }
        return value.toString();
    }
}
