package frc.robot;

import com.moandjiezana.toml.Toml;

public class TomlUtil {
    public static <T> T mapString(Toml toml, String key, String defaultVal, String[] options, T[] fields) {
        var value = toml.getString(key, defaultVal);
        for (int i = 0; i < options.length; i++) {
            if (value.equals(options[i])) {
                return fields[i];
            }
        }
        throw new IllegalArgumentException("Invalid key");
    }
}
