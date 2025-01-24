package frc.robot;


import com.moandjiezana.toml.Toml;

import jakarta.ws.rs.NotFoundException;

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
    
    public static <T> T mapString(Toml toml, Toml defaultToml, String key, String defaultVal, String[] options, T[] fields) {
        var value = toml.getString(key, defaultToml.getString(key, defaultVal));
        for (int i = 0; i < options.length; i++) {
            if (value.equals(options[i])) {
                return fields[i];
            }
        }
        throw new IllegalArgumentException("Invalid key");
    }

    public static Toml getTableOrDefault(Toml toml, Toml defaultToml, String key) {
        if (toml.contains(key)) {
            return toml.getTable(key);
        } else if (defaultToml.contains(key)) {
            return defaultToml.getTable(key);
        } else {
            throw new NotFoundException(String.format("Key %s, not found in table", key));
        }
    }
}
