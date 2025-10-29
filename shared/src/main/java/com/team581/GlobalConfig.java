package com.team581;

import static java.util.Objects.requireNonNullElse;

public final class GlobalConfig {
  public static final boolean IS_DEVELOPMENT = true;

  public static final String SERIAL_NUMBER =
      requireNonNullElse(System.getenv("serialnum"), "unknown");

  private GlobalConfig() {}
}
