package com.team581;

import java.util.Optional;

public final class GlobalConfig {
  public static final boolean IS_DEVELOPMENT = true;

  public static final String SERIAL_NUMBER =
      Optional.ofNullable(System.getenv("serialnum")).orElse("unknown");

  private GlobalConfig() {}
}
