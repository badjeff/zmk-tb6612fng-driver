# Copyright (c) 2024 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_ZMK_BEHAVIOR_TB6612FNG src/behavior_tb6612fng.c)
zephyr_library_sources_ifdef(CONFIG_TB6612FNG src/tb6612fng.c)
zephyr_include_directories(include)

zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
