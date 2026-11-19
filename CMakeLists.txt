cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rapoo_npm1300)

target_sources(app PRIVATE
  src/main.c
  src/pwr_service.c
  src/advertising_ext.c
  src/fuel_gauge.c
)
