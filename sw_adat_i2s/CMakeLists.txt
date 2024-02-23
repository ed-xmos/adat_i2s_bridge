cmake_minimum_required(VERSION 3.21)
include($ENV{XMOS_CMAKE_PATH}/xcommon.cmake)
project(adat_to_i2s)

set(XMOS_SANDBOX_DIR ${CMAKE_CURRENT_LIST_DIR}/..)
set(APP_HW_TARGET xk-audio-316-mc.xn)
set(APP_DEPENDENT_MODULES lib_i2c lib_i2s lib_logging "ed-xmos/lib_src(feature/asrc_wrapper)" lib_xassert "lib_xcore_math(xcommon_cmake)")
set(APP_COMPILER_FLAGS -O3 -g -D ADAT_REF=100 -D ADAT_TX_USE_SHARED_BUFF=1 -report -D ASRC_TASK_CONFIG=1 -D DEBUG_ASRC_TASK=1)

XMOS_REGISTER_APP()