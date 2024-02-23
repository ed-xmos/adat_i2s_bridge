cmake_minimum_required(VERSION 3.21)
include($ENV{XMOS_CMAKE_PATH}/xcommon.cmake)
project(adat_to_i2s)

set(XMOS_SANDBOX_DIR ${CMAKE_CURRENT_LIST_DIR}/..)
set(APP_HW_TARGET xk-audio-316-mc.xn)
include(${CMAKE_CURRENT_LIST_DIR}/deps.cmake)
set(APP_COMPILER_FLAGS -O3 -g -DADAT_REF=100 -DADAT_TX_USE_SHARED_BUFF=1 -report -DASRC_TASK_CONFIG=1 -DDEBUG_ASRC_TASK=1)
set(APP_INCLUDES src ${CMAKE_CURRENT_LIST_DIR}/../../lib_src/lib_src/src/asrc_task/)

XMOS_REGISTER_APP()