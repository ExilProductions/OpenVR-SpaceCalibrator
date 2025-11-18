cmake_minimum_required(VERSION 3.10)
project(OpenVR-SpaceCalibratorDriver)

add_library(openvr-spacecalibratordriver SHARED Hooking.cpp InterfaceHookInjector.cpp IPCServer.cpp Logging.cpp
										 OpenVR-SpaceCalibratorDriver.cpp ServerTrackedDeviceProvider.cpp)
target_include_directories(openvr-spacecalibratordriver PRIVATE ../lib/openvr/)