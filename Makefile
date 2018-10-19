#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#
VERBOSE := 1
PROJECT_NAME := dht-sensor
CFLAGS += -I$(PROJECT_PATH)/components/dht22
include $(IDF_PATH)/make/project.mk

