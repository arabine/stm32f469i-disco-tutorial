
LOCAL_DIR = $(call my-dir)/Source
INCLUDES += $(LOCAL_DIR)/include $(LOCAL_DIR)/CMSIS_RTOS $(LOCAL_DIR)/portable/GCC/ARM_CM4F

SOURCES += $(addprefix $(LOCAL_DIR)/, \
portable/MemMang/heap_4.c \
croutine.c \
tasks.c \
portable/GCC/ARM_CM4F/port.c \
timers.c \
list.c \
event_groups.c \
CMSIS_RTOS/cmsis_os.c \
queue.c)



