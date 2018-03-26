
LOCAL_DIR = $(call my-dir)/src

INCLUDES += $(LOCAL_DIR)

SOURCES += $(addprefix $(LOCAL_DIR)/, ff.c diskio.c ff_gen_drv.c option/syscall.c)

