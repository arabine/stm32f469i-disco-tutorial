
LOCAL_DIR = $(call my-dir)

INCLUDES += $(LOCAL_DIR)/Class/CDC/Inc $(LOCAL_DIR)/Core/Inc

SOURCES += $(addprefix $(LOCAL_DIR)/Core/Src/, usbh_core.c \
usbh_ioreq.c \
usbh_pipes.c \
usbh_ctlreq.c)

SOURCES += $(addprefix $(LOCAL_DIR)/Class/CDC/Src/, usbh_cdc.c)


