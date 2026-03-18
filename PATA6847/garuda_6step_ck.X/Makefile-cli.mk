# Makefile for garuda_6step_ck — dsPIC33CK64MP205 + ATA6847
# CLI build: make clean all
# Debug build: make clean all TYPE_IMAGE=DEBUG_RUN

CC = /opt/microchip/xc16/v2.10/bin/xc16-gcc
BIN2HEX = /opt/microchip/xc16/v2.10/bin/xc16-bin2hex

MCU = 33CK64MP205
LINKER_SCRIPT = p$(MCU).gld

TYPE_IMAGE ?= PRODUCTION
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
  IMAGE_TYPE = debug
  DEBUG_FLAGS = -g -D__DEBUG
else
  IMAGE_TYPE = production
  DEBUG_FLAGS =
endif

BUILDDIR = build/default/$(IMAGE_TYPE)
DISTDIR  = dist/default/$(IMAGE_TYPE)

CFLAGS = -mcpu=$(MCU) -O1 -Wall -mno-eds-warn -msmart-io=1 \
         -msfr-warn=off $(DEBUG_FLAGS)

LDFLAGS = -mcpu=$(MCU) -Wl,--script=$(LINKER_SCRIPT),--report-mem \
          -Wl,--gc-sections

SRCS = main.c \
       garuda_service.c \
       hal/clock.c \
       hal/port_config.c \
       hal/hal_uart.c \
       hal/hal_diag.c \
       hal/hal_spi.c \
       hal/hal_ata6847.c \
       hal/hal_adc.c \
       hal/hal_opa.c \
       hal/hal_pwm.c \
       hal/hal_timer1.c \
       hal/hal_ic.c \
       hal/hal_com_timer.c \
       hal/hal_clc.c \
       hal/hal_trap.c \
       hal/board_service.c \
       motor/commutation.c \
       motor/startup.c \
       motor/bemf_zc.c \
       gsp/gsp.c \
       gsp/gsp_commands.c \
       gsp/gsp_snapshot.c

OBJS = $(SRCS:%.c=$(BUILDDIR)/%.o)
ELF  = $(DISTDIR)/garuda_6step_ck.X.$(IMAGE_TYPE).elf
HEX  = $(DISTDIR)/garuda_6step_ck.X.$(IMAGE_TYPE).hex

.PHONY: all clean

all: $(HEX)
	@echo "Build complete: $(HEX)"

$(HEX): $(ELF)
	$(BIN2HEX) $<

$(ELF): $(OBJS) | $(DISTDIR)
	$(CC) $(LDFLAGS) -o $@ $(OBJS)

$(BUILDDIR)/%.o: %.c | $(BUILDDIR)/hal $(BUILDDIR)/motor $(BUILDDIR)/gsp
	$(CC) $(CFLAGS) -c $< -o $@ -MP -MMD -MF $@.d

$(BUILDDIR):
	mkdir -p $@

$(BUILDDIR)/hal: | $(BUILDDIR)
	mkdir -p $@

$(BUILDDIR)/motor: | $(BUILDDIR)
	mkdir -p $@

$(BUILDDIR)/gsp: | $(BUILDDIR)
	mkdir -p $@

$(DISTDIR):
	mkdir -p $@

clean:
	rm -rf build dist

-include $(OBJS:.o=.o.d)
