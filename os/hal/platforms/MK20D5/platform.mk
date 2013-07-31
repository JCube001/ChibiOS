# List of all the template platform files.
PLATFORMSRC = ${CHIBIOS}/os/hal/MK20D5/hal_lld.c \
              ${CHIBIOS}/os/hal/MK20D5/adc_lld.c \
              ${CHIBIOS}/os/hal/MK20D5/can_lld.c \
              ${CHIBIOS}/os/hal/MK20D5/ext_lld.c \
              ${CHIBIOS}/os/hal/MK20D5/gpt_lld.c \
 			  ${CHIBIOS}/os/hal/MK20D5/i2c_lld.c \
              ${CHIBIOS}/os/hal/MK20D5/icu_lld.c \
              ${CHIBIOS}/os/hal/MK20D5/mac_lld.c \
              ${CHIBIOS}/os/hal/MK20D5/pal_lld.c \
              ${CHIBIOS}/os/hal/MK20D5/pwm_lld.c \
              ${CHIBIOS}/os/hal/MK20D5/sdc_lld.c \
              ${CHIBIOS}/os/hal/MK20D5/serial_lld.c \
              ${CHIBIOS}/os/hal/MK20D5/spi_lld.c \
              ${CHIBIOS}/os/hal/MK20D5/uart_lld.c \
              ${CHIBIOS}/os/hal/MK20D5/usb_lld.c

# Required include directories
PLATFORMINC = ${CHIBIOS}/os/hal/MK20D5
