SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR ?= $(abspath ../../../../../../..)

include $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/imports.mak

CC = "$(IAR_ARMCOMPILER)/bin/iccarm"
LNK = "$(IAR_ARMCOMPILER)/bin/ilinkarm"
HEX = "$(IAR_ARMCOMPILER)/bin/ielftool"

SYSCFG_CMD_STUB = $(SYSCONFIG_TOOL) --compiler iar --product $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/.metadata/product.json
SYSCFG_FILES := $(shell $(SYSCFG_CMD_STUB) --listGeneratedFiles --listReferencedFiles --output . ../../freertos/host_test.syscfg)

SYSCFG_C_FILES = $(filter %.c,$(SYSCFG_FILES))
SYSCFG_H_FILES = $(filter %.h,$(SYSCFG_FILES))
SYSCFG_OPT_FILES = $(filter %.opt,$(SYSCFG_FILES))

OBJECTS = Application_host_test_app.obj Startup_osal_icall_ble.obj common_Util_util.obj Drivers_NV_crc.obj Drivers_NV_nvocmp.obj iCallBLE_ble_user_config.obj iCallBLE_ble_user_config_stack.obj iCallBLE_icall_api_lite.obj iCallBLE_icall_hci_tl.obj NPI_Transport_SPI_npi_tl_spi.obj NPI_Transport_UART_npi_tl_uart.obj NPI_Transport_npi_tl.obj NPI_npi_frame_hci.obj NPI_npi_rxbuf.obj NPI_npi_task.obj StackWrapper_HAL_Common_hal_assert.obj StackWrapper_HAL_Target_CC2650_Drivers_hal_flash_wrapper.obj StackWrapper_Host_gap.obj StackWrapper_Host_gapbondmgr.obj StackWrapper_Host_gatt_uuid.obj StackWrapper_Host_gattservapp_util.obj StackWrapper_Host_sm_ecc.obj StackWrapper_ICallBLE_ble_dispatch_JT.obj StackWrapper_ICallBLE_ble_dispatch_lite.obj StackWrapper_ICallBLE_icall_lite_translation.obj StackWrapper_OSAL_osal.obj StackWrapper_OSAL_osal_bufmgr.obj StackWrapper_OSAL_osal_cbtimer.obj StackWrapper_OSAL_osal_clock.obj StackWrapper_OSAL_osal_list.obj StackWrapper_OSAL_osal_memory_icall.obj StackWrapper_OSAL_osal_pwrmgr.obj StackWrapper_OSAL_osal_snv_wrapper.obj StackWrapper_OSAL_osal_timers.obj StackWrapper_ROM_common_rom_init.obj StackWrapper_Startup_icall_startup.obj StackWrapper_Tools_onboard.obj Startup_rom_init.obj iCall_icall_POSIX.obj Startup_main_freertos.obj Application_Queue_freertos.obj iCall_icall_cc23x0.obj iCall_icall_user_config.obj $(patsubst %.c,%.obj,$(notdir $(SYSCFG_C_FILES)))
NAME = host_test

CFLAGS += -I../.. \
    -I. \
    $(addprefix -f,$(SYSCFG_OPT_FILES)) \
    -I../../Application \
    -f../../Tools/build_config.opt \
    -f../../Tools/Defines/host_test.opt \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/common/cc26xx/npi/stack" \
    -Ohz \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/controller/cc26xx/inc" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/inc" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/rom" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/common/cc26xx" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/inc" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/hal/src/target/_common" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/hal/src/inc" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/heapmgr" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/profiles/dev_info" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/profiles/simple_profile" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/src/inc" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/npi/src" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/osal/src/inc" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/services/src/saddr" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/services/src/sdata" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/common/nv" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/common/cc26xx" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/src" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/bleapp/profiles/health_thermometer" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/bleapp/services/health_thermometer" \
    "-f$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/config/build_components.opt" \
    "-f$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/config/factory_config.opt" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/npi/src/inc" \
    -DCC23X0 \
    -DNVOCMP_NWSAMEITEM=1 \
    -DFLASH_ONLY_BUILD \
    -DUSE_RCL \
    -DNVOCMP_NVPAGES=6 \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/drivers/rcl" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/posix/iar" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/third_party/freertos/include" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/third_party/freertos/portable/IAR/ARM_CM0" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/kernel/freertos" \
    -DFREERTOS \
    -DNVOCMP_POSIX_MUTEX \
    --debug \
    --silent \
    -e \
    --aeabi \
    --thumb \
    --diag_suppress=Pa050 \
    --cpu=Cortex-M0+ \
    --vla

LFLAGS += --keep __vector_table \
    "$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/lib_projects/CC2340R5/OneLib/lib/iar/m0p/OneLib.a" \
    "-L$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source" \
    "-L$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/kernel" \
    --config_def PAGE_ALIGN=-1 \
    --threaded_lib \
    -fti_utils_build_linker.cmd.genlibs \
    --config ../../freertos/iar/lpf3_app_and_stack_freertos.icf \
    --map "$(NAME).map" \
    --silent \
    --semihosting=iar_breakpoint \
    --cpu=Cortex-M0+

all: postbuild

.PHONY: postbuild
postbuild: $(NAME).hex
	"$(IAR_ARMCOMPILER)/bin/ielftool" --ihex --checksum "__ccfg_boot_cfg_crc32_end+1:4,crc32:1mi,0xFFFFFFFF;__ccfg_boot_cfg_crc32_begin-__ccfg_boot_cfg_crc32_end" --checksum "__ccfg_crc32_end+1:4,crc32:1mi,0xFFFFFFFF;__ccfg_crc32_begin-__ccfg_crc32_end" --checksum "__ccfg_user_record_crc32_end+1:4,crc32:1mi,0xFFFFFFFF;__ccfg_user_record_crc32_begin-__ccfg_user_record_crc32_end" --checksum "__ccfg_debug_cfg_crc32_end+1:4,crc32:1mi,0xFFFFFFFF;__ccfg_debug_cfg_crc32_begin-__ccfg_debug_cfg_crc32_end" "./$(NAME).out" "./$(NAME).hex"
	"$(IAR_ARMCOMPILER)/bin/ielftool" --checksum "__ccfg_boot_cfg_crc32_end+1:4,crc32:1mi,0xFFFFFFFF;__ccfg_boot_cfg_crc32_begin-__ccfg_boot_cfg_crc32_end" --checksum "__ccfg_crc32_end+1:4,crc32:1mi,0xFFFFFFFF;__ccfg_crc32_begin-__ccfg_crc32_end" --checksum "__ccfg_user_record_crc32_end+1:4,crc32:1mi,0xFFFFFFFF;__ccfg_user_record_crc32_begin-__ccfg_user_record_crc32_end" --checksum "__ccfg_debug_cfg_crc32_end+1:4,crc32:1mi,0xFFFFFFFF;__ccfg_debug_cfg_crc32_begin-__ccfg_debug_cfg_crc32_end" "./$(NAME).out" "./$(NAME).out"

.INTERMEDIATE: syscfg
$(SYSCFG_FILES): syscfg
	@ echo generation complete

syscfg: ../../freertos/host_test.syscfg
	@ echo Generating configuration files...
	@ $(SYSCFG_CMD_STUB) --output $(@D) $<

define C_RULE
$(basename $(notdir $(1))).obj: $(1) $(SYSCFG_H_FILES)
	@ echo Building $$@
	@ $(CC) $(CFLAGS) $$< -o $$@
endef

$(foreach c_file,$(SYSCFG_C_FILES),$(eval $(call C_RULE,$(c_file))))

Application_host_test_app.obj: ../../Application/host_test_app.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

Startup_osal_icall_ble.obj: ../../Startup/osal_icall_ble.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_Util_util.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/common/cc26xx/util.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

Drivers_NV_crc.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/common/nv/crc.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

Drivers_NV_nvocmp.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/common/nv/nvocmp.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

iCallBLE_ble_user_config.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/app/ble_user_config.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

iCallBLE_ble_user_config_stack.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/stack/ble_user_config_stack.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

iCallBLE_icall_api_lite.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/app/icall_api_lite.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

iCallBLE_icall_hci_tl.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/app/icall_hci_tl.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

NPI_Transport_SPI_npi_tl_spi.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/npi/src/npi_tl_spi.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

NPI_Transport_UART_npi_tl_uart.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/npi/src/npi_tl_uart.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

NPI_Transport_npi_tl.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/npi/src/npi_tl.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

NPI_npi_frame_hci.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/npi/src/npi_frame_hci.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

NPI_npi_rxbuf.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/npi/src/npi_rxbuf.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

NPI_npi_task.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/npi/src/npi_task.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

StackWrapper_HAL_Common_hal_assert.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/hal/src/common/hal_assert.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

StackWrapper_HAL_Target_CC2650_Drivers_hal_flash_wrapper.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/hal/src/target/_common/hal_flash_wrapper.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

StackWrapper_Host_gap.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gap.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

StackWrapper_Host_gapbondmgr.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gapbondmgr.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

StackWrapper_Host_gatt_uuid.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gatt_uuid.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

StackWrapper_Host_gattservapp_util.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gattservapp_util.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

StackWrapper_Host_sm_ecc.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/sm_ecc.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

StackWrapper_ICallBLE_ble_dispatch_JT.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/stack/ble_dispatch_JT.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

StackWrapper_ICallBLE_ble_dispatch_lite.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/stack/ble_dispatch_lite.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

StackWrapper_ICallBLE_icall_lite_translation.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/src/icall_lite_translation.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

StackWrapper_OSAL_osal.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/osal/src/common/osal.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

StackWrapper_OSAL_osal_bufmgr.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/osal/src/common/osal_bufmgr.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

StackWrapper_OSAL_osal_cbtimer.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/osal/src/common/osal_cbtimer.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

StackWrapper_OSAL_osal_clock.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/osal/src/common/osal_clock.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

StackWrapper_OSAL_osal_list.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/osal/src/common/osal_list.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

StackWrapper_OSAL_osal_memory_icall.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/osal/src/common/osal_memory_icall.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

StackWrapper_OSAL_osal_pwrmgr.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/osal/src/common/osal_pwrmgr.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

StackWrapper_OSAL_osal_snv_wrapper.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/osal/src/mcu/cc26xx/osal_snv_wrapper.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

StackWrapper_OSAL_osal_timers.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/osal/src/common/osal_timers.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

StackWrapper_ROM_common_rom_init.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/rom/agama_r1/common_rom_init.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

StackWrapper_Startup_icall_startup.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/common/cc26xx/icall_startup.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

StackWrapper_Tools_onboard.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/common/cc26xx/onboard.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

Startup_rom_init.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/rom/agama_r1/rom_init.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

iCall_icall_POSIX.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/src/icall_POSIX.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

Startup_main_freertos.obj: ../../Startup/main_freertos.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

Application_Queue_freertos.obj: ../../Application/Queue_freertos.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

iCall_icall_cc23x0.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/src/icall_cc23x0.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

iCall_icall_user_config.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/src/icall_user_config.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

$(NAME).out: $(OBJECTS)
	@ echo linking $@
	@ $(LNK)  $(OBJECTS)  $(LFLAGS) -o $(NAME).out

$(NAME).hex: $(NAME).out
	@ echo building $@
	@ $(HEX) --ihex --verbose $? $@

clean:
	@ echo Cleaning...
	@ $(RM) $(OBJECTS) > $(DEVNULL) 2>&1
	@ $(RM) $(NAME).out > $(DEVNULL) 2>&1
	@ $(RM) $(NAME).map > $(DEVNULL) 2>&1
	@ $(RM) $(NAME).hex > $(DEVNULL) 2>&1
	@ $(RM) $(SYSCFG_FILES)> $(DEVNULL) 2>&1
