SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR ?= $(abspath ../../../../../../..)

include $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/imports.mak

CC = "$(IAR_ARMCOMPILER)/bin/iccarm"
LNK = "$(IAR_ARMCOMPILER)/bin/ilinkarm"
HEX = "$(IAR_ARMCOMPILER)/bin/ielftool"

SYSCFG_CMD_STUB = $(SYSCONFIG_TOOL) --compiler iar --product $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/.metadata/product.json
SYSCFG_FILES := $(shell $(SYSCFG_CMD_STUB) --listGeneratedFiles --listReferencedFiles --output . ../../freertos/basic_ble_ptm.syscfg)

SYSCFG_C_FILES = $(filter %.c,$(SYSCFG_FILES))
SYSCFG_H_FILES = $(filter %.h,$(SYSCFG_FILES))
SYSCFG_OPT_FILES = $(filter %.opt,$(SYSCFG_FILES))

OBJECTS = app_app_main.obj app_app_ptm.obj common_DPL_Queue_freertos.obj common_BLEAppUtil_bleapputil_task.obj common_BLEAppUtil_bleapputil_init.obj common_BLEAppUtil_bleapputil_process.obj common_BLEAppUtil_bleapputil_stack_callbacks.obj app_Profiles_app_dev_info.obj common_Services_dev_info_dev_info_service.obj common_Startup_osal_icall_ble.obj common_Startup_rom_init.obj common_Startup_ble_user_config_stack.obj common_Drivers_NV_crc.obj common_Drivers_NV_nvocmp.obj common_iCall_icall_cc23x0.obj common_iCall_icall_user_config.obj common_iCallBLE_ble_stack_api.obj common_iCallBLE_ble_user_config.obj common_iCallBLE_icall_api_lite.obj common_NPI_npi_tl.obj common_NPI_npi_rxbuf.obj common_NPI_npi_task.obj common_NPI_npi_frame_hci.obj common_NPI_npi_tl_uart.obj common_iCallBLE_icall_hci_tl.obj common_Util_util.obj common_iCall_icall_POSIX.obj common_Startup_main_freertos.obj $(patsubst %.c,%.obj,$(notdir $(SYSCFG_C_FILES)))
NAME = basic_ble_ptm

CFLAGS += -I../.. \
    -I. \
    $(addprefix -f,$(SYSCFG_OPT_FILES)) \
    -I../../common/DPL \
    -DDisplay_DISABLE_ALL \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/npi/src" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/npi/src/inc" \
    -I../../app \
    -DICALL_NO_APP_EVENTS \
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
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/common/cc26xx/npi/stack" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/hal/src/inc" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/heapmgr" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/profiles/dev_info" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/profiles/simple_profile" \
    "-I$(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/src/inc" \
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

syscfg: ../../freertos/basic_ble_ptm.syscfg
	@ echo Generating configuration files...
	@ $(SYSCFG_CMD_STUB) --output $(@D) $<

define C_RULE
$(basename $(notdir $(1))).obj: $(1) $(SYSCFG_H_FILES)
	@ echo Building $$@
	@ $(CC) $(CFLAGS) $$< -o $$@
endef

$(foreach c_file,$(SYSCFG_C_FILES),$(eval $(call C_RULE,$(c_file))))

app_app_main.obj: ../../app/app_main.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

app_app_ptm.obj: ../../app/app_ptm.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_DPL_Queue_freertos.obj: ../../common/DPL/Queue_freertos.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_BLEAppUtil_bleapputil_task.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/bleapp/ble_app_util/src/bleapputil_task.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_BLEAppUtil_bleapputil_init.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/bleapp/ble_app_util/src/bleapputil_init.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_BLEAppUtil_bleapputil_process.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/bleapp/ble_app_util/src/bleapputil_process.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_BLEAppUtil_bleapputil_stack_callbacks.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/bleapp/ble_app_util/src/bleapputil_stack_callbacks.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

app_Profiles_app_dev_info.obj: ../../app/Profiles/app_dev_info.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_Services_dev_info_dev_info_service.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/bleapp/services/dev_info/dev_info_service.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_Startup_osal_icall_ble.obj: ../../common/Startup/osal_icall_ble.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_Startup_rom_init.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/rom/agama_r1/rom_init.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_Startup_ble_user_config_stack.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/stack/ble_user_config_stack.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_Drivers_NV_crc.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/common/nv/crc.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_Drivers_NV_nvocmp.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/common/nv/nvocmp.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_iCall_icall_cc23x0.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/src/icall_cc23x0.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_iCall_icall_user_config.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/src/icall_user_config.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_iCallBLE_ble_stack_api.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/app/ble_stack_api.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_iCallBLE_ble_user_config.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/app/ble_user_config.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_iCallBLE_icall_api_lite.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/app/icall_api_lite.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_NPI_npi_tl.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/npi/src/npi_tl.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_NPI_npi_rxbuf.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/npi/src/npi_rxbuf.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_NPI_npi_task.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/npi/src/npi_task.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_NPI_npi_frame_hci.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/npi/src/npi_frame_hci.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_NPI_npi_tl_uart.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/npi/src/npi_tl_uart.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_iCallBLE_icall_hci_tl.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/app/icall_hci_tl.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_Util_util.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/common/cc26xx/util.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_iCall_icall_POSIX.obj: $(SIMPLELINK_LOWPOWER_F3_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/src/icall_POSIX.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_Startup_main_freertos.obj: ../../common/Startup/main_freertos.c $(SYSCFG_H_FILES)
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
