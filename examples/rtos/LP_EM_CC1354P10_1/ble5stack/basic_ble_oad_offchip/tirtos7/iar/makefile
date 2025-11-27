SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR ?= $(abspath ../../../../../../..)

include $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/imports.mak

CC = "$(IAR_ARMCOMPILER)/bin/iccarm"
LNK = "$(IAR_ARMCOMPILER)/bin/ilinkarm"
HEX = "$(IAR_ARMCOMPILER)/bin/ielftool"

SYSCONFIG_GUI_TOOL = $(dir $(SYSCONFIG_TOOL))sysconfig_gui$(suffix $(SYSCONFIG_TOOL))
SYSCFG_CMD_STUB = $(SYSCONFIG_TOOL) --compiler iar --product $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/.metadata/product.json
SYSCFG_GUI_CMD_STUB = $(SYSCONFIG_GUI_TOOL) --compiler iar --product $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/.metadata/product.json
SYSCFG_FILES := $(shell $(SYSCFG_CMD_STUB) --listGeneratedFiles --listReferencedFiles --output . ../../tirtos7/basic_ble_oad_offchip.syscfg)

SYSCFG_C_FILES = $(filter %.c,$(SYSCFG_FILES))
SYSCFG_H_FILES = $(filter %.h,$(SYSCFG_FILES))
SYSCFG_OPT_FILES = $(filter %.opt,$(SYSCFG_FILES))

OBJECTS = app_app_main.obj app_app_menu.obj app_app_broadcaster.obj app_app_peripheral.obj app_app_observer.obj app_app_central.obj app_app_connection.obj app_app_pairing.obj app_app_data.obj app_Profiles_app_oad.obj common_Startup_osal_icall_ble.obj common_Profiles_oad_oad_profile.obj common_Services_oad_oad_service.obj common_Util_SwUpdate_flash_interface_ext_rtos_NVS.obj common_Util_SwUpdate_sw_update.obj common_Util_util.obj common_BLEAppUtil_bleapputil_task.obj common_BLEAppUtil_bleapputil_init.obj common_BLEAppUtil_bleapputil_process.obj common_BLEAppUtil_bleapputil_stack_callbacks.obj common_MenuModule_menu_module.obj common_Profiles_simple_gatt_simple_gatt_profile.obj common_Services_dev_info_dev_info_service.obj app_Profiles_app_dev_info.obj app_Profiles_app_simple_gatt.obj common_Startup_rom_init.obj common_Startup_ble_user_config_stack.obj common_Drivers_NV_crc.obj common_Drivers_NV_nvocmp.obj common_iCallBLE_ble_stack_api.obj common_iCallBLE_ble_user_config.obj common_iCallBLE_icall_api_lite.obj common_Startup_main.obj iCall_icall.obj iCall_icall_cc2650.obj iCall_icall_user_config.obj $(patsubst %.c,%.obj,$(notdir $(SYSCFG_C_FILES)))
NAME = basic_ble_oad_offchip

CFLAGS += -I../.. \
    -I. \
    $(addprefix -f,$(SYSCFG_OPT_FILES)) \
    "-f$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/config/build_components.opt" \
    "-f$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/config/factory_config.opt" \
    -I../../App \
    -DICALL_NO_APP_EVENTS \
    -DOAD_CFG \
    -DAPP_HDR_ADDR=0x6000 \
    -DOAD_APP_OFFCHIP \
    -Ohz \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/controller/cc26xx/inc" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/inc" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/rom" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/common/cc26xx" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/inc" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/hal/src/target/_common" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/common/cc26xx/npi/stack" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/hal/src/inc" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/heapmgr" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/profiles/dev_info" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/profiles/simple_profile" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/src/inc" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/npi/src" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/osal/src/inc" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/services/src/saddr" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/services/src/sdata" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/common/nv" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/common/cc26xx" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/src" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/bleapp/profiles/health_thermometer" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/bleapp/services/health_thermometer" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/third_party/mcuboot/boot/bootutil/include" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/third_party/mcuboot/boot/ti/source/mcuboot_app" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/common/cc26xx/flash_interface" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/drivers/nvs" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/devices/cc13x4_cc26x4/driverlib" \
    -DFLASH_ONLY_BUILD \
    -DNVOCMP_NWSAMEITEM=1 \
    -DNVOCMP_NVPAGES=4 \
    --diag_suppress=Pa205 \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack/profiles/simple_profile" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack/profiles/simple_profile/cc26xx" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack/hal/src/target/cc2650/rom" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/devices/cc13x4_cc26x4" \
    -DEM_CC1354P10_1_LP \
    -DCC13XX \
    -DCC13X4 \
    -DHEAPMGR_CONFIG=0x80 \
    -DHEAPMGR_SIZE=0x0 \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/kernel/tirtos7/packages" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/posix/iar" \
    --debug \
    --silent \
    -e \
    --aeabi \
    --thumb \
    --diag_suppress=Pa050 \
    --cpu Cortex-M33 \
    --fpu VFPv5_sp \
    --vla

LFLAGS += --keep __vector_table \
    --config_def CC2642=1 \
    --config_def PAGE_ALIGN=1 \
    --config_def APP_HDR_ADDR=0x6000 \
    --config_def OAD_APP_OFFCHIP=1 \
    "-L$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source" \
    --config_def PAGE_ALIGN=-1 \
    --config_def FLASH_ONLY_BUILD=2 \
    "-L$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/kernel/tirtos7/packages" \
    -fti_utils_build_linker.cmd.genlibs \
    --config ../../tirtos7/iar/cc13x4_app_and_stack_agama_tirtos7.icf \
    --map "$(NAME).map" \
    --silent \
    --semihosting=iar_breakpoint \
    --cpu=Cortex-M33 \
    --fpu VFPv5_sp

all: postbuild

.PHONY: postbuild
postbuild: $(NAME).hex
	"$(IAR_ARMCOMPILER)/bin/ielftool" --bin ""./$(NAME)".out" ""./$(NAME)"_noheader.bin"
	$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/tools/common/mcuboot/imgtool sign --header-size 0x100 --align 4 --slot-size 0xF6000 --version 1.0.0 --pad-header --pad --key $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/third_party/mcuboot/root-ec-p256.pem "./$(NAME)_noheader.bin" "./$(NAME)_v1.bin"
	$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/tools/common/mcuboot/imgtool sign --header-size 0x100 --align 4 --slot-size 0xF6000 --version 2.0.0 --pad-header --pad --key $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/third_party/mcuboot/root-ec-p256.pem "./$(NAME)_noheader.bin" "./$(NAME)_v2.bin"

.INTERMEDIATE: syscfg
$(SYSCFG_FILES): syscfg
	@ echo generation complete

syscfg: ../../tirtos7/basic_ble_oad_offchip.syscfg
	@ echo Generating configuration files...
	@ $(SYSCFG_CMD_STUB) --output $(@D) $<


# Helpful hint that the user needs to use a standalone SysConfig installation
$(SYSCONFIG_GUI_TOOL):
	$(error $(dir $(SYSCONFIG_TOOL)) does not contain the GUI framework \
        necessary to launch the SysConfig GUI.  Please set SYSCONFIG_TOOL \
        (in your SDK's imports.mak) to a standalone SysConfig installation \
        rather than one inside CCS)

syscfg-gui: ../../tirtos7/basic_ble_oad_offchip.syscfg $(SYSCONFIG_GUI_TOOL)
	@ echo Opening SysConfig GUI
	@ $(SYSCFG_GUI_CMD_STUB) $<

define C_RULE
$(basename $(notdir $(1))).obj: $(1) $(SYSCFG_H_FILES)
	@ echo Building $$@
	@ $(CC) $(CFLAGS) $$< -o $$@
endef

$(foreach c_file,$(SYSCFG_C_FILES),$(eval $(call C_RULE,$(c_file))))

app_app_main.obj: ../../app/app_main.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

app_app_menu.obj: ../../app/app_menu.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

app_app_broadcaster.obj: ../../app/app_broadcaster.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

app_app_peripheral.obj: ../../app/app_peripheral.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

app_app_observer.obj: ../../app/app_observer.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

app_app_central.obj: ../../app/app_central.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

app_app_connection.obj: ../../app/app_connection.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

app_app_pairing.obj: ../../app/app_pairing.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

app_app_data.obj: ../../app/app_data.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

app_Profiles_app_oad.obj: ../../app/Profiles/app_oad.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_Startup_osal_icall_ble.obj: ../../common/Startup/osal_icall_ble.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_Profiles_oad_oad_profile.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/bleapp/profiles/oad/oad_profile.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_Services_oad_oad_service.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/bleapp/services/oad/oad_service.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_Util_SwUpdate_flash_interface_ext_rtos_NVS.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/common/cc26xx/flash_interface/external/flash_interface_ext_rtos_NVS.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_Util_SwUpdate_sw_update.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/bleapp/util/sw_update/sw_update.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_Util_util.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/common/cc26xx/util.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_BLEAppUtil_bleapputil_task.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/bleapp/ble_app_util/src/bleapputil_task.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_BLEAppUtil_bleapputil_init.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/bleapp/ble_app_util/src/bleapputil_init.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_BLEAppUtil_bleapputil_process.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/bleapp/ble_app_util/src/bleapputil_process.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_BLEAppUtil_bleapputil_stack_callbacks.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/bleapp/ble_app_util/src/bleapputil_stack_callbacks.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_MenuModule_menu_module.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/bleapp/menu_module/menu_module.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_Profiles_simple_gatt_simple_gatt_profile.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/bleapp/profiles/simple_gatt/simple_gatt_profile.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_Services_dev_info_dev_info_service.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/bleapp/services/dev_info/dev_info_service.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

app_Profiles_app_dev_info.obj: ../../app/Profiles/app_dev_info.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

app_Profiles_app_simple_gatt.obj: ../../app/Profiles/app_simple_gatt.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_Startup_rom_init.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/rom/agama_r1/rom_init.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_Startup_ble_user_config_stack.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/stack/ble_user_config_stack.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_Drivers_NV_crc.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/common/nv/crc.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_Drivers_NV_nvocmp.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/common/nv/nvocmp.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_iCallBLE_ble_stack_api.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/app/ble_stack_api.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_iCallBLE_ble_user_config.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/app/ble_user_config.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_iCallBLE_icall_api_lite.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/app/icall_api_lite.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

common_Startup_main.obj: ../../common/Startup/main.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

iCall_icall.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/src/icall.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

iCall_icall_cc2650.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/src/icall_cc2650.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

iCall_icall_user_config.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/src/icall_user_config.c $(SYSCFG_H_FILES)
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
