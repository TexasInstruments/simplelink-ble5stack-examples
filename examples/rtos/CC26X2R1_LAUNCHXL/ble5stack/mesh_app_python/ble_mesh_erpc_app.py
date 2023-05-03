#!/usr/bin/python

import os
import sys
import time

# Import the relevant classes for the Example Application from the ble_mesh_erpc_python package
from ble_mesh_erpc_python.ble_mesh_node_erpc import BleMeshNodeErpc
from ble_mesh_erpc_python.ble_mesh_composition_data import BleMeshModelPub
from ble_mesh_erpc_python.lprf_erpc.lprf_api import common as lprf_api_common

# Import the BLE Mesh Supported Models
from ble_mesh_erpc_python.ble_mesh_composition_data import BleMeshHealthSrvModel
from ble_mesh_erpc_python.ble_mesh_vnd_model import VndModelExample
from ble_mesh_erpc_python.ble_mesh_generic_models import GenericOnOffClientModel, GenericOnOffServerModel


###################### Basic CLI Menu Screen object ###########################################################################################

class MenuScreenCLI:

    def __init__(self, menu_header, commands, upper_menu_screen=None):
        """
            Init the Basic CLI Menu Screen attributes
        :param menu_header: menu header string
        :param commands: menu commands data dict {'command key':{"DESC": str value, "ACTION": str function name to call}}
        :param upper_menu_screen: upper menu screen object
        """

        self._menu_header = menu_header
        self._commands = commands
        self._upper_menu_screen = upper_menu_screen

    def set_commands(self, commands):
        """
            Set the command data dict attribute
        :param commands: menu commands data dict {'command key':{"DESC": str value, "ACTION": str function name to call}}
        """

        self._commands = commands

    def get_commands(self):
        """
            Get the menu commands data dict attribute
        :return: self._commands
        """

        return self._commands

    def set_menu_header(self, msg):
        """
            Set the menu header attribute
        :param msg: header string
        """

        self._menu_header = "{}".format(msg)

    def get_menu_header(self):
        """
            Get the menu header attribute
        :return: self._menu_header
        """

        return self._menu_header

    def set_upper_menu(self, menu_screen):
        """
            Set the upper menu object attribute
        :param menu_screen: upper menu object
        """

        self._upper_menu_screen = menu_screen

    def get_upper_menu(self):
        """
            Get the upper menu object attribute
        :return: self._upper_menu_screen
        """

        return self._upper_menu_screen

    @staticmethod
    def get_int_input(msg):
        """
            Gets an integer input from the user
        :param msg: The msg to display to the user
        """

        while True:
            print(msg)
            try:
                value = int(input())
                return value
            except ValueError:
                print("Value int input error")

    @staticmethod
    def get_hex_input(msg):
        """
            Gets an hex input from the user
        :param msg: The msg to display to the user
        """

        while True:
            print(msg)
            try:
                value = hex(input())
                return value
            except TypeError:
                print("Value hex input error")

    def update_command(self, key, description, action):
        """
            Update menu command data
        :param key: command key
        :param description: command description
        :param action: command action function
        """

        self._commands[key] = {"DESC": description, "ACTION": action}

    def del_command(self, key):
        """
            Delete command from menu
        :param key: command key
        """

        if key in self._commands:
            del self._commands[key]

    def print_menu(self, cls=True):
        """
            Print the menu header and commands to CLI
        """

        if cls:
            os.system("cls")

        print(self._menu_header)
        for cmd in self._commands:
            print("{}. {}".format(cmd, self._commands[cmd]["DESC"]))
        print("")

    def run_menu(self):
        """
            Run The Menu CLI Screen
        """

        self.print_menu()
        self.wait_for_input()

    def execute_command(self, cmd):
        """
            Execute command action
        :param cmd: command key
        """

        if cmd in self._commands:
            exec("self.{}".format(self._commands[cmd]["ACTION"]))
        else:
            print("Command Value {} is not supported in this menu!!!".format(cmd))

    def wait_for_input(self):
        """
            Wait for user input (the main loop)
        """

        while True:
            ret_value = input("\nPlease Enter Command: ")
            self.execute_command(ret_value)
            time.sleep(1.5)

    def back(self):
        """
            Return to upper menu screen
        """

        if self._upper_menu_screen is not None:
            self._upper_menu_screen.run_menu()

    def print_help(self):
        self.print_menu(cls=False)

    @staticmethod
    def quit():
        """
            Exit CLI Menu Application
        """

        print("Quit")
        sys.exit()


###################### BLE Mesh Node eRPC Example CLI Menu Screen objects #####################################################################

class ErpcMainMenu(MenuScreenCLI):

    def __init__(self, device_port):
        """
            Set eRPC Main Menu Screen Attributes
        :param device_port: device serial port name
        """

        # Set the MenuScreen Attributes
        header = "--------BLE Mesh eRPC Node--------\n\n" \
                 "Connected to device in port {}\n".format(device_port)

        commands = {'1': {"DESC": "Mesh init and unprovisioned beacon", "ACTION": "mesh_init_unprovisioned_beacon()"},
                    '2': {"DESC": "Mesh Init and static provisioning", "ACTION": "mesh_init_static_provisioning()"},
                    '3': {"DESC": "Mesh Init and load from NV", "ACTION": "mesh_init_load_from_nv()"},
                    '4': {"DESC": "Mesh load provisioning information from NV", "ACTION": "load()"},
                    '5': {"DESC": "Mesh is provisioned", "ACTION": "mesh_is_provisioned()"},
                    '6': {"DESC": "Mesh Prov Enable", "ACTION": "mesh_prov_enable()"},
                    '7': {"DESC": "Mesh Reset", "ACTION": "mesh_reset()"},
                    '8': {"DESC": "Mesh Generic OnOff Client Menu", "ACTION": "generic_onoff_cli_menu()"},
                    '9': {"DESC": "Mesh VND Model Menu", "ACTION": "vnd_model_example_menu()"},
                    '10': {"DESC": "Mesh find model index", "ACTION": "find_model()"},
                    '11': {"DESC": "Mesh find vnd model index", "ACTION": "find_vnd_model()"},
                    '12': {"DESC": "Mesh LPN set", "ACTION": "lpn_set()"},
                    '13': {"DESC": "Mesh LPN poll", "ACTION": "lpn_poll()"},
                    '14': {"DESC": "Mesh PROXY Identity enable", "ACTION": "identity_enable()"},
                    '15': {"DESC": "Generate Fault on Helath server (for testing only)", "ACTION": "generate_fault()"},
                    'h': {"DESC": "Print Menu Help", "ACTION": "print_help()"},
                    'q': {"DESC": "Exit", "ACTION": "quit()"}}

        MenuScreenCLI.__init__(self, header, commands)

        # Create and Start the eRPC BLE Mesh Node eRPC
        self._device_port = device_port
        self.ble_mesh_node_erpc = BleMeshNodeErpc(self._device_port)
        self.ble_mesh_node_erpc.start()

        self._generic_onoff_cli_menu_screen = None
        self._vnd_model_example_menu_screen = None

    def get_device_port_number(self):
        """
            Get the device serial port name
        :return: self._device_port
        """

        return self._device_port

    def quit(self):
        """
            Close the eRPC communication and exit the app
        """

        print("Quit")
        self.ble_mesh_node_erpc.stop()
        print("Disconnect from device in port {}".format(self._device_port))
        sys.exit()

    def generic_onoff_cli_menu(self):
        """
            Generic OnOff Client Model Menu: Init and start the generic onoff client Menu Screen
        """

        if self._generic_onoff_cli_menu_screen is None:
            self._generic_onoff_cli_menu_screen = GenericOnOffClientMenu(self.ble_mesh_node_erpc)
        self._generic_onoff_cli_menu_screen.set_upper_menu(self)
        self._generic_onoff_cli_menu_screen.run_menu()

    def vnd_model_example_menu(self):
        """
            VND Model Menu: Init and start the vnd model example Menu Screen
        """

        if self._vnd_model_example_menu_screen is None:
            self._vnd_model_example_menu_screen = VndModelExampleMenu(self.ble_mesh_node_erpc)
        self._vnd_model_example_menu_screen.set_upper_menu(self)
        self._vnd_model_example_menu_screen.run_menu()

    def mesh_init_unprovisioned_beacon(self):
        """
            Init the Mesh data and call mesh_prov_enable
        """

        self.mesh_prov_data_init()
        self.mesh_comp_data_init(elem_count=1)
        self.mesh_primary_element_init(model_count=4, vnd_model_count=1)
        self.mesh_cfg_srv_init()
        self.mesh_health_srv_init()
        self.mesh_gen_onoff_client_init()
        self.mesh_gen_onoff_server_init()
        self.mesh_vnd_model_init()
        self.mesh_init()
        self.mesh_prov_enable()

    def mesh_init_static_provisioning(self):
        """
            Init the Mesh data and do static provisioning
        """

        node_address = self.get_int_input("Enter the node address:")
        self.mesh_prov_data_init()
        self.mesh_comp_data_init(elem_count=1)
        self.mesh_primary_element_init(model_count=5, vnd_model_count=1)
        self.mesh_cfg_srv_init()
        self.mesh_health_srv_init()
        self.mesh_gen_onoff_client_init()
        self.mesh_gen_onoff_server_init()
        self.mesh_cfg_cli_init()
        self.mesh_vnd_model_init()
        self.mesh_init()
        self.mesh_provision(node_address=node_address)

    def mesh_init_load_from_nv(self):
        """
            Init the Mesh data and load from NV
        """

        self.mesh_prov_data_init()
        self.mesh_comp_data_init(elem_count=1)
        self.mesh_primary_element_init(model_count=4, vnd_model_count=1)
        self.mesh_cfg_srv_init()
        self.mesh_health_srv_init()
        self.mesh_gen_onoff_client_init()
        self.mesh_gen_onoff_server_init()
        self.mesh_vnd_model_init()
        self.mesh_init()
        self.load()

    def mesh_prov_data_init(self):
        """
            Configure bt_mesh_prov data
        """

        print("")
        print("Configure bt_mesh_prov data")
        print("---------------------------")

        rc = self.ble_mesh_node_erpc.client_api.bt_mesh_init_prov(uuid=[0x53, 0x69, 0x6d, 0x70,
                                                                        0x6c, 0x65, 0x10, 0x4d,
                                                                        0xa5, 0x73, 0x68, 0x20,
                                                                        0x4e, 0x6f, 0x64, 0x65], uri='tiagamaled')

        print("bt_mesh_init_prov_init(prov_raw): rc=%d" % rc)
        print("")

    def mesh_comp_data_init(self, elem_count):
        """
            Configure bt_mesh_comp data
        """

        print("")
        print("Configure bt_mesh_comp data")
        print("---------------------------")

        rc = self.ble_mesh_node_erpc.client_api.bt_mesh_init_comp(cid=0x000D, pid=0, vid=0, elem_count=elem_count)

        print("bt_mesh_init_comp_init(comp_raw): rc=%d" % rc)
        print("")

    def mesh_primary_element_init(self, model_count, vnd_model_count=0):
        """
            Configure the primary_element
        """

        print("")
        print("Configure primary_element data")
        print("----------------------------")
        rc = self.ble_mesh_node_erpc.client_api.bt_mesh_init_primary_element(addr=1, loc=1, model_count=model_count, vnd_model_count=vnd_model_count)
        print("bt_mesh_init_primary_element(elem_raw): rc=%d" % rc)
        print("")

    def mesh_cfg_srv_init(self):
        """
            Configure cfg_srv model data
        """

        print("")
        print("Configure cfg_srv model data")
        print("----------------------------")

        rc = self.ble_mesh_node_erpc.client_api.bt_mesh_init_cfg_srv()

        print("bt_mesh_cfg_srv_init(cfg_srv_raw): rc=%d" % rc)
        print("")

    def mesh_cfg_cli_init(self):
        """
            Configure cfg_cli model data
        """

        print("")
        print("Configure cfg_cli model")
        print("-----------------------")

        rc = self.ble_mesh_node_erpc.client_api.bt_mesh_init_cfg_cli(elem_index=0)

        print("bt_mesh_cfg_cli_init(cfg_srv_raw): rc=%d" % rc)
        print("")

    def mesh_health_srv_init(self):
        """
            Configure health_srv model data
        """

        print("")
        print("Configure health_srv model data")
        print("----------------------------")

        rc = self.ble_mesh_node_erpc.client_api.bt_mesh_init_health_srv(elem_index=0, max_faults=0)

        print("bt_mesh_init_health_srv(health_srv_raw): rc=%d" % rc)
        print("")

    def mesh_gen_onoff_client_init(self):
        """
            Configure generic_onoff client model data
        """

        print("")
        print("Configure generic_onoff client model")
        print("----------------------------")

        gen_onoff_cli = GenericOnOffClientModel(element_index=0)

        rc, model_idx = self.ble_mesh_node_erpc.client_api.bt_mesh_init_sig_model(sig_model=gen_onoff_cli)

        # Set the app key index
        if rc == 0:
            self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].sig_models[model_idx].set_app_key_index(0)

        print("bt_mesh_vnd_model_init(model_raw): rc=%d" % rc)
        print("")

    def mesh_gen_onoff_server_init(self):
        """
            Configure generic_onoff server model data
        """

        print("")
        print("Configure generic_onoff server model")
        print("----------------------------")

        gen_onoff_srv = GenericOnOffServerModel(element_index=0)

        rc, model_idx = self.ble_mesh_node_erpc.client_api.bt_mesh_init_sig_model(sig_model=gen_onoff_srv)

        # Set the app key index
        if rc == 0:
            self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].sig_models[model_idx].set_app_key_index(0)

        print("bt_mesh_vnd_model_init(model_raw): rc=%d" % rc)
        print("")

    def mesh_vnd_model_init(self):
        """
            Configure bt_mesh_model data (vnd models)
        """

        print("")
        print("Configure bt_mesh_model data (vnd models)")
        print("-----------------------------------------")

        # Create the model publication object

        vnd_model_pub_examp = BleMeshModelPub()
        vnd_model_examp = VndModelExample(element_index=0, model_pub=vnd_model_pub_examp)
        vnd_model_examp.model_pub.set_model_pub_raw(addr=1, key=0, ttl=5, retransmit=5, period=50, period_div=1,
                                                    period_start=7,
                                                    msg=vnd_model_examp.model_api.net_buf_simple_raw([5, 4, 3], 5, 0))
        # Call bt_mesh_vnd_model_init
        rc, model_idx = self.ble_mesh_node_erpc.client_api.bt_mesh_init_vnd_model(vnd_model=vnd_model_examp)
        print("BLE Mesh VND Model init: elem_idx={}, model_idx={}".format(0, model_idx))
        for opcode in vnd_model_examp.opcodes:
            print("opcode={:X}, min_len={}, cb={}".format(opcode[0], opcode[1], opcode[2]))

        # Set the app key index
        if rc == 0:
            self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].vnd_models[model_idx].set_app_key_index(0)

        print("bt_mesh_vnd_model_init(model_raw): rc=%d" % rc)
        print("")

    def extend_model(self):
        """
               Let a model extend another model
        """

        rc = self.ble_mesh_node_erpc.client_api.ble_mesh_api.bt_mesh_model_extend_wrapper(mod_elem_idx=0,
                                                                                          mod_is_vnd=1,
                                                                                          mod_idx=0,
                                                                                          base_mod_elem_idx=0,
                                                                                          base_mod_is_vnd=0,
                                                                                          base_mod_idx=1)
        print("bt_mesh_model_extend_wrapper(): rc=%d" % rc)
        print("")

    def mesh_init(self):
        """
            Call bt_mesh_init
        """

        print("")
        print("Call mesh_init")
        print("--------------")

        rc = self.ble_mesh_node_erpc.client_api.bt_mesh_init()

        print("bt_mesh_init: rc=%d" % rc)
        print("")

    def mesh_provision(self, node_address=1):
        """
            Configure and Call bt_mesh_provision
        """

        print("")
        print("Call bt_mesh_provision")
        print("-----------------")
        net_key = [0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef]
        net_idx = 0
        dev_key = [0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef]
        app_key = [0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef]
        app_idx = 0

        rc = self.ble_mesh_node_erpc.client_api.bt_mesh_provision(addr=node_address,
                                                                  net_key=net_key, net_idx=net_idx,
                                                                  dev_key=dev_key)
        print("bt_mesh_provision: rc=%d" % rc)

        print("")
        print("Configure the Node")
        print("-----------------")

        # Add application key
        rc = self.ble_mesh_node_erpc.client_api.bt_mesh_cfg_app_key_add(app_key=app_key,
                                                                        app_idx=app_idx)
        time.sleep(0.5)
        print("bt_mesh_cfg_app_key_add: rc=%d" % rc)

        # Bind the Health Server model
        rc = self.ble_mesh_node_erpc.client_api.bt_mesh_cfg_mod_app_bind(elem_addr=self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].elem_addr,
                                                                         mod_app_idx=app_idx,
                                                                         mod_id=lprf_api_common.BT_MESH_MODEL_ID_HEALTH_SRV)
        time.sleep(0.5)
        print("bt_mesh_cfg_mod_app_bind: rc=%d" % rc)

        # bind the Generic OnOff Client model
        rc = self.ble_mesh_node_erpc.client_api.bt_mesh_cfg_mod_app_bind(elem_addr=self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].elem_addr,
                                                                         mod_app_idx=app_idx,
                                                                         mod_id=self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].sig_models[2].model_id)
        time.sleep(0.5)
        print("bt_mesh_cfg_mod_app_bind: rc=%d" % rc)

        # bind the Generic OnOff Server model
        rc = self.ble_mesh_node_erpc.client_api.bt_mesh_cfg_mod_app_bind(elem_addr=self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].elem_addr,
                                                                         mod_app_idx=app_idx,
                                                                         mod_id=self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].sig_models[3].model_id)
        time.sleep(0.5)
        print("bt_mesh_cfg_mod_app_bind: rc=%d" % rc)

        # bind the VND model
        rc = self.ble_mesh_node_erpc.client_api.bt_mesh_cfg_mod_app_bind(elem_addr=self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].elem_addr,
                                                                         mod_app_idx=app_idx,
                                                                         mod_id=self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].vnd_models[0].model_id,
                                                                         mod_cid=self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].vnd_models[0].model_cid)
        time.sleep(0.5)
        print("bt_mesh_cfg_mod_app_bind_vnd: rc=%d" % rc)

        # Add model subscription to the VND model
        rc = self.ble_mesh_node_erpc.client_api.bt_mesh_cfg_mod_sub_add(elem_addr=self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].elem_addr,
                                                                        sub_addr=0xC000,
                                                                        mod_id=self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].vnd_models[0].model_id,
                                                                        mod_cid=self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].vnd_models[0].model_cid)
        time.sleep(0.5)
        print("bt_mesh_cfg_mod_sub_add_vnd: rc=%d" % rc)

        print("")
        print("Call bt_mesh_is_provisioned")
        print("-----------------")

        rc = self.ble_mesh_node_erpc.client_api.bt_mesh_is_provisioned()
        time.sleep(0.5)
        print("bt_mesh_is_provisioned: rc=%d" % rc)
        print("")

    def mesh_prov_enable(self):
        """
            Call bt_mesh_prov_enable ADV bearers
        """

        print("")
        print("Call bt_mesh_prov_enable ADV and GATT bearers")
        print("--------------")

        self.ble_mesh_node_erpc.client_api.bt_mesh_prov_enable("ADV|GATT")

    def load(self):
        """
            Call settings_load
        """

        print("")
        print("Call settings_load to load provisioning information from NV")
        print("--------------")

        rc = self.ble_mesh_node_erpc.client_api.ble_mesh_api.settings_load_wrapper()
        print("settings_load: rc=%d" % rc)
        print("")

    def mesh_is_provisioned(self):
        """
            Call bt_mesh_is_provisioned
        """

        rc = self.ble_mesh_node_erpc.client_api.bt_mesh_is_provisioned()
        print("bt_mesh_is_provisioned: rc=%d" % rc)
        print("")

    def mesh_reset(self):
        """
            Call bt_mesh_reset
        """

        print("")
        print("Call bt_mesh_reset")
        print("--------------")

        self.ble_mesh_node_erpc.client_api.bt_mesh_reset()

    def find_model(self):
        """
            Find a SIG model in a specific element
        """

        model_index = self.ble_mesh_node_erpc.client_api.ble_mesh_api.bt_mesh_model_find_wrapper(elem_idx=0, id=0)
        print("bt_mesh_model_extend_wrapper(): model_index=%d" % model_index)
        print("")

    def find_vnd_model(self):
        """
            Find a vnd model in a specific element
        """

        model_index = self.ble_mesh_node_erpc.client_api.ble_mesh_api.bt_mesh_model_find_vnd_wrapper(elem_idx=0,
                                                                                                     company=0x000d, id=1)
        print("bt_mesh_model_extend_wrapper(): model_index=%d" % model_index)
        print("")

    def lpn_set(self):
        """
            Calls the bt_mesh_lpn_set
        """

        rc = self.ble_mesh_node_erpc.client_api.bt_mesh_lpn_set(True)
        print("bt_mesh_lpn_set(): rc=%d" % rc)
        print("")

    def lpn_poll(self):
        """
            Calls the bt_mesh_lpn_poll
        """

        rc = self.ble_mesh_node_erpc.client_api.bt_mesh_lpn_poll()
        print("lpn_poll(): rc=%d" % rc)
        print("")

    def identity_enable(self):
        """
            Call identity_enable
        """

        print("")
        print("IDENTITY ENABLE")
        rc = self.ble_mesh_node_erpc.client_api.ble_mesh_api.bt_mesh_proxy_identity_enable()
        print("bt_mesh_proxy_identity_enable: rc=%d" % rc)
        print("")

    def generate_fault(self):
        """
            Generate Fault on Helath server callback
            This is done for testing ONLY
        """

        print("")
        print("GENERATE FAULT")
        for model in self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].sig_models:
            if type(model) is BleMeshHealthSrvModel:
                model.add_fault(0x1)
                break
        print("")


class VndModelExampleMenu(MenuScreenCLI):

    def __init__(self, ble_mesh_node_erpc):
        """
            Set Vendor Model example Menu Screen Attributes
        :param ble_mesh_node_erpc: ble_mesh_node_erpc object
        """

        # Set the MenuScreen Attributes
        header = "--------Vendor Model Example Menu--------\n\n" \

        commands = {'1': {"DESC": "Mesh VND LED ON", "ACTION": "vnd_led_on()"},
                    '2': {"DESC": "Mesh VND LED OFF", "ACTION": "vnd_led_off()"},
                    '3': {"DESC": "Mesh VND Notify LED Status", "ACTION": "vnd_notify_led()"},
                    '4': {"DESC": "Mesh VND Get LED Status", "ACTION": "vnd_get_led()"},
                    '5': {"DESC": "Mesh VND Send Large MSG", "ACTION": "vnd_large_msg()"},
                    'h': {"DESC": "Print Menu Help", "ACTION": "print_help()"},
                    'b': {"DESC": "Back", "ACTION": "back()"}}

        MenuScreenCLI.__init__(self, header, commands)

        self.ble_mesh_node_erpc = ble_mesh_node_erpc

    def vnd_led_on(self):
        """
            Call VND Model Api led_on
        """

        print("")
        print("BTN ON")
        target_address = self.get_int_input("Enter the target address:")
        rc = self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].vnd_models[0].model_api.vnd_led_on_off(addr=target_address, on_off_state=1)
        print("bt_mesh_model_send: rc=%d" % rc)
        print("")

    def vnd_led_off(self):
        """
            Call VND Model Api led_off
        """

        print("")
        print("BTN OFF")
        target_address = self.get_int_input("Enter the target address:")
        rc = self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].vnd_models[0].model_api.vnd_led_on_off(addr=target_address, on_off_state=0)
        print("bt_mesh_model_send: rc=%d" % rc)
        print("")

    def vnd_notify_led(self):
        """
            Call VND Model Api notify_led_status
        """

        print("")
        print("Notify LED status")
        target_address = self.get_int_input("Enter the target address:")
        rc = self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].vnd_models[0].model_api.vnd_notify_led_status(addr=target_address)
        print("bt_mesh_model_send: rc=%d" % rc)
        print("")

    def vnd_get_led(self):
        """
            Call VND Model Api get_led_status
        """

        print("")
        print("Get LED status")
        target_address = self.get_int_input("Enter the target address:")
        rc = self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].vnd_models[0].model_api.vnd_get_led_status(addr=target_address)
        print("bt_mesh_model_send: rc=%d" % rc)
        print("")

    def vnd_large_msg(self):

        print("")
        print("Large MSG")
        target_address = self.get_int_input("Enter the target address:")
        msg_length = self.get_int_input("Enter the msg length:")
        rc = self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].vnd_models[0].model_api.vnd_large_msg(target_addr=target_address,
                                                                                                                     msg_size=msg_length)
        print("bt_mesh_model_send: rc=%d" % rc)
        print("")


class GenericOnOffClientMenu(MenuScreenCLI):

    def __init__(self, ble_mesh_node_erpc):
        """
            Set Generic OnOff Client Menu Screen Attributes
        :param ble_mesh_node_erpc: ble_mesh_node_erpc object
        """

        # Set the MenuScreen Attributes
        header = "--------Generic OnOff Client Model Menu--------\n\n" \

        commands = {'1': {"DESC": "Mesh Generic OnOff Client get msg", "ACTION": "mesh_gen_onoff_cli_onoff_get()"},
                    '2': {"DESC": "Mesh Generic OnOff Client set msg", "ACTION": "mesh_gen_onoff_cli_onoff_set()"},
                    '3': {"DESC": "Mesh Generic OnOff Client set msg with transition", "ACTION": "mesh_gen_onoff_cli_onoff_set_with_transition()"},
                    '4': {"DESC": "Mesh Generic OnOff Client set unack msg", "ACTION": "mesh_gen_onoff_cli_onoff_set_unack()"},
                    '5': {"DESC": "Mesh Generic OnOff Client set unack msg with transition", "ACTION": "mesh_gen_onoff_cli_onoff_set_unack_with_transition()"},
                    'h': {"DESC": "Print Menu Help", "ACTION": "print_help()"},
                    'b': {"DESC": "Back", "ACTION": "back()"}}

        MenuScreenCLI.__init__(self, header, commands)

        self.ble_mesh_node_erpc = ble_mesh_node_erpc

    def mesh_gen_onoff_cli_onoff_get(self):
        """
            Call get OnOff State
        """

        print("")
        print("Get Gen OnOff State")
        target_address = self.get_int_input("Enter the target address:")
        rc = self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].sig_models[2].model_api.onoff_get(addr=target_address)
        print("Sent onoff_get msg: rc=%d" % rc)
        print("")

    def mesh_gen_onoff_cli_onoff_set(self):
        """
            Call set OnOff State
        """

        print("")
        print("Set Gen OnOff State")
        target_address = self.get_int_input("Enter the target address:")
        onoff_state = self.get_int_input("Enter the Target OnOff State:")
        tid = self.get_int_input("Enter the transaction id:")
        rc = self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].sig_models[2].model_api.onoff_set(addr=target_address,
                                                                                                                 onoff=onoff_state, tid=tid)
        print("onoff_set: rc=%d" % rc)
        print("")

    def mesh_gen_onoff_cli_onoff_set_with_transition(self):
        """
            Call set OnOff State
        """

        print("")
        print("Set Gen OnOff State with Transition")
        target_address = self.get_int_input("Enter the target address:")
        onoff_state = self.get_int_input("Enter the Target OnOff State:")
        tid = self.get_int_input("Enter the transaction id:")
        transition_time = self.get_int_input("Enter the transition time:")
        delay = self.get_int_input("Enter the delay:")
        rc = self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].sig_models[2].model_api.onoff_set(addr=target_address,
                                                                                                                 onoff=onoff_state, tid=tid,
                                                                                                                 transition_time=transition_time, delay=delay)
        print("onoff_set: rc=%d" % rc)
        print("")

    def mesh_gen_onoff_cli_onoff_set_unack(self):
        """
            Call set OnOff State
        """

        print("")
        print("Set Gen OnOff Unacknowledged State")
        target_address = self.get_int_input("Enter the target address:")
        onoff_state = self.get_int_input("Enter the Target OnOff State:")
        tid = self.get_int_input("Enter the transaction id:")
        rc = self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].sig_models[2].model_api.onoff_set(addr=target_address, onoff=onoff_state,
                                                                                                                 tid=tid, ack=False)
        print("onoff_set: rc=%d" % rc)
        print("")

    def mesh_gen_onoff_cli_onoff_set_unack_with_transition(self):
        """
            Call set OnOff State
        """

        print("")
        print("Set Gen OnOff Unacknowledged State with Transition")
        target_address = self.get_int_input("Enter the target address:")
        onoff_state = self.get_int_input("Enter the Target OnOff State:")
        tid = self.get_int_input("Enter the transaction id:")
        transition_time = self.get_int_input("Enter the transition time:")
        delay = self.get_int_input("Enter the delay:")
        rc = self.ble_mesh_node_erpc.client_api.ble_mesh_comp_data.elements[0].sig_models[2].model_api.onoff_set(addr=target_address, onoff=onoff_state, tid=tid,
                                                                                                                 transition_time=transition_time, delay=delay, ack=False)
        print("onoff_set: rc=%d" % rc)
        print("")


class SelectDeviceMenu(MenuScreenCLI):

    def __init__(self):
        """
            Set Select Device Menu Screen Attributes
        """

        # Set the MenuScreen Attributes
        header = "--------BLE Mesh eRPC Node--------\n"

        commands = {'q': {"DESC": "Exit", "ACTION": "quit()"}}

        MenuScreenCLI.__init__(self, header, commands)
        self._erpc_main_menu_screen = None

    def connect_to_device(self, device_port):
        """
            Start the eRPC Main Menu Screen with the input device serial port name
        :param device_port: device serial port name
        """

        # Init and start the eRPC Main Menu Screen
        if self._erpc_main_menu_screen is None:
            self._erpc_main_menu_screen = ErpcMainMenu(device_port)
            self._erpc_main_menu_screen.set_upper_menu(self)
            self._erpc_main_menu_screen.run_menu()
        else:
            print("Already connected to device in port {}".format(self._erpc_main_menu_screen.get_device_port_number()))

    def wait_for_input(self):
        """
            Wait for device serial port name input from user
        """

        while True:
            ret_value = input("Please Enter Device serial port name: ")
            if ret_value == 'q':
                self.execute_command(ret_value)
            else:
                self.connect_to_device(ret_value)


###########################################################################################################

if __name__ == "__main__":
    # Start the Example with SelectDevice CLI Menu
    erpc_app = SelectDeviceMenu()
    erpc_app.run_menu()
