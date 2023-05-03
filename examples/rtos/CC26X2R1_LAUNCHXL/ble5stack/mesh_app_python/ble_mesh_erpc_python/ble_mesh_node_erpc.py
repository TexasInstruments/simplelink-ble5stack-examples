#!/usr/bin/python

import threading

# import the EmbeddedRPC framework
import erpc

# import The eRPC APIs, CBKs and all the relevant server and client modules that are used in this application
from .lprf_erpc import lprf_api
from .lprf_erpc import lprf_cbk

# import the BLE Mesh Composition Data classes
from .ble_mesh_composition_data import BleMeshCompData, BleMeshCfgCliModel, BleMeshCfgSrvModel, BleMeshHealthSrvModel, BleMeshModel

################################### BLE Mesh Provision Data Objects #################################################


class BleMeshProvOobData:
    """ BLE Mesh Provision OOB Data class"""

    def __init__(self, oob_info):
        """
            Init the BLE Mesh Provision OOB Data attributes
        :param oob_info: lprf_api.common.bt_mesh_prov_oob_info
        """

        self.oob_info = oob_info

        # Set No OOB cfg
        self.static_val = None
        self.output_size = 0
        self.output_actions = 0
        self.input_size = 0
        self.input_actions = 0

        # Set OOB actions attributes
        self.output_number = None
        self.output_string = None

    def set_no_oob(self):
        """
            Set No OOB configuration parameters
        """

        self.static_val = None
        self.output_size = 0
        self.output_actions = 0
        self.input_size = 0
        self.input_actions = 0

    def set_static_oob(self, static_val):
        """
            Set Static OOB configuration parameters
        :param static_val: list of values
        """

        self.static_val = static_val
        self.output_size = 0
        self.output_actions = 0
        self.input_size = 0
        self.input_actions = 0

    def set_output_oob(self, output_size, output_actions):
        """
            Set Output OOB configuration parameters
        :param output_size: maximum size of output oob supported 0x01 -> 0x08
        :param output_actions: supported output oob actions lprf_api.common.bt_mesh_output_action
        """

        self.static_val = None
        self.output_size = output_size
        self.output_actions = output_actions
        self.input_size = 0
        self.input_actions = 0

    def set_input_oob(self, input_size, input_actions):
        """
            Set Input OOB configuration parameters
        :param input_size: maximum size of input oob supported 0x01 -> 0x08
        :param input_actions: supported input oob actions lprf_api.common.bt_mesh_input_action
        """

        self.static_val = None
        self.output_size = 0
        self.output_actions = 0
        self.input_size = input_size
        self.input_actions = input_actions


class BleMeshProvData:
    """ BLE Mesh Provision Data class"""

    def __init__(self, uuid, uri=''):
        """
            Init the BLE Mesh Provision Data attributes
        :param uuid: universally unique identifier
        :param uri: uniform resource indicator
        """

        self.uuid = uuid
        self.uri = uri
        self.addr = 0

        self.dev_key = list()
        self.net_key = list()
        self.net_idx = 0

        self.app_key = list()

        self.flags = 0
        self.iv_index = 0

        self.prov_oob_data = None

    def reset_prov(self):
        """ Reset the Provisionning data attributes"""

        self.addr = 0
        self.dev_key = list()
        self.net_key = list()
        self.net_idx = 0

        self.app_key = list()
        self.flags = 0
        self.iv_index = 0

################################### BLE Mesh eRPC Node Objects #################################################


class BleMeshCbkHandler:
    """ BLE Mesh CBK Handler class"""

    def __init__(self):
        """
            Init the BLE Mesh Cbk Handler attributes
        """

        self.client_api = None

    @staticmethod
    def print_msg(msg):
        """ BLE Mesh CBK Handler messages print function """

        print("\n{}\nMeshCBK MSG: {}\n{}".format('~'*50, msg, '~'*50))

    @staticmethod
    def _hb_sub_to_str(sub=None):
        """
            Helper method to parse field of bt_mesh_hb_sub class
        :param sub: Object of type bt_mesh_hb_sub
        :return: A string with printed format of all sub parameters
        """

        if type(sub) is not lprf_cbk.common.bt_mesh_hb_sub:
            return None

        ret_val = ""
        ret_val += "period = {}, ".format(sub.period)  # uint32
        ret_val += "remaining = {}, ".format(sub.remaining)  # uint32
        ret_val += "src = {}, ".format(sub.src)  # uint16
        ret_val += "dst = {}, ".format(sub.dst)  # uint16
        ret_val += "count = {}, ".format(sub.count)  # uint16
        ret_val += "min_hops = {}, ".format(sub.min_hops)  # uint8
        ret_val += "max_hops = {}".format(sub.max_hops)  # uint8

        return ret_val

    def set_client_api(self, client_api):
        """
            Set the BLE Mesh Client Api object
        :param client_api: BLE Mesh Client Api object
        """

        self.client_api = client_api

    ######### BLE Mesh Cbk #########
    def output_number_cb(self, act, num):
        self.print_msg("output_number_cb cbk: act={}, num={}".format(act, num))
        self.client_api.ble_mesh_prov_data.prov_oob_data.output_number = num

    def output_string_cb(self, str):
        self.print_msg("output_string_cb cbk: str={}".format(str))
        self.client_api.ble_mesh_prov_data.prov_oob_data.output_string = str

    def input_cb(self, act, size):
        self.print_msg("input_cb cbk: act={}, size={}".format(act, size))

    def input_complete_cb(self):
        self.print_msg("input_complete_cb cbk")

    def unprovisioned_beacon_cb(self, uuid, oob_info, uri_hash):
        self.print_msg("unprovisioned_beacon_cb cbk: uuid={}, oob_info={}, uri_hash={}".format("".join(list("{:02x}".format(byte) for byte in uuid)),
                                                                                               oob_info,
                                                                                               uri_hash))

    def link_open_cb(self, bearer):
        self.print_msg("link_open_cb cbk: bearer={}".format(bearer))

    def link_close_cb(self, bearer):
        self.print_msg("link_close_cb cbk: bearer={}".format(bearer))

    def node_added_cb(self, net_idx, uuid, addr, num_elem):
        self.print_msg("node_added_cb cbk: net_idx={}, uuid={}, addr={:04x}, num_elem={}".format(net_idx, "".join(list("{:02x}".format(byte) for byte in uuid)), addr, num_elem))

    def complete_cb(self, net_idx, addr):
        self.print_msg("complete_cb cbk: net_idx={}, addr={:04x}".format(net_idx, addr))

        # Set ble_mesh_prov_data attributes
        self.client_api.ble_mesh_prov_data.addr = addr
        self.client_api.ble_mesh_prov_data.net_idx = net_idx

        # Set elements address
        for element in self.client_api.ble_mesh_comp_data.elements:
            element.elem_addr = addr + element.elem_idx
            self.print_msg("complete_cb cbk: elem_idx={}, elem_addr={:04x}".format(element.elem_idx, element.elem_addr))

    def reset_prov_cb(self):
        self.print_msg("reset_prov_cb cbk")

        # Reset ble_mesh_prov_data
        if self.client_api.ble_mesh_prov_data is not None:
            self.client_api.ble_mesh_prov_data.reset_prov()

    def hb_recv_cb(self, sub, hops, feat):
        sub_format_str = self._hb_sub_to_str(sub)
        if sub_format_str is None:
            self.print_msg("Error in parsing bt_mesh_hb_sub class (at hb_recv_cb)")
            return

        self.print_msg("hb_recv_cb: hops = {}, feat = {}".format(hops, feat))
        self.print_msg("sub: {}".format(sub_format_str))

    def hb_sub_end_cb(self, sub):
        sub_format_str = self._hb_sub_to_str(sub)

        if sub_format_str is None:
            self.print_msg("Error in parsing bt_mesh_hb_sub class (at hb_recv_cb)")
            return

        self.print_msg("hb_sub_end:")
        self.print_msg("sub: {}".format(sub_format_str))

    def lpn_friendship_established_cb(self, net_idx, friend_addr, queue_size, recv_window):
        self.print_msg("lpn_friendship_established: net_idx = {}, friend_addr = 0x{:04x},"
                       " queue_size = {}, recv_window = {}".format(net_idx, friend_addr, queue_size, recv_window))

    def lpn_friendship_terminated_cb(self, net_idx, friend_addr):
        self.print_msg("lpn_friendship_terminated: net_idx = {}, friend_addr = 0x{:04x}".format(net_idx, friend_addr))

    def lpn_polled_cb(self, net_idx, friend_addr, retry):
        self.print_msg("lpn_polled: net_idx = {}, friend_addr = 0x{:04x}, retry = {}".format(net_idx, friend_addr, retry))

    def friend_friendship_established_cb(self, net_idx, lpn_addr, recv_delay, polltimeout):
        self.print_msg("friend_friendship_established: net_idx = {}, lpn_addr = 0x{:04x},"
                       " recv_delay = {}, polltimeout = {}".format(net_idx, lpn_addr, recv_delay, polltimeout))

    def friend_friendship_terminated_cb(self, net_idx, lpn_addr):
        self.print_msg("friend_friendship_terminated: net_idx = {}, lpn_addr = {}".format(net_idx, lpn_addr))

    def appkey_evt_cb(self, app_idx, net_idx, evt):
        evt_to_str = [
            "BT_MESH_KEY_ADDED",
            "BT_MESH_KEY_DELETED",
            "BT_MESH_KEY_UPDATED",
            "BT_MESH_KEY_SWAPPED",
            "BT_MESH_KEY_REVOKED"]

        evt_str = evt_to_str[evt] if evt < len(evt_to_str) else 'NOT_SUPPORTED_EVENT'
        self.print_msg("appkey_evt_cb: app_idx = {}, net_idx = {}, evt = {}".format(app_idx, net_idx, evt_str))

    ######### BLE Mesh Model Cbk #########
    def func_cb(self, opcode, elem_idx, is_vnd, model_index, ctx, buf):
        """
            Call to the relevant opcode function from model_cbk object
        :param opcode: cbk opcode
        :param elem_idx: cbk element index
        :param is_vnd: cbk is vnd model
        :param model_index: cbk model index
        :param ctx: cbk msg ctx
        :param buf: cbk msg buf
        """

        self.print_msg("func_cb cbk: elem_idx={}, is_vnd={}, model_index={}, opcode={:04x}".format(elem_idx, is_vnd, model_index, opcode))
        if is_vnd == 1:
            self.client_api.ble_mesh_comp_data.elements[elem_idx].vnd_models[model_index].model_cbk.execute_opcode_cbk(opcode, elem_idx, is_vnd, model_index, ctx, buf)
        else:
            self.client_api.ble_mesh_comp_data.elements[elem_idx].sig_models[model_index].model_cbk.execute_opcode_cbk(opcode, elem_idx, is_vnd, model_index, ctx, buf)

    def start_cb(self, elem_idx, is_vnd, model_index):
        """
            Call to the relevant model_cbk.start_model_cb function
        :param elem_idx: cbk element index
        :param is_vnd: cbk is vnd model
        :param model_index: cbk model index
        """

        self.print_msg("start_cb cbk: elem_idx={}, is_vnd={}, model_index={}".format(elem_idx, is_vnd, model_index))
        if is_vnd == 1:
            self.client_api.ble_mesh_comp_data.elements[elem_idx].vnd_models[model_index].model_cbk.start_model_cb(elem_idx, is_vnd, model_index)
        else:
            self.client_api.ble_mesh_comp_data.elements[elem_idx].sig_models[model_index].model_cbk.start_model_cb(elem_idx, is_vnd, model_index)

    def update_cb(self, elem_idx, is_vnd, model_index):
        """
            Call to the relevant model_cbk.update_cb function
        :param elem_idx: cbk element index
        :param is_vnd: cbk is vnd model
        :param model_index: cbk model index
        """

        self.print_msg("update_cb cbk: elem_idx={}, is_vnd={}, model_index={}".format(elem_idx, is_vnd, model_index))
        if is_vnd == 1:
            self.client_api.ble_mesh_comp_data.elements[elem_idx].vnd_models[model_index].model_cbk.update_model_cb(elem_idx, is_vnd, model_index)
        else:
            self.client_api.ble_mesh_comp_data.elements[elem_idx].sig_models[model_index].model_cbk.update_model_cb(elem_idx, is_vnd, model_index)

    def init_cb(self, elem_idx, is_vnd, model_index):
        """
            Call to the relevant model_cbk.init_cb function
        :param elem_idx: cbk element index
        :param is_vnd: cbk is vnd model
        :param model_index: cbk model index
        """

        self.print_msg("init_cb cbk: elem_idx={}, is_vnd={}, model_index={}".format(elem_idx, is_vnd, model_index))
        if is_vnd == 1:
            self.client_api.ble_mesh_comp_data.elements[elem_idx].vnd_models[model_index].model_cbk.init_model_cb(elem_idx, is_vnd, model_index)
        else:
            self.client_api.ble_mesh_comp_data.elements[elem_idx].sig_models[model_index].model_cbk.init_model_cb(elem_idx, is_vnd, model_index)

    def reset_cb(self, elem_idx, is_vnd, model_index):
        """
            Call to the relevant model_cbk.reset_cb function
        :param elem_idx: cbk element index
        :param is_vnd: cbk is vnd model
        :param model_index: cbk model index
        """

        self.print_msg("reset_cb cbk: elem_idx={}, is_vnd={}, model_index={}".format(elem_idx, is_vnd, model_index))
        if is_vnd == 1:
            self.client_api.ble_mesh_comp_data.elements[elem_idx].vnd_models[model_index].model_cbk.reset_model_cb(elem_idx, is_vnd, model_index)
        else:
            self.client_api.ble_mesh_comp_data.elements[elem_idx].sig_models[model_index].model_cbk.reset_model_cb(elem_idx, is_vnd, model_index)

    def settings_set_cb(self, elem_idx, is_vnd, model_index, name, data):
        """
            Call to the relevant model_cbk.settings_set_cb function
        :param elem_idx: cbk element index
        :param is_vnd: cbk is vnd model
        :param model_index: cbk model index
        :param name: name of the data read from NV
        :param data: the data read from NV
        """

        self.print_msg("settings_set_cb cbk: elem_idx={}, is_vnd={}, model_index={}".format(elem_idx, is_vnd, model_index))
        if is_vnd == 1:
            self.client_api.ble_mesh_comp_data.elements[elem_idx].vnd_models[model_index].model_cbk.settings_set_model_cb(elem_idx, is_vnd, model_index, name, data)
        else:
            self.client_api.ble_mesh_comp_data.elements[elem_idx].sig_models[model_index].model_cbk.settings_set_model_cb(elem_idx, is_vnd, model_index, name, data)

    ######### BLE Mesh Health Server Cbk #########
    def fault_get_cur_cb(self, elem_idx, model_index, test_id, company_id, faults):
        """
            Call to the relevant health_srv_cbk.fault_get_cur_cb function
        :param elem_idx:    cbk element index
        :param model_index: cbk model index
        :param test_id:     Test ID response buffer.
        :param company_id:  Company ID response buffer.
        :param faults:      List to fill with current faults.
        """

        self.print_msg("fault_get_cur_cb cbk: elem_idx={}, model_index={}, test_id={}, company_id={}, \
                        faults={}".format(elem_idx, model_index, test_id.value, company_id.value, faults.value))
        return self.client_api.ble_mesh_comp_data.elements[elem_idx].sig_models[model_index].health_srv_cbk.fault_get_cur_cb(elem_idx=elem_idx,
                                                                                                                             model_index=model_index,
                                                                                                                             test_id=test_id,
                                                                                                                             company_id=company_id,
                                                                                                                             faults=faults)

    def fault_get_reg_cb(self, elem_idx, model_index, company_id, test_id, faults):
        """
            Call to the relevant health_srv_cbk.fault_get_reg_cb function
        :param elem_idx: cbk element index
        :param model_index: cbk model index
        :param company_id: cbk model cid
        :param test_id:     Test ID response buffer.
        :param faults:      List to fill with registered faults.
        """

        self.print_msg("fault_get_reg_cb cbk: elem_idx={}, model_index={}, company_id={},\
                        test_id={}, faults={}".format(elem_idx, model_index, company_id, test_id.value, faults.value))
        return self.client_api.ble_mesh_comp_data.elements[elem_idx].sig_models[model_index].health_srv_cbk.fault_get_reg_cb(elem_idx=elem_idx,
                                                                                                                             model_index=model_index,
                                                                                                                             company_id=company_id,
                                                                                                                             test_id=test_id,
                                                                                                                             faults=faults)

    def fault_clear_cb(self, elem_idx, model_index, company_id):
        """
            Call to the relevant health_srv_cbk.fault_clear_cb function
        :param elem_idx: cbk element index
        :param model_index: cbk model index
        :param company_id: cbk company id
        """

        self.print_msg("fault_clear_cb cbk: elem_idx={}, model_index={}, company_id={:04x}".format(elem_idx, model_index, company_id))
        return self.client_api.ble_mesh_comp_data.elements[elem_idx].sig_models[model_index].health_srv_cbk.fault_clear_cb(elem_idx, model_index, company_id)

    def fault_test_cb(self, elem_idx, model_index, test_id, company_id):
        """
            Call to the relevant health_srv_cbk.fault_test_cb function
        :param elem_idx: cbk element index
        :param model_index: cbk model index
        :param test_id:
        :param company_id: cbk company id
        """

        self.print_msg("fault_test_cb cbk: elem_idx={}, model_index={}, test_id={:04x}, company_id={:04x}".format(elem_idx, model_index, test_id, company_id))
        return self.client_api.ble_mesh_comp_data.elements[elem_idx].sig_models[model_index].health_srv_cbk.fault_test_cb(elem_idx, model_index, test_id, company_id)

    def attn_on_cb(self, elem_idx, model_index):
        """
            Call to the relevant health_srv_cbk.attn_on_cb function
        :param elem_idx: cbk element index
        :param model_index: cbk model index
        """

        self.print_msg("attn_on_cb cbk: elem_idx={}, model_index={}".format(elem_idx, model_index))
        self.client_api.ble_mesh_comp_data.elements[elem_idx].sig_models[model_index].health_srv_cbk.attn_on_cb(elem_idx, model_index)

    def attn_off_cb(self, elem_idx, model_index):
        """
            Call to the relevant health_srv_cbk.attn_off_cb function
        :param elem_idx: cbk element index
        :param model_index: cbk model index
        """

        self.print_msg("attn_off_cb cbk: elem_idx={}, model_index={}".format(elem_idx, model_index))
        self.client_api.ble_mesh_comp_data.elements[elem_idx].sig_models[model_index].health_srv_cbk.attn_off_cb(elem_idx, model_index)


class BleMeshClientApi:
    """ BLE Mesh Client Api class"""

    def __init__(self, transport, arbitrator, codec, cbk_handler):
        """
            Init the BLE Mesh Client Api attributes
        :param transport: erpc transport
        :param arbitrator: erpc arbitrator
        :param codec: erpc codec
        :param cbk_handler: ble mesh cbk handler object
        """

        self.transport = transport
        self.arbitrator = arbitrator
        self.codec = codec

        # Set callback Handler
        self.cbk_handler = cbk_handler

        # Set the LPRF_API client
        self.client_manager = erpc.client.ClientManager(self.transport, self.codec)
        self.client_manager.arbitrator = self.arbitrator
        self.ble_mesh_api = lprf_api.client.BLEmesh_apiClient(self.client_manager)

        # Init the ble mesh composition data
        self.ble_mesh_comp_data = None

        # Init the ble mesh prov oob data
        self.ble_mesh_prov_data = None

    def __delete__(self):
        """
            Destruct the BLE Mesh Composition Data object attribute
        """

        if self.ble_mesh_comp_data is not None:
            self.ble_mesh_comp_data.__delete__()

    def bt_mesh_settings_load(self):
        """
            Load Mesh Data from NV
        :return rc: return code from settings_load_wrapper
        """

        rc = self.ble_mesh_api.settings_load_wrapper()
        return rc

    def bt_mesh_init(self):
        """
            Call mesh_init
        :return rc: return code from mesh_init
        """

        rc = self.ble_mesh_api.mesh_init()
        return rc

    def bt_mesh_reset(self):
        """
            Call bt_mesh_reset
        """

        # Call bt_mesh_reset
        self.ble_mesh_api.bt_mesh_reset()

    def bt_mesh_suspend(self):
        """
            Call bt_mesh_suspend
        :return rc: return code from bt_mesh_suspend
        """

        rc = self.ble_mesh_api.bt_mesh_suspend()
        return rc

    def bt_mesh_resume(self):
        """
            Call bt_mesh_resume
        :return rc: return code from bt_mesh_resume
        """

        rc = self.ble_mesh_api.bt_mesh_resume()
        return rc

    def bt_mesh_input_string(self, str_input):
        """
            Call bt_mesh_input_string
        :param str_input: input string value
        :return: rc: return code from bt_mesh_input_string
        """

        rc = self.ble_mesh_api.bt_mesh_input_string(str_input)
        return rc

    def bt_mesh_input_number(self, num):
        """
            Call bt_mesh_input_number
        :param num: input number value
        :return: rc: return code from bt_mesh_input_number
        """

        rc = self.ble_mesh_api.bt_mesh_input_number(num)
        return rc

    def bt_mesh_prov_enable(self, bearers):
        """
            Call bt_mesh_prov_enable
        :param bearers: ADV/GATT/ADV|GATT srt
        :return: rc: return code from bt_mesh_prov_enable_wrapper
        """

        # Call bt_mesh_prov_enable with the relevant bearers
        if bearers == "ADV":
            rc = self.ble_mesh_api.bt_mesh_prov_enable_wrapper(lprf_api.common.bt_mesh_prov_bearer.BT_MESH_PROV_ADV_ERPC)
        elif bearers == "GATT":
            rc = self.ble_mesh_api.bt_mesh_prov_enable_wrapper(lprf_api.common.bt_mesh_prov_bearer.BT_MESH_PROV_GATT_ERPC)
        elif bearers == "ADV|GATT":
            rc = self.ble_mesh_api.bt_mesh_prov_enable_wrapper(lprf_api.common.bt_mesh_prov_bearer.BT_MESH_PROV_ADV_ERPC | lprf_api.common.bt_mesh_prov_bearer.BT_MESH_PROV_GATT_ERPC)
        else:
            rc = -1

        return rc

    def bt_mesh_prov_disable(self, bearers):
        """
            Call bt_mesh_prov_disable
        :param bearers: ADV/GATT/ADV|GATT srt
        :return: rc: return code from bt_mesh_prov_disable_wrapper
        """

        # Call bt_mesh_prov_disable with the relevant bearers
        if bearers == "ADV":
            rc = self.ble_mesh_api.bt_mesh_prov_disable_wrapper(lprf_api.common.bt_mesh_prov_bearer.BT_MESH_PROV_ADV_ERPC)
        elif bearers == "GATT":
            rc = self.ble_mesh_api.bt_mesh_prov_disable_wrapper(lprf_api.common.bt_mesh_prov_bearer.BT_MESH_PROV_GATT_ERPC)
        elif bearers == "ADV|GATT":
            rc = self.ble_mesh_api.bt_mesh_prov_disable_wrapper(lprf_api.common.bt_mesh_prov_bearer.BT_MESH_PROV_ADV_ERPC | lprf_api.common.bt_mesh_prov_bearer.BT_MESH_PROV_GATT_ERPC)
        else:
            rc = -1

        return rc

    def bt_mesh_provision(self, addr, net_key, net_idx, dev_key, flags=0, iv_index=0):
        """
            Call bt_mesh_provision
        :param addr: own address
        :param net_key: network key
        :param net_idx: network index
        :param dev_key: device key
        :param flags:
        :param iv_index:
        :return: rc: return code from bt_mesh_provision
        """

        # Init ble_mesh_prov_data attributes
        self.ble_mesh_prov_data.addr = addr
        self.ble_mesh_prov_data.net_key = net_key
        self.ble_mesh_prov_data.net_idx = net_idx
        self.ble_mesh_prov_data.dev_key = dev_key
        self.ble_mesh_prov_data.flags = flags
        self.ble_mesh_prov_data.iv_index = iv_index

        # Call bt_mesh_provision
        rc = self.ble_mesh_api.bt_mesh_provision(net_key=net_key, net_idx=net_idx,
                                                 flags=flags, iv_index=iv_index,
                                                 addr=addr, dev_key=dev_key)
        return rc

    def bt_mesh_is_provisioned(self):
        """
            Call bt_mesh_is_provisioned
        :return: rc: return code from bt_mesh_is_provisioned
        """

        rc = self.ble_mesh_api.bt_mesh_is_provisioned()
        return rc

    def bt_mesh_iv_update_test(self, enable):
        """
            Set IV Update TestMode Enable/Disable(for QUAL tests)
        :param enable: True/False
        :return: rc: return code from bt_mesh_iv_update_test
        """

        self.ble_mesh_api.bt_mesh_iv_update_test(enable)

    def bt_mesh_iv_update(self):
        """
        :return: rc: return code from bt_mesh_iv_update
        """

        rc = self.ble_mesh_api.bt_mesh_iv_update()
        return rc

    def bt_mesh_init_prov(self, uuid, uri='', prov_oob_data=None):
        """
            Init the prov raw data and call bt_mesh_init_prov_raw_init
        :param uuid: universally unique identifier
        :param uri: uniform resource indicator
        :param prov_oob_data: BLE Mesh Provision OOB Data object, if not set default is No OOB configuration
        :return: rc: return code from bt_mesh_init_prov_raw_init
        """

        if self.ble_mesh_prov_data is not None:
            del self.ble_mesh_prov_data
            self.ble_mesh_prov_data = None

        # Init the BLE Mesh Provision Data object
        self.ble_mesh_prov_data = BleMeshProvData(uuid=uuid, uri=uri)

        # if no prov_oob_data input set default prov oob data to no oob configuration
        if prov_oob_data is None:
            self.ble_mesh_prov_data.prov_oob_data = BleMeshProvOobData(oob_info=lprf_api.common.bt_mesh_prov_oob_info.BT_MESH_PROV_OOB_OTHER_ERPC)
        else:
            if type(prov_oob_data) is not BleMeshProvOobData:
                raise TypeError("No BleMeshProvOobData object")

            self.ble_mesh_prov_data.prov_oob_data = prov_oob_data

        # Create and init prov raw data
        prov_raw = lprf_api.common.bt_mesh_prov_raw(uuid=uuid, uri=uri, oob_info=self.ble_mesh_prov_data.prov_oob_data.oob_info,
                                                    static_val=self.ble_mesh_prov_data.prov_oob_data.static_val,
                                                    output_size=self.ble_mesh_prov_data.prov_oob_data.output_size,
                                                    output_actions=self.ble_mesh_prov_data.prov_oob_data.output_actions,
                                                    input_size=self.ble_mesh_prov_data.prov_oob_data.input_size,
                                                    input_actions=self.ble_mesh_prov_data.prov_oob_data.input_actions,
                                                    output_number=self.cbk_handler.output_number_cb,
                                                    output_string=self.cbk_handler.output_string_cb,
                                                    input=self.cbk_handler.input_cb,
                                                    input_complete=self.cbk_handler.input_complete_cb,
                                                    unprovisioned_beacon=None,
                                                    link_open=self.cbk_handler.link_open_cb,
                                                    link_close=self.cbk_handler.link_close_cb,
                                                    complete=self.cbk_handler.complete_cb,
                                                    node_added=self.cbk_handler.node_added_cb,
                                                    reset=self.cbk_handler.reset_prov_cb)

        # Call bt_mesh_init_prov_raw_init
        rc = self.ble_mesh_api.bt_mesh_init_prov_raw_init(prov_raw)
        return rc

    def bt_mesh_init_comp(self, cid, pid, vid, elem_count):
        """
            Init the ble mesh composition data and call bt_mesh_init_comp_raw_init
        :param cid: company identifier
        :param pid: product identifier
        :param vid: version identifier
        :param elem_count: the number of elements in this Node
        :return: rc: return code from bt_mesh_init_comp_raw_init
        """

        if self.ble_mesh_comp_data is not None:
            self.ble_mesh_comp_data.__delete__()
            self.ble_mesh_comp_data = None

        self.ble_mesh_comp_data = BleMeshCompData(cid=cid, pid=pid, vid=vid, elem_count=elem_count)
        rc = self.ble_mesh_api.bt_mesh_init_comp_raw_init(comp_raw=self.ble_mesh_comp_data.comp_raw)
        return rc

    def bt_mesh_init_primary_element(self, addr, loc, model_count, vnd_model_count=0):
        """
            Init the primary element raw
        :param addr: element address
        :param loc: element location Descriptor
        :param model_count: the number of SIG models in this element
        :param vnd_model_count: the number of vendor models in this element.
        :return: rc: return code from bt_mesh_init_elem_raw_init
        """

        elem_index = self.ble_mesh_comp_data.set_primary_element(addr=addr, loc=loc, model_count=model_count, vnd_model_count=vnd_model_count)
        rc = self.ble_mesh_api.bt_mesh_init_elem_raw_init(elem_index=elem_index, elem_raw=self.ble_mesh_comp_data.elements[elem_index].elem_raw)
        return rc

    def bt_mesh_init_secondary_element(self, addr, loc, model_count, vnd_model_count=0):
        """
            Init a secondary element raw
        :param addr: element address
        :param loc: element location Descriptor
        :param model_count: the number of SIG models in this element
        :param vnd_model_count: the number of vendor models in this element.
        :return: rc, elem_index: return code from bt_mesh_init_elem_raw_init, added element index
        """

        elem_index = self.ble_mesh_comp_data.add_secondary_element(addr=addr, loc=loc, model_count=model_count, vnd_model_count=vnd_model_count)
        rc = self.ble_mesh_api.bt_mesh_init_elem_raw_init(elem_index=elem_index, elem_raw=self.ble_mesh_comp_data.elements[elem_index].elem_raw)
        return rc, elem_index

    def bt_mesh_init_cfg_cli(self, elem_index):
        """
            Init the ble mesh configuration server model at the primary element and call bt_mesh_cfg_srv_raw_init
        :param elem_index: element index to add the health server model
        :return: rc: return code from bt_mesh_cfg_cli_raw_init
        """

        if self.ble_mesh_comp_data is not None:
            cfg_cli_model = BleMeshCfgCliModel(elem_idx=elem_index)

            # Set the configuration client model data at relevant element
            if len(self.ble_mesh_comp_data.elements) > elem_index:
                model_index = self.ble_mesh_comp_data.elements[elem_index].add_model(lprf_api.common.SIG_MODEL, cfg_cli_model)
                self.ble_mesh_comp_data.elements[elem_index].sig_models[model_index].model_index = model_index
                rc = self.ble_mesh_api.bt_mesh_cfg_cli_raw_init(elem_index=elem_index, model_index=model_index)
                return rc
            else:
                return -1
        else:
            return -1

    def bt_mesh_init_cfg_srv(self):
        """
            Init the ble mesh configuration server model at the primary element and call bt_mesh_cfg_srv_raw_init
        :return: rc: return code from bt_mesh_cfg_srv_raw_init
        """

        if self.ble_mesh_comp_data is not None:
            cfg_srv_model = BleMeshCfgSrvModel()

            # Set the configuration server model data at primary element (index 0)
            if len(self.ble_mesh_comp_data.elements) > 0:
                model_index = self.ble_mesh_comp_data.elements[cfg_srv_model.elem_idx].add_model(lprf_api.common.SIG_MODEL, cfg_srv_model)
                self.ble_mesh_comp_data.elements[cfg_srv_model.elem_idx].sig_models[model_index].model_index = model_index
                rc = self.ble_mesh_api.bt_mesh_cfg_srv_raw_init(elem_index=cfg_srv_model.elem_idx)
                return rc
            else:
                return -1
        else:
            return -1

    def bt_mesh_init_health_srv(self, elem_index, max_faults=0, cbk_handler=None):
        """
            Init the ble mesh health server model and call bt_mesh_health_srv_raw_init
        :param elem_index: element index to add the health server model
        :param max_faults: health server model maximum faults
        :param cbk_handler: BLE Mesh HealthSrvCbk object
        :return: rc: return code from bt_mesh_health_srv_raw_init
        """

        if self.ble_mesh_comp_data is not None:
            health_srv = BleMeshHealthSrvModel(elem_idx=elem_index, cbk_handler=cbk_handler,
                                               cid=self.ble_mesh_comp_data.cid,
                                               health_srv_cb_raw=lprf_api.common.bt_mesh_health_srv_cb_raw(fault_get_cur=self.cbk_handler.fault_get_cur_cb,
                                                                                                           fault_get_reg=self.cbk_handler.fault_get_reg_cb,
                                                                                                           fault_clear=self.cbk_handler.fault_clear_cb,
                                                                                                           fault_test=self.cbk_handler.fault_test_cb,
                                                                                                           attn_on=self.cbk_handler.attn_on_cb,
                                                                                                           attn_off=self.cbk_handler.attn_off_cb))

            # Set the health server model data at relevant element
            if len(self.ble_mesh_comp_data.elements) > elem_index:
                model_index = self.ble_mesh_comp_data.elements[elem_index].add_model(lprf_api.common.SIG_MODEL, health_srv)
                self.ble_mesh_comp_data.elements[elem_index].sig_models[model_index].model_index = model_index
                rc = self.ble_mesh_api.bt_mesh_health_srv_raw_init(elem_index=elem_index,
                                                                   model_index=model_index,
                                                                   health_srv=health_srv.health_srv_raw,
                                                                   max_faults=max_faults)
                return rc
            else:
                return -1
        else:
            return -1

    def bt_mesh_init_sig_model(self, sig_model):
        """
            Init the ble mesh sig model and call bt_mesh_init_model_raw_init
        :param: sig_model: sig model data object (instance of ble_mesh_composition_data.BleMeshModelData)
        :return: rc, model_index: return code from bt_mesh_init_model_raw_init, model index
        """

        if self.ble_mesh_comp_data is not None:
            comp_sig_model = BleMeshModel(client_api=self,
                                          model_api=sig_model.model_api,
                                          model_cbk=sig_model.model_cbk,
                                          model_pub=sig_model.model_pub)

            comp_sig_model.set_model_opcodes(sig_model.opcodes)

            # Set the sig model raw data
            comp_sig_model.set_sig_model_raw(sid=sig_model.model_id, user_data=sig_model.user_data)

            # Set the sig model data at relevant element
            if len(self.ble_mesh_comp_data.elements) > sig_model.elem_index:
                model_index = self.ble_mesh_comp_data.elements[sig_model.elem_index].add_model(model_type=lprf_api.common.SIG_MODEL, ble_mesh_model=comp_sig_model)
                rc = self.ble_mesh_api.bt_mesh_init_model_raw_init(elem_index=sig_model.elem_index,
                                                                   model_index=model_index,
                                                                   model_raw=self.ble_mesh_comp_data.elements[sig_model.elem_index].sig_models[model_index].model_raw,
                                                                   op_raw=self.ble_mesh_comp_data.elements[sig_model.elem_index].sig_models[model_index].model_opcodes)

                return rc, model_index
            else:
                return -1, -1
        else:
            return -1, -1

    def bt_mesh_init_vnd_model(self, vnd_model):
        """
            Init the ble mesh vnd model and call bt_mesh_init_model_raw_init
        :param: vnd_model: vnd model data object (instance of ble_mesh_composition_data.BleMeshModelData)
        :return: rc, model_index: return code from bt_mesh_init_model_raw_init, model index
        """

        if self.ble_mesh_comp_data is not None:
            comp_vnd_model = BleMeshModel(client_api=self,
                                          model_api=vnd_model.model_api,
                                          model_cbk=vnd_model.model_cbk,
                                          model_pub=vnd_model.model_pub)

            comp_vnd_model.set_model_opcodes(vnd_model.opcodes)

            # Set the vnd model raw data
            comp_vnd_model.set_vnd_model_raw(cid=self.ble_mesh_comp_data.cid,
                                             vid=vnd_model.model_id,
                                             user_data=vnd_model.user_data)

            # Set the vnd model data at relevant element
            if len(self.ble_mesh_comp_data.elements) > vnd_model.elem_index:
                model_index = self.ble_mesh_comp_data.elements[vnd_model.elem_index].add_model(model_type=lprf_api.common.VND_MODEL, ble_mesh_model=comp_vnd_model)
                rc = self.ble_mesh_api.bt_mesh_init_model_raw_init(elem_index=vnd_model.elem_index,
                                                                   model_index=model_index,
                                                                   model_raw=self.ble_mesh_comp_data.elements[vnd_model.elem_index].vnd_models[model_index].model_raw,
                                                                   op_raw=self.ble_mesh_comp_data.elements[vnd_model.elem_index].vnd_models[model_index].model_opcodes)

                return rc, model_index
            else:
                return -1, -1
        else:
            return -1, -1

    def bt_mesh_cfg_app_key_add(self, app_key, app_idx):
        """
            Add application key
        :param app_key: application key value
        :param app_idx: application key index
        :return: rc: return code from bt_mesh_cfg_app_key_add_wrapper
        """

        rc = self.ble_mesh_api.bt_mesh_cfg_app_key_add_wrapper(net_idx=self.ble_mesh_prov_data.net_idx,
                                                               addr=self.ble_mesh_prov_data.addr,
                                                               key_net_idx=self.ble_mesh_prov_data.net_idx,
                                                               key_app_idx=app_idx,
                                                               app_key=app_key)

        return rc

    def bt_mesh_cfg_mod_app_bind(self, elem_addr, mod_app_idx, mod_id, mod_cid=None):
        """
            Bind Model Application Key
        :param elem_addr: model element address
        :param mod_app_idx: model application index
        :param mod_id: model id
        :param mod_cid: for vnd model enter cid
        :return: rc: return code from bt_mesh_cfg_mod_app_bind_wrapper or bt_mesh_cfg_mod_app_bind_vnd_wrapper
        """

        if mod_cid is None:
            rc = self.ble_mesh_api.bt_mesh_cfg_mod_app_bind_wrapper(net_idx=self.ble_mesh_prov_data.net_idx,
                                                                    addr=self.ble_mesh_prov_data.addr,
                                                                    elem_addr=elem_addr,
                                                                    mod_app_idx=mod_app_idx,
                                                                    mod_id=mod_id)

            # Set the model app key index in host if success bind
            if rc == 0:
                elem_idx = elem_addr - self.ble_mesh_prov_data.addr
                for model in self.ble_mesh_comp_data.elements[int(elem_idx)].sig_models:
                    if model.model_id == mod_id:
                        model.set_app_key_index(mod_app_idx)

        else:
            rc = self.ble_mesh_api.bt_mesh_cfg_mod_app_bind_vnd_wrapper(net_idx=self.ble_mesh_prov_data.net_idx,
                                                                        addr=self.ble_mesh_prov_data.addr,
                                                                        elem_addr=elem_addr,
                                                                        mod_app_idx=mod_app_idx,
                                                                        mod_id=mod_id,
                                                                        cid=mod_cid)

            # Set the model app key index in host if success bind
            if rc == 0:
                elem_idx = elem_addr - self.ble_mesh_prov_data.addr
                for model in self.ble_mesh_comp_data.elements[int(elem_idx)].vnd_models:
                    if model.model_id == mod_id and model.model_cid == mod_cid:
                        model.set_app_key_index(mod_app_idx)

        return rc

    def bt_mesh_cfg_mod_sub_add(self, elem_addr, sub_addr, mod_id, mod_cid=None):
        """
            Add Address to Model Subscription List
        :param elem_addr: model element address
        :param sub_addr: address to add
        :param mod_id: model id
        :param mod_cid: for vnd model enter cid
        :return: rc: return code from bt_mesh_cfg_mod_sub_add_wrapper or bt_mesh_cfg_mod_sub_add_vnd_wrapper
        """

        if mod_cid is None:
            rc = self.ble_mesh_api.bt_mesh_cfg_mod_sub_add_wrapper(net_idx=self.ble_mesh_prov_data.net_idx,
                                                                   addr=self.ble_mesh_prov_data.addr,
                                                                   elem_addr=elem_addr,
                                                                   sub_addr=sub_addr,
                                                                   mod_id=mod_id)

            # add the model sub address to the group address list in host if success add
            if rc == 0:
                elem_idx = elem_addr - self.ble_mesh_prov_data.addr
                for model in self.ble_mesh_comp_data.elements[int(elem_idx)].sig_models:
                    if model.model_id == mod_id:
                        model.add_sub_addr(sub_addr)

        else:
            rc = self.ble_mesh_api.bt_mesh_cfg_mod_sub_add_vnd_wrapper(net_idx=self.ble_mesh_prov_data.net_idx,
                                                                       addr=self.ble_mesh_prov_data.addr,
                                                                       elem_addr=elem_addr,
                                                                       sub_addr=sub_addr,
                                                                       mod_id=mod_id,
                                                                       cid=mod_cid)

            # add the model sub address to the group address list in host if success add
            if rc == 0:
                elem_idx = elem_addr - self.ble_mesh_prov_data.addr
                for model in self.ble_mesh_comp_data.elements[int(elem_idx)].vnd_models:
                    if model.model_id == mod_id and model.model_cid == mod_cid:
                        model.add_sub_addr(sub_addr)

        return rc

    def bt_mesh_cfg_mod_sub_del(self, elem_addr, sub_addr, mod_id, mod_cid=None):
        """
            Detete Address from Model Subscription List
        :param elem_addr: model element address
        :param sub_addr: address to delete
        :param mod_id: model id
        :param mod_cid: for vnd model enter cid
        :return: rc: return code from bt_mesh_cfg_mod_sub_del_wrapper or bt_mesh_cfg_mod_sub_del_vnd_wrapper
        """

        if mod_cid is None:
            rc = self.ble_mesh_api.bt_mesh_cfg_mod_sub_del_wrapper(net_idx=self.ble_mesh_prov_data.net_idx,
                                                                   addr=self.ble_mesh_prov_data.addr,
                                                                   elem_addr=elem_addr,
                                                                   sub_addr=sub_addr,
                                                                   mod_id=mod_id)

            # remove the model sub address from the group address list in host if success delete
            if rc == 0:
                elem_idx = elem_addr - self.ble_mesh_prov_data.addr
                for model in self.ble_mesh_comp_data.elements[int(elem_idx)].sig_models:
                    if model.model_id == mod_id:
                        model.delete_sub_addr(sub_addr)

        else:
            rc = self.ble_mesh_api.bt_mesh_cfg_mod_sub_del_vnd_wrapper(net_idx=self.ble_mesh_prov_data.net_idx,
                                                                       addr=self.ble_mesh_prov_data.addr,
                                                                       elem_addr=elem_addr,
                                                                       sub_addr=sub_addr,
                                                                       mod_id=mod_id,
                                                                       cid=mod_cid)

            # remove the model sub address from the group address list in host if success delete
            if rc == 0:
                elem_idx = elem_addr - self.ble_mesh_prov_data.addr
                for model in self.ble_mesh_comp_data.elements[int(elem_idx)].vnd_models:
                    if model.model_id == mod_id and model.model_cid == mod_cid:
                        model.delete_sub_addr(sub_addr)

        return rc

    def bt_mesh_cfg_mod_va_add(self, elem_addr, label, mod_id, mod_cid=None):
        """
            Add Virtual Address to Model Subscription List
        :param elem_addr: model element address
        :param label: virtual address label to add to the subscription list
        :param mod_id: model id
        :param mod_cid: for vnd model enter cid
        :return: rc: return code from bt_mesh_cfg_mod_sub_va_add_wrapper or bt_mesh_cfg_mod_sub_va_add_vnd_wrapper
        """

        if mod_cid is None:
            rc = self.ble_mesh_api.bt_mesh_cfg_mod_sub_va_add_wrapper(net_idx=self.ble_mesh_prov_data.net_idx,
                                                                      addr=self.ble_mesh_prov_data.addr,
                                                                      elem_addr=elem_addr,
                                                                      label=label,
                                                                      mod_id=mod_id)

        else:
            rc = self.ble_mesh_api.bt_mesh_cfg_mod_sub_va_add_vnd_wrapper(net_idx=self.ble_mesh_prov_data.net_idx,
                                                                          addr=self.ble_mesh_prov_data.addr,
                                                                          elem_addr=elem_addr,
                                                                          label=label,
                                                                          mod_id=mod_id,
                                                                          cid=mod_cid)

        return rc

    def bt_mesh_cfg_mod_pub_set(self, elem_addr, pub, mod_id, mod_cid=None):
        """
            Set publish parameters for a SIG/VND model
        :param elem_addr: model element address
        :param pub: Publication parameters
        :param mod_id: model id
        :param mod_cid: for vnd model enter cid
        :return: rc: return code from bt_mesh_cfg_mod_pub_set_wrapper or bt_mesh_cfg_mod_pub_set_vnd_wrapper
        """

        if mod_cid is None:
            rc = self.ble_mesh_api.bt_mesh_cfg_mod_pub_set_wrapper(net_idx=self.ble_mesh_prov_data.net_idx,
                                                                   addr=self.ble_mesh_prov_data.addr,
                                                                   elem_addr=elem_addr,
                                                                   mod_id=mod_id,
                                                                   pub=pub)

        else:
            rc = self.ble_mesh_api.bt_mesh_cfg_mod_pub_set_vnd_wrapper(net_idx=self.ble_mesh_prov_data.net_idx,
                                                                       addr=self.ble_mesh_prov_data.addr,
                                                                       elem_addr=elem_addr,
                                                                       mod_id=mod_id,
                                                                       cid=mod_cid,
                                                                       pub=pub)

        return rc

    def bt_mesh_model_find(self, elem_idx, mod_id, mod_cid=None):
        """
            Find Model in Composition Data
        :param elem_idx: model element index
        :param mod_id: model id
        :param mod_cid: model company id
        :return: if model in comp data return model idx (-1 not found)
        """

        if mod_cid is None:
            model_idx = self.ble_mesh_api.bt_mesh_model_find_wrapper(elem_idx=elem_idx, id=mod_id)
        else:
            model_idx = self.ble_mesh_api.bt_mesh_model_find_vnd_wrapper(elem_idx=elem_idx, company=mod_cid, id=mod_id)

        return model_idx

    def bt_mesh_fault_update(self, elem_idx):
        """
        :return: rc: return code from bt_mesh_fault_update_wrapper
        """

        rc = self.ble_mesh_api.bt_mesh_fault_update_wrapper(elem_idx)
        return rc

    def bt_mesh_proxy_identity_enable(self):
        """
        :return: rc: return code from bt_mesh_proxy_identity_enable
        """

        rc = self.ble_mesh_api.bt_mesh_proxy_identity_enable()
        return rc

    def bt_mesh_lpn_set(self, enable):
        """
            Set Node LPN mode Enable/Disable
        :param enable: True/False
        :return: rc: return code from bt_mesh_lpn_set
        """

        rc = self.ble_mesh_api.bt_mesh_lpn_set(enable)
        return rc

    def bt_mesh_lpn_poll(self):
        """
        :return: rc: return code from bt_mesh_lpn_poll
        """

        rc = self.ble_mesh_api.bt_mesh_lpn_poll()
        return rc


class BleMeshServerCbk(threading.Thread):
    """ BLE Mesh Server CBK listener class"""

    def __init__(self, arbitrator, codec, cbk_handler):
        """
            Init the BLE Mesh Server Cbk attributes
        :param arbitrator: erpc arbitrator
        :param codec: erpc basic_codec
        :param cbk_handler: BLE Mesh CBK Handler object
        """

        # Set the Server listener thread
        threading.Thread.__init__(self)

        # Set the eRPC arbitrator and codec
        self.arbitrator = arbitrator
        self.codec = codec

        # Set the eRPC cbk server
        self.server = erpc.simple_server.SimpleServer(self.arbitrator, self.codec)

        # Create and register BLE Mesh Callback service with server
        self.cbk_handler = cbk_handler
        self.cbk_service = lprf_cbk.server.BLEmesh_cbkService(self.cbk_handler)
        self.cbk_service_access = lprf_cbk.server.BLEmesh_cbk_accessService(self.cbk_handler)
        self.cbk_service_health_srv = lprf_cbk.server.BLEmesh_cbk_health_srvService(self.cbk_handler)

        self.server.add_service(self.cbk_service)
        self.server.add_service(self.cbk_service_access)
        self.server.add_service(self.cbk_service_health_srv)

    def run(self):
        """
            Run the BLE Mesh Server Cbk
        """

        self.server.run()


class BleMeshNodeErpc:
    """ BLE Mesh Node eRPC class"""

    def __init__(self, port, cbk_handler=None):
        """
            Init the BLE Mesh Node eRPC attributes
        :param port: device serial port name
        :param cbk_handler: BLE Mesh CBK Handler object to set
        """

        # Create shared eRPC transport and arbitrator
        self.transport = erpc.transport.SerialTransport(port, 115200, timeout=5)
        self.codec = erpc.basic_codec.BasicCodec
        self.arbitrator = erpc.arbitrator.TransportArbitrator(self.transport, self.codec())

        # Set the BLE Mesh CBK Handler, if None create default
        if cbk_handler is None:
            self.cbk_handler = BleMeshCbkHandler()
        else:
            self.cbk_handler = cbk_handler

        # Create BLE Mesh Client Api
        self.client_api = BleMeshClientApi(self.transport, self.arbitrator, self.codec, self.cbk_handler)
        self.cbk_handler.set_client_api(self.client_api)

        # Create the BLE Mesh Server Cbk
        self.callback_server = BleMeshServerCbk(self.arbitrator, self.codec, self.cbk_handler)

    def start(self):
        """
            Start the eRPC communication with the device
        """

        # Start the BLE Mesh Server Cbk
        self.callback_server.start()

    def stop(self):
        """
            Stop the eRPC communication with the device
        :return:
        """

        # Signal BLE Mesh Server Cbk thread to terminate
        self.transport.terminate = True

        # Wait for BLE Mesh Server Cbk thread to terminate
        self.callback_server.join()

        # Close the serial transport
        self.transport.close()

        # Destruct the BLE Mesh Client Api object
        self.client_api.__delete__()
