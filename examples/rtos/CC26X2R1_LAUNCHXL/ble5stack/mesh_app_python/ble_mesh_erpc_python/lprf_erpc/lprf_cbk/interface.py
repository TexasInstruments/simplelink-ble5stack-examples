#
# Generated by erpcgen 1.7.4 on Thu Jun  3 17:44:56 2021.
#
# AUTOGENERATED - DO NOT EDIT
#

import erpc

# Abstract base class for BLEmesh_cbk_access
class IBLEmesh_cbk_access(object):
    SERVICE_ID = 1
    START_CB_ID = 1
    INIT_CB_ID = 2
    RESET_CB_ID = 3
    SETTINGS_SET_CB_ID = 4
    UPDATE_CB_ID = 5
    FUNC_CB_ID = 6

    def start_cb(self, elem_idx, is_vnd, model_index):
        raise NotImplementedError()

    def init_cb(self, elem_idx, is_vnd, model_index):
        raise NotImplementedError()

    def reset_cb(self, elem_idx, is_vnd, model_index):
        raise NotImplementedError()

    def settings_set_cb(self, elem_idx, is_vnd, model_index, name, data):
        raise NotImplementedError()

    def update_cb(self, elem_idx, is_vnd, model_index):
        raise NotImplementedError()

    def func_cb(self, opcode, elem_idx, is_vnd, model_index, ctx, buf):
        raise NotImplementedError()


# Abstract base class for BLEmesh_cbk_health_srv
class IBLEmesh_cbk_health_srv(object):
    SERVICE_ID = 2
    FAULT_GET_CUR_CB_ID = 1
    FAULT_GET_REG_CB_ID = 2
    FAULT_CLEAR_CB_ID = 3
    FAULT_TEST_CB_ID = 4
    ATTN_ON_CB_ID = 5
    ATTN_OFF_CB_ID = 6

    def fault_get_cur_cb(self, elem_idx, model_index, test_id, company_id, faults):
        raise NotImplementedError()

    def fault_get_reg_cb(self, elem_idx, model_index, company_id, test_id, faults):
        raise NotImplementedError()

    def fault_clear_cb(self, elem_idx, model_index, company_id):
        raise NotImplementedError()

    def fault_test_cb(self, elem_idx, model_index, test_id, company_id):
        raise NotImplementedError()

    def attn_on_cb(self, elem_idx, model_index):
        raise NotImplementedError()

    def attn_off_cb(self, elem_idx, model_index):
        raise NotImplementedError()


# Abstract base class for BLEmesh_cbk
class IBLEmesh_cbk(object):
    SERVICE_ID = 4
    OUTPUT_NUMBER_CB_ID = 1
    OUTPUT_STRING_CB_ID = 2
    INPUT_CB_ID = 3
    INPUT_COMPLETE_CB_ID = 4
    UNPROVISIONED_BEACON_CB_ID = 5
    LINK_OPEN_CB_ID = 6
    LINK_CLOSE_CB_ID = 7
    COMPLETE_CB_ID = 8
    NODE_ADDED_CB_ID = 9
    RESET_PROV_CB_ID = 10
    HB_RECV_CB_ID = 11
    HB_SUB_END_CB_ID = 12
    LPN_FRIENDSHIP_ESTABLISHED_CB_ID = 13
    LPN_FRIENDSHIP_TERMINATED_CB_ID = 14
    LPN_POLLED_CB_ID = 15
    FRIEND_FRIENDSHIP_ESTABLISHED_CB_ID = 16
    FRIEND_FRIENDSHIP_TERMINATED_CB_ID = 17
    APPKEY_EVT_CB_ID = 18

    def output_number_cb(self, act, num):
        raise NotImplementedError()

    def output_string_cb(self, str):
        raise NotImplementedError()

    def input_cb(self, act, size):
        raise NotImplementedError()

    def input_complete_cb(self):
        raise NotImplementedError()

    def unprovisioned_beacon_cb(self, uuid, oob_info, uri_hash):
        raise NotImplementedError()

    def link_open_cb(self, bearer):
        raise NotImplementedError()

    def link_close_cb(self, bearer):
        raise NotImplementedError()

    def complete_cb(self, net_idx, addr):
        raise NotImplementedError()

    def node_added_cb(self, net_idx, uuid, addr, num_elem):
        raise NotImplementedError()

    def reset_prov_cb(self):
        raise NotImplementedError()

    def hb_recv_cb(self, sub, hops, feat):
        raise NotImplementedError()

    def hb_sub_end_cb(self, sub):
        raise NotImplementedError()

    def lpn_friendship_established_cb(self, net_idx, friend_addr, queue_size, recv_window):
        raise NotImplementedError()

    def lpn_friendship_terminated_cb(self, net_idx, friend_addr):
        raise NotImplementedError()

    def lpn_polled_cb(self, net_idx, friend_addr, retry):
        raise NotImplementedError()

    def friend_friendship_established_cb(self, net_idx, lpn_addr, recv_delay, polltimeout):
        raise NotImplementedError()

    def friend_friendship_terminated_cb(self, net_idx, lpn_addr):
        raise NotImplementedError()

    def appkey_evt_cb(self, app_idx, net_idx, evt):
        raise NotImplementedError()


