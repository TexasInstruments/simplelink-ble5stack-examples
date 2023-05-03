#!/usr/bin/python

# Import the relevant BLE Mesh Model classes for the Example VND Model from the ble_mesh_erpc_python package
from .ble_mesh_composition_data import BleMeshModelData, BleMeshModelApi, BleMeshModelCbk


###################### BLE Mesh VND Model Example API and CBK objects #########################################################################

class VndModelExample(BleMeshModelData):
    """ TI LED VND Model Example"""

    def __init__(self, element_index, user_data='', model_pub=None):
        """
            Init TI LED Model Example attributes
        """

        BleMeshModelData.__init__(self)

        # Model opcodes
        self.opcodes = [[0xC0000D, 0, "vnd_button_pressed_cb"],
                        [0xC1000D, 0, "vnd_get_status_cb"],
                        [0xC2000D, 0, "vnd_notify_status_cb"],
                        [0xC3000D, 0, "vnd_large_msg_cb"]]

        # Init model api and cbk objects
        self.model_api = VndModelApiExample()
        self.model_cbk = VndModelCbkExample()
        self.model_pub = model_pub

        # Init model attributes
        self.elem_index = element_index
        self.model_id = 0
        self.user_data = user_data
        self.is_vnd = True


class VndModelApiExample(BleMeshModelApi):

    def __init__(self):
        """
            Init the VND Model Api attributes
        """

        BleMeshModelApi.__init__(self)

        # VND Model BTN and LED status attributes
        self.button_status = 0
        self.led_status = 0

    def vnd_large_msg(self, target_addr, msg_size=8, send_ttl=3, send_rel=0):
        """
            Send Generic Test Model message with input CTX values
        :param target_addr: target message address
        :param msg_size: mag size in bits
        :param send_ttl: send TTl value
        :param send_rel: send REL value
        :return rc: return code from bt_mesh_model_send_data
        """

        # Create and Init the VND Model message
        msg = self.net_buf_simple_raw([], msg_size + 3 + 4, 0)
        self.bt_mesh_model_msg_init(msg=msg, opcode=self.bt_mesh_model_op_3(0x03, 0x000D))
        for i in range(int(msg_size)):
            self.net_buf_simple_add_u8(msg=msg, val=0xAB)

        # Init message ctx
        ctx = self.bt_mesh_msg_ctx_raw(net_idx=self.client_api.ble_mesh_prov_data.net_idx,
                                       app_idx=self.get_model_app_index(),
                                       addr=target_addr,
                                       recv_dst=0, recv_rssi=0, recv_ttl=0,
                                       send_rel=send_rel, send_ttl=send_ttl)

        # Call bt_mesh_model_send
        elem_idx, model_index = self.get_model_indexes()
        self.bt_mesh_model_send_data(elem_idx=elem_idx, model_index=model_index,
                                     is_vnd=self.is_vnd(),
                                     ctx=ctx, msg=msg)

        return self.get_model_api_execute_rc()

    def vnd_led_on_off(self, addr, on_off_state):
        """
            Send Toggle LED ON/OFF, Toggle on BTN state ON/OFF
        :param addr: target message address
        :param on_off_state: ON(1)/OFF(0)
        """

        # Set own BTN status
        self.button_status = on_off_state

        # Create and Init the VND Model message
        msg = self.net_buf_simple_raw([], 8, 0)
        self.bt_mesh_model_msg_init(msg=msg, opcode=0xC0000D)
        self.net_buf_simple_add_u8(msg=msg, val=on_off_state)

        # Init message ctx
        ctx = self.bt_mesh_msg_ctx_raw(net_idx=self.client_api.ble_mesh_prov_data.net_idx,
                                       app_idx=self.get_model_app_index(),
                                       addr=addr, recv_dst=0, recv_rssi=0, recv_ttl=0,
                                       send_rel=0, send_ttl=0x05)

        # Call bt_mesh_model_send
        elem_idx, model_index = self.get_model_indexes()
        self.bt_mesh_model_send_data(elem_idx=elem_idx, model_index=model_index, is_vnd=self.is_vnd(),
                                     ctx=ctx, msg=msg)

        return self.get_model_api_execute_rc()

    def vnd_get_led_status(self, addr):
        """
            Send Get LED status request message
        :param addr: target message address
        """

        # Create and Init the VND Model message
        msg = self.net_buf_simple_raw([], 8, 0)
        self.bt_mesh_model_msg_init(msg=msg, opcode=0xC1000D)

        # Init message ctx
        ctx = self.bt_mesh_msg_ctx_raw(net_idx=self.client_api.ble_mesh_prov_data.net_idx,
                                       app_idx=self.get_model_app_index(),
                                       addr=addr, recv_dst=0, recv_rssi=0, recv_ttl=0,
                                       send_rel=0, send_ttl=5)

        # Call bt_mesh_model_send
        elem_idx, model_index = self.get_model_indexes()
        self.bt_mesh_model_send_data(elem_idx=elem_idx, model_index=model_index, is_vnd=self.is_vnd(),
                                     ctx=ctx, msg=msg)

        return self.get_model_api_execute_rc()

    def vnd_notify_led_status(self, addr):
        """
            Send Notify LED status message
        :param addr: target message address
        """

        # Create and Init the VND Model message
        msg = self.net_buf_simple_raw([], 8, 0)
        self.bt_mesh_model_msg_init(msg=msg, opcode=0xC2000D)
        self.net_buf_simple_add_u8(msg=msg, val=self.led_status)

        # Init message ctx
        ctx = self.bt_mesh_msg_ctx_raw(net_idx=self.client_api.ble_mesh_prov_data.net_idx,
                                       app_idx=self.get_model_app_index(),
                                       addr=addr, recv_dst=0, recv_rssi=0, recv_ttl=0,
                                       send_rel=0, send_ttl=5)

        # Call bt_mesh_model_send
        elem_idx, model_index = self.get_model_indexes()
        self.bt_mesh_model_send_data(elem_idx=elem_idx, model_index=model_index, is_vnd=self.is_vnd(),
                                     ctx=ctx, msg=msg)

        return self.get_model_api_execute_rc()


class VndModelCbkExample(BleMeshModelCbk):

    def __init__(self):
        """
            Init the VND Model CBK attributes
        """

        BleMeshModelCbk.__init__(self)

        # VND Model Notify and CallBack status data attributes
        self.notify_data = {}
        self.cbk_data = {}

    def vnd_button_pressed_cb(self, elem_index, is_vnd, model_index, ctx, buf):
        """
            VND Model BTN pressed callback, called when Toggle LED ON/OFF message received
        :param elem_index: element index
        :param is_vnd: is vnd model
        :param model_index: model index
        :param ctx: message ctx
        :param buf: message buffer
        """

        # Update own LED status and cbk data
        cbk_msg = "Button pressed callback from addr={:04x}, ".format(ctx[0].addr)
        self.cbk_data[hex(ctx[0].addr)] = "BTN"

        self.model_api.led_status = buf[0].data[0]
        if self.model_api.led_status == 0:
            cbk_msg += "LED OFF"
        else:
            cbk_msg += "LED ON"

        self.print_msg(cbk_msg)

    def vnd_get_status_cb(self, elem_index, is_vnd, model_index, ctx, buf):
        """
            VND Model Get LED Status callback, called when Get LED status message received
        :param elem_index: element index
        :param is_vnd: is vnd model
        :param model_index: model index
        :param ctx: message ctx
        :param buf: message buffer
        """

        cbk_msg = "Get status callback from addr={:04x}, ".format(ctx[0].addr)
        self.cbk_data[hex(ctx[0].addr)] = "GET"

        # Send notify led status as a result of get status request
        # Create and Init the VND Model message
        msg = self.model_api.net_buf_simple_raw([], 8, 0)
        self.model_api.bt_mesh_model_msg_init(msg=msg, opcode=0xC2000D)
        self.model_api.net_buf_simple_add_u8(msg=msg, val=self.model_api.led_status)

        # Init message ctx
        ctx = self.model_api.bt_mesh_msg_ctx_raw(net_idx=self.model_api.client_api.ble_mesh_prov_data.net_idx,
                                                 app_idx=self.model_api.get_model_app_index(),
                                                 addr=ctx[0].addr, recv_dst=0, recv_rssi=0, recv_ttl=0,
                                                 send_rel=0, send_ttl=0x05)

        # Call bt_mesh_model_send
        self.model_api.bt_mesh_model_send_data(elem_idx=elem_index, is_vnd=is_vnd, model_index=model_index,
                                               ctx=ctx, msg=msg)

        cbk_msg += "Send Notify LED status Response"
        self.print_msg(cbk_msg)

    def vnd_notify_status_cb(self, elem_index, is_vnd, model_index, ctx, buf):
        """
            VND Model Notify LED Status callback, called when Notify LED status message received
        :param elem_index: element index
        :param is_vnd: is vnd model
        :param model_index: model index
        :param ctx: message ctx
        :param buf: message buffer
        """

        self.notify_data[hex(ctx[0].addr)] = buf[0].data[0]
        self.print_msg("NTFY: from addr={:04x}, led_status={}".format(ctx[0].addr, buf[0].data[0]))

    def vnd_large_msg_cb(self, elem_index, is_vnd, model_index, ctx, buf):
        """
            Test Model generic ctx msg callback, called when Generic CTX message received
        :param elem_index: element index
        :param is_vnd: is vnd model
        :param model_index: model index
        :param ctx: message ctx
        :param buf: message buffer
        """

        src_addr = hex(ctx[0].addr)
        self.cbk_data[src_addr] = "GEN_CTX"

        ctx_msg = "Generic CTX callback from addr={:04x}, ".format(ctx[0].addr)
        ctx_msg += "net_idx={}, ".format(ctx[0].net_idx)
        ctx_msg += "app_idx={}, ".format(ctx[0].app_idx)
        ctx_msg += "recv_dst={:04x}, ".format(ctx[0].recv_dst)
        ctx_msg += "recv_rssi={}, ".format(ctx[0].recv_rssi)
        ctx_msg += "recv_ttl={:04x}, ".format(ctx[0].recv_ttl)
        ctx_msg += "send_rel={:04x}, ".format(ctx[0].send_rel)
        ctx_msg += "send_ttl={:04x}".format(ctx[0].send_ttl)
        self.print_msg(ctx_msg)

        buf_data = "".join(list("{:02x}".format(item) for item in buf[0].data))
        buf_msg = "Received Buffer Data={}".format(buf_data)
        self.print_msg(buf_msg)
