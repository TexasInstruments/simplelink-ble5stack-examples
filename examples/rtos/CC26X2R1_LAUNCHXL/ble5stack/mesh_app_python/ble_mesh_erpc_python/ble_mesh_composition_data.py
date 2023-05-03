#!/usr/bin/python

from threading import Thread
from queue import Queue
from multiprocessing import Semaphore
from .lprf_erpc import lprf_api

################################### BLE Mesh Composition Data Objects #################################################


class BleMeshCompData:
    """ BLE Mesh Composition Data class"""

    def __init__(self, cid, pid, vid, elem_count):
        """
            Init the BLE Mesh Composition Data attributes
        :param cid: company identifier
        :param pid: product identifier
        :param vid: version identifier
        :param elem_count: the number of elements in this Node
        """

        # Init attributes for elements
        self.elements = list()
        self.elem_count = elem_count

        # Company and Product data
        self.cid = cid
        self.pid = pid
        self.vid = vid

        # Init the comp raw
        self.comp_raw = lprf_api.common.bt_mesh_comp_raw(cid=cid, pid=pid, vid=vid,
                                                         elem_count=self.elem_count,
                                                         elem_placeholder=0)

    def __delete__(self):
        """
            Destruct the BLE Mesh Elements list attribute
        """

        # Destruct the elements list
        for i in range(len(self.elements)):
            self.elements[i].__delete__()
        self.elements.clear()

    def clear(self):
        """
            Clear BLE Mesh Composition Data Elements
        """

        self.elements.clear()

    def set_primary_element(self, addr, loc, model_count, vnd_model_count=0):
        """
            Set the primary element in index 0
        :param addr: element address
        :param loc: element location Descriptor
        :param model_count: the number of SIG models in this element
        :param vnd_model_count: the number of vendor models in this element
        :return primary_element_idx: the primary element index that created
        """

        # Set the primary ble mesh element in index 0
        primary_element_idx = 0
        if len(self.elements) == primary_element_idx:
            self.elements.append(BleMeshElement(elem_idx=primary_element_idx, addr=addr, loc=loc,
                                                model_count=model_count, vnd_model_count=vnd_model_count))

            return primary_element_idx
        else:
            raise IndexError("Error: primary_element already set!!")

    def add_secondary_element(self, addr, loc, model_count, vnd_model_count):
        """
            Add secondary element
        :param addr: unicast element address
        :param loc: element location Descriptor
        :param model_count: the number of SIG models in this element
        :param vnd_model_count: the number of vendor models in this element
        :return current_element_idx: the secondary element index that created
        """

        current_element_idx = len(self.elements)
        if current_element_idx < self.elem_count:
            self.elements.append(BleMeshElement(elem_idx=current_element_idx, addr=addr, loc=loc,
                                                model_count=model_count, vnd_model_count=vnd_model_count))
            return current_element_idx
        else:
            raise IndexError("Error: No place for new element: {}/{}".format(current_element_idx, self.elem_count))


class BleMeshElement:
    """ BLE Mesh Element class"""

    def __init__(self, elem_idx, addr, loc, model_count, vnd_model_count):
        """
            Init the BLE Mesh Element attributes
        :param elem_idx: mesh Element index
        :param addr: element address
        :param loc: element location Descriptor
        :param model_count: the number of SIG models the element
        :param vnd_model_count: the number of vendor models in this element
        """

        self.elem_idx = elem_idx
        self.elem_addr = addr

        # Init attributes for SIG & VND models
        self.sig_models = list()
        self.vnd_models = list()

        # Init the element raw
        self.elem_raw = lprf_api.common.bt_mesh_elem_raw(addr=addr, loc=loc,
                                                         model_count=model_count,
                                                         vnd_model_count=vnd_model_count,
                                                         models_placeholder=0,
                                                         vnd_models_placeholder=0)

    def __delete__(self):
        """
            Destruct the BLE Mesh Models list attributes
        """

        # Destruct the sig models list
        for i in range(len(self.sig_models)):
            if type(self.sig_models[i]) is BleMeshModel:
                self.sig_models[i].__delete__()
        self.sig_models.clear()

        # Destruct the vnd models list
        for i in range(len(self.vnd_models)):
            self.vnd_models[i].__delete__()
        self.vnd_models.clear()

    def add_model(self, model_type, ble_mesh_model):
        """
            Add Mesh Model to the element
        :param model_type: mesh model type SIG(0)/VND(1)
        :param ble_mesh_model: BLE Mesh Model object
        """

        # Add SIG Model
        if model_type == lprf_api.common.SIG_MODEL:

            # Set SIG Model
            if type(ble_mesh_model) is BleMeshModel:
                current_model_index = len(self.sig_models)
                ble_mesh_model.set_model_indexes(elem_idx=self.elem_idx, model_index=current_model_index, is_vnd=0)
                self.sig_models.append(ble_mesh_model)
                return current_model_index

            # Set CfgCli Model
            elif type(ble_mesh_model) is BleMeshCfgCliModel:
                current_model_index = len(self.sig_models)
                self.sig_models.append(ble_mesh_model)
                return current_model_index

            # Set CfgSrv Model
            elif type(ble_mesh_model) is BleMeshCfgSrvModel:
                current_model_index = len(self.sig_models)
                self.sig_models.append(ble_mesh_model)
                return current_model_index

            # Set HealthSrv Model
            elif type(ble_mesh_model) is BleMeshHealthSrvModel:
                current_model_index = len(self.sig_models)
                self.sig_models.append(ble_mesh_model)
                return current_model_index
            else:
                raise TypeError("Not BleMeshModel or BleMeshCfgCliModel or BleMeshCfgSrvModel or BleMeshHealthSrvModel Object")

        # Add VND Model
        elif model_type == lprf_api.common.VND_MODEL:

            # Verify ble_mesh_model type
            if type(ble_mesh_model) is not BleMeshModel:
                raise TypeError("Not BleMeshModel Object")

            current_model_index = len(self.vnd_models)
            ble_mesh_model.set_model_indexes(elem_idx=self.elem_idx, model_index=current_model_index, is_vnd=1)
            self.vnd_models.append(ble_mesh_model)
            return current_model_index

        else:
            raise TypeError("{} is Not Supported Ble Mesh Model Type SIG(0)/VND(1)".format(model_type))

################################### BLE Mesh Configuration Client Model Objects #######################################


class BleMeshCfgCliModel:
    """ BLE Mesh Configuration Client Model class"""

    def __init__(self, elem_idx):
        """
            Init the BLE Mesh Configuration Client Model attributes
        :param elem_idx: element index
        """

        self.elem_idx = elem_idx
        self.model_index = None
        self.model_id = lprf_api.common.BT_MESH_MODEL_ID_CFG_CLI

################################### BLE Mesh Configuration Server Model Objects #######################################


class BleMeshCfgSrvModel:
    """ BLE Mesh Configuration Server Model class"""

    def __init__(self):
        """
            Init the BLE Mesh Configuration Server Model attributes
        """

        self.elem_idx = 0
        self.model_index = None
        self.model_id = lprf_api.common.BT_MESH_MODEL_ID_CFG_SRV

################################### BLE Mesh Health Server Model Objects #############################################


class BleMeshHealthSrvModel:
    """ BLE Mesh Health Server Model class"""

    def __init__(self, elem_idx, health_srv_cb_raw, cid, cbk_handler=None):
        """
            Init the BLE Mesh Health Server Model attributes
        :param elem_idx: element index
        :param health_srv_cb_raw: lprf_api.common.bt_mesh_health_srv_raw
        :param cid: The company ID as in the composition data object
        :param cbk_handler: BleMeshHealthSrvCbk object
        """

        self.elem_idx = elem_idx
        self.model_index = None
        self.model_id = lprf_api.common.BT_MESH_MODEL_ID_HEALTH_SRV
        self.cid = cid
        self.model_app_idx = None

        if cbk_handler is None:
            self.health_srv_cbk = BleMeshHealthSrvCbk(cid=cid)
        else:
            self.health_srv_cbk = cbk_handler

        self.health_srv_cbk.health_srv_cb_raw = health_srv_cb_raw
        self.health_srv_raw = lprf_api.common.bt_mesh_health_srv_raw(model_placeholder=0,
                                                                     k_delayed_work_placeholder=0,
                                                                     cb=self.health_srv_cbk.health_srv_cb_raw)

    def set_app_key_index(self, app_idx):
        """
            Set model app key index
        :param app_idx: app key index
        """

        self.model_app_idx = app_idx

    def add_supported_test(self, test_id, test_func):
        """
            Add fault test to the supported fault tests list
        :param test_id: test id (in hex)
        :param test_func: test def func name to execute
        """

        if test_id not in self.health_srv_cbk.supported_tests:
            self.health_srv_cbk.supported_tests[test_id] = test_func

    def remove_supported_test(self, test_id):
        """
            Remove fault test from the supported fault tests list
        :param test_id: test id (in hex)
        """

        if test_id in self.health_srv_cbk.supported_tests:
            del self.health_srv_cbk.supported_tests[test_id]

    def get_supported_test_list(self):
        """
            Get the supported fault tests list
        """

        return list(self.health_srv_cbk.supported_tests.key())

    def get_last_test_result(self):
        """
            Get the Last Fault Test ID results dict {Test ID: Test Return Value}
        """

        return {self.health_srv_cbk.last_run_test_id: self.health_srv_cbk.last_run_test_res}

    def add_fault(self, fault_id):
        """
            Add new fault to the fault array of the health server
        :param fault_id: The fault number (either an error or warning)
        """

        if fault_id not in self.health_srv_cbk.cur_faults_list:
            self.health_srv_cbk.cur_faults_list.append(fault_id)
        if fault_id not in self.health_srv_cbk.reg_faults_list:
            self.health_srv_cbk.reg_faults_list.append(fault_id)

    def clear_cur_fault_list(self):
        """
            Clear the current fault list of the health server
        """

        self.health_srv_cbk.cur_faults_list.clear()

    def clear_reg_fault_list(self):
        """
            Clear the registered fault list of the health server
        """

        self.health_srv_cbk.reg_faults_list.clear()


class BleMeshHealthSrvCbk:
    """ Basic BLE Mesh Health Server Cbk class"""

    def __init__(self, cid):
        """
            Init the BLE Mesh Health Server Cbk attributes
        :param cid: The company ID as in the composition data object
        """

        self.health_srv_cb_raw = None
        self.cid = cid
        self.supported_tests = {0x00: "standard_test"}
        self.last_run_test_id = 0x0
        self.last_run_test_res = 0
        self.reg_faults_list = list()
        self.cur_faults_list = list()

    @staticmethod
    def print_msg(msg):
        """ BLE Mesh Health Server Cbk messages print function """

        print("\n{}\nHealthSrvModelCBK MSG: {}\n{}".format('~'*50, msg, '~'*50))

    def fault_get_cur_cb(self, elem_idx, model_index, test_id, company_id, faults):
        """
            Health server fault_get_cur_cb function
        :param elem_idx:    cbk element index
        :param model_index: cbk model index
        :param test_id:     Test ID response buffer.
        :param company_id:  Company ID response buffer.
        :param faults:      List to fill with current faults.
        """

        faults.value = self.reg_faults_list.copy()
        test_id.value = self.last_run_test_id
        company_id.value = self.cid
        self.print_msg("fault_get_cur_cb cbk: elem_idx={}, model_index={}, test_id={}, company_id={}, faults={}".format(elem_idx,
                                                                                                                        model_index,
                                                                                                                        test_id.value,
                                                                                                                        company_id.value,
                                                                                                                        faults.value))
        return 0

    def fault_get_reg_cb(self, elem_idx, model_index, company_id, test_id, faults):
        """
            Health server fault_get_cur_cb function
        :param elem_idx:    cbk element index
        :param model_index: cbk model index
        :param company_id: cbk model cid
        :param test_id: cbk test ID response buffer.
        :param faults: cbk list to fill with current faults.
        """

        if company_id != self.cid:
            return -1

        faults.value = self.reg_faults_list.copy()
        test_id.value = self.last_run_test_id
        self.print_msg("fault_get_reg_cb cbk: elem_idx={}, model_index={}, company_id={}, test_id={}, faults={}".format(elem_idx,
                                                                                                                        model_index,
                                                                                                                        company_id,
                                                                                                                        test_id.value,
                                                                                                                        faults.value))
        return 0

    def fault_clear_cb(self, elem_idx, model_index, company_id):
        """
            Health server fault_clear_cb function
        :param elem_idx: cbk element index
        :param model_index: cbk model index
        :param company_id: cbk company id
        """

        self.reg_faults_list.clear()
        self.print_msg("fault_clear_cb cbk: elem_idx={}, model_index={}, company_id={:04x}".format(elem_idx, model_index, company_id))
        return 0

    def fault_test_cb(self, elem_idx, model_index, test_id, company_id):
        """
            Health server fault_test_cb function
        :param elem_idx: cbk element index
        :param model_index: cbk model index
        :param test_id: cbk test id
        :param company_id: cbk company id
        """

        self.print_msg("fault_test_cb cbk: elem_idx={}, model_index={}, test_id={:04x}, company_id={:04x}".format(elem_idx, model_index, test_id, company_id))
        self.last_run_test_id = test_id

        # Verify fault test test id
        if test_id not in self.supported_tests:
            self.print_msg("Error: Test ID {:04x} Not in supported_tests".format(test_id))
            self.last_run_test_res = -1
            return -1

        # Verify fault test company_id
        if company_id != self.cid:
            self.print_msg("Error: Company ID {:04x} is Not Supported".format(company_id))
            self.last_run_test_res = -1
            return -1

        # Verify fault test self function def implementation
        if not self.supported_tests[test_id] in dir(self):
            self.print_msg("Error: No Implementation for Test Function '{}' Under {} Object!!!".format(self.supported_tests[test_id], self.__class__.__name__))
            self.last_run_test_res = -1
            return -1

        try:
            exec("res = self.{}()".format(self.supported_tests[test_id]))
            test_results = locals()['res']
            self.last_run_test_res = test_results
            return test_results
        except Exception as test_error:
            self.print_msg("Error While Running Test {}({:04x}): {}".format(self.supported_tests[test_id], test_id, test_error))
            self.last_run_test_res = -1
            return -1

    def attn_on_cb(self, elem_idx, model_index):
        """
            Health server attn_on_cb function
        :param elem_idx: cbk element index
        :param model_index: cbk model index
        """

        self.print_msg("attn_on_cb cbk: elem_idx={}, model_index={}".format(elem_idx, model_index))

    def attn_off_cb(self, elem_idx, model_index):
        """
            Health server attn_off_cb function
        :param elem_idx: cbk element index
        :param model_index: cbk model index
        """

        self.print_msg("attn_off_cb cbk: elem_idx={}, model_index={}".format(elem_idx, model_index))

    def standard_test(self):
        """ Standard Test (0x00) fault test cb process"""

        self.print_msg("Standard Fault Test (0x00) Success")
        return 0

################################### BLE Mesh Model Objects ###########################################################


class BleMeshModelData:
    """ BLE Mesh Model Data Template class (Using in Models Implementation)"""

    def __init__(self):
        """
            Set The Default BLE Mesh Model Data Attributes
        """

        # Model opcodes list [['opcode(hex)', min_len, 'cbk_func_name'],,]
        self.opcodes = list()

        # Init model api and cbk objects
        self.model_api = None
        self.model_cbk = None
        self.model_pub = None

        # Init model attributes
        self.elem_index = None
        self.model_id = None
        self.user_data = None
        self.is_vnd = False


class BleMeshModel:
    """ BLE Mesh Model class (Using Under BleMeshCompData while init Model)"""

    def __init__(self, client_api, model_api, model_cbk, model_pub=None):
        """
            Init the BLE Mesh Model attributes
        :param client_api: BLE Mesh Client Api object
        :param model_api: BLE Mesh Model Api object
        :param model_cbk: BLE Mesh Model Cbk object
        :param model_pub: BLE Mesh Model Pub object
        """

        # Init model attributes
        self.model_opcodes = list()
        self.model_sub_list = list()
        self.elem_idx = None
        self.model_index = None
        self.is_vnd = None
        self.model_app_idx = None
        self.model_raw = None
        self.model_id = None
        self.model_cid = None

        # Set the model api object and init his client api
        self.model_api = model_api
        self.model_api.set_model(self)
        self.model_api.set_client_api(client_api)

        # Set the model cbk object and init his model api
        self.model_cbk = model_cbk
        self.model_cbk.set_model_api(self.model_api)

        # Set the model pub object
        if model_pub is not None:
            self.model_pub = model_pub
            if self.model_pub.update_cb_enable:
                self.model_pub.model_pub_raw.update = self.model_api.client_api.cbk_handler.update_cb
        else:
            self.model_pub = BleMeshModelPub()

        # Init model cbk raw
        self.model_cb_raw = lprf_api.common.bt_mesh_model_cb_raw(self.model_api.client_api.cbk_handler.start_cb,
                                                                 self.model_api.client_api.cbk_handler.init_cb,
                                                                 self.model_api.client_api.cbk_handler.reset_cb,
                                                                 self.model_api.client_api.cbk_handler.settings_set_cb)

    def __delete__(self):
        """
            Destruct the BLE Mesh Model Api attribute
        """

        self.model_api.__delete__()

    def set_model_indexes(self, elem_idx, model_index, is_vnd):
        """
            Set the model indexes values
        :param elem_idx: element index
        :param model_index: model index
        :param is_vnd: is vnd model (1/0)
        """

        self.elem_idx = elem_idx
        self.model_index = model_index
        self.is_vnd = is_vnd

    def set_model_opcodes(self, opcodes_list):
        """
            Set Model Opcodes data
        :param opcodes_list: opcodes list [(opcode, min_len, "cbk function name"),..]
        """

        for opcode in opcodes_list:
            # Create the model opcode raw
            model_op_raw = lprf_api.common.bt_mesh_model_op_raw(opcode=opcode[0],
                                                                min_len=opcode[1],
                                                                func=self.model_api.client_api.cbk_handler.func_cb)
            self.model_opcodes.append(model_op_raw)

            # Add the opcode function call to the model_cbk handler
            self.model_cbk.add_model_opcode_cbk(opcode=opcode[0], cbk_call=opcode[2])

    def set_app_key_index(self, app_idx):
        """
            Set model app key index
        :param app_idx: app key index
        """

        self.model_app_idx = app_idx

    def add_sub_addr(self, sub_addr):
        """
            Add Address to model_sub_list
        :param sub_addr: address to add
        """

        if sub_addr not in self.model_sub_list:
            self.model_sub_list.append(sub_addr)

    def delete_sub_addr(self, sub_addr):
        """
            Delete Address from model_sub_list
        :param sub_addr: address to delete
        """

        if sub_addr in self.model_sub_list:
            self.model_sub_list.remove(sub_addr)

    def set_vnd_model_raw(self, cid, vid, user_data):
        """
            Set the model raw data for VND model
        :param cid: company identifier
        :param vid: model ID
        :param user_data: model-specific user data
        """

        # Set the Company model data
        model = lprf_api.common.bt_mesh_model_raw.model_union()
        model.company = cid
        model.vnd_id = vid
        self.model_id = vid
        self.model_cid = cid

        # Init the model raw
        self.model_raw = lprf_api.common.bt_mesh_model_raw(model_type=lprf_api.common.VND_MODEL,
                                                           model=model, pub=self.model_pub.model_pub_raw,
                                                           user_data=user_data, cb=self.model_cb_raw)

    def set_sig_model_raw(self, sid, user_data):
        """
            Set the model raw data for SIG model
        :param sid: SIG identifier
        :param user_data: model-specific user data
        """

        # Set the Company model data
        model = lprf_api.common.bt_mesh_model_raw.model_union()
        model.id = sid
        self.model_id = sid

        # Init the model raw
        self.model_raw = lprf_api.common.bt_mesh_model_raw(model_type=lprf_api.common.SIG_MODEL,
                                                           model=model, pub=self.model_pub.model_pub_raw,
                                                           user_data=user_data, cb=self.model_cb_raw)


class BleMeshModelPub:
    """ Basic BLE Mesh Model Publication class"""

    def __init__(self):
        """
            Init BLE Mesh Model Publication attributes
        """

        self.model_pub_raw = None
        self.update_cb_enable = False

    def set_model_pub_raw(self, addr, key, ttl, retransmit, period, period_div, period_start, msg, update=False):
        """
            Set the Model Publication parameters
        :param addr: publish address
        :param key: publish app key index
        :param ttl: publish time to live
        :param retransmit: retransmit count & interval steps
        :param period: publish period
        :param period_div: divisor for the period
        :param period_start: start of the current period
        :param msg: publication buffer, containing the publication message
        :param update: True/False Link update_cb function handler to BleMeshCbkHandler.update_cb
        """

        # Init the model pub raw
        self.update_cb_enable = update
        self.model_pub_raw = lprf_api.common.bt_mesh_model_pub_raw(addr=addr, key=key, ttl=ttl, retransmit=retransmit,
                                                                   period=period, period_div=period_div, period_start=period_start,
                                                                   msg=msg)


class BleMeshModelApi:
    """ Basic BLE Mesh Model Api class"""

    def __init__(self):
        """
            Init the BLE Mesh Model Api attributes
        """

        self.ble_mesh_model = None
        self.client_api = None

        # Init the model send thread attributes for sending model message
        self._model_api_execute_th = Thread(target=self._model_api_execute)
        self._model_api_execute_q = Queue()
        self._model_api_execute_semaphore = Semaphore()
        self._model_api_execute_semaphore.acquire()
        self._model_api_execute_trigger = True
        self._model_api_execute_rc = None
        self._model_api_execute_th.start()

    def __delete__(self):
        """
            Destruct the BLE Mesh Model Api
            Signal the model_api_execute thread to terminate
        """

        self._model_api_execute_trigger = False
        self._model_api_execute_semaphore.release()
        self._model_api_execute_th.join()

    @staticmethod
    def print_msg(msg):
        """ BLE Mesh Model Api messages print function """

        print("\n{}\nModelAPI MSG: {}\n{}".format('~'*50, msg, '~'*50))

    def _model_api_execute(self):
        """
            Main Model Api Execute Thread function,
            Get and Run the Api from the _model_api_execute_q, Queue element: (func_call, **func_args)
            Set return code from api to _model_api_execute_rc attribute
        """

        while self._model_api_execute_trigger:

            # Acquire the semaphore, wait for release to check the queue
            self._model_api_execute_semaphore.acquire()

            # check if there is message in the queue
            if not self._model_api_execute_q.empty():

                # Get api call element from the queue
                model_api_func_call, model_api_func_args = self._model_api_execute_q.get()

                # Call the api Func
                rc = model_api_func_call(**model_api_func_args)

                # Store the return code from the api
                self._model_api_execute_rc = rc

    def get_model_api_execute_rc(self):
        """
            Get the last Api return code from model_api_execute
        :return: self._model_api_execute_rc
        """

        # Waite for valid rc
        while self._model_api_execute_rc is None: pass

        current_rc = self._model_api_execute_rc
        self._model_api_execute_rc = None
        return current_rc

    def bt_mesh_model_data_store(self, elem_index, is_vnd, model_index, name, data):
        """
            Process store_data_wrapper execute to model_api_execute queue,
            Release the execution semaphore for run the Api
        :param elem_index: element index
        :param is_vnd: is vnd model
        :param model_index: model index
        :param name: name of the data to be stored in NV
        :param data: list containing the data to be stored in NV
        """

        # Init the store_data_wrapper api args data
        model_data_store_msg = dict()
        model_data_store_msg["elem_index"] = elem_index
        model_data_store_msg["is_vnd"] = is_vnd
        model_data_store_msg["model_index"] = model_index
        model_data_store_msg["name"] = name
        model_data_store_msg["data"] = data

        # put store_data_wrapper api call tuple element in the model api execute queue and release the execution semaphore
        self._model_api_execute_q.put((self.client_api.ble_mesh_api.store_data_wrapper, model_data_store_msg))
        self._model_api_execute_semaphore.release()

    def bt_mesh_model_send_data(self, elem_idx, is_vnd, model_index, ctx, msg):
        """
            Process bt_mesh_model_send_data_wrapper execute to model_api_execute queue,
            Release the execution semaphore for run the Api
        :param elem_idx: element index
        :param is_vnd: is vnd model
        :param model_index: model index
        :param ctx: msg ctx buffer
        :param msg: msg buffer
        """

        # Init the bt_mesh_model_send_data_wrapper api args data
        model_msg_data = dict()
        model_msg_data["elem_idx"] = elem_idx
        model_msg_data["is_vnd"] = is_vnd
        model_msg_data["model_index"] = model_index
        model_msg_data["ctx"] = ctx
        model_msg_data["msg"] = msg

        # put bt_mesh_model_send_data_wrapper api call tuple element in the model api execute queue and release the execution semaphore
        self._model_api_execute_q.put((self.client_api.ble_mesh_api.bt_mesh_model_send_data_wrapper, model_msg_data))
        self._model_api_execute_semaphore.release()

    def bt_mesh_model_publish(self, elem_idx, is_vnd, model_index, msg):
        """
            Process bt_mesh_model_publish_wrapper execute to model_api_execute queue,
            Release the execution semaphore for run the Api
        :param elem_idx: element index
        :param is_vnd: is vnd model
        :param model_index: model index
        :param msg: msg buffer
        """

        # Init the bt_mesh_model_publish_wrapper api args data
        model_publish_msg = dict()
        model_publish_msg["elem_idx"] = elem_idx
        model_publish_msg["is_vnd"] = is_vnd
        model_publish_msg["model_index"] = model_index
        model_publish_msg["msg"] = msg

        # put bt_mesh_model_publish_wrapper api call tuple element in the model api execute queue and release the execution semaphore
        self._model_api_execute_q.put((self.client_api.ble_mesh_api.bt_mesh_model_publish_wrapper, model_publish_msg))
        self._model_api_execute_semaphore.release()

    def set_model(self, ble_mesh_model):
        """
            Set the BLE Mesh Model object
        :param ble_mesh_model: BLE Mesh Model object
        """

        self.ble_mesh_model = ble_mesh_model

    def set_client_api(self, client_api):
        """
            Set the BLE Mesh Client Api object
        :param client_api: BLE Mesh Client Api object
        """

        self.client_api = client_api

    def get_model_indexes(self):
        """
            Get the model indexes
        :return: elem_idx, model_index
        """

        return self.ble_mesh_model.elem_idx, self.ble_mesh_model.model_index

    def get_model_app_index(self):
        """
        :return: model bind app key index
        """

        return self.ble_mesh_model.model_app_idx

    def is_vnd(self):
        """
        :return: is vnd model value (1/0)
        """

        return self.ble_mesh_model.is_vnd

    def bt_mesh_model_msg_init(self, msg, opcode):
        """
            Init model msg
        :param msg: model msg buf
        :param opcode: msg opcode
        """

        # The opcode length possible cases
        op_len_switch = {1: self.opcode_1_byte, 2: self.opcode_2_bytes, 3: self.opcode_3_bytes}

        # Get the function name according to the length of the opcode
        func_name = op_len_switch.get(self.bt_mesh_model_op_len(opcode))
        if func_name is None:
            raise TypeError("Unknown opcode format")
        else:
            func_name(msg, opcode)

    def opcode_1_byte(self, msg, opcode):
        """
            Add 1 byte opcode to msg
        :param msg: model msg buf
        :param opcode: opcode value
        """

        self.net_buf_simple_add_u8(msg, opcode)

    def opcode_2_bytes(self, msg, opcode):
        """
            Add 2 byte opcode to msg
        :param msg: model msg buf
        :param opcode: opcode value
        """

        self.net_buf_simple_add_be16(msg, opcode)

    def opcode_3_bytes(self, msg, opcode):
        """
            Add 3 byte opcode to msg
        :param msg: model msg buf
        :param opcode: opcode value
        """

        self.net_buf_simple_add_u8(msg, ((opcode >> 16) & 0xff))
        self.net_buf_simple_add_le16(msg, (opcode & 0xffff))

    def bt_mesh_model_in_primary(self):
        """
            Get whether the model is in the primary element
        """

        if self.ble_mesh_model.elem_idx == 0:
            return True
        else:
            return False

    def bt_mesh_model_elem(self):
        """
            Get the element index that a model belongs to
        """

        return self.ble_mesh_model.elem_idx

    @staticmethod
    def bt_mesh_model_op_len(_op):
        """
            Detect the opcode len
        :param _op: opcode value
        :return: bytes len
        """

        if _op <= 0xff:
            return 1
        elif _op <= 0xffff:
            return 2
        elif _op <= 0xffffff:
            return 3
        else:
            return "Unknown"

    @staticmethod
    def net_buf_simple_add_u8(msg, val):
        """
            Append u8 value to msg buf
        :param msg: model msg buf
        :param val: value to add
        """

        msg.data.append(val)

    @staticmethod
    def net_buf_simple_add_le16(msg, val):
        """
            Append le16 value to msg buf
        :param msg: model msg buf
        :param val: value to add
        """

        msg.data.append((val & 0xff))
        msg.data.append(val >> 8)

    @staticmethod
    def net_buf_simple_add_be16(msg, val):
        """
            Append be16 value to msg buf
        :param msg: model msg buf
        :param val: value to add
        """

        msg.data.append(val >> 8)
        msg.data.append((val & 0xff))

    @staticmethod
    def bt_mesh_model_op_2(b0, b1):
        """
            Construct the SIG models messages opcodes
        :param b0: b0 value
        :param b1: b1 value
        """

        return (b0 << 8) | b1

    @staticmethod
    def bt_mesh_model_op_3(b0, cid):
        """
            Construct the VND models messages opcodes
        :param b0: b0 value
        :param cid: the vnd model's company id
        """

        return ((b0 << 16) | 0xc00000) | cid

    @staticmethod
    def net_buf_simple_raw(data, size, buf):
        """
            Create model msg net buf raw
        :param data: msg data
        :param size: mag size
        :param buf: msg buffer
        :return: lprf_api.common.net_buf_simple_raw
        """

        return lprf_api.common.net_buf_simple_raw(data, size, buf)

    @staticmethod
    def bt_mesh_msg_ctx_raw(net_idx, app_idx, addr, recv_dst, recv_rssi, recv_ttl, send_rel, send_ttl):
        """
            Create model msg ctx raw
        :param net_idx: net key index of the subnet to send the message on
        :param app_idx: app key index to encrypt the message with
        :param addr: remote address
        :param recv_dst: destination address of a received message
        :param recv_rssi: rssi of received packet
        :param recv_ttl: received ttl value
        :param send_rel: force sending reliably by using segment acknowledgement
        :param send_ttl: send ttl value
        :return: lprf_api.common.bt_mesh_msg_ctx_raw
        """

        return lprf_api.common.bt_mesh_msg_ctx_raw(net_idx=net_idx, app_idx=app_idx, addr=addr,
                                                   recv_dst=recv_dst, recv_rssi=recv_rssi, recv_ttl=recv_ttl,
                                                   send_rel=send_rel, send_ttl=send_ttl)


class BleMeshModelCbk:
    """ Basic BLE Mesh Model Cbk class"""

    def __init__(self):
        """
            Init the BLE Mesh Model Cbk attributes
        """

        self.model_opcodes_info = {}
        self.model_api = None

    @staticmethod
    def print_msg(msg):
        """ BLE Mesh Model Cbk messages print function """

        print("\n{}\nModelCBK MSG: {}\n{}".format('~'*50, msg, '~'*50))

    def set_model_api(self, model_api):
        """
            Set the BLE Mesh Model Api object
        :param model_api: BLE Mesh Model Api object
        """

        self.model_api = model_api

    def execute_opcode_cbk(self, opcode, elem_index, is_vnd, model_index, ctx, buf):
        """
            Execute the selected model opcode callback
        :param opcode: opcode value
        :param elem_index: element index
        :param is_vnd: is vnd model
        :param model_index: model index
        :param ctx: msg ctx buffer
        :param buf: msg buffer
        """

        # Verify model opcode
        if opcode in self.model_opcodes_info:
            func_to_exec = self.model_opcodes_info[opcode]

            # Verify for model opcode cbk self function def implementation
            if func_to_exec in dir(self):
                try:
                    exec("self.{}(elem_index, is_vnd, model_index, ctx, buf)".format(func_to_exec),
                         {'self': self, 'elem_index': elem_index, 'is_vnd': is_vnd, 'model_index': model_index, 'ctx': ctx, 'buf': buf})
                except Exception as opcode_error:
                    self.print_msg("Error While Running Model Opcode CBK {}({:04x}): {}".format(func_to_exec, opcode, opcode_error))
            else:
                self.print_msg("Error: No Implementation for Model Opcode CBK Function '{}' Under {} Object!!!".format(func_to_exec, self.__class__.__name__))
        else:
            self.print_msg("Error: Opcode {:04x} Not in model_opcodes_info".format(opcode))

    def add_model_opcode_cbk(self, opcode, cbk_call):
        """
            Add execution callback function for model opcode
        :param opcode: opcode value
        :param cbk_call: function call name
        """

        self.model_opcodes_info[opcode] = "{}".format(cbk_call)

    def start_model_cb(self, elem_idx, is_vnd, model_index):
        """
            VND Model start_cb function
        :param elem_idx: element index
        :param is_vnd: is vnd model
        :param model_index: model index
        """

        self.print_msg("start_model_cb: elem_idx={}, is_vnd={}, model_index={}".format(elem_idx, is_vnd, model_index))

    def update_model_cb(self, elem_idx, is_vnd, model_index):
        """
            VND Model update_cb function
        :param elem_idx: element index
        :param is_vnd: is vnd model
        :param model_index: model index
        """

        self.print_msg("update_model_cb: elem_idx={}, is_vnd={}, model_index={}".format(elem_idx, is_vnd, model_index))

    def init_model_cb(self, elem_idx, is_vnd, model_index):
        """
            Model init_cb function
        :param elem_idx: element index
        :param is_vnd: is vnd model
        :param model_index: model index
        """

        self.print_msg("init_model_cb: elem_idx={}, is_vnd={}, model_index={}".format(elem_idx, is_vnd, model_index))

    def reset_model_cb(self, elem_idx, is_vnd, model_index):
        """
            Model reset_cb function
        :param elem_idx: element index
        :param is_vnd: is vnd model
        :param model_index: model index
        """

        self.print_msg("reset_model_cb: elem_idx={}, is_vnd={}, model_index={}".format(elem_idx, is_vnd, model_index))

    def settings_set_model_cb(self, elem_idx, is_vnd, model_index, name, data):
        """
            Model settings_set_cb function
        :param elem_idx: element index
        :param is_vnd: is vnd model
        :param model_index: model index
        :param name: name of the data read from NV
        :param data: the data read from NV
        """

        self.print_msg("settings_set_model_cb: elem_idx={}, is_vnd={}, model_index={},"
                       "name={}, data={}".format(elem_idx, is_vnd, model_index, name, data))
