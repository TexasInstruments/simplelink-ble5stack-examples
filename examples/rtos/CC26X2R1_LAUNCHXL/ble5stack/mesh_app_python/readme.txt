#run sample application
  cd <SDK_ROOT>\examples\rtos\CC26X2R1_LAUNCHXL\ble5stack\erpc_gen_mesh\mesh_app_python
  python ble_test_appl.py com45

#Install python(once)
  https://www.python.org/downloads/release/python-376/
  Windows x86-64 executable installer

#Install eRpc (once)
  cd <SDK_ROOT>\source\third_party\erpc\erpc_python
  <YOUR_PYTHON_FOLDER>\Scripts\pip.exe install enum34 --proxy wwwgate.ti.com
  <YOUR_PYTHON_FOLDER>\Scripts\pip.exe install pyserial --proxy wwwgate.ti.com
  <YOUR_PYTHON_FOLDER>\python.exe setup.py install

