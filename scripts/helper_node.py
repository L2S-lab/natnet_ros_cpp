#!/usr/bin/python3

PACKAGE='natnet_ros_cpp'
import roslib
import rospy
import rospkg
import rosparam
from natnet_ros_cpp.srv import MarkerPoses
roslib.load_manifest(PACKAGE)

PKG_PATH = rospkg.RosPack().get_path(PACKAGE)

import subprocess
import os
import re
import yaml

from PyQt5 import QtWidgets, uic
import sys

from PyQt5 import QtGui, QtWidgets
from PyQt5.QtCore import QObject, QThread, pyqtSignal

class WorkerThread(QThread):
    finished_signal = pyqtSignal(int)
    def __init__(self, cmd) -> None:
        super().__init__()
        
        self.cmd = cmd

    def run(self):
        result = subprocess.run(self.cmd,shell=True).returncode
        self.finished_signal.emit(result)

class PyQt5Widget(QtWidgets.QMainWindow):
  def __init__(self):
    super(PyQt5Widget, self).__init__()

    ui_file = os.path.join(PKG_PATH, 'ui', 'helper.ui')
    uic.loadUi(ui_file, self)
    self.setWindowIcon(QtGui.QIcon(os.path.join(PKG_PATH,'ui','logo.png')))

    try:
      self.ros_ip = re.sub(r"[^a-zA-Z0-9.]","",os.environ['ROS_MASTER_URI'].split(':')[1])
    except:
      self.ros_ip=None
    self.ros_dist = os.environ['ROS_DISTRO']
    self.pwd = os.environ['PWD']
    self.config_file = os.path.join(PKG_PATH, 'config','conf_autogen.yaml')
    self.name = rospy.get_param("~mocap_node_name")
    self.pub_params = {"pub_individual_marker":False,
                      "pub_rigid_body": False,
                      "pub_rigid_body_marker": False,
                      "pub_pointcloud": False,
                      "pub_rigid_body_wmc": False,
                      }
    self.log_params = {"log_internals": False,
                      "log_frames": False,
                      "log_latencies": False,
                      }
    self.conn_params = {"serverIP": None,
                        "clientIP": None,
                        "serverType": None,
                        "multicastAddress": None,
                        "serverCommandPort": None,
                        "serverDataPort": None,
                        "globalFrame": None,
                        }
    self.natnet_params = {"{name}".format(name=self.name):None}
    self.error_pass = True
    self.num_of_markers=0
    self.x_position=None
    self.y_position=None
    self.z_position=None

    
    self.outputBox.append("""<div style="color: #ff8c00">[NOTE]</div>""" +
                          ' This prompt does not show all the errors. For full error, check the terminal where this node has been excecuted.'+ 
                          'It is required to launch the node from your main workspace folder where this package has been built')
    self.Log('info',' If your WIFI or LAN IP is not detected in the list of the network, You can use the remote IP option and add the ip address manually.')
    self.Log('info',' rosmaster will be selected initially based on the environment variables. You can change it anytime. Provided rosmaster will be used to run the main natnet node.')
    #self.Log('info',' ')
    IP_data = subprocess.check_output(['lshw','-c','network']).decode('utf-8')
    IP_data=re.sub(r"[^a-zA-Z0-9. ]", "", IP_data)
    IP_data=IP_data.split('network')
    IP_data.pop(0)
    self.IP_LIST = []

    for data in IP_data:
      try:
        _ip = [ t for t in data.split() if t.startswith('ip') ]
        self.outputBox.append("""<div style="color: #656565">[INFO]</div>""" +'Found the connection on the IP ending with ' + str(_ip[0].split('.')[-1]) )
        self.IP_LIST.append(_ip[0].replace('ip',''))
      except:
        pass
        #self.message1 = """<div style="color: #ffae42">[WARN]</div>"""+ 'unable to find the WIFI connection WIFI not connected. If that is not the case, please select the remote IP and add the ip manually'

    #self.outputBox.append("""<div style="color: #656565">[INFO]</div>""" +'If ')
    del IP_data
    if len(self.IP_LIST)>0:
      self.client_ip_spin.setEnabled(True)
      self.client_ip_spin.setMinimum(1)
      self.textClientIP.setText(str(self.IP_LIST[self.client_ip_spin.value()-1]))
      self.textServerIP.setText('.'.join(str(self.IP_LIST[self.client_ip_spin.value()-1]).split('.')[:-1],)+'.')
      self.conn_params["clientIP"] = str(self.IP_LIST[self.client_ip_spin.value()-1])
    self.client_ip_spin.setMaximum(len(self.IP_LIST))

    if self.ros_ip!=None:
      if self.ros_ip=='localhost':
        self.localhost_radio.setChecked(True)
        self.select_localhost()
      elif self.ros_ip in self.IP_LIST:
        i = self.IP_LIST.index(self.ros_ip)
        self.network_radio.setChecked(True)
        self.network_spin.setEnabled(True)
        self.network_spin.setValue(i+1)
        self.select_network()
    
    self.pub_rbm.setEnabled(False)
    self.remoteip_radio.clicked.connect(self.select_remote)
    self.network_radio.clicked.connect(self.select_network)
    self.localhost_radio.clicked.connect(self.select_localhost)
    self.network_spin.valueChanged.connect(self.select_network)
    self.client_ip_spin.valueChanged.connect(self.spin_clientIP)
    self.multicast_radio.clicked.connect(self.clicked_multicast)
    self.unicast_radio.clicked.connect(self.clicked_unicast)
    self.start_node.clicked.connect(self.start)
    self.stop_node.clicked.connect(self.stop)
    self.log_frames.clicked.connect(self.log_frames_setting)
    self.log_internal.clicked.connect(self.log_internal_setting)
    self.log_latencies.clicked.connect(self.log_latencies_setting)
    self.pub_im.clicked.connect(self.pub_im_setting)
    self.pub_rb.clicked.connect(self.pub_rb_setting)
    self.pub_rbm.clicked.connect(self.pub_rbm_setting)
    self.pub_rbwmc.clicked.connect(self.pub_rbwmc_setting)
    self.pub_pc.clicked.connect(self.pub_pc_setting)
    self.push_refresh.clicked.connect(self.call_MarkerPoses_srv)
    self.push_ok.clicked.connect(self.yaml_dump)

  def Log(self,type:str,msg=''):
    if type=="info":
      self.outputBox.append("""<div style="color: #656565">[INFO]</div>""" +msg)
    if type=="error":
      self.outputBox.append("""<div style="color:red">[ERROR]</div>""" +msg)
    if type=="warn":
      self.outputBox.append("""<div style="color: #ffae42">[WARN]</div>""" +msg)
    if type=="block":
      self.outputBox.append("""<div style="color: #656565">--------------------</div>""")

#-----------------------------------------------------------------------------------------
# START/STOP NATNET BUTTON

  def start(self):
    self.Log('block')
    self.set_all_params()
    rospy.Rate(1).sleep()
    #export ROS_HOSTNAME={host}
    command='''
    export ROS_MASTER_URI=http://{ros_ip}:11311;
    source /opt/ros/{dist}/setup.bash;
    source {pwd}/devel/setup.bash;
    rosrun natnet_ros_cpp natnet_ros_cpp __name:={name}
    '''.format(ros_ip=self.ros_ip ,dist=self.ros_dist, pwd=self.pwd, name=self.name)
    self.start_node_thread = WorkerThread(command)
    self.start_node_thread.start()
    #subprocess.Popen(command, shell=True, bufsize=0, 
    #                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    #                universal_newlines=True, 
    #                executable='/bin/bash')

  def stop(self):
    try:
      command='''
      export ROS_MASTER_URI=http://{ros_ip}:11311
      source /opt/ros/{dist}/setup.bash
      rosnode kill {name}
      '''.format(ros_ip=self.ros_ip, host=self.ros_ip ,dist=self.ros_dist, pwd=self.pwd, name=self.name)
      subprocess.check_call(command, shell=True, bufsize=0,  
                      universal_newlines=True, 
                      executable='/bin/bash')
      self.Log('info',' Killed natnet node' )
      self.Log('block')
    except subprocess.CalledProcessError as e:
      self.Log('error','Error killing natnet node try excecuting \"rosnode kill natnet_ros\"')
      self.Log('block')
#----------------------------------------------------------------------------------------
# PUBLISHING RELATED STUFF

  def pub_im_setting(self):
    self.Log('block')
    
    if self.pub_im.isChecked():
      self.set_conn_params()
    if self.error_pass:
      if self.pub_im.isChecked():
        self.pub_params["pub_individual_marker"] = True
        self.Log('block')
        self.Log('info','Go to Single marker naming tab and press the refresh button to complete the configuration of for initial position of the markers (wait for a second or two after pressing the refresh).')
        self.Log('info',' If you do not wish to name some marker, you can leave it empty. Do not repeat names of the markers.')
      else:
        self.pub_params["pub_individual_marker"] = False
    else:
      self.Log('error','Can not get the data of markers from the natnet server. One or more parameters from conenction settings are missing')

  def pub_rb_setting(self):
    if self.pub_rb.isChecked():
      self.pub_params["pub_rigid_body"] = True
      self.Log('info','Enabled publishing rigidbody')
      self.pub_rbm.setEnabled(True)
    else:
      self.pub_params["pub_rigid_body"] = False
      self.pub_rbm.setEnabled(False)

  def pub_rbm_setting(self):
    if self.pub_rbm.isChecked():
      self.pub_params["pub_rigid_body_marker"] = True
      self.Log('info','Enabled publishing rigidbody markers')
    else:
      self.pub_params["pub_rigid_body_marker"] = False

  def pub_pc_setting(self):
    if self.pub_pc.isChecked():
      self.pub_params["pub_pointcloud"] = True
      self.Log('info','Enabled publishing pointcloud')
    else:
      self.pub_params["pub_pointcloud"] = False
  
  def pub_rbwmc_setting(self):
    self.Log('info','This functionality is not supported yet.')

#----------------------------------------------------------------------------------------
# NETWORK SELECTION THINGS

  def select_localhost(self):
    self.remotip_edit.setEnabled(False)
    self.network_spin.setEnabled(False)
    self.ros_ip = 'localhost'
    self.Log('info','Using ROS on localhost')

  def select_network(self):
    self.remotip_edit.setEnabled(False)
    self.network_spin.setEnabled(True)
    self.text_network.setEnabled(True)
    if len(self.IP_LIST)>0:
      self.network_spin.setMinimum(1)
    self.network_spin.setMaximum(len(self.IP_LIST))
    self.text_network.setText(str(self.IP_LIST[self.network_spin.value()-1]))
    self.ros_ip = str(self.IP_LIST[self.network_spin.value()-1])
    self.Log('info','Using ROS on IP ending '+self.ros_ip.split('.')[-1])

  def select_remote(self):
    self.remotip_edit.setEnabled(True)
    self.network_spin.setEnabled(False)
    self.text_network.setEnabled(False)
    self.ros_ip = self.remotip_edit.text()
    if self.ros_ip=='':
      self.Log('warn','Please enter the IP adddress and press the radio button again.')
    else:
      self.Log('info','Using ROS on IP ending '+self.ros_ip.split('.')[-1])

#----------------------------------------------------------------------------------------
# NATNET CONNECTION RELATED

  def get_server_ip(self):
    self.conn_params["serverIP"] = self.textServerIP.text()
    self.Log('info','setting servet ip '+str(self.conn_params["serverIP"]))
    #self.Log('info','setting server ip '+str(self.conn_params["serverIP"]).split('.')[0]+'***'+'***'+str(self.conn_params["serverIP"]).split('.')[-1])

  def get_client_ip(self):
    self.conn_params["clientIP"] = self.textClientIP.text()
    self.Log('info','setting client ip '+str(self.conn_params["clientIP"]))
    #self.Log('info','setting client ip '+str(self.conn_params["clientIP"]).split('.')[0]+'***'+'***'+str(self.conn_params["clientIP"]).split('.')[-1])

  def get_server_type(self):
    if self.multicast_radio.isChecked():
      self.conn_params["serverType"] = 'multicast'
      self.Log('info','setting broadcat to '+str(self.conn_params["serverType"]))
      self.get_multicast_addr()
    if self.unicast_radio.isChecked():
      self.conn_params["serverType"] = 'unicast'
      self.Log('info','setting broadcat to '+str(self.conn_params["serverType"]))

  def clicked_unicast(self):
    self.conn_params["serverType"] = 'unicast'
    self.textMulticastAddr.setEnabled(False)
    self.Log('info','setting broadcat to '+str(self.conn_params["serverType"]))

  def clicked_multicast(self):
    self.conn_params["serverType"] = 'multicast'
    self.textMulticastAddr.setEnabled(True)
    self.Log('info','setting broadcat to '+str(self.conn_params["serverType"]))

  def get_multicast_addr(self):
    self.conn_params["multicastAddress"] = self.textMulticastAddr.text()
    self.Log('info','setting multicast address '+str(self.conn_params["multicastAddress"]))

  def get_command_port(self):
    self.conn_params["serverCommandPort"] = int(self.textCommandPort.text())
    self.Log('info','setting command port '+str(self.conn_params["serverCommandPort"]))

  def get_data_port(self):
    self.conn_params["serverDataPort"] = int(self.textDataPort.text())
    self.Log('info','setting data port '+str(self.conn_params["serverDataPort"]))

  def get_world_frame(self):
    self.conn_params["globalFrame"] = self.textFrameName.text()
    if self.conn_params["globalFrame"]=='':
      self.conn_params["globalFrame"]='world'
      self.Log('warn','setting world frame name to world as no input provided')
    else:
      self.Log('info','setting world frame name '+str(self.conn_params["globalFrame"]))

  def spin_clientIP(self):
    self.textClientIP.setText(str(self.IP_LIST[self.client_ip_spin.value()-1]))
    self.conn_params["clientIP"] = str(self.IP_LIST[self.client_ip_spin.value()-1])
    self.textServerIP.setText('.'.join(str(self.IP_LIST[self.client_ip_spin.value()-1]).split('.')[:-1],)+'.')
    self.Log('info','setting client ip '+str(self.conn_params["clientIP"]))


#----------------------------------------------------------------------------------------
# MARKER POSE SERVER RELATED

  def set_lcds(self,num_of_markers,x_position,y_position,z_position):
    msg = '''
    It will only take upto 40 unlabled markers from the available list.
    If you do not see the marker in the list, make sure things other than markers are masked and markera are clearly visible.

    '''
    self.Log('block')
    self.Log('info',msg)
    print(len(x_position), len(y_position), len(z_position), num_of_markers)
    if (len(x_position) or len(y_position) or len(z_position))!=num_of_markers:
      self.Log('error',' Length of positions and number of detected markers are not matching, Press refresh to retry.')
    else:
      for i in range(min(40,num_of_markers)):
        eval('self.markerBox_'+str(i+1)+'.setEnabled(True)')
        eval('self.X_'+str(i+1)+'.display(x_position[i])')
        eval('self.Y_'+str(i+1)+'.display(y_position[i])')
        eval('self.Z_'+str(i+1)+'.display(z_position[i])')
      self.num_of_markers = num_of_markers
      self.x_position = x_position
      self.y_position = y_position
      self.z_position = z_position

  def yaml_dump(self):
    empty=0
    object_names={'object_names':[]}
    if self.num_of_markers!=0:
      for i in range(min(40,self.num_of_markers)):
        if eval('self.name_'+str(i+1)+'.text()') == '': empty+=1
        else:
          object_names['object_names'].append(eval('self.name_'+str(i+1)+'.text()'))
          object_names[object_names['object_names'][i-empty]]={'marker_config':0,
                                                    'pose':
                                                    {'position':[self.x_position[i],self.y_position[i],self.z_position[i]],
                                                    'orientation':[]}}
      self.natnet_params = object_names
      try:
        os.remove(os.path.join(self.config_file))
      except OSError:
        pass
      with open(self.config_file,'w') as f:
        yaml.dump(self.natnet_params,f,indent=2,default_flow_style=False)
        f.close()
      self.del_conn_params()
    else:
      self.Log('error','Number of markers are not recieved. Something went wrong.')

  def call_MarkerPoses_srv(self):
    self.set_conn_params()
    if self.error_pass:
      rospy.wait_for_service('get_marker_position')
      try:
        get_marker_position = rospy.ServiceProxy('get_marker_position', MarkerPoses)
        res = get_marker_position()
        self.set_lcds(res.num_of_markers,res.x_position,res.y_position,res.z_position)
        #self.set_lcds(5,[1.3545,5.45864521,0.4546264,8.8645413241],[45.864521,0.4546264,8.8645413241,128.4564531],[1.3545,0.4546264,8.8645413241,128.4564531])
      except rospy.ServiceException as e:
        self.Log('error','Service call failed: '+str(e))

#----------------------------------------------------------------------------------------
# PARAM CHECK RELATED

  def chek_conn_params(self):
    self.get_server_ip()
    if self.conn_params["serverIP"]==None or self.conn_params["serverIP"]=='':
      self.Log('error','No server ip provided. Can not connect.')
      self.error_pass=False
    self.get_client_ip()
    if self.conn_params["clientIP"]==None or self.conn_params["clientIP"]=='':
      self.Log('error','No client ip provided. Can not connect.')
      self.error_pass=False
    self.get_server_type()
    self.get_command_port()
    if self.conn_params["serverCommandPort"]==None or self.conn_params["serverCommandPort"]=='':
      self.Log('error','No command port provided. Can not connect.')
      self.error_pass=False
    self.get_data_port()
    if self.conn_params["serverDataPort"]==None or self.conn_params["serverDataPort"]=='':
      self.Log('error','No data port provided. Can not connect.')
      self.error_pass=False
    self.get_world_frame()

  def set_conn_params(self):
    self.chek_conn_params()
    if self.error_pass:
      rosparam.upload_params('conn_params',self.conn_params)

  def del_conn_params(self):
    if rospy.has_param("/conn_params/serverIP"):
      rospy.delete_param("/conn_params/serverIP")
    if rospy.has_param("/conn_params/clientIP"):
      rospy.delete_param("/conn_params/clientIP")
    if rospy.has_param("/conn_params/serverType"):
      if self.conn_params["serverType"]=='multicast' and rospy.has_param("/conn_params/multicastAddress"):
        rospy.delete_param("/conn_params/multicastAddress")
      rospy.delete_param("/conn_params/serverType")
    if rospy.has_param("/conn_params/serverCommandPort"):
      rospy.delete_param("/conn_params/serverCommandPort")
    if rospy.has_param("/conn_params/serverDataPort"):
      rospy.delete_param("/conn_params/serverDataPort")
    if rospy.has_param("/conn_params/globalFrame"):
      rospy.delete_param("/conn_params/globalFrame")

  def check_all_params(self):
    self.chek_conn_params()
    self.pub_im_setting()
    self.pub_rb_setting()
    self.pub_rbm_setting()
    self.pub_pc_setting()
    self.pub_params["pub_rigid_body_wmc"] = False
    self.log_frames_setting()
    self.log_internal_setting()
    self.log_latencies_setting()

  def set_other_params(self):
    try:
      if self.pub_params["pub_individual_marker"]:
        if self.natnet_params!=None:
          rosparam.upload_params(self.name, self.natnet_params)
        else:
          l = rosparam.load_file(self.config_file,self.name)
          rosparam.upload_params(l[0][1],l[0][0])
    except FileNotFoundError as e:
      self.error_pass = False
      self.Log('error',str(e))
    
    rosparam.upload_params(self.name,self.pub_params)
    rosparam.upload_params(self.name,self.log_params)

  def set_all_params(self):
    self.set_conn_params()
    self.set_other_params()

#----------------------------------------------------------------------------------------
# LOGGING RELATED

  def log_frames_setting(self):
    if self.log_frames.isChecked():
      self.log_params["log_frames"] = True
      self.Log('info','Enabled logging frames in terminal')
    else:
      self.log_params["log_frames"] = False

  def log_internal_setting(self):
    if self.log_internal.isChecked():
      self.log_params["log_internals"] = True
      self.Log('info','Enabled logging internal in terminal')
    else:
      self.log_params["log_internals"] = False

  def log_latencies_setting(self):
    if self.log_latencies.isChecked():
      self.log_params["log_latencies"] = True
      self.Log('info','Enabled logging latencies in terminal')
    else:
      self.log_params["log_latencies"] = False

## Main 
if __name__ == "__main__":
  rospy.init_node('natnet_helper',anonymous=True)
  app = QtWidgets.QApplication(sys.argv)
  window = PyQt5Widget()
  window.show()
  if rospy.is_shutdown():
    sys.exit(app.exec_())
  sys.exit(app.exec_())
