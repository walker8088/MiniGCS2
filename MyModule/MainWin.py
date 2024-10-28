#coding: utf-8
import sys

from PySide6 import *
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *

from .FirmwareUploader import *
from .ConnectionMgr import *
from .AttitudeView import *

from .utils import *

app = None

#-------------------------------------------------#

class ParamTableModel(QAbstractTableModel):
    def __init__(self, parent=None, *args):
        QAbstractTableModel.__init__(self, parent, *args)
        self.headerdata = [
            '序号', '参数ID', '参数类型', "参数值", "保存值", "取值范围", "单位", "参数名称"
        ]
        self.clear()

    def clear(self):
        self.params = []

    def set_total(self, count):
        self.layoutAboutToBeChanged.emit()
        to_be_new = count - len(self.params)
        if to_be_new > 0:
            self.params.extend([None for x in range(to_be_new)])
        self.layoutChanged.emit()

    def update(self, param):
        #print param[0], param[1]
        index = int(param[0])
        new_index = 0
        if index == 65535:
            for p in self.params:
                if p[1] == param[1]:
                    p[3] = param[3]
                    break
                new_index += 1
        else:
            self.params[index] = param
            new_index = index

        row_index = self.createIndex(new_index, 3)
        self.dataChanged.emit(row_index, row_index)

    def rowCount(self, parent):
        return len(self.params)

    def columnCount(self, parent):
        return len(self.headerdata)

    def data(self, index, role):
        if not index.isValid():
            return None
        elif role != Qt.DisplayRole:
            return None
        param = self.params[index.row()]
        if param:
            return param[index.column()]
        else:
            return None

    def headerData(self, col, orientation, role):
        if orientation == Qt.Horizontal and role == Qt.DisplayRole:
            return self.headerdata[col]
        return None

    def flags(self, index):
        if index.column() == 4:
            return Qt.ItemIsSelectable | Qt.ItemIsEditable | Qt.ItemIsEnabled
        else:
            return Qt.ItemIsSelectable | Qt.ItemIsEnabled

    def sort(self, Ncol, order):
        """Sort table by given column number.
        """
        self.emit(SIGNAL("layoutAboutToBeChanged()"))
        self.params = sorted(self.params, key=operator.itemgetter(Ncol))
        if order == Qt.DescendingOrder:
            self.params.reverse()
        self.emit(SIGNAL("layoutChanged()"))


#-------------------------------------------------#
class MainWindow(QMainWindow):
    info_signal = Signal(str)
    params_done_signal = Signal()
    status_signal = Signal(str)
    show_msg_signal = Signal((str, str))
    accel_cal_signal = Signal((str, int))

    show_param_signal = Signal((int, int, str, int, str))
    update_attitude_signal = Signal((float, float, float))
    update_gps_raw_int_signal = Signal((int, int, int, int, int, int))

    def __init__(self):
        super(MainWindow, self).__init__()

        self.setWindowTitle("MiniGCS")

        self.tab = QTabWidget()
        self.tab.setTabPosition(QTabWidget.South)

        self.infoView = QTextEdit()
        self.attitudeView = AttitudeWidget()

        splitter = QSplitter(Qt.Vertical)

        splitter.addWidget(self.tab)

        h_splitter = QSplitter()
        h_splitter.addWidget(self.infoView)
        h_splitter.addWidget(self.attitudeView)

        splitter.addWidget(h_splitter)

        #layout = QVBoxLayout()
        #layout.addWidget(splitter)

        self.setCentralWidget(splitter)
        #self.setLayout(layout)

        self.paramView = QTableView()

        # set the table model
        header = [
            '序号', '参数ID', '参数类型', "参数值", "保存值", "取值范围", "单位", "参数名称"
        ]
        self.paramModel = ParamTableModel(self)
        self.paramView.setModel(self.paramModel)
        #self.paramView.verticalHeader().setVisible(False)
        '''
        self.paramView.setColumnWidth(0,35)
        self.paramView.setColumnWidth(1,150)
        self.paramView.setColumnWidth(2,50)
        self.paramView.setColumnWidth(3,80)
        self.paramView.setColumnWidth(4,80)
        self.paramView.setColumnWidth(5,100)
        self.paramView.setColumnWidth(6,100)
        self.paramView.setColumnWidth(7,400)
         '''
        #self.paramView.setSelectionMode(QTableWidget.SingleSelection)
        #self.paramView.setEditTriggers(QAbstractItemView.NoEditTriggers)
        #self.paramView.setAlternatingRowColors(True)

        self.msgView = QTableWidget(10, 2)
        self.msgView.setHorizontalHeaderLabels(['消息ID', "消息内容"])
        self.msgView.setColumnWidth(0, 180)
        self.msgView.setColumnWidth(1, 1000)
        #self.msgView.setColumnWidth(2,900)

        self.msgView.setSelectionMode(QTableWidget.SingleSelection)
        self.msgView.setAlternatingRowColors(True)
        self.msgView.verticalHeader().setVisible(False)

        self.msg_dict = {}

        self.tab.addTab(self.msgView, "系统状态")
        self.tab.addTab(self.paramView, "参数列表")

        self.createActions()
        self.createMenus()
        self.createToolBars()
        self.createStatusBar()

        self.params_info = {}
        if os.path.isfile("param_def.xml"):
            self.load_param_doc("param_def.xml")

        self.readSettings()

        self.info_signal.connect(self.on_info)
        self.params_done_signal.connect(self.on_params_done)
        self.status_signal.connect(self.on_status)
        self.show_msg_signal.connect(self.on_show_msg)
        self.show_param_signal.connect(self.on_show_param)
        self.update_attitude_signal.connect(self.on_update_attitude)
        self.update_gps_raw_int_signal.connect(self.on_update_gps_raw_int)
        self.accel_cal_signal.connect(self.on_accel_cal)

        self.last_info = None
        self.conn_mgr = None

        self.onPortDetect()

        self.accel_cal_count = -1

    def info(self, msg):
        self.info_signal.emit(msg)

    def params_done(self):
        self.params_done_signal.emit()

    def info_status(self, status):
        self.status_signal.emit(status)

    def show_param(self, total, index, name, type, value):
        self.show_param_signal.emit(total, index, name, type, value)

    def show_msg(self, id, text):
        self.show_msg_signal.emit(id, text)

    def update_attitude(self, pitch, roll, yaw):
        self.update_attitude_signal.emit(pitch, roll, yaw)

    def update_gps_raw_int(self, fix_type, lat, lon, alt, eph,
                           satellites_visible):
        self.update_gps_raw_int_signal.emit(fix_type, lat, lon, alt, eph,
                                            satellites_visible)

    def update_accel_cal(self, text, count):
        self.accel_cal_signal.emit(text, count)

    #-------------------------------------------------------------#
    #slots
    #@pyqtSlot(str)
    def on_status(self, status):
        pass

    #@pyqtSlot(str)
    def on_info(self, msg):
        if self.last_info and self.last_info == msg:
            return
        self.infoView.append(msg)
        self.infoView.repaint()
        self.last_info = msg

    #@pyqtSlot(str, str)
    def on_show_msg(self, msg_id, msg_text):
        if msg_id not in self.msg_dict:
            self.msgView.setRowCount(len(self.msg_dict) + 1)
            item = QTableWidgetItem(msg_text)
            index = len(self.msg_dict)

            self.msgView.setItem(index, 0, QTableWidgetItem(msg_id))
            self.msgView.setItem(index, 1, item)

            self.msg_dict[msg_id] = item

        else:
            item = self.msg_dict[msg_id]
            item.setText(msg_text)

    #@pyqtSlot()
    def on_params_done(self):
        self.paramView.resizeColumnsToContents()

    #@pyqtSlot(int, int, str, int, str)
    def on_show_param(self, total, index, param_id, type, value):
        type_s = ("unknown", "u_int_8", "int_8", "u_int_16", "int_16",
                  "u_int_32", "int_32", "u_int_64", "real_32", "real_64")

        self.paramModel.set_total(total)

        #[u'序号', u'参数ID', u'参数类型', u"参数值", u"保存值", u"取值范围", u"单位",  u"参数名称"]
        param = [
            str(index), param_id, type_s[type],
            str(value), '', '', '', '', ''
        ]
        p_id = str(param_id)
        if p_id in self.params_info:
            param[5] = self.params_info[p_id][0]
            param[6] = self.params_info[p_id][1]
            param[7] = self.params_info[p_id][2]
            param[8] = self.params_info[p_id][3]

        self.paramModel.update(param)
        #if str(param_id).startswith('ATC_RAT_'):
        #    print(param_id, value)

    #@pyqtSlot(float, float, float)
    def on_update_attitude(self, pitch, roll, yaw):
        #print pitch, roll, yaw
        self.attitudeView.update_attitude(pitch, roll, yaw)

    #@pyqtSlot(int, int, int, int, int, int)
    def on_update_gps_raw_int(self, fix_type, lat, lon, alt, eph,
                              satellites_visible):
        gps_types = {0: '无GPS', 1: 'GPS通讯正常', 2: 'GPS啥状态？', 3: 'GPS锁定'}
        float_ratio = 10000000

        if fix_type < 2:
            msg = gps_types[fix_type]
        else:
            msg = '%s 经度：%f 纬度：%f 高度：%f 精度:%.2f米 卫星数：%d' \
              % (gps_types[fix_type],float(lat)/float_ratio, float(lon)/float_ratio, float(alt)/float_ratio, eph/100.0, satellites_visible)

        self.statusBar().showMessage(msg)

    #@pyqtSlot(str)
    def onPortViewIndexChanged(self, new_value):
        if len(new_value) == 0:
            self.connectAct.setEnabled(False)
        elif not self.conn_mgr:
            self.connectAct.setEnabled(True)

    #@pyqtSlot(str, int)
    def on_accel_cal(self, text, count):
        self.accel_cal_count = count
        self.calibrationAct.setText("下一步")

        if 'level' in text:
            info = "请将机身水平放置后，点击“下一步”按钮"
        elif 'BACK' in text:
            info = "请将机身水平翻转放置后，点击“下一步”按钮"
        elif 'LEFT' in text:
            info = "请将机身左侧朝下放置后，点击“下一步”按钮"
        elif 'RIGHT' in text:
            info = "请将机身右侧朝下放置后，点击“下一步”按钮"
        elif 'UP' in text:
            info = "请将机头朝上放置后，点击“下一步”按钮"
        elif 'DOWN' in text:
            info = "请将机头朝下放置后，点击“下一步”按钮"
        else:
            info = text

        self.info(info)

    #slots end
    #-------------------------------------------------------------#

    def onPortDetect(self):

        ports = get_all_comports()
        self.portView.clear()
        for port in ports:
            self.portView.addItem(port)

    def onConnect(self):

        if (not self.conn_mgr) or (not self.conn_mgr.running):
            try:
                self.conn_mgr = ConnectionManager(self)
                self.conn_mgr.open_comm(str(self.portView.currentText()),
                                        int(self.baudView.currentText()))
            except:
                self.conn_mgr = None
                return
            self.connectAct.setText("断开连接")
            #self.connectUdpAct.setEnabled(False)
            self.paramsMgrAct.setEnabled(True)
            self.portDetectAct.setEnabled(False)
            self.calibrationAct.setEnabled(True)
            self.calibrationMagAct.setEnabled(True)
            self.burnRomAct.setEnabled(False)
        else:
            self.connectAct.setText("连接飞控")
            self.conn_mgr.close()
            self.conn_mgr = None
            #self.connectUdpAct.setEnabled(True)
            self.paramsMgrAct.setEnabled(False)
            self.portDetectAct.setEnabled(True)
            self.calibrationAct.setEnabled(False)
            self.calibrationMagAct.setEnabled(False)
            self.burnRomAct.setEnabled(True)
    '''
    def onConnectUdp(self):
        if (not self.conn_mgr) or (not self.conn_mgr.running):
            try:
                self.conn_mgr = ConnectionManager(self)
                self.conn_mgr.open_tcp('192.168.1.63', 5762)
                #self.conn_mgr.open_udp('192.168.43.1', 14550)

            except:
                self.conn_mgr = None
                return
            #self.connectUdpAct.setText("断开连接")
            self.connectAct.setEnabled(False)
            self.paramsMgrAct.setEnabled(True)
            self.portDetectAct.setEnabled(False)
            self.calibrationAct.setEnabled(True)
            self.calibrationMagAct.setEnabled(True)
            self.burnRomAct.setEnabled(False)
        else:
            #self.connectUdpAct.setText("连接奥丁")
            self.conn_mgr.close()
            self.conn_mgr = None
            if len(str(self.portView.currentText())) > 0:
                self.connectAct.setEnabled(True)
            self.paramsMgrAct.setEnabled(False)
            self.portDetectAct.setEnabled(True)
            self.calibrationAct.setEnabled(False)
            self.calibrationMagAct.setEnabled(False)
            self.burnRomAct.setEnabled(True)
    '''
    def onBurnRom(self):
        port = str(self.portView.currentText())
        fileName = QFileDialog.getOpenFileName(self, "选择固件文件", "",
                                               "Firmware Image Files(*.px4)")
        if not fileName:
            return

        QMessageBox.information(self, '操作提示',
                                '把未安装存储卡的飞控从USB口接入本台电脑，然后点击“OK”按钮',
                                QMessageBox.Ok)

        do_upload(fileName, port, self)

    def onWriteParams(self):

        #fileName =  QFileDialog.getOpenFileName(self,  u"选择参数文件", "", "Param Files(*.param)")
        #if not fileName:
        #        return

        fileName = 'odin.param'
        with open(fileName) as f:
            params = f.readlines()

        error_count = 0
        for line in params:
            it = line.strip().split(",")

            if (len(it) == 0) or line.startswith("#"):
                continue

            if len(it) != 2:
                self.info("参数格式错误：%s" % line)
                continue

            times = 0
            while times < 3:
                #if times > 0:
                #	self.info(u"！正在重写参数 %s  %s" % (it[0], it[1]) )
                app.processEvents()
                if self.conn_mgr.mav_set_param(it[0], float(it[1])):
                    self.info("参数写入成功  %s  %s" % (it[0], it[1]))
                    break
                elif times >= 2:
                    self.info("！参数写入失败 %s  %s" % (it[0], it[1]))
                    error_count += 1
                times += 1
                app.processEvents()

        if error_count == 0:
            self.info("参数文件%s写入成功" % (fileName, ))
        else:
            self.info("参数文件%s中%d个参数写入失败，请检查硬件。" % (fileName, error_count))

    def onCalibration(self):
        if self.accel_cal_count == -1:
            self.conn_mgr.cmd_accel_cal()
        else:
            self.conn_mgr.accel_cal_ready(self.accel_cal_count)

        if self.accel_cal_count == 6:
            self.accel_cal_count == -1
            self.calibrationAct.setText("姿态校准")

    def onCalibrationMag(self):
        self.conn_mgr.cmd_mag_cal('start')

    def load_param_doc(self, file_name):
        pass
        '''
	xml = open(file_name).read()
        objectify.enable_recursive_str()
        tree = objectify.fromstring(xml)
        param_tree = {}
        for p in tree.vehicles.parameters.param:
            n = p.get('name') #.split(':')
            param_tree[n] = p
            
        for lib in tree.libraries.parameters:
            for p in lib.param:
                n = p.get('name')
                param_tree[n] = p
    
        self.params_info = {}
        
        for key in param_tree:        
            node =  param_tree[key]
            text_hname = node.get("humanName")
            doc = node.get("documentation")
            text_range = ""
            text_unit = ""
            for child_node in node.iterchildren("field"):
                    name = child_node.get("name")
                    if  name == "Range" :
                        text_range = str(child_node).replace(" ",  "--")
                    elif name == "Increment" :  
                        text_range += "(step %s)" % str(child_node)     
                    elif name == "Units" :
                        text_unit = str(child_node)         
            self.params_info[key.strip()] = (text_range, text_unit, text_hname, doc)
        '''

    def createActions(self):

        self.portDetectAct = QAction("检测串口",
                                     self,
                                     triggered=self.onPortDetect)
        self.burnRomAct = QAction("烧写ROM", self, triggered=self.onBurnRom)
        self.connectAct = QAction("连接飞控", self, triggered=self.onConnect)
        #self.connectUdpAct = QAction("连接奥丁",
        #                             self,
        #                             triggered=self.onConnectUdp)
        self.paramsMgrAct = QAction("参数写入",
                                    self,
                                    triggered=self.onWriteParams)
        self.calibrationAct = QAction("姿态校准",
                                      self,
                                      triggered=self.onCalibration)
        self.calibrationMagAct = QAction("磁罗盘校准",
                                         self,
                                         triggered=self.onCalibrationMag)

        #self.connectAct.setEnabled(False)
        self.paramsMgrAct.setEnabled(False)
        self.calibrationAct.setEnabled(False)
        self.calibrationMagAct.setEnabled(False)

        self.exitAct = QAction("结束退出",
                               self,
                               shortcut="Ctrl+Q",
                               statusTip="Exit the application",
                               triggered=self.close)

        self.aboutAct = QAction("&A关于...", self, triggered=self.about)

    def createMenus(self):
        self.fileMenu = self.menuBar().addMenu("&File")

        self.fileMenu.addSeparator()
        self.fileMenu.addAction(self.exitAct)

        self.menuBar().addSeparator()

        self.helpMenu = self.menuBar().addMenu("&Help")
        self.helpMenu.addAction(self.aboutAct)

    def createToolBars(self):

        self.portView = QComboBox()
        #self.portView.currentIndexChanged['QString'].connect(
        #    self.onPortViewIndexChanged)

        self.baudView = QComboBox()
        self.baudView.addItem("25600")
        self.baudView.addItem("38400")
        self.baudView.addItem("57600")
        self.baudView.addItem("115200")
        self.baudView.setCurrentIndex(3)

        self.toolBar = self.addToolBar("")

        self.toolBar.addAction(self.portDetectAct)

        self.toolBar.addWidget(self.portView)

        self.toolBar.addWidget(QLabel("  波特率："))
        self.toolBar.addWidget(self.baudView)

        self.toolBar.addAction(self.connectAct)
        #self.toolBar.addAction(self.connectUdpAct)

        self.toolBar.addAction(self.paramsMgrAct)
        self.toolBar.addAction(self.calibrationAct)
        self.toolBar.addAction(self.calibrationMagAct)

        self.toolBar.addAction(self.burnRomAct)

        self.toolBar.addAction(self.exitAct)

    def createStatusBar(self):
        self.statusBar().showMessage("Ready")

    def readSettings(self):
        settings = QSettings("Trolltech", "Application Example")
        pos = settings.value("pos", QPoint(200, 200))
        size = settings.value("size", QSize(400, 400))
        self.resize(size)
        self.move(pos)

    def writeSettings(self):
        settings = QSettings("Trolltech", "Application Example")
        settings.setValue("pos", self.pos())
        settings.setValue("size", self.size())

    def loadFile(self, fileName):
        file = QFile(fileName)
        if not file.open(QtCore.QFile.ReadOnly | QFile.Text):
            QMessageBox.warning(
                self, "Application",
                "Cannot read file %s:\n%s." % (fileName, file.errorString()))
            return

        inf = QTextStream(file)
        QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        self.info.setPlainText(inf.readAll())
        QApplication.restoreOverrideCursor()

        self.setCurrentFile(fileName)
        self.statusBar().showMessage("File loaded", 2000)

    def saveFile(self, fileName):
        file = QFile(fileName)
        if not file.open(QtCore.QFile.WriteOnly | QFile.Text):
            QMessageBox.warning(
                self, "Application",
                "Cannot write file %s:\n%s." % (fileName, file.errorString()))
            return False

        outf = QTextStream(file)
        QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        outf << self.info.toPlainText()
        QApplication.restoreOverrideCursor()

        self.setCurrentFile(fileName)
        self.statusBar().showMessage("File saved", 2000)
        return True

    def about(self):
        QMessageBox.about(self, "About MiniGCS", "MiniGCS  V1.0 By Walker Li")

    def closeEvent(self, event):
        #event.ignore()
        if self.conn_mgr and self.conn_mgr.running:
            self.conn_mgr.close()


#-------------------------------------------------#
def main():
    global app
    app = QApplication(sys.argv)
    mainWin = MainWindow()
    mainWin.show()
    sys.exit(app.exec_())
