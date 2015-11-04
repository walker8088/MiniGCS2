
import sys

# This is only needed for Python v2 but is harmless for Python v3.
import sip
sip.setapi('QVariant', 2)

from PyQt4.QtCore import *
from PyQt4.QtGui import *

from PyMiniGCS import MainWindow

app =  QApplication(sys.argv)
mainWin = MainWindow()
mainWin.show()
sys.exit(app.exec_())
