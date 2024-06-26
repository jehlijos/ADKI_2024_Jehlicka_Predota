# Form implementation generated from reading ui file 'settings.ui'
#
# Created by: PyQt6 UI code generator 6.6.1
#
# WARNING: Any manual changes made to this file will be lost when pyuic6 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt6 import QtCore, QtGui, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        self.Dialog = Dialog  # Store the Dialog instance as an attribute
        with open("settings.conf", "r") as file:
            # Reading the configuration file
            lines = file.readlines()
            # Extracting the values of zmin, zmax, and dz
            self.zmin = int(round(float((lines[0]))))
            self.zmax = int(round(float((lines[1]))))
            self.dz = int(round(float((lines[2]))))

        Dialog.setObjectName("Dialog")
        Dialog.resize(421, 309)
        Dialog.setWindowIcon(QtGui.QIcon("images/icons/settings.png"))
        self.Settings = QtWidgets.QDialogButtonBox(parent=Dialog)
        self.Settings.setGeometry(QtCore.QRect(110, 240, 201, 32))
        self.Settings.setOrientation(QtCore.Qt.Orientation.Horizontal)
        self.Settings.setStandardButtons(
            QtWidgets.QDialogButtonBox.StandardButton.Cancel | QtWidgets.QDialogButtonBox.StandardButton.Ok)
        self.Settings.setObjectName("Settings")
        self.groupBox = QtWidgets.QGroupBox(parent=Dialog)
        self.groupBox.setGeometry(QtCore.QRect(20, 10, 371, 211))
        self.groupBox.setObjectName("groupBox")
        self.label = QtWidgets.QLabel(parent=self.groupBox)
        self.label.setGeometry(QtCore.QRect(20, 50, 181, 16))
        self.label.setObjectName("label")
        self.lineEdit = QtWidgets.QLineEdit(parent=self.groupBox)
        self.lineEdit.setGeometry(QtCore.QRect(210, 50, 71, 20))
        self.lineEdit.setObjectName("lineEdit")
        self.label_2 = QtWidgets.QLabel(parent=self.groupBox)
        self.label_2.setGeometry(QtCore.QRect(290, 50, 47, 13))
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(parent=self.groupBox)
        self.label_3.setGeometry(QtCore.QRect(290, 100, 47, 13))
        self.label_3.setObjectName("label_3")
        self.lineEdit_2 = QtWidgets.QLineEdit(parent=self.groupBox)
        self.lineEdit_2.setGeometry(QtCore.QRect(210, 100, 71, 20))
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.label_4 = QtWidgets.QLabel(parent=self.groupBox)
        self.label_4.setGeometry(QtCore.QRect(20, 100, 171, 16))
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(parent=self.groupBox)
        self.label_5.setGeometry(QtCore.QRect(290, 150, 47, 13))
        self.label_5.setObjectName("label_5")
        self.lineEdit_3 = QtWidgets.QLineEdit(parent=self.groupBox)
        self.lineEdit_3.setGeometry(QtCore.QRect(210, 150, 71, 20))
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.label_6 = QtWidgets.QLabel(parent=self.groupBox)
        self.label_6.setGeometry(QtCore.QRect(20, 150, 161, 16))
        self.label_6.setObjectName("label_6")

        self.retranslateUi(Dialog)
        self.Settings.accepted.connect(self.saveSettings)  # type: ignore
        self.Settings.rejected.connect(Dialog.reject)  # type: ignore
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Settings"))
        self.groupBox.setTitle(_translate("Dialog", "Contour line propoerties"))
        self.label.setText(_translate("Dialog", "Minimum contour line  height"))
        self.label_2.setText(_translate("Dialog", "m"))
        self.label_3.setText(_translate("Dialog", "m"))
        self.label_4.setText(_translate("Dialog", "Maximum contour line  height"))
        self.label_5.setText(_translate("Dialog", "m"))
        self.label_6.setText(_translate("Dialog", "Contour line height interval"))

        self.lineEdit.setText(str(self.zmin))
        self.lineEdit_2.setText(str(self.zmax))
        self.lineEdit_3.setText(str(self.dz))

    def getZmin(self):
        # Get the minimum contour line height
        self.zmin = float(self.lineEdit.text())
        return self.zmin

    def getZmax(self):
        # Get the maximum contour line height
        self.zmax = float(self.lineEdit_2.text())
        return self.zmax

    def getDz(self):
        # Get the contour line height interval
        self.dz = float(self.lineEdit_3.text())
        return self.dz

    def saveSettings(self):
        # Get the new settings
        zmin = self.getZmin()
        zmax = self.getZmax()
        dz = self.getDz()

        # Write the new settings to the settings.conf file
        with open("settings.conf", "w") as file:
            file.write(f"{zmin}\n{zmax}\n{dz}\n")

        # Close the dialog
        self.Dialog.accept()


if __name__ == "__main__":
    import sys

    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    Dialog.show()
    sys.exit(app.exec())
