import os
import rospy
import rospkg

from functools import partial

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QLabel, QTableWidgetItem, QTreeWidgetItem, QWidget, QCheckBox, QComboBox, QLineEdit, QDialog, QSizePolicy

import skiros2_common.tools.logger as log
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
import skiros2_world_model.ros.world_model_interface as wmi
import skiros2_skill.ros.skill_layer_interface as sli
from copy import deepcopy

class SkirosAddObjectDialog(QDialog):
#==============================================================================
#  Modal dialog
#==============================================================================
    def __init__(self, *args, **kwargs):
        super(SkirosAddObjectDialog, self).__init__(*args, **kwargs)
        self.setObjectName('SkirosAddObjectDialog')
        ui_file = os.path.join(rospkg.RosPack().get_path('skiros2_gui'), 'src/skiros2_gui/core', 'skiros_gui_add_object_dialog.ui')
        loadUi(ui_file, self)

        self._comboBoxes = []
        self.create_comboBox(label='Type')
        self.comboBox_individual.clear()
        self.comboBox_individual.addItems(self.get_individuals())


    def get_types(self, subtype='sumo:Object'):
        return self.parent()._wmi.getSubClasses(subtype, False)


    def on_select_type(self, id, index):
        # log.debug(self.__class__.__name__, 'Selected {}: {}'.format(id, index))
        while id < len(self._comboBoxes)-1:
            # log.debug(self.__class__.__name__, 'Delete {}'.format(id+1))
            label = self.formLayout.labelForField(self._comboBoxes[id+1])
            label.deleteLater()
            self._comboBoxes[id+1].deleteLater()
            del self._comboBoxes[id+1]
        if index > 0:
            selected = self._comboBoxes[id].currentText()
            self.create_comboBox(selected)
            # log.debug(self.__class__.__name__, 'Created {}'.format(len(self._comboBoxes)-1))
        else:
            selected = 'sumo:Object'
        self.comboBox_individual.clear()
        self.comboBox_individual.addItems(self.get_individuals(selected))
        QTimer.singleShot(0, self.adjustSize)


    def create_comboBox(self, subtype='sumo:Object', label='Filter'):
        types = self.get_types(subtype)
        if not types: return None
        comboBox = QComboBox()
        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        comboBox.setSizePolicy(sizePolicy)
        comboBox.addItem('All')
        comboBox.addItems(types)
        comboBox.currentIndexChanged.connect(partial(self.on_select_type, len(self._comboBoxes)))
        self.formLayout.insertRow(len(self._comboBoxes), label, comboBox)
        self._comboBoxes.append(comboBox)


    def get_individuals(self, subtype='sumo:Object'):
        return self.parent()._wmi.getIndividuals(subtype, True)




class SkirosWidget(QWidget):
#==============================================================================
#  General
#==============================================================================
    def __init__(self):
        super(SkirosWidget, self).__init__()
        self.setObjectName('SkirosWidget')
        ui_file = os.path.join(rospkg.RosPack().get_path('skiros2_gui'), 'src/skiros2_gui/core', 'skiros_gui.ui')
        loadUi(ui_file, self)
        self.reset()

    def reset(self):
        #The plugin should not call init_node as this is performed by rqt_gui_py.
        #Due to restrictions in Qt, you cannot manipulate Qt widgets directly within ROS callbacks,
        #because they are running in a different thread.
        self._wmi = wmi.WorldModelInterface()
        self._sli = sli.SkillLayerInterface(self._wmi)
        self._author_name = "skiros_gui"

        #Setup a timer to keep interface updated
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self.refresh_timer_cb)
        self.refresh_timer.start(1000)
        #World model tab
        #node_name = "Whatever"
        #top_level_item = self._recursive_create_widget_items(None, node_name)

        # add top level item to tree widget
        #self.wm_tree_widget.addTopLevelItem(top_level_item)

        #self.wm_tree_widget.currentItemChanged.connect(self.wm_tree_widget_currentItemChanged)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
        pass



#==============================================================================
#  General
#==============================================================================
    def refresh_timer_cb(self):
        """
        Keeps interface updated
        """
        if self._sli.hasChanges():
            self.skill_combo_box.clear()
            for ak, e in self._sli._agents.iteritems():
                for sk, s in e._skill_list.iteritems():
                    s.manager = ak
                    self.skill_combo_box.addItem(sk, s)

#==============================================================================
#  World model tab
#==============================================================================

    @Slot()
    def on_add_object_button_clicked(self):
        dialog = SkirosAddObjectDialog(self)
        ret = dialog.exec_()
        print(ret)


    @Slot()
    def on_modify_object_button_clicked(self):
        print "modify_object_button"

    @Slot()
    def on_remove_object_button_clicked(self):
        print "remove_object_button"


    @Slot()
    def on_load_scene_button_clicked(self):
        file = self.scene_file_lineEdit.text()
        log.debug(self.__class__.__name__, 'Loading world model from <{}>'.format(file))
        self._wmi.load(file)


    @Slot()
    def on_save_scene_button_clicked(self):
        file = self.scene_file_lineEdit.text()
        log.debug(self.__class__.__name__, 'Saving world model to <{}>'.format(file))
        self._wmi.save(file)


    @Slot('QTreeWidgetItem*', 'QTreeWidgetItem*')
    def wm_tree_widget_currentItemChanged(self, item, prev_item):
        while self.wm_properties_widget.rowCount()>0:
           self.wm_properties_widget.removeRow(0)
        item = QTableWidgetItem(str(item.text(0)))
        self.wm_properties_widget.insertRow(0)
        self.wm_properties_widget.setItem(0, 0, item)
        while self.wm_relations_widget.rowCount()>0:
           self.wm_relations_widget.removeRow(0)
        item = QTableWidgetItem("")
        self.wm_relations_widget.insertRow(0)
        self.wm_relations_widget.setItem(0, 0, item)


    def _recursive_create_widget_items(self, parent, node_name):
        item = QTreeWidgetItem(parent)
        #if is_editable:
         #   item.setFlags(item.flags() | Qt.ItemIsEditable)

        item.setText(0, node_name)
        #item.setText(self._column_index['type'], type_name)

        #item.setData(0, Qt.UserRole, topic_name)

        #if hasattr(message, '__slots__') and hasattr(message, '_slot_types'):
        #    for slot_name, type_name in zip(message.__slots__, message._slot_types):
        #        self._recursive_create_widget_items(item, topic_name + '/' + slot_name, type_name, getattr(message, slot_name), is_editable)

        item.setText(1, "Wgerever")

        item2 = QTreeWidgetItem(item)
        item2.setText(0, "IIIII")
        item3 = QTreeWidgetItem(item2)
        item3.setText(0, "IIIII")
        return item

#==============================================================================
# Task tab
#==============================================================================

#==============================================================================
#     @Slot(str)
#     def on_service_combo_box_currentIndexChanged(self, service_name):
#         pass
#
#         # add top level item to tree widget
#         self.wm_tree_widget.addTopLevelItem(top_level_item)
#
#         # resize columns
#         self.wm_tree_widget.expandAll()
#         for i in range(self.request_tree_widget.columnCount()):
#             self.wm_tree_widget.resizeColumnToContents(i)
#==============================================================================

#==============================================================================
# Goal
#==============================================================================


#==============================================================================
# Skill
#==============================================================================
    def _getParameters(self, layout, params):
        i = -1
        for k, param in params.iteritems():
            if not self.modality_checkBox.isChecked() and (param.paramTypeIs(ParamTypes.Optional) or param.paramTypeIs(ParamTypes.System)):
                continue
            i += 1
            widget = layout.itemAtPosition(i, 1).widget()
            if param.dataTypeIs(bool):
                param.setValue(widget.isChecked())
            elif param.dataTypeIs(Element):
                data = widget.itemData(widget.currentIndex())
                if data:
                    param.setValue(self._wmi.getElement(data))
            else:
                try:
                    param.setValueFromStr(widget.text())
                except ValueError:
                    log.error("getParameters", "Failed to set param {}".format(param._key))
                    return False
        return True


    def _addParameter(self, layout, row, param):
        layout.setColumnStretch(2, 10)
        layout.setRowMinimumHeight(row, 2)
        layout.addWidget(QLabel(param._key), row, 0)
        if param.dataTypeIs(bool):
            cbox = QCheckBox()
            if param.isSpecified():
                cbox.setChecked(param.getValue())
            layout.addWidget(cbox, row, 1)
        elif param.dataTypeIs(Element):
            combobox = QComboBox()
            layout.addWidget(combobox, row, 1)
            matches = self._wmi.resolveElements(param.getValue())
            for e in matches:
                combobox.addItem(e.printState(), e._id)
        else:
            lineedit = QLineEdit()
            if param.isSpecified():
                lineedit.setText(str(param.getValue()))
            layout.addWidget(lineedit, row, 1)

    @Slot(str)
    def on_skill_combo_box_currentIndexChanged(self, skill_name):
        skill = self.skill_combo_box.itemData(self.skill_combo_box.currentIndex())
        if skill:
            #Clean
            while self.skill_params_layout.count():
                item = self.skill_params_layout.takeAt(0)
                item.widget().deleteLater()
            #Add params
            i = 0
            for _, p in skill.ph.iteritems():
                if self.modality_checkBox.isChecked() or not (p.paramTypeIs(ParamTypes.Optional) or p.paramTypeIs(ParamTypes.System)):
                    self._addParameter(self.skill_params_layout, i, p)
                    i += 1

    @Slot()
    def on_modality_checkBox_clicked(self):
        self.on_skill_combo_box_currentIndexChanged("")

    @Slot()
    def on_skill_exe_button_clicked(self):
        skill = deepcopy(self.skill_combo_box.itemData(self.skill_combo_box.currentIndex()))
        if self._getParameters(self.skill_params_layout, skill.ph):
            self._curr_task = (skill.manager, self._sli.getAgent(skill.manager).execute([skill], self._author_name))

    @Slot()
    def on_skill_stop_button_clicked(self):
        self._sli.getAgent(self._curr_task[0]).preempt(self._curr_task[1], self._author_name)

