import os
import rospy
import rospkg

from functools import partial

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QLabel, QTableWidgetItem, QTreeWidgetItem, QWidget, QCheckBox, QComboBox, QLineEdit, QDialog, QSizePolicy

import skiros2_common.tools.logger as log
import skiros2_common.core.utils as utils
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
import skiros2_world_model.ros.world_model_interface as wmi
import skiros2_skill.ros.skill_layer_interface as sli
from copy import deepcopy

class SkirosAddObjectDialog(QDialog):
#==============================================================================
#  Modal dialog
#==============================================================================

    default_type = 'sumo:Object'

    def __init__(self, *args, **kwargs):
        """Implements a dialog to create a new object for the world model.

        Implementation of the modal dialog to select object types from the available ontology/world model.
        Allows filtering of the objects by (sub)type.
        The dialog saves the selection in the 'object' property.

        Args:
            *args: Description
            **kwargs: Description
        """
        super(SkirosAddObjectDialog, self).__init__(*args, **kwargs)
        self.setObjectName('SkirosAddObjectDialog')
        ui_file = os.path.join(rospkg.RosPack().get_path('skiros2_gui'), 'src/skiros2_gui/core', 'skiros_gui_add_object_dialog.ui')
        loadUi(ui_file, self)

        self._comboBoxes = []
        self.create_comboBox(label='Type')
        self.comboBox_individual.clear()
        [self.comboBox_individual.addItem(l,d) for l,d in self.get_individuals(self.default_type).iteritems()]


    @property
    def object(self):
        """Access to the currently selected object type.

        Returns:
            str: Selected (ontology) type (e.g. skiros:Product)
        """
        return self.comboBox_individual.itemData(self.comboBox_individual.currentIndex())


    def on_select_type(self, id, index):
        """Callback for change selection in dropdown lists.

        Adds and removes dropdown list to/from the dialog that are used to filter subtypes based on the current selection.

        Args:
            id (int): Number of the combobox that dispatched the callback
            index (int): Number of the selected item in the current combobox (id)
        """
        # log.debug(self.__class__.__name__, 'Selected {}: {}'.format(id, index))

        # clear filters after selected
        while id < len(self._comboBoxes)-1:
            # log.debug(self.__class__.__name__, 'Delete {}'.format(id+1))
            label = self.formLayout.labelForField(self._comboBoxes[id+1])
            label.deleteLater()
            self._comboBoxes[id+1].deleteLater()
            del self._comboBoxes[id+1]

        # get current selection
        if index > 0: # if not 'All' is selected
            selected = self._comboBoxes[id].itemData(self._comboBoxes[id].currentIndex())
        elif id > 0: # if 'All' is selected and it is not the first combo box
            selected = self._comboBoxes[id-1].itemData(self._comboBoxes[id-1].currentIndex())
        else:  # if 'All' is selected and it is the first combo box
            selected = self.default_type
        # log.debug(self.__class__.__name__, 'Selected type {}'.format(selected))

        # create new combo box if not 'All' is selected
        if index > 0:
            self.create_comboBox(selected)
            # log.debug(self.__class__.__name__, 'Created {}'.format(len(self._comboBoxes)-1))

        # update list of individuals
        self.comboBox_individual.clear()
        if index > 0 or (id > 0 and index == 0):
            self.comboBox_individual.addItem('new ' + utils.ontology_type2name(selected))
        [self.comboBox_individual.addItem(l,d) for l,d in self.get_individuals(selected).iteritems()]
        QTimer.singleShot(0, self.adjustSize)


    def create_comboBox(self, subtype='sumo:Object', label='Subtype'):
        """Inserts a new combobox in the dialog based on the subtype.

        Helper function that creates a combobox and fills the list with filtered items from the ontology/world model.

        Args:
            subtype (str, optional): Type to be used to retrieve world model items for the dropdown list
            label (str, optional): Label for the dropdown list
        """
        comboBox = QComboBox()
        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        comboBox.setSizePolicy(sizePolicy)
        comboBox.addItem('All')
        [comboBox.addItem(l,d) for l,d in self.get_types(subtype).iteritems()]
        comboBox.currentIndexChanged.connect(partial(self.on_select_type, len(self._comboBoxes)))
        self.formLayout.insertRow(len(self._comboBoxes), label, comboBox)
        self._comboBoxes.append(comboBox)


    def get_types(self, subtype):
        """Retrieves available subtype from the ontology.

        Args:
            subtype (str): Filter for object types

        Returns:
            dict(str, str): Keys: Short type name. Values: Type identifier (e.g. {'Product': 'skiros:Product'})
        """
        return utils.ontology_type2name_dict(self.parent()._wmi.getSubClasses(subtype, False))


    def get_individuals(self, subtype):
        """Retrieves available individuals from the world model.

        Args:
            subtype (str): Filter for object types

        Returns:
            dict(str, str): Keys: Short type name. Values: Type identifier (e.g. {'starter': 'skiros:starter'})
        """
        return utils.ontology_type2name_dict(self.parent()._wmi.getIndividuals(subtype, True))




class SkirosWidget(QWidget):
#==============================================================================
#  General
#==============================================================================
    def __init__(self):
        super(SkirosWidget, self).__init__()
        self.setObjectName('SkirosWidget')
        ui_file = os.path.join(rospkg.RosPack().get_path('skiros2_gui'), 'src/skiros2_gui/core', 'skiros_gui.ui')
        loadUi(ui_file, self)

        # self.wm_tree_widget.getStyleSheet('''
        #     QTreeView::branch:has-children {image: url(folderclosed.png)}
        #     QTreeView::branch:open {image: url(folderopened.png)}
        # ''')
        self.wm_tree_widget.itemSelectionChanged.connect( lambda : self.on_wm_tree_widget_item_selection_changed( self.wm_tree_widget.currentItem()) )

        self.reset()


    def reset(self):
        #The plugin should not call init_node as this is performed by rqt_gui_py.
        #Due to restrictions in Qt, you cannot manipulate Qt widgets directly within ROS callbacks,
        #because they are running in a different thread.
        self._wmi = wmi.WorldModelInterface()
        self._sli = sli.SkillLayerInterface(self._wmi)
        self._author_name = "skiros_gui"

        self.generate_wm_tree()

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
    def on_load_scene_button_clicked(self):
        file = self.scene_file_lineEdit.text()
        log.debug(self.__class__.__name__, 'Loading world model from <{}>'.format(file))
        self._wmi.load(file)
        self.generate_wm_tree()


    @Slot()
    def on_save_scene_button_clicked(self):
        file = self.scene_file_lineEdit.text()
        log.debug(self.__class__.__name__, 'Saving world model to <{}>'.format(file))
        self._wmi.save(file)


    @Slot()
    def on_add_object_button_clicked(self):
        dialog = SkirosAddObjectDialog(self)
        ret = dialog.exec_()
        if not ret: return

        print(dialog.object)


    @Slot()
    def on_remove_object_button_clicked(self):
        print "remove_object_button"


    @Slot()
    def on_wm_tree_widget_item_selection_changed(self, item):
        self.fill_property_table(item)



    def generate_wm_tree(self):
        scene = {elem.id: elem for elem in self._wmi.getScene()}
        root = scene['skiros:Scene-0']
        self.wm_tree_widget.clear()
        self._generate_wm_tree(self.wm_tree_widget, scene, root)
        self.wm_tree_widget.setCurrentIndex(self.wm_tree_widget.model().index(0, 0))
        self.wm_tree_widget.expandAll()

    def _generate_wm_tree(self, node, scene, elem):
        item = QTreeWidgetItem(node, [utils.ontology_type2name(elem.id)])
        item.setData(2, Qt.EditRole, elem.id)
        for rel in elem.getRelations(subj='-1'):
            self._generate_wm_tree(item, scene, scene[rel['dst']])


    def fill_property_table(self, item):
        id = item.data(2, Qt.EditRole)
        elem = self._wmi.getElement(id)

        self.wm_properties_widget.clear()

        # print(elem.printState(True))
        key = QTableWidgetItem('ID')
        val = QTableWidgetItem(utils.ontology_type2name(elem.id))

        self.wm_properties_widget.insertRow(self.wm_properties_widget.rowCount())
        self.wm_properties_widget.setItem(self.wm_properties_widget.rowCount()-1, 0, key)
        self.wm_properties_widget.setItem(self.wm_properties_widget.rowCount()-1, 1, val)
        # print(item.properties)













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

