import os
import rclpy
from rclpy.time import Time
from ament_index_python.packages import get_resource

from functools import partial

from collections import OrderedDict

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot, pyqtSignal
import python_qt_binding.QtGui as QtGui
from python_qt_binding.QtWidgets import QTableWidgetItem, QTreeWidgetItem, QWidget, QCheckBox, QComboBox, QLineEdit, QDialog, QSizePolicy, QShortcut

import skiros2_common.tools.logger as log
import skiros2_common.core.utils as utils
import skiros2_common.ros.utils as rosutils
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.property import Property
import skiros2_msgs.msg as msgs
import skiros2_world_model.ros.world_model_interface as wmi
import skiros2_skill.ros.skill_layer_interface as sli
from skiros2_common.core.abstract_skill import State
from copy import deepcopy
from std_msgs.msg import String

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback
from numpy.linalg import norm
from threading import Lock
from datetime import datetime
from collections import OrderedDict


class SkirosSkillInfo(QDialog):
    # ==============================================================================
    #  UI visualizing skill progress msgs and parameters
    # ==============================================================================

    def __init__(self, *args, **kwargs):
        super(SkirosSkillInfo, self).__init__(*args, **kwargs)
        self.setObjectName('SkirosSkillInfo')
        _, self.package_path = get_resource('packages', 'skiros2_gui')
        ui_file = os.path.join(self.package_path , 'share/skiros2_gui/core', 'skiros_skill_info.ui')
        loadUi(ui_file, self)

    def update_progress_table(self, msg):
        row = self.progress_table.rowCount()
        self.progress_table.insertRow(row)
        self.progress_table.setItem(row, 0, QTableWidgetItem("{:0.3f}".format(msg.progress_time)))
        self.progress_table.setItem(row, 1, QTableWidgetItem(State(msg.state).name))
        self.progress_table.setItem(row, 2, QTableWidgetItem(str(msg.progress_code)))
        progress = QTableWidgetItem(str(msg.progress_message))
        progress.setToolTip(str(msg.progress_message))
        self.progress_table.setItem(row, 3, progress)
        self.progress_table.scrollToBottom()

    def update_params(self, msg):
        params = rosutils.deserializeParamMap(msg)
        self.params_table.setSortingEnabled(False)
        self.params_table.sortByColumn(0, Qt.AscendingOrder)
        self.params_table.setRowCount(0)
        for i, v in enumerate(params.values()):
            self.params_table.insertRow(i)
            key = QTableWidgetItem(v.key)
            key.setFlags(key.flags() & ~Qt.ItemIsEditable)
            value = QTableWidgetItem(str(v.values))
            value.setFlags(value.flags() & ~Qt.ItemIsEditable)
            if v.dataType() == Element:
                value.setToolTip("\n".join([v2.printState(True) for v2 in v.values]))
            self.params_table.setItem(i, 0, key)
            self.params_table.setItem(i, 1, value)
        self.params_table.setSortingEnabled(True)

    def log_info(self, msg):
        skill_name = "{}_{}".format(msg.label, msg.id)
        if self.skill_label.text() != skill_name:
            self.progress_table.setRowCount(0)
            self.skill_label.setText(skill_name)
        self.update_params(msg.params)
        self.update_progress_table(msg)


class SkirosModifyRelationDialog(QDialog):
    # ==============================================================================
    #  Dialog window to add new items to wm
    # ==============================================================================

    default_type = 'sumo:Object'

    def __init__(self, *args, **kwargs):
        super(SkirosModifyRelationDialog, self).__init__(*args, **kwargs)
        self.setObjectName('SkirosModifyRelationDialog')
        _, self.package_path = get_resource('packages', 'skiros2_gui')
        ui_file = os.path.join(self.package_path ,
                               'share/skiros2_gui/core', 'skiros_modify_object_dialog.ui')
        loadUi(ui_file, self)
        self._rows = []
        self.create_comboBox('Predicate', self.parent().get_relations())
        self.create_comboBox('Subject', self.parent().get_individuals(SkirosModifyRelationDialog.default_type))
        self.create_comboBox('Object', self.parent().get_individuals(SkirosModifyRelationDialog.default_type))

    def create_comboBox(self, label, elements):
        """
        Inserts a new combobox in the dialog

        Helper function that creates a combobox and fills the list.

        @param      label     Label for the dropdown list
        @param      elements  dictionary of elements for the dropdown list
        """
        comboBox = QComboBox()
        size_policy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        comboBox.setSizePolicy(size_policy)
        for alias, key in elements.items():
            element_label = self.parent()._wmi.get_element(key).label if self.parent()._wmi.is_scene_element(key) else ""
            comboBox.addItem("{} {}".format(alias, element_label), key)
        comboBox.model().sort(0)
        self.formLayout.insertRow(len(self._rows), label, comboBox)
        self._rows.append(comboBox)

    def set_combos_item(self, item_id):
        text = item_id.split(':')[-1]
        for r in self._rows:
            r.setCurrentIndex(r.findText(text))

    def get_result(self):
        """
        Return the type and value selected from user

        @return     The result
        """
        return (self._rows[1].itemData(self._rows[1].currentIndex()),
                self._rows[0].itemData(self._rows[0].currentIndex()),
                self._rows[2].itemData(self._rows[2].currentIndex()))


class SkirosModifyPropertyDialog(QDialog):
    # ==============================================================================
    #  Dialog window to add new items to wm
    # ==============================================================================

    default_type = 'sumo:Object'

    def __init__(self, *args, **kwargs):
        super(SkirosModifyPropertyDialog, self).__init__(*args, **kwargs)
        self.setObjectName('SkirosModifyPropertyDialog')
        _, self.package_path = get_resource('packages', 'skiros2_gui')
        ui_file = os.path.join(self.package_path ,
                               'share/skiros2_gui/core', 'skiros_modify_object_dialog.ui')
        loadUi(ui_file, self)
        self._rows = []
        self.create_comboBox('Type', self.parent().get_properties())
        self.create_text_box('Value')

    def create_comboBox(self, label, elements):
        """
        Inserts a new combobox in the dialog

        Helper function that creates a combobox and fills the list.

        @param      label     (str): Label for the dropdown list
        @param      elements  (list): dictionary of elements for the dropdown list
        """
        comboBox = QComboBox()
        size_policy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        comboBox.setSizePolicy(size_policy)
        [comboBox.addItem(l, d) for l, d in elements.items()]
        comboBox.model().sort(0)
        self.formLayout.insertRow(len(self._rows), label, comboBox)
        self._rows.append(comboBox)

    def create_text_box(self, label):
        """
        Inserts a new textbox in the dialog

        @param      label  (str): Label for the text box
        """
        lineedit = QLineEdit()
        size_policy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        lineedit.setSizePolicy(size_policy)
        self.formLayout.insertRow(len(self._rows), label, lineedit)
        self._rows.append(lineedit)

    def get_result(self):
        """
        Return the type and value selected from user

        @return     tuple(property_type, value, dataype)
        """
        property_type = self._rows[0].itemData(self._rows[0].currentIndex())
        property_value = self._rows[1].text()
        return (property_type, property_value, self.parent()._wmi.get_datatype(property_type))


class SkirosAddObjectDialog(QDialog):
    # ==============================================================================
    #  Dialog window to add new items to wm
    # ==============================================================================

    default_type = 'sumo:Object'

    def __init__(self, *args, **kwargs):
        """
        Implements a dialog to create a new object for the world model.

        Implementation of the modal dialog to select object types from the
        available ontology/world model. Allows filtering of the objects by
        (sub)type. The dialog saves the selection in the 'object' property.

        @param      args    The arguments
        @param      kwargs  The keywords arguments
        """
        super(SkirosAddObjectDialog, self).__init__(*args, **kwargs)
        self.setObjectName('SkirosAddObjectDialog')
        _, self.package_path = get_resource('packages', 'skiros2_gui')
        ui_file = os.path.join(self.package_path, 'share/skiros2_gui/core', 'skiros_gui_add_object_dialog.ui')
        loadUi(ui_file, self)

        self._comboBoxes = []
        self.create_comboBox(label='Type')
        self.comboBox_individual.clear()
        [self.comboBox_individual.addItem(l, d) for l, d in self.parent().get_individuals(self.default_type).items()]
        self.comboBox_individual.model().sort(0)

    @property
    def object(self):
        """
        Access to the currently selected object type.

        @return     str: Selected (ontology) type (e.g. skiros:Product)
        """
        return self.comboBox_individual.itemData(self.comboBox_individual.currentIndex())

    def on_select_type(self, id, index):
        """
        Callback for change selection in dropdown lists.

        Adds and removes dropdown list to/from the dialog that are used to
        filter subtypes based on the current selection.

        @param      id     (int): Number of the combobox that dispatched the
                           callback
        @param      index  (int): Number of the selected item in the current
                           combobox (id)
        """
        # log.debug(self.__class__.__name__, 'Selected {}: {}'.format(id, index))

        # clear filters after selected
        while id < len(self._comboBoxes) - 1:
            # log.debug(self.__class__.__name__, 'Delete {}'.format(id+1))
            label = self.formLayout.labelForField(self._comboBoxes[id + 1])
            label.deleteLater()
            self._comboBoxes[id + 1].deleteLater()
            del self._comboBoxes[id + 1]

        # get current selection
        if index > 0:  # if not 'All' is selected
            selected = self._comboBoxes[id].itemData(self._comboBoxes[id].currentIndex())
        elif id > 0:  # if 'All' is selected and it is not the first combo box
            selected = self._comboBoxes[id - 1].itemData(self._comboBoxes[id - 1].currentIndex())
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
            self.comboBox_individual.addItem('new ' + utils.ontology_type2name(selected), selected)
        [self.comboBox_individual.addItem(l, d) for l, d in self.parent().get_individuals(selected).items()]
        self.comboBox_individual.model().sort(0)
        QTimer.singleShot(0, self.adjustSize)

    def create_comboBox(self, subtype='sumo:Object', label='Subtype'):
        """
        Inserts a new combobox in the dialog based on the subtype.

        Helper function that creates a combobox and fills the list with filtered
        items from the ontology/world model.

        Args:  label

        @param      subtype  (str, optional): Type to be used to retrieve world
                             model items for the dropdown list
        @param      label    (str, optional): Label for the dropdown list
        """
        comboBox = QComboBox()
        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        comboBox.setSizePolicy(sizePolicy)
        comboBox.addItem('All')
        [comboBox.addItem(l, d) for l, d in self.parent().get_types(subtype).items()]
        comboBox.currentIndexChanged.connect(partial(self.on_select_type, len(self._comboBoxes)))
        self.formLayout.insertRow(len(self._comboBoxes), label, comboBox)
        self._comboBoxes.append(comboBox)


class SkirosInteractiveMarkers:
    default_box_size = 0.1

    def on_marker_feedback(self, feedback):
        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"

        mp = ""
        if feedback.mouse_point_valid:
            mp = " at " + str(feedback.mouse_point.x)
            mp += ", " + str(feedback.mouse_point.y)
            mp += ", " + str(feedback.mouse_point.z)
            mp += " in frame " + feedback.header.frame_id
        print(s)
        print(mp)

    def _make_box(self, msg, size):
        marker = Marker()
        marker.type = Marker.CUBE
        if None in size:
            size = [SkirosInteractiveMarkers.default_box_size,
                    SkirosInteractiveMarkers.default_box_size, SkirosInteractiveMarkers.default_box_size]
        marker.scale.x = msg.scale * size[0]
        marker.scale.y = msg.scale * size[1]
        marker.scale.z = msg.scale * size[2]
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.5
        marker.color.a = 0.5
        return marker

    def _make_box_control(self, msg, size):
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self._make_box(msg, size))
        msg.controls.append(control)
        return control

    def initInteractiveServer(self, node, name):
        """
        @brief Start the interactive marker server
        """
        self._server = InteractiveMarkerServer(node, name)

    def clear_markers(self):
        self._server.clear()

    def make_6dof_marker(self, pose, size, frame_id, base_frame_id, interaction_mode):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = base_frame_id
        int_marker.pose.position.x = pose[0][0]
        int_marker.pose.position.y = pose[0][1]
        int_marker.pose.position.z = pose[0][2]
        int_marker.pose.orientation.x = pose[1][0]
        int_marker.pose.orientation.y = pose[1][1]
        int_marker.pose.orientation.z = pose[1][2]
        int_marker.pose.orientation.w = pose[1][3]
        int_marker.scale = 1.0

        int_marker.name = frame_id
        int_marker.description = frame_id

        # insert a box
        self._make_box_control(int_marker, size)
        int_marker.controls[0].interaction_mode = interaction_mode

        n = norm([1, 1])
        control = InteractiveMarkerControl()
        control.orientation.w = 1. / n
        control.orientation.x = 1. / n
        control.orientation.y = 0.
        control.orientation.z = 0.
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1. / n
        control.orientation.x = 1. / n
        control.orientation.y = 0.
        control.orientation.z = 0.
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1. / n
        control.orientation.x = 0.
        control.orientation.y = 1. / n
        control.orientation.z = 0.
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1. / n
        control.orientation.x = 0.
        control.orientation.y = 1. / n
        control.orientation.z = 0.
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1. / n
        control.orientation.x = 0.
        control.orientation.y = 0.
        control.orientation.z = 1. / n
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1. / n
        control.orientation.x = 0.
        control.orientation.y = 0.
        control.orientation.z = 1. / n
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        self._server.insert(int_marker, feedback_callback=self.on_marker_feedback)
        self._server.applyChanges()


class SkirosWidget(QWidget, SkirosInteractiveMarkers):
    # ==============================================================================
    #  General
    # ==============================================================================

    widget_id = 'skiros_gui'

    wm_update_signal = pyqtSignal(msgs.WmMonitor)
    task_progress_signal = pyqtSignal(msgs.TreeProgress)

    def __init__(self, node, initial_topics=None, start_paused=False):
        super(SkirosWidget, self).__init__()
        self._node = node
        self._marker_update = None
        self.setObjectName('SkirosWidget')

        _, self.package_path = get_resource('packages', 'skiros2_gui')
        ui_file = os.path.join(self.package_path, 'share', 'skiros2_gui/core', 'skiros_gui.ui')
        loadUi(ui_file, self)

        self.skill_tree_widget.currentItemChanged.connect(
            lambda: self.on_skill_tree_widget_item_selection_changed(self.skill_tree_widget.currentItem()))
        self.wm_tree_widget.itemSelectionChanged.connect(
            lambda: self.on_wm_tree_widget_item_selection_changed(self.wm_tree_widget.currentItem()))
        self.task_tree_widget.itemSelectionChanged.connect(
            lambda: self.on_task_tree_widget_item_selection_changed(self.task_tree_widget.currentItem()))
        self.wm_properties_widget.itemChanged.connect(
            lambda p: self.on_properties_table_item_changed(self.wm_tree_widget.currentItem(), p.row()))
        self.wm_relations_widget.resizeEvent = self.on_wm_relations_widget_resized
        self.wm_update_signal.connect(lambda d: self.on_wm_update(d))
        self.task_progress_signal.connect(lambda d: self.on_progress_update(d))

        self.reset()
        self.skill_info_widget = SkirosSkillInfo(self)
        self.main_layout.addWidget(self.skill_info_widget)

    def reset(self):
        # The plugin should not call init_node as this is performed by rqt_gui_py.
        # Due to restrictions in Qt, you cannot manipulate Qt widgets directly within ROS callbacks,
        # because they are running in a different thread.
        self.initInteractiveServer(self._node, SkirosWidget.widget_id)
        # We can not spin in rqt application:
        self._wmi = wmi.WorldModelInterface(self._node, SkirosWidget.widget_id, allow_spinning=False)
        self._sli = sli.SkillLayerInterface(self._node, SkirosWidget.widget_id, allow_spinning=False)

        # Setup a timer to keep interface updated
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self.refresh_timer_cb)
        self.refresh_timer.start(500)
        self.robot_sub = self._node.create_subscription(String, '/robot_output', self.robot_output_cb, 10)
        self.robot_text = ""

        # World model tab
        self._wmi.set_monitor_cb(lambda d: self.wm_update_signal.emit(d))
        self._sli.set_monitor_cb(lambda d: self.task_progress_signal.emit(d))
        self._snapshot_id = ""
        self._snapshot_stamp = self._node.get_clock().now()
        self._wm_mutex = Lock()
        self._task_mutex = Lock()

        # Skill tab
        self.last_executed_skill = ""
        self.skill_stop_button.setEnabled(False)
        self.skill_pause_button.setEnabled(False)
        self.space_shortcut = QShortcut(QtGui.QKeySequence(Qt.Key_Space), self)
        self.plus_shortcut = QShortcut(QtGui.QKeySequence(Qt.Key_Plus), self)
        self.space_shortcut.activated.connect(self.skill_start_stop)
        self.plus_shortcut.activated.connect(self.skill_start_stop)
        self.task_tree_widget.setColumnWidth(0, 480)
        self.task_tree_widget.setColumnWidth(1, 60)
        self.skill_item = OrderedDict()
        # self.space_shortcut.setContext(QtCore.Qt.WidgetWithChildrenShortcut)
        # Log tab
        self.log_file = None
        self.icons = dict()

    def get_types(self, subtype):
        """Retrieves available subtype from the ontology.

        Args:
            subtype (str): Filter for object types

        Returns:
            dict(str, str): Keys: Short type name. Values: Type identifier (e.g. {'Product': 'skiros:Product'})
        """
        return utils.ontology_type2name_dict(self._wmi.get_sub_classes(subtype, False))

    def get_individuals(self, subtype):
        """Retrieves available individuals from the world model.

        Args:
            subtype (str): Filter for object types

        Returns:
            dict(str, str): Keys: Short type name. Values: Type identifier (e.g. {'starter': 'skiros:starter'})
        """
        return utils.ontology_type2name_dict(self._wmi.get_individuals(subtype, True))

    def get_properties(self, otype="owl:DatatypeProperty"):
        """Retrieves properties.

        Args:
            subtype (str): Filter for object types

        Returns:
            dict(str, str): Keys: Short type name. Values: Type identifier (e.g. {'starter': 'skiros:starter'})
        """
        return utils.ontology_type2name_dict(self._wmi.get_types(otype))

    def get_relations(self, rtype="owl:ObjectProperty"):
        """Retrieves relations.

        Args:
            subtype (str): Filter for object types

        Returns:
            dict(str, str): Keys: Short type name. Values: Type identifier (e.g. {'starter': 'skiros:starter'})
        """
        return utils.ontology_type2name_dict(self._wmi.get_types(rtype))

    def shutdown_plugin(self):
        with self._wm_mutex and self._task_mutex:
            self._wmi.set_monitor_cb(None)
            self._sli.set_monitor_cb(None)
            del self._sli
            del self._wmi

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        instance_settings.set_value("scene_name", self.scene_file_lineEdit.text())
        instance_settings.set_value("save_logs", self.save_logs_checkBox.isChecked())
        instance_settings.set_value("logs_file_name", self.logs_file_lineEdit.text())
        instance_settings.set_value("last_executed_skill", self.last_executed_skill)
        instance_settings.set_value("debug_info", self.debug_checkBox.isChecked())
        instance_settings.set_value("skill_info", self.skill_info_checkBox.isChecked())
        instance_settings.set_value("include_filters", self.include_filters_lineEdit.text())
        instance_settings.set_value("exclude_filters", self.exclude_filters_lineEdit.text())

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        if self._wmi.get_scene_name() != "":
            self.scene_file_lineEdit.setText(self._wmi.get_scene_name())
        elif instance_settings.value("scene_name") is not None and instance_settings.value("scene_name") != "":
            self.scene_file_lineEdit.setText(instance_settings.value("scene_name"))
        if instance_settings.value("logs_file_name") is not None:
            self.logs_file_lineEdit.setText(instance_settings.value("logs_file_name"))
        if instance_settings.value("include_filters") is not None:
            self.include_filters_lineEdit.setText(instance_settings.value("include_filters"))
        if instance_settings.value("exclude_filters") is not None:
            self.exclude_filters_lineEdit.setText(instance_settings.value("exclude_filters"))
        if instance_settings.value("save_logs") is not None:
            self.save_logs_checkBox.setChecked(instance_settings.value("save_logs") == 'true')
        if instance_settings.value("last_executed_skill") is not None:
            self.last_executed_skill = instance_settings.value("last_executed_skill")
        if instance_settings.value("debug_info") is not None:
            self.debug_checkBox.setChecked(instance_settings.value("debug_info") == 'true')
        if instance_settings.value("skill_info") is not None:
            self.skill_info_checkBox.setChecked(instance_settings.value("skill_info") == 'true')
            self.on_skill_info_checkBox_clicked()

    def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
        pass


# ==============================================================================
#  General
# ==============================================================================


    def robot_output_cb(self, msg):
        self.robot_text = msg.data

    def refresh_timer_cb(self):
        """
        Keeps ui updated
        """
        # Update skill list
        if self._sli.has_changes:
            self.create_skill_tree()
            self._sli.set_debug(self.debug_checkBox.isChecked())
        # Update WM
        if self._wmi.is_connected() and self._snapshot_id == "":
            self.create_wm_tree()
        # Process marker feedback
        self._process_marker_feedback()
        # Update robot BT rate
        if self._sli.agents:
            robot_info = ""
            agents = self._sli.agents.copy()
            for name, manager in agents.items():
                robot_info += "{}: {:0.1f}hz ".format(name.replace("/", ""), manager.get_tick_rate())
            self.robot_rate_info.setText(robot_info)
            self.robot_output.setText(self.robot_text)
        else:
            self.robot_rate_info.setText("No robot connected.")
            self.robot_output.setText("")
            self.stop_task_tracking()

    def create_skill_tree(self):
        self.skill_tree_widget.setSortingEnabled(False)
        self.skill_tree_widget.sortByColumn(0, Qt.AscendingOrder)
        self.skill_tree_widget.clear()
        self.skill_tree_widget.setColumnCount(3)
        self.skill_tree_widget.hideColumn(2)
        self.skill_tree_widget.hideColumn(1)
        fu = QTreeWidgetItem(self.skill_tree_widget, ["Frequently used", "fu"])
        fu.setExpanded(True)
        root = QTreeWidgetItem(self.skill_tree_widget, ["All", "All"])
        root.setExpanded(True)
        agents = self._sli._agents.copy()
        for ak, e in agents.items():
            for s in e.get_skill_list().values():
                s.manager = ak
                self._add_available_skill(s)
        # simplifies hierarchy
        self.simplify_tree_hierarchy(root)
        self.skill_tree_widget.setSortingEnabled(True)
        # select last skill
        s = self.skill_tree_widget.findItems(self.last_executed_skill, Qt.MatchRecursive | Qt.MatchFixedString, 1)
        self.skill_params_table.setRowCount(0)
        if s:
            self.skill_tree_widget.setCurrentItem(s[0])

    def simplify_tree_hierarchy(self, root):
        i = 0
        while i < root.childCount():
            c = root.child(i)
            if c.childCount() == 1:
                root.addChildren(c.takeChildren())
                root.removeChild(c)
            else:
                self.simplify_tree_hierarchy(c)
                i += 1

# ==============================================================================
#  World model tab
# ==============================================================================

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

    @Slot()
    def on_add_object_button_clicked(self):
        dialog = SkirosAddObjectDialog(self)
        ret = dialog.exec_()
        if not ret:
            return

        log.debug(self.__class__.__name__, 'Create element based on {}'.format(dialog.object))

        parent = self.wm_tree_widget.currentItem()
        parent_id = parent.text(1)
        if not parent_id:
            return
        if "owl:NamedIndividual" in self._wmi.get_type(dialog.object):
            elem = self._wmi.get_template_element(dialog.object)
            elem.label = utils.ontology_type2name(dialog.object)
            elem_id = self._wmi.instanciate(elem, recursive=True, relations=[
                                            {'src': parent_id, 'type': 'skiros:contain', 'dst': '-1'}])
        else:
            elem = Element(dialog.object)
            elem_id = self._wmi.instanciate(elem, relations=[{'src': parent_id, 'type': 'skiros:contain', 'dst': '-1'}])
        log.debug(self.__class__.__name__, 'Added element {} to {}'.format(elem_id, parent_id))

    @Slot()
    def on_remove_object_button_clicked(self):
        item = self.wm_tree_widget.currentItem()
        item_id = item.text(1)
        if not item_id:
            return
        parent = item.parent()
        self.wm_tree_widget.setCurrentItem(parent)
        try:
            elem = self._wmi.get_element(item_id)
            self._wmi.remove_element(elem)

            log.debug(self.__class__.__name__, 'Removed element {}'.format(item_id))
        except wmi.WmException as e:
            log.error("[remove_object]", "{}".format(e))

    @Slot()
    def on_add_property_button_clicked(self):
        dialog = SkirosModifyPropertyDialog(self)
        ret = dialog.exec_()
        if not ret:
            return

        # Call widget
        item_id = self.wm_tree_widget.currentItem().text(1)
        elem = self._wmi.get_element(item_id)
        try:
            log.info("[add_property]", "Adding: {} {} {}".format(*dialog.get_result()))
            elem.setProperty(*dialog.get_result(), force_convertion=True)
            self._wmi.update_element_properties(elem)
        except ValueError as e:
            log.error("[on_add_property_button_clicked]", "{}".format(e))

    @Slot()
    def on_remove_property_button_clicked(self):
        row = self.wm_properties_widget.currentRow()
        item = self.wm_properties_widget.item(row, 0)
        if item is not None:
            key = item.id
            item_id = self.wm_tree_widget.currentItem().text(1)
            elem = self._wmi.get_element(item_id)
            if elem.getAssociatedReasonerId(key):
                log.error("[removeProperty]", "{} is managed by {}, can't remove directly.".format(
                    key, elem.getAssociatedReasonerId(key)))
                return
            log.info("[removeProperty]", "Remove {}".format(key))
            try:
                elem.removeProperty(key)
                self._wmi.update_element_properties(elem)
            except KeyError:
                log.error("[removeProperty]", "Can't remove property {} from {}".format(key, elem.printState()))

    @Slot()
    def on_add_relation_button_clicked(self):
        # Call widget
        item_id = self.wm_tree_widget.currentItem().text(1)
        elem = self._wmi.get_element(item_id)
        dialog = SkirosModifyRelationDialog(self)
        dialog.set_combos_item(item_id)
        ret = dialog.exec_()
        if not ret:
            return

        try:
            result = list(dialog.get_result())
            i = result.index(item_id)
            self._add_relations_table_row({'src': result[0], 'type': result[1], 'dst': result[2]})
            result[i] = "-1"
            log.info("[add_relation]", "Adding: {} {} {}".format(*result))
            elem.addRelation(*dialog.get_result())
            self._wmi.update_element(elem)
        except ValueError as e:
            log.error("[on_add_relation_button_clicked]",
                      "Current object must be subject and/or object of the relation.")

    @Slot()
    def on_remove_relation_button_clicked(self):
        item_id = self.wm_tree_widget.currentItem().text(1)
        elem = self._wmi.get_element(item_id)
        row = self.wm_relations_widget.currentRow()
        rel = ("-1" if item_id == self.wm_relations_widget.item(row, 0).id else self.wm_relations_widget.item(row, 0).id,
               self.wm_relations_widget.item(row, 1).id,
               "-1" if item_id == self.wm_relations_widget.item(row, 2).id else self.wm_relations_widget.item(row, 2).id)
        if not rel[1] in ["skiros:hasSkill"]:  # Protected relations
            log.info("[remove_relation]", "{} {} {}".format(*rel))
            elem.removeRelation2(*rel)
            self._wmi.update_element(elem)
            self.wm_relations_widget.removeRow(row)
        else:
            log.error("[remove_relation]", "Can't remove protected relation: {} {} {}".format(*rel))

    @Slot()
    def on_wm_update(self, data):
        with self._wm_mutex:
            # Discard msgs not in sync with local wm version
            if self._snapshot_id == data.prev_snapshot_id and data.action != "reset":
                self._snapshot_id = data.snapshot_id
                cur_item = self.wm_tree_widget.currentItem()
                cur_item_id = cur_item.text(1)
                if data.action == 'update' or data.action == 'update_properties':
                    for elem in data.elements:
                        elem = rosutils.msg2element(elem)
                        self._update_wm_node(elem, cur_item_id)
                elif data.action == 'add':
                    for elem in data.elements:
                        elem = rosutils.msg2element(elem)
                        if not self._add_wm_node(elem):
                            self._snapshot_id = ""
                elif data.action == 'remove' or data.action == 'remove_recursive':
                    for elem in data.elements:
                        elem = rosutils.msg2element(elem)
                        self._remove_wm_node(elem)
                # reselect current item
                items = self.wm_tree_widget.findItems(cur_item_id, Qt.MatchRecursive | Qt.MatchFixedString, 1)
                if items:
                    self.wm_tree_widget.setCurrentItem(items[0])
            elif Time.from_msg(data.stamp) > self._snapshot_stamp or self._snapshot_id == "":  # Ignores obsolete msgs
                log.info("[wm_update]", "Wm not in sync, querying wm scene")
                self.create_wm_tree()

    def on_marker_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self._marker_update = feedback

    def _process_marker_feedback(self):
        if self._marker_update is not None:
            with self._wm_mutex:
                elem = self._wmi.get_element(self._marker_update.marker_name)
                elem.setData(":PoseStampedMsg", self._marker_update)
                self._marker_update = None
                self._wmi.update_element_properties(elem)

    @Slot()
    def on_wm_tree_widget_item_selection_changed(self, item):
        self.wm_properties_widget.blockSignals(True)
        self.clear_markers()
        if item.text(1).find("_skills") < 0:
            elem = self._wmi.get_element(item.text(1))
            self.fill_properties_table(elem)
            self.fill_relations_table(elem)
            if elem.hasProperty("skiros:DiscreteReasoner", "AauSpatialReasoner"):
                p = elem.getData(":Pose")
                size = elem.getData(":Size")
                if not None in p[0] and not None in p[1]:
                    self.make_6dof_marker(p, size, elem.id, elem.getProperty("skiros:BaseFrameId").value,
                                          InteractiveMarkerControl.NONE)  # NONE,MOVE_3D, MOVe_ROTATE_3D
        else:
            self.wm_properties_widget.setRowCount(0)
            self.wm_relations_widget.setRowCount(0)
        self.wm_properties_widget.blockSignals(False)

    @Slot()
    def on_properties_table_item_changed(self, item, row):
        item_key = self.wm_properties_widget.item(row, 0)
        item_val = self.wm_properties_widget.item(row, 1)
        key = item_key.id
        elem = self._wmi.get_element(item.text(1))

        if elem.hasProperty(key):
            prop = elem.getProperty(key)
            if prop.dataTypeIs(bool):
                value = item_val.checkState() == Qt.Checked
            else:
                if item_val.text() == '':
                    value = None
                else:
                    value = str(item_val.text())
                    if value[0] == "[":
                        value = value.replace("[", "").replace("]", "").replace('\'', '')
                    value = [v.strip() for v in value.split(",")]
            try:
                elem.setProperty(prop.key, value, force_convertion=value is not None)
                log.debug(self.__class__.__name__, '<{}> property {} to {}'.format(item.text(1), prop.key, value))
            except ValueError:
                log.error(self.__class__.__name__, 'Changing <{}> property {} to {} failed'.format(
                    item.text(1), prop.key, value))
                item_val.setText(str(prop.value))
        elif hasattr(elem, key.lower()):
            value = None if item_val.text() == '' else str(item_val.text())
            key = key.lower()
            attr = getattr(elem, key)
            setattr(elem, key, type(attr)(value))
            log.debug(self.__class__.__name__, '<{}> attribute {} to {}'.format(item.text(1), key, value))
        else:
            log.error(self.__class__.__name__, 'Changing <{}> property {} to {} failed'.format(item.text(1), key, value))

        self._wmi.update_element_properties(elem)
        name = utils.ontology_type2name(elem.id) if not elem.label else utils.ontology_type2name(elem.label)
        item.setText(0, name)

    @Slot()
    def on_wm_relations_widget_resized(self, event):
        width = self.wm_relations_widget.size().width() - 2
        cols = self.wm_relations_widget.columnCount()
        for i in range(cols):
            self.wm_relations_widget.setColumnWidth(i, width // cols)

    def create_wm_tree(self):
        scene_tuple = None
        while scene_tuple is None:
            try:
                scene_tuple = self._wmi.get_scene()
            except wmi.WmException:
                log.warn("[create_wm_tree]", "Failed to retrieve scene, will try again.")
        # print "GOT SCENE {}".format([e.id for e in scene_tuple[0]])
        self._snapshot_id = scene_tuple[1]
        self._snapshot_stamp = self._node.get_clock().now()
        scene = {elem.id: elem for elem in scene_tuple[0]}
        root = scene['skiros:Scene-0']
        self.wm_tree_widget.clear()
        self.wm_tree_widget.setColumnCount(2)
        self.wm_tree_widget.hideColumn(1)
        self._create_wm_tree(self.wm_tree_widget, scene, root)
        self.wm_tree_widget.setCurrentIndex(self.wm_tree_widget.model().index(0, 0))

    def _remove_wm_node(self, elem):
        # print "Removing {}".format(elem.id)
        items = self.wm_tree_widget.findItems(elem.id, Qt.MatchRecursive | Qt.MatchFixedString, 1)
        if items:
            item = items[0]
            item.parent().removeChild(item)

    def _update_wm_node(self, elem, cur_item_id):
        # check if element is already in tree
        items = self.wm_tree_widget.findItems(elem.id, Qt.MatchRecursive | Qt.MatchFixedString, 1)
        if not items:
            # Element is not in the tree. Can happen if the node didn't have a spatial relation before
            return self._add_wm_node(elem)
        item = items[0]
        # check if updated item is selected
        if elem.id == cur_item_id:
            # update properties table if selected item has changed
            self.wm_properties_widget.blockSignals(True)
            self.fill_properties_table(elem)
            self.wm_properties_widget.blockSignals(False)

        # get parent node in GUI tree
        parent = item.parent()
        if not parent:
            return

        # get parent relation
        parent_rel = elem.getRelation(pred=self._wmi.get_sub_properties('skiros:spatiallyRelated'), obj='-1')
        if not parent_rel:
            parent_rel = elem.getRelation(pred=self._wmi.get_sub_properties('skiros:skillProperty'), obj='-1')
        if not parent_rel:
            parent_rel = elem.getRelation(pred='skiros:hasSkill', obj='-1')
        # check if the GUI parent is still parent of the updated element
        if not parent.text(1) in parent_rel['src']:
            # elem moved spatially
            item.parent().removeChild(item)
            parents = self.wm_tree_widget.findItems(parent_rel['src'], Qt.MatchRecursive | Qt.MatchFixedString, 1)
            if not parents:
                log.warn("[update_wm_node]", "No parent found for {}".format(elem.id))
                return
            item.setText(0, utils.ontology_type2name(elem.id)
                         if not elem.label else utils.ontology_type2name(elem.label))
            item.setText(1, elem.id)
            parents[0].addChild(item)

    def _add_wm_node(self, elem):
        parent_rel = elem.getRelation(pred=self._wmi.get_sub_properties('skiros:spatiallyRelated'), obj='-1')
        to_expand = True
        if not parent_rel:
            parent_rel = elem.getRelation(pred=self._wmi.get_sub_properties('skiros:skillProperty'), obj='-1')
            to_expand = False
        if not parent_rel:
            to_expand = False
            parent_rel = elem.getRelation(pred='skiros:hasSkill', obj='-1')
            if not parent_rel:
                log.warn("[add_wm_node]", "Skipping element without declared parent: {}".format(elem.id))
                return True
            parent_id = '{}_skills'.format(parent_rel['src'])
            item = self.wm_tree_widget.findItems(parent_id, Qt.MatchRecursive | Qt.MatchFixedString, 1)
            if not item:  # In case it is still not existing i create the "support" skill node
                item = self.wm_tree_widget.findItems(parent_rel['src'], Qt.MatchRecursive | Qt.MatchFixedString, 1)[0]
                item = QTreeWidgetItem(item, ['Skills', parent_id])
            else:
                item = item[0]
        else:
            items = self.wm_tree_widget.findItems(parent_rel['src'], Qt.MatchRecursive | Qt.MatchFixedString, 1)
            if not items:
                log.warn("[add_wm_node]", "Parent {} of node {} is not in the known tree.".format(parent_rel['src'], elem.id))
                return False
            item = items[0]
        name = utils.ontology_type2name(elem.id) if not elem.label else utils.ontology_type2name(elem.label)
        item = QTreeWidgetItem(item, [name, elem.id])
        item.setExpanded(to_expand)
        return True

    def _create_wm_tree(self, item, scene, elem):
        name = utils.ontology_type2name(elem.id) if not elem.label else utils.ontology_type2name(elem.label)
        item = QTreeWidgetItem(item, [name, elem.id])

        spatialRel = sorted(elem.getRelations(
            subj='-1', pred=self._wmi.get_sub_properties('skiros:spatiallyRelated')), key=lambda r: r['dst'])
        for rel in spatialRel:
            if rel['dst'] in scene:
                self._create_wm_tree(item, scene, scene[rel['dst']])
                item.setExpanded(True)

        skillRel = sorted(elem.getRelations(subj='-1', pred='skiros:hasSkill'), key=lambda r: r['dst'])
        if skillRel:
            skillItem = QTreeWidgetItem(item, ['Skills', '{}_skills'.format(elem.id)])
            for rel in skillRel:
                if rel['dst'] in scene:
                    self._create_wm_tree(skillItem, scene, scene[rel['dst']])
                    skillItem.setExpanded(True)

        skillPropRel = sorted(elem.getRelations(
            subj='-1', pred=self._wmi.get_sub_properties('skiros:skillProperty')), key=lambda r: r['dst'])
        for rel in skillPropRel:
            if rel['dst'] in scene:
                self._create_wm_tree(item, scene, scene[rel['dst']])

    def fill_properties_table(self, elem):
        row = self.wm_properties_widget.currentRow()
        self.wm_properties_widget.setRowCount(0)
        type = elem.id[:elem.id.rfind('-')]
        id = elem.id[elem.id.rfind('-') + 1:]
        self._add_properties_table_row(Property('ID', id), editable_value=False)
        self._add_properties_table_row(Property('Type', type), editable_value=False)
        self._add_properties_table_row(Property('Label', elem.label), editable_value=True)
        props = sorted(elem.properties, key=lambda e: e.key)
        [self._add_properties_table_row(p, False, True) for p in props]
        if row < self.wm_properties_widget.rowCount():
            self.wm_properties_widget.setCurrentCell(row, 1)

    def _add_properties_table_row(self, prop, editable_key=False, editable_value=True):
        key = QTableWidgetItem(utils.ontology_type2name(prop.key))
        if not editable_key:
            key.setFlags(key.flags() & ~Qt.ItemIsEditable)
        value = prop.values if prop.isList() else prop.value
        if prop.dataTypeIs(float):
            val = QTableWidgetItem(format(value, '.4f') if value is not None else '')
        else:
            val = QTableWidgetItem(str(value) if value is not None else '')
        if not editable_value:
            val.setFlags(val.flags() & ~Qt.ItemIsEditable)

        if prop.dataTypeIs(bool):
            val.setText('')
            val.setFlags(val.flags() & ~Qt.ItemIsEditable)
            val.setCheckState(Qt.Checked if prop.value else Qt.Unchecked)
        # if isinstance(prop.dataType(), bool):
        #     val.setCheckState(prop.value)

        key.id = val.id = prop.key

        self.wm_properties_widget.insertRow(self.wm_properties_widget.rowCount())
        self.wm_properties_widget.setItem(self.wm_properties_widget.rowCount() - 1, 0, key)
        self.wm_properties_widget.setItem(self.wm_properties_widget.rowCount() - 1, 1, val)

    def fill_relations_table(self, elem):
        self.wm_relations_widget.setRowCount(0)
        rel = sorted(elem.getRelations(), key=lambda r: r['type'])
        rel = sorted(rel, key=lambda r: r['src'], reverse=True)
        rel = map(lambda r: {
            'src': r['src'] if r['src'] != '-1' else elem.id,
            'type': r['type'],
            'dst': r['dst'] if r['dst'] != '-1' else elem.id
        }, rel)
        [self._add_relations_table_row(r) for r in rel]

    def _add_relations_table_row(self, relation, editable=False):
        src = QTableWidgetItem(utils.ontology_type2name(relation['src']))
        rel = QTableWidgetItem(utils.ontology_type2name(relation['type']))
        dst = QTableWidgetItem(utils.ontology_type2name(relation['dst']))

        src.id = relation['src']
        rel.id = relation['type']
        dst.id = relation['dst']

        src.setTextAlignment(Qt.AlignRight)
        rel.setTextAlignment(Qt.AlignHCenter)
        dst.setTextAlignment(Qt.AlignLeft)

        if not editable:
            src.setFlags(src.flags() & ~Qt.ItemIsEditable)
            rel.setFlags(rel.flags() & ~Qt.ItemIsEditable)
            dst.setFlags(dst.flags() & ~Qt.ItemIsEditable)

        self.wm_relations_widget.insertRow(self.wm_relations_widget.rowCount())
        # self.wm_relations_widget.setSpan(self.wm_relations_widget.rowCount()-1, 0, 1, 3)
        self.wm_relations_widget.setItem(self.wm_relations_widget.rowCount() - 1, 0, src)
        self.wm_relations_widget.setItem(self.wm_relations_widget.rowCount() - 1, 1, rel)
        self.wm_relations_widget.setItem(self.wm_relations_widget.rowCount() - 1, 2, dst)

    @Slot('QTreeWidgetItem*', 'QTreeWidgetItem*')
    def wm_tree_widget_currentItemChanged(self, item, prev_item):
        while self.wm_properties_widget.rowCount() > 0:
            self.wm_properties_widget.removeRow(0)
        item = QTableWidgetItem(str(item.text(0)))
        self.wm_properties_widget.insertRow(0)
        self.wm_properties_widget.setItem(0, 0, item)
        while self.wm_relations_widget.rowCount() > 0:
            self.wm_relations_widget.removeRow(0)
        item = QTableWidgetItem("")
        self.wm_relations_widget.insertRow(0)
        self.wm_relations_widget.setItem(0, 0, item)


# ==============================================================================
# Skill
# ==============================================================================


    def _format_skill_tooltip(self, msg):
        to_ret = "{}".format(msg.label)
        return to_ret

    def update_task_tree(self, msgs):
        with self._task_mutex:
            current_ids = self.skill_item.keys()

            # Removes the nodes duplicates, keeping only the postprocess msg
            progress = OrderedDict()
            for m in msgs.progress:
                progress[m.id] = m
            progress = progress.values()

            create = [m for m in progress if m.id not in current_ids]
            for m in create:
                parent = self.skill_item[m.parent_id]
                item = QTreeWidgetItem(parent, [""])
                item.setIcon(0, self.get_icon(m.processor))
                item.setExpanded(True)
                item.setFont(0, self.bold_font)
                self.skill_item[m.id] = item
                self.skills_msgs[m.id] = [m]

            update = create + [m for m in progress if m.id in current_ids]
            for m in update:
                item = self.skill_item[m.id]
                item.setData(0, 0, "{}".format(m.label))  # , "! SLOW !" if m.progress_period>0.04 else ""))
                item.setToolTip(0, self._format_skill_tooltip(m))
                item.setData(2, 0, "{}".format(m.progress_message))
                item.setToolTip(2, str(m.progress_message))
                item.setData(3, 0, m)
                if State(m.state) == State.Success:
                    item.setForeground(0, QtGui.QBrush(QtGui.QColor("#009933")))
                elif State(m.state) == State.Failure:
                    item.setForeground(0, QtGui.QBrush(QtGui.QColor("#cc0000")))
                elif State(m.state) == State.Running:
                    item.setForeground(0, QtGui.QBrush(QtGui.QColor("#ffbe00")))
                elif State(m.state) == State.Idle:
                    item.setForeground(0, QtGui.QBrush(QtGui.QColor("#aaaaaa")))
                if item == self.task_tree_widget.currentItem():
                    self.on_task_tree_widget_item_selection_changed(item)

            remove = [idd for idd in current_ids if idd not in [m.id for m in progress]]
            for idd in reversed(remove):
                parent_id = self.skills_msgs[idd][-1].parent_id
                parent = self.skill_item[parent_id]
                parent.removeChild(self.skill_item[idd])
                del self.skill_item[idd]
                del self.skills_msgs[idd]

    @Slot()
    def on_task_tree_widget_item_selection_changed(self, item):
        self.skill_info_widget.log_info(item.data(3, 0))

    @Slot()
    def on_progress_update(self, msg):
        # Update buttons
        root = [r for r in msg.progress if r.type.find("Root") >= 0]
        if root:
            if not self.skill_stop_button.isEnabled():
                self.create_task_tree(root[-1])
                self._toggle_task_active()
                for manager in self._sli.agents.values():
                    manager.reset_tick_rate()
            if abs(root[-1].progress_code) == 1:
                self._toggle_task_active()
        self.update_task_tree(msg)
        self._save_log(msg)
        # self.update_progress_table(msg)

    def get_icon(self, skill_type):
        if skill_type not in self.icons:
            file_name = os.path.join(self.package_path, "share", "skiros2_gui/core/imgs/",
                                     "{}.png".format(skill_type if skill_type else "skill"))
            self.icons[skill_type] = QtGui.QIcon(file_name)
        return self.icons[skill_type]

    def create_task_tree(self, msg):
        self.bold_font = QtGui.QFont()
        self.bold_font.setBold(True)
        self.skills_msgs = dict()
        self.skill_item = OrderedDict()
        self.task_tree_widget.clear()
        self.task_tree_widget.setUniformRowHeights(True)
        item = QTreeWidgetItem(self.task_tree_widget, ["Task {}".format(msg.id)])
        item.setExpanded(True)
        item.setFont(0, self.bold_font)
        self.skill_item[msg.id] = item
        self.skills_msgs[msg.id] = [msg]
        return item

    def _get_parameters(self, params):
        layout = self.skill_params_table
        for i in range(0, layout.rowCount()):
            key = layout.item(i, 0).text()
            widget = layout.cellWidget(i, 1)
            if params[key].dataTypeIs(bool):
                params[key].setValue(widget.isChecked())
            elif params[key].dataTypeIs(Element):
                data = widget.itemData(widget.currentIndex())
                if data:
                    params[key].setValue(self._wmi.get_element(data))
                    # print "Set param {} to {}".format(params[key].key, params[key].value.printState())
            else:
                try:
                    if widget.text():
                        params[key].setValuesFromStr(widget.text())
                except ValueError:
                    log.error("getParameters", "Failed to set param {}".format(params[key].key))
                    return False
        return True

    def _add_parameter(self, param):
        key = QTableWidgetItem(param.key)
        key.setFlags(key.flags() & ~Qt.ItemIsEditable)
        row = self.skill_params_table.rowCount()
        self.skill_params_table.insertRow(row)
        self.skill_params_table.setItem(row, 0, key)
        if param.dataTypeIs(bool):
            cbox = QCheckBox()
            if param.hasSpecifiedDefault():
                cbox.setChecked(param.default)
            self.skill_params_table.setCellWidget(row, 1, cbox)
        elif param.dataTypeIs(Element):
            combobox = QComboBox()
            self.skill_params_table.setCellWidget(row, 1, combobox)
            matches = self._wmi.resolve_elements(param.default)
            if param.paramTypeIs(ParamTypes.Optional):
                combobox.addItem("", None)
            for e in matches:
                combobox.addItem(e.id.split(':')[-1] + " {}".format(e.label), e._id)
            combobox.model().sort(0)
        else:
            lineedit = QLineEdit()
            if param.isSpecified():
                lineedit.setText(param.getValuesStr())
            self.skill_params_table.setCellWidget(row, 1, lineedit)

    def _add_available_skill(self, s):
        stype = self.skill_tree_widget.findItems(s.type, Qt.MatchRecursive | Qt.MatchFixedString, 1)
        if not stype:  # If it is the first of its type, add the parents hierarchy to the tree
            hierarchy = self._wmi.query_ontology('SELECT ?x {{ {} rdfs:subClassOf*  ?x }}'.format(s.type))
            if "skiros:Skill" not in hierarchy:
                log.warn("[add_available_skill]", f"Skill {s.name} is not a subclass of 'skiros:Skill'. Ignoring.")
                return
            hierarchy = hierarchy[:hierarchy.index("skiros:Skill")]
            hierarchy.reverse()
            parent = self.skill_tree_widget.findItems("All", Qt.MatchRecursive | Qt.MatchFixedString, 1)[0]
            for c in hierarchy:
                child = self.skill_tree_widget.findItems(c, Qt.MatchRecursive | Qt.MatchFixedString, 1)
                if child:
                    parent = child[0]
                else:
                    parent = QTreeWidgetItem(parent, [c.replace("skiros:", ""), c])
                    parent.setExpanded(True)
        else:
            parent = stype[0]
        skill = QTreeWidgetItem(parent, [s.name, s.name])
        skill.setData(2, 0, s)

    def _add_frequently_used_skill(self, s):
        self.last_executed_skill = s.name
        fu = self.skill_tree_widget.findItems("fu", Qt.MatchRecursive | Qt.MatchFixedString, 1)[0]
        for i in range(0, fu.childCount()):  # avoid adding same node multiple times
            if s.name == fu.child(i).data(1, 0):
                return
        skill = QTreeWidgetItem(fu, [s.name, s.name])
        skill.setData(2, 0, s)

    def _toggle_task_active(self):
        if self.skill_stop_button.isEnabled():
            self.stop_task_tracking()
        else:
            self.start_logging()
            self.skill_stop_button.setEnabled(True)
            self.skill_pause_button.setEnabled(True)
            self.skill_exe_button.setEnabled(False)

    def stop_task_tracking(self):
        self.end_logging()
        self.skill_stop_button.setEnabled(False)
        self.skill_pause_button.setEnabled(False)
        self.skill_exe_button.setEnabled(True)

    @Slot()
    def on_debug_checkBox_clicked(self):
        self._sli.set_debug(self.debug_checkBox.isChecked())

    @Slot()
    def on_skill_info_checkBox_clicked(self):
        if self.skill_info_checkBox.isChecked():
            self.skill_info_widget.show()
        else:
            self.skill_info_widget.hide()

    @Slot()
    def skill_start_stop(self):
        if self.skill_stop_button.isEnabled():
            self.on_skill_stop_button_clicked()
        else:
            self.on_skill_exe_button_clicked()

    @Slot()
    def on_skill_tree_widget_item_selection_changed(self, item):
        if item is None:
            return
        skill = item.data(2, 0)
        if skill:
            # Clean
            self.skill_params_table.setRowCount(0)
            # Add params
            self.skill_name_label.setText(skill.name)
            for p in skill.ph.values():
                if not p.paramTypeIs(ParamTypes.Inferred) and (self.modality_checkBox.isChecked() or p.paramTypeIs(ParamTypes.Required)):
                    self._add_parameter(p)
            self.skill_params_table.resizeRowsToContents()

    @Slot()
    def on_modality_checkBox_clicked(self):
        self.on_skill_tree_widget_item_selection_changed(self.skill_tree_widget.currentItem())

    @Slot()
    def on_skill_exe_button_clicked(self):
        if self._sli.has_active_agents:
            if self._sli.agent.execute(execution_id=self._sli.agent.task):
                self.skill_exe_button.setEnabled(False)
        else:
            skill = self.get_skill_for_execution()
            if skill is not None:
                if self._sli.agent.execute(skill_list=[skill]):
                    self.skill_exe_button.setEnabled(False)

    @Slot()
    def on_skill_stop_button_clicked(self):
        self._sli.agent.preempt_all()

    @Slot()
    def on_skill_pause_button_clicked(self):
        if self._sli.agent.pause_all():
            self.skill_exe_button.setEnabled(True)

    @Slot()
    def on_skill_step_button_clicked(self):
        if self._sli.has_active_agents:
            self._sli.agent.tick_once(execution_id=self._sli.agent.task)
        else:
            skill = self.get_skill_for_execution()
            if skill is not None:
                self._sli.agent.tick_once(skill_list=[skill])

    def get_skill_for_execution(self):
        if self.skill_tree_widget.currentItem() is None:
            return None
        skill = deepcopy(self.skill_tree_widget.currentItem().data(2, 0))
        if skill is None:
            return None
        self._add_frequently_used_skill(self.skill_tree_widget.currentItem().data(2, 0))
        # Send command
        if not self._get_parameters(skill.ph):
            return None
        return skill

# ==============================================================================
# Logs
# ==============================================================================
    def start_logging(self):
        self.end_logging()
        self.set_log_filters()
        self.logs_textEdit.clear()
        try:
            # Open new log file
            if self.save_logs_checkBox.isChecked():
                directory, self.file_name = self._get_new_log_filename()
                self.log_file = open("{}/{}.log".format(directory, self.file_name), "a")
        except (IOError) as e:
            log.error("[IOError]", str(e))
            self.save_logs_checkBox.setChecked(False)
        except (AttributeError) as e:
            pass

    def end_logging(self):
        # Close old log file
        if self.log_file is not None:
            self.log_file.close()
            self.log_file = None

    def set_log_filters(self):
        self.include_filters = list()
        self.exclude_filters = list()
        tokens = self.include_filters_lineEdit.text().split(",")
        for t in tokens:
            if t.strip():
                self.include_filters.append(t.strip())
        tokens = self.exclude_filters_lineEdit.text().split(",")
        for t in tokens:
            if t.strip():
                self.exclude_filters.append(t.strip())

    def in_filters(self, names):
        for m in self.exclude_filters:
            for n in names:
                if n.find(m) >= 0:
                    return False
        for m in self.include_filters:
            for n in names:
                if n.find(m) >= 0:
                    return True
        return not bool(self.include_filters)

    @property
    def log_directory(self):
        directory = self.logs_file_lineEdit.text().strip()
        directory = directory if directory[-1] != "/" else directory[:-1]
        return os.path.expanduser(directory)

    def _get_new_log_filename(self):
        skill = self.skill_tree_widget.currentItem().data(2, 0)
        file_name = "{}_{}".format(datetime.now().strftime("%Y-%m-%d:%H:%M:%S"), skill.name)
        directory = self.log_directory
        if not os.path.exists(directory):
            os.makedirs(directory)
        elif os.path.exists(file_name):
            with open(file_name, "r") as f:
                self.logs_textEdit.setText(f.read())
        return directory, file_name

    def _save_log(self, msgs):
        for msg in msgs.progress:
            if not self.in_filters([msg.label, msg.progress_message, State(msg.state).name]):
                return

            string = "{};{:0.4f};{};{};{};{};{};{};{}".format(datetime.now().strftime("%H:%M:%S"),
                                                              msg.progress_time, msg.parent_label, msg.parent_id,
                                                              msg.label, msg.id, State(msg.state).name,
                                                              msg.progress_code, msg.progress_message)
            self.logs_textEdit.append(string)

            if self.save_logs_checkBox.isChecked() and self.log_file is not None:
                self.log_file.write(string + "\n")
