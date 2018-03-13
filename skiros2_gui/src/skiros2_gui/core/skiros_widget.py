import os
import rospy
import rospkg

import time
from functools import partial

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot, QMetaObject, pyqtSignal
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QLabel, QTableWidgetItem, QTreeWidgetItem, QWidget, QCheckBox, QComboBox, QLineEdit, QDialog, QSizePolicy

import skiros2_common.tools.logger as log
import skiros2_common.core.utils as utils
import skiros2_common.ros.utils as rosutils
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.property import Property
import skiros2_msgs.msg as msgs
import skiros2_world_model.ros.world_model_interface as wmi
import skiros2_skill.ros.skill_layer_interface as sli
from copy import deepcopy

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback
from numpy.linalg import norm

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
            self.comboBox_individual.addItem('new ' + utils.ontology_type2name(selected), selected)
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
        print s
        print mp

    def _make_box(self, msg, size):
        marker = Marker()
        marker.type = Marker.CUBE
        if None in size:
            size = [SkirosInteractiveMarkers.default_box_size, SkirosInteractiveMarkers.default_box_size, SkirosInteractiveMarkers.default_box_size]
        marker.scale.x = msg.scale * size[0]
        marker.scale.y = msg.scale * size[1]
        marker.scale.z = msg.scale * size[2]
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.5
        marker.color.a = 0.5
        return marker

    def _make_box_control(self, msg, size):
        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append( self._make_box(msg, size) )
        msg.controls.append( control )
        return control

    def initInteractiveServer(self, name):
        """
        @brief Start the interactive marker server
        """
        self._server = InteractiveMarkerServer(name)

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
        int_marker.scale = 1

        int_marker.name = frame_id
        int_marker.description = frame_id

        # insert a box
        self._make_box_control(int_marker, size)
        int_marker.controls[0].interaction_mode = interaction_mode

        n = norm([1,1])
        control = InteractiveMarkerControl()
        control.orientation.w = 1/n
        control.orientation.x = 1/n
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1/n
        control.orientation.x = 1/n
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1/n
        control.orientation.x = 0
        control.orientation.y = 1/n
        control.orientation.z = 0
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1/n
        control.orientation.x = 0
        control.orientation.y = 1/n
        control.orientation.z = 0
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1/n
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1/n
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1/n
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1/n
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        self._server.insert(int_marker, self.on_marker_feedback)
        self._server.applyChanges()

class SkirosWidget(QWidget, SkirosInteractiveMarkers):
#==============================================================================
#  General
#==============================================================================

    widget_id = 'skiros_gui'

    wm_update_signal = pyqtSignal(msgs.WmMonitor)

    def __init__(self):
        super(SkirosWidget, self).__init__()
        self.setObjectName('SkirosWidget')
        ui_file = os.path.join(rospkg.RosPack().get_path('skiros2_gui'), 'src/skiros2_gui/core', 'skiros_gui.ui')
        loadUi(ui_file, self)

        self.wm_tree_widget.itemSelectionChanged.connect( lambda : self.on_wm_tree_widget_item_selection_changed( self.wm_tree_widget.currentItem()) )
        self.wm_properties_widget.itemChanged.connect( lambda p: self.on_properties_table_item_changed( self.wm_tree_widget.currentItem(), p.row() ) )
        self.wm_relations_widget.resizeEvent = self.on_wm_relations_widget_resized
        self.wm_update_signal.connect(lambda d: self.on_wm_update(d))
        self.reset()


    def reset(self):
        #The plugin should not call init_node as this is performed by rqt_gui_py.
        #Due to restrictions in Qt, you cannot manipulate Qt widgets directly within ROS callbacks,
        #because they are running in a different thread.
        self.initInteractiveServer(SkirosWidget.widget_id)
        self._wmi = wmi.WorldModelInterface(SkirosWidget.widget_id)
        self._sli = sli.SkillLayerInterface(self._wmi)

        #Setup a timer to keep interface updated
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self.refresh_timer_cb)
        self.refresh_timer.start(1000)

        #World model tab
        self._wmi.setMonitorCallback(lambda d: self.wm_update_signal.emit(d))
        self.create_wm_tree()


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
        self.create_wm_tree()


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

        log.debug(self.__class__.__name__, 'Create element based on {}'.format(dialog.object))

        parent = self.wm_tree_widget.currentItem()
        parent_id = parent.text(1)
        if not parent_id: return


        elem = self._wmi.getTemplateElement(dialog.object)
        elem.label = utils.ontology_type2name(dialog.object)
        elem_id = self._wmi.instanciate(elem, recursive=True, relations=[{'src': parent_id, 'type': 'skiros:contain', 'dst': '-1'}])

        # parent = self.wm_tree_widget.currentItem()
        # parent_id = parent.text(1)
        log.debug(self.__class__.__name__, 'Added element {} to {}'.format(elem_id, parent_id))

        # item = QTreeWidgetItem(parent, [utils.ontology_type2name(elem_id), elem_id])
        # self.wm_tree_widget.setCurrentItem(item)


    @Slot()
    def on_remove_object_button_clicked(self):
        item = self.wm_tree_widget.currentItem()
        item_id = item.text(1)
        if not item_id: return
        parent = item.parent()
        self.wm_tree_widget.setCurrentItem(parent)

        elem = self._wmi.getElement(item_id)
        self._wmi.removeElement(elem)

        # parent = self.wm_tree_widget.currentItem()
        # parent_id = parent.text(1)
        log.debug(self.__class__.__name__, 'Removed element {}'.format(item_id))


        #item = self.wm_tree_widget.currentItem()
        # self.remove_wm_tree_widget_item(item)
        # parent.removeChild(item)

    # def remove_wm_tree_widget_item(self, item):
    #     if hasattr(item, 'id'):
    #         log.debug(self.__class__.__name__, 'Removing item: <{}>'.format(item.text(0)))
    #         elem = self._wmi.getElement(item.text(1))
    #         self._wmi.removeElement(elem)
    #     else:
    #         [self.remove_wm_tree_widget_item(child) for child in item.takeChildren()]


    @Slot()
    def on_wm_update(self, data):
        if data.action == 'update':
            cur_item_id = self.wm_tree_widget.currentItem().text(1)
            elems = [e for e in data.elements if e.id == cur_item_id]
            if elems:
                elem = rosutils.msg2element(elems[0])
                self.wm_properties_widget.blockSignals(True)
                self.fill_properties_table(elem)
                self.wm_properties_widget.blockSignals(False)
        else:
            # for now, I'm just doing a lazy update by recreating the whole tree
            log.debug(self.__class__.__name__, '{} scene elements: {}'.format(data.action.capitalize(), [elem.id for elem in data.elements]))
            cur_item_id = self.wm_tree_widget.currentItem().text(1)
            self.create_wm_tree()
            items = self.wm_tree_widget.findItems(cur_item_id, Qt.MatchRecursive | Qt.MatchFixedString, 1)
            if items:
                self.wm_tree_widget.setCurrentItem(items[0])


    def on_marker_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            elem = self._wmi.getElement(feedback.marker_name)
            elem.setData(":PoseStampedMsg", feedback)
            self._wmi.updateElement(elem)

    @Slot()
    def on_wm_tree_widget_item_selection_changed(self, item):
        self.wm_properties_widget.blockSignals(True)
        self.clear_markers()
        if item.text(1):
            elem = self._wmi.getElement(item.text(1))
            self.fill_properties_table(elem)
            self.fill_relations_table(elem)
            if elem.hasProperty("skiros:DiscreteReasoner", "AauSpatialReasoner"):
                p = elem.getData(":Pose")
                size = elem.getData(":Size")
                if not None in p[0] and not None in p[1]:
                    self.make_6dof_marker(p, size, elem.id, elem.getProperty("skiros:BaseFrameId").value, InteractiveMarkerControl.NONE) # NONE,MOVE_3D, MOVe_ROTATE_3D
        else:
            self.wm_properties_widget.setRowCount(0)
            self.wm_relations_widget.setRowCount(0)
        self.wm_properties_widget.blockSignals(False)


    @Slot()
    def on_properties_table_item_changed(self, item, row):
        item_key = self.wm_properties_widget.item(row, 0)
        item_val = self.wm_properties_widget.item(row, 1)
        key = item_key.id
        value = item_val.text()
        elem = self._wmi.getElement(item.text(1))

        if elem.hasProperty(key):
            prop = elem.getProperty(key)
            if value == '': value = None
            if prop.dataTypeIs(bool): value = item_val.checkState() == Qt.Checked
            try:
                elem.setProperty(prop.key, value, is_list=prop.isList(), force_convertion=value is not None)
                log.debug(self.__class__.__name__, '<{}> property {} to {}'.format(item.text(1), prop.key, value))
            except ValueError:
                log.error(self.__class__.__name__, 'Changing <{}> property {} to {} failed'.format(item.text(1), prop.key, value))
                item_val.setText(str(prop.value))
        elif hasattr(elem, key.lower()):
            key = key.lower()
            attr = getattr(elem, key)
            setattr(elem, key, type(attr)(value))
            log.debug(self.__class__.__name__, '<{}> attribute {} to {}'.format(item.text(1), key, value))
        else:
            log.error(self.__class__.__name__, 'Changing <{}> property {} to {} failed'.format(item.text(1), key, val))

        self._wmi.updateElement(elem)
        name = utils.ontology_type2name(elem.id) if not elem.label else utils.ontology_type2name(elem.label)
        item.setText(0, name)



    @Slot()
    def on_wm_relations_widget_resized(self, event):
        width = self.wm_relations_widget.size().width()-2
        cols = self.wm_relations_widget.columnCount()
        for i in range(cols):
            self.wm_relations_widget.setColumnWidth(i, float(width)/cols)


    def create_wm_tree(self):
        scene = {elem.id: elem for elem in self._wmi.getScene()}
        root = scene['skiros:Scene-0']
        self.wm_tree_widget.clear()
        self.wm_tree_widget.setColumnCount(2)
        self.wm_tree_widget.hideColumn(1)
        self._create_wm_tree(self.wm_tree_widget, scene, root)
        self.wm_tree_widget.setCurrentIndex(self.wm_tree_widget.model().index(0, 0))

    def _create_wm_tree(self, item, scene, elem):
        name = utils.ontology_type2name(elem.id) if not elem.label else utils.ontology_type2name(elem.label)
        item = QTreeWidgetItem(item, [name, elem.id])

        spatialRel = sorted(elem.getRelations(subj='-1', pred=self._wmi.getSubProperties('skiros:spatiallyRelated')), key=lambda r: r['dst'])
        for rel in spatialRel:
            self._create_wm_tree(item, scene, scene[rel['dst']])
            item.setExpanded(True)

        skillRel = sorted(elem.getRelations(subj='-1', pred='skiros:hasSkill'), key=lambda r: r['dst'])
        if skillRel:
            skillItem = QTreeWidgetItem(item, ['Skills'])
            for rel in skillRel:
                self._create_wm_tree(skillItem, scene, scene[rel['dst']])
                skillItem.setExpanded(True)

        skillPropRel = sorted(elem.getRelations(subj='-1', pred=self._wmi.getSubProperties('skiros:skillProperty')), key=lambda r: r['dst'])
        for rel in skillPropRel:
            self._create_wm_tree(item, scene, scene[rel['dst']])


    # def update_wm_tree_item(self, item)


    def fill_properties_table(self, elem):
        self.wm_properties_widget.setRowCount(0)
        type =  elem.id[:elem.id.rfind('-')]
        id =  elem.id[elem.id.rfind('-')+1:]
        self._add_properties_table_row(Property('ID', id), editable_value=False)
        self._add_properties_table_row(Property('Type', type), editable_value=False)
        self._add_properties_table_row(Property('Label', elem.label), editable_value=True)
        props = sorted(elem.properties, key=lambda e: e.key)
        [self._add_properties_table_row(p, False, True) for p in props]


    def _add_properties_table_row(self, prop, editable_key=False, editable_value=True):
        key = QTableWidgetItem(utils.ontology_type2name(prop.key))
        if not editable_key: key.setFlags(key.flags() & ~Qt.ItemIsEditable)
        value = prop.values if prop.isList() else prop.value
        if prop.dataTypeIs(float):
            val = QTableWidgetItem(format(value, '.4f') if value is not None else '')
        else:
            val = QTableWidgetItem(str(value) if value is not None else '')
        if not editable_value: val.setFlags(val.flags() & ~Qt.ItemIsEditable)

        if prop.dataTypeIs(bool):
            val.setText('')
            val.setFlags(val.flags() & ~Qt.ItemIsEditable)
            val.setCheckState(Qt.Checked if prop.value else Qt.Unchecked)
        # if isinstance(prop.dataType(), bool):
        #     val.setCheckState(prop.value)

        key.id = val.id = prop.key

        self.wm_properties_widget.insertRow(self.wm_properties_widget.rowCount())
        self.wm_properties_widget.setItem(self.wm_properties_widget.rowCount()-1, 0, key)
        self.wm_properties_widget.setItem(self.wm_properties_widget.rowCount()-1, 1, val)


    def fill_relations_table(self, elem):
        self.wm_relations_widget.setRowCount(0)
        rel = sorted(elem.getRelations(), key=lambda r: r['type'])
        rel = sorted(rel, key=lambda r: r['src'], reverse=True)
        rel = map(lambda r: {
            'src': r['src'] if r['src'] != '-1' else elem.id,
            'type':r['type'],
            'dst': r['dst'] if r['dst'] != '-1' else elem.id
            }, rel)
        [self._add_relations_table_row(r, r['src'] != elem.id, r['dst'] != elem.id) for r in rel]


    def _add_relations_table_row(self, relation, editable_src=False, editable_dst=False):
        src = QTableWidgetItem(utils.ontology_type2name(relation['src']))
        rel = QTableWidgetItem(utils.ontology_type2name(relation['type']))
        dst = QTableWidgetItem(utils.ontology_type2name(relation['dst']))

        src.id = relation['src']
        rel.id = relation['type']
        dst.id = relation['dst']

        src.setTextAlignment(Qt.AlignRight)
        rel.setTextAlignment(Qt.AlignHCenter)
        dst.setTextAlignment(Qt.AlignLeft)

        if not editable_src: src.setFlags(src.flags() & ~Qt.ItemIsEditable)
        rel.setFlags(rel.flags() & ~Qt.ItemIsEditable)
        if not editable_dst: dst.setFlags(dst.flags() & ~Qt.ItemIsEditable)

        self.wm_relations_widget.insertRow(self.wm_relations_widget.rowCount())
        # self.wm_relations_widget.setSpan(self.wm_relations_widget.rowCount()-1, 0, 1, 3)
        self.wm_relations_widget.setItem(self.wm_relations_widget.rowCount()-1, 0, src)
        self.wm_relations_widget.setItem(self.wm_relations_widget.rowCount()-1, 1, rel)
        self.wm_relations_widget.setItem(self.wm_relations_widget.rowCount()-1, 2, dst)










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
        for i in range(0, layout.count()/2):
            key = layout.itemAtPosition(i, 0).widget().text()
            widget = layout.itemAtPosition(i, 1).widget()
            if params[key].dataTypeIs(bool):
                params[key].setValue(widget.isChecked())
            elif params[key].dataTypeIs(Element):
                data = widget.itemData(widget.currentIndex())
                if data:
                    params[key].setValue(self._wmi.getElement(data))
                    #print "Set param {} to {}".format(params[key].key, params[key].value.printState())
            else:
                try:
                    if widget.text():
                        params[key].setValueFromStr(widget.text())
                except ValueError:
                    log.error("getParameters", "Failed to set param {}".format(params[key].key))
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
            if param.paramTypeIs(ParamTypes.Optional):
                combobox.addItem("", None)
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
                if self.modality_checkBox.isChecked() or not p.paramTypeIs(ParamTypes.Optional):
                    self._addParameter(self.skill_params_layout, i, p)
                    i += 1

    @Slot()
    def on_modality_checkBox_clicked(self):
        self.on_skill_combo_box_currentIndexChanged("")

    @Slot()
    def on_skill_exe_button_clicked(self):
        skill = deepcopy(self.skill_combo_box.itemData(self.skill_combo_box.currentIndex()))
        if self._getParameters(self.skill_params_layout, skill.ph):
            self._curr_task = (skill.manager, self._sli.getAgent(skill.manager).execute([skill], SkirosWidget.widget_id))

    @Slot()
    def on_skill_stop_button_clicked(self):
        self._sli.getAgent(self._curr_task[0]).preempt(self._curr_task[1], SkirosWidget.widget_id)

