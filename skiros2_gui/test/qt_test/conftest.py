import os
import rospy
import pytest
import skiros2_world_model.ros.world_model_interface as wmi
from skiros2_common.core.world_element import Element
from pytestqt import qt_compat
from pytestqt.qt_compat import qt_api
from python_qt_binding.QtCore import Qt
import python_qt_binding.QtCore as QtCore
from skiros2_gui.core.skiros_widget import SkirosWidget
import shutil

@pytest.fixture(scope="module", autouse=True)
def initial_fixture():
    rospy.init_node("gui_test_node")

@pytest.fixture(scope="module")
def wm_tmp_dir():
    tmp_dir = 'tmp_dir'
    init_scene = 'init_scene.turtle'
    tmp_name = os.path.join(tmp_dir, init_scene)
    wm_inst = wmi.WorldModelInterface()
    wm_inst.save(tmp_name)
    yield [tmp_dir, init_scene]
    #tear down
    tmp_path = wm_file_path(tmp_dir)
    shutil.rmtree(tmp_path)

@pytest.fixture
def skiros_bot(qtbot, wm_tmp_dir):
    bot = SkirosBot(qtbot, *wm_tmp_dir)
    yield bot
    bot.tear_down()

def wm_file_path(filename):
    "Return the abs path to a file in the world model workspace_dir"
    ws_dir = rospy.get_param("/wm/workspace_dir")
    return os.path.join(ws_dir, filename)

class SkirosBot:
    """Simple bot for interacting with the SkirosWidget.

    Automatically removes created files after each test and ensures that
    changes made on the world model server are not propagted between tests.

    Attributes:
        qtbot ()
        tmp_dir (str): Name of base temporary directory.
        init_scene (str): Name of fixture scene.
        widget (SkirosWidget)
        wmi (WorldModelInterface)
    """
    def __init__(self, qtbot, tmp_dir, init_scene):
        self.qtbot = qtbot
        self.tmp_dir = tmp_dir
        self.init_scene = init_scene
        self.widget = SkirosWidget()
        self.qtbot.addWidget(self.widget)
        self.wmi = wmi.WorldModelInterface()
        self._file_paths = []

    def save_scene(self, filename):
        """Save a scene in a temporary directory through the gui.

        The created file will be automatically removed after the test.
        """
        tmp_name = os.path.join(self.tmp_dir, filename)
        self._file_paths.append(wm_file_path(tmp_name))
        self.widget.scene_file_lineEdit.setText(tmp_name)
        self.qtbot.mouseClick(self.widget.save_scene_button, QtCore.Qt.LeftButton)

    def load_scene(self, filename, tmp_file = True):
        """Load a scene through the gui.

        Args:
            filename (str): name of the file
            tmp_file (bool): if the file is in a temporary directory
        """
        if tmp_file:
            tmp_name = os.path.join(self.tmp_dir, filename)
            self.widget.scene_file_lineEdit.setText(tmp_name)
        else:
            self.widget.scene_file_lineEdit.setText(filename)
        self.qtbot.mouseClick(self.widget.load_scene_button, QtCore.Qt.LeftButton)

    def add_object(self, element, parent = 'skiros:Scene-0'):
        """Add an object to the scene

        Args:
            element (Element): The element to be added.
            parent (str): The parent of the element in the scene graph.
        Return:
            The instanciated elment.
        """
        self.wmi.instanciate(element,
            relations=[{'src': parent, 'type': 'skiros:contain', 'dst': '-1'}])
        element_inst = self.wmi.resolve_element(element)
        return element_inst

    def remove_object(self, element_inst):
        """Remove an object from the scene

        Args:
            element (Element): The element to be removed. Note that it has to be instanciated.
        """
        items = self.wm_tree_items(element_inst.id)
        self.widget.wm_tree_widget.setCurrentItem(items[0])
        self.qtbot.mouseClick(self.widget.remove_object_button, QtCore.Qt.LeftButton)

    def gui_has_element(self, element_inst):
        "Test if gui has registered the element"
        items = self.wm_tree_items(element_inst.id)
        return items != []

    def wm_tree_items(self, id):
        "Find elements in wm_tree_widget matching a string"
        self.widget.create_wm_tree()
        return self.widget.wm_tree_widget.findItems(id, Qt.MatchRecursive | Qt.MatchFixedString, 1)

    def get_tmp_path(self, filename):
        "Get the path to a file in the temporary directory"
        tmp_name = os.path.join(self.tmp_dir, filename)
        return wm_file_path(tmp_name)

    def start_skill(self, skill):
        items = self.skill_tree_items(skill)
        self.widget.skill_tree_widget.setCurrentItem(items[0])
        self.qtbot.mouseClick(self.widget.skill_exe_button, QtCore.Qt.LeftButton)

    def skill_tree_items(self, skill):
        self.widget.create_skill_tree()
        return self.widget.skill_tree_widget.findItems(skill, Qt.MatchRecursive | Qt.MatchFixedString, 1)

    def reset_scene(self):
        "Clear the changes on the wm server"
        self.load_scene(self.init_scene, tmp_file = True)

    def tear_down(self):
        self.reset_scene()
        for path in self._file_paths:
            os.remove(path)
