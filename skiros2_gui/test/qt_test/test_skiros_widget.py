import os
import rospy
from skiros2_common.core.world_element import Element

def test_remove_element(skiros_bot):
    "test that an element can be removed from wm through the gui"
    element = Element(":testType")
    element_inst = skiros_bot.add_object(element)
    #test that element is there
    assert skiros_bot.gui_has_element(element_inst)
    skiros_bot.remove_object(element_inst)
    #test that it was removed
    assert not skiros_bot.gui_has_element(element_inst)

def test_simple_save_scene(skiros_bot):
    """Simple test for saving a scene.

    This test only tests that the file was created."""
    filename = "test_scene.turtle"
    file_path = skiros_bot.get_tmp_path(filename)
    assert os.path.isfile(file_path) == False
    skiros_bot.save_scene(filename)
    assert os.path.isfile(file_path) == True

def test_save_and_load_scene(skiros_bot):
    "Test saving a scene and loading it back"
    #setup a scene and save it
    scene_name = "test_scene.turtle"
    element = Element(":testType")
    element_inst = skiros_bot.add_object(element)
    skiros_bot.save_scene(scene_name)
    #reset the scene to clear the changes
    skiros_bot.reset_scene()
    assert not skiros_bot.gui_has_element(element_inst)
    #load the changes back in
    skiros_bot.load_scene(scene_name)
    assert skiros_bot.gui_has_element(element_inst)

def test_start_skill(skiros_bot):
    skill = 'test_primitive'
    #it is necessary to wait before calling start_skill for some reason
    rospy.sleep(0.5)
    skiros_bot.start_skill(skill)
    assert skiros_bot.widget.last_executed_skill == skill
