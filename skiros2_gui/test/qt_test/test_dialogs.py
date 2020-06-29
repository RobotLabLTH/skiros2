from skiros2_gui.core.skiros_widget import SkirosWidget, SkirosAddObjectDialog

def test_AddObjectDialog_select_type(qtbot):
    "Test that subtype comboboxes are handled correctly when selecting type."
    widget = SkirosWidget()
    dialog = SkirosAddObjectDialog(widget)
    qtbot.addWidget(dialog)
    qtbot.addWidget(widget)
    #Select 'Location' as type and test that a new subtype combobox was added
    assert len(dialog._comboBoxes) == 1
    index = dialog._comboBoxes[0].findText('Location')
    dialog._comboBoxes[0].setCurrentIndex(index)
    assert len(dialog._comboBoxes) == 2
    #Reset type to 'all' and test that subtype combobox was removed
    index = dialog._comboBoxes[0].findText('All')
    dialog._comboBoxes[0].setCurrentIndex(index)
    assert len(dialog._comboBoxes) == 1

def test_AddObjectDialog_type_restriction(qtbot):
    "Test that number of possible individuals decreases when restricting type."
    widget = SkirosWidget()
    dialog = SkirosAddObjectDialog(widget)
    qtbot.addWidget(dialog)
    qtbot.addWidget(widget)
    all_individuals = dialog.comboBox_individual.count()
    index = dialog._comboBoxes[0].findText('Location')
    dialog._comboBoxes[0].setCurrentIndex(index)
    assert dialog.comboBox_individual.count() < all_individuals
