import Rhino
import Eto
import Eto.Drawing as drawing
import Eto.Forms as forms

from compas_fab.robots import RobotCellLibrary
from compas.scene import Scene



class EtoFormSliderTest(forms.Dialog[bool]):
    def __init__(self):        
        self.Title = "Sample Slider Eto Test"
        self.Padding = drawing.Padding(5)
        self.ClientSize = drawing.Size(300,50)
        self.Resizable = True
        
        self.slider = forms.Slider()
        self.slider.MinValue = 1
        self.slider.MaxValue = 50
        self.slider.Value = 5
        self.slider.ValueChanged += self.OnSliderValueChanged
        self.sliderLabelTest = forms.Label(Text = str(self.slider.Value))
        
        layout= forms.DynamicLayout()
        layout.AddRow(None)
        layout.AddRow(self.slider)
        layout.AddRow(self.sliderLabelTest)
        
        self.Content = layout
        
        # Create Robot and Scene Object
        robot_cell, robot_cell_state = RobotCellLibrary.ur5(load_geometry=True)
        scene = Scene()
        self.scene_object = scene.add(robot_cell.robot_model)
        self.scene_object.draw()

        
    def OnSliderValueChanged(self,sender,e):
        value = self.slider.Value * 0.1
        print(value)
        self.sliderLabelTest.Text = str(value)
        configuration = self.scene_object.item.zero_configuration()
        configuration.joint_values[0] = value
        self.scene_object.update(configuration)
#        self.scene_object.clear_layer()
        self.scene_object.draw()

        
def OpenEtoWindow():
    form = EtoFormSliderTest()
    Rhino.UI.EtoExtensions.ShowSemiModal(form, Rhino.RhinoDoc.ActiveDoc, Rhino.UI.RhinoEtoApp.MainWindow)

if __name__ == '__main__':
    OpenEtoWindow()