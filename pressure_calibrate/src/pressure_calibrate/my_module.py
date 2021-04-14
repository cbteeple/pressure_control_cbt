import os
import rospy
import rospkg
import actionlib
import copy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import qDebug, Qt
from python_qt_binding.QtWidgets import QWidget, QLayout, QVBoxLayout, QHBoxLayout, QSlider, QSpinBox, QPushButton
from qt_gui_py_common.simple_settings_dialog import SimpleSettingsDialog

import pressure_controller_ros.msg

try:
    from pyqtgraph_data_plot import PyQtGraphDataPlot
except ImportError:
    qDebug('[DEBUG] rqt_plot.plot: import of PyQtGraphDataPlot failed (trying other backends)')
    PyQtGraphDataPlot = None

try:
    from mat_data_plot import MatDataPlot
except ImportError:
    qDebug('[DEBUG] rqt_plot.plot: import of MatDataPlot failed (trying other backends)')
    MatDataPlot = None

try:
    from qwt_data_plot import QwtDataPlot
except ImportError:
    qDebug('[DEBUG] rqt_plot.plot: import of QwtDataPlot failed (trying other backends)')
    QwtDataPlot = None

class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('pressure_calibrate'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')

        self.settings=self.get_settings()
        
        self.init_sliders()
        self.init_config()

        self._client = actionlib.SimpleActionClient('pressure_control', pressure_controller_ros.msg.CommandAction)
        self._client_connected=self._client.wait_for_server(timeout=rospy.rostime.Duration(1))

        if not self._client_connected:
            print("No command server avaiable... changes will not be sent")

        self.set_graph_state(True)
        self.send_channel_state(0,self.settings['channel_states'][0])


        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)


    def init_config(self):
        self._plot_type_index = 0
        self.plot_types = [
            {
                'title': 'PyQtGraph',
                'widget_class': PyQtGraphDataPlot,
                'description': 'Based on PyQtGraph\n- installer: http://luke.campagnola.me/code/pyqtgraph',
                'enabled': True,#PyQtGraphDataPlot is not None,
            },
            {
                'title': 'MatPlot',
                'widget_class': MatDataPlot,
                'description': 'Based on MatPlotLib\n- needs most CPU\n- needs matplotlib >= 1.1.0\n- if using PySide: PySide > 1.1.0',
                'enabled': True,#MatDataPlot is not None,
            },
            {
                'title': 'QwtPlot',
                'widget_class': QwtDataPlot,
                'description': 'Based on QwtPlot\n- does not use timestamps\n- uses least CPU\n- needs Python Qwt bindings',
                'enabled': True,#QwtDataPlot is not None,
            },
        ]

    def init_sliders(self):
        sliderbox = self._widget.findChild(QLayout,'Sliders')

        graph_button=QPushButton()
        graph_button.setCheckable(True)
        graph_button.setText("Graph Off")
        graph_button.toggle()
        graph_button.clicked.connect(self.set_graph_state)
        self.graph_button = graph_button

        sliderbox.addWidget(graph_button)

        self.sliders = []

        for i in range(self.settings['num_channels']):
            idx = i*1

            slider_group={'slider_p':None, 'number_p':None, 'slider_v':None, 'number_v':None, 'on_off':None}

            layout_cluster = QVBoxLayout()
            slider_cluster = QHBoxLayout()
            for j in range(2):
                layout = QVBoxLayout()
                layout.setAlignment(Qt.AlignHCenter)

                slider = QSlider(Qt.Vertical)
                slider.setMinimum(0)
                slider.setMaximum(255)
                slider.setValue(self.settings['valve_offsets'][i][j])
                slider.setTickPosition(QSlider.TicksRight)
                slider.setTickInterval(5)

                spinbox = QSpinBox()
                spinbox.setRange(0, 255)
                spinbox.setValue(self.settings['valve_offsets'][i][j])

                slider.valueChanged.connect(spinbox.setValue)
                spinbox.valueChanged.connect(slider.setValue)

                cb_function_curr = lambda value, idx=idx: self.send_slider_value(idx,value)
                slider.valueChanged.connect(cb_function_curr)

                layout.addWidget(slider, Qt.AlignHCenter)
                layout.addWidget(spinbox, Qt.AlignHCenter)
                slider_cluster.addLayout(layout)

                if j==0:
                    slider_group['slider_p'] = slider
                    slider_group['number_p'] = spinbox
                else:
                    slider_group['slider_v'] = slider
                    slider_group['number_v'] = spinbox

            on_button=QPushButton()
            on_button.setCheckable(True)
            on_button.setText("Off")

            if self.settings['channel_states'][i]:
                on_button.toggle()
                on_button.setText("On")

            on_button.clicked.connect(lambda state, idx=idx: self.send_channel_state(idx,state))

            slider_group['on_off'] = on_button

            layout_cluster.addLayout(slider_cluster)
            layout_cluster.addWidget(on_button)

            sliderbox.addLayout(layout_cluster)
            sliderbox.addSpacing(20)

            self.sliders.append(slider_group)

        #self._widget.setLayout(sliderbox)

    def get_settings(self):
        settings={}
        settings['channel_states'] = rospy.get_param('/config_node/channels/states',[1,1,1,1,1,1,1,1,1,1])
        settings['num_channels']   = len(settings['channel_states'])
        settings['valve_offsets']  = rospy.get_param('/config_node/valve_offsets',[[220,220]]*settings['num_channels'])

        return settings

    def send_slider_value(self, idx, value):

        v1 = self.sliders[idx]['slider_p'].value()
        v2 = self.sliders[idx]['slider_v'].value()

        self.settings['valve_offsets'][idx][0]=v1
        self.settings['valve_offsets'][idx][1]=v2

        if self._client_connected:
            # Send commands to the command server and wait for things to be taken care of
            goal = pressure_controller_ros.msg.CommandGoal(command='voffset', args=[idx, v1, v2], wait_for_ack = False)
            self._client.send_goal(goal)
            self._client.wait_for_result()

            if not self._client.get_result():
                raise ('Something went wrong and a setting was not validated')
                pass
            else:
                pass
        else:
            print(idx, v1, v2)


    def send_channel_state(self, idx, state):
        if state:
            self.settings['channel_states'][idx] = 1
            self.sliders[idx]['on_off'].setText("On")
        else:
            self.settings['channel_states'][idx] = 0
            self.sliders[idx]['on_off'].setText("Off")
        
        if self._client_connected:
            # Send commands to the command server and wait for things to be taken care of
            goal = pressure_controller_ros.msg.CommandGoal(command='chan', args=self.settings['channel_states'], wait_for_ack = False)
            self._client.send_goal(goal)
            self._client.wait_for_result()

            if not self._client.get_result():
                raise ('Something went wrong and a setting was not validated')
                pass
            else:
                pass
        else:
            print(self.settings['channel_states'])


    def set_graph_state(self, value):

        if value:
            on_off_str='on'
            self.graph_button.setText("Graph ON")
        else:
            on_off_str='off'
            self.graph_button.setText("Graph OFF")


        if self._client_connected:
            # Send commands to the command server and wait for things to be taken care of
            goal = pressure_controller_ros.msg.CommandGoal(command=on_off_str, args=[], wait_for_ack = False)
            self._client.send_goal(goal)
            self._client.wait_for_result()
        else:
            print(on_off_str)



    def shutdown_sliders(self):
        print("")
        print("Final Valve Offsets:")
        print(self.settings['valve_offsets'])
        print("")
        print("Copy these settings into 'valve_offsets' in your control config file")

        self.set_graph_state(False)

        for slider_group in self.sliders:
            for widget in slider_group:
                if 'on_off' in widget:
                    slider_group[widget].clicked.disconnect()
                else:
                    slider_group[widget].valueChanged.disconnect()
        

    def shutdown_plugin(self):
        self._client.cancel_all_goals()
        self.shutdown_sliders()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def trigger_configuration(self):
        dialog = SimpleSettingsDialog(title='Plot Options')
        dialog.add_exclusive_option_group(title='Plot Type', options=self.plot_types, selected_index=self._plot_type_index)
        plot_type = dialog.get_settings()[0]
        if plot_type is not None:
            qDebug(str(plot_type['selected_index']))
        if plot_type is not None and plot_type['selected_index'] is not None and self._plot_type_index != plot_type['selected_index']:
            qDebug('[DEBUG] SWITCHING')
            self._plot_type_index=plot_type['selected_index']