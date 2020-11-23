#!/usr/bin/env python

import sys

from rqt_mypkg.my_module import MyPlugin
from rqt_gui.main import Main

plugin = 'rqt_mypkg'
main = Main(filename=plugin)
Test.clicked.connect(on_button_clicked)
#main.Test.clicked.connect(on_button_click)


sys.exit(main.main(standalone=plugin))