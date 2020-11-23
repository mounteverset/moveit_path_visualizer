#!/usr/bin/env python

import sys

from rqt_mypkg.my_module import MyPlugin
from rqt_gui.main import Main

plugin = 'rqt_mypkg'
main = Main(filename=plugin)

MyPlugin.Test.clicked.connect(on_button_click)

def on_button_click():
    print("geklickt!")
    
sys.exit(main.main(standalone=plugin))