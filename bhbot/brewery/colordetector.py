# encoding: utf-8

import os
import subprocess
import time

import builder
import logger




class ColorDetector:

    def __init__(self):
        self.count = 1
        self.process = None
        working_dir = os.path.join(os.path.dirname(__file__), "colordetector")
        cmds = ["g++", "-o", "colordetector", "-l", "opencv_core", "-l", "opencv_highgui", "colordetector.cpp"]
        bld = builder.Builder("colordetector.cpp", "colordetector", cmds, working_dir)
        bld.build()


    def quit(self):
        if self.process is not None:
            self.process.stdin.write(b"quit\n")
            self.process.wait()


    def reinit(self):
        self.quit()
        exe = os.path.join(os.path.dirname(__file__), "colordetector", "colordetector")
        self.process = subprocess.Popen([exe], stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.PIPE)
        cfg_path = exe + ".cfg"
        if os.path.exists(cfg_path):
            cfg = open(cfg_path)
            for line in cfg:
                self.invoke(line[:-1])
        log = "{}-{}.jpg".format(os.path.splitext(logger.filepath)[0], self.count)
        self.invoke("set_log_file", log)
        self.count += 1


    def invoke(self, *args):
        ret = None
        if self.process:
            cmd = " ".join(args) + "\n"
            self.process.stdin.write(cmd.encode("utf-8"))
            out = self.process.stdout.readline()
            ret = eval(out)
        return ret

