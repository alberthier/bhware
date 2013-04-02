# encoding: utf-8

import os
import sys
import hashlib
import subprocess

import logger




class Builder:

    def __init__(self, source, output, commands, working_dir):
        self.source = source
        self.output = output
        self.commands = commands
        self.working_dir = working_dir
        self.source_path = os.path.join(self.working_dir, self.source)
        self.output_path = os.path.join(self.working_dir, self.output)
        self.hfile_path = self.output_path + ".bldh"


    def is_up_to_date(self):
        sha1 = hashlib.sha1()
        for cmd in self.commands:
            sha1.update(cmd.encode())
        f = open(self.source_path, "rb")
        for line in f:
            sha1.update(line)
        f.close()
        newhash = sha1.hexdigest()

        if os.path.exists(self.hfile_path) and os.path.exists(self.output_path):
            f = open(self.hfile_path)
            oldhash = f.readline()
            f.close()

            if oldhash == newhash:
                return (True, oldhash)

        return (False, newhash)


    def build(self):
        (up_to_date, newhash) = self.is_up_to_date()
        if up_to_date:
            return True

        if os.path.exists(self.output_path):
            os.remove(self.output_path)
        logger.log(self.commands)
        proc = subprocess.Popen(self.commands, stdout = subprocess.PIPE, stderr = subprocess.PIPE, cwd=self.working_dir)
        (out, err) = proc.communicate()
        if proc.returncode == 0:
            logger.log("'{}' compiled successfully".format(self.source))
        for l in out.splitlines():
            logger.log(str(l, "utf-8"))
        for l in err.splitlines():
            logger.log(str(l, "utf-8"))

        f = open(self.hfile_path, "w")
        f.write(newhash)
        f.close()

