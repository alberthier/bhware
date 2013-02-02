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


    def is_up_to_date(self):
        src = os.path.join(self.working_dir, self.source)
        dst = os.path.join(self.working_dir, self.output)
        sha1 = hashlib.sha1()
        for cmd in self.commands:
            sha1.update(cmd.encode())
        f = open(src, "rb")
        for line in f:
            sha1.update(line)
        f.close()
        newhash = sha1.hexdigest()

        hfile = dst + ".bldh"
        if os.path.exists(hfile) and os.path.exists(dst):
            f = open(hfile)
            oldhash = f.readline()
            f.close()

            if oldhash == newhash:
                return True

        f = open(hfile, "w")
        f.write(newhash)
        f.close()

        return False


    def build(self):
        if self.is_up_to_date():
            return True
        logger.log(self.commands)
        proc = subprocess.Popen(self.commands, stdout = subprocess.PIPE, stderr = subprocess.PIPE, cwd=self.working_dir)
        (out, err) = proc.communicate()
        if proc.returncode == 0:
            logger.log("'{}' compiled successfully".format(self.source))
        for l in out.splitlines():
            logger.log(l)
        for l in err.splitlines():
            logger.log(l)
