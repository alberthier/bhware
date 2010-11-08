#!/usr/bin/env python
# encoding: utf-8

import os
import stat




class Logger(object):

    __shared_state = None

    def __init__(self):
        # Singleton (Borg) pattern, see http://fr.wikipedia.org/wiki/Singleton_%28patron_de_conception%29#Python
        if Logger.__shared_state == None:
            # First time initialization
            Logger.__shared_state = {}
            self.__dict__ = Logger.__shared_state

            self.filepath = self.get_next_log_filepath()
            self.log_file = file(self.filepath, "w")
            self.log_file.write("#!/usr/bin/env python\n")
            self.log_file.write("# encoding: utf-8\n\n")
            self.log_file.write("log = []\n\n")
        else:
            self.__dict__ = Logger.__shared_state


    def close(self):
        self.log_file.write("\nif __name__ == '__main__':\n")
        self.log_file.write("    for line in log:\n")
        self.log_file.write("        print(line)\n")
        self.log_file.close()
        os.chmod(self.filepath, stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR | stat.S_IRGRP | stat.S_IXGRP | stat.S_IROTH | stat.S_IXOTH)
        Logger.__shared_state = None


    def log_packet(self, packet):
        self.log_file.write("log.append(" + str(packet.to_dict()) + ")\n")


    def log(self, text):
        self.log_file.write("log.append(\"" + text + "\")\n")


    def get_next_log_filepath(self):
        brewery_root_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        log_dir = os.path.join(brewery_root_path, "logs")
        index = 0
        if not os.path.exists(log_dir):
            os.mkdir(log_dir)
        while True:
            filepath = os.path.join(log_dir, "brewerylog_{0:=#04}.py".format(index))
            if os.path.exists(filepath):
                index += 1
            else:
                return filepath
