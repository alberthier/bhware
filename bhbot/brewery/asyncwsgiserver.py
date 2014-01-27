# encoding: utf-8


import asynchat
import asyncore
import io
import socket
import sys
import urllib
import wsgiref.handlers

import logger


def is_port_open(ip,port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.connect((ip, int(port)))
        s.shutdown(2)
        return True
    except:
        return False


def find_open_port(host, start, end):
    for p in range(start, end+1):
        if not is_port_open(host, p) :
            return p
    return None




class WsgiRequestHandler(asynchat.async_chat):

    READING_HTTP_HEADER = 0
    READING_HTTP_POST_DATA = 1
    READING_DONE = 2


    def __init__(self, sock, address, server):
        asynchat.async_chat.__init__(self, sock)
        self.address = address
        self.server = server
        self.ibuffer = bytes()
        self.environ = {}
        self.post_data = io.BytesIO()
        self.set_terminator(b"\r\n\r\n")
        self.state = WsgiRequestHandler.READING_HTTP_HEADER


    def collect_incoming_data(self, data):
        self.ibuffer += data


    def found_terminator(self):
        if self.state == WsgiRequestHandler.READING_HTTP_HEADER:
            self.parse_header()
            if self.environ["REQUEST_METHOD"] == "POST":
                if "CONTENT_LENGTH" in self.environ:
                    content_length = int(self.environ["CONTENT_LENGTH"])
                    self.set_terminator(content_length)
                else:
                    self.set_terminator(b"\0")
                self.state = WsgiRequestHandler.READING_HTTP_POST_DATA
            else:
                self.set_terminator(None)
                self.state = WsgiRequestHandler.READING_DONE
            self.ibuffer = bytes()
        elif self.state == WsgiRequestHandler.READING_HTTP_POST_DATA:
            self.set_terminator(None)
            self.post_data.write(self.ibuffer)
            self.post_data.seek(0)
            self.ibuffer = bytes()
            self.state = WsgiRequestHandler.READING_DONE
        if self.state == WsgiRequestHandler.READING_DONE:
            errors = io.StringIO()
            self.server.current_environ = self.environ.copy()
            handler = wsgiref.handlers.SimpleHandler(self.post_data, self, errors, self.environ, False, False)
            handler.server_software = "BHWebServer/2012" + " Python/" + sys.version.split()[0]
            handler.run(self.server.get_app())
            if len(errors.getvalue()) != 0:
                logger.log(errors.getvalue())


    def parse_header(self):
        lines = self.ibuffer.split(b"\r\n")
        request_line = str(lines[0], "ascii").lstrip()
        lines = lines[1:]
        request_line.lstrip()
        request_parts = request_line.split()
        if len(request_parts) != 3:
            raise Exception("Bad HTTP protocol")
        self.environ["REQUEST_METHOD"] = request_parts[0]
        self.environ["SERVER_PROTOCOL"] = request_parts[2]
        path = request_parts[1]
        idx = path.find('?')
        if idx != -1:
            self.environ["PATH_INFO"] = path[: idx]
            self.environ["QUERY_STRING"] = path[idx + 1 :]
        else:
            self.environ["PATH_INFO"] = path
            self.environ["QUERY_STRING"] = ""
        for header in lines:
            name, value = str(header, "ascii").split(':', 1)
            value = value.strip()
            name = name.strip().replace('-', '_').upper()
            if name in ["CONTENT_LENGTH", "CONTENT_TYPE"]:
                self.environ[name] = value
            else:
                name = "HTTP_" + name
                if name in self.environ:
                    self.environ[name] += "," + value
                else:
                    self.environ[name] = value
        self.environ["SCRIPT_NAME"] = ""
        self.environ["SERVER_NAME"] = self.server.host
        self.environ["SERVER_PORT"] = str(self.server.port)
        self.environ['REMOTE_HOST'] = self.address[0]
        self.environ['REMOTE_ADDR'] = self.address[0]


    def write(self, data):
        self.push(data)


    def flush(self):
        pass




class WsgiServer(asyncore.dispatcher):

    def __init__(self, host, port, application):
        asyncore.dispatcher.__init__ (self)
        self.host = host
        self.port = port
        self.application = application
        self.current_environ = {}
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_reuse_addr()
        bind_port = find_open_port(host, port, port+1001)
        logger.log("Webserver port is : {}".format(bind_port))
        self.bind((self.host, bind_port))
        self.listen(5)


    def handle_accepted(self, sock, addr):
        WsgiRequestHandler(sock, addr, self)


    def get_app(self):
        return self.application
