# encoding: utf-8

import urllib.parse
import os
import os.path




class StaticDir:

    def __init__(self, root):
        self.root = root


    def get(self, splitted_path, headers, vars):
        fullpath = os.path.join(self.root, *splitted_path[1:])
        if not os.path.exists(fullpath):
            raise OSError(fullpath)
        if os.path.isdir(fullpath):
            return self.listdir(splitted_path, fullpath, headers, vars)
        else:
            return self.getfile(splitted_path, fullpath, headers, vars)


    def listdir(self, splitted_path, fullpath, headers, vars):
        path = '/'.join(splitted_path)
        response = "<!doctype html><html><head><title>{}</title></head><body><table>".format(path)
        for (root, dirs, files) in os.walk(fullpath):
            dirs.sort()
            files.sort()
            tpl = "<tr><td><a href='/{}'>{}</a></td></tr>"
            for d in dirs:
                response += tpl.format(path + '/' + d, d)
            for f in files:
                response += tpl.format(path + '/' + f, f)
            break
        response += "</table></body></html>"
        return ("text/html", response)


    def getfile(self, splitted_path, fullpath, headers, vars):
        f = open(fullpath, "rb")
        buf = f.read()
        f.close()
        return (None, buf)




class Application:
    """
    WSGI application.
    Use this class with a WSGI web server to expose the handler's methods
    """

    def __init__(self, handler):
        """
        handler is the class implementing the callbacks used to respond to HTTP requests
        """
        self.handler = handler


    def convert_header_name(self, header_var):
        header = ""
        first = True
        for c in header_var:
            if c == "_":
                first = True
                header += "-"
            else:
                if first:
                    header += c
                    first = False
                else:
                    header += c.lower()
        return header


    def build_headers(self, environ):
        headers = {}
        for name, value in environ.items():
            if name in ["CONTENT_LENGTH", "CONTENT_TYPE"]:
                headers[self.convert_header_name(name)] = value
            if name.startswith("HTTP_"):
                headers[self.convert_header_name(name[5:])] = value
        return headers


    def build_vars(self, environ):
        args = {}
        for name, values in urllib.parse.parse_qs(environ["QUERY_STRING"], True).items():
            args[name] = values[-1]
        content_length = environ.get("CONTENT_LENGTH", -1)
        content_type = environ.get("CONTENT_TYPE", "")
        if content_type == "application/x-www-form-urlencoded" and content_length != -1:
            post = environ['wsgi.input'].read(content_length)
            for name, values in urllib.parse.parse_qs(post, True).items():
                args[name] = values[-1]
        return args


    def __call__(self, environ, start_response):
        url = urllib.parse.unquote(environ["PATH_INFO"])
        splitted_path = url[1:].split('/')
        success = False
        mime = None
        if len(splitted_path[0]) == 0 :
            splitted_path = ['index']
        if hasattr(self.handler, splitted_path[0]):
            try:
                headers = self.build_headers(environ)
                vars = self.build_vars(environ)
                response_handler = getattr(self.handler, splitted_path[0])
                if isinstance(response_handler, StaticDir):
                    response = response_handler.get(splitted_path, headers, vars)
                else:
                    response = response_handler(headers, vars)
                if type(response) is str:
                    mime = "text/html"
                    content = response
                else:
                    (mime, content) = response
                success = True
            except Exception as e:
                mime = "text/plain"
                content = str(e)
        else:
            content = "'{}' not found".format(environ["PATH_INFO"])

        if success:
            status = '200 OK'
        else:
            status = '404 Not Found'

        response_headers = [('Content-Length', str(len(content))),
                            ('Cache-Control', 'no-cache')]

        if mime is not None:
            if  mime.startswith("text/"):
                mime += "; charset=utf-8"
            response_headers.append(('Content-Type', mime))
        start_response(status, response_headers)
        if type(content) is str:
            return [bytes(content, 'utf-8')]
        else:
            return [content]
