#!/usr/bin/env python3

import os
import http.client
import datetime

content_type = "text/plain"

try:
    def print_dir_entry(directory, name, is_file):
        path = os.path.join(directory, name)
        if is_file:
            cls = "file"
            icon = "&#9750;"
        else:
            cls = "folder"
            icon = "&#9751;"
        if os.path.islink(path):
            cls += " link"
            icon = "&#8599;"
        out = "<tr class=\"{}\">".format(cls)
        out += "<td>{}</td>".format(icon)
        out += "<td><a href={0}>{0}</a></td>".format(name)
        stat = os.lstat(path)
        if stat.st_size < 1024:
            size = str(stat.st_size) + " B"
        elif stat.st_size < 1024 * 1024:
            size = "{:0.2f} K".format(stat.st_size / 1024)
        else:
            size = "{:0.2f} M".format(stat.st_size / (1024 * 1024))
        out += "<td>{}</td>".format(size)
        mtime = str(datetime.datetime.fromtimestamp(stat.st_mtime))[:19]
        out += "<td>{}</td>".format(mtime)
        out += "</tr>"
        return out


    def create_html():
        out = "<!DOCTYPE html>\n"
        directory = os.environ["REQUEST_URI"]
        out += """<html>
<head>
  <title>{0}</title>
  <style>
.folder {{
    background-color: #ff6;
}}
.link {{
    font-style: italic;
}}
  </style>
</head>
<body>
<h3>{0}</h3>
<table>
""".format(directory)
        dirs = []
        if directory != "/":
            dirs.append("..")
        files = []
        for f in os.listdir(directory):
            path = os.path.join(directory, f)
            if os.path.isdir(path):
                dirs.append(f)
            else:
                files.append(f)
        dirs.sort()
        files.sort()
        for d in dirs:
            out += print_dir_entry(directory, d, False)
        for f in files:
            out += print_dir_entry(directory, f, True)
        out += "</table>\n</body>\n</html>\n"
        return out

    if "text/html" in os.environ["HTTP_ACCEPT"]:
        content_type = "text/html"
        out = create_html()
    else:
        out = ""
        items = os.listdir(os.environ["REQUEST_URI"])
        items.sort()
        for i in items:
            out += i + "\n"

except Exception as e:
    import traceback
    content_type = "text/plain"
    out = "".join(traceback.format_exception(type(e), e, None))

m = http.client.HTTPMessage()
m.add_header("Content-Type", content_type)
m.add_header("Content-Length", str(len(out)))
m.set_payload(out)

print("200 OK")
print(str(m))
