import socket
import sys
import web
import string

HOST, PORT = "localhost", 50000

urls = (
  "/", "index",
  "/act/(.*)", "act"
)

class index:
  def GET(self):
    web.HTTPError('301', {'Location': 'static/'})

class act:
  def GET(self, name):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    s.send(name);
    while 1:
      szBuf = s.recv(1024);
      if szBuf:
        break
    s.close();
    return szBuf;

if __name__ == "__main__":
  app = web.application(urls, globals())
  app.run()
