CorexapodWebController
======================

Frontend and gateway of a hexapod controller

Before you build this Web App, please use `$ bower install` to install front-end dependency.

Then, run `$ grunt build` to build the Web App and copy all files under app/dist/ into ../Server/static.

Run `$ python ../Server/backend.py 8080` to get the Web App running and listening on TCP port 8080.

Before you run this Web App, you shoul run the CorexapodWebController-Daemon on TCP port 50000.
