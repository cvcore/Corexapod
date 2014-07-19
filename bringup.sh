#!/bin/bash
bin/cdm >engine.log 2>&1 &
(cd src/frontend/Server && python backend.py 192.168.1.7) >frontend.log 2>&1 &
