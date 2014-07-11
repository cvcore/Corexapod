A hexapod robot developed on Raspberry Pi by STC Dev Team.

Prerequsites:
	Eigen http://eigen.tuxfamily.org
	Boost http://boost.org
	wiringPi http://wiringpi.com/download-and-install/

Build:
	mkdir build
	cd build
	cmake ..
	make

Running:
	bin/cdm (Daemon)
	python backend.py xxx.xxx.xxx.xxx