#!/usr/bin/env python
import subprocess

DIRECTORIES = {
	"ros_source" : "/home/miguel/bebop_WS/devel/setup.bash",
	"ros_repo" : "~/bebop_WS/"	
}

XBOX = {
	"jso" : "/dev/input/js0"
}
subprocess.call(["sudo","ln","-sf","/dev/input/js0"])

