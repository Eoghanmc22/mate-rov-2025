#!/bin/bash

if v4l2-ctl -d $1 -l | grep focus; then
	v4l2-ctl -d $1 --set-ctrl=focus_automatic_continuous=0
	v4l2-ctl -d $1 --set-ctrl=focus_absolute=0
fi

# Day
if v4l2-ctl -d $1 -l | grep auto_exposure; then
	# Manual mode
	v4l2-ctl -d $1 --set-ctrl=auto_exposure=3
fi

# Night
# if v4l2-ctl -d $1 -l | grep auto_exposure; then
# 	# Manual mode
# 	v4l2-ctl -d $1 --set-ctrl=auto_exposure=1
# 	v4l2-ctl -d $1 --set-ctrl=exposure_time_absolute=500
# fi
