#!/bin/bash
while :
do
	ign topic -t "/translation_cmd" -m ignition.msgs.Double -p "data: -0.15"

	sleep 0.75

	ign topic -t "/translation_cmd" -m ignition.msgs.Double -p "data: 0.0"
	sleep 0.75
done
