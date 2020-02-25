#!/bin/bash
STATUSFILE="/home/robotino/rcll-refbox/view3/view3recv.gameState.txt"

if [ -f $STATUSFILE ] ; then
	read seq time phase state cyanPoint magentaPoint < $STATUSFILE
	case $phase in
	  0)
		echo "pre_game"
		;;
	  10)
		echo "setup"
		;;
	  20)
		echo "exploration"
		;;
	  30)
		echo "production"
		;;
	  40)
		echo "post_game"
		;;
	esac
else
	echo "none"
fi
