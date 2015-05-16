flash: all
	  ../../../Tools/jenprog/jenprog -f 0 Build/*.bin

reset:
	  ../../../Tools/jenprog/jenprog -r 0 || echo "reset success!"
