Import('env')
env.Replace(FUSESCMD="avrdude $UPLOADERFLAGS -U lfuse:w:0xEE:m -U hfuse:w:0xD6:m -U efuse:w:0xFE:m")