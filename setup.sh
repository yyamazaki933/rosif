#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0);pwd)

sudo cp $SCRIPT_DIR/img/rosif.png /usr/share/pixmaps/
cat $SCRIPT_DIR/desktop/rosif.desktop | sed -e "s?PATH?$SCRIPT_DIR?" > ~/.local/share/applications/rosif.desktop
