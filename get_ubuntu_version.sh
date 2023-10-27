#!/bin/bash

function main(){
    local -r VERSION=`cat /etc/os-release | grep VERSION_ID | sed 's/^.*"\(.*\)".*$/\1/'`
    
    if [ $# != 0 ]; then
        echo "${VERSION}"
    else
        echo "${VERSION/./}"
    fi
}

main "$@"
