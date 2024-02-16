#!/bin/bash

function main(){
    local PROGS=`ps -aux | grep ros | grep -v grep | awk '{ printf("%d ", $2);for(i=11;i<=NF;++i){ printf("%s ",$i) };printf("\n") }'`
    while read p
    do
        arr=($p)
        cmd="kill -9 ${arr[0]}"
        echo "${cmd} ${arr[1]} ${arr[2]}"
        eval ${cmd}
    done <<< "${PROGS}"
}

main "$@"
